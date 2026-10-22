/*
 * test_routing.c — route computation (10.2) directly on the C core.
 *
 * Diamond topology:
 *
 *            C (class 7, cost 20)
 *           /                    \
 *   A ---(class 1, cost 228)--- B
 *           \                    /
 *            D (class 6, cost 29)
 *
 * A's link to B is confirmed symmetric but of the worst routable
 * class; C and D are confirmed symmetric with good links and know B.
 * ETX decides: two good hops (20+20=40 via C) beat one bad hop (228),
 * so the first transmission of A's unicast to B must go to C.
 *
 * Verifies (10.2, building block 2, step 1):
 *  - cost table: the direct bad link loses against the 2-hop detour;
 *  - the chosen first hop is a confirmed symmetric neighbour (10.1);
 *  - age multiplier: when C's map entry is old (age above half of
 *    T_active), the route shifts to D (cheaper after the multiplier);
 *  - unknown destination: refused with -1 (no flood);
 *  - an unreachable destination (in the map, no routable path):
 *    refused as well.
 */
#include <stdio.h>
#include <string.h>
#include <assert.h>

#include "core/core.h"

#define PKT_DATA_VAL 5

/* --- mini virtual radio ---------------------------------------------- */
#define MAXEV 2048
typedef struct { uint64_t at; mimeshi_node *n; void (*cb)(mimeshi_node *, void *); void *ud; int live; } ev;
static ev q[MAXEV]; static int qn;
static uint64_t now;
static void sched(mimeshi_node *n, uint64_t delay, void (*cb)(mimeshi_node *, void *), void *ud) {
    assert(qn < MAXEV);
    q[qn] = (ev){ now + delay, n, cb, ud, 1 }; qn++;
}
static void pump_until(uint64_t until) {
    for (;;) {
        int best = -1;
        for (int i = 0; i < qn; i++)
            if (q[i].live && q[i].at <= until && (best < 0 || q[i].at < q[best].at)) best = i;
        if (best < 0) return;
        now = q[best].at; q[best].live = 0;
        ev e = q[best]; e.cb(e.n, e.ud);
    }
}

static uint8_t last_type; static uint8_t last_ch; static uint16_t last_len;
static int data_tx;
static void vrtx(mimeshi_node *n, uint8_t ch, const uint8_t *pkt, uint16_t len, uint64_t air) {
    (void)n; (void)air;
    last_type = pkt[0]; last_ch = ch; last_len = len;
    if ((pkt[0] & 0x0F) == PKT_DATA_VAL) data_tx++;
}
static void vrsw(mimeshi_node *n, uint8_t ch, uint64_t s, void (*done)(mimeshi_node *, void *), void *ud) {
    (void)ch;
    if (done) sched(n, s, done, ud);
}
static uint32_t vrrand(mimeshi_node *n) { (void)n; return 7u; }
static bool vrcad(mimeshi_node *n, uint8_t ch) { (void)n; (void)ch; return true; }

static void r16w(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }

/* hello-ack addressed to `hsender` from `responder`, injected at
 * hsender with the given RSSI quality: proves symmetry there (4.2). */
static void confirm_sym(mimeshi_node *hsender, uint16_t responder, int8_t rssi) {
    uint8_t ha[6] = { 2, 0,0, 0,0, 0 };
    r16w(ha + 1, hsender->id); r16w(ha + 3, responder); ha[5] = 0;
    mimeshi_rx(hsender, (uint8_t)(hsender->id % MC_N_CHANNELS), rssi, ha, 6);
}

/* Build a prefix chunk (4.3 format): owner with two entries e1 and e2
 * (class in both nibbles, given age), plus a mutual link e1<->e2 of
 * the given class if link_cls > 0. Returns the length. */
static uint16_t chunk_build(uint8_t *ck, uint16_t owner,
                            uint16_t e1, uint8_t cls1, uint8_t age1,
                            uint16_t e2, uint8_t cls2, uint8_t age2,
                            uint8_t link_cls) {
    uint16_t len = 10;
    ck[0] = 4; r16w(ck + 1, owner); ck[3] = 0; ck[4] = 2;
    r16w(ck + 5, 100); r16w(ck + 7, 200); ck[9] = 1;
    r16w(ck + len, e1); ck[len + 2] = (uint8_t)((cls1 << 4) | cls1);
    ck[len + 3] = age1; r16w(ck + len + 4, 0); r16w(ck + len + 6, 0);
    ck[len + 8] = link_cls ? 1 : 0; len += 9;
    r16w(ck + len, e2); ck[len + 2] = (uint8_t)((cls2 << 4) | cls2);
    ck[len + 3] = age2; r16w(ck + len + 4, 0); r16w(ck + len + 6, 0);
    ck[len + 8] = 0; len += 9;
    if (link_cls) {
        r16w(ck + len, e2); ck[len + 2] = (uint8_t)((link_cls << 4) | link_cls);
        len += 3;
    }
    return len;
}

int main(void) {
    setvbuf(stdout, NULL, _IONBF, 0);
    static mimeshi_node A;
    uint16_t idB = 0x0104, idC = 0x0106, idD = 0x0105;   /* homes: B=2, C=1, D=0 */
    qn = 0; now = 0;
    mimeshi_init(&A, 0x0100, vrtx, sched, vrsw, vrrand, vrcad, 0);

    /* A has three confirmed symmetric neighbours: B (bad, -100+ dBm
     * measured here as quality just above the symmetry threshold), C
     * and D (good). Symmetry comes from hello-acks addressed to A. */
    confirm_sym(&A, idB, -100);     /* worst link: class 1 */
    confirm_sym(&A, idC, -60);       /* good: class 7 */
    confirm_sym(&A, idD, -70);       /* good: class 6..7 */
    {
        int symB = 0, symC = 0, symD = 0;
        for (int i = 0; i < MC_MAX_NEIGHBORS; i++) {
            if (!A.nbr[i].alive) continue;
            if (A.nbr[i].id == idB) symB = A.nbr[i].sym;
            if (A.nbr[i].id == idC) symC = A.nbr[i].sym;
            if (A.nbr[i].id == idD) symD = A.nbr[i].sym;
        }
        assert(symB == 1 && symC == 1 && symD == 1);
    }

    /* The graph: C and D each share a chunk naming B with a good class,
     * so A knows the paths C->B (class 7) and D->B (class 6). */
    uint8_t ck[64];
    uint16_t len = chunk_build(ck, idC, A.id, 7, 0, idB, 7, 0, 0);
    mimeshi_rx(&A, (uint8_t)(idC % MC_N_CHANNELS), -60, ck, len);
    len = chunk_build(ck, idD, A.id, 7, 0, idB, 6, 0, 0);
    mimeshi_rx(&A, (uint8_t)(idD % MC_N_CHANNELS), -70, ck, len);

    /* B is also in the map (via C's and D's entries). */
    {
        int inmap = 0;
        for (int i = 0; i < MC_MAP_NODES; i++)
            if (A.map[i].alive && A.map[i].id == idB) inmap = 1;
        assert(inmap);
    }

    /* Test 1: ETX route choice. A sends to B; the direct link costs
     * 228, via C 20+20=40, via D 29+43=72 (D->B class 6 = 29). The
     * first transmission must go to C: channel = C's home channel. */
    data_tx = 0;
    assert(mimeshi_send_data(&A, idB, (const uint8_t *)"route me", 8, false) == 0);
    pump_until(now + 50000ull);            /* switch (5 ms) + tx; far under the ack1 timeout */
    assert(data_tx == 1);
    assert((last_type & 0x0F) == PKT_DATA_VAL);
    assert(last_ch == (uint8_t)(idC % MC_N_CHANNELS));   /* first hop: C, not B */

    /* Test 2: unknown destination -> refuse (no flood, 10.2). */
    assert(mimeshi_send_data(&A, 0x0BEE, (const uint8_t *)"x", 1, false) == -1);

    /* Test 4: unreachable destination - in the map (C names E with
     * id and geo), but without a single known link toward it (class 0
     * = never reported, 4.3): map material, not route material
     * (10.2). Refused, no flood. */
    {
        uint8_t ck2[64];
        uint16_t l2 = chunk_build(ck2, idC, A.id, 7, 0, 0x0C0C, 0, 0, 0);
        mimeshi_rx(&A, (uint8_t)(idC % MC_N_CHANNELS), -60, ck2, l2);
        int inmap = 0;
        for (int i = 0; i < MC_MAP_NODES; i++)
            if (A.map[i].alive && A.map[i].id == 0x0C0C) inmap = 1;
        assert(inmap);                     /* E is known... */
        assert(mimeshi_send_data(&A, 0x0C0C, (const uint8_t *)"x", 1, false) == -1);
    }

    /* Test 3: age multiplier (10.2). Let proof ticks pass so every
     * entry ages above half of T_active (~6 h = 385 ticks of ~56 s);
     * then refresh ONLY D's path (fresh age). C's entries stay old:
     * the C-route costs 1.5x and D wins. */
    mimeshi_start(&A);                       /* starts the proof tick (~56 s) */
    pump_until(400 * 60000000ull);            /* ~6.3 h: C and D entries now stale */
    len = chunk_build(ck, idD, A.id, 7, 0, idB, 6, 0, 0);
    mimeshi_rx(&A, (uint8_t)(idD % MC_N_CHANNELS), -70, ck, len);   /* D fresh again */
    data_tx = 0;
    assert(mimeshi_send_data(&A, idB, (const uint8_t *)"route me", 8, false) == 0);
    pump_until(now + 50000ull);
    assert(data_tx == 1);
    assert(last_ch == (uint8_t)(idD % MC_N_CHANNELS));   /* C stale: D wins */

    printf("routing: OK (ETX detour via C, age shift to D, refusal of the unknown and the unreachable)\n");
    return 0;
}
