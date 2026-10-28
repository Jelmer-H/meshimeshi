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

/* Build a star-round content packet in the 4.3 wire form: the star of
 * `origin` - its entry (classes in both nibbles) plus the links the
 * origin OWNS. Ownership follows the 4.3 rule (one bit of a CRC-8
 * over the normalised pair); the helper mirrors the core so the test
 * injects exactly what the owner's star carries on air. A wanted link
 * toward `peer` (class both nibbles) is included when the origin owns
 * it; the caller passes the owner-side node as `origin`. */
static uint8_t t_pair_bit(uint16_t x, uint16_t y) {
    uint16_t a = x < y ? x : y, b = x < y ? y : x;
    uint8_t bytes[4] = { (uint8_t)a, (uint8_t)(a >> 8), (uint8_t)b, (uint8_t)(b >> 8) };
    uint8_t crc = 0;
    for (int i = 0; i < 4; i++) {
        crc ^= bytes[i];
        for (int k = 0; k < 8; k++)
            crc = (uint8_t)((crc & 0x80) ? (crc << 1) ^ 0x07 : crc << 1);
    }
    return (uint8_t)(crc & 1);
}
static uint16_t t_owner(uint16_t x, uint16_t y) {
    uint16_t a = x < y ? x : y, b = x < y ? y : x;
    return t_pair_bit(x, y) ? a : b;
}
static uint16_t star_build(uint8_t *ck, uint16_t origin,
                            uint8_t cls, uint8_t age,
                            const uint16_t *peers, const uint8_t *lcls,
                            uint8_t nlinks) {
    uint16_t len = 10;
    ck[0] = 4; r16w(ck + 1, origin); ck[3] = 0; ck[4] = 1;
    r16w(ck + 5, 100); r16w(ck + 7, 200); ck[9] = 1;
    r16w(ck + len, origin); ck[len + 2] = (uint8_t)((cls << 4) | cls);
    ck[len + 3] = age; r16w(ck + len + 4, 0); r16w(ck + len + 6, 0);
    ck[len + 8] = nlinks; len += 9;
    for (uint8_t i = 0; i < nlinks; i++) {
        r16w(ck + len, peers[i]); ck[len + 2] = lcls[i];
        len += 3;
    }
    return len;
}
/* the OLD helper name, kept for the few one-entry injection sites:
 * one star of `owner` (entry e1, classes cls1) that owns the link
 * toward e2 with the given class - the 4.3 claim "owner knows e2". */
static uint16_t chunk_build(uint8_t *ck, uint16_t owner,
                            uint16_t e1, uint8_t cls1, uint8_t age1,
                            uint16_t e2, uint8_t cls2, uint8_t age2,
                            uint8_t link_cls) {
    (void)e1; (void)e2; (void)cls2; (void)age2;
    uint16_t peers[1] = {0}; uint8_t lcls[1] = {0}; uint8_t nlinks = 0;
    if (link_cls && t_owner(owner, e2) == owner) {
        peers[0] = e2;
        lcls[0] = (uint8_t)((link_cls << 4) | link_cls);
        nlinks = 1;
    }
    return star_build(ck, owner, cls1, age1, peers, lcls, nlinks);
}

/* --- chain radio: four wired nodes, switch rule applied --------- */
static uint16_t rd16p(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static mimeshi_node *f_nodes[4];
static int f_delivered[4], f_fwd[4];
static uint8_t f_last_ch[4];
static void f_rx(mimeshi_node *r, uint8_t ch, const uint8_t *pkt, uint16_t len);
static void ftx(mimeshi_node *n, uint8_t ch, const uint8_t *pkt, uint16_t len, uint64_t air) {
    (void)air;
    int from = -1;
    for (int i = 0; i < 4; i++) if (f_nodes[i] == n) { from = i; break; }
    assert(from >= 0 && len <= 230);
    uint8_t t = pkt[0] & 0x0F;
    f_last_ch[from] = ch;
    if (t == PKT_DATA_VAL && len > 12 && rd16p(pkt + 5) != f_nodes[from]->id)
        f_fwd[from]++;                        /* a DATA tx toward someone else: a forward */
    /* the line: a packet on channel ch reaches the wired neighbours
     * that are AT HOME there (home channel) - plus a node waiting in
     * an exchange on that channel. */
    static const int lk[3][2] = { {0,1}, {1,2}, {2,3} };
    for (int k = 0; k < 3; k++) {
        int a = lk[k][0], b = lk[k][1];
        if (a == from && f_nodes[b]->home_ch == ch) f_rx(f_nodes[b], ch, pkt, len);
        if (b == from && f_nodes[a]->home_ch == ch) f_rx(f_nodes[a], ch, pkt, len);
    }
    for (int i = 0; i < 4; i++) {
        if (i == from || f_nodes[i]->home_ch == ch) continue;
        for (int j = 0; j < MC_MAX_PENDING; j++)
            if (f_nodes[i]->pend[j].active && (uint8_t)(f_nodes[i]->pend[j].next_hop % MC_N_CHANNELS) == ch) {
                f_rx(f_nodes[i], ch, pkt, len);   /* waiting exchange partner */
                break;
            }
    }
}
static void f_rx(mimeshi_node *r, uint8_t ch, const uint8_t *pkt, uint16_t len) {
    if ((pkt[0] & 0x0F) == PKT_DATA_VAL && len > 12 &&
        rd16p(pkt + 5) == r->id && rd16p(pkt + 10) == r->id) {
        int i; for (i = 0; i < 4; i++) if (f_nodes[i] == r) break;
        f_delivered[i]++;                      /* the destination received its DATA */
    }
    mimeshi_rx(r, ch, -55, pkt, len);
}
static void f_confirm(mimeshi_node *hs, uint16_t responder) {
    uint8_t ha[6] = { 2, 0,0, 0,0, 0 };
    ha[1] = (uint8_t)hs->id; ha[2] = (uint8_t)(hs->id >> 8);
    ha[3] = (uint8_t)responder; ha[4] = (uint8_t)(responder >> 8);
    mimeshi_rx(hs, (uint8_t)(hs->id % MC_N_CHANNELS), -55, ha, 6);
}
static void f_deliver(const uint8_t *pkt, uint16_t len) {
    for (int i = 0; i < 4; i++)
        mimeshi_rx(f_nodes[i], (uint8_t)(f_nodes[i]->id % MC_N_CHANNELS), -55, pkt, len);
}
static void f_data_tx_reset(void) {
    f_delivered[0] = f_delivered[1] = f_delivered[2] = f_delivered[3] = 0;
    f_fwd[0] = f_fwd[1] = f_fwd[2] = f_fwd[3] = 0;
}

/* --- test-6 radio: star around A, one blackholed node --------- */
static mimeshi_node *e_nodes[4];
static uint16_t e_blackout;
static int e_tx[4], e_deliv[4];
static void e_confirm(mimeshi_node *hs, uint16_t responder) {
    uint8_t ha[6] = { 2, 0,0, 0,0, 0 };
    ha[1] = (uint8_t)hs->id; ha[2] = (uint8_t)(hs->id >> 8);
    ha[3] = (uint8_t)responder; ha[4] = (uint8_t)(responder >> 8);
    mimeshi_rx(hs, (uint8_t)(hs->id % MC_N_CHANNELS), -55, ha, 6);
}
static void e_rxnode(mimeshi_node *r, uint8_t ch, const uint8_t *pkt, uint16_t len) {
    if ((pkt[0] & 0x0F) == PKT_DATA_VAL && len > 12 &&
        rd16p(pkt + 5) == r->id && rd16p(pkt + 10) == r->id) {
        int i; for (i = 0; i < 4; i++) if (e_nodes[i] == r) break;
        e_deliv[i]++;
    }
    mimeshi_rx(r, ch, -55, pkt, len);
}
static void etx_cb(mimeshi_node *n, uint8_t ch, const uint8_t *pkt, uint16_t len, uint64_t air) {
    (void)air;
    int from = -1;
    for (int i = 0; i < 4; i++) if (e_nodes[i] == n) { from = i; break; }
    assert(from >= 0 && len <= 230);
    if (e_blackout && (n->id == e_blackout ||
        rd16p(pkt + 10) == e_blackout || (len > 7 && rd16p(pkt + 3) == e_blackout)))
        return;                                /* the black hole swallows */
    if ((pkt[0] & 0x0F) == PKT_DATA_VAL) e_tx[from]++;
    /* full mesh below A, switch rule: home channel + waiting partners */
    for (int i = 0; i < 4; i++) {
        if (e_nodes[i] == n) continue;
        if (e_nodes[i]->home_ch == ch) { e_rxnode(e_nodes[i], ch, pkt, len); continue; }
        for (int j = 0; j < MC_MAX_PENDING; j++)
            if (e_nodes[i]->pend[j].active && (uint8_t)(e_nodes[i]->pend[j].next_hop % MC_N_CHANNELS) == ch) {
                e_rxnode(e_nodes[i], ch, pkt, len);
                break;
            }
    }
}
static void e_deliver_all(const uint8_t *pkt, uint16_t len) {
    for (int i = 0; i < 4; i++)
        mimeshi_rx(e_nodes[i], (uint8_t)(e_nodes[i]->id % MC_N_CHANNELS), -55, pkt, len);
}
static int e_deliv_count(uint16_t id) {
    for (int i = 0; i < 4; i++) if (e_nodes[i]->id == id) return e_deliv[i];
    return -1;
}
static int e_tx_count(int slot) { return e_tx[slot]; }

/* --- test-7 radio: A - B - C with a bridge destination --------- */
static mimeshi_node *g_nodes[3];
static uint8_t g_last_ch;
static void g_confirm(mimeshi_node *hs, uint16_t responder) {
    uint8_t ha[6] = { 2, 0,0, 0,0, 0 };
    ha[1] = (uint8_t)hs->id; ha[2] = (uint8_t)(hs->id >> 8);
    ha[3] = (uint8_t)responder; ha[4] = (uint8_t)(responder >> 8);
    mimeshi_rx(hs, (uint8_t)(hs->id % MC_N_CHANNELS), -55, ha, 6);
}
static void g_deliv_reset(void) { g_last_ch = 0xFF; }
static void gtx(mimeshi_node *n, uint8_t ch, const uint8_t *pkt, uint16_t len, uint64_t air) {
    (void)air; (void)len; (void)pkt;
    int from = -1;
    for (int i = 0; i < 3; i++) if (g_nodes[i] == n) { from = i; break; }
    assert(from >= 0);
    g_last_ch = ch;
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

    /* The graph: A learns the paths C->B (class 7) and D->B (class 6)
     * from the stars of the link OWNERS (4.3): C owns {C,B}, so C's
     * star carries the link; {D,B} is owned by B, so B's star carries
     * it. Injected at A as star-round content packets. */
    uint8_t ck[64];
    {
        uint16_t peers[1] = { idB };
        uint8_t lcls[1] = { 0x77 };
        uint16_t len = star_build(ck, idC, 7, 0, peers, lcls, 1);
        mimeshi_rx(&A, (uint8_t)(idC % MC_N_CHANNELS), -60, ck, len);
    }
    {
        uint16_t peers[1] = { idD };
        uint8_t lcls[1] = { 0x66 };
        uint16_t len = star_build(ck, idB, 7, 0, peers, lcls, 1);
        mimeshi_rx(&A, (uint8_t)(idB % MC_N_CHANNELS), -70, ck, len);
    }

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
    assert(mimeshi_send_data(&A, idB, (const uint8_t *)"route me", 8, false, 0, 0) == 0);
    pump_until(now + 50000ull);            /* switch (5 ms) + tx; far under the ack1 timeout */
    assert(data_tx == 1);
    assert((last_type & 0x0F) == PKT_DATA_VAL);
    assert(last_ch == (uint8_t)(idC % MC_N_CHANNELS));   /* first hop: C, not B */

    /* Test 2: unknown destination -> refuse (no flood, 10.2). */
    assert(mimeshi_send_data(&A, 0x0BEE, (const uint8_t *)"x", 1, false, 0, 0) == -1);

    /* Test 4: unreachable destination - in the map (C names E with
     * id and geo), but without a single known link toward it (class 0
     * = never reported, 4.3): map material, not route material
     * (10.2). Refused, no flood. */
    {
        uint8_t ck2[64];
        uint16_t l2 = star_build(ck2, 0x0C0C, 7, 0, 0, 0, 0);
        mimeshi_rx(&A, (uint8_t)(0x0C0C % MC_N_CHANNELS), -60, ck2, l2);
        int inmap = 0;
        for (int i = 0; i < MC_MAP_NODES; i++)
            if (A.map[i].alive && A.map[i].id == 0x0C0C) inmap = 1;
        assert(inmap);                     /* E is known... */
        assert(mimeshi_send_data(&A, 0x0C0C, (const uint8_t *)"x", 1, false, 0, 0) == -1);
    }

    /* Test 3: age multiplier (10.2). Let proof ticks pass so every
     * entry ages above half of T_active (~6 h = 385 ticks of ~56 s);
     * then refresh ONLY D's path (fresh age). C's entries stay old:
     * the C-route costs 1.5x and D wins. */
    mimeshi_start(&A);                       /* starts the proof tick (~56 s) */
    pump_until(400 * 60000000ull);            /* ~6.3 h: C and D entries now stale */
    /* D comes back as a confirmed neighbour the way the protocol does
     * it: a hello ack addressed to us (4.2) - neighbour freshness is
     * the hello/ack/traffic path, not a star reception. C stays
     * silent: its entries keep the stale age. The D->B link class 6
     * arrives as B's owned star (4.3). */
    confirm_sym(&A, idD, -70);
    {
        uint16_t peers[1] = { idD };
        uint8_t lcls[1] = { 0x66 };
        uint16_t l3 = star_build(ck, idB, 7, 0, peers, lcls, 1);
        mimeshi_rx(&A, (uint8_t)(idB % MC_N_CHANNELS), -70, ck, l3);
    }
    data_tx = 0;
    assert(mimeshi_send_data(&A, idB, (const uint8_t *)"route me", 8, false, 0, 0) == 0);
    pump_until(now + 50000ull);
    assert(data_tx == 1);
    assert(last_ch == (uint8_t)(idD % MC_N_CHANNELS));   /* C stale: D wins */

    /* ------------------------------------------------------------------
     * Test 5: forwarding over a chain (10.2 step 2). Four nodes in a
     * line: A - B - C - D, all links confirmed symmetric and good.
     * A sends to D; the message must run A->B->C->D, every hop with a
     * fresh next-hop computation, and every hop acks its predecessor.
     * The radio below wires the line and applies the switch rule. */
    {
        static mimeshi_node B, C, D;
        mimeshi_node *N4[4] = { &A, &B, &C, &D };
        uint16_t idn[4] = { 0x0200, 0x0201, 0x0202, 0x0203 };  /* homes: 2,0,1,2 */
        extern void f_reset(void);
        /* fresh nodes for the chain */
        qn = 0; now = 0;
        for (int i = 0; i < 4; i++)
            mimeshi_init(N4[i], idn[i], ftx, sched, vrsw, vrrand, vrcad, 0);
        f_nodes[0] = &A; f_nodes[1] = &B; f_nodes[2] = &C; f_nodes[3] = &D;
        /* symmetric confirmations along the line: crossed hello-acks */
        f_confirm(&A, idn[1]); f_confirm(&B, idn[0]);
        f_confirm(&B, idn[2]); f_confirm(&C, idn[1]);
        f_confirm(&C, idn[3]); f_confirm(&D, idn[2]);
        /* the graph under link ownership (4.3): C owns BOTH {B,C} and
         * {C,D} (per-link hash bit), so C's star carries the two
         * links - A learns the path A-B-C-D from it: A-B is an own
         * edge, B-C and C-D travel in C's star. B owns nothing on
         * the line ({A,B} belongs to A), and B needs no map links:
         * its next hops A and C are direct neighbours. */
        uint8_t ckf[64];
        {
            uint16_t peers[2] = { idn[1], idn[3] };   /* sorted: B before D */
            uint8_t lcls[2] = { 0x77, 0x77 };
            uint16_t lf = star_build(ckf, idn[2], 7, 0, peers, lcls, 2);
            f_deliver(ckf, lf);
        }
        /* A -> D must be accepted (D is in A's graph) */
        f_data_tx_reset();
        assert(mimeshi_send_data(&A, idn[3], (const uint8_t *)"hop", 3, false, 0, 0) == 0);
        pump_until(now + 50000000ull);      /* the whole chain: switches, hops, acks */
        /* D received the DATA (a retry of the last hop may arrive too
         * and gets its ack1 under the 7.4 retry rule - it counts as a
         * reception at the destination, the core dedups the delivery;
         * the [core] delivery print appears exactly once). */
        assert(f_delivered[3] >= 1);
        assert(f_fwd[1] >= 1 && f_fwd[2] >= 1);   /* B and C each forwarded */
        /* every pending record closed: all hops acked */
        int act = 0;
        for (int i = 0; i < MC_MAX_PENDING; i++) {
            if (A.pend[i].active || B.pend[i].active || C.pend[i].active || D.pend[i].active) act++;
        }
        assert(act == 0);
    }

    /* ------------------------------------------------------------------
     * Test 6: retry series and cumulative exclusion (7.4). A has two
     * confirmed neighbours toward B: C (best) and D (second). The
     * radio blackholes C: both attempts toward C stay unanswered; the
     * series fails, C goes into the exclusion, and the recompute must
     * pick D - the message still delivers, via D. */
    {
        /* nodes: A (0x0300), C (0x0301, blackholed), D (0x0302), B (0x0303) */
        static mimeshi_node AA, CC, DD, BB;
        mimeshi_node *E[4] = { &AA, &CC, &DD, &BB };
        uint16_t ei[4] = { 0x0300, 0x0301, 0x0302, 0x0303 };
        extern void e_reset(int blackout);
        qn = 0; now = 0;
        for (int i = 0; i < 4; i++) mimeshi_init(E[i], ei[i], etx_cb, sched, vrsw, vrrand, vrcad, 0);
        e_nodes[0] = &AA; e_nodes[1] = &CC; e_nodes[2] = &DD; e_nodes[3] = &BB;
        e_confirm(&AA, ei[1]); e_confirm(&CC, ei[0]);
        e_confirm(&AA, ei[2]); e_confirm(&DD, ei[0]);
        e_confirm(&CC, ei[3]); e_confirm(&BB, ei[1]);
        e_confirm(&DD, ei[3]); e_confirm(&BB, ei[2]);
        uint8_t cke[64];
        uint16_t le = chunk_build(cke, ei[1], ei[0], 7, 0, ei[3], 7, 0, 7);
        e_deliver_all(cke, le);
        le = chunk_build(cke, ei[2], ei[0], 7, 0, ei[3], 7, 0, 7);
        e_deliver_all(cke, le);
        /* A -> B: first choice is C (both cost 20+20, C first in the
         * table order). Blackhole C; after the failed series (2
         * attempts, 2 s + jitter apart) the exclusion must switch to
         * D and deliver. */
        e_blackout = ei[1];                    /* radio drops everything to/from C */
        assert(mimeshi_send_data(&AA, ei[3], (const uint8_t *)"alt", 3, false, 0, 0) == 0);
        pump_until(now + 30000000ull);          /* series: 2 x (350 ms + 2 s + jitter) + the D route */
        assert(e_deliv_count(ei[3]) >= 1);      /* B delivered via D */
        assert(e_tx_count(2) >= 1);              /* D forwarded at least once */
        /* the exclusion is per message: a fresh send uses C again */
        e_blackout = 0;
    }

    /* ------------------------------------------------------------------
     * Test 7: the bridge-node rule (10.2). A knows B (confirmed, with
     * geo) and B knows C (with geo) - but A does NOT know C: C lies
     * outside A's graph. The client supplies C's geo. The bridge rule
     * must pick the graph node closest to C's geo - B - and route the
     * first hop toward B. B, knowing C, delivers in one more hop. */
    {
        static mimeshi_node A2, B2, C2;
        mimeshi_node *G[3] = { &A2, &B2, &C2 };
        uint16_t gi[3] = { 0x0400, 0x0401, 0x0402 };   /* homes: 0, 1, 2 */
        qn = 0; now = 0;
        for (int i = 0; i < 3; i++) mimeshi_init(G[i], gi[i], gtx, sched, vrsw, vrrand, vrcad, 0);
        g_nodes[0] = &A2; g_nodes[1] = &B2; g_nodes[2] = &C2;
        g_confirm(&A2, gi[1]); g_confirm(&B2, gi[0]);
        g_confirm(&B2, gi[2]); g_confirm(&C2, gi[1]);
        /* B's chunk names A and C with geos; A thereby knows B's geo
         * and C's geo-as-entry... C appears as a map node of B - A
         * KNOWS C indirectly. To make C genuinely unknown to A, this
         * test injects B's chunk ONLY at A without C: a two-entry
         * chunk without C means A knows only B. Simpler: a chunk with
         * just A as entry, plus the mutual link nothing. A's graph:
         * {B}. C's geo comes from the client. */
        uint8_t ckg[64];
        uint16_t lg = chunk_build(ckg, gi[1], gi[0], 7, 0, gi[0], 7, 0, 0);
        /* build gives owner B with entries A and... we need A-only:
         * hand-build a 1-entry chunk: owner B, entry A, no links */
        ckg[0] = 4; ckg[1] = (uint8_t)gi[1]; ckg[2] = (uint8_t)(gi[1] >> 8);
        ckg[3] = 0; ckg[4] = 1;
        /* owner geo in the chunk header (offsets 5..8): B stands at
         * grid (250, 250) - the bridge candidate needs a geo */
        ckg[5] = 250; ckg[6] = 0; ckg[7] = 250; ckg[8] = 0; ckg[9] = 1;
        ckg[10] = (uint8_t)gi[0]; ckg[11] = (uint8_t)(gi[0] >> 8);
        ckg[12] = 0x77; ckg[13] = 0;
        ckg[14] = 0; ckg[15] = 0; ckg[16] = 0; ckg[17] = 0; ckg[18] = 0;
        lg = 19;
        (void)lg;
        mimeshi_rx(&A2, (uint8_t)(gi[1] % MC_N_CHANNELS), -60, ckg, 19);
        /* A -> C with C's geo from the "client": bridge = B (the only,
         * and closest, graph node); first hop must go to B */
        uint16_t clat = 300, clon = 300;             /* C's client geo */
        g_deliv_reset();
        assert(mimeshi_send_data(&A2, gi[2], (const uint8_t *)"via bridge", 10, false, clat, clon) == 0);
        pump_until(now + 50000ull);
        /* the first transmission goes to B (B's home channel 1) */
        assert(g_last_ch == (uint8_t)(gi[1] % MC_N_CHANNELS));
    }

    printf("routing: OK (ETX detour, age shift, refusals, chain forward, retry series, bridge-node rule)\n");
    return 0;
}
