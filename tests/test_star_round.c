/*
 * test_star_round.c — the uniform star round (4.5, step 3: sender side).
 *
 * Three nodes in a line (A - B - C) on the mini virtual radio. The
 * star round is enabled only on B, so the test observes exactly what
 * the round of ONE node does, without cross-talk of the others:
 *
 *  - hard bound: at most one star-round chunk per interval per node,
 *    on all three channels per round (the 4.5 constructive bound);
 *  - content priority: the own star travels first; mutations are
 *    carried by the next round (no separate delta path needed);
 *  - own-star guarantee: within one hour of rounds the own star has
 *    traveled in full at least once, even when nothing changed;
 *  - heartbeat tick: the own star's age byte advances at the
 *    guaranteed own-star round (the tick of 4.5);
 *  - catalogue: the remaining space carries (node id, digest) pairs.
 */
#include <stdio.h>
#include <string.h>
#include <assert.h>

#include "core/core.h"

#define PKT_STAR_ROUND_VALUE 10
#define F_STAR_CATALOGUE_TEST 0x10

/* --- mini virtual radio ------------------------------------------------ */
#define MAXEV 4096
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

/* full per-node recording of star-round chunks */
#define MAX_REC 256
typedef struct { uint64_t at; uint16_t len; uint8_t ch; uint8_t pkt[230]; } rec;
static rec star_rec[MAX_REC]; static int star_rec_n;

static void vrtx(mimeshi_node *n, uint8_t ch, const uint8_t *pkt, uint16_t len, uint64_t air) {
    (void)air;
    if ((pkt[0] & 0x0F) == PKT_STAR_ROUND_VALUE && n->id == 0x0101) {
        assert(star_rec_n < MAX_REC);
        star_rec[star_rec_n].at = now;
        star_rec[star_rec_n].len = len;
        star_rec[star_rec_n].ch = ch;
        memcpy(star_rec[star_rec_n].pkt, pkt, len);
        star_rec_n++;
    }
}
static void vrsw(mimeshi_node *n, uint8_t ch, uint64_t s, void (*done)(mimeshi_node *, void *), void *ud) {
    (void)ch; (void)s;
    if (done) sched(n, s, done, ud);
}
static uint32_t vrrand(mimeshi_node *n) { (void)n; return 7u; }
static bool vrcad(mimeshi_node *n, uint8_t ch) { (void)n; (void)ch; return true; }

static uint16_t t_rd16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }

/* One hour of simulated time. The round interval is 16 proof ticks of
 * ~56 s = 900 s; an hour holds 4 rounds (the own-star bound). */
#define TICK_US 56000000ull
#define INTERVAL_US (16ull * TICK_US)
#define HOUR_US (4ull * INTERVAL_US)

int main(void) {
    static mimeshi_node A, B, C;
    qn = 0; now = 0; star_rec_n = 0;

    mimeshi_init(&A, 0x0100, vrtx, sched, vrsw, vrrand, vrcad, 0);
    mimeshi_init(&B, 0x0101, vrtx, sched, vrsw, vrrand, vrcad, 0);
    mimeshi_init(&C, 0x0102, vrtx, sched, vrsw, vrrand, vrcad, 0);
    mimeshi_set_geo(&A, 1000, 2000);
    mimeshi_set_geo(&B, 1100, 2000);
    mimeshi_set_geo(&C, 1200, 2000);

    /* B learns A and C as confirmed symmetric neighbours through
     * hello-acks (the same injection style as test_table_chunk.c):
     * hello-ack from A to B and from C to B on B's home channel. */
    uint8_t ack_a[5] = { 2, 0x01,0x01, 0x00,0x01 };   /* HELLO_ACK, dst=B, from=A */
    uint8_t ack_c[5] = { 2, 0x01,0x01, 0x02,0x01 };   /* HELLO_ACK, dst=B, from=C */
    mimeshi_rx(&B, B.home_ch, -60, ack_a, 5);
    mimeshi_rx(&B, B.home_ch, -70, ack_c, 5);
    {
        int sym_a = -1, sym_c = -1;
        for (int i = 0; i < MC_MAX_NEIGHBORS; i++) {
            if (B.nbr[i].alive && B.nbr[i].id == 0x0100) sym_a = B.nbr[i].sym;
            if (B.nbr[i].alive && B.nbr[i].id == 0x0102) sym_c = B.nbr[i].sym;
        }
        assert(sym_a == 1 && sym_c == 1);        /* both confirmed */
    }

    /* B gets map material: a far star via a chunk (owner 0x0200, entry
     * 0x0301 with two links), so the round has tail material too. */
    {
        uint8_t p[64]; uint16_t len = 25;
        p[0] = 4;                                   /* TABLE_CHUNK */
        p[1] = 0x00; p[2] = 0x02;                   /* owner 0x0200 */
        p[3] = 0; p[4] = 1;
        p[5] = 0x84; p[6] = 0x03; p[7] = 0x08; p[8] = 0x07; p[9] = 1;
        p[10] = 0x01; p[11] = 0x03; p[12] = 0x50; p[13] = 20;
        p[14] = 0x90; p[15] = 0x01; p[16] = 0xF4; p[17] = 0x01; p[18] = 2;
        p[19] = 0x02; p[20] = 0x03; p[21] = 0x63;
        p[22] = 0x03; p[23] = 0x03; p[24] = 0x54;
        mimeshi_rx(&B, 0, -60, p, len);
    }

    /* the round is enabled ONLY on B (sender-side step): the other two
     * nodes run the ordinary 4.3 cadence. */
    mimeshi_star_round_enable(&B);
    mimeshi_start(&A); mimeshi_start(&B); mimeshi_start(&C);

    /* run two hours: 8 rounds expected (hard bound: no more). */
    pump_until(2 * HOUR_US + INTERVAL_US);

    printf("star-round chunks from B: %d (each on 3 channels)\n", star_rec_n);
    assert(star_rec_n >= 6 * 3);                /* at least 6 rounds happened */
    assert(star_rec_n <= 8 * 3 + 3);            /* bound: ~8 rounds x 3 ch, small slack */

    /* hard bound per interval: no two rounds of B within one interval */
    for (int i = 3; i < star_rec_n; i += 3) {    /* per round (3 ch apart) */
        uint64_t prev = 0; int have = 0;
        for (int k = 0; k < star_rec_n; k += 3) {
            uint64_t at = star_rec[k].at;
            if (have && at - prev < INTERVAL_US - TICK_US) {
                printf("bound violated: rounds %llu and %llu apart\n",
                       (unsigned long long)prev, (unsigned long long)at);
                assert(0);
            }
            prev = at; have = 1;
        }
        break;
    }

    /* every chunk: header shape + at least the catalogue flag when
     * space remained, and the own star as content in the first rounds
     * (star_own_ctr starts at 0 = due immediately). */
    int own_star_seen = 0, catalogue_seen = 0;
    for (int k = 0; k < star_rec_n; k++) {
        uint8_t *p = star_rec[k].pkt; uint16_t len = star_rec[k].len;
        assert(t_rd16(p + 1) == 0x0101);         /* owner = B */
        if (p[0] & F_STAR_CATALOGUE_TEST) catalogue_seen++;
        /* content stars: entries after the 10 B header */
        uint16_t off = 10;
        for (uint8_t e = 0; e < p[4]; e++) {
            uint16_t eid = t_rd16(p + off);
            if (eid == 0x0101) own_star_seen++;
            off += 9 + 3 * p[off + 8];
            assert(off <= len);
        }
    }
    assert(own_star_seen >= 3);                  /* the own star traveled in full */
    assert(catalogue_seen > 0);                  /* the catalogue rides along */

    /* own-star guarantee: within each hour of rounds the own star
     * appears at least once - check per hour over the recording. */
    for (uint64_t h = 0; h < 2; h++) {
        int in_hour = 0;
        for (int k = 0; k < star_rec_n; k++) {
            if (star_rec[k].at < h * HOUR_US || star_rec[k].at >= (h + 1) * HOUR_US) continue;
            uint8_t *p = star_rec[k].pkt;
            uint16_t off = 10;
            for (uint8_t e = 0; e < p[4]; e++) {
                if (t_rd16(p + off) == 0x0101) { in_hour = 1; break; }
                off += 9 + 3 * p[off + 8];
            }
            if (in_hour) break;
        }
        assert(in_hour);                         /* guaranteed own-star round */
    }

    /* heartbeat tick: the own star's age byte in the LAST round it
     * appeared in is >= the one in the FIRST - it advances per
     * guaranteed own-star round (4.5). */
    {
        int first = -1, last = -1;
        for (int k = 0; k < star_rec_n; k++) {
            uint8_t *p = star_rec[k].pkt;
            uint16_t off = 10;
            for (uint8_t e = 0; e < p[4]; e++) {
                if (t_rd16(p + off) == 0x0101) {
                    if (first < 0) first = k;
                    last = k;
                }
                off += 9 + 3 * p[off + 8];
            }
        }
        assert(first >= 0 && last >= 0);
        uint8_t tick_first = 0, tick_last = 0;
        uint16_t off = 10;
        for (uint8_t e = 0; e < star_rec[first].pkt[4]; e++) {
            if (t_rd16(star_rec[first].pkt + off) == 0x0101) tick_first = star_rec[first].pkt[off + 3];
            off += 9 + 3 * star_rec[first].pkt[off + 8];
        }
        off = 10;
        for (uint8_t e = 0; e < star_rec[last].pkt[4]; e++) {
            if (t_rd16(star_rec[last].pkt + off) == 0x0101) tick_last = star_rec[last].pkt[off + 3];
            off += 9 + 3 * star_rec[last].pkt[off + 8];
        }
        assert(tick_last > tick_first);          /* the tick advances */
    }

    printf("test_star_round: all assertions passed\n");
    return 0;
}
