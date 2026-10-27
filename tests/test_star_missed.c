/*
 * test_star_missed.c — the missed list (4.5, step 4b).
 *
 * The closed request loop, driven through the real round timer:
 *  1. B hears A's catalogue advertising a star B does not have ->
 *     a miss entry (A, star) is created;
 *  2. B's own round carries the request in the missed area
 *     (nb=A, n=1, star) - observed on the virtual radio;
 *  3. A, named in the request, resets the last-carried state of the
 *     star so it re-enters the content queue - its next round carries
 *     the star as CONTENT;
 *  4. B receives the star as content: the miss entry clears (the
 *     damping rule: no re-request after fetch);
 *  5. a second mismatch of the same star does not duplicate the entry;
 *  6. requests to a non-neighbour are refused (only direct
 *     neighbours, 4.5);
 *  7. damping: an unanswered entry is dropped after 2 attempts.
 */
#include <stdio.h>
#include <string.h>
#include <assert.h>

#include "core/core.h"

#define PKT_STAR_ROUND_VALUE 10
#define F_STAR_CATALOGUE_TEST 0x10
#define F_STAR_MISSED_TEST 0x20

/* --- mini virtual radio ---------------------------------------------- */
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
        if (best < 0) { now = until; return; }
        now = q[best].at; q[best].live = 0;
        ev e = q[best]; e.cb(e.n, e.ud);
    }
}

/* capture the last star-round chunk per sender id */
static uint8_t cap_pkt[2][230]; static uint16_t cap_len[2]; /* [0]=A, [1]=B */
static void vrtx(mimeshi_node *n, uint8_t ch, const uint8_t *pkt, uint16_t len, uint64_t air) {
    (void)ch; (void)air;
    if ((pkt[0] & 0x0F) != PKT_STAR_ROUND_VALUE) return;
    if (n->id == 0x0100) { memcpy(cap_pkt[0], pkt, len); cap_len[0] = len; }
    if (n->id == 0x0101) { memcpy(cap_pkt[1], pkt, len); cap_len[1] = len; }
}
static void vrsw(mimeshi_node *n, uint8_t ch, uint64_t s, void (*done)(mimeshi_node *, void *), void *ud) {
    (void)ch; (void)s;
    if (done) sched(n, s, done, ud);
}
static uint32_t vrrand(mimeshi_node *n) { (void)n; return 7u; }
static bool vrcad(mimeshi_node *n, uint8_t ch) { (void)n; (void)ch; return true; }

#define TICK_US 56000000ull
#define INTERVAL_US (16ull * TICK_US)

/* craft a catalogue-only round chunk from `owner` advertising one star */
static uint16_t craft_catalogue(uint8_t *p, uint16_t owner, uint16_t star, uint8_t digest) {
    p[0] = (uint8_t)(PKT_STAR_ROUND_VALUE | F_STAR_CATALOGUE_TEST);
    p[1] = (uint8_t)(owner & 0xFF); p[2] = (uint8_t)(owner >> 8);
    p[3] = 1; p[4] = 0;                 /* 1 catalogue entry, 0 content stars */
    p[5] = 0xE8; p[6] = 0x03; p[7] = 0xD0; p[8] = 0x07; p[9] = 1;
    p[10] = (uint8_t)(star & 0xFF); p[11] = (uint8_t)(star >> 8);
    p[12] = digest;
    return 13;
}

/* deliver a captured round of one node to the other node */
static void deliver(int from, mimeshi_node *to) {
    assert(cap_len[from] > 0);
    mimeshi_rx(to, 0, -60, cap_pkt[from], cap_len[from]);
    cap_len[from] = 0;                       /* consume: one delivery per capture */
}

static int miss_count(mimeshi_node *n, uint16_t nb, uint16_t star) {
    int c = 0;
    for (uint8_t i = 0; i < MC_STAR_MISS_MAX; i++)
        if (n->star_miss[i].alive && n->star_miss[i].nb == nb
            && n->star_miss[i].star == star) c++;
    return c;
}

int main(void) {
    static mimeshi_node A, B;
    qn = 0; now = 0; cap_len[0] = cap_len[1] = 0;
    mimeshi_init(&A, 0x0100, vrtx, sched, vrsw, vrrand, vrcad, 0);
    mimeshi_init(&B, 0x0101, vrtx, sched, vrsw, vrrand, vrcad, 0);
    mimeshi_set_geo(&A, 1000, 2000);
    mimeshi_set_geo(&B, 1100, 2000);

    /* both nodes see each other as direct neighbours (hello-acks). */
    uint8_t ack_a[5] = { 2, 0x01,0x01, 0x00,0x01 };   /* from A to B */
    uint8_t ack_b[5] = { 2, 0x00,0x01, 0x01,0x01 };   /* from B to A */
    mimeshi_rx(&B, B.home_ch, -60, ack_a, 5);
    mimeshi_rx(&A, A.home_ch, -60, ack_b, 5);

    /* A knows the star 0x0301 (far star via a 4.3 chunk) and has
     * already carried it once: the last-carried state is set, so the
     * queue would NOT re-send it without a request. */
    {
        uint8_t p[64]; uint16_t len = 25;
        p[0] = 4;
        p[1] = 0x00; p[2] = 0x02;
        p[3] = 0; p[4] = 1;
        p[5] = 0x84; p[6] = 0x03; p[7] = 0x08; p[8] = 0x07; p[9] = 1;
        p[10] = 0x01; p[11] = 0x03; p[12] = 0x50; p[13] = 30;
        p[14] = 0x90; p[15] = 0x01; p[16] = 0xF4; p[17] = 0x01; p[18] = 2;
        p[19] = 0x02; p[20] = 0x03; p[21] = 0x63;
        p[22] = 0x03; p[23] = 0x03; p[24] = 0x54;
        mimeshi_rx(&A, 0, -60, p, len);
    }
    {
        mimeshi_mapnode *m = 0;
        for (int i = 0; i < MC_MAP_NODES; i++)
            if (A.map[i].alive && A.map[i].id == 0x0301) m = &A.map[i];
        assert(m);
        uint8_t sbuf[MC_STAR_BUF];
        uint16_t sl = mimeshi_star_serialize(&A, 0x0301, sbuf, sizeof sbuf, 0);
        assert(sl > 0);
        m->star_fwd_tick = 1;                 /* carried once */
        m->star_fwd_hash = mimeshi_star_content_hash(sbuf, sl, 3);
        m->star_tick = 7;
    }

    /* 1. B hears A's catalogue: star 0x0301 unknown at B -> miss entry. */
    {
        uint8_t p[230];
        uint16_t len = craft_catalogue(p, 0x0100, 0x0301, 0x00);
        mimeshi_rx(&B, 0, -60, p, len);
    }
    assert(miss_count(&B, 0x0100, 0x0301) == 1);
    printf("1. miss entry created: ok\n");

    /* 5. a second mismatch does not duplicate. */
    {
        uint8_t p[230];
        uint16_t len = craft_catalogue(p, 0x0100, 0x0301, 0x00);
        mimeshi_rx(&B, 0, -60, p, len);
    }
    assert(miss_count(&B, 0x0100, 0x0301) == 1);
    printf("5. no duplicate: ok\n");

    /* 2. B's round carries the request in the missed area. */
    mimeshi_star_round_enable(&B);
    pump_until(INTERVAL_US);                   /* B's first round fires */
    assert(cap_len[1] > 0);
    {
        uint8_t *p = cap_pkt[1];
        assert(p[0] & F_STAR_MISSED_TEST);     /* the missed area is present */
        /* parse like the receiver: skip content stars, skip the
         * p[3] catalogue entries, then the missed area begins. */
        uint16_t off = 10;
        for (uint8_t e = 0; e < p[4]; e++) off += 9 + 3 * p[off + 8];
        off += 3 * p[3];                        /* the catalogue */
        assert((p[off] | (p[off+1] << 8)) == 0x0100);   /* addressed to A */
        assert(p[off + 2] == 1);                          /* one star */
        assert((p[off+3] | (p[off+4] << 8)) == 0x0301);
    }
    assert(miss_count(&B, 0x0100, 0x0301) == 1);   /* still live: awaiting the answer */
    printf("2. request carried in B's round: ok\n");

    /* 3. A receives the request and re-queues the star; its next
     * round carries it as CONTENT. */
    deliver(1, &A);                            /* B's captured round -> A */
    mimeshi_star_round_enable(&A);
    pump_until(now + INTERVAL_US);             /* A's first round after enable */
    assert(cap_len[0] > 0);
    {
        uint8_t *p = cap_pkt[0];
        /* the star 0x0301 must be CONTENT in A's round */
        uint16_t off = 10;
        int found = 0;
        for (uint8_t e = 0; e < p[4]; e++) {
            if ((p[off] | (p[off+1] << 8)) == 0x0301) { found = 1; break; }
            off += 9 + 3 * p[off + 8];
        }
        assert(found);
        assert(p[4] >= 1);
    }
    printf("3. A's round answers with content: ok\n");

    /* 4. B receives the star as content: the miss entry clears. */
    {
        uint8_t *pp = cap_pkt[0];
        printf("DBG A-round content ids:");
        uint16_t o = 10;
        for (uint8_t e = 0; e < pp[4]; e++) {
            printf(" %04x", (pp[o] | (pp[o+1] << 8)));
            o += 9 + 3 * pp[o + 8];
        }
        printf("\n");
    }
    deliver(0, &B);
    assert(miss_count(&B, 0x0100, 0x0301) == 0);
    printf("4. miss entry cleared on arrival: ok\n");

    /* 6. a request to a DEAD neighbour is dropped at pack time: the
     * entry stays in the list (it may still be answered by a returning
     * neighbour), but it never travels - and it is dropped when the
     * neighbour is gone from the table at pack time. */
    {
        uint8_t p[230];
        uint16_t len = craft_catalogue(p, 0x0100, 0x0306, 0x00);
        mimeshi_rx(&B, 0, -60, p, len);
        assert(miss_count(&B, 0x0100, 0x0306) == 1);   /* created: A is a neighbour */
        /* A dies before B's next round */
        for (int i = 0; i < MC_MAX_NEIGHBORS; i++)
            if (B.nbr[i].alive && B.nbr[i].id == 0x0100) B.nbr[i].alive = 0;
        pump_until(now + INTERVAL_US);                 /* B's next round: the pack
                                                           drops the entry */
        assert(cap_len[1] > 0);
        assert(miss_count(&B, 0x0100, 0x0306) == 0);   /* dropped at pack */
    }
    printf("6. dead-neighbour request dropped at pack: ok\n");

    /* 7. damping: an UNANSWERED entry is dropped after 2 attempts. */
    {
        /* fresh gap: star 0x0304 unknown at B, advertised by A */
        uint8_t p[230];
        uint16_t len = craft_catalogue(p, 0x0100, 0x0304, 0x00);
        mimeshi_rx(&B, 0, -60, p, len);
    }
    assert(miss_count(&B, 0x0100, 0x0304) == 1);
    pump_until(now + 3 * INTERVAL_US);         /* rounds of B: attempt 1, attempt 2,
                                                   then the pack of the third drops
                                                   the damped-out entry (4.5) */
    assert(miss_count(&B, 0x0100, 0x0304) == 0);   /* damped out after 2 tries */
    printf("7. damping drops the entry after 2 attempts: ok\n");

    printf("test_star_missed: all assertions passed\n");
    return 0;
}
