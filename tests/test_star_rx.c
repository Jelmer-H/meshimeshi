/*
 * test_star_rx.c — star-round receiver (4.5, step 4: catalogue with
 * derivable match).
 *
 * The smallest receiver behaviour of the star round: a node that
 * KNOWS a star hears a neighbour's catalogue entry for it. When the
 * digest is derivable (content unchanged, origin tick advanced), the
 * receiver confirms the knowledge without a single content byte:
 *
 *  - the travelled age of the star resets (arrived_tick = now): the
 *    knowledge is fresh again, exactly as if the content had been
 *    re-sent, at 3 bytes of airtime instead of ~20;
 *  - the star's tick advances to the derived value (the heartbeat);
 *  - a digest that fits NO tick in the window (real content change)
 *    does NOT reset anything - that is the missed-list path of the
 *    next step and must stay untouched here;
 *  - a catalogue entry for an unknown star is ignored (missed-list
 *    step, not built yet).
 *
 * Setup: node B knows the far star 0x0301 (via a chunk). Node A - a
 * neighbour - sends a star-round chunk whose catalogue contains
 * (0x0301, digest). We craft the digest from OUR copy of the star,
 * so the test controls the fit exactly.
 */
#include <stdio.h>
#include <string.h>
#include <assert.h>

#include "core/core.h"

#define PKT_STAR_ROUND_VALUE 10
#define F_STAR_CATALOGUE_TEST 0x10

/* --- mini virtual radio ---------------------------------------------- */
#define MAXEV 2048
typedef struct { uint64_t at; mimeshi_node *n; void (*cb)(mimeshi_node *, void *); void *ud; int live; } ev;
static ev q[MAXEV]; static int qn;
static uint64_t now;
static void sched(mimeshi_node *n, uint64_t delay, void (*cb)(mimeshi_node *, void *), void *ud) {
    assert(qn < MAXEV);
    q[qn] = (ev){ now + delay, n, cb, ud, 1 }; qn++;
}
static void vrtx(mimeshi_node *n, uint8_t ch, const uint8_t *pkt, uint16_t len, uint64_t air) {
    (void)n; (void)ch; (void)pkt; (void)len; (void)air;
}
static void vrsw(mimeshi_node *n, uint8_t ch, uint64_t s, void (*done)(mimeshi_node *, void *), void *ud) {
    (void)ch; (void)s; (void)n; (void)done; (void)ud;
}
static uint32_t vrrand(mimeshi_node *n) { (void)n; return 7u; }
static bool vrcad(mimeshi_node *n, uint8_t ch) { (void)n; (void)ch; return true; }

/* helper: inject a chunk so B knows the far star 0x0301 (same as
 * test_star.c); returns nothing, B's map now holds the star. */
static void inject_far_star(mimeshi_node *B) {
    uint8_t p[64]; uint16_t len = 25;
    p[0] = 4;                                   /* TABLE_CHUNK */
    p[1] = 0x00; p[2] = 0x02;                   /* owner 0x0200 */
    p[3] = 0; p[4] = 1;
    p[5] = 0x84; p[6] = 0x03; p[7] = 0x08; p[8] = 0x07; p[9] = 1;
    p[10] = 0x01; p[11] = 0x03; p[12] = 0x50; p[13] = 30;   /* age 30 */
    p[14] = 0x90; p[15] = 0x01; p[16] = 0xF4; p[17] = 0x01; p[18] = 2;
    p[19] = 0x02; p[20] = 0x03; p[21] = 0x63;
    p[22] = 0x03; p[23] = 0x03; p[24] = 0x54;
    mimeshi_rx(B, 0, -60, p, len);
}

/* craft a star-round chunk: header + NO content stars + catalogue
 * with the given (id, digest) entries. */
static uint16_t craft_catalogue(uint8_t *p, uint16_t owner,
                                const uint16_t *ids, const uint8_t *digests, uint8_t cnt) {
    uint16_t len = 10;
    p[0] = (uint8_t)(PKT_STAR_ROUND_VALUE | F_STAR_CATALOGUE_TEST);
    p[1] = (uint8_t)(owner & 0xFF); p[2] = (uint8_t)(owner >> 8);
    p[3] = 0; p[4] = 0;                          /* no content stars */
    p[5] = 0x4B; p[6] = 0x03; p[7] = 0xE8; p[8] = 0x07; p[9] = 1;
    for (uint8_t i = 0; i < cnt; i++) {
        p[len] = (uint8_t)(ids[i] & 0xFF); p[len+1] = (uint8_t)(ids[i] >> 8);
        p[len+2] = digests[i];
        len += 3;
    }
    return len;
}

int main(void) {
    static mimeshi_node A, B;
    qn = 0; now = 0;
    mimeshi_init(&A, 0x0100, vrtx, sched, vrsw, vrrand, vrcad, 0);
    mimeshi_init(&B, 0x0101, vrtx, sched, vrsw, vrrand, vrcad, 0);
    mimeshi_set_geo(&A, 1000, 2000);
    mimeshi_set_geo(&B, 1100, 2000);

    /* B knows the far star 0x0301 with age 30 at arrival. */
    inject_far_star(&B);
    mimeshi_mapnode *m = 0;
    for (int i = 0; i < MC_MAP_NODES; i++)
        if (B.map[i].alive && B.map[i].id == 0x0301) m = &B.map[i];
    assert(m);
    uint16_t arrived_before = m->arrived_tick;
    uint8_t age_before = m->age_ticks;
    printf("far star: age_ticks=%u arrived=%u\n", age_before, arrived_before);
    assert(age_before == 30);
    /* The star arrived via a plain 4.3 chunk: no heartbeat tick yet
     * (star_tick = 0, its meaning). Give it one, as a star-round
     * carry would have: the origin's tick as it reached us. */
    m->star_tick = 40;

    /* let the local time base advance so the freshness reset is
     * provable: the star arrived at tick 0; at tick 5 a reset must
     * visibly move arrived_tick. (Radio-less test: we advance the
     * local tick directly instead of pumping timers.) */
    B.tick = 5;
    assert(m->arrived_tick == arrived_before);       /* untouched so far */

    /* A's catalogue: (0x0301, digest) derived from OUR copy: hash + age
     * (the tick as it reached us) + 2 ticks of lag. */
    {
        uint8_t sbuf[MC_STAR_BUF];
        uint16_t sl = mimeshi_star_serialize(&B, 0x0301, sbuf, sizeof sbuf, 0);
        assert(sl > 0);
        uint8_t h = mimeshi_star_content_hash(sbuf, sl, 3);
        uint8_t digest = mimeshi_star_digest(h, (uint8_t)(m->star_tick + 2));
        uint16_t ids[1] = { 0x0301 };
        uint8_t dig[1] = { digest };
        uint8_t p[230];
        uint16_t len = craft_catalogue(p, 0x0100, ids, dig, 1);
        mimeshi_rx(&B, 0, -60, p, len);
    }
    /* derivable match (4.5): the HEARTBEAT TICK advanced by 2
     * (liveness) and the travelled age RESTARTED at 0 (freshness) -
     * two separate fields now: age_ticks keeps its 4.4 routing
     * meaning, star_tick is the origin's liveness. */
    assert(m->star_tick == 42);                           /* heartbeat advanced */
    assert(m->age_ticks == 0);                            /* travelled age restarted */
    assert(m->arrived_tick == B.tick);                    /* fresh as of NOW, visibly moved */
    assert(m->arrived_tick != arrived_before);            /* the reset is real, not a no-op */
    printf("after match: star_tick=%u age_ticks=%u arrived=%u\n",
           m->star_tick, m->age_ticks, m->arrived_tick);

    /* a catalogue entry that fits NO tick (content changed upstream):
     * nothing may reset. */
    uint16_t arrived_stale = m->arrived_tick;
    uint8_t age_stale = m->age_ticks;
    {
        uint16_t ids[1] = { 0x0301 };
        uint8_t dig[1] = { 0xA5 };          /* arbitrary non-fitting digest */
        uint8_t p[230];
        uint16_t len = craft_catalogue(p, 0x0100, ids, dig, 1);
        mimeshi_rx(&B, 0, -60, p, len);
    }
    assert(m->age_ticks == age_stale);          /* untouched */
    assert(m->star_tick == 42);                 /* the heartbeat also untouched */
    assert(m->arrived_tick == arrived_stale);   /* untouched */

    /* a catalogue entry for a star B does not have: ignored, no crash,
     * no state change. */
    {
        uint16_t ids[1] = { 0x0777 };
        uint8_t dig[1] = { 0x00 };
        uint8_t p[230];
        uint16_t len = craft_catalogue(p, 0x0100, ids, dig, 1);
        mimeshi_rx(&B, 0, -60, p, len);
    }

    printf("test_star_rx: all assertions passed\n");
    return 0;
}
