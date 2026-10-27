/*
 * test_star.c — star canonicalisation and digest (4.5, step 1).
 *
 * Radio-less unit tests of the smallest building block of the star
 * round concept (design 4.5, proposal):
 *  - canonical serialisation: entry (id, quality classes, age, geo) +
 *    links, each (peer id, classes per direction), SORTED by peer id;
 *  - order independence: the internal slot order does not change the
 *    bytes — only content matters (canonicity: the basis for content
 *    digests and the derivable heartbeat match of the later steps);
 *  - a content change changes the bytes; a silent age change does not;
 *  - the content hash excludes the age byte;
 *  - digest derivation (content_hash + tick) and the heartbeat tick
 *    match, including the k-window edge and the wrap at 256.
 *
 * The star state is built the way the core builds it in practice:
 * hello-acks and table chunks through the public rx path (same style
 * as test_table_chunk.c), so the serialiser runs against real core
 * state rather than hand-crafted structs.
 */
#include <stdio.h>
#include <string.h>
#include <assert.h>

#include "core/core.h"

#define PKT_TABLE_CHUNK_VALUE 4

/* --- mini virtual radio (same pattern as test_table_chunk.c) -------- */
#define MAXEV 2048
typedef struct { uint64_t at; mimeshi_node *n; void (*cb)(mimeshi_node *, void *); void *ud; int live; } ev;
static ev q[MAXEV]; static int qn;
static uint64_t now;
static void sched(mimeshi_node *n, uint64_t delay, void (*cb)(mimeshi_node *, void *), void *ud) {
    assert(qn < MAXEV);
    q[qn] = (ev){ now + delay, n, cb, ud, 1 }; qn++;
}
static uint8_t last_pkt[230]; static uint16_t last_len;
static void vrtx(mimeshi_node *n, uint8_t ch, const uint8_t *pkt, uint16_t len, uint64_t air) {
    (void)ch; (void)air; (void)n;
    memcpy(last_pkt, pkt, len); last_len = len;
}
static void vrsw(mimeshi_node *n, uint8_t ch, uint64_t s, void (*done)(mimeshi_node *, void *), void *ud) {
    (void)ch; (void)s;
    if (done) sched(n, 5000, done, ud);
}
static uint32_t vrrand(mimeshi_node *n) { (void)n; return 7u; }
static bool vrcad(mimeshi_node *n, uint8_t ch) { (void)n; (void)ch; return true; }

static uint16_t t_rd16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static void wr(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }

/* helper: does a chunk contain an entry for `id`? (test_table_chunk.c) */
static int chunk_has_entry(const uint8_t *p, uint16_t len, uint16_t id) {
    uint8_t n_ent = p[4];
    uint16_t off = 10;
    for (uint8_t e = 0; e < n_ent && off + 9 <= len; e++) {
        uint16_t eid = t_rd16(p + off);
        if (eid == id) return 1;
        off += 9 + 3 * p[off + 8];
    }
    return 0;
}

/* Build a table chunk as the wire format of 4.3 prescribes, carrying
 * one entry (id, classes, age, geo) plus its links — the same layout
 * a star-round content area carries. Used to give a receiver real
 * map/link state through the public rx path. */
static void inject_chunk_1star(mimeshi_node *rx_n,
                               uint16_t owner, uint16_t owner_lat, uint16_t owner_lon,
                               uint16_t eid, uint8_t classes, uint8_t age,
                               uint16_t lat, uint16_t lon,
                               const uint16_t *peers, const uint8_t *pclasses, uint8_t nlinks) {
    uint8_t p[230];
    uint16_t len = 10;
    p[0] = PKT_TABLE_CHUNK_VALUE;
    wr(p + 1, owner);
    p[3] = 0; p[4] = 1;
    wr(p + 5, owner_lat); wr(p + 7, owner_lon);
    p[9] = 1;
    wr(p + 10, eid);
    p[12] = classes; p[13] = age;
    wr(p + 14, lat); wr(p + 16, lon);
    p[18] = nlinks;
    len = 19;
    for (uint8_t i = 0; i < nlinks; i++) {
        wr(p + len, peers[i]);
        p[len+2] = pclasses[i];
        len += 3;
    }
    mimeshi_rx(rx_n, 0, -60, p, len);
    (void)chunk_has_entry;
}

int main(void) {
    static mimeshi_node S;       /* the node whose star view we test */
    qn = 0; now = 0;
    mimeshi_init(&S, 0x0102, vrtx, sched, vrsw, vrrand, vrcad, 0);
    mimeshi_set_geo(&S, 1000, 2000);

    /* S receives the star of 0x0301 (origin 0x0301, entry classes 5,
     * tick 20, geo) with its OWNED links: {0x0301, 0x0200} and
     * {0x0301, 0x0303} are owned by 0x0301 (per-link hash bit, 4.3),
     * {0x0301, 0x0302} is owned by 0x0302 and never travels in this
     * star. Injected UNSORTED (peer 0x0303 before 0x0200) to prove
     * the serialiser sorts. The link bytes as received: class
     * center->peer in the high nibble, peer->center in the low. */
    {
        uint16_t peers[2] = { 0x0303, 0x0200 };
        uint8_t pc[2] = { 0x54, 0x50 };
        inject_chunk_1star(&S, 0x0301, 900, 1800,
                          0x0301, 0x55, 20, 400, 500, peers, pc, 2);
    }

    uint8_t star[MC_STAR_BUF];
    uint16_t age_off = 0xFFFF;
    uint16_t slen = mimeshi_star_serialize(&S, 0x0301, star, sizeof star, &age_off);
    /* the star of 0x0301: the entry plus the two OWNED links - the
     * share the origin advertises (4.3), stored as received. */
    assert(slen == 9 + 3 * 2);                 /* entry + two owned links */
    assert(age_off == 3);                     /* age byte position */
    assert(t_rd16(star) == 0x0301);           /* entry id */
    assert(star[2] == 0x55);                  /* entry classes as received */
    assert(t_rd16(star + 4) == 400);          /* geo */
    assert(t_rd16(star + 6) == 500);
    /* canonical: links sorted by peer id */
    assert(t_rd16(star + 9) == 0x0200);       /* sorted: 0x0200 first */
    assert(star[9 + 2] == 0x50);              /* link bytes as received */
    assert(t_rd16(star + 12) == 0x0303);
    assert(star[12 + 2] == 0x54);

    /* unknown star: length 0 */
    assert(mimeshi_star_serialize(&S, 0x9999, star, sizeof star, 0) == 0);

    /* too small buffer: length 0, no overflow */
    assert(mimeshi_star_serialize(&S, 0x0301, star, 11, 0) == 0);

    /* canonicity: the same content, injected again in a DIFFERENT link
     * order (0x0200 before 0x0303), serialises to the SAME bytes. */
    {
        uint16_t peers[2] = { 0x0200, 0x0303 };
        uint8_t pc[2] = { 0x50, 0x54 };
        inject_chunk_1star(&S, 0x0301, 900, 1800,
                          0x0301, 0x55, 20, 400, 500, peers, pc, 2);
    }
    uint8_t star2[MC_STAR_BUF];
    uint16_t slen2 = mimeshi_star_serialize(&S, 0x0301, star2, sizeof star2, 0);
    assert(slen2 == slen);
    assert(memcmp(star, star2, slen) == 0);   /* order independent */

    /* content change: a link class changes -> different bytes */
    {
        uint16_t peers[2] = { 0x0303, 0x0200 };
        uint8_t pc[2] = { 0x64, 0x50 };        /* the 0x0303 link class differs */
        inject_chunk_1star(&S, 0x0301, 900, 1800,
                          0x0301, 0x55, 20, 400, 500, peers, pc, 2);
    }
    uint8_t star3[MC_STAR_BUF];
    uint16_t slen3 = mimeshi_star_serialize(&S, 0x0301, star3, sizeof star3, 0);
    assert(slen3 == slen);
    assert(memcmp(star, star3, slen) != 0);   /* content change visible */

    /* content hash: excludes the age byte — two stars that differ ONLY
     * in age hash the same; a content difference hashes differently. */
    {
        uint8_t a[MC_STAR_BUF], b[MC_STAR_BUF];
        memcpy(a, star3, slen3);
        memcpy(b, star3, slen3);
        b[3] = (uint8_t)(b[3] + 7);           /* age moved, content identical */
        uint8_t ha = mimeshi_star_content_hash(a, slen3, 3);
        uint8_t hb = mimeshi_star_content_hash(b, slen3, 3);
        assert(ha == hb);                     /* age excluded (4.5) */
        uint8_t c[MC_STAR_BUF];
        memcpy(c, star3, slen3);
        c[9 + 2] ^= 0x01;                     /* one link class differs */
        assert(mimeshi_star_content_hash(c, slen3, 3) != ha);
    }

    /* digest derivation and heartbeat match (4.5) */
    {
        uint8_t h = mimeshi_star_content_hash(star3, slen3, 3);
        uint8_t tick = 40;
        uint8_t d = mimeshi_star_digest(h, tick);
        assert(d == (uint8_t)(h + 40));
        /* same tick: exact match */
        assert(mimeshi_star_tick_match(d, h, tick, 8) == 0);
        /* tick +1..+8: derivable match, returns the advance */
        assert(mimeshi_star_tick_match(mimeshi_star_digest(h, tick + 1), h, tick, 8) == 1);
        assert(mimeshi_star_tick_match(mimeshi_star_digest(h, tick + 8), h, tick, 8) == 8);
        /* tick +9: outside the k-window -> no fit */
        assert(mimeshi_star_tick_match(mimeshi_star_digest(h, tick + 9), h, tick, 8) == -1);
        /* content change: no k fits */
        assert(mimeshi_star_tick_match((uint8_t)(d ^ 0x5A), h, tick, 8) == -1);
        /* wrap at 256: tick 250 advancing by 10 wraps to 4; within a
         * k-window of 16 it must still match (modulo arithmetic) */
        uint8_t wrapped = (uint8_t)(250u + 10u);
        assert(mimeshi_star_tick_match(mimeshi_star_digest(h, wrapped), h, 250, 16) == 10);
    }

    printf("test_star: all assertions passed\n");
    return 0;
}
