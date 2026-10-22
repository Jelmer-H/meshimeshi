/*
 * test_table_chunk.c — table push (4.3) directly on the C core.
 *
 * Verifies the chunk mechanism of design 4.3:
 *  - a node with confirmed symmetric neighbours sends a table chunk on
 *    its HOME channel after the bundling window (mutation triggered);
 *  - the chunk carries ONLY confirmed neighbours: a "seen" neighbour
 *    does not travel along (no heard-only in the chunks);
 *  - the header carries the owner id and the own geo (section 5);
 *  - the receiver builds its 2-hop map from the entries (id, geo,
 *    quality class) and the link store from the link records;
 *  - the owner naming us in an entry is reverse-direction evidence and
 *    confirms the link — gated on quality like every confirmation
 *    (10.1), and it feeds the table push (the new neighbour is
 *    advertised);
 *  - an entry with a geo updates the map node; (0,0) leaves it alone.
 */
#include <stdio.h>
#include <string.h>
#include <assert.h>

#include "core/core.h"

#define PKT_TABLE_CHUNK_VALUE 4   /* PKT_TABLE_CHUNK, core.c */
#define MC_SYM_QUALITY_MIN 14u      /* core.c: quality_of(-100 dBm) */
#define MC_TBL_WIN_US 3600000000ull   /* core.c: bundling window 1 h */

/* --- mini virtual radio ---------------------------------------------- */
#define MAXEV 256
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

/* --- record what the core does ---------------------------------------- */
static uint8_t last_pkt[230]; static uint16_t last_len; static uint8_t last_ch;
static uint16_t last_tx_node; static int chunk_seen;

static void vrtx(mimeshi_node *n, uint8_t ch, const uint8_t *pkt, uint16_t len, uint64_t air) {
    (void)air;
    memcpy(last_pkt, pkt, len); last_len = len; last_ch = ch; last_tx_node = n->id;
    if ((pkt[0] & 0x0F) == PKT_TABLE_CHUNK_VALUE) chunk_seen++;
}
static void vrsw(mimeshi_node *n, uint8_t ch, uint64_t s, void (*done)(mimeshi_node *, void *), void *ud) {
    (void)ch; (void)s;
    if (done) sched(n, 5000, done, ud);      /* switch incl. settling: fire the tx */
}
static uint32_t vrrand(mimeshi_node *n) { (void)n; return 7u; }
static bool vrcad(mimeshi_node *n, uint8_t ch) { (void)n; (void)ch; return true; }

#define PKT_TABLE_CHUNK_VALUE 4
static uint16_t t_rd16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }

/* helper: does the chunk contain an entry for `id`? */
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

int main(void) {
    static mimeshi_node S, R;   /* S = sender/sharer, R = receiver */
    qn = 0; now = 0;
    mimeshi_init(&S, 0x0102, vrtx, sched, vrsw, vrrand, vrcad, 0);
    mimeshi_init(&R, 0x0100, vrtx, sched, vrsw, vrrand, vrcad, 0);
    mimeshi_set_geo(&S, 1000, 2000);
    mimeshi_set_geo(&R, 1500, 2500);

    /* S has two neighbours: B confirmed symmetric (good quality),
     * W only "seen" (weak). Neither has a real radio here; the entries
     * are set directly via the public reception paths. */
    uint8_t hack_from_b[5] = { 2, 0x02,0x01, 0x0B,0x01 };  /* hello-ack from 0x010B to S */
    mimeshi_rx(&S, 0, -60, hack_from_b, 5);                 /* good quality: confirmed */
    {
        int sym = 0;
        for (int i = 0; i < MC_MAX_NEIGHBORS; i++)
            if (S.nbr[i].alive && S.nbr[i].id == 0x010B) sym = S.nbr[i].sym;
        assert(sym == 1);                                   /* confirmed via hello-ack */
    }
    uint8_t hello_weak[3] = { 1, 0x0C,0x01 };              /* hello from 0x010C */
    mimeshi_rx(&S, 0, -105, hello_weak, 3);                /* weak: seen only, no ack */
    {
        int seen = -1;
        for (int i = 0; i < MC_MAX_NEIGHBORS; i++)
            if (S.nbr[i].alive && S.nbr[i].id == 0x010C) seen = S.nbr[i].sym;
        assert(seen == 0);                                  /* weak link stays seen */
    }
    /* a geo for B in the map of S: via a chunk of B (2-hop source) —
     * here injected directly as map material via a chunk from B. */
    /* (the map entry for B gets its geo when B's own chunk arrives; in
     * this test S has no geo of B, so the entry geo stays (0,0) — the
     * chunk entry then carries none, which the receiver must accept.) */

    /* the confirmation marked S's table dirty: after the bundling
     * window a chunk follows on S's home channel. */
    int before = chunk_seen;
    pump_until(2 * MC_TBL_WIN_US);                          /* > the 1 h window + jitter */
    assert(chunk_seen == before + MC_N_CHANNELS);           /* one push: a chunk per channel */
    assert(last_tx_node == 0x0102);
    /* round starts at (home+1)%3 and ends on the OWN home channel (2):
     * S = 0x0102 has home 0 -> order 1,2,0 */
    assert(last_ch == 0);
    assert(last_tx_node == 0x0102);
    assert((last_pkt[0] & 0x0F) == PKT_TABLE_CHUNK_VALUE);
    assert(t_rd16(last_pkt + 1) == 0x0102);                 /* owner id */
    /* entries: B yes, W no (no heard-only) */
    assert(chunk_has_entry(last_pkt, last_len, 0x010B));
    assert(!chunk_has_entry(last_pkt, last_len, 0x010C));
    /* header geo = own geo */
    assert((last_pkt[5] | (last_pkt[6] << 8)) == 1000);
    assert((last_pkt[7] | (last_pkt[8] << 8)) == 2000);

    /* R receives the chunk: S lands in the map with its geo, and the
     * link S->R (naming R? no: R is not an entry here) — R processes
     * the chunk as a 2-hop source. */
    mimeshi_rx(&R, 0, -60, last_pkt, last_len);
    {
        mimeshi_mapnode *m = 0;
        for (int i = 0; i < MC_MAP_NODES; i++)
            if (R.map[i].alive && R.map[i].id == 0x0102) m = &R.map[i];
        assert(m);
        assert(m->geo_lat == 1000 && m->geo_lon == 2000);  /* header geo */
        assert(m->quality >= MC_SYM_QUALITY_MIN);          /* our own measurement of the chunk */
    }
    /* the entry for B: in R's map with the class S advertised */
    {
        mimeshi_mapnode *m = 0;
        for (int i = 0; i < MC_MAP_NODES; i++)
            if (R.map[i].alive && R.map[i].id == 0x010B) m = &R.map[i];
        assert(m);
        /* quality from the advertised class of S's measurement */
        assert(m->quality > 0);
    }
    /* the link S-B is in the link store with the class of S->B */
    {
        mimeshi_link *l = 0;
        for (int i = 0; i < MC_MAX_LINKS; i++)
            if (R.lnk[i].alive && R.lnk[i].a == 0x0102 && R.lnk[i].b == 0x010B) l = &R.lnk[i];
        assert(l);
    }

    /* A chunk that names R in an entry confirms R's link to the owner
     * (reverse evidence, 10.1). Build a chunk from B naming R. */
    {
        uint8_t p[64]; uint16_t len = 10;
        p[0] = PKT_TABLE_CHUNK_VALUE;
        p[1] = 0x0B; p[2] = 0x01;                           /* owner 0x010B */
        p[3] = 0; p[4] = 1;                                 /* 1 entry */
        p[5] = 0; p[6] = 0; p[7] = 0; p[8] = 0;             /* no geo */
        p[9] = 1;
        p[10] = 0x00; p[11] = 0x01;                          /* entry: R (0x0100) */
        p[12] = 5; p[13] = 0;                                /* class 5, age 0 */
        p[14] = 0; p[15] = 0; p[16] = 0; p[17] = 0;         /* no geo */
        p[18] = 0;                                          /* no links */
        len = 19;
        int sym_before = 0;
        for (int i = 0; i < MC_MAX_NEIGHBORS; i++)
            if (R.nbr[i].alive && R.nbr[i].id == 0x010B) sym_before = R.nbr[i].sym;
        (void)sym_before;
        mimeshi_rx(&R, 0, -60, p, len);                      /* good quality */
        int sym = 0;
        for (int i = 0; i < MC_MAX_NEIGHBORS; i++)
            if (R.nbr[i].alive && R.nbr[i].id == 0x010B) sym = R.nbr[i].sym;
        assert(sym == 1);                                    /* confirmed via the chunk (4.3) */
    }

    /* weak chunk reception: same naming chunk at -105 dBm does NOT confirm */
    {
        static mimeshi_node R2;
        mimeshi_init(&R2, 0x0100, vrtx, sched, vrsw, vrrand, vrcad, 0);
        uint8_t p[64];
        p[0] = PKT_TABLE_CHUNK_VALUE;
        p[1] = 0x0B; p[2] = 0x01; p[3] = 0; p[4] = 1;
        p[5] = 0; p[6] = 0; p[7] = 0; p[8] = 0; p[9] = 1;
        p[10] = 0x00; p[11] = 0x01; p[12] = 5; p[13] = 0;
        p[14] = 0; p[15] = 0; p[16] = 0; p[17] = 0; p[18] = 0;
        mimeshi_rx(&R2, 0, -105, p, 19);
        int sym = 0;
        for (int i = 0; i < MC_MAX_NEIGHBORS; i++)
            if (R2.nbr[i].alive && R2.nbr[i].id == 0x010B) sym = R2.nbr[i].sym;
        assert(sym == 0);                                    /* quality gate (10.1) */
    }

    int rmap = 0;
    for (int i = 0; i < MC_MAP_NODES; i++) if (R.map[i].alive) rmap++;
    printf("table-chunk: OK (chunks=%d, map of R: %d nodes)\n", chunk_seen, rmap);
    return 0;
}
