/*
 * Meshimeshi — protocol core, implementation. See core.h (API + node struct)
 * and docs/design.md (the section references below point there).
 *
 * Building block 1 (link test): hello round, neighbor table, hello-acks with
 * RSSI spreading, unicast with ack1/ack2/ack3, retries, budget.
 * Forwarding, routing (10.2), T_ack2 window (7.3) and tablepull (4.3)
 * are building block 2 — where that behavior is still missing it is noted.
 *
 * Architecture (13b): the core has no clock, radio or RNG. Everything
 * comes in via the four callbacks from mimeshi_init(): tx (transmit),
 * timer (single-shot time window), switch_ch (channel switch with
 * done-callback) and rand (jitter, msg-id). Platform-independent and
 * without dynamic allocation: all buffers live fixed in the node struct
 * from core.h.
 *
 * Call model:
 *  - Platform -> core:  mimeshi_rx() per received packet (channel + RSSI),
 *                       mimeshi_send_data() for a unicast, mimeshi_start().
 *  - Core -> platform:  tx() and switch_ch() for every transmission, timer()
 *                       for every wait, rand() for every random value.
 *  - All times in microseconds (uint64_t): the simulator computes with
 *    integers, firmware with a local hardware counter (13). No clock sync.
 *
 * Switch rule (2), enforced via bound_send():
 *  - Exchange = switch to the partner's channel
 *    (MC_S_SWITCH_US), transmit as soon as the radio is on channel. The
 *    core has no explicit "return home" call: the platform layer returns
 *    to the home channel by itself after the exchange (for data after
 *    the ack1 window on the exchange channel, 7.3). The simulator
 *    (sim/sim1.py, class Node) is the reference implementation of that
 *    platform contract.
 *  - Ack1 is the exception: it goes without a switch via tx() on the
 *    channel of the ongoing exchange, because the sender waits there (7.1).
 *
 * Logging: the printf lines are intended for the link test and simulator.
 * Firmware can damp them; later they become a platform callback.
 */
#include "core.h"
#include <string.h>
#include <stdio.h>

/* ---------------- configuration (firmware constants) ----------------
 * Values and status live in the design document; change them there first
 * (working rule: design -> core -> ctest + sim1.py grow along). */
#define MC_HELLO_START_US  30000000ull     /* 30 s after startup (4.1) */
#define MC_HELLO_MAX_US    3600000000ull   /* 1 h stable (4.1); = T_keepalive */
#define MC_KEEPALIVE_TICKS 64u             /* silence ticks = T_keepalive (4.1) */
#define MC_KEEPALIVE_TICK_US (MC_HELLO_MAX_US / MC_KEEPALIVE_TICKS)  /* ~56 s: the time base without a clock */
#define MC_HELLO_JITTER_US 1000000ull      /* random phase per round (4.1, open point 3) */
#define MC_HELLO_ACK_WIN_US 3000000ull     /* max response time hello-acks: 3 s (2) */
#define MC_HELLO_ACK_JITTER_US (MC_HELLO_ACK_WIN_US / 16u)  /* ~187 ms, > t_ack + S (2) */
#define MC_BUDGET_INIT     32              /* initial value budget byte (9) */
#define MC_RETRY_TX         2              /* transmit attempts per neighbor per series (7.4) */
#define MC_ACK1_MARGIN_US   80000ull       /* processing + margin on top of t_data + t_ack (7.3) */
#define MC_S_SWITCH_US      5000ull        /* channel switch incl. settling (14.1) */
#define MC_SYM_QUALITY_MIN  14u            /* symmetry requires this quality (10.1):
                                               quality_of(-100 dBm); 10 dB above the
                                               reception threshold incl. fade margin */
#define MC_CAD_BACKOFF_US   20000ull       /* CAD backoff: random 0..19 ms (2) */
#define MC_CAD_MAX_TRIES    8u             /* ask at most 8x per transmission; then transmit anyway */

/* ---------------- airtime (SF7, 62.5 kHz, CR4/5) ----------------
 * Fixed values from the table in 14.1. The core passes the airtime to
 * tx() so the platform layer can schedule the transmit window; the
 * functions are deliberately simple (three classes) instead of an exact
 * LoRa airtime formula. hello: 3 B, hello-ack/proof list: 5 B and 3+2n B
 * (6.2); ack1/ack2: 7 B. */
static uint64_t t_ack_us(void)        { return 78000ull; }   /* ack1/ack2: 7 B */
static uint64_t t_hello_us(void)      { return 62000ull; }   /* hello: 3 B (6.3) */
static uint64_t t_hack_us(void)       { return 70000ull; }   /* hello-ack: 5 B (6.2) */
static uint64_t t_proof_us(uint8_t n) { (void)n; return 70000ull; }  /* proof list: 3+2n B, n<=17 -> 37 B max; one class (4.2) */
static uint64_t t_data_us(uint16_t l) { return l <= 40 ? 173000ull : l <= 150 ? 459000ull : 869000ull; }

/* ---------------- packet types and formats ----------------
 * Type byte: low nibble = type, high nibble = flags (6.1). All
 * multi-byte fields little-endian (rd16/wr16). Layouts:
 *
 *   HELLO       3 B   type, node-id(2)  (4.1, 6.3). Startup only: after the
 *                     adaptive phase the hello no longer exists; the state
 *                     version is dropped (no pull anymore, 4.3).
 *   HELLO_ACK   5 B   type, destination-id(2) = hello sender,
 *                     sender-id(2) — directional: the address makes the ack
 *                     an exact symmetry proof (5.1 ms symbol time incl.; 2, 4.2)
 *   PROOF       3+2n B type, node-id(2), n x neighbour-id(2), n <= 17 (4.2).
 *                     Reactionless keepalive + symmetry flush; an empty list
 *                     is the minimal keepalive beacon (type + node-id).
 *   DATA      10+n B  type+F_ACK3_REQ, msg-id(2), source(2), destination(2),
 *                     previous-hop(2), budget(1, offset 9), payload(n)  (6.1)
 *   ACK1        7 B   type, msg-id(2), source of the message(2), sender(2) (6.2)
 *   ACK2        7 B   same as ACK1; F_NEG_ACK2 makes it negative       (6.2, 7.2)
 *   ACK3       14 B   DATA format, payload 4 B: the original msg-id (6.2, 11)
 *
 * TABLE_PULL / TABLE_CHUNK (4.3) and FAIL_RPT (10.5) are reserved for
 * their future function and are ignored at the moment.
 */
enum {
    PKT_HELLO = 1, PKT_HELLO_ACK, PKT_PROOF, PKT_TABLE_CHUNK,
    PKT_DATA, PKT_ACK1, PKT_ACK2, PKT_ACK3, PKT_FAIL_RPT
};
#define F_ACK3_REQ 0x10   /* requested by the source of the message (11) */
#define F_NEG_ACK2 0x40   /* next hop could not forward (7.2) */

/* little-endian reading/writing of 16-bit fields (6.1) */
static uint16_t rd16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static void wr16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }

/* ---------------- keepalive counters (4.1) ----------------
 * Per channel the node counts how many keepalive ticks have passed since
 * it last "called" there itself — an identifying transmission on that
 * channel (hello, hello-ack, ack1, ack2, data as prev_hop, ack3, soon
 * pull traffic). ANN and FULL are anonymous and therefore do NOT reset.
 * Every own call sets the counter of that channel to 0; after
 * MC_KEEPALIVE_TICKS of silence exactly one hello follows on that
 * channel (ka_tick). The core has no clock: the tick timer IS the time
 * base. */
static void ka_called(mimeshi_node *n, uint8_t ch) {
    if (ch < MC_N_CHANNELS) n->ka_idle[ch] = 0;
}

/* ---------------- neighbor table and dedup ---------------- */

/* Look up a neighbor by id. Linear search: MC_MAX_NEIGHBORS is small and
 * the table is only touched on reception or transmit choice. */
static mimeshi_neighbor *nbr_find(mimeshi_node *n, uint16_t id) {
    for (int i = 0; i < MC_MAX_NEIGHBORS; i++) if (n->nbr[i].alive && n->nbr[i].id == id) return &n->nbr[i];
    return 0;
}

/* Quality byte from an RSSI measurement (4.2): halving of the normalized
 * RSSI leaves room for a future SNR component in the same byte. */
static uint8_t quality_of(int8_t rssi) {
    return (uint8_t)(rssi < 0 ? (rssi + 128) >> 1 : 63);
}

/* Quality gate for symmetry confirmation (10.1): a link is used only
 * when it is above the quality threshold and works in both directions.
 * Each node can only measure the direction toward itself (the RSSI of
 * the received transmissions); the neighbour measures the opposite
 * direction itself and gates its own confirmation with the same rule.
 * The combined effect: a link is confirmed symmetric only when BOTH
 * directions are above the threshold. */
static bool quality_ok(uint8_t q) { return q >= MC_SYM_QUALITY_MIN; }

/* Advertised quality class (4.3): the upper bits of the long-term
 * average, 3 bits (8 levels) in the prefix chunks. */
static uint8_t adv_class_of(uint8_t q) { return (uint8_t)(q >> 3); }   /* 0..7 */

/* Quality byte back from an advertised class (receiver side): the
 * middle of the class, so route comparisons stay monotone. */
static uint8_t quality_from_class(uint8_t cls) { return (uint8_t)((cls << 3) | 4); }

/* EWMA on the long-term average (4.3): weight 1/32. */
static uint8_t quality_ewma(uint8_t stored, uint8_t fresh) {
    return (uint8_t)(((uint16_t)stored * 31u + fresh) / 32u);
}


/* Add a neighbor (from a hello, hello-ack or proof list). Full = return 0;
 * the caller continues without the neighbor. A new entry starts as "seen":
 * one direction, sym = 0 (4.2). The local table counter feeds push-window
 * bundling (4.3); it never goes on air. */
static mimeshi_neighbor *nbr_add(mimeshi_node *n, uint16_t id) {
    for (int i = 0; i < MC_MAX_NEIGHBORS; i++)
        if (!n->nbr[i].alive) {
            n->nbr[i].alive = 1; n->nbr[i].id = id;
            n->nbr[i].sym = 0; n->nbr[i].ack_try = 0;
            n->local_tbl_ctr++;
            return &n->nbr[i];
        }
    return 0;
}

/* Dedup on (msg-id, source). false = duplicate, do not process again.
 * msg-id 0 counts as an empty slot: every message with id 0 is therefore
 * passed through; msg-ids come from the RNG (16 bit) and may be 0 —
 * known and acceptable for building block 1, included in the dedup
 * hardening. Full cache = receive anyway; LRU replacement comes later
 * (14, layer 3). */
static bool dedup_fresh(mimeshi_node *n, uint16_t msg_id, uint16_t src) {
    for (int i = 0; i < MC_MAX_DEDUP; i++)
        if (n->dedup[i].msg_id == msg_id && n->dedup[i].src == src) return false;
    for (int i = 0; i < MC_MAX_DEDUP; i++) if (n->dedup[i].msg_id == 0) {
        n->dedup[i].msg_id = msg_id; n->dedup[i].src = src; return true;
    }
    return true; /* full cache: receive anyway (LRU comes later) */
}

/* ---------------- exchange (switch rule, section 2) ---------
 * bound_send() is the only path for anything that must be sent on a
 * foreign channel: hellos, hello-acks, ack2, ack3, data. The core copies
 * the packet into a fixed pool and asks the platform layer to switch;
 * the done-callback (bound_fire) transmits as soon as the radio is on
 * channel and then frees the pool slot. The platform layer must
 * serialize switches per node (one radio) and return home after the
 * transmission.
 *
 * The pool has as many slots as pending records: every active message
 * holds at most one exchange at a time, because a retry only starts
 * after the ack1 timeout. No heap (13b); len == 0 marks a free slot.
 * On firmware one node runs per process, so MC_MAX_PENDING suffices;
 * the simulator shares the core between all nodes of the scenario —
 * build the .so with -DMC_BOUND_POOL=256 in that case (see README).
 */
#ifndef MC_BOUND_POOL
#define MC_BOUND_POOL MC_MAX_PENDING
#endif
typedef struct { uint8_t ch; uint8_t cad_tries; uint16_t len; uint64_t airtime; uint8_t pkt[10 + MC_MAX_PAYLOAD]; } bound_ctx;
static bound_ctx bctx[MC_BOUND_POOL]; /* pool per transmission; small and fixed */
static uint32_t bctx_drops;          /* full pool = defect, never silent */

static void bound_fire(mimeshi_node *n, void *ud) {
    bound_ctx *c = (bound_ctx *)ud;
    /* CAD, channel activity detection (2): after the switch the radio
     * is on channel and can sense the carrier. Busy = wait with random
     * backoff on the channel and ask again; the pool slot stays
     * reserved, so a busy channel delays but never drops the
     * transmission. After MC_CAD_MAX_TRIES the transmission goes
     * anyway: a permanently busy channel must not block the node
     * forever (hello round, proof list). Ack1 never comes through here:
     * it is a direct reply and goes without a switch and without a
     * check via tx() (7.1). Not RTS/CTS against hidden nodes (8). */
    if (n->channel_free && c->cad_tries < MC_CAD_MAX_TRIES &&
        !n->channel_free(n, c->ch)) {
        c->cad_tries++;
        n->timer(n, 1 + (n->rand(n) % MC_CAD_BACKOFF_US), bound_fire, c);
        return;
    }
    n->tx(n, c->ch, c->pkt, c->len, c->airtime);
    c->len = 0; /* slot free */
}

/* Transmit on channel ch after a switch. Drop the transmission if the
 * pool is full — with the right pool size that is a defect, so with
 * counter and message, never silent. */
static void bound_send(mimeshi_node *n, uint8_t ch, const uint8_t *pkt, uint16_t len, uint64_t airtime) {
    for (int i = 0; i < MC_BOUND_POOL; i++) {
        if (bctx[i].len == 0) {
            bctx[i].ch = ch; bctx[i].cad_tries = 0; bctx[i].len = len; bctx[i].airtime = airtime;
            memcpy(bctx[i].pkt, pkt, len);
            n->switch_ch(n, ch, MC_S_SWITCH_US, bound_fire, &bctx[i]);
            return;
        }
    }
    bctx_drops++;
    printf("[core] node %u: bound-pool full, transmission dropped (counter %u)\n", n->id, bctx_drops);
}

/* ---------------- hello (4.1) ---------------- */

/* One hello round: the same 3 B packet on all three channels, in fixed
 * order. Reasons (2): neighbors sit randomly on all home channels, and
 * the fixed order gives responders a predictable moment when the sender
 * is listening on its own channel again — their hello-acks go there.
 * Startup only (4.1). Cost: 3 x 3 B with adaptive interval. */
static void hello_round(mimeshi_node *n) {
    uint8_t p[3] = { PKT_HELLO, 0, 0 };
    wr16(p + 1, n->id);
    for (uint8_t c = 0; c < MC_N_CHANNELS; c++) {          /* fixed order */
        bound_send(n, c, p, 3, t_hello_us());
        ka_called(n, c);                                   /* own call on c (4.1) */
    }
    /* after the round the node is home; hello-acks arrive there (within the window) */
}

/* Random phase per round. Without jitter the following holds (4.1): nodes
 * that start up simultaneously (e.g. after a power outage) keep exactly
 * synchronous rounds — the intervals double in sync — and never hear
 * each other, because during its own round a node is away from home.
 * Range in hello_jitter_us so the simulation can measure it per
 * scenario (open point 3). */
static uint64_t hello_jitter(mimeshi_node *n) {
    return n->hello_jitter_us ? (n->rand(n) % n->hello_jitter_us) : 0;
}

/* Run a round, double the interval (30 s -> 60 s -> ... -> 1 h) and
 * schedule the next round. The doubling is per (4.1): nodes that have
 * been present longer disturb less. The jitter sits on the timer, not
 * on the round itself. At the maximum (1 h) the startup phase stops
 * (4.1): this tick no longer sends a round and no longer schedules
 * one — from now on the hello only survives as per-channel keepalive
 * (ka_tick below). */
static void hello_tick(mimeshi_node *n, void *ud) {
    (void)ud;
    if (n->hello_interval_us >= MC_HELLO_MAX_US) return;
    hello_round(n);
    n->hello_interval_us *= 2;
    if (n->hello_interval_us > MC_HELLO_MAX_US) n->hello_interval_us = MC_HELLO_MAX_US;
    n->timer(n, n->hello_interval_us + hello_jitter(n), hello_tick, 0);
}

/* Proof list for channel c (4.2): type, node-id, then the ids of the
 * direct neighbors at home on c whose reverse-direction evidence
 * ("last proven") expires within the flush margin. Reactionless by
 * design: no receiver ever replies. An empty list is the minimal
 * keepalive beacon (4.1). The packet is an own call on c and resets the
 * keepalive counter of c.
 *
 * Combined trigger (4.2): the flush check runs every 20 min rotating
 * per channel; a channel that has been silent for T_keepalive pulls
 * its moment forward -- same packet, one airtime for both roles.
 * Cap: at most one proof list per channel per hour. */
#define MC_PROOF_MAX_IDS 17

static void proof_send(mimeshi_node *n, uint8_t c) {
    uint8_t p[3 + 2 * MC_PROOF_MAX_IDS] = { PKT_PROOF, 0, 0 };
    uint8_t cnt = 0;
    wr16(p + 1, n->id);
    for (int i = 0; i < MC_MAX_NEIGHBORS && cnt < MC_PROOF_MAX_IDS; i++) {
        mimeshi_neighbor *nb = &n->nbr[i];
        if (!nb->alive || !nb->sym) continue;       /* flush works on confirmed links */
        if ((uint8_t)(nb->id % MC_N_CHANNELS) != c) continue;  /* sub-list per channel */
        if (nb->proven_age_us >= MC_T_SYM_US - MC_SYM_MARGIN_US) {  /* inside flush margin */
            wr16(p + 3 + 2 * cnt, nb->id);
            cnt++;
        }
    }
    bound_send(n, c, p, (uint16_t)(3 + 2 * cnt), t_proof_us(cnt));
    ka_called(n, c);            /* own call: keepalive counter of c resets (4.1) */
}

static void proof_fire(mimeshi_node *n, void *ud) {
    proof_send(n, (uint8_t)(size_t)ud);
}

/* Flush check, rotating per channel (4.2). Each channel has its own
 * countdown in proof_wait[] ticks; the stagger (c x MC_PROOF_ROT_TICKS)
 * gives each channel one flush moment per hour, 20 minutes apart: no
 * self-congestion. Next to the countdown, a channel silent for
 * T_keepalive (ka_idle at max) is due NOW even without its slot -- the
 * keepalive trigger pulls the moment forward (4.1). The countdown
 * restarts at MC_KEEPALIVE_TICKS (= 1 h) on every fire: that is the cap,
 * one proof list per channel per hour maximum.
 */
#define MC_PROOF_ROT_TICKS (MC_KEEPALIVE_TICKS / MC_N_CHANNELS)  /* 20 min in ticks (4.2) */

/* ---------------- 2-hop map and link store (4.3, 10.1) --------
 * The map holds every node we know: direct neighbours (alive in the
 * neighbor table) and 2-hop nodes from table chunks. The link store
 * holds the advertised links between map nodes (per direction a 3-bit
 * class). Both are bounded (13.2): full = the entry is dropped with a
 * message, never silently. */

static mimeshi_mapnode *map_find(mimeshi_node *n, uint16_t id) {
    for (int i = 0; i < MC_MAP_NODES; i++)
        if (n->map[i].alive && n->map[i].id == id) return &n->map[i];
    return 0;
}

static mimeshi_mapnode *map_add(mimeshi_node *n, uint16_t id) {
    mimeshi_mapnode *m = map_find(n, id);
    if (m) return m;
    for (int i = 0; i < MC_MAP_NODES; i++)
        if (!n->map[i].alive) {
            n->map[i].alive = 1; n->map[i].id = id;
            n->map[i].geo_lat = 0; n->map[i].geo_lon = 0;
            n->map[i].quality = 0; n->map[i].age_ticks = 255;
            n->map[i].arrived_tick = n->tick;
            return &n->map[i];
        }
    printf("[core] node %u: map full, entry %u dropped\n", n->id, id);
    return 0;
}

/* Link between two map nodes, endpoints normalized a < b. */
static mimeshi_link *lnk_find(mimeshi_node *n, uint16_t a, uint16_t b) {
    if (a > b) { uint16_t t = a; a = b; b = t; }
    for (int i = 0; i < MC_MAX_LINKS; i++)
        if (n->lnk[i].alive && n->lnk[i].a == a && n->lnk[i].b == b) return &n->lnk[i];
    return 0;
}

static mimeshi_link *lnk_add(mimeshi_node *n, uint16_t a, uint16_t b) {
    mimeshi_link *l = lnk_find(n, a, b);
    if (l) return l;
    if (a > b) { uint16_t t = a; a = b; b = t; }
    for (int i = 0; i < MC_MAX_LINKS; i++)
        if (!n->lnk[i].alive) {
            n->lnk[i].alive = 1; n->lnk[i].a = a; n->lnk[i].b = b;
            return &n->lnk[i];
        }
    printf("[core] node %u: link store full, link %u-%u dropped\n", n->id, a, b);
    return 0;
}

/* Table push mutation feed (4.3): the local table counter drives the
 * push-window bundling. A mutation schedules the prefix chunks after
 * the short window; mutations inside the window bundle automatically
 * because only the current state goes on air. */
#define MC_TBL_WIN_US   30000000ull   /* bundling window: 30 s (4.3, to measure) */
#define MC_TBL_PERIOD_TICKS 10u           /* prefix re-share: ~10 min (4.3 cadence, to measure) */

static void tbl_push_fire(mimeshi_node *n, void *ud);

static void tbl_mark_dirty(mimeshi_node *n) {
    n->tbl_dirty = 1;
    if (!n->tbl_push_sched) {
        n->tbl_push_sched = 1;
        n->timer(n, MC_TBL_WIN_US + (n->rand(n) % MC_TBL_WIN_US), tbl_push_fire, 0);
    }
}

/* ---------------- table chunks (4.3) ----------------
 * Prefix chunk, node-centric and self-contained (4.3):
 *   header: type(1) owner-id(2) chunk-nr(1) n-entries(1)
 *           owner-geo(4) fw-version(1)                      = 10 B
 *   entry:  node-id(2) class(1) age(1) geo(4) link-count(1) = 9 B
 *   link:   endpoint-id(2) class-entry->endpoint(4 bits)
 *           class-endpoint->entry(4 bits)                   = 3 B
 * Only confirmed symmetric neighbours are advertised (decided:
 * no "heard only" in the chunks). The entry class is the owner's
 * measured quality of the link (EWMA average, 3 bits); the fading
 * asymmetry per direction (+-5 dB) is below the class width, so one
 * class stands for both directions of a direct link. The link
 * records under an entry carry the links between the entry-node and
 * our other confirmed neighbours, with the classes per direction as
 * they are known in the link store. */
#define MC_CHUNK_MAX_ENTRIES 8        /* 8 entries x (9 B + links) fits 230 B */

/* Direction-aware access to the link store: nibbles keyed to the
 * sorted endpoints (a < b): high nibble = a->b, low = b->a. */
static uint8_t lnk_class(mimeshi_node *n, uint16_t from, uint16_t to) {
    mimeshi_link *l = lnk_find(n, from, to);
    if (!l) return 0xFF;                        /* unknown */
    uint16_t a = from < to ? from : to;
    return (from == a) ? (uint8_t)((l->q >> 4) & 0x0F) : (uint8_t)(l->q & 0x0F);
}

static void lnk_set_class(mimeshi_node *n, uint16_t from, uint16_t to, uint8_t cls) {
    if (cls > 7) return;                        /* unknown stays unknown */
    mimeshi_link *l = lnk_add(n, from, to);
    if (!l) return;
    uint16_t a = from < to ? from : to;
    if (from == a) l->q = (uint8_t)((l->q & 0x0F) | (cls << 4));
    else           l->q = (uint8_t)((l->q & 0xF0) | cls);
}

/* Effective age of a map entry in proof ticks (~56 s), capped at 255
 * (= ~4 h, expired, 4.4). */
static uint8_t map_age_eff(mimeshi_node *n, mimeshi_mapnode *m) {
    uint32_t a = (uint32_t)m->age_ticks + (n->tick - m->arrived_tick);
    return (uint8_t)(a > 255 ? 255 : a);
}

static void chunk_send(mimeshi_node *n) {
    uint8_t p[230];
    uint16_t len = 10;
    uint8_t cnt = 0;
    int start = n->chunk_cursor;                  /* continuation: the table cycles (4.3) */
    for (int k = 0; k < MC_MAX_NEIGHBORS && cnt < MC_CHUNK_MAX_ENTRIES && len + 9 <= 230; k++) {
        int i = (start + k) % MC_MAX_NEIGHBORS;
        mimeshi_neighbor *nb = &n->nbr[i];
        if (!nb->alive || !nb->sym) continue;      /* only confirmed symmetric (4.3) */
        wr16(p + len, nb->id);
        p[len + 2] = adv_class_of(nb->quality);   /* measured EWMA class (3 bits) */
        p[len + 3] = 0;                            /* direct measurement: fresh */
        mimeshi_mapnode *m = map_find(n, nb->id);
        if (m && m->geo_lat) { wr16(p + len + 4, m->geo_lat); wr16(p + len + 6, m->geo_lon); }
        else { wr16(p + len + 4, 0); wr16(p + len + 6, 0); }
        uint16_t lcnt_off = len + 8;
        len += 9;
        uint8_t lcnt = 0;
        for (int j = 0; j < MC_MAX_NEIGHBORS && len + 3 <= 230; j++) {
            if (j == i) continue;
            mimeshi_neighbor *nb2 = &n->nbr[j];
            if (!nb2->alive || !nb2->sym) continue;
            uint8_t cf = lnk_class(n, nb->id, nb2->id);
            if (cf == 0xFF) continue;              /* never heard about this pair */
            uint8_t cr = lnk_class(n, nb2->id, nb->id);
            wr16(p + len, nb2->id);
            p[len + 2] = (uint8_t)((cf << 4) | (cr == 0xFF ? 0 : cr));
            len += 3; lcnt++;
        }
        p[lcnt_off] = lcnt;
        cnt++;
        n->chunk_cursor = (uint8_t)((i + 1) % MC_MAX_NEIGHBORS);   /* continue here next push */
    }
    if (cnt == 0) { n->chunk_cursor = 0; return; } /* nothing to share yet */
    p[0] = PKT_TABLE_CHUNK;
    wr16(p + 1, n->id);
    p[3] = n->chunk_nr++;
    p[4] = cnt;
    wr16(p + 5, n->geo_lat); wr16(p + 7, n->geo_lon);
    p[9] = 1;                                     /* firmware version */
    /* All three channels, in fixed order — the same reason as the hello
     * round (2): confirmed neighbours sit on all home channels. A chunk
     * on the own home channel alone would reach only the same-channel
     * third; the links between cross-channel neighbours would then never
     * propagate and the 2-hop map would stay blind for exactly the
     * links that Gabriel and route choice need. Cost per chunk: 3 x
     * 869 ms air per push round; a full tower table stays far under the
     * duty limit. The push bundling (one chunk per window) caps the
     * rate; the pool has a slot per transmission like the hello round. */
    for (uint8_t c = 0; c < MC_N_CHANNELS; c++) {
        bound_send(n, c, p, len, t_data_us(len));
        ka_called(n, c);                          /* own call on c (4.1) */
    }
}

static void tbl_push_fire(mimeshi_node *n, void *ud) {
    (void)ud;
    n->tbl_push_sched = 0;
    if (!n->tbl_dirty) return;
    n->tbl_dirty = 0;
    chunk_send(n);
}

/* Reception of a table chunk (4.3). Self-contained: every entry is
 * processed on its own against the own map and link store. The owner
 * naming US in an entry is reverse-direction evidence for the link
 * owner->us (10.1) — gated on quality like every confirmation. */
static void chunk_rx(mimeshi_node *n, int8_t rssi, const uint8_t *pkt, uint16_t len) {
    if (len < 10) return;
    uint16_t owner = rd16(pkt + 1);
    uint8_t n_ent = pkt[4];
    mimeshi_mapnode *om = map_add(n, owner);
    if (om) {
        om->geo_lat = rd16(pkt + 5); om->geo_lon = rd16(pkt + 7);
        om->quality = quality_of(rssi);       /* our measurement of owner->us */
        om->age_ticks = 0; om->arrived_tick = n->tick;
    }
    mimeshi_neighbor *nb = nbr_find(n, owner);
    if (!nb) nb = nbr_add(n, owner);
    if (nb) nb->quality = quality_ewma(nb->quality ? nb->quality : quality_of(rssi), quality_of(rssi));
    uint16_t off = 10;
    for (uint8_t e = 0; e < n_ent && off + 9 <= len; e++) {
        uint16_t eid = rd16(pkt + off);
        uint8_t cls = pkt[off + 2] & 0x07;     /* owner->entry class (4.3) */
        uint8_t age = pkt[off + 3];
        uint16_t gl = rd16(pkt + off + 4), go = rd16(pkt + off + 6);
        uint8_t lcnt = pkt[off + 8];
        off += 9;
        if (eid == n->id) {                    /* owner names us: reverse evidence (10.1) */
            if (nb) {
                nb->proven_age_us = 0;
                if (quality_ok(nb->quality)) {
                    if (!nb->sym) tbl_mark_dirty(n);
                    nb->sym = 1; nb->ack_try = 0;
                }
            }
        } else {
            mimeshi_mapnode *em = map_add(n, eid);
            if (em) {
                if (gl) { em->geo_lat = gl; em->geo_lon = go; }   /* (0,0) = none (5) */
                uint8_t travelled = age;
                if (!em->quality || travelled < map_age_eff(n, em)) {
                    em->quality = quality_from_class(cls);        /* fresh beats old (4.3) */
                    em->age_ticks = travelled; em->arrived_tick = n->tick;
                }
            }
            lnk_set_class(n, owner, eid, cls); /* owner -> entry */
        }
        for (uint8_t k = 0; k < lcnt && off + 3 <= len; k++) {
            uint16_t peer = rd16(pkt + off);
            uint8_t lb = pkt[off + 2];
            off += 3;
            if (peer == n->id) continue;       /* link to us: covered by our own entry */
            map_add(n, peer);                 /* presence in the 2-hop set (10.1) */
            lnk_set_class(n, eid, peer, (uint8_t)(lb >> 4));
            lnk_set_class(n, peer, eid, (uint8_t)(lb & 0x0F));
        }
    }
}

static void proof_tick(mimeshi_node *n, void *ud) {
    (void)ud;
    n->tick++;                                 /* local time base, ~56 s per tick (4.4) */
    /* Periodic prefix re-share (4.3): without traffic there are no
     * mutations, and a chunk missed by a neighbour (collision, away
     * from home) would then never return. Every MC_TBL_PERIOD_TICKS
     * (~10 min) the prefix goes out again. The per-node phase offset
     * (id mod period) spreads the re-share moments over the whole
     * period: 40 nodes at the same tick would put 40 x 869 ms on each
     * channel inside one bundling window - guaranteed collisions. The
     * random jitter in the bundling window adds the fine spreading. */
    if ((n->tick + (n->id % MC_TBL_PERIOD_TICKS)) % MC_TBL_PERIOD_TICKS == 0)
        tbl_mark_dirty(n);
    for (int i = 0; i < MC_MAX_NEIGHBORS; i++)          /* evidence ages per tick (4.2) */
        if (n->nbr[i].alive && n->nbr[i].proven_age_us < MC_T_SYM_US)
            n->nbr[i].proven_age_us += MC_KEEPALIVE_TICK_US;
    for (uint8_t c = 0; c < MC_N_CHANNELS; c++) {
        if (n->ka_idle[c] < MC_KEEPALIVE_TICKS) n->ka_idle[c]++;
        uint8_t due = n->proof_wait[c] == 0 || n->ka_idle[c] >= MC_KEEPALIVE_TICKS;
        if (due) {
            n->ka_idle[c] = 0;              /* reset on scheduling: one packet, no storm */
            n->proof_wait[c] = MC_KEEPALIVE_TICKS;        /* cap: next slot after 1 h */
            n->timer(n, hello_jitter(n), proof_fire, (void *)(size_t)c);
        } else n->proof_wait[c]--;
    }
    n->timer(n, MC_KEEPALIVE_TICK_US, proof_tick, 0);
}

/* Deliver a hello-ack on the HOME channel of the hello sender (2): the
 * sender is there after its round and catches all answers within the
 * response window. One exchange, then home.
 *
 * Directional format (5 B): the destination is the hello sender. Only
 * because of that does the ack prove symmetry — the responder received
 * OUR hello (its ack is a reaction to it) and its ack reached us
 * (4.2, 10.1). That is the proof channel of the hello process:
 * available before the first unicast. Eavesdroppers process the sender
 * as a minimal hello ("seen"); the state version is dropped (4.3: no
 * pull anymore). */
static void hello_ack_fire(mimeshi_node *n, void *ud) {
    uint16_t to = (uint16_t)(uintptr_t)(size_t)ud;
    uint8_t p[5] = { PKT_HELLO_ACK, 0,0, 0,0 };
    wr16(p + 1, to);                       /* destination: the hello sender */
    wr16(p + 3, n->id);
    bound_send(n, (uint8_t)(to % MC_N_CHANNELS), p, 5, t_hack_us()); /* to HOME of the sender */
    ka_called(n, (uint8_t)(to % MC_N_CHANNELS));    /* own call on that channel (4.1) */
}

/* Wait time and decision for the hello-ack, after receiving a hello:
 *
 * Strongest first (deliberate), with slot + jitter:
 *   - RSSI determines the SLOT (the rank): the strongest responder
 *     transmits in the first part of the window.
 *   - The jitter is larger than the airtime of an ack (Oct 2026, ~187 ms
 *     versus 78 ms + switch): with the old window/64 (47 ms), acks with
 *     exactly equal rank certainly overlapped. Now the jitter spreads
 *     equivalent responders out.
 *   - The slot calculation reserves jitter + S + t_ack at the end of
 *     the window, so every ack (incl. switch and airtime) falls within
 *     the reception window of the sender.
 *   - Price: the ranking is "soft" — responders within ~16 dB can swap
 *     order. Preventing collisions weighs heavier than strict order (2).
 *
 * Ack on demand (4.1, Oct 2026): a confirmed neighbor (sym, 4.2) gets
 * two more courtesy acks after our own confirmation and after that not
 * a single one — snooping already maintains liveness, version and
 * quality. The two extra acks are needed because confirmation almost
 * always proceeds asymmetrically: if one direction succeeds first, the
 * acks from that side stop, while the neighbor still has to take its
 * proof from our ack (in the simulation: capture lost N's direction
 * while the opposite side was already confirmed — without courtesy
 * that proof starves until an eventual tablepull).
 * An unconfirmed neighbor stays in the table, but the ack chance
 * thins out over time: every HEARD transmission of the neighbor counts
 * (counter per neighbor), and only the counter values 1, 2, 4, 8, 16
 * yield an attempt, after that a drip of on average 1 in 16
 * transmissions. This way an ack storm around a tower site resolves
 * itself: the pressure on the window drops, the remaining attempts
 * continue, successful proofs stop the acks (after the courtesy)
 * completely — and one-directional links cost at most a drip. Reset of
 * the counter on confirmation or renewed contact (nbr_add); count on
 * transmissions, not on attempts — otherwise lost attempts get stuck
 * without a third chance.
 */
static void hello_ack_wait(mimeshi_node *n, uint16_t from, int8_t rssi) {
    /* Quality gate (10.1), same threshold as the confirmation: a hello
     * below it can never become a routing link (our confirmation would
     * be blocked by the gate, the neighbour's too), so an ack on it is
     * pure airtime -- and around boundary links it feeds the ack
     * storm the on-demand scheme is there to prevent. The counter does
     * not count this transmission: a weak hello is not evidence of a
     * usable link. */
    if (!quality_ok(quality_of(rssi))) return;
    mimeshi_neighbor *nb = nbr_find(n, from);
    int ack = 1;                                 /* unknown sender: always ack */
    if (nb) {
        if (nb->sym) {
            if (nb->ack_try >= 3) return;       /* courtesy used up (after 2) */
            nb->ack_try++;
        } else {
            if (nb->ack_try < 255) nb->ack_try++;   /* counter per neighbor */
            uint32_t c = nb->ack_try;
            ack = c <= 16 && (c & (c - 1)) == 0;    /* attempts at counter value 1,2,4,8,16 */
            if (!ack && c > 16) ack = (n->rand(n) % 16u) == 0;  /* drip: on average 1 in 16 transmissions */
        }
    }
    if (!ack) return;
    int norm = (rssi + 128) > 255 ? 255 : (rssi + 128);   /* 0..255 */
    uint32_t span = (uint32_t)(MC_HELLO_ACK_WIN_US - MC_HELLO_ACK_JITTER_US
                             - t_ack_us() - MC_S_SWITCH_US);
    uint32_t slot = (uint32_t)((255 - norm) * span / 255u);
    uint32_t jitter = n->rand(n) % (MC_HELLO_ACK_JITTER_US + 1u);
    n->timer(n, slot + jitter, hello_ack_fire, (void *)(size_t)from);
}

/* ---------------- unicast + acks (7) ---------------- */

/* Ack1: immediately, on the channel of the ongoing data exchange — the
 * sender waits there for the answer (switch rule, 2). The sender-id
 * travels along so the sender knows who has been heard. */
static void ack1_send(mimeshi_node *n, uint16_t msg_id, uint16_t src, uint8_t ch) {
    uint8_t p[7] = { PKT_ACK1, 0,0, 0,0, 0,0 };
    wr16(p + 1, msg_id); wr16(p + 3, src); wr16(p + 5, n->id);
    n->tx(n, ch, p, 7, t_ack_us());       /* same channel as the data (switch rule) */
    ka_called(n, ch);                     /* own call on the data channel (4.1) */
}

/* Ack2 to the previous hop, on ITS home channel (2, 7.1): the previous
 * hop listens there without its own window. Positive = received (not
 * necessarily forwarded, 7.2); negative = forwarding failed, via
 * F_NEG_ACK2. */
static void ack2_send(mimeshi_node *n, uint16_t msg_id, uint16_t src, uint16_t prev, bool neg) {
    uint8_t p[7] = { (uint8_t)(PKT_ACK2 | (neg ? F_NEG_ACK2 : 0)), 0,0, 0,0, 0,0 };
    wr16(p + 1, msg_id); wr16(p + 3, src); wr16(p + 5, n->id);
    bound_send(n, (uint8_t)(prev % MC_N_CHANNELS), p, 7, t_ack_us()); /* HOME of previous hop */
    ka_called(n, (uint8_t)(prev % MC_N_CHANNELS));  /* own call there (4.1) */
}

static void pending_try(mimeshi_node *n, mimeshi_pending *p);

/* Ack1 stayed out (7.3/7.4). structure of the retries:
 *  - tries_left > 0:  next attempt toward THE SAME next hop,
 *                      tries and budget decrease by 1 per attempt;
 *  - tries exhausted: new attempt series (toward a different neighbor,
 *                      as soon as routing exists — 10.2), budget first
 *                      checks whether there is still room;
 *  - budget exhausted: drop the message. The real failed delivery
 *                      report (10.5, routed message to the source) comes
 *                      with building block 2; for now only the message
 *                      on stdout. */
static void ack1_timeout(mimeshi_node *n, void *ud) {
    mimeshi_pending *p = (mimeshi_pending *)ud;
    if (!p->active) return;
    if (p->tries_left > 0) {
        p->tries_left--; p->budget_left--;
        pending_try(n, p);
    } else {
        p->tries_left = MC_RETRY_TX;      /* next attempt series */
        if (p->budget_left == 0) {
            p->active = 0;
            printf("[core] node %u: FAIL msg %u (budget exhausted, no ack1)\n", n->id, p->msg_id);
        } else {
            p->budget_left--;
            pending_try(n, p);            /* neighbor choice: real core ETX-driven (10.2) */
        }
    }
}

/* One transmit attempt: update the budget in the header (offset 9,
 * 6.1/9), the exchange toward next_hop, and schedule the ack1 timeout.
 * The timeout is (7.3) S + t_data + t_ack + margin, measured from the
 * moment of submission. Assumption: the platform layer delivers the
 * transmission quickly (queue short); if the switch takes longer, the
 * timeout fires too early and the attempt counts as failed. */
static void pending_try(mimeshi_node *n, mimeshi_pending *p) {
    p->pkt[9] = p->budget_left;
    bound_send(n, (uint8_t)(p->next_hop % MC_N_CHANNELS), p->pkt, p->pkt_len, t_data_us(p->pkt_len));
    ka_called(n, (uint8_t)(p->next_hop % MC_N_CHANNELS));   /* data as prev_hop (4.1) */
    n->timer(n, MC_S_SWITCH_US + t_data_us(p->pkt_len) + t_ack_us() + MC_ACK1_MARGIN_US, ack1_timeout, p);
}

/* Submit a unicast. Return codes (core.h): 0 = accepted, -1 =
 * destination not in the neighbor table (10.2: do not flood, wait until
 * the map knows it), -2 = no free pending slot, -3 = payload too large.
 * Building block 1 chooses the destination itself as next hop (direct
 * neighbor); from building block 2 a router here chooses the best hop
 * on the 2-hop map.
 *
 * One-direction proof: every neighbor table entry originated from a
 * reception (hello or hello-ack) and therefore only proves the
 * direction neighbor->us. The opposite direction (us->neighbor) is only
 * proven by the neighbor itself: via its tablepull (4.3) or via a
 * successful unicast with ack1. Routing (building block 2, 10.1) must
 * demand that confirmation before forwarding; until then this direct
 * sending is bounded by retries+budget and fails visibly on the ack1
 * timeout. */
int mimeshi_send_data(mimeshi_node *n, uint16_t dst, const uint8_t *payload, uint16_t len, bool ack3) {
    if (!nbr_find(n, dst)) return -1;          /* not in the map: do not flood (10.2) */
    if (len > MC_MAX_PAYLOAD) return -3;
    mimeshi_pending *p = 0;
    for (int i = 0; i < MC_MAX_PENDING; i++) if (!n->pend[i].active) { p = &n->pend[i]; break; }
    if (!p) return -2;
    memset(p, 0, sizeof(*p));
    p->active = 1; p->tries_left = MC_RETRY_TX; p->budget_left = MC_BUDGET_INIT;
    p->msg_id = (uint16_t)n->rand(n);
    p->src = n->id; p->dst = dst; p->prev_hop = n->id; p->next_hop = dst;
    uint8_t *k = p->pkt;
    k[0] = (uint8_t)(PKT_DATA | (ack3 ? F_ACK3_REQ : 0));
    wr16(k + 1, p->msg_id); wr16(k + 3, n->id); wr16(k + 5, dst); wr16(k + 7, n->id);
    k[9] = p->budget_left;                 /* header: budget at offset 9 (section 6.1) */
    memcpy(k + 10, payload, len);
    p->pkt_len = (uint16_t)(10 + len);
    pending_try(n, p);
    return 0;
}

/* ---------------- reception (platform -> core) ----------------
 * Every received packet enters here, with the channel of reception
 * (determines where ack1 must go back) and the RSSI (determines the
 * hello-ack slot). Packets for another destination are forwarding work
 * and are deliberately ignored until building block 2. */
void mimeshi_rx(mimeshi_node *n, uint8_t ch, int8_t rssi, const uint8_t *pkt, uint16_t len) {
    (void)len;
    uint8_t type = pkt[0] & 0x0F, flags = pkt[0] & 0xF0;
    switch (type) {
    case PKT_HELLO: {
        /* Startup beacon (3 B, 4.1): update or add the neighbor, then
         * schedule an ack. Quality is the RSSI measurement of this
         * reception; last_seen is still a stub — real aging policy (4.4)
         * follows. No state version on air anymore (4.3). */
        uint16_t from = rd16(pkt + 1);
        mimeshi_neighbor *nb = nbr_find(n, from);
        if (!nb) nb = nbr_add(n, from);
        if (nb) {
            nb->last_seen = 0;
            nb->quality = quality_of(rssi);
            nb->proven_age_us = 0;      /* fresh reception: reverse direction freshly measured */
        }
        hello_ack_wait(n, from, rssi);
        break;
    }
    case PKT_HELLO_ACK: {
        /* Directional ack (5 B): destination = the original hello
         * sender.
         *  - Addressed to US: the responder received our hello and
         *    this ack reached us — both directions worked at that
         *    moment: symmetry proven (4.2, 10.1). This is the proof
         *    channel of the hello process, available before the first
         *    unicast.
         *  - Addressed to someone else (snooping, 2): the sender is
         *    processed as a minimal hello — id and the quality measured
         *    here — but stays "seen": one-direction proof, no symmetry.
         *    The ack triggers no new reaction: the type byte serves
         *    that purpose. */
        uint16_t dst = rd16(pkt + 1), from = rd16(pkt + 3);
        mimeshi_neighbor *nb = nbr_find(n, from);
        if (!nb) nb = nbr_add(n, from);
        if (nb) {
            nb->last_seen = 0;
            nb->quality = quality_of(rssi);
            nb->proven_age_us = 0;      /* fresh reception of an identifying transmission */
            if (dst == n->id && quality_ok(nb->quality)) {
                if (!nb->sym) tbl_mark_dirty(n);       /* new advertised neighbour (4.3) */
                nb->sym = 1; nb->ack_try = 0;          /* confirmed symmetric (4.2, 10.1) */
            }
        }
        break;
    }
    case PKT_PROOF: {
        /* Proof list (4.2): type, node-id, n x neighbour-id. Reactionless:
         * never reply, whatever the content. Two effects:
         *  - WE appear in the list: the sender claims it hears us —
         *    both directions confirmed in a single reception (4.2),
         *    reverse evidence freshly refreshed.
         *  - We do not appear: snooping — liveness and quality for the
         *    sender, "seen" at most. No downgrade either: absence in
         *    one packet is no negative evidence (4.2, decided). */
        uint16_t from = rd16(pkt + 1);
        uint16_t ids = (uint16_t)((len - 3) / 2);
        mimeshi_neighbor *nb = nbr_find(n, from);
        if (!nb) nb = nbr_add(n, from);
        if (nb) {
            nb->last_seen = 0;
            nb->quality = quality_of(rssi);
        }
        /* appearing in the list = reverse evidence refreshed (4.2);
         * symmetry only above the quality threshold (10.1) */
        for (uint16_t i = 0; i < ids; i++) {
            if (rd16(pkt + 3 + 2 * i) == n->id && nb) {
                nb->proven_age_us = 0;
                if (quality_ok(nb->quality) && !nb->sym) { nb->sym = 1; nb->ack_try = 0; tbl_mark_dirty(n); }
            }
        }
        break;
    }
    case PKT_ACK1: {
        /* our own outstanding transmission has arrived at the next hop:
         * free the pending record. The hop is done; ack2 arrives on its
         * own on our home channel (7.1). Matching on msg-id suffices in
         * building block 1 (ids are random per transmission); with
         * routing added, the source should be counted along.
         *
         * The sender of this ack1 received our message AND its ack
         * reached us: the link is hereby proven symmetric (4.2). This
         * is the first exact symmetry proof the core knows; the second
         * is the tablepull of the neighbor (4.3, building block 2). A
         * hello-ack proves no symmetry — unaddressed on a shared home
         * channel. */
        uint16_t msg_id = rd16(pkt + 1);
        for (int i = 0; i < MC_MAX_PENDING; i++) {
            mimeshi_pending *p = &n->pend[i];
            if (p->active && p->msg_id == msg_id) {
                p->active = 0;                     /* hop succeeded; ack2 arrives on its own on our home channel */
                mimeshi_neighbor *nb = nbr_find(n, rd16(pkt + 5));
                if (nb) {                               /* confirmed symmetric (4.2, 10.1) */
                    nb->quality = quality_of(rssi);     /* the ack1 itself is the measurement (its direction) */
                    nb->proven_age_us = 0;
                    if (quality_ok(nb->quality)) {
                        if (!nb->sym) tbl_mark_dirty(n);
                        nb->sym = 1; nb->ack_try = 0;
                    }
                }
                printf("[core] node %u: ACK1 for msg %u\n", n->id, msg_id);
                return;
            }
        }
        break;
    }
    case PKT_ACK2: {
        /* confirmation of the next hop toward the previous one (7.2).
         * For now only visible; the T_ack2 window and the pending
         * handling on the sender (7.3) belong to building block 2. */
        printf("[core] node %u: ACK2 %s for msg %u\n", n->id,
               (flags & F_NEG_ACK2) ? "NEGATIVE" : "positive", rd16(pkt + 1));
        break;
    }
    case PKT_ACK3: {
        /* final destination has had the message delivered (11): routed
         * back to the source, here still without routing logic. */
        printf("[core] node %u: ACK3 — delivered at destination\n", n->id);
        break;
    }
    case PKT_TABLE_CHUNK: {
        chunk_rx(n, rssi, pkt, len);
        break;
    }
    case PKT_DATA: {
        uint16_t msg_id = rd16(pkt + 1), src = rd16(pkt + 3), dst = rd16(pkt + 5), prev = rd16(pkt + 7);
        if (dst != n->id) return;                  /* forwarding: next building block */
        /* dedup before the acks: a retry of the same hop must not
         * yield ack1/ack2/ack3 again. */
        if (!dedup_fresh(n, msg_id, src)) return;
        printf("[core] node %u: DATA from %u: '%.*s'\n", n->id, src, (int)(len - 10), (const char *)(pkt + 10));
        ack1_send(n, msg_id, src, ch);             /* same channel */
        ack2_send(n, msg_id, src, prev, false);    /* home channel previous hop */
        if (flags & F_ACK3_REQ) {
            /* ack3 as a routed message (6.2): DATA format, payload = the
             * original msg-id (4 B incl. reservation), back toward the
             * previous hop — until there is real routing (building
             * block 2). */
            uint8_t a[14] = { PKT_ACK3, 0,0, 0,0, 0,0, 0,0, 0, 0,0, 0,0 };
            wr16(a + 1, (uint16_t)n->rand(n)); wr16(a + 3, n->id);
            wr16(a + 5, src); wr16(a + 7, n->id); a[9] = MC_BUDGET_INIT; wr16(a + 10, msg_id);
            bound_send(n, (uint8_t)(prev % MC_N_CHANNELS), a, 14, t_data_us(14));
            ka_called(n, (uint8_t)(prev % MC_N_CHANNELS));   /* own call (4.1) */
        }
        break;
    }
    default: break;
    }
}

/* ---------------- init/start ---------------- */

/* Zeroing + filling in the callbacks. The home channel follows from the
 * id (id mod 3, 2). hello_jitter_us gets the firmware default; the
 * simulator may change it before mimeshi_start() to measure the range
 * (open point 3). The node struct lives in core.h so callers can
 * reserve it statically — hence the zeroing here. */
void mimeshi_init(mimeshi_node *n, uint16_t id, mimeshi_tx_fn tx, mimeshi_timer_fn timer,
                mimeshi_switch_fn sw, mimeshi_rand_fn rnd,
                mimeshi_channel_free_fn cad, void *user) {
    memset(n, 0, sizeof(*n));
    n->id = id; n->home_ch = (uint8_t)(id % MC_N_CHANNELS);
    n->local_tbl_ctr = 1; n->hello_interval_us = MC_HELLO_START_US;
    n->hello_jitter_us = MC_HELLO_JITTER_US;
    n->tx = tx; n->timer = timer; n->switch_ch = sw; n->rand = rnd; n->user = user;
    n->channel_free = cad;
}

/* First hello round immediately (fast map building, 4.1), then the
 * adaptive timer with random phase. Idempotent for the rest of the
 * core: after start the hello runs on the timer; do not repeat
 * mimeshi_start() itself. After the startup phase the hello stops; the
 * proof tick carries keepalive and symmetry flush from here (4.1, 4.2). */
/* Stagger the flush moments (4.2): channel c is due for the first time
 * after c x 20 min, so the three channels lie 20 minutes apart. */
static void proof_stagger(mimeshi_node *n) {
    for (uint8_t c = 0; c < MC_N_CHANNELS; c++)
        n->proof_wait[c] = (uint8_t)(c * MC_PROOF_ROT_TICKS);
}

/* Own geo in the 16-bit grid (5): (0,0) = none. A later call with a
 * different value is a mutation and feeds the table push (4.3). */
void mimeshi_set_geo(mimeshi_node *n, uint16_t lat, uint16_t lon) {
    if (lat == n->geo_lat && lon == n->geo_lon) return;
    n->geo_lat = lat; n->geo_lon = lon;
    tbl_mark_dirty(n);
}

void mimeshi_start(mimeshi_node *n) {
    hello_round(n);
    proof_stagger(n);
    /* random phase: prevents synchronous rounds on simultaneous startup (4.1) */
    n->timer(n, n->hello_interval_us + hello_jitter(n), hello_tick, 0);
    /* proof tick: time base for keepalive + symmetry flush (4.1, 4.2) */
    n->timer(n, MC_KEEPALIVE_TICK_US, proof_tick, 0);
}
