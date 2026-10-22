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
#define MC_CAD_TRIES        8               /* busy-channel backoff attempts before
                                            transmitting anyway (2) */
#define MC_CAD_WAIT_US      20000ull        /* base backoff on a busy channel (2) */

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
 *   DATA      15+n B  type+F_ACK3_REQ, msg-id(2), source(2), destination(2),
 *                     previous-hop(2), next-hop(2, routing: only that node
 *                     forwards and acks), budget(1, offset 10),
 *                     dst-geo(4, always present, 6.1: routing tandem 10.2),
 *                     payload(n)
 *   ACK1        7 B   type, msg-id(2), source of the message(2), sender(2) (6.2)
 *                     -- the sender is the next hop that received it
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
#define F_NO_REPORT 0x20  /* this message is a failed-delivery report:
                              never trigger a report of its own (10.5) */
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

/* Update the quality of a neighbour per 4.3: an EWMA of the long-term
 * average, stored = (31 x stored + new) / 32, weight 1/32; the first
 * measurement initialises the stored value. A marginal link is thereby
 * classified CONSISTENTLY: it stays under the routing threshold instead
 * of slipping through on a good moment -- the decided principle: if it
 * works it works well, and a marginal link fails clearly (the owner
 * fixes the antenna) instead of delivering half the time.
 * A change of at least 4 steps (advertisement hysteresis, 4.3) is a table
 * mutation: it moves the local table counter for the next chunk push. */
static void nbr_set_quality(mimeshi_node *n, mimeshi_neighbor *nb, int8_t rssi) {
    uint8_t q = quality_of(rssi);
    uint8_t before = nb->quality;
    if (nb->quality == 0) nb->quality = q;                          /* first measurement (4.3) */
    else nb->quality = (uint8_t)(((uint32_t)nb->quality * 31u + q) / 32u);   /* EWMA (4.3) */
    int diff = (int)nb->quality - (int)before;
    if (diff < 0) diff = -diff;
    if (diff >= 4) n->local_tbl_ctr++;
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
typedef struct { uint8_t ch; uint16_t len; uint64_t airtime; uint8_t tries; uint8_t pkt[10 + MC_MAX_PAYLOAD]; } bound_ctx;
static bound_ctx bctx[MC_BOUND_POOL]; /* pool per transmission; small and fixed */
static uint32_t bctx_drops;          /* full pool = defect, never silent */

static void bound_fire(mimeshi_node *n, void *ud);

/* CAD check at the moment of transmission (2): if the channel is busy,
 * back off with a random wait and re-check. After MC_CAD_TRIES attempts
 * the packet goes out anyway -- best effort beats starving. Without a
 * busy callback (NULL) the transmission is immediate. */
static void bound_check_and_tx(mimeshi_node *n, bound_ctx *c) {
    if (n->busy && c->tries < MC_CAD_TRIES && n->busy(n, c->ch)) {
        c->tries++;
        n->timer(n, (n->rand(n) % MC_CAD_WAIT_US) + MC_CAD_WAIT_US / 2, bound_fire, c);
        return;
    }
    n->tx(n, c->ch, c->pkt, c->len, c->airtime);
    c->len = 0; /* slot free */
}

static void bound_fire(mimeshi_node *n, void *ud) {
    bound_check_and_tx(n, (bound_ctx *)ud);
}

/* Transmit on channel ch after a switch. Drop the transmission if the
 * pool is full — with the right pool size that is a defect, so with
 * counter and message, never silent. */
static void bound_send(mimeshi_node *n, uint8_t ch, const uint8_t *pkt, uint16_t len, uint64_t airtime) {
    for (int i = 0; i < MC_BOUND_POOL; i++) {
        if (bctx[i].len == 0) {
            bctx[i].ch = ch; bctx[i].len = len; bctx[i].airtime = airtime; bctx[i].tries = 0;
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
static void proof_tick(mimeshi_node *n, void *ud) {
    (void)ud;
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

static mimeshi_2hop *known_find(mimeshi_node *n, uint16_t id);
static mimeshi_2hop *known_upsert(mimeshi_node *n, uint16_t id, uint16_t via,
                                  uint8_t quality, int16_t gx, int16_t gy,
                                  const uint16_t *links, const uint8_t *lq, uint8_t nlinks);

/* ---------------- table chunks (4.3) ----------------
 * Push, no pull, no ack (4.3). Node-centric slice format per 4.3: the
 * entry of a node is followed directly by ITS links, so the receiver
 * builds a link graph and can compute paths (10.2) -- not just guess
 * on position. Layout per entry: id(2), quality(1), nlinks(1),
 * geox(2), geoy(2), then nlinks x (endpoint-id(2), quality(1)).
 * Variable length, self-contained, no order dependence.
 */
static uint8_t chunk_entry_put(mimeshi_node *n, uint8_t *e, uint16_t id,
                               uint8_t quality, int16_t gx, int16_t gy,
                               const uint16_t *links, const uint8_t *lq, uint8_t nlinks) {
    (void)n;
    wr16(e, id);
    e[2] = quality;
    e[3] = (uint8_t)nlinks;
    wr16(e + 4, (uint16_t)gx); wr16(e + 6, (uint16_t)gy);
    for (uint8_t i = 0; i < nlinks; i++) {
        wr16(e + 8 + 3 * i, links[i]);
        e[8 + 3 * i + 2] = lq[i];
    }
    return (uint8_t)(8 + 3 * nlinks);
}

static void chunk_send(mimeshi_node *n, uint8_t ch, uint8_t with_links) {
    uint8_t p[240];
    p[0] = PKT_TABLE_CHUNK; wr16(p + 1, n->id);
    wr16(p + 3, (uint16_t)n->geox); wr16(p + 5, (uint16_t)n->geoy);  /* owner geo (4.3) */
    uint16_t off = 8;
    uint8_t cnt = 0;
    /* FIRST entry: ourselves (4.3 node-centric: our own links -- the only
     * place where our full degree exists; every receiver learns our
     * complete link set from this single entry) */
    {
        uint16_t mylinks[MC_CHUNK_LINKS]; uint8_t myq[MC_CHUNK_LINKS];
        uint8_t ml = 0;
        for (int i = 0; i < MC_MAX_NEIGHBORS && ml < MC_CHUNK_LINKS; i++) {
            mimeshi_neighbor *nb = &n->nbr[i];
            if (!nb->alive || !nb->sym) continue;
            mylinks[ml] = nb->id; myq[ml] = nb->quality; ml++;
        }
        off += chunk_entry_put(n, p + off, n->id, 0, n->geox, n->geoy, mylinks, myq, ml);
        cnt++;
    }
    /* confirmed direct neighbours (10.1): the entry carries the links of
     * that neighbour as we know them -- our edge to it plus everything we
     * learned FROM its own adverts (its full degree lives in its own
     * first entry; we propagate what we received) */
    uint16_t lbuf[MC_CHUNK_LINKS]; uint8_t qbuf[MC_CHUNK_LINKS];
    for (int i = 0; i < MC_MAX_NEIGHBORS && cnt < MC_CHUNK_ENTRIES; i++) {
        mimeshi_neighbor *nb = &n->nbr[i];
        if (!nb->alive || !nb->sym) continue;
        /* the links OF this neighbour, as we know them (4.3: node-centric):
         * first our own link to it (the edge nb--us), then the links we
         * learned for nb from other chunks. Without the learned links the
         * graph never fills: nobody could ever forward a third node's links. */
        uint8_t nl = 0;
        if (nb->sym && nl < MC_CHUNK_LINKS) {
            lbuf[nl] = n->id;                       /* our own edge to nb */
            qbuf[nl] = nb->quality;
            nl++;
        }
        mimeshi_2hop *leh = known_find(n, nb->id);
        if (leh) {
            for (uint8_t l = 0; l < leh->nlinks && nl < MC_CHUNK_LINKS; l++) {
                uint8_t dup = 0;
                for (uint8_t d = 0; d < nl; d++) if (lbuf[d] == leh->link[l]) { dup = 1; break; }
                if (dup) continue;
                lbuf[nl] = leh->link[l];
                qbuf[nl] = leh->lq[l] ? leh->lq[l] : leh->quality;
                nl++;
            }
        }
        off += chunk_entry_put(n, p + off, nb->id, nb->quality, nb->geox, nb->geoy,
                               with_links ? lbuf : (const uint16_t *)0,
                               with_links ? qbuf : (const uint8_t *)0,
                               with_links ? nl : 0);
        cnt++;
        if (off > 240 - 30) break;
    }
    /* then known-map entries round-robin (4.3), with their learned links.
     * The cursor advances by the number of MAP entries actually sent
     * (map_sent) -- the self-entry and the direct neighbours occupy fixed
     * slots every slice, so advancing by the total count (cnt) skips most
     * of the map (measured: 78 pushes carried only 13 of 26 map ids). */
    uint8_t map_sent = 0;
    for (int i = 0; i < MC_MAX_KNOWN && cnt < MC_CHUNK_ENTRIES && off <= 240 - 30; i++) {
        mimeshi_2hop *h = &n->known[(n->chunk_rr + i) % MC_MAX_KNOWN];
        if (!h->alive || nbr_find(n, h->id)) continue;
        off += chunk_entry_put(n, p + off, h->id, h->quality, h->geox, h->geoy,
                               with_links ? h->link : (const uint16_t *)0,
                               with_links ? h->lq : (const uint8_t *)0,
                               with_links ? h->nlinks : 0);
        cnt++;
        map_sent++;
    }
    n->chunk_rr = (uint8_t)((n->chunk_rr + map_sent) % MC_MAX_KNOWN);
    p[7] = cnt;
    bound_send(n, ch, p, off, t_data_us(off));   /* CAD applies (2): chunks are
                                                    never direct exchange replies */
    ka_called(n, ch);                        /* own call on that channel (4.1) */
}

static void chunk_tick(mimeshi_node *n, void *ud) {
    (void)ud;
    /* 4.3: push on mutation (the counter moved -- short window, slices
     * bundle), AND the periodic repetition: knowledge diffuses outward
     * only if slices keep going out even in a stable net ("recovery
     * consists of the regularity of the push itself", 4.3). The repeat
     * counter runs in chunk ticks; MC_CHUNK_REPEAT_TICKS is the slow
     * tail cadence. */
    n->chunk_ticks++;
    uint8_t mutated = n->chunk_ctr_last != n->local_tbl_ctr
                   && n->chunk_ticks >= MC_CHUNK_MUT_MIN_TICKS;   /* bundle mutations: the
                                                                     push waits for the window */
    /* the tail cadence rises with the number of direct neighbours n
     * (4.3: per doubling of n the pause doubles) -- the marginal
     * airtime of a slice rises with n, and hub sites carry most of it.
     * Base 10 ticks (~20 min) at n<=8; every doubling halves the rate. */
    uint32_t nbrs = 0;
    for (int i = 0; i < MC_MAX_NEIGHBORS; i++) if (n->nbr[i].alive) nbrs++;
    uint32_t repeat_ticks = MC_CHUNK_REPEAT_TICKS;
    while (nbrs > 8) { repeat_ticks *= 2; nbrs >>= 1; }
    if (repeat_ticks > 250) repeat_ticks = 250;
    if (mutated || n->chunk_ticks >= repeat_ticks) {
        n->chunk_ctr_last = n->local_tbl_ctr;
        n->chunk_ticks = 0;
        if (n->hello_interval_us < MC_HELLO_MAX_US) {
            for (uint8_t ch = 0; ch < MC_N_CHANNELS; ch++)
                chunk_send(n, ch, 1);       /* startup: all channels, with links (4.3) */
        } else {
            /* stable: mutations carry links (fresh knowledge needs the
             * graph, 4.3); the periodic tail repeats geo+quality only --
             * the link graph fills via the mutation pushes, the tail
             * keeps the map fresh at 8 B per entry */
            /* stable tail: ROTATE over the channels (4.3). One slice per
             * repetition -- the airtime stays one slice, the rotation only
             * broadens where it lands, so every channel community receives
             * the map over the course of three repetitions. Without this,
             * links between nodes on different home channels never enter
             * anyone's graph (measured: the dominant void cause). */
            uint8_t ch = n->chunk_rr_ch;
            chunk_send(n, ch, 1);
            n->chunk_rr_ch = (uint8_t)((n->chunk_rr_ch + 1) % MC_N_CHANNELS);
        }
    }
    n->timer(n, MC_CHUNK_PERIOD_US + (n->rand(n) % 1000000ull), chunk_tick, 0);
}

/* Process a received chunk: every entry is a 2-hop node heard via the
 * sender (10.1). Also refreshes the sender itself (snooping, 2). */
static void chunk_recv(mimeshi_node *n, uint8_t ch, int8_t rssi, const uint8_t *pkt, uint16_t len) {
    (void)ch;
    uint16_t from = rd16(pkt + 1);
    if (len < 8) return;
    uint8_t cnt = pkt[7];
    if ((uint16_t)(8 + 7 * cnt) > len) return;        /* malformed: ignore */
    /* owner geo from the header (4.3): the position of the direct
     * neighbour -- the only geo source for direct neighbours, because
     * the hello carries none. */
    int16_t ogx = (int16_t)rd16(pkt + 3), ogy = (int16_t)rd16(pkt + 5);
    mimeshi_neighbor *snb = nbr_find(n, from);
    if (snb) {
        nbr_set_quality(n, snb, rssi); snb->last_seen = 0;
        if (ogx != MC_GEO_UNKNOWN) { snb->geox = ogx; snb->geoy = ogy; }
    }
    const uint8_t *e = pkt + 8;
    const uint8_t *end = pkt + len;
    for (uint8_t i = 0; i < cnt && e + 8 <= end; i++) {
        uint16_t x = rd16(e);
        uint8_t nl = e[3];
        if (e + 8 + 3 * (uint16_t)nl > end) break;    /* malformed: ignore the rest */
        int16_t gx = (int16_t)rd16(e + 4), gy = (int16_t)rd16(e + 6);
        /* the links of x, straight from the entry (4.3) */
        uint16_t links[MC_CHUNK_LINKS]; uint8_t lq[MC_CHUNK_LINKS];
        uint8_t take = nl < MC_CHUNK_LINKS ? nl : MC_CHUNK_LINKS;
        for (uint8_t j = 0; j < take; j++) {
            links[j] = rd16(e + 8 + 3 * j);
            lq[j] = e[8 + 3 * j + 2];
        }
        if (x == n->id && snb) {
            /* an entry for US in the neighbour's chunk is reverse evidence
             * (4.3): "I hear X" confirms at X both directions. */
            snb->sym = 1; snb->ack_try = 0; snb->proven_age_us = 0;
        } else if (x == from) {
            /* the SENDER's own first entry: its links are the sender's
             * complete degree (4.3 node-centric), each quality being the
             * direction the SENDER measures (4.3: quality per direction).
             * A link to US carries their measurement of our transmissions:
             * that is the us->sender direction -> quality_back. */
            for (uint8_t j = 0; j < take; j++) {
                if (links[j] == n->id && snb) {
                    if (lq[j]) {
                        /* EWMA (4.3): their advertised measurement of our
                         * direction is also a long-term average */
                        if (snb->quality_back == 0) snb->quality_back = lq[j];
                        else snb->quality_back = (uint8_t)(((uint32_t)snb->quality_back * 31u + lq[j]) / 32u);
                    }
                }
            }
            known_upsert(n, from, from, 63, gx, gy, links, lq, take);
        } else {
            known_upsert(n, x, from, e[2], gx, gy, links, lq, take);
        }
        e += 8 + 3 * nl;
    }
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

/* ---------------- routing and forwarding (7, 10) ----------------
 * Building block 2. The next hop is chosen per 10.2:
 *  1. dst is a confirmed direct neighbour: direct hop;
 *  2. dst is in the 2-hop map (10.1): via the neighbour that hears dst,
 *     if that neighbour is confirmed symmetric;
 *  3. dst outside the map but a geo known (own table or source geo in
 *     the header): greedy — the confirmed neighbour closest to dst
 *     that is closer than we are (10.2);
 *  4. no neighbour closer (void): fail this attempt series — waypoint
 *     and perimeter follow later (10.3);
 *  5. nothing known about dst: refuse (do not flood, 10.2).
 * Only confirmed links are used for the choice (10.1); direct replies
 * in an exchange are exempt (4.3) and run via the normal ack path.
 */

/* squared distance on the geo grid (10.2); unknown geo never wins */
static int32_t geo_dist2(int16_t ax, int16_t ay, int16_t bx, int16_t by) {
    int32_t dx = (int32_t)ax - bx, dy = (int32_t)ay - by;
    return dx * dx + dy * dy;
}

/* Look up a 2-hop entry: node id -> via which direct neighbour. */
static mimeshi_2hop *known_find(mimeshi_node *n, uint16_t id) {
    for (int i = 0; i < MC_MAX_KNOWN; i++)
        if (n->known[i].alive && n->known[i].id == id) return &n->known[i];
    return 0;
}

/* Eviction score of a known-map entry (10.1): quality of the link that
 * carried it, freshness, and hub role. Direct neighbours score highest
 * (they are the hop candidates); far entries with a low quality and a
 * weak hub role are displaced first. Purely local. */
static uint8_t known_score(mimeshi_node *n, uint16_t via, uint8_t quality) {
    mimeshi_neighbor *nb = nbr_find(n, via);
    int s = quality;
    if (nb && nb->sym) s += 32;                   /* confirmed direct via: strongest */
    else if (nb) s += 16;                         /* seen direct via */
    for (int i = 0; i < MC_MAX_KNOWN; i++)        /* hub role: others travel via this entry */
        if (n->known[i].alive && n->known[i].via == via) s++;
    if (s > 255) s = 255;
    return (uint8_t)s;
}

/* Insert or refresh a known-map entry; on a full map the weakest entry
 * is displaced (10.1). Weaker-than-existing entries do not displace. */
static mimeshi_2hop *known_upsert(mimeshi_node *n, uint16_t id, uint16_t via,
                                  uint8_t quality, int16_t gx, int16_t gy,
                                  const uint16_t *links, const uint8_t *lq, uint8_t nlinks) {
    if (id == n->id) return 0;
    mimeshi_2hop *h = known_find(n, id);
    if (!h) {
        int slot = -1;
        uint8_t ns = known_score(n, via, quality);
        for (int i = 0; i < MC_MAX_KNOWN; i++)
            if (!n->known[i].alive) { slot = i; break; }
        if (slot < 0) {                            /* full: displace the weakest */
            uint8_t worst = 255; int wi = -1;
            for (int i = 0; i < MC_MAX_KNOWN; i++)
                if (n->known[i].score < worst) { worst = n->known[i].score; wi = i; }
            if (worst >= ns) return 0;             /* the newcomer is the weakest: skip */
            slot = wi;
        }
        h = &n->known[slot];
        h->alive = 1; h->id = id; h->via = via;
    }
    h->quality = quality;
    h->score = known_score(n, via, quality);
    if (gx != MC_GEO_UNKNOWN) { h->geox = gx; h->geoy = gy; }
    if (nlinks) {                                  /* merge learned links (4.3) */
        for (uint8_t i = 0; i < nlinks && i < MC_CHUNK_LINKS; i++) {
            uint8_t found = 0;
            for (uint8_t j = 0; j < h->nlinks; j++)
                if (h->link[j] == links[i]) { found = 1; break; }
            if (!found && h->nlinks < MC_CHUNK_LINKS) {
                h->link[h->nlinks] = links[i];
                h->lq[h->nlinks] = lq ? lq[i] : 0;
                h->nlinks++;
            }
        }
    }
    return h;
}

/* Candidate for the next hop: quality, penalised on failures (10.4).
 * Links under the quality threshold are no candidates at all (10.1):
 * "links are used only if they are above a quality threshold". A link at
 * the receive edge (-110 dBm ~ quality 9) drops half its packets; routing
 * must prefer the slightly longer reliable path. Threshold: quality 10
 * (about -108 dBm). */
#define MC_ROUTE_MIN_QUALITY 10

/* Effective quality of a learned link (a, b): the minimum of both
 * advertised directions. Entry a's link to b carries b->a (a measured);
 * entry b's link to a carries a->b (b measured). min = the bottleneck. */
static uint8_t link_eff_quality(mimeshi_node *n, uint16_t a, uint16_t b) {
    mimeshi_2hop *ha = known_find(n, a);
    uint8_t q = 0;
    if (ha) {
        for (uint8_t l = 0; l < ha->nlinks; l++)
            if (ha->link[l] == b) { q = ha->lq[l]; break; }   /* direction b->a */
    }
    mimeshi_2hop *hb = known_find(n, b);
    if (hb) {
        for (uint8_t l = 0; l < hb->nlinks; l++) {
            if (hb->link[l] == a) {                          /* direction a->b */
                if (!q || hb->lq[l] < q) q = hb->lq[l];
                break;
            }
        }
    }
    return q;
}

/* Directional thresholds (asymmetric links, decided): the DATA travels
 * us->neighbour and must pass MC_ROUTE_MIN_QUALITY. The ACK travels back
 * (7 B, a fraction of the airtime) and survives a weaker link: it needs
 * only MC_ACK_MIN_QUALITY. A link that is good toward the neighbour but
 * weak back is therefore USABLE for data (the small ack still makes it);
 * only both-weak links are no candidates. */
#define MC_ACK_MIN_QUALITY 6      /* ~ -116 dBm: the ack direction floor */
static int route_score(mimeshi_node *n, mimeshi_neighbor *nb) {
    (void)n;
    if (!nb->sym) return -1;                 /* only confirmed links (10.1) */
    if (!nb->quality_back) return -1;        /* the DATA direction must be MEASURED by
                                                the neighbour (its chunk self-entry): a
                                                one-direction view is not enough to
                                                route a 173-459 ms packet over this link */
    uint8_t qfwd = nb->quality_back;         /* us->nb (as the neighbour measures) */
    uint8_t qback = nb->quality;              /* nb->us (as we measure) */
    if (qfwd < MC_ROUTE_MIN_QUALITY) return -1;   /* the DATA direction must pass */
    if (qback < MC_ACK_MIN_QUALITY) return -1;    /* the small ack still fits */
    return ((int)qfwd + (int)qback) / 2 - (nb->penalty << 3);  /* penalty (10.4) */
}

/* Path computation on the learned link graph (10.2). The graph: our own
 * confirmed direct links (the freshest, always present) plus the links
 * learned from the chunks (4.3: node-centric entries carry their links).
 * Dijkstra with link quality as weight: cost = (256 - quality); the
 * entries link[] arrays give the edges between known nodes. Result: the
 * FIRST hop of the best path to dst, or 0 when dst is unreachable in
 * the graph. excl is not taken as the first hop (7.2: no ping-pong). */
#define MC_DIJ_INF 0x7fffffff
static int32_t dij_cost(uint8_t q) { return 256 - (int32_t)q; }

static mimeshi_neighbor *route_path(mimeshi_node *n, uint16_t dst, mimeshi_neighbor *excl) {
    /* node ids -> compact index: 0 = us, 1.. our neighbours and known ids */
    static uint16_t ids[MC_MAX_NEIGHBORS + MC_MAX_KNOWN + 1];
    static int32_t dist[MC_MAX_NEIGHBORS + MC_MAX_KNOWN + 1];
    static uint16_t first[MC_MAX_NEIGHBORS + MC_MAX_KNOWN + 1];   /* first hop toward that node */
    static uint8_t done[MC_MAX_NEIGHBORS + MC_MAX_KNOWN + 1];
    int nn = 1;                                     /* index 0 = ourselves */
    ids[0] = n->id;

    /* index map: our confirmed direct neighbours (hop candidates) and
     * every alive known-map entry */
    int idx_of[MC_MAX_NEIGHBORS + MC_MAX_KNOWN + 1];
    (void)idx_of;
    for (int i = 0; i < MC_MAX_NEIGHBORS; i++) {
        mimeshi_neighbor *nb = &n->nbr[i];
        if (!nb->alive) continue;
        ids[nn] = nb->id; nn++;
    }
    for (int i = 0; i < MC_MAX_KNOWN; i++) {
        mimeshi_2hop *h = &n->known[i];
        if (!h->alive) continue;
        int dup = 0;
        for (int j = 0; j < nn; j++) if (ids[j] == h->id) { dup = 1; break; }
        if (!dup) { ids[nn] = h->id; nn++; }
    }

    /* Dijkstra init */
    for (int i = 0; i < nn; i++) { dist[i] = MC_DIJ_INF; first[i] = 0; done[i] = 0; }
    dist[0] = 0;

    /* helper: index of a node id, -1 if absent */
    #define IDX(x) ({ int r_ = -1; for (int j_ = 0; j_ < nn; j_++) if (ids[j_] == (x)) { r_ = j_; break; } r_; })

    for (int round = 0; round < nn; round++) {
        int u = -1; int32_t du = MC_DIJ_INF;
        for (int i = 0; i < nn; i++)
            if (!done[i] && dist[i] < du) { du = dist[i]; u = i; }
        if (u < 0 || du == MC_DIJ_INF) break;
        done[u] = 1;
        if (ids[u] == dst) break;

        /* edges from u: our own direct links when u == 0, otherwise the
         * learned links of the known entry with this id */
        if (u == 0) {
            for (int i = 0; i < MC_MAX_NEIGHBORS; i++) {
                mimeshi_neighbor *nb = &n->nbr[i];
                if (!nb->alive || !nb->sym) continue;
                if (!nb->quality_back) continue;               /* DATA direction must be measured (10.1) */
                if (nb->quality_back < MC_ROUTE_MIN_QUALITY) continue;   /* DATA direction (10.1) */
                if (nb->quality < MC_ACK_MIN_QUALITY) continue;  /* ack direction */
                int v = IDX(nb->id);
                if (v < 0) continue;
                int32_t nd = dist[0] + dij_cost((int32_t)(nb->quality_back + nb->quality) / 2);
                if (nd < dist[v]) {
                    dist[v] = nd;
                    first[v] = nb->id;            /* the first hop on the path to v */
                }
            }
        } else {
            /* the learned links of this known node */
            for (int k = 0; k < MC_MAX_KNOWN; k++) {
                mimeshi_2hop *h = &n->known[k];
                if (!h->alive || h->id != ids[u]) continue;
                for (uint8_t l = 0; l < h->nlinks; l++) {
                    int v = IDX(h->link[l]);
                    if (v < 0) continue;
                    uint8_t lq2 = link_eff_quality(n, ids[u], h->link[l]);
                    int32_t nd = dist[u] + dij_cost(lq2 ? lq2 : h->quality);
                    if (nd < dist[v]) {
                        dist[v] = nd;
                        first[v] = first[u] ? first[u] : ids[u];   /* propagate the first hop */
                        if (!first[v]) first[v] = ids[u];
                    }
                }
                break;
            }
        }
    }

    int t = IDX(dst);
    if (t < 0 || dist[t] == MC_DIJ_INF || first[t] == 0) return 0;
    mimeshi_neighbor *hop = nbr_find(n, first[t]);
    if (!hop || !hop->sym || hop == excl) return 0;
    if (!hop->quality_back) return 0;               /* DATA direction must be measured (10.1) */
    if (hop->quality_back < MC_ROUTE_MIN_QUALITY) return 0;   /* DATA direction (10.1) */
    if (hop->quality < MC_ACK_MIN_QUALITY) return 0;  /* ack direction */
    return hop;
    #undef IDX
}

/* Choose the next hop toward dst (10.2). dst_geo: the destination geo if
 * known (own map or source geo). Returns the neighbour or 0. */
static mimeshi_neighbor *route_next(mimeshi_node *n, uint16_t dst,
                                     int16_t dgx, int16_t dgy, mimeshi_neighbor *excl) {
    /* excl excludes the previous ATTEMPT (retry series). Callers pass
     * the hop where the packet came from as well: a packet must never
     * bounce straight back (7.2: re-route, not ping-pong). */
    /* 1. direct, confirmed */
    mimeshi_neighbor *best = 0;
    int best_score = -1;
    for (int i = 0; i < MC_MAX_NEIGHBORS; i++) {
        mimeshi_neighbor *nb = &n->nbr[i];
        if (!nb->alive || nb == excl) continue;
        if (nb->id == dst && nb->sym) return nb;
    }
    /* 2. path on the learned link graph (10.2): with the links from the
     * chunks (4.3) the node computes a real path -- the first hop of the
     * best-quality path wins over every heuristic. The direct case (1)
     * above is the one-hop path and already returned. */
    mimeshi_neighbor *ph = route_path(n, dst, excl);
    if (ph) return ph;

    /* 2b. fallback: the known map without links -- the entry names a via;
     * if that via is a confirmed direct neighbour it is the hop candidate
     * (the graph was incomplete; this is the old behaviour). */
    mimeshi_2hop *h = known_find(n, dst);
    if (h) {
        mimeshi_neighbor *via = nbr_find(n, h->via);
        if (via && via->sym && via != excl) {
            best = via; best_score = route_score(n, via) + 64;
        }
    }
    /* 3. geo-greedy via the tandem (10.2, decided): every hop only needs
     * to make progress TOWARD THE DESTINATION GEO -- not to reach it. The
     * next hop looks at its own map and continues; the message converges
     * hop by hop. The rule: the best routable neighbour that is closer to
     * the destination geo than we are (measured at the DIRECT hop: a small
     * step forward is enough -- the rest is the next hop's problem). This
     * is the original tandem: the source does not need to see the whole
     * route, only a step in the right direction. */
    if (dgx != MC_GEO_UNKNOWN && n->geox != MC_GEO_UNKNOWN) {
        int32_t own_dst = geo_dist2(n->geox, n->geoy, dgx, dgy);
        for (int i = 0; i < MC_MAX_NEIGHBORS; i++) {
            mimeshi_neighbor *nb = &n->nbr[i];
            if (!nb->alive || !nb->sym || nb == excl) continue;
            if (!nb->quality_back) continue;   /* DATA direction must be measured (10.1) */
            if (nb->quality_back < MC_ROUTE_MIN_QUALITY) continue;   /* DATA direction (10.1) */
            if (nb->quality < MC_ACK_MIN_QUALITY) continue;  /* ack direction */
            if (nb->geox == MC_GEO_UNKNOWN) continue;
            int32_t d = geo_dist2(nb->geox, nb->geoy, dgx, dgy);
            if (d >= own_dst) continue;          /* must make progress toward the geo */
            int s = route_score(n, nb) - (int)(d >> 14);   /* prefer the closest hop, quality first */
            if (s > best_score) { best_score = s; best = nb; }
        }
    }
    return best;                              /* 0 = void or unknown (10.2/10.3) */
}

/* destination geo for dst: the 2-hop map first, then the direct table */
static void route_dst_geo(mimeshi_node *n, uint16_t dst, int16_t *gx, int16_t *gy) {
    mimeshi_2hop *h = known_find(n, dst);
    if (h && h->geox != MC_GEO_UNKNOWN) { *gx = h->geox; *gy = h->geoy; return; }
    mimeshi_neighbor *nb = nbr_find(n, dst);
    if (nb && nb->geox != MC_GEO_UNKNOWN) { *gx = nb->geox; *gy = nb->geoy; return; }
    *gx = MC_GEO_UNKNOWN; *gy = MC_GEO_UNKNOWN;
}

/* Ack1: immediately, on the channel of the ongoing data exchange --
 * the sender waits there for the answer (switch rule, 2). */
static void ack1_send(mimeshi_node *n, uint16_t msg_id, uint16_t src, uint8_t ch) {
    uint8_t p[7] = { PKT_ACK1, 0,0, 0,0, 0,0 };
    wr16(p + 1, msg_id); wr16(p + 3, src); wr16(p + 5, n->id);
    n->tx(n, ch, p, 7, t_ack_us());       /* same channel as the data (switch rule) */
    ka_called(n, ch);                     /* own call on the data channel (4.1) */
}

/* Ack2 to the previous hop, on ITS home channel (2, 7.1): the previous
 * hop listens there without its own window. Positive = the message has
 * come two hops (received, not necessarily forwarded, 7.2); negative =
 * forwarding failed, via F_NEG_ACK2. */
static void ack2_send(mimeshi_node *n, uint16_t msg_id, uint16_t src, uint16_t prev, bool neg) {
    uint8_t p[7] = { (uint8_t)(PKT_ACK2 | (neg ? F_NEG_ACK2 : 0)), 0,0, 0,0, 0,0 };
    wr16(p + 1, msg_id); wr16(p + 3, src); wr16(p + 5, n->id);
    bound_send(n, (uint8_t)(prev % MC_N_CHANNELS), p, 7, t_ack_us()); /* HOME of previous hop */
    ka_called(n, (uint8_t)(prev % MC_N_CHANNELS));  /* own call there (4.1) */
}

static void pending_try(mimeshi_node *n, mimeshi_pending *p);
static void record_fail(mimeshi_node *n, mimeshi_pending *p);
static void pending_retry_timer(mimeshi_node *n, void *ud);
static int send_data_internal(mimeshi_node *n, uint16_t dst, int16_t cgx, int16_t cgy,
                              const uint8_t *payload, uint16_t len, bool ack3, uint8_t no_report);

/* Failed delivery report (10.5): a routed message back to the source,
 * flagged so it never triggers a report of its own. Payload: msg-id,
 * reporter, cause. Routed like normal data with half the budget. */
enum { MC_FAIL_NOACK1 = 1, MC_FAIL_VOID = 2, MC_FAIL_NEGACK2 = 3 };
static void fail_report(mimeshi_node *n, uint16_t orig_msg, uint16_t orig_src) {
    if (orig_src == n->id) return;           /* we are the source: it stays here */
    uint8_t rpt[6];
    wr16(rpt, orig_msg); wr16(rpt + 2, n->id); rpt[4] = MC_FAIL_NOACK1; rpt[5] = 0;
    int rc = send_data_internal(n, orig_src, MC_GEO_UNKNOWN, MC_GEO_UNKNOWN, rpt, 5, false, 1);
    (void)rc;
    printf("[core] node %u: FAIL-RPT for msg %u back to %u (rc=%d)\n",
           n->id, orig_msg, orig_src, rc);
}

/* Ack1 timeout (7.3/7.4). Retry structure per hop, per the decided rule:
 *  - a hop tries its best next hop twice (2 attempts per candidate);
 *  - a candidate exhausted: recompute the best next hop WITHOUT the failed
 *    candidate, and so on, up to 6 attempts per hop (3 candidates x 2);
 *  - the recomputed candidate must not point back toward the source
 *    (prev_hop, 7.2): if it does, the hop sends a negative ack2 upstream
 *    and hands the message over (record_fail);
 *  - budget check BEFORE the attempt (9): the fail report names the
 *    real cause. */
static void ack1_timeout(mimeshi_node *n, void *ud) {
    mimeshi_pending *p = (mimeshi_pending *)ud;
    if (!p->active || p->state != 0) return;
    mimeshi_neighbor *nb = nbr_find(n, p->next_hop);
    if (nb && nb->penalty < 255) nb->penalty++;               /* 10.4 */
    p->attempts++;                                           /* the hop attempt cap (7.4) */
    p->tries_left--;                                        /* attempts within this candidate */
    if (p->tries_left > 0) {
        /* another attempt toward the SAME candidate (7.4), after a random
         * wait longer than the airtime of the next packet */
        p->budget_left--;
        if (p->budget_left == 0) { record_fail(n, p); return; }
        n->timer(n, (n->rand(n) % 500000ull) + 100000ull, pending_retry_timer, p);
        return;
    }
    /* candidate exhausted: recompute the best next hop without the failed one */
    mimeshi_neighbor *excl = nbr_find(n, p->next_hop);
    mimeshi_neighbor *back = nbr_find(n, p->prev_hop);
    int16_t dgx, dgy;
    if (p->dst_geox == MC_GEO_UNKNOWN) route_dst_geo(n, p->dst, &dgx, &dgy);
    else { dgx = p->dst_geox; dgy = p->dst_geoy; }
    mimeshi_neighbor *nx = route_next(n, p->dst, dgx, dgy, excl);
    if (nx == back) {
        /* the best remaining route points back toward the source: negative
         * ack2 upstream (7.4). The message hands over. */
        record_fail(n, p);
        return;
    }
    if (!nx && p->attempts >= MC_HOP_ATTEMPTS) {
        /* no alternative candidate AND the 6 attempts of this hop are used:
         * negative ack2 upstream (7.4). */
        record_fail(n, p);
        return;
    }
    if (!nx) {
        /* no alternative candidate, but attempts remain (7.4: 'and so on,
         * up to 6 attempts'): the failure may have been a one-off collision
         * -- retry the same candidate with fresh attempts. */
        p->tries_left = MC_RETRY_TX;
        p->budget_left--;
        if (p->budget_left == 0) { record_fail(n, p); return; }
        n->timer(n, (n->rand(n) % 500000ull) + 100000ull, pending_retry_timer, p);
        return;
    }
    p->next_hop = nx->id;
    p->tries_left = MC_RETRY_TX;              /* fresh candidate: two attempts */
    p->budget_left--;
    if (p->budget_left == 0) { record_fail(n, p); return; }
    pending_try(n, p);
}

/* The record has definitively failed (all neighbours/budget). */
static void record_fail(mimeshi_node *n, mimeshi_pending *p) {
    p->active = 0;
    printf("[core] node %u: FAIL msg %u toward %u\n", n->id, p->msg_id, p->next_hop);
    if (p->fwd) {
        ack2_send(n, p->msg_id, p->src, p->prev_hop, true);    /* negative ack2 (7.2) */
        if (!p->no_report) fail_report(n, p->msg_id, p->src);  /* 10.5 */
    } else if (!p->no_report) {
        fail_report(n, p->msg_id, p->src);
    }
}

/* Ack2 timeout at the sender (7.3): T_ack2 after the ack1, the record
 * ends -- positive ack2 has not come. The hop counts as succeeded (the
 * next hop DID receive it), the deeper chain is now the responsibility
 * of the next hop (7.2). */
static void ack2_timeout(mimeshi_node *n, void *ud) {
    mimeshi_pending *p = (mimeshi_pending *)ud;
    if (!p->active) return;
    p->active = 0;
    printf("[core] node %u: ack2 window expired for msg %u (hop done)\n", n->id, p->msg_id);
}

static void pending_retry_timer(mimeshi_node *n, void *ud) {
    pending_try(n, (mimeshi_pending *)ud);
}

/* One transmit attempt: update the budget in the header (offset 9,
 * 6.1/9), the exchange toward next_hop, and schedule the ack1 timeout.
 * The timeout is (7.3) S + t_data + t_ack + margin. */
static void pending_try(mimeshi_node *n, mimeshi_pending *p) {
    p->pkt[10] = p->budget_left;
    wr16(p->pkt + 7, n->id);            /* previous hop: we (6.1) */
    wr16(p->pkt + 8, p->next_hop);       /* next hop: only that node forwards/acks */
    bound_send(n, (uint8_t)(p->next_hop % MC_N_CHANNELS), p->pkt, p->pkt_len, t_data_us(p->pkt_len));
    ka_called(n, (uint8_t)(p->next_hop % MC_N_CHANNELS));   /* data as prev_hop (4.1) */
    n->timer(n, MC_S_SWITCH_US + t_data_us(p->pkt_len) + t_ack_us() + MC_ACK1_MARGIN_US, ack1_timeout, p);
}

/* Submit a unicast (own message). Return codes: see core.h. The route
 * is chosen per 10.2 at every hop, also at the source; the header can
 * carry the source geo (6.1) so intermediate hops can route greedily
 * even when the source is outside their map. */
static int send_data_internal(mimeshi_node *n, uint16_t dst, int16_t cgx, int16_t cgy,
                              const uint8_t *payload, uint16_t len, bool ack3, uint8_t no_report) {
    if (len > MC_MAX_PAYLOAD) return -3;
    int16_t dgx, dgy;
    route_dst_geo(n, dst, &dgx, &dgy);
    if (dgx == MC_GEO_UNKNOWN && cgx != MC_GEO_UNKNOWN) {
        dgx = cgx; dgy = cgy;       /* client geo (11): the tandem has its starting point */ 
    }
    if (dgx == MC_GEO_UNKNOWN) return -1;  /* no geo from map or client: nothing to
                                              fall back on (10.2); do not flood */
    mimeshi_neighbor *nx = route_next(n, dst, dgx, dgy, 0);
    if (!nx) {
        /* geo known but void (10.3): refused here, waypoint/perimeter follow later */
        return -1;
    }
    mimeshi_pending *p = 0;
    for (int i = 0; i < MC_MAX_PENDING; i++) if (!n->pend[i].active) { p = &n->pend[i]; break; }
    if (!p) return -2;
    memset(p, 0, sizeof(*p));
    p->active = 1; p->state = 0; p->tries_left = MC_RETRY_TX; p->attempts = 0;
    p->budget_left = MC_BUDGET_INIT;
    p->msg_id = (uint16_t)n->rand(n);
    p->src = n->id; p->dst = dst; p->prev_hop = n->id; p->next_hop = nx->id;
    p->no_report = no_report;
    p->dst_geox = dgx; p->dst_geoy = dgy;
    uint8_t *k = p->pkt;
    k[0] = (uint8_t)(PKT_DATA | (ack3 ? F_ACK3_REQ : 0)
                     | (no_report ? F_NO_REPORT : 0));
    wr16(k + 1, p->msg_id); wr16(k + 3, n->id); wr16(k + 5, dst); wr16(k + 7, n->id);
    k[10] = p->budget_left;                /* header: budget at offset 10 */
    wr16(k + 8, nx->id);                   /* next hop: only that node forwards/acks */
    wr16(k + 11, (uint16_t)dgx); wr16(k + 13, (uint16_t)dgy);  /* dst geo: always (6.1) */
    memcpy(k + 15, payload, len);
    p->pkt_len = (uint16_t)(15 + len);
    pending_try(n, p);
    return 0;
}

int mimeshi_send_data(mimeshi_node *n, uint16_t dst, const uint8_t *payload, uint16_t len, bool ack3) {
    return send_data_internal(n, dst, MC_GEO_UNKNOWN, MC_GEO_UNKNOWN, payload, len, ack3, 0);
}

int mimeshi_send_data_geo(mimeshi_node *n, uint16_t dst, int16_t geox, int16_t geoy,
                          const uint8_t *payload, uint16_t len, bool ack3) {
    return send_data_internal(n, dst, geox, geoy, payload, len, ack3, 0);
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
            nbr_set_quality(n, nb, rssi);
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
            nbr_set_quality(n, nb, rssi);
            nb->proven_age_us = 0;      /* fresh reception of an identifying transmission */
            if (dst == n->id) { nb->sym = 1; nb->ack_try = 0; }   /* confirmed symmetric (4.2) */
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
            nbr_set_quality(n, nb, rssi);
        }
        /* appearing in the list = reverse evidence refreshed (4.2) */
        for (uint16_t i = 0; i < ids; i++) {
            if (rd16(pkt + 3 + 2 * i) == n->id && nb) {
                nb->sym = 1; nb->ack_try = 0; nb->proven_age_us = 0;
            }
        }
        break;
    }
    case PKT_ACK1: {
        /* our own outstanding transmission has arrived at the next hop
         * (7.1). The record stays active in state 1: the ack2 window
         * (7.3) now runs -- a positive ack2 closes the hop, the window
         * expiring also closes it (the next hop did receive it, 7.2).
         *
         * The sender of this ack1 received our message AND its ack
         * reached us: the link is hereby proven symmetric (4.2, 10.1).
         * Success decays the penalty (10.4). */
        uint16_t msg_id = rd16(pkt + 1);
        for (int i = 0; i < MC_MAX_PENDING; i++) {
            mimeshi_pending *p = &n->pend[i];
            if (p->active && p->state == 0 && p->msg_id == msg_id) {
                p->state = 1;                          /* ack1 done: ack2 window (7.3) */
                n->timer(n, MC_T_ACK2_US, ack2_timeout, p);
                mimeshi_neighbor *nb = nbr_find(n, rd16(pkt + 5));
                if (nb) {
                    nb->sym = 1; nb->ack_try = 0; nb->proven_age_us = 0;
                    if (nb->penalty > 0) nb->penalty--;               /* 10.4 */
                }
                printf("[core] node %u: ACK1 for msg %u (ack2 window up)\n", n->id, msg_id);
                return;
            }
        }
        break;
    }
    case PKT_ACK2: {
        /* reply of the next hop toward the previous hop (7.2).
         *  - Positive: the message has come two hops; the record at the
         *    sender closes (7.1).
         *  - Negative: the next hop could not forward. The sender
         *    immediately tries the next neighbour in the series,
         *    without waiting (7.4). */
        uint16_t msg_id = rd16(pkt + 1);
        bool neg = (flags & F_NEG_ACK2) != 0;
        printf("[core] node %u: ACK2 %s for msg %u\n", n->id,
               neg ? "NEGATIVE" : "positive", msg_id);
        for (int i = 0; i < MC_MAX_PENDING; i++) {
            mimeshi_pending *p = &n->pend[i];
            if (p->active && p->state == 1 && p->msg_id == msg_id) {
                p->active = 0;                          /* positive: hop done */
                if (!neg) return;
                /* negative ack2: the failed sub-route is accounted HERE
                 * (7.4/9: lump 6, the downstream's spend died with its
                 * record). Then ONE next-best attempt. If that points back
                 * toward the source too, the message has failed. */
                if (p->budget_left > MC_ACK2_CHARGE) p->budget_left -= MC_ACK2_CHARGE;
                else p->budget_left = 0;
                if (p->budget_left == 0) { record_fail(n, p); return; }
                p->active = 1; p->state = 0;
                p->tries_left = MC_RETRY_TX;            /* the one next-best candidate: 2 attempts */
                mimeshi_neighbor *excl = nbr_find(n, p->next_hop);
                mimeshi_neighbor *back = nbr_find(n, p->prev_hop);
                int16_t dgx, dgy;
                if (p->dst_geox == MC_GEO_UNKNOWN) route_dst_geo(n, p->dst, &dgx, &dgy);
                else { dgx = p->dst_geox; dgy = p->dst_geoy; }
                mimeshi_neighbor *nx = route_next(n, p->dst, dgx, dgy, excl);
                if (!nx || nx == back) {
                    /* the next best hop points back toward the source:
                     * the message has failed (7.4). */
                    record_fail(n, p);
                    return;
                }
                p->next_hop = nx->id;
                p->budget_left--;
                if (p->budget_left == 0) { record_fail(n, p); return; }
                pending_try(n, p);
                return;
            }
        }
        break;
    }
    case PKT_ACK3: {
        /* the final destination has delivered the message to its client
         * (11): routed back to the source. Until clients exist, the
         * ack3 is visibility only. */
        printf("[core] node %u: ACK3 - delivered at destination\n", n->id);
        break;
    }
    case PKT_TABLE_CHUNK: {
        chunk_recv(n, ch, rssi, pkt, len);
        break;
    }
    case PKT_DATA: {
        uint16_t msg_id = rd16(pkt + 1), src = rd16(pkt + 3), dst = rd16(pkt + 5), prev = rd16(pkt + 7);
        uint16_t nhop = rd16(pkt + 8);
        /* dedup before the handling: a retry of the same hop must not
         * deliver or forward twice. But the ack1 is repeated: the first
         * ack may have been lost (a 7 B ack survives a weak direction
         * where the data did not, 7.4 -- asymmetric links are usable in
         * the good direction), and without a re-ack the sender exhausts
         * its attempts on a delivered message. */
        if (!dedup_fresh(n, msg_id, src)) {
            if (nhop == n->id) {
                ack1_send(n, msg_id, src, ch);   /* idempotent re-ack (7.4) */
            }
            return;
        }
        /* snooping (2): the sender of the data is a direct neighbour
         * candidate -- quality and liveness, never symmetry. */
        mimeshi_neighbor *snb = nbr_find(n, prev);
        if (snb) { nbr_set_quality(n, snb, rssi); snb->last_seen = 0; }
        if (nhop != n->id) {
            /* not addressed to us: the exchange runs between prev and
             * nhop. Overhearing yields liveness/quality only (2);
             * forwarding would duplicate the message at every listener. */
            break;
        }
        if (dst == n->id) {
            /* final destination: ack1 on the exchange channel, ack2
             * positive to the previous hop (7.1), optionally ack3 back
             * to the source (11). */
            printf("[core] node %u: DATA from %u: '%.*s'\n", n->id, src, (int)(len - 15), (const char *)(pkt + 15));
            ack1_send(n, msg_id, src, ch);             /* same channel */
            ack2_send(n, msg_id, src, prev, false);   /* positive: home channel of previous hop */
            if (flags & F_ACK3_REQ) {
                /* ack3 as a routed message (6.2): DATA format, payload
                 * = the original msg-id, routed back to the source. */
                uint8_t a[15] = { PKT_ACK3 };
                wr16(a + 1, (uint16_t)n->rand(n)); wr16(a + 3, n->id);
                wr16(a + 5, src); wr16(a + 7, n->id);
                a[10] = MC_BUDGET_INIT; wr16(a + 11, msg_id);
                int rc2 = send_data_internal(n, src, MC_GEO_UNKNOWN, MC_GEO_UNKNOWN, a + 11, 4, false, 1);
                if (rc2 != 0)
                    printf("[core] node %u: ack3 for msg %u not submittable (rc=%d)\n", n->id, msg_id, rc2);
            }
            break;
        }
        /* forwarding (7.1): we are an intermediate hop. ack1 goes back
         * on the exchange channel, then the message is routed further
         * per 10.2 -- the dst geo travels in the header (6.1), so every
         * hop can greedy-route even without its own map entry. The
         * own map wins when it knows the destination: fresher, with
         * quality (10.2 tandem). */
        ack1_send(n, msg_id, src, ch);
        int16_t dgx = (int16_t)rd16(pkt + 11), dgy = (int16_t)rd16(pkt + 13);
        int16_t mgx, mgy;
        route_dst_geo(n, dst, &mgx, &mgy);
        if (mgx != MC_GEO_UNKNOWN) { dgx = mgx; dgy = mgy; }  /* own map beats the header when it knows dst */
        mimeshi_neighbor *skip = nbr_find(n, prev);   /* never bounce back (7.2) */
        mimeshi_neighbor *nx = route_next(n, dst, dgx, dgy, skip);
        if (!nx) {
            /* no route from here (void or unknown, 10.2/10.3): negative
             * ack2 immediately; the previous hop re-routes. */
            ack2_send(n, msg_id, src, prev, true);
            printf("[core] node %u: no route for msg %u toward %u: negative ack2\n", n->id, msg_id, dst);
            break;
        }
        mimeshi_pending *p = 0;
        for (int i = 0; i < MC_MAX_PENDING; i++) if (!n->pend[i].active) { p = &n->pend[i]; break; }
        if (!p) {
            ack2_send(n, msg_id, src, prev, true);      /* no slot: refuse the hop */
            break;
        }
        memset(p, 0, sizeof(*p));
        p->active = 1; p->state = 0; p->fwd = 1;
        p->no_report = (flags & F_NO_REPORT) ? 1 : 0;   /* reports never report (10.5) */
        p->tries_left = MC_RETRY_TX; p->attempts = 0;
        p->budget_left = pkt[10];                      /* budget from the header (10): one shared
                                                            budget for the whole path (7.4, 9); the
                                                            fail reports have their own half budget (10.5) */
        p->msg_id = msg_id; p->src = src; p->dst = dst;
        p->prev_hop = prev; p->next_hop = nx->id;
        p->dst_geox = dgx; p->dst_geoy = dgy;
        memcpy(p->pkt, pkt, len);                      /* the message itself is not stored, only the
                                                          header copy for the next hop (7.2) */
        p->pkt_len = len;
        printf("[core] node %u: FWD msg %u from %u via %u toward %u\n",
               n->id, msg_id, src, nx->id, dst);
        pending_try(n, p);
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
                mimeshi_switch_fn sw, mimeshi_rand_fn rnd, mimeshi_busy_fn busy, void *user) {
    memset(n, 0, sizeof(*n));
    n->busy = busy;                        /* CAD (2); NULL = no check */
    n->id = id; n->home_ch = (uint8_t)(id % MC_N_CHANNELS);
    n->local_tbl_ctr = 1; n->hello_interval_us = MC_HELLO_START_US;
    n->chunk_rr_ch = n->home_ch;               /* tail rotation starts at home */
    n->hello_jitter_us = MC_HELLO_JITTER_US;
    n->tx = tx; n->timer = timer; n->switch_ch = sw; n->rand = rnd; n->user = user;
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

void mimeshi_start(mimeshi_node *n) {
    hello_round(n);
    proof_stagger(n);
    /* random phase: prevents synchronous rounds on simultaneous startup (4.1) */
    n->timer(n, n->hello_interval_us + hello_jitter(n), hello_tick, 0);
    /* proof tick: time base for keepalive + symmetry flush (4.1, 4.2) */
    n->timer(n, MC_KEEPALIVE_TICK_US, proof_tick, 0);
    /* table chunks: push the own table on the home channel (4.3) */
    n->timer(n, MC_CHUNK_PERIOD_US / 2 + (n->rand(n) % 1000000ull), chunk_tick, 0);
}
