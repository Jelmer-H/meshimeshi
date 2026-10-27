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
/* ---------------- attempt schedule (7.4) -------------------------
 * One schedule per hop, no rebuild. Waits: a small fixed gap inside a
 * pass over the candidates (the next attempt goes to a DIFFERENT
 * node), an escalating gap at each pass boundary (the same node comes
 * back there - the damper belongs there). +-25% jitter on every gap:
 * two nodes that collided once must not retry in lock-step. With 1
 * candidate every attempt is a boundary; the pass table then runs
 * 0.8, 1.2, 1.8 s. Worst case total (6 attempts, 3 candidates):
 * ~6 s incl. attempt times - inside T_ack2 = 8 s (7.3). */
#define PLAN_INTRA_MS       400u           /* A->B within a pass (different node) */
static const uint32_t PLAN_PASS_MS[3] = { 800, 1200, 1800 };  /* escalating per pass (same node) */
#define MC_ACK1_MARGIN_US   80000ull       /* processing + margin on top of t_data + t_ack (7.3) */
#define MC_T_ACK2_US        8000000ull    /* ack2 timeout (7.3, revised Oct 2026): 8 s - the attempt
                                   schedule of the next hop (7.4) completes in ~6 s worst case,
                                   so silence beyond 8 s means its forwarding truly stalled;
                                   measured against 15 s: fired 17x in 6h, all legitimate */
#define MC_ACK2_BUDGET_PENALTY 6u           /* budget units charged on the T_ack2 timeout (7.3) */
#define MC_S_SWITCH_US      5000ull        /* channel switch incl. settling (14.1) */
#define MC_SYM_QUALITY_MIN  13u            /* symmetry requires this quality (10.1):
                                               ~-102 dBm (decided Oct 2026, measured:
                                               at 14 the scenario kept 2 of 40 nodes
                                               without any routable link and lost ~10%
                                               of the unicasts to refusals; at 13
                                               every node has at least one confirmed
                                               neighbour and delivery reaches 98.9%.
                                               The retry series (7.4) absorb the
                                               weaker links' higher loss rate) */
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
 *   DATA      12+n B  type+F_ACK3_REQ+F_DEST_GEO, msg-id(2), source(2),
 *                     destination(2), previous-hop(2), budget(1, offset 9),
 *                     next-hop(2, offset 10), [dest geo 4 B when F_DEST_GEO,
 *                     offset 12 (5, 10.2)], payload(n)  (6.1)
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
#define F_DEST_GEO 0x20   /* destination geo present: 4 B after the fixed
                             header (6.1, 10.2) - every hop can recompute
                             the bridge node toward an unknown destination */

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

/* Hysteresis on the advertised link byte of the own star (4.3): each
 * direction nibble moves only when the fresh class is >= 2 steps from
 * the last published one - sub-threshold EWMA movement leaves the
 * advertised form untouched, so the content hash of the own star
 * stays stable while measurements drift. First publication takes the
 * fresh value. */
static uint8_t adv_byte_hyst(uint8_t published, uint8_t fresh) {
    uint8_t ph = (uint8_t)((published >> 4) & 0x07), pl = published & 0x07;
    uint8_t fh = (uint8_t)((fresh >> 4) & 0x07), fl = fresh & 0x07;
    if (ph) { int d = (int)fh - (int)ph; if (d < -1 || d > 1) ph = fh; }
    else ph = fh;
    if (pl) { int d = (int)fl - (int)pl; if (d < -1 || d > 1) pl = fl; }
    else pl = fl;
    return (uint8_t)((ph << 4) | pl);
}

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
 * hardening. Full cache = FIFO replacement (decided Oct 2026, measured
 * under load): the oldest entry is evicted and the fresh one takes its
 * place. "Receive anyway" after 128 messages turned dedup off
 * completely, and a routing ping-pong forwarded one message 166 times
 * (each FORWARD is a fresh pending, and the ack1 rule of 7.4 kept the
 * loop fed). The recent 128 (msg-id, source) pairs always block. */
static bool dedup_fresh(mimeshi_node *n, uint16_t msg_id, uint16_t src) {
    for (int i = 0; i < MC_MAX_DEDUP; i++)
        if (n->dedup[i].msg_id == msg_id && n->dedup[i].src == src) return false;
    int victim = 0;
    for (int i = 0; i < MC_MAX_DEDUP; i++) {
        if (n->dedup[i].msg_id == 0) { victim = i; goto insert; }
        if (n->dedup[i].seq < n->dedup[victim].seq) victim = i;
    }
insert:
    n->dedup[victim].msg_id = msg_id; n->dedup[victim].src = src;
    n->dedup_seq = (uint16_t)(n->dedup_seq + 1);
    n->dedup[victim].seq = n->dedup_seq;
    return true;
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
typedef struct { uint8_t ch; uint8_t cad_tries; uint16_t len; uint64_t airtime; uint8_t pkt[16 + MC_MAX_PAYLOAD]; } bound_ctx;
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

/* Effective age of a map entry in full proof ticks (~56 s each),
 * uncapped for internal comparisons (4.4); on air the age byte caps at
 * 255 (~4 h) - anything older advertises simply as expired. */
static uint32_t map_age_ticks(mimeshi_node *n, mimeshi_mapnode *m) {
    return (uint32_t)m->age_ticks + (n->tick - m->arrived_tick);
}


/* Eviction of a map entry (4.4): the entry goes, and with it every
 * stored link that touches it - the link store stays consistent with
 * the map, and a link is only usable while both endpoints stand. */
static void map_evict(mimeshi_node *n, mimeshi_mapnode *m) {
    for (int i = 0; i < MC_MAX_LINKS; i++)
        if (n->lnk[i].alive && (n->lnk[i].a == m->id || n->lnk[i].b == m->id))
            n->lnk[i].alive = 0;
    m->alive = 0;
}

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
    /* pool full (4.4): recycle the LEAST FRESH entry - never the
     * newest; the freshest knowledge is the routing capital. */
    int victim = -1; uint32_t worst = 0;
    for (int i = 0; i < MC_MAP_NODES; i++) {
        if (!n->map[i].alive) continue;
        /* direct neighbours never: they live in the prefix */
        if (nbr_find(n, n->map[i].id)) continue;
        uint32_t a = map_age_ticks(n, &n->map[i]);
        if (a > worst) { worst = a; victim = i; }
    }
    if (victim < 0) {                            /* only direct neighbours in the map */
        printf("[core] node %u: map full, entry %u dropped\n", n->id, id);
        return 0;
    }
    map_evict(n, &n->map[victim]);
    n->map[victim].alive = 1; n->map[victim].id = id;
    n->map[victim].geo_lat = 0; n->map[victim].geo_lon = 0;
    n->map[victim].quality = 0; n->map[victim].age_ticks = 255;
    n->map[victim].arrived_tick = n->tick;
    return &n->map[victim];
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

/* Content stars per round packet (4.3): 8 x (9 B + links) fits 230 B. */
#define MC_CHUNK_MAX_ENTRIES 8

/* Table mutation feed (4.3): the local table counter marks the own
 * star as changed; the next round carries it as content. Nothing is
 * scheduled here - the round timer is the only send path (4.3 hard
 * bound). */
static void tbl_mark_dirty(mimeshi_node *n) {
    n->tbl_dirty = 1;
}

/* Neighbour addition (4.3): the new neighbour is a content change of
 * the own star - the next round carries it. */
/* Neighbour confirmation (4.3): the own star gains the link - and the
 * newcomer acceleration (4.3) turns the queue for it: the forward
 * state of all prefix stars resets, so the stars of the direct
 * neighbours re-enter the content queue and travel with the coming
 * rounds. The old cadence answered a newcomer with a delta plus a
 * full prefix; the star-round translation is the queue reset - zero
 * extra bytes, the same stars that round anyway, only prioritised. */
static void tbl_nbr_added(mimeshi_node *n, uint16_t id) {
    (void)id;
    tbl_mark_dirty(n);
    for (int i = 0; i < MC_MAP_NODES; i++) {
        mimeshi_mapnode *m = &n->map[i];
        if (!m->alive || m->id == n->id) continue;
        if (!nbr_find(n, m->id)) continue;         /* prefix stars only */
        m->star_fwd_tick = 0;                      /* as never carried */
        m->star_fwd_hash = 0;
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

/* ---- star canonicalisation (4.3) --------------------------------------- */
/* The star is the unit of sharing in the star-round concept: one node
 * entry plus the links it OWNS. Canonical form (4.3): links sorted by
 * neighbour id, so the star has a deterministic byte representation
 * independent of the internal table order - the basis for content
 * digests and the derivable heartbeat match. The entry layout follows
 * the chunk entry of 4.3 exactly (id, quality byte, age, geo), so a
 * star serialises to bytes a chunk receiver already understands. */
#define MC_STAR_MAX_LINKS 64
#define MC_STAR_MAX_BYTES (9 + 3 * MC_STAR_MAX_LINKS)

/* Link ownership (4.3): one bit of a CRC-8 over the normalised pair.
 * The owner of {A,B}: normalise a=min, b=max, hash the 4 bytes, bit 0
 * decides - set: a owns, else b. Both endpoints (and every receiver,
 * whatever the hop distance) compute the same owner from the pair
 * alone; the normalisation makes the rule symmetric by construction:
 * both sides hash the very same bytes, so no AB-vs-BA comparison and
 * no tie-break can arise. Measured fair split (~50/50), so each node
 * owns about half its links and star sizes even out across a
 * neighbourhood. Ownership is permanent (IDs and the hash are fixed):
 * a link's advertised form stays bound to exactly one star. */
static uint8_t lnk_pair_bit(uint16_t x, uint16_t y) {
    uint16_t a = x < y ? x : y, b = x < y ? y : x;
    uint8_t bytes[4];
    wr16(bytes, a); wr16(bytes + 2, b);
    uint8_t crc = 0;
    for (int i = 0; i < 4; i++) {
        crc ^= bytes[i];
        for (int k = 0; k < 8; k++)
            crc = (uint8_t)((crc & 0x80) ? (crc << 1) ^ 0x07 : crc << 1);
    }
    return (uint8_t)(crc & 1);
}
static uint16_t lnk_owner(uint16_t x, uint16_t y) {
    uint16_t a = x < y ? x : y, b = x < y ? y : x;
    return lnk_pair_bit(x, y) ? a : b;
}

/* Serialise the star of node `center` as known by n - storage as
 * received (4.3): the entry (id, both quality classes, tick, geo) and
 * every link OWNED by center, reproduced exactly as the origin
 * advertised them. For the OWN star (center == n->id) the classes of
 * the direct-neighbour links come from the neighbor table (the own
 * measurement and the peer's reported reverse quality, 4.2); for a
 * foreign star the entry class byte is adv_q as received and the
 * link bytes come from the link store as received - the form the
 * origin put on air, so the content hash is a property of the
 * origin, not of the observer (the 6a finding, resolved in design).
 * age_out receives the age byte position in buf (the heartbeat tick
 * lives there, 4.3); the age byte itself is written but excluded from
 * the content hash (mimeshi_star_content_hash). Returns the total
 * length, or 0 if the star does not exist or does not fit maxlen. */
uint16_t mimeshi_star_serialize(const mimeshi_node *n, uint16_t center,
                               uint8_t *buf, uint16_t maxlen,
                               uint16_t *age_off_out) {
    mimeshi_mapnode *m = map_find((mimeshi_node *)n, center);
    if (!m) return 0;
    if (maxlen < 9) return 0;

    wr16(buf, center);
    if (center == n->id) {
        /* the OWN entry: the origin publishes itself. The entry byte
         * is the own link picture toward the strongest confirmed
         * neighbour (own measured class | the class that neighbour
         * reported about us, 4.2) - a stable, self-computed value.
         * Self-publication (4.3 storage-as-received): the form is
         * stored in the own adv_q, so the own catalogue digest is
         * computed over exactly the bytes the neighbours hold. */
        uint8_t q = 0;
        int best = -1;
        for (int i = 0; i < MC_MAX_NEIGHBORS; i++) {
            const mimeshi_neighbor *nb = &n->nbr[i];
            if (!nb->alive || !nb->sym) continue;
            if (best < 0 || nb->quality > n->nbr[best].quality) best = i;
        }
        if (best >= 0) {
            const mimeshi_neighbor *nb = &n->nbr[best];
            q = (uint8_t)((adv_class_of(nb->quality) << 4)
                          | adv_class_of(nb->rev_quality));
        }
        m->adv_q = adv_byte_hyst(m->adv_q, q);   /* hysteresis (4.3) */
        buf[2] = m->adv_q;
    } else if (m->adv_q) {
        buf[2] = m->adv_q;              /* entry classes exactly as the
                                           origin advertised them (4.3) */
    } else {
        /* never received the star itself, only map material: the local
           measurement of the node's signal as the known class */
        buf[2] = (uint8_t)((adv_class_of(m->quality) << 4) | 0);
    }
    buf[3] = m->star_tick;              /* heartbeat tick in the age byte (4.3):
                                           the origin's liveness, not the
                                           travelled age - travelled age is
                                           local (4.4) and never travels in
                                           the canonical star form */
    wr16(buf + 4, m->geo_lat);
    wr16(buf + 6, m->geo_lon);
    if (age_off_out) *age_off_out = 3;

    /* links of the star, sorted by peer id (canonical form, 4.3).
     * OWNED share only (4.3): the link {center, peer} travels in
     * exactly one star - the owner's. The owner bit is derived from
     * the pair itself, so every observer packs the same share. */
    uint16_t ids[MC_STAR_MAX_LINKS];
    uint8_t qc[MC_STAR_MAX_LINKS];
    uint16_t cnt = 0;
    if (center == n->id) {
        /* the OWN star: the origin's own links come from the NEIGHBOR
         * TABLE, not the link store (4.3) - the origin is the source
         * of its own star, and at startup the store is still empty
         * (bootstrap: no star received yet, no link stored yet). Every
         * confirmed symmetric neighbour with the ownership bit on our
         * side travels, with the own measured class and the peer's
         * reported reverse class (4.2), published under hysteresis
         * (4.3): a >= 2-step class move re-publishes, sub-threshold
         * EWMA movement does not. */
        for (int i = 0; i < MC_MAX_NEIGHBORS && cnt < MC_STAR_MAX_LINKS; i++) {
            mimeshi_neighbor *pnb = &((mimeshi_node *)n)->nbr[i];
            if (!pnb->alive || !pnb->sym) continue;
            if (lnk_owner(n->id, pnb->id) != n->id) continue;   /* owned share */
            uint8_t fresh = (uint8_t)((adv_class_of(pnb->quality) << 4)
                                      | adv_class_of(pnb->rev_quality));
            pnb->adv_class = adv_byte_hyst(pnb->adv_class, fresh);
            ids[cnt] = pnb->id;
            qc[cnt] = pnb->adv_class;
            cnt++;
        }
        for (uint16_t i = 1; i < cnt; i++) {         /* insertion sort by peer id */
            uint16_t id = ids[i]; uint8_t q = qc[i];
            uint16_t j = i;
            while (j > 0 && ids[j-1] > id) { ids[j] = ids[j-1]; qc[j] = qc[j-1]; j--; }
            ids[j] = id; qc[j] = q;
        }
        if (9 + 3 * cnt > maxlen) return 0;          /* does not fit */
        buf[8] = (uint8_t)cnt;                       /* link count, chunk entry layout (4.3) */
        for (uint16_t i = 0; i < cnt; i++) {
            wr16(buf + 9 + 3 * i, ids[i]);
            buf[9 + 3 * i + 2] = qc[i];
        }
        return (uint16_t)(9 + 3 * cnt);
    }
    for (int i = 0; i < MC_MAX_LINKS && cnt < MC_STAR_MAX_LINKS; i++) {
        if (!n->lnk[i].alive) continue;
        uint16_t peer = 0;
        if (n->lnk[i].a == center)      peer = n->lnk[i].b;
        else if (n->lnk[i].b == center) peer = n->lnk[i].a;
        else continue;
        if (lnk_owner(center, peer) != center) continue;   /* owned share (4.3) */
        if (map_find((mimeshi_node *)n, peer) == 0) continue;  /* link endpoints are map nodes */
        uint8_t cf = lnk_class((mimeshi_node *)n, center, peer);
        if (cf == 0xFF) continue;
        uint8_t cr = lnk_class((mimeshi_node *)n, peer, center);
        ids[cnt] = peer;
        qc[cnt] = (uint8_t)((cf << 4) | (cr == 0xFF ? 0 : cr));
        cnt++;
    }
    for (uint16_t i = 1; i < cnt; i++) {         /* insertion sort by peer id */
        uint16_t id = ids[i]; uint8_t q = qc[i];
        uint16_t j = i;
        while (j > 0 && ids[j-1] > id) { ids[j] = ids[j-1]; qc[j] = qc[j-1]; j--; }
        ids[j] = id; qc[j] = q;
    }
    if (9 + 3 * cnt > maxlen) return 0;          /* does not fit */
    buf[8] = (uint8_t)cnt;                       /* link count, chunk entry layout (4.3) */
    for (uint16_t i = 0; i < cnt; i++) {
        wr16(buf + 9 + 3 * i, ids[i]);
        buf[9 + 3 * i + 2] = qc[i];
    }
    return (uint16_t)(9 + 3 * cnt);
}

/* 1-byte content hash over the canonical star bytes EXCLUDING the age
 * byte (4.5): the digest covers what is advertised (identity, links,
 * quality classes), not how old the advertisement is - silent EWMA
 * movement under the hysteresis does not change it, and the heartbeat
 * tick (added separately in the digest) can change freely. CRC-8/ATM
 * (poly 0x07, init 0): cheap, fixed table-free, good avalanche for the
 * small fixed-size input. */
uint8_t mimeshi_star_content_hash(const uint8_t *star, uint16_t len,
                                  uint16_t age_off) {
    uint8_t crc = 0;
    for (uint16_t i = 0; i < len; i++) {
        if (i == age_off) continue;              /* age excluded (4.5) */
        crc ^= star[i];
        for (int b = 0; b < 8; b++)
            crc = (uint8_t)((crc & 0x80) ? (crc << 1) ^ 0x07 : crc << 1);
    }
    return crc;
}

/* Digest of a star as carried in the catalogue (4.5):
 * digest = (content_hash + tick) mod 256 - derivable: a receiver that
 * has the star can compute the expected digest for tick + k for small
 * k, which is the heartbeat match. */
uint8_t mimeshi_star_digest(uint8_t content_hash, uint8_t tick) {
    return (uint8_t)(content_hash + tick);
}

/* Heartbeat match (4.5): does `received` fit content_hash at
 * stored_tick + k for some k in 0..k_max? Returns the matched tick
 * advance (1..k_max), 0 for an exact same-tick match, or -1 when no
 * k fits (real content change, or a star we do not have). */
/* ---- star round (4.5, proposal): the uniform convergence cadence --- */
/* One chunk per interval (starting value 15 min = 16 proof ticks of
 * ~56 s), all three channels like the re-share round. Content first
 * (stars that changed since they were last forwarded, nearest hops
 * first, the own star at the front), catalogue in the remaining space
 * ((node id, digest) at 3 B per star). Hard bound: never more than
 * one chunk per node per interval, whatever happens (4.5). The own
 * star travels in full at least once per hour regardless of its
 * queue position (4.5): the guaranteed own-star round. */
#define MC_STAR_ROUND_TICKS   16u    /* 15 min interval in proof ticks */
#define MC_STAR_ROUNDS_PER_H  4u     /* 60 min / 15 min: own-star bound */
#define MC_STAR_CAT_RESERVE   30u     /* catalogue bytes reserved per round (4.5):
                                        content may fill the chunk, but the last
                                        30 B are for catalogue entries - the age
                                        maintenance that keeps the resting map
                                        alive must not be starved by a content
                                        backlog; 4 rounds/h x 30 B = 40 entries/h
                                        minimum on air */
#define PKT_STAR_ROUND        10     /* new packet type (4.5) */
#define F_STAR_CATALOGUE 0x10       /* chunk contains catalogue entries */
#define F_STAR_MISSED   0x20         /* chunk contains missed-star requests (4.5) */

/* Star age byte (4.5 heartbeat): the tick counter of the origin.
 * The round is also the tick driver of the own star: at the
 * guaranteed own-star round the tick advances and rides in the age
 * byte; neighbours derive the new digest without content. */
static uint8_t star_own_tick(mimeshi_node *n) { return n->star_tick; }

static int r_idx(mimeshi_node *n, uint16_t id);

/* Star-forward usefulness (4.5, graph check): does forwarding the star
 * of `center` reach any neighbour that lies strictly FARTHER from the
 * center than we do? Hop distances over the known graph (map + link
 * store + our own edges), the same material routing uses - but pure
 * hop counting, no ETX: the question is reach, not cost.
 *
 * A BFS from the center over the routing-reachable edges gives the
 * distance of every known node. Our own distance: the minimum over
 * our neighbours (we are the virtual start edge, as in route_next_hop).
 * If no known neighbour lies strictly farther, the forwarding is an
 * echo back into the gradient: everyone we could reach got the star
 * from the same side we did. Unknown neighbours (no map slot, no
 * edges) count as FARTHER - absence of knowledge never silences the
 * forward (never prune on missing information, 4.5).
 *
 * Return 1 = useful (forward), 0 = useless (skip). */
static uint8_t star_dist[MC_MAP_NODES];     /* BFS distances, 0xFF = unreached */
static uint8_t star_fwd_useful(mimeshi_node *n, uint16_t center) {
    int ci = r_idx(n, center);
    if (ci < 0) return 1;                     /* center unknown to us: forward */
    for (int i = 0; i < MC_MAP_NODES; i++) star_dist[i] = 0xFF;

    /* BFS from the center. Edges: the symmetric threshold links of the
     * link store (both directions usable when the class is known) -
     * hop reach, not route cost, so undirected relaxation over both
     * nibbles with a non-zero class. */
    static uint16_t qbuf[MC_MAP_NODES];       /* BFS queue of map indices */
    uint16_t qh = 0, qt = 0;
    star_dist[ci] = 0;
    qbuf[qt++] = (uint16_t)ci;
    while (qh < qt) {
        uint16_t u = qbuf[qh++];
        if (star_dist[u] >= 0xFE) continue;
        uint16_t uid = n->map[u].id;
        for (int j = 0; j < MC_MAX_LINKS; j++) {
            if (!n->lnk[j].alive) continue;
            uint16_t o;
            if (n->lnk[j].a == uid) o = n->lnk[j].b;
            else if (n->lnk[j].b == uid) o = n->lnk[j].a;
            else continue;
            /* either direction known with a class = a hop for reach */
            uint8_t cf = (uint8_t)((n->lnk[j].q >> 4) & 0x0F);
            uint8_t cr = (uint8_t)(n->lnk[j].q & 0x0F);
            if (!cf && !cr) continue;         /* no measured direction: no hop */
            int oi = r_idx(n, o);
            if (oi < 0 || star_dist[oi] != 0xFF) continue;
            star_dist[oi] = (uint8_t)(star_dist[u] + 1);
            qbuf[qt++] = (uint16_t)oi;
        }
    }

    /* our distance: the best hop over our own edges (virtual start);
     * plus the trivial case: the center IS a direct neighbour (dist 1) */
    int my_dist = 0x7FFFFFFF;
    if (nbr_find(n, center) && nbr_find(n, center)->sym) my_dist = 1;
    for (int i = 0; i < MC_MAX_NEIGHBORS; i++) {
        mimeshi_neighbor *nb = &n->nbr[i];
        if (!nb->alive || !nb->sym) continue;
        int bi = r_idx(n, nb->id);
        if (bi < 0) continue;
        if (star_dist[bi] == 0xFF) continue;
        int d = star_dist[bi] + 1;
        if (d < my_dist) my_dist = d;
    }
    if (my_dist == 0x7FFFFFFF) return 1;       /* we are not on the graph of the
                                                  star: our buren are unknown
                                                  territory - forward */

    /* useful iff at least one neighbour lies STRICTLY farther */
    for (int i = 0; i < MC_MAX_NEIGHBORS; i++) {
        mimeshi_neighbor *nb = &n->nbr[i];
        if (!nb->alive || !nb->sym) continue;
        int bi = r_idx(n, nb->id);
        if (bi < 0) return 1;                 /* unknown neighbour: forward */
        if (star_dist[bi] == 0xFF) return 1;  /* not reached by the star's graph:
                                                 forward */
        if (star_dist[bi] > my_dist) return 1;/* strictly farther: reach to gain */
    }
    return 0;                                 /* everyone is closer or equal: echo */
}

/* Build and send one star-round chunk (4.5). Return value: number of
 * content stars carried (0 = the chunk still went out, catalogue-only). */
static uint8_t star_round_send(mimeshi_node *n) {
    uint8_t p[230];
    uint16_t len = 10;
    p[0] = (uint8_t)PKT_STAR_ROUND;
    wr16(p + 1, n->id);
    p[3] = 0;                             /* catalogue count, filled below */
    p[4] = 0;                                   /* content star count, filled below */
    wr16(p + 5, n->geo_lat); wr16(p + 7, n->geo_lon);
    p[9] = 1;                                   /* firmware version (4.3 header shape) */

    /* --- content area: own star first, then nearest-first queue ---
     * The own star: the entry of n itself with its confirmed links,
     * serialised through the same canonical form. It is always a
     * candidate; with the own-star guarantee it goes in as soon as
     * the hour bound is reached, and its tick advances then. */
    uint8_t carried = 0;
    int own_due = (uint8_t)(n->star_own_ctr + 1) >= MC_STAR_ROUNDS_PER_H;
    int own_star_changed = n->tbl_dirty;         /* prefix mutation pending = own star changed */
    if (own_due || own_star_changed) {
        /* The own star in the chunk entry layout: id, both classes,
         * tick, geo, links - via the canonical serialiser over the
         * own map entry. */
        uint8_t sbuf[MC_STAR_BUF]; uint16_t age_off;
        /* ensure the own map entry exists (geo, presence) */
        mimeshi_mapnode *om = map_find(n, n->id);
        if (!om) { om = map_add(n, n->id); }
        if (om) {
            om->geo_lat = n->geo_lat; om->geo_lon = n->geo_lon;
            uint16_t sl = mimeshi_star_serialize(n, n->id, sbuf, sizeof sbuf, &age_off);
            if (sl > 0 && len + sl <= 230) {
                if (own_due) { n->star_own_ctr = 0; n->star_tick++; }
                else n->star_own_ctr++;
                sbuf[age_off] = star_own_tick(n);   /* heartbeat tick in the age byte (4.5) */
                memcpy(p + len, sbuf, sl);
                len = (uint16_t)(len + sl);
                carried++;
            }
        }
    } else {
        n->star_own_ctr++;
    }

    /* queue: stars whose CONTENT changed since they were last
     * carried, nearest hops first - prefix stars (direct neighbours)
     * before tail stars, in map order. The queue decision is the
     * content hash (4.5): a star goes back on the queue only when its
     * advertised content differs from the hash of its last carried
     * form; a mere age/refresh touch or sub-hysteresis EWMA movement
     * does NOT re-send it - the catalogue entry below is the cheap
     * confirmation. star_fwd_tick uses the same 16-bit time base as
     * arrived_tick, so wrap arithmetic is exact and no heuristic
     * guard is needed.
     *
     * No entry cap: stars pack until they do not fit the chunk - the
     * byte budget of the packet is the bound (4.5), not an entry
     * count inherited from the 4.3 chunk format. Thin stars pack many
     * per round (fast convergence where the air allows it), thick
     * stars few. What was carried is remembered per star
     * (star_fwd_tick + star_fwd_hash), so the next round simply
     * continues with what did not fit: a backlog drains over rounds,
     * bounded by the hard one-chunk-per-interval rule.
     *
     * One exception (4.5): the last MC_STAR_CAT_RESERVE bytes stay free
     * for catalogue entries. The catalogue is the heartbeat of the
     * resting network - without a guaranteed trickle of digests the
     * ages would run dry during a sustained content backlog and the
     * maintenance side of the round would starve. A star that does
     * not fit in the reserved space simply waits a round - the queue
     * drains, a round slower under pressure. */
    for (int pass = 0; pass < 2; pass++) {
        for (int i = 0; i < MC_MAP_NODES; i++) {
            mimeshi_mapnode *m = &n->map[i];
            if (!m->alive || m->id == n->id) continue;
            int is_prefix = nbr_find(n, m->id) != 0;
            if ((pass == 0) != (is_prefix != 0)) continue;   /* pass 0: prefix first */
            /* graph pruning (4.5): skip the star when every known
             * neighbour lies closer to (or level with) its center than
             * we do - the forward would be an echo back into the
             * gradient. Unknown neighbours never silence: no pruning
             * on missing information. The star stays in the map and
             * keeps its queue state; the missed list of a neighbour
             * that nevertheless wants it re-queues it explicitly. */
            /* pruning - unless a neighbour explicitly asked (4.3): a
             * fresh star_wanted overrides the gradient check for a few
             * intervals; the answer rides the next rounds. */
            int wanted = m->star_wanted != 0
                        && (uint32_t)(n->tick - m->star_wanted) < 2u * MC_STAR_ROUND_TICKS;
            if (!wanted && !star_fwd_useful(n, m->id)) continue;
            uint8_t sbuf[MC_STAR_BUF];
            uint16_t sl = mimeshi_star_serialize(n, m->id, sbuf, sizeof sbuf, 0);
            if (sl == 0 || len + sl > 230 - MC_STAR_CAT_RESERVE) continue;  /* full
                                          or would eat the catalogue reserve (4.3):
                                          stays queued for the next round */
            uint8_t h = mimeshi_star_content_hash(sbuf, sl, 3);
            if (m->star_fwd_tick != 0 && h == m->star_fwd_hash) continue;
             /* unchanged since the last carry (content, not age) */
            memcpy(p + len, sbuf, sl);
            len = (uint16_t)(len + sl);
            carried++;
            m->star_fwd_tick = n->tick;
            m->star_fwd_hash = h;
        }
    }
    p[4] = carried;

    /* --- catalogue area: (node id, digest) in the remaining space ---
     * 3 B per star: what we have, in which version - the receiver
     * resets ages on a derivable match and prunes its own queue (4.5).
     * Own entry first (content hash of the own star + tick), then the
     * map, nearest first again. */
    uint16_t cat_cnt = 0;
    if (len + 3 <= 230) {
        uint8_t sbuf[MC_STAR_BUF];
        mimeshi_mapnode *om = map_find(n, n->id);
        if (om) {
            uint16_t sl = mimeshi_star_serialize(n, n->id, sbuf, sizeof sbuf, 0);
            if (sl > 0) {
                uint8_t h = mimeshi_star_content_hash(sbuf, sl, 3);
                wr16(p + len, n->id);
                p[len + 2] = mimeshi_star_digest(h, star_own_tick(n));
                len += 3; cat_cnt++;
            }
        }
    }
    for (int pass = 0; pass < 2; pass++) {
        for (int i = 0; i < MC_MAP_NODES && len + 3 <= 230; i++) {
            mimeshi_mapnode *m = &n->map[i];
            if (!m->alive || m->id == n->id) continue;
            int is_prefix = nbr_find(n, m->id) != 0;
            if ((pass == 0) != (is_prefix != 0)) continue;
            uint8_t sbuf[MC_STAR_BUF];
            uint16_t sl = mimeshi_star_serialize(n, m->id, sbuf, sizeof sbuf, 0);
            if (sl == 0) continue;
            uint8_t h = mimeshi_star_content_hash(sbuf, sl, 3);
            /* tick of a foreign star: its forwarded age byte at arrival
             * (the origin's tick as it reached us); stored per map entry
             * as the low bits of arrived_tick are local - the tick
             * itself rides in the age byte of the star content, which
             * the digest derivation adds. Use the star's own age byte:
             * the serialised age IS the tick (4.5). */
            uint8_t tick = sbuf[3];
            wr16(p + len, m->id);
            p[len + 2] = mimeshi_star_digest(h, tick);
            len += 3; cat_cnt++;
        }
    }
    (void)cat_cnt;
    if (cat_cnt > 255) cat_cnt = 255;
    p[3] = (uint8_t)cat_cnt;               /* catalogue count in the header:
                                              the receiver knows where the
                                              catalogue ends and the missed
                                              area begins (the chunk-nr slot
                                              is free: star-round chunks are
                                              self-contained, no cursor) */
    if (cat_cnt) p[0] |= F_STAR_CATALOGUE;

    /* --- missed area (4.5): requests to specific neighbours ---
     * (neighbour id, n, star ids...) at 3 B + 2 B per star; the named
     * neighbour resets the last-carried state of those stars, so they
     * re-enter the content queue and travel with its next round. The
     * damping lives per entry: tries++ here, dropped after 2. */
    {
        uint16_t off = len;
        uint16_t moff = off;                    /* patch the count later */
        (void)moff;
        uint8_t carried_miss = 0;
        for (uint8_t i = 0; i < MC_STAR_MISS_MAX; i++) {
            if (!n->star_miss[i].alive) continue;
            if (n->star_miss[i].tries >= 2) {   /* damped out (4.5) */
                n->star_miss[i].alive = 0;
                continue;
            }
            if (!nbr_find(n, n->star_miss[i].nb)) {   /* neighbour gone (4.5):
                                                          the request could never
                                                          be answered - drop it */
                n->star_miss[i].alive = 0;
                continue;
            }
            /* group consecutive requests to the same neighbour: this
             * simple form packs one (nb, n=1, star) triple per entry;
             * the receiver accepts any grouping. */
            if (off + 5 > 230) break;           /* area full */
            wr16(p + off, n->star_miss[i].nb);
            p[off + 2] = 1;
            wr16(p + off + 3, n->star_miss[i].star);
            off += 5;
            n->star_miss[i].tries++;
            carried_miss++;
        }
        if (carried_miss) { p[0] |= F_STAR_MISSED; len = off; }
    }

    for (uint8_t k = 0; k < MC_N_CHANNELS; k++) {
        uint8_t c = (uint8_t)((n->home_ch + 1 + k) % MC_N_CHANNELS);
        bound_send(n, c, p, len, t_data_us(len));
        ka_called(n, c);
    }
    return carried;
}

/* Reception of a star-round chunk (4.5, receiver step).
 *
 * The CONTENT area carries stars in exactly the chunk entry layout of
 * 4.3 (id, classes, age, geo, links) - so the content processing goes
 * through the ordinary chunk entry path: a receiver that only
 * understands chunks still gets the full benefit of the content, and
 * the symmetry evidence (an owner naming us) keeps working (4.3).
 *
 * The CATALOGUE area carries (node id, digest) at 3 B: what the
 * sender has, in which version. For each entry about a star we know:
 *  - derivable match (digest = content_hash + tick + k, k <= window):
 *    the star is unchanged and its origin is alive; we advance the
 *    tick and RESET the travelled age of that star as received from
 *    this source (4.5): fresh-beats-old keeps working without a
 *    single content byte.
 *  - no fit: a real content change or a gap - the missed-list step
 *    (later sub-step of 4.5) takes this from here.
 *
 * We cannot see the sender's own star tick directly: the catalogue
 * entry for the sender's own star carries its digest, and a match
 * there also confirms the sender's liveness.
 *
 * The digest of a foreign star in OUR store: the content hash over
 * our canonical serialisation (age excluded) plus the tick we last
 * saw riding in the star's age byte - stored per map entry.
 */
#define MC_STAR_K_MAX 8u        /* tick-lag window (4.5, starting value) */

static void chunk_rx_mode(mimeshi_node *n, int8_t rssi, const uint8_t *pkt,
                          uint16_t len, int star_mode);
static void chunk_rx(mimeshi_node *n, int8_t rssi, const uint8_t *pkt, uint16_t len);

/* Missed list (4.5): a catalogue mismatch (star unknown here, or a
 * digest that fits no tick) becomes a request to the ADVERTISER - not
 * a pull exchange: the request rides the own round, the answer rides
 * the neighbour's next round as ordinary content. Damped: at most 2
 * attempts per (neighbour, star); a star that arrives meanwhile clears
 * the entry. Bounded by MC_STAR_MISS_MAX; a full list drops the new
 * request (fresh-beats-old and the regular rounds remain the net). */
static void star_miss_add(mimeshi_node *n, uint16_t nb, uint16_t star) {
    if (nb == n->id) return;
    if (!nbr_find(n, nb)) return;              /* only direct neighbours (4.5:
                                                  channel-filtered by design) */
    for (uint8_t i = 0; i < MC_STAR_MISS_MAX; i++) {
        if (n->star_miss[i].alive && n->star_miss[i].nb == nb
            && n->star_miss[i].star == star) return;      /* already queued */
    }
    for (uint8_t i = 0; i < MC_STAR_MISS_MAX; i++) {
        if (!n->star_miss[i].alive) {
            n->star_miss[i].alive = 1;
            n->star_miss[i].nb = nb;
            n->star_miss[i].star = star;
            n->star_miss[i].tries = 0;
            return;
        }
    }
    /* full: drop silently - the damping rule keeps the list short in
     * practice; a persistent gap resurfaces via the next mismatch. */
}

/* A star we were missing arrived (content or derivable match): clear
 * every miss entry about it (4.5 damping: no re-request after fetch). */
static void star_miss_clear(mimeshi_node *n, uint16_t star) {
    for (uint8_t i = 0; i < MC_STAR_MISS_MAX; i++)
        if (n->star_miss[i].alive && n->star_miss[i].star == star)
            n->star_miss[i].alive = 0;
}

static void star_round_rx(mimeshi_node *n, int8_t rssi, const uint8_t *pkt, uint16_t len) {
    if (len < 10) return;
    uint16_t owner = rd16(pkt + 1);      /* the advertiser: miss requests address it */

    /* --- content area: chunk entry layout, processed via chunk_rx ---
     * The header (10 B) is identical to a 4.3 chunk, and the entries
     * follow the same layout. chunk_rx() expects p[4] entries and
     * processes exactly that: reuse it wholesale by handing it the
     * same packet; the type nibble differs but chunk_rx does not
     * look at it.
     *
     * After the pass, the star semantics differ from a plain 4.3
     * chunk in exactly one field: the age byte of a star-round entry
     * carries the ORIGIN HEARTBEAT TICK (4.5), not the travelled age.
     * chunk_rx stored it into age_ticks; we move it to star_tick and
     * reset age_ticks to 0 - the star arrived NOW, its travelled age
     * starts here (4.4 semantics), and the tick is the liveness. */
    uint16_t cend = 10;                      /* end of the content area */
    for (uint8_t e = 0; e < pkt[4] && cend + 9 <= len; e++)
        cend = (uint16_t)(cend + 9 + 3 * pkt[cend + 8]);   /* walk the content stars */
    chunk_rx_mode(n, rssi, pkt, cend, 1);   /* entries only: the catalogue and
                                               missed bytes are not entries */
    uint16_t rx_star[MC_CHUNK_MAX_ENTRIES];     /* content stars of THIS packet */
    uint8_t rx_star_n = 0;
    {
        uint16_t off = 10;
        for (uint8_t e = 0; e < pkt[4] && off + 9 <= cend; e++) {
            uint16_t eid = rd16(pkt + off);
            if (eid != n->id) {                    /* our own star is not in the map */
                mimeshi_mapnode *em = map_find(n, eid);
                if (em) {
                    /* fresh beats old (4.3): a form that lags more than
                     * the k-window behind the stored tick is a stale
                     * re-send - the newer knowledge stands, only the
                     * request is settled. */
                    int16_t d = (int16_t)((int16_t)em->star_tick - (int16_t)pkt[off + 3]);
                    if (!(em->adv_q
                          && (d > (int16_t)MC_STAR_K_MAX || d < -(int16_t)MC_STAR_K_MAX))) {
                        em->star_tick = pkt[off + 3];  /* the tick as it traveled (4.3) */
                        em->age_ticks = 0;              /* arrived NOW: travelled age starts here */
                        em->arrived_tick = n->tick;
                    }
                }
                star_miss_clear(n, eid);            /* a missing star arrived:
                                                     its requests are settled (4.3) */
            }
            if (rx_star_n < MC_CHUNK_MAX_ENTRIES) rx_star[rx_star_n++] = eid;
            off += 9 + 3 * pkt[off + 8];
        }
    }

    /* --- catalogue area: after the content stars (4.5) ---
     * The number of content stars is p[4]; the entries follow the
     * chunk entry layout, so the catalogue starts after the last
     * content star. (n, digest) triples fill the rest. */
    uint16_t off = cend;                       /* the content walk ended here */
    if (!(pkt[0] & F_STAR_CATALOGUE)) { off = len; }
    uint8_t cat_n = pkt[3];                    /* catalogue count (header) */
    uint16_t cat_end = off + 3 * cat_n;
    if (cat_end > len) { cat_end = len; }      /* malformed: clip */
    for (; off + 3 <= cat_end; off += 3) {
        uint16_t sid = rd16(pkt + off);
        uint8_t digest = pkt[off + 2];
        if (sid == n->id) continue;              /* our own star: we are the origin */
        int in_content = 0;                       /* this packet already carried it */
        for (uint8_t i = 0; i < rx_star_n; i++)
            if (rx_star[i] == sid) { in_content = 1; break; }
        if (in_content) continue;                 /* fresh content beats the
                                                     catalogue mismatch: no
                                                     request (4.5) */
        mimeshi_mapnode *m = map_find(n, sid);
        if (!m) {
            /* star not known here while the neighbour advertises it:
             * a gap - onto the missed list (4.5). */
            star_miss_add(n, owner, sid);
            continue;
        }
        /* our version: content hash over our canonical bytes (age
         * excluded), tick = the heartbeat as it reached us. */
        uint8_t sbuf[MC_STAR_BUF];
        uint16_t sl = mimeshi_star_serialize(n, sid, sbuf, sizeof sbuf, 0);
        if (sl == 0) continue;
        uint8_t h = mimeshi_star_content_hash(sbuf, sl, 3);
        uint8_t our_tick = m->star_tick;        /* origin heartbeat tick (4.5):
                                                   separate from age_ticks, which
                                                   keeps its 4.4 meaning */
        int k = mimeshi_star_tick_match(digest, h, our_tick, MC_STAR_K_MAX);
        if (k != -1) {
            /* derivable match (4.3): unchanged, origin alive. Advance
             * the tick and reset the travelled age - the knowledge is
             * confirmed without content. A star we were missing is
             * hereby settled: clear its miss entries. A lagging
             * advertisement (k > 16: a slower route) confirms the same
             * content: only the age refreshes, the newer stored tick
             * stands (the origin has already reached it). */
            if (k > 0 && k <= (int)MC_STAR_K_MAX)
                m->star_tick = (uint8_t)(our_tick + (uint8_t)k);
            m->age_ticks = 0;                   /* travelled age restarts (4.4) */
            m->arrived_tick = n->tick;          /* fresh as of NOW (4.4) */
            star_miss_clear(n, sid);
        } else {
            /* no tick fits: a real content change upstream (or a hash
             * collision) - onto the missed list; the neighbour re-sends
             * it as content in its next round (4.5). */
            star_miss_add(n, owner, sid);
        }
    }

    /* --- missed area (4.5): requests addressed to US ---
     * A neighbour is missing a star (or saw a content change): reset
     * the last-carried state so the star re-enters the content queue
     * and travels with our next round - no exchange, no ack, the
     * regular cadence is the answer (4.5). */
    if (pkt[0] & F_STAR_MISSED) {
        uint16_t moff = cat_end;                /* where the catalogue ended */
        for (; moff + 3 <= len;) {
            uint16_t nb = rd16(pkt + moff);
            uint8_t cnt = pkt[moff + 2];
            moff += 3;
            if (moff + 2 * cnt > len) break;    /* malformed: ignore the rest */
            if (nb == n->id) {
                for (uint8_t i = 0; i < cnt; i++) {
                    uint16_t star = rd16(pkt + moff + 2 * i);
                    mimeshi_mapnode *m = map_find(n, star);
                    if (m) {
                        m->star_fwd_tick = 0;   /* as never carried: queued again */
                        m->star_fwd_hash = 0;
                        m->star_wanted = n->tick;   /* explicit ask: the
                                        pruning yields for a few
                                        intervals (4.3) */
                    }
                }
            }
            moff += 2 * cnt;
        }
    }
}

/* Round timer (4.3): per-node phase offset; the interval is the hard
 * bound - one chunk per node per interval. */
static void star_round_tick(mimeshi_node *n, void *ud) {
    (void)ud;
    n->star_round_sched = 0;
    if (n->tbl_dirty) { n->tbl_dirty = 0; }       /* the round carries mutations (4.3) */
    star_round_send(n);
    n->star_round_sched = 1;
    n->timer(n, MC_STAR_ROUND_TICKS * MC_KEEPALIVE_TICK_US
             + (n->rand(n) % MC_KEEPALIVE_TICK_US), star_round_tick, 0);
}

/* Heartbeat tick match (4.3): the received digest fits when it was
 * computed from the same content hash at stored_tick + k, for small
 * k. The window is SYMMETRIC: a catalogue may lag behind our stored
 * tick (a slower route carried the newer tick here first, 4.3) - an
 * older advertisement with the same content hash still confirms the
 * content. Return convention: 0 = exact, 1..k_max = the advertisement
 * is AHEAD (advance the stored tick by k), 17..16+k_max = the
 * advertisement LAGS by (r - 16) (keep the stored tick, only the age
 * refreshes), -1 = no fit (a real content change, or an unknown
 * star). The two windows cannot collide with the sentinel. */
#define MC_TICK_LAG_BASE 16
int mimeshi_star_tick_match(uint8_t received, uint8_t content_hash,
                           uint8_t stored_tick, uint8_t k_max) {
    if (received == mimeshi_star_digest(content_hash, stored_tick)) return 0;
    for (uint8_t k = 1; k <= k_max; k++) {
        if (received == mimeshi_star_digest(content_hash,
                                            (uint8_t)(stored_tick + k)))
            return (int)k;
        if (received == mimeshi_star_digest(content_hash,
                                            (uint8_t)(stored_tick - k)))
            return (int)k + MC_TICK_LAG_BASE;
    }
    return -1;
}

/* ---------------- route computation (10.2, building block 2) ------
 * Stateless per message: no path in the header, every hop computes its
 * own first step on the own known graph. Cost model (decided, Oct
 * 2026): ETX per link = 1/p on the integer scale x16, one lookup table
 * indexed on the advertised quality class; the route cost is the sum
 * over the path, so the computation minimises the expected number of
 * transmissions. Age multiplier (10.2): an endpoint above half of
 * T_active makes the edge 1.5x as expensive, above T_active itself
 * 3x - stale knowledge loses against fresh. First step (hard
 * requirement, 10.1): only confirmed symmetric neighbours above the
 * quality threshold. */
#define MC_ROUTE_SCALE 16u                /* cost 1.0 (10.2) */
static const uint16_t mc_etx_class[8] = {
    0,                                /* class 0: not routable (10.2) */
    228, 145, 94, 64, 43, 29, 20      /* classes 1..7: cost x16 (10.2 table) */
};

/* Age multiplier on an edge endpoint: follows the effective age of the
 * map entry (4.4), in proof ticks (~56 s). Returns 2 = 1.5x, 3 = 3x. */
static uint16_t route_age_mult(uint32_t age) {
    if (age >= MC_T_ACTIVE_TICKS) return 3;
    if (age >= MC_T_ACTIVE_TICKS / 2) return 2;
    return 0;
}

static uint32_t mult_apply(uint32_t c, uint16_t m) {
    if (m == 3) return c * 3;
    if (m == 2) return c * 3 / 2;
    return c;
}

/* Cost of the own edge toward a confirmed neighbour: the measured
 * quality class (finer than the advertised classes), with the age
 * addition of the map entry when we have one. Penalty (10.4) follows
 * in a later building step. */
static uint32_t route_edge_own(mimeshi_node *n, mimeshi_neighbor *nb) {
    uint8_t cls = adv_class_of(nb->quality);
    uint32_t c = (cls < 8 && mc_etx_class[cls]) ? mc_etx_class[cls] : MC_ROUTE_SCALE;
    mimeshi_mapnode *m = map_find(n, nb->id);
    if (m) c = mult_apply(c, route_age_mult(map_age_ticks(n, m)));
    return c;
}

/* Dijkstra scratch: bounded like the map (13.2); static because the
 * core has no heap and is single-threaded per node (13b). */
#define MC_DIST_INF 0x7FFFFFFFu
static uint32_t r_dist[MC_MAP_NODES];
static uint16_t r_prev[MC_MAP_NODES];   /* predecessor map slot; 0xFFFE = own edge */
static uint8_t  r_done[MC_MAP_NODES];

static int r_idx(mimeshi_node *n, uint16_t id) {
    for (int i = 0; i < MC_MAP_NODES; i++)
        if (n->map[i].alive && n->map[i].id == id) return i;
    return -1;
}

/* The first hop toward dst on the own known graph, or 0 when dst is
 * unreachable. The first step runs only over confirmed symmetric
 * neighbours above the quality threshold (10.1); further edges run
 * over the link store, direction of travel known, endpoints not
 * expired (4.4). A confirmed direct neighbour is its own first hop
 * even without a map slot: it is the destination itself.
 * `excl` (excl_cnt entries) are next hops that already failed for
 * THIS message: cumulative exclusion per series (7.4) - the
 * computation must not propose a hop the series has buried. */
static uint16_t route_next_hop_ex(mimeshi_node *n, uint16_t dst,
                                  const uint16_t *excl, uint8_t excl_cnt) {
    if (dst == n->id) return n->id;
    int di = r_idx(n, dst);
    /* A direct neighbour is NOT its own first hop by definition: its
     * own edge competes in the computation like every other edge, and
     * two good hops beat one bad hop (10.2, ETX). But a confirmed
     * neighbour without a map slot must still be able to WIN that
     * competition when it is the destination itself - otherwise a
     * fresh symmetric neighbour is unreachable until its chunk
     * arrives (measured: 'no candidate hop' losses under load). */
    mimeshi_neighbor *dnb = nbr_find(n, dst);
    int direct_ok = (dnb && dnb->sym && quality_ok(dnb->quality));
    if (di < 0) {
        if (direct_ok) {
            for (uint8_t e = 0; e < excl_cnt; e++) if (excl[e] == dst) return 0;
            return dst;                          /* the only edge toward it is the own one */
        }
        return 0;                                /* not in the graph: no flood (10.2) */
    }
    for (int i = 0; i < MC_MAP_NODES; i++) { r_dist[i] = MC_DIST_INF; r_prev[i] = 0xFFFF; r_done[i] = 0; }
    for (int i = 0; i < MC_MAX_NEIGHBORS; i++) {           /* own edges (virtual start) */
        mimeshi_neighbor *nb = &n->nbr[i];
        if (!nb->alive || !nb->sym || !quality_ok(nb->quality)) continue;
        int ex = 0;
        for (uint8_t e = 0; e < excl_cnt; e++) if (excl[e] == nb->id) { ex = 1; break; }
        if (ex) continue;                          /* failed for this message (7.4) */
        int bi = r_idx(n, nb->id);
        if (bi < 0) continue;                     /* no map slot: no further edges */
        uint32_t c = route_edge_own(n, nb);
        if (c < r_dist[bi]) { r_dist[bi] = c; r_prev[bi] = 0xFFFE; }
    }
    for (;;) {
        int u = -1;
        for (int i = 0; i < MC_MAP_NODES; i++)
            if (!r_done[i] && r_dist[i] < MC_DIST_INF && (u < 0 || r_dist[i] < r_dist[u])) u = i;
        if (u < 0 || u == di) break;              /* nothing left, or target settled */
        r_done[u] = 1;
        uint16_t mult_u = route_age_mult(map_age_ticks(n, &n->map[u]));
        for (int j = 0; j < MC_MAX_LINKS; j++) {           /* relax the edges of map[u] */
            mimeshi_link *l = &n->lnk[j];
            if (!l->alive) continue;
            uint16_t from = n->map[u].id;
            if (l->a != from && l->b != from) continue;
            uint8_t cf = (l->a == from) ? (uint8_t)((l->q >> 4) & 0x0F) : (uint8_t)(l->q & 0x0F);
            if (cf == 0) continue;                          /* direction of travel unknown */
            uint32_t c = mc_etx_class[cf & 7];
            if (!c) continue;
            c = mult_apply(c, mult_u);                     /* stale endpoint u */
            uint16_t o = (l->a == from) ? l->b : l->a;
            int oi = r_idx(n, o);
            if (oi < 0 || r_done[oi]) continue;
            c = mult_apply(c, route_age_mult(map_age_ticks(n, &n->map[oi])));
            if (r_dist[u] + c < r_dist[oi]) { r_dist[oi] = r_dist[u] + c; r_prev[oi] = (uint16_t)u; }
        }
    }
    if (r_dist[di] >= MC_DIST_INF) return 0;     /* unreachable in the graph */
    int cur = di;                                 /* walk back to the own edge */
    while (r_prev[cur] != 0xFFFE) {
        cur = r_prev[cur];
        if (cur == 0xFFFF) return 0;
    }
    return n->map[cur].id;
}

static uint16_t route_bridge_node(mimeshi_node *n, uint16_t lat, uint16_t lon,
                                  const uint16_t *excl, uint8_t excl_cnt);

/* Plain variant: no exclusions (first choice, 10.2). */
static uint16_t route_next_hop(mimeshi_node *n, uint16_t dst) {
    return route_next_hop_ex(n, dst, 0, 0);
}

/* First hop toward dst with, when known, the dest geo (10.2): the
 * graph decides when the destination lies in it; otherwise the
 * bridge-node rule routes toward the closest graph node near the
 * dest geo (15% candidate band, best route cost wins). geo (0,0)
 * = unknown: then a destination outside the graph is refused. */
static uint16_t route_next_hop_geo(mimeshi_node *n, uint16_t dst,
                                  uint16_t lat, uint16_t lon,
                                  const uint16_t *excl, uint8_t excl_cnt) {
    uint16_t nh = route_next_hop_ex(n, dst, excl, excl_cnt);
    if (nh) return nh;
    if (!lat && !lon) return 0;               /* no geo: no flood (10.2) */
    uint16_t bridge = route_bridge_node(n, lat, lon, excl, excl_cnt);
    if (!bridge) return 0;
    return route_next_hop_ex(n, bridge, excl, excl_cnt);
}

/* Squared distance on the coarse 16-bit geo grid (5): comparison only,
 * so no sqrt. Longitude steps shrink with cos(latitude); at the NL
 * latitudes the factor is roughly 3/4 - folded into a fixed weight,
 * good enough to rank bridge-node candidates (10.2). */
static uint32_t geo_dist2(uint16_t lat1, uint16_t lon1, uint16_t lat2, uint16_t lon2) {
    int32_t dlat = (int32_t)lat1 - (int32_t)lat2;
    int32_t dlon = ((int32_t)lon1 - (int32_t)lon2) * 3 / 4;
    return (uint32_t)(dlat * dlat + dlon * dlon);
}

/* Route cost toward dst on the own graph (10.2), or MC_DIST_INF when
 * unreachable. Shares the Dijkstra scratch with route_next_hop_ex: a
 * run settles every reachable node, so the cost of any candidate can
 * be read from r_dist afterwards. */
static uint32_t route_cost_to(mimeshi_node *n, uint16_t dst,
                              const uint16_t *excl, uint8_t excl_cnt) {
    if (!route_next_hop_ex(n, dst, excl, excl_cnt)) return MC_DIST_INF;
    int di = r_idx(n, dst);
    if (di < 0) return MC_DIST_INF;
    return r_dist[di];
}

/* Bridge-node rule (10.2, decided Oct 2026): the destination lies
 * outside the own graph but its geo is known. Candidates: all graph
 * nodes within 15% of the distance of the closest one (the margin
 * grows with the absolute distance - far from the destination there
 * is more freedom to pick a better path); among the candidates the
 * best route cost wins. Distance comparison on the square: 1.15^2 =
 * 1.3225, so the margin is +32.25% on d2. */
static uint16_t route_bridge_node(mimeshi_node *n, uint16_t lat, uint16_t lon,
                                  const uint16_t *excl, uint8_t excl_cnt) {
    uint32_t best_d = 0;
    int have = 0;
    for (int i = 0; i < MC_MAP_NODES; i++) {
        if (!n->map[i].alive || !n->map[i].geo_lat) continue;   /* no geo: no candidate (5) */
        uint32_t d = geo_dist2(n->map[i].geo_lat, n->map[i].geo_lon, lat, lon);
        if (!have || d < best_d) { best_d = d; have = 1; }
    }
    if (!have) return 0;                       /* no geo material at all */
    uint32_t margin = best_d + best_d * 3225u / 10000u;
    /* first pass: find the best (minimum) route cost among the
     * candidates; the Dijkstra scratch is filled per candidate run.
     * Candidates are usually few; the closest-first scan keeps the
     * result deterministic. */
    uint16_t pick = 0;
    uint32_t pick_cost = MC_DIST_INF;
    for (int i = 0; i < MC_MAP_NODES; i++) {
        if (!n->map[i].alive || !n->map[i].geo_lat) continue;
        uint32_t d = geo_dist2(n->map[i].geo_lat, n->map[i].geo_lon, lat, lon);
        if (d > margin) continue;              /* outside the 15% band (10.2) */
        uint32_t c = route_cost_to(n, n->map[i].id, excl, excl_cnt);
        if (c < pick_cost) { pick_cost = c; pick = n->map[i].id; }
    }
    return pick;                                /* 0 when no candidate routable */
}




/* Reception of a table chunk (4.3). Self-contained: every entry is
 * processed on its own against the own map and link store. The owner
 * naming US in an entry is reverse-direction evidence for the link
 * owner->us (10.1) — gated on quality like every confirmation. */
static void chunk_rx(mimeshi_node *n, int8_t rssi, const uint8_t *pkt, uint16_t len) {
    chunk_rx_mode(n, rssi, pkt, len, 0);
}
static void chunk_rx_mode(mimeshi_node *n, int8_t rssi, const uint8_t *pkt,
                          uint16_t len, int star_mode) {
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
        uint8_t qbyte = pkt[off + 2];
        uint8_t cls = qbyte >> 4;              /* origin->entry class (4.3) */
        uint8_t age = pkt[off + 3];
        uint16_t gl = rd16(pkt + off + 4), go = rd16(pkt + off + 6);
        uint8_t lcnt = pkt[off + 8];
        off += 9;
        if (eid == n->id) {                    /* the origin's own entry naming US: reverse evidence (10.1) */
            if (nb) {
                nb->proven_age_us = 0;
                /* the origin's entry class for us IS its measurement of
                 * our signal: our reverse quality, refreshed with every
                 * star that names us (6.2) — no extra airtime */
                uint8_t oq = quality_from_class(cls);
                nb->rev_quality = quality_ewma(nb->rev_quality ? nb->rev_quality : oq, oq);
                if (quality_ok(nb->quality)) {
                    if (!nb->sym) tbl_nbr_added(n, owner);
                    nb->sym = 1; nb->ack_try = 0;
                }
            }
            /* our entry in the origin's star carries the origin's links,
             * ours included: consume them like every entry's links and
             * store them as received (the owner's advertised form of OUR
             * link is routing material, 4.3) - the link loop below runs
             * for this entry too. */
            for (uint8_t k = 0; k < lcnt && off + 3 <= len; k++) {
                uint16_t peer = rd16(pkt + off);
                uint8_t lb = pkt[off + 2];
                off += 3;
                if (peer == n->id || peer == owner) continue;  /* our own entry in
                    the origin's star names only us; a self-link does not exist */
                mimeshi_mapnode *pm = map_add(n, peer);  /* named by the link (10.1) */
                if (pm) { pm->age_ticks = age; pm->arrived_tick = n->tick; }
                lnk_set_class(n, eid, peer, (uint8_t)(lb >> 4));
                lnk_set_class(n, peer, eid, (uint8_t)(lb & 0x0F));
            }
        } else {
            mimeshi_mapnode *em = map_add(n, eid);
            int stale = 0;
            if (em && star_mode) {
                /* fresh beats old on CONTENT (4.3): a form whose
                 * advertised entry byte differs from the stored one is
                 * only accepted when its heartbeat tick is AHEAD of
                 * the stored tick - a differing form at an equal or
                 * older tick is a stale copy racing a newer one (a
                 * slow route, a missed-list answer) and must not roll
                 * the newer knowledge back. Identical content is
                 * always fine (an echo refreshes nothing but the
                 * age). Geo and presence still refresh - map
                 * material is never wasted. */
                int16_t ahead = (int16_t)((int16_t)age - (int16_t)em->star_tick);
                if (em->adv_q && qbyte != em->adv_q && ahead <= 0)
                    stale = 1;
            }
            if (em && !stale) {
                if (gl) { em->geo_lat = gl; em->geo_lon = go; }   /* (0,0) = none (5) */
                /* storage as received (4.3): the entry class byte is
                 * stored exactly as the origin advertised it - reproduced
                 * unchanged on every hop, hashed by the content hash.
                 * The local quality (our own measurement of the node's
                 * signal, refreshed below on any identifying reception)
                 * stays separate: routing material, never forwarded. */
                em->adv_q = qbyte;
                em->age_ticks = age; em->arrived_tick = n->tick;
            }
            /* the star's links: claims of the ORIGIN (eid), stored as
             * received (4.3) - the link store is the forwarded picture,
             * the advertised forms of the owning stars. Skipped for a
             * stale form: its link claims belong to the older copy. */
            if (!stale) {
                for (uint8_t k = 0; k < lcnt && off + 3 <= len; k++) {
                    uint16_t peer = rd16(pkt + off);
                    uint8_t lb = pkt[off + 2];
                    off += 3;
                    if (peer == n->id) {
                        /* the origin's (eid's) link to US: reverse-direction
                         * evidence (10.1) - the high nibble is the origin's
                         * measurement of our signal, the low nibble our class
                         * as it reported it. Only meaningful when the origin
                         * is a direct neighbour of ours (the claim reaches us
                         * possibly via a forwarder, 4.3). The link itself is
                         * stored as received like every other claim (4.3):
                         * the owner's advertised form of OUR link is routing
                         * material - the old cadence covered it in our own
                         * entry, the star round carries it only here. */
                        lnk_set_class(n, eid, peer, (uint8_t)(lb >> 4));
                        lnk_set_class(n, peer, eid, (uint8_t)(lb & 0x0F));
                        mimeshi_neighbor *enb = nbr_find(n, eid);
                        if (enb) {
                            enb->proven_age_us = 0;
                            uint8_t oq = quality_from_class((uint8_t)(lb >> 4));
                            enb->rev_quality = quality_ewma(enb->rev_quality ? enb->rev_quality : oq, oq);
                            if (quality_ok(enb->quality)) {
                                if (!enb->sym) tbl_nbr_added(n, eid);
                                enb->sym = 1; enb->ack_try = 0;
                            }
                        }
                        continue;
                    }
                    mimeshi_mapnode *pm = map_add(n, peer);  /* named by the link (10.1) */
                    if (pm) {
                        /* the peer is exactly as fresh as the star that named
                         * it: the origin just advertised knowledge of it. */
                        pm->age_ticks = age; pm->arrived_tick = n->tick;
                    }
                    lnk_set_class(n, eid, peer, (uint8_t)(lb >> 4));
                    lnk_set_class(n, peer, eid, (uint8_t)(lb & 0x0F));
                }
            } else {
                off += 3 * lcnt;                /* skip the stale form's links */
            }
        }
    }
}

static void nbr_expire(mimeshi_node *n);

/* Death detection (4.4): every identifying reception refreshes
 * last_seen; per proof tick it ages. At T_expire (~4 h) the neighbour
 * is dead: the entry disappears from our table, and with it from our
 * own star - silence plus the round propagates the removal (4.4:
 * no tombstone, no negative delta). */
static void nbr_expire(mimeshi_node *n) {
    for (int i = 0; i < MC_MAX_NEIGHBORS; i++) {
        mimeshi_neighbor *nb = &n->nbr[i];
        if (!nb->alive) continue;
        if (nb->last_seen < 0xFFFF) nb->last_seen++;
        if (nb->last_seen >= MC_T_EXPIRE_TICKS) {
            nb->alive = 0; nb->sym = 0; nb->ack_try = 0;
            nb->quality = 0; nb->rev_quality = 0; nb->proven_age_us = 0;
            tbl_mark_dirty(n);              /* the own star loses the link (4.3) */
            printf("[core] node %u: neighbour %u expired after ~4 h of silence\n", n->id, nb->id);
        }
    }
}

static void proof_tick(mimeshi_node *n, void *ud) {
    (void)ud;
    n->tick++;                                 /* local time base, ~56 s per tick (4.4) */
    nbr_expire(n);                             /* death detection (4.4) */
    /* map aging (4.4): an entry nobody has refreshed anymore dies at
     * 2 x T_active (~24 h) - after that it is stale beyond every
     * cadence and only takes pool space from fresher knowledge. */
    for (int i = 0; i < MC_MAP_NODES; i++) {
        if (!n->map[i].alive) continue;
        if (nbr_find(n, n->map[i].id)) continue;           /* prefix work */
        if (map_age_ticks(n, &n->map[i]) >= 2 * MC_T_ACTIVE_TICKS)
            map_evict(n, &n->map[i]);
    }
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
/* Pending hello-ack between wait and fire: the measured quality of the
 * hello travels along to the fire moment (no extra airtime). */
#define MC_MAX_PENDING_HACKS 8
typedef struct { uint16_t to; uint8_t quality; uint8_t used; } pending_hack;
static pending_hack phack[MC_MAX_PENDING_HACKS];

static pending_hack *phack_get(uint16_t to) {
    for (int i = 0; i < MC_MAX_PENDING_HACKS; i++)
        if (phack[i].used && phack[i].to == to) return &phack[i];
    for (int i = 0; i < MC_MAX_PENDING_HACKS; i++)
        if (!phack[i].used) { phack[i].used = 1; phack[i].to = to; return &phack[i]; }
    return 0;                                   /* full: rare, ack goes without quality */
}

static void hello_ack_fire(mimeshi_node *n, void *ud) {
    uint16_t to = (uint16_t)(uintptr_t)(size_t)ud;
    uint8_t p[6] = { PKT_HELLO_ACK, 0,0, 0,0, 0 };
    wr16(p + 1, to);                       /* destination: the hello sender */
    wr16(p + 3, n->id);
    p[5] = 0;
    for (int i = 0; i < MC_MAX_PENDING_HACKS; i++)
        if (phack[i].used && phack[i].to == to) { p[5] = phack[i].quality; phack[i].used = 0; break; }
    bound_send(n, (uint8_t)(to % MC_N_CHANNELS), p, 6, t_hack_us()); /* to HOME of the sender */
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
    pending_hack *ph = phack_get(from);
    if (ph) ph->quality = quality_of(rssi);      /* what we measured on the hello */
    n->timer(n, slot + jitter, hello_ack_fire, (void *)(size_t)from);
}

/* ---------------- unicast + acks (7) ---------------- */

/* Ack1: immediately, on the channel of the ongoing data exchange — the
 * sender waits there for the answer (switch rule, 2). The sender-id
 * travels along so the sender knows who has been heard. */
static void ack1_send(mimeshi_node *n, uint16_t msg_id, uint16_t src, uint8_t ch, int8_t rssi) {
    uint8_t p[8] = { PKT_ACK1, 0,0, 0,0, 0,0, 0 };
    wr16(p + 1, msg_id); wr16(p + 3, src); wr16(p + 5, n->id);
    p[7] = quality_of(rssi);              /* what we measured on the data (6.2) */
    n->tx(n, ch, p, 8, t_ack_us());       /* same channel as the data (switch rule) */
    ka_called(n, ch);                     /* own call on the data channel (4.1) */
}

/* Ack2 to the previous hop, on ITS home channel (2, 7.1): the previous
 * hop listens there without its own window. Positive = the own next
 * hop has ack1'd, so the message is two hops along (7.1/7.2, revised
 * Oct 2026); negative = forwarding failed after every series, via
 * F_NEG_ACK2, with the spent budget of the message in byte 7 (7.4):
 * the previous hop sees what is left and can stop against an
 * exhausted budget. */
static void ack2_send(mimeshi_node *n, uint16_t msg_id, uint16_t src, uint16_t prev,
                      bool neg, uint8_t budget_left) {
    uint8_t p[8] = { (uint8_t)(PKT_ACK2 | (neg ? F_NEG_ACK2 : 0)), 0,0, 0,0, 0,0, 0 };
    wr16(p + 1, msg_id); wr16(p + 3, src); wr16(p + 5, n->id);
    p[7] = budget_left;                       /* spent budget travels along (6.2, 7.4) */
    bound_send(n, (uint8_t)(prev % MC_N_CHANNELS), p, 8, t_ack_us()); /* HOME of previous hop */
    ka_called(n, (uint8_t)(prev % MC_N_CHANNELS));  /* own call there (4.1) */
}

static void pending_try(mimeshi_node *n, mimeshi_pending *p);
static void schedule_fail(mimeshi_node *n, mimeshi_pending *p);

/* Candidate collector for the attempt schedule (7.4): every candidate is
 * the BEST HOP of a full Dijkstra run - not a ranking of own edges.
 * Candidate 1 is the computed route toward dst; candidate 2 is the
 * best route that does not start with candidate 1; candidate 3
 * likewise without the first two. So each candidate is a real,
 * computed alternative over the whole known graph, and the best
 * candidate always leads. Same admission rules as every route
 * (10.1/10.2): confirmed symmetric, above the quality gate, never
 * the previous hop (no bounce). */
static uint8_t schedule_candidates(mimeshi_node *n, uint16_t dst,
                                uint16_t gl, uint16_t go,
                                uint16_t prev,
                                const uint16_t *excl, uint8_t excl_cnt,
                                uint16_t *cand) {
    uint16_t tmp[3];
    uint8_t cnt = 0;
    for (int round = 0; round < 3; round++) {
        /* prev goes into the Dijkstra exclusion itself, exactly like
         * the baseline first choice (10.2): a post-filter that drops
         * the result when it happens to be prev would discard the
         * whole plan when the best route leads back - the second-best
         * non-prev route exists but was never computed (measured:
         * silent hop drops, 115 T_ack2 timeouts at the sender). */
        uint8_t ec = 0;
        if (prev) tmp[ec++] = prev;
        for (uint8_t e = 0; e < excl_cnt && ec < 3; e++) tmp[ec++] = excl[e];
        for (uint8_t k = 0; k < cnt && ec < 3; k++) tmp[ec++] = cand[k];
        uint16_t nh = route_next_hop_geo(n, dst, gl, go, tmp, ec);
        if (!nh) break;
        cand[cnt++] = nh;
    }
    return cnt;
}

/* attempts of the plan by its size: 1 candidate = 4 (1 fast + 3 slow);
 * 2 = 5 alternating (the best hop gets the extra attempt); 3 = 6. */
/* Total attempts of the schedule by its size (7.4): 1 candidate = 4
 * (A,A,A,A - the damper table then runs 0.8, 1.2, 1.8 s); 2 = 6
 * (A,B,A,B,A,B - each node 3x); 3 = 6 (A,B,C,A,B,C - each node 2x).
 * At most 4 toward the same node, at most 6 total. */
static uint8_t schedule_attempts(const mimeshi_pending *p) {
    return p->cand_cnt == 1 ? 4 : 6;
}

/* The wait before attempt try_idx+1 (7.4). Small fixed gap inside a
 * pass over the candidates - the next attempt goes to a DIFFERENT
 * node, so no damper needed; escalating gap at each pass boundary -
 * the same node comes back there. */
static uint64_t schedule_wait_us(mimeshi_node *n, const mimeshi_pending *p) {
    uint8_t pass = p->try_idx / p->cand_cnt;            /* completed passes */
    uint8_t boundary = (uint8_t)((p->try_idx % p->cand_cnt) == 0 && pass > 0);
    uint32_t base = boundary ? PLAN_PASS_MS[pass > 2 ? 2 : pass] : PLAN_INTRA_MS;
    uint32_t jit = base / 4;                            /* +-25% */
    return 1000ull * (base - jit + (n->rand(n) % (2 * jit + 1)));
}

static void retry_fire(mimeshi_node *n, void *ud) {
    mimeshi_pending *p = (mimeshi_pending *)ud;
    if (!p->active || p->acked1) return;
    p->try_idx++;
    p->budget_left--;
    pending_try(n, p);
}

/* Ack1 stayed out: attempts left -> retry after schedule_wait_us; otherwise
 * the schedule for this hop failed definitively (cumulative exclusion +
 * negative ack2, 7.4). */
static void ack1_timeout(mimeshi_node *n, void *ud) {
    mimeshi_pending *p = (mimeshi_pending *)ud;
    if (!p->active || p->acked1) return;
    if (p->try_idx < schedule_attempts(p) - 1 && p->budget_left > 0) {
        n->timer(n, schedule_wait_us(n, p), retry_fire, p);
        return;
    }
    schedule_fail(n, p);
}

/* The attempt schedule toward this destination is exhausted (7.4,
 * revised Oct 2026 - no rebuild: the 6 attempts ARE the retry space of
 * the hop). All candidates go into the cumulative exclusion together
 * and the message fails definitively with the negative ack2 to the
 * previous hop, the spent budget along. The previous hop decides with
 * its own plan and budget whether another route remains. */
static void schedule_fail(mimeshi_node *n, mimeshi_pending *p) {
    printf("[core] node %u: schedule failed toward %u..%u (msg %u)\n",
           n->id, p->cand[0], p->cand[p->cand_cnt ? p->cand_cnt - 1 : 0], p->msg_id);
    for (uint8_t k = 0; k < p->cand_cnt && p->excl_cnt < 3; k++) {
        p->excl[p->excl_cnt++] = p->cand[k];      /* cumulative (7.4) */
        mimeshi_neighbor *fnb = nbr_find(n, p->cand[k]);
        if (fnb && fnb->penalty < 255) fnb->penalty++;   /* long-term lesson (10.4) */
    }
    p->active = 0;
    printf("[core] node %u: FAIL msg %u (schedule exhausted, budget %u)\n",
           n->id, p->msg_id, p->budget_left);
    if (p->prev_hop != n->id)
        ack2_send(n, p->msg_id, p->src, p->prev_hop, true, p->budget_left);
}

/* One transmit attempt: update the budget in the header (offset 9,
 * 6.1/9), the exchange toward next_hop, and schedule the ack1 timeout.
 * The timeout is (7.3) S + t_data + t_ack + margin, measured from the
 * moment of submission. Assumption: the platform layer delivers the
 * transmission quickly (queue short); if the switch takes longer, the
 * timeout fires too early and the attempt counts as failed. */
/* T_ack2 timeout (7.3, fixed 15 s): the next hop received (ack1 came)
 * but its own forwarding stayed silent beyond every series. The
 * silence counts as a failed series toward this hop: 6 units off the
 * budget, then the 7.4 structure takes over - next-best hop if
 * attempts remain, otherwise the definitive fail. */
static void ack2_timeout(mimeshi_node *n, void *ud) {
    mimeshi_pending *p = (mimeshi_pending *)ud;
    if (!p->active || !p->acked1) return;
    printf("[core] node %u: T_ack2 timeout for msg %u (budget %u -> %u)\n",
           n->id, p->msg_id, p->budget_left,
           (uint8_t)(p->budget_left > MC_ACK2_BUDGET_PENALTY ? p->budget_left - MC_ACK2_BUDGET_PENALTY : 0));
    p->budget_left = (uint8_t)(p->budget_left > MC_ACK2_BUDGET_PENALTY ? p->budget_left - MC_ACK2_BUDGET_PENALTY : 0);
    p->acked1 = 0;
    p->try_idx = (uint8_t)(schedule_attempts(p) - 1);   /* the wait consumed the remaining attempts (7.3) */
    schedule_fail(n, p);
}

static void pending_try(mimeshi_node *n, mimeshi_pending *p) {
    if (p->cand_cnt == 0) return;             /* no schedule: nothing to send (defensive) */
    p->next_hop = p->cand[p->try_idx % p->cand_cnt];   /* round-robin over the schedule (7.4) */
    p->pkt[9] = p->budget_left;
    wr16(p->pkt + 10, p->next_hop);           /* next hop in the header (6.1) */
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
int mimeshi_send_data(mimeshi_node *n, uint16_t dst, const uint8_t *payload, uint16_t len, bool ack3,
                      uint16_t dest_lat, uint16_t dest_lon) {
    /* routing (10.2): the first hop follows from the own known graph -
     * a direct neighbour only when its link is confirmed symmetric
     * (10.1), otherwise the computation decides; a destination outside
     * the graph routes via the bridge-node rule when the client
     * supplied its geo, and is refused without geo (no flood). */
    if (len > MC_MAX_PAYLOAD) return -3;
    mimeshi_pending *p = 0;
    for (int i = 0; i < MC_MAX_PENDING; i++) if (!n->pend[i].active) { p = &n->pend[i]; break; }
    if (!p) return -2;
    memset(p, 0, sizeof(*p));
    p->active = 1;
    p->budget_left = MC_BUDGET_INIT;
    /* attempt schedule (7.4): up to 3 Dijkstra-best hops toward dst; the
     * route itself is the first candidate - or the send is refused. */
    p->cand_cnt = schedule_candidates(n, dst, dest_lat, dest_lon, n->id, 0, 0, p->cand);
    if (p->cand_cnt == 0) return -1;           /* not in the graph: no flood (10.2) */
    uint16_t nh = p->cand[0];
    p->msg_id = (uint16_t)n->rand(n);
    p->src = n->id; p->dst = dst; p->prev_hop = n->id; p->next_hop = nh;
    uint8_t *k = p->pkt;
    bool geo = (dest_lat || dest_lon);
    k[0] = (uint8_t)(PKT_DATA | (ack3 ? F_ACK3_REQ : 0) | (geo ? F_DEST_GEO : 0));
    wr16(k + 1, p->msg_id); wr16(k + 3, n->id); wr16(k + 5, dst); wr16(k + 7, n->id);
    k[9] = p->budget_left;                 /* header: budget at offset 9 (section 6.1) */
    wr16(k + 10, nh);                      /* next hop at offset 10 (6.1) */
    uint8_t off = 12;
    if (geo) {                              /* dest geo after the fixed header (6.1, 10.2) */
        wr16(k + 12, dest_lat); wr16(k + 14, dest_lon);
        off = 16;
    }
    memcpy(k + off, payload, len);
    p->pkt_len = (uint16_t)(off + len);
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
            if (dst == n->id && len >= 6) {
                uint8_t rq = pkt[5];     /* what the responder measured on OUR signal (6.2) */
                nb->rev_quality = rq ? quality_ewma(nb->rev_quality ? nb->rev_quality : rq, rq)
                                     : nb->rev_quality;
            }
            if (dst == n->id && quality_ok(nb->quality)) {
                if (!nb->sym) tbl_nbr_added(n, from);  /* delta + table round (4.3) */
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
                if (quality_ok(nb->quality) && !nb->sym) { nb->sym = 1; nb->ack_try = 0; tbl_nbr_added(n, from); }
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
            if (p->active && p->msg_id == msg_id && !p->acked1) {
                p->acked1 = 1;                     /* the own hop succeeded (7.1) */
                mimeshi_neighbor *nb = nbr_find(n, rd16(pkt + 5));
                if (nb) {                               /* confirmed symmetric (4.2, 10.1) */
                    nb->quality = quality_of(rssi);     /* the ack1 itself is the measurement (its direction) */
                    nb->proven_age_us = 0;
                    if (nb->penalty) nb->penalty--;      /* recovery via ordinary traffic (10.4) */
                    if (len >= 8 && pkt[7]) {            /* what the acker measured on OUR data (6.2) */
                        nb->rev_quality = quality_ewma(nb->rev_quality ? nb->rev_quality : pkt[7], pkt[7]);
                    }
                    if (quality_ok(nb->quality)) {
                        if (!nb->sym) tbl_nbr_added(n, rd16(pkt + 5));
                        nb->sym = 1; nb->ack_try = 0;
                    }
                }
                printf("[core] node %u: ACK1 for msg %u\n", n->id, msg_id);
                if (p->prev_hop != n->id) {
                    /* forwarder (7.1, revised Oct 2026): the ack2 goes
                     * out NOW - two hops of progress per ack2 - and the
                     * record closes. */
                    ack2_send(n, msg_id, p->src, p->prev_hop, false, p->budget_left);
                    p->active = 0;
                } else {
                    /* source: the hop is done; ack2 comes back on the
                     * own home channel. T_ack2 (15 s) is the safety
                     * net (7.3). Exception (decided Oct 2026, the
                     * one-hop closing piece): when the own next hop
                     * IS the final destination, its ack1 is the end
                     * confirmation - there is no second hop whose
                     * ack2 the source would wait for. The record
                     * closes here; waiting would burn budget and
                     * drive retries through unrelated neighbours
                     * (measured: T_ack2 timeouts on delivered
                     * one-hop unicasts under load). */
                    if (p->next_hop == p->dst) {
                        printf("[core] node %u: ACK1 from destination for msg %u - done\n",
                               n->id, msg_id);
                        p->active = 0;
                        return;
                    }
                    n->timer(n, MC_T_ACK2_US, ack2_timeout, p);
                }
                return;
            }
        }
        break;
    }
    case PKT_ACK2: {
        /* Ack2 from the next hop (7.1/7.2, revised Oct 2026). Positive
         * = two hops of progress: the next hop received AND its own hop
         * succeeded - the pending record closes. Negative = its
         * forwarding failed after every series; the spent budget
         * travels along (byte 7, 7.4): weigh the own alternatives
         * against what is left, stop when it is gone. */
        uint16_t msg_id = rd16(pkt + 1);
        for (int i = 0; i < MC_MAX_PENDING; i++) {
            mimeshi_pending *p = &n->pend[i];
            if (p->active && p->acked1 && p->msg_id == msg_id) {
                printf("[core] node %u: ACK2 %s for msg %u (budget left %u)\n", n->id,
                       (flags & F_NEG_ACK2) ? "NEGATIVE" : "positive", msg_id,
                       len >= 8 ? pkt[7] : 0);
                p->active = 0;
                if (flags & F_NEG_ACK2) {
                    if (len >= 8 && pkt[7] < p->budget_left) p->budget_left = pkt[7];
                    p->acked1 = 0;
                    p->try_idx = (uint8_t)(schedule_attempts(p) - 1);   /* the plan toward this hop is done */
                    schedule_fail(n, p);          /* exclusion + next plan (7.4) */
                }
                return;
            }
        }
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
    case PKT_STAR_ROUND: {
        star_round_rx(n, rssi, pkt, len);
        break;
    }
    case PKT_DATA: {
        uint16_t msg_id = rd16(pkt + 1), src = rd16(pkt + 3), dst = rd16(pkt + 5), prev = rd16(pkt + 7);
        /* next-hop addressing (6.1): only the hop the sender chose
         * processes the packet; other hearers on the home channel
         * snoop (liveness, quality) - a routed unicast must not
         * multiply. */
        if (rd16(pkt + 10) != n->id) return;
        /* dedup gates the processing (no second delivery, no second
         * forward), but a retry STILL gets its ack1 (7.4, decided Oct
         * 2026): the most likely reason for the retry is a lost ack1,
         * not a lost data packet - silence would make the sender
         * declare the link dead and detour. */
        bool fresh = dedup_fresh(n, msg_id, src);
        if (!fresh) { ack1_send(n, msg_id, src, ch, rssi); return; }
        if (dst == n->id) {
            uint8_t poff = (flags & F_DEST_GEO) ? 16 : 12;   /* geo sits between header and payload (6.1) */
            printf("[core] node %u: DATA from %u: '%.*s\n", n->id, src, (int)(len - poff), (const char *)(pkt + poff));
            ack1_send(n, msg_id, src, ch, rssi);       /* same channel; quality of the data travels along (7.1) */
            /* No ack2 from the destination: the positive ack2 of this
             * final hop goes out by the previous forwarder on OUR
             * ack1 (7.1, revised Oct 2026: two hops per ack2). */
            if (flags & F_ACK3_REQ) {
                /* ack3 as a routed message (6.2): DATA format, payload
                 * = the original msg-id, back toward the source - with
                 * routing, via the first hop computation. */
                uint16_t ah = route_next_hop(n, src);
                if (ah) {
                    uint8_t a[16] = { PKT_ACK3, 0,0, 0,0, 0,0, 0,0, 0, 0,0, 0,0, 0,0 };
                    wr16(a + 1, (uint16_t)n->rand(n)); wr16(a + 3, n->id);
                    wr16(a + 5, src); wr16(a + 7, n->id); a[9] = MC_BUDGET_INIT;
                    wr16(a + 10, ah); wr16(a + 12, msg_id);
                    bound_send(n, (uint8_t)(ah % MC_N_CHANNELS), a, 16, t_data_us(16));
                    ka_called(n, (uint8_t)(ah % MC_N_CHANNELS));   /* own call (4.1) */
                }
            }
            break;
        }
        /* forwarding (10.2, stateless per message): ack the previous
         * hop, then recompute the own first step toward the
         * destination. The budget bounds the chain (9); the next-hop
         * field in the header makes the chain unambiguous (6.1). */
        if (len < 12 || src == n->id) return;    /* malformed, or our own message echoed back */
        uint8_t budget = pkt[9];
        ack1_send(n, msg_id, src, ch, rssi);      /* the exchange of this hop (7.1) */
        if (budget == 0) return;                  /* budget spent: the chain stops here */
        uint16_t gl = 0, go = 0;
        if (flags & F_DEST_GEO) { gl = rd16(pkt + 12); go = rd16(pkt + 14); }   /* geo stays in the header (10.2) */
        /* prev-hop exclusion at the FIRST forward choice (decided Oct
         * 2026, measured under load): the hop the message came from
         * is not a candidate for the next one. The known graph is
         * local and incomplete, so two nodes can each other's best
         * first hop toward the destination (measured: 279<->285
         * bounced one message 166 times, kept alive by the dedup
         * pass-through, only broken by a 15 s T_ack2). The retry
         * machinery of 7.4 applies this exclusion from its second
         * series onward; here it prevents the two-hop loop from ever
         * starting. The source itself stays reachable as a fallback:
         * when the exclusion leaves no candidate, the hop simply does
         * not forward (the negative ack2 / failure report of 10.5
         * follows later). */
        {
            mimeshi_pending *p = 0;
            for (int i = 0; i < MC_MAX_PENDING; i++) if (!n->pend[i].active) { p = &n->pend[i]; break; }
            if (!p) return;                       /* no slot: cannot forward (step 3 sends the negative ack2) */
            memset(p, 0, sizeof(*p));
            p->budget_left = (uint8_t)(budget - 1);
            /* attempt schedule (7.4): up to 3 Dijkstra-best hops toward
             * dst; the collector never proposes the previous hop (the
             * 10.2 prev-hop exclusion is thereby part of every
             * candidate, not only the first). active goes on only
             * after the check - an unforwardable message must not
             * leak its pending slot. */
            p->cand_cnt = schedule_candidates(n, dst, gl, go, prev, 0, 0, p->cand);
            if (p->cand_cnt == 0) return;         /* no routable path: the hop fails (negative ack2 / report follow in step 3) */
            p->active = 1;
            p->msg_id = msg_id; p->src = src; p->dst = dst; p->prev_hop = prev;
            memcpy(p->pkt, pkt, len); p->pkt_len = len;
            p->pkt[7] = (uint8_t)(n->id & 0xFF); p->pkt[8] = (uint8_t)(n->id >> 8);  /* prev hop = us */
            printf("[core] node %u: FORWARD msg %u from %u to %u via %u\n", n->id, msg_id, src, dst, p->cand[0]);
            pending_try(n, p);
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
    /* star round (4.3): the uniform convergence cadence - the first
     * round after a random phase so simultaneous boots do not sync. */
    n->star_own_ctr = 0;
    n->star_tick = 0;
    n->star_round_sched = 1;
    n->timer(n, (n->rand(n) % MC_STAR_ROUND_TICKS) * MC_KEEPALIVE_TICK_US,
             star_round_tick, 0);
}
