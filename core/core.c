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

/* ---------------- airtime (SF7, 62.5 kHz, CR4/5) ----------------
 * Fixed values from the table in 14.1. The core passes the airtime to
 * tx() so the platform layer can schedule the transmit window; the
 * functions are deliberately simple (three classes) instead of an exact
 * LoRa airtime formula. */
static uint64_t t_ack_us(void)        { return 78000ull; }
static uint64_t t_hello_us(void)      { return 70000ull; }
static uint64_t t_data_us(uint16_t l) { return l <= 40 ? 173000ull : l <= 150 ? 459000ull : 869000ull; }

/* ---------------- packet types and formats ----------------
 * Type byte: low nibble = type, high nibble = flags (6.1). All
 * multi-byte fields little-endian (rd16/wr16). Layouts:
 *
 *   HELLO       5 B   type, node-id(2), state version neighbor table(2)  (4.1, 6.3)
 *   HELLO_ACK   7 B   type, destination-id(2) = hello sender,
 *                     sender-id(2), state version(2) — directional:
 *                     the address makes the ack an exact symmetry proof
 *                     (78 ms airtime; 2, 4.2)
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
    PKT_HELLO = 1, PKT_HELLO_ACK, PKT_TABLE_PULL, PKT_TABLE_CHUNK,
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

/* Add a neighbor (from a hello or hello-ack). Full = return 0; the caller
 * continues without the neighbor. A new entry starts as "seen": one
 * direction, sym = 0 (4.2). my_state_ver increments on every mutation:
 * the hello carries that version so neighbors see that a tablepull is
 * needed (4.3). */
static mimeshi_neighbor *nbr_add(mimeshi_node *n, uint16_t id) {
    for (int i = 0; i < MC_MAX_NEIGHBORS; i++)
        if (!n->nbr[i].alive) {
            n->nbr[i].alive = 1; n->nbr[i].id = id;
            n->nbr[i].sym = 0; n->nbr[i].ack_try = 0;
            n->my_state_ver++;
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
typedef struct { uint8_t ch; uint16_t len; uint64_t airtime; uint8_t pkt[10 + MC_MAX_PAYLOAD]; } bound_ctx;
static bound_ctx bctx[MC_BOUND_POOL]; /* pool per transmission; small and fixed */
static uint32_t bctx_drops;          /* full pool = defect, never silent */

static void bound_fire(mimeshi_node *n, void *ud) {
    bound_ctx *c = (bound_ctx *)ud;
    n->tx(n, c->ch, c->pkt, c->len, c->airtime);
    c->len = 0; /* slot free */
}

/* Transmit on channel ch after a switch. Drop the transmission if the
 * pool is full — with the right pool size that is a defect, so with
 * counter and message, never silent. */
static void bound_send(mimeshi_node *n, uint8_t ch, const uint8_t *pkt, uint16_t len, uint64_t airtime) {
    for (int i = 0; i < MC_BOUND_POOL; i++) {
        if (bctx[i].len == 0) {
            bctx[i].ch = ch; bctx[i].len = len; bctx[i].airtime = airtime;
            memcpy(bctx[i].pkt, pkt, len);
            n->switch_ch(n, ch, MC_S_SWITCH_US, bound_fire, &bctx[i]);
            return;
        }
    }
    bctx_drops++;
    printf("[core] node %u: bound-pool full, transmission dropped (counter %u)\n", n->id, bctx_drops);
}

/* ---------------- hello (4.1) ---------------- */

/* One hello round: the same packet on all three channels, in fixed
 * order. Reasons (2): neighbors sit randomly on all home channels, and
 * the fixed order gives responders a predictable moment when the sender
 * is listening on its own channel again — their hello-acks go there.
 * Cost: 3 x 5 B with adaptive interval. */
static void hello_round(mimeshi_node *n) {
    uint8_t p[5] = { PKT_HELLO, 0,0,0,0 };
    wr16(p + 1, n->id); wr16(p + 3, n->my_state_ver);
    for (uint8_t c = 0; c < MC_N_CHANNELS; c++) {          /* fixed order */
        bound_send(n, c, p, 5, t_hello_us());
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

/* Keepalive hello on channel c (4.1): exactly one hello, not a round —
 * only the channel that has been silent for an hour. With the usual
 * random phase, so quiet regions desynchronize on their own (4.1,
 * link test). */
static void ka_fire(mimeshi_node *n, void *ud) {
    uint8_t c = (uint8_t)(size_t)ud;
    uint8_t p[5] = { PKT_HELLO, 0,0,0,0 };
    wr16(p + 1, n->id); wr16(p + 3, n->my_state_ver);
    bound_send(n, c, p, 5, t_hello_us());
    ka_called(n, c);                        /* counter only runs again from this call */
}

/* The keepalive time base: the core has no clock, so it counts silence
 * in ticks. If MC_KEEPALIVE_TICKS of silence (= T_keepalive) have passed
 * on channel c, exactly one hello follows on c, scheduled with jitter.
 * Reschedules itself; started in mimeshi_start(). During the startup
 * phase the round rhythm keeps resetting the counter — keepalive and
 * round therefore do not disturb each other. */
static void ka_tick(mimeshi_node *n, void *ud) {
    (void)ud;
    for (uint8_t c = 0; c < MC_N_CHANNELS; c++) {
        if (n->ka_idle[c] < MC_KEEPALIVE_TICKS) n->ka_idle[c]++;
        if (n->ka_idle[c] >= MC_KEEPALIVE_TICKS) {
            n->ka_idle[c] = 0;              /* counter reset on scheduling: one hello, no storm */
            n->timer(n, hello_jitter(n), ka_fire, (void *)(size_t)c);
        }
    }
    n->timer(n, MC_KEEPALIVE_TICK_US, ka_tick, 0);
}

/* Deliver a hello-ack on the HOME channel of the hello sender (2): the
 * sender is there after its round and catches all answers within the
 * response window. One exchange, then home.
 *
 * Directional format (7 B): the destination is the hello sender. Only
 * because of that does the ack prove symmetry — the responder received
 * OUR hello (its ack is a reaction to it) and its ack reached us
 * (4.2, 10.1). That is the proof channel of the hello process:
 * available before the first unicast. Our own state version travels
 * along for the pull decision (4.3); eavesdroppers process the sender
 * as a minimal hello ("seen"). */
static void hello_ack_fire(mimeshi_node *n, void *ud) {
    uint16_t to = (uint16_t)(uintptr_t)(size_t)ud;
    uint8_t p[7] = { PKT_HELLO_ACK, 0,0, 0,0, 0,0 };
    wr16(p + 1, to);                       /* destination: the hello sender */
    wr16(p + 3, n->id);
    wr16(p + 5, n->my_state_ver);
    bound_send(n, (uint8_t)(to % MC_N_CHANNELS), p, 7, t_ack_us()); /* to HOME of the sender */
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
        /* update or add the neighbor, then schedule an ack. quality is
         * the RSSI measurement of this reception; the state version of
         * the neighbor is tracked for future tablepulls (4.3);
         * last_seen is still a stub for now — real aging policy (4.4)
         * follows. */
        uint16_t from = rd16(pkt + 1);
        mimeshi_neighbor *nb = nbr_find(n, from);
        if (!nb) nb = nbr_add(n, from);
        if (nb) {
            nb->last_seen = 0;
            nb->quality = quality_of(rssi);
            nb->state_ver = rd16(pkt + 3);
        }
        hello_ack_wait(n, from, rssi);
        break;
    }
    case PKT_HELLO_ACK: {
        /* Directional ack (7 B): destination = the original hello
         * sender.
         *  - Addressed to US: the responder received our hello and
         *    this ack reached us — both directions worked at that
         *    moment: symmetry proven (4.2, 10.1). This is the proof
         *    channel of the hello process, available before the first
         *    unicast.
         *  - Addressed to someone else (snooping, 2): the sender is
         *    processed as a minimal hello — id, state version (directly
         *    from the source) and the quality measured here — but stays
         *    "seen": one-direction proof, no symmetry. The ack triggers
         *    no new reaction: the type byte serves that purpose. */
        uint16_t dst = rd16(pkt + 1), from = rd16(pkt + 3);
        mimeshi_neighbor *nb = nbr_find(n, from);
        if (!nb) nb = nbr_add(n, from);
        if (nb) {
            nb->last_seen = 0;
            nb->quality = quality_of(rssi);
            nb->state_ver = rd16(pkt + 5);
            if (dst == n->id) { nb->sym = 1; nb->ack_try = 0; }   /* confirmed symmetric (4.2) */
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
                if (nb) { nb->sym = 1; nb->ack_try = 0; }  /* confirmed symmetric (4.2, 10.1) */
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
                mimeshi_switch_fn sw, mimeshi_rand_fn rnd, void *user) {
    memset(n, 0, sizeof(*n));
    n->id = id; n->home_ch = (uint8_t)(id % MC_N_CHANNELS);
    n->my_state_ver = 1; n->hello_interval_us = MC_HELLO_START_US;
    n->hello_jitter_us = MC_HELLO_JITTER_US;
    n->tx = tx; n->timer = timer; n->switch_ch = sw; n->rand = rnd; n->user = user;
}

/* First hello round immediately (fast map building, 4.1), then the
 * adaptive timer with random phase. Idempotent for the rest of the
 * core: after start the hello runs on the timer; do not repeat
 * mimeshi_start() itself. */
void mimeshi_start(mimeshi_node *n) {
    hello_round(n);
    /* random phase: prevents synchronous rounds on simultaneous startup (4.1) */
    n->timer(n, n->hello_interval_us + hello_jitter(n), hello_tick, 0);
    /* keepalive tick: the time base for the per-channel keepalive (4.1) */
    n->timer(n, MC_KEEPALIVE_TICK_US, ka_tick, 0);
}
