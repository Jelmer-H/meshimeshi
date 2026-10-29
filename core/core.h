/*
 * Meshimeshi — protocol core, public API (core.h)
 * Design: docs/design.md (section 13b: shared core for firmware and simulator).
 *
 * The core has no own clock, radio or RNG: everything arrives via the
 * callbacks in mimeshi_init(). The simulator provides them (virtual radio);
 * on firmware the platform layer provides them. No dynamic allocation;
 * all buffers are fixed.
 *
 * The node struct lives entirely in the header so nodes can be reserved
 * statically (no heap). Internal fields are read-only for the platform
 * layer.
 */
#ifndef MCORE_H
#define MCORE_H

#include <stdint.h>
#include <stdbool.h>

/* ---------------- configuration (firmware constants) ---------------- */
#define MC_N_CHANNELS    3
#define MC_MAX_NEIGHBORS 64
#define MC_MAX_PENDING   64            /* was 16: the 480-node scale run showed
                                          the forwarding hops on tile crossings run
                                          out of slots under load (554 of 573 lost
                                          messages were the silent no-slot drop);
                                          64 records x ~250 B = 16 kB - fits the
                                          13.2 budget (the freed 4 kB chunk-reassembly
                                          space covers it) */
#define MC_MAX_DEDUP    128
#define MC_MAX_PAYLOAD  230
#ifndef MC_MAP_NODES
#define MC_MAP_NODES    256             /* bounded 2-hop map (13.2, 13.3) */
#endif   /* the sim can override at build time for pool-pressure tests */
#define MC_MAX_LINKS   1536            /* link store: links between map nodes (13.2) */
#define MC_T_SYM_US      21600000000ull  /* 6 h: validity of confirmed symmetry (4.2) */
#define MC_T_EXPIRE_TICKS 257u           /* ~4 h in proof ticks: neighbour-table expiry (4.4) */
#define MC_T_ACTIVE_TICKS 771u           /* ~12 h: map entry stays fully usable (4.4); ~2
                                            re-shares, so a live node with at least one
                                            neighbour never falls below this */
#define MC_SYM_MARGIN_US  7200000000ull  /* 2 h flush margin (4.2) */

/* ---------------- internal types (visible for fixed allocation) ---- */
typedef struct {
    uint16_t id;
    uint8_t  quality;      /* derived from RSSI, updated per hello/proof list */
    uint8_t  rev_quality;  /* quality the NEIGHBOUR measured on our signal
                              (its hello-ack/ack1 quality byte, 6.2);
                              0 = not yet known. EWMA-smoothed. */
    uint8_t  ack_try;      /* counter per neighbor (4.1): every identifying
                              transmission of the neighbor counts; ack
                              attempts at counter values 1, 2, 4, 8, 16,
                              then on average 1 in 16; reset on
                              confirmation or renewed contact */
    uint32_t last_seen;
    uint64_t proven_age_us;  /* age of the reverse-direction evidence
                               ("last proven", 4.2): refreshed on a hello-ack
                               addressed to us, a proof list in which we
                               appear, an ack1 on our data; aged in the
                               proof tick; >= T_sym - margin = flush */
    uint8_t  alive;
    uint8_t  sym;           /* 1 = confirmed symmetric (hello-ack addressed
                               to us, proof list in which we appear, ack1
                               on our data, or table chunk naming us);
                               0 = only seen, one direction (4.2, 10.1) */
    uint8_t  adv_class;     /* the link byte last advertised for this
                              neighbour in the own star (both direction
                              nibbles) - hysteresis: only a >= 2-step
                              class change on either side re-publishes,
                              sub-threshold EWMA movement does not (4.3) */
    uint8_t  penalty;       /* local failure counter (10.4): +1 per failed
                              series toward this neighbour, -1 per ack1,
                              floor 0; raises the own-edge route cost.
                              Never transmitted. */
} mimeshi_neighbor;

typedef struct {
    uint8_t  active, tries_left, budget_left;
    uint8_t  acked1;        /* ack1 of the own next hop is in (7.1): a
                              forwarder now sends its ack2 and closes;
                              the source enters the T_ack2 wait (7.3) */
    uint8_t  cand_cnt;      /* candidates of the attempt plan (7.4): 1..3
                              next hops, each the best hop of a full
                              Dijkstra run without the earlier ones */
    uint8_t  try_idx;       /* attempt counter over the plan; cand[try_idx
                              % cand_cnt] is the next hop of the attempt */
    uint8_t  excl_cnt;      /* failed next hops of THIS message (7.4:
                              cumulative exclusion per message, no
                              protocol field - the penalty 10.4 carries
                              the long-term lesson) */
    uint16_t msg_id, src, dst, prev_hop, next_hop;
    uint16_t cand[3];      /* the attempt plan, best first (7.4) */
    uint16_t excl[3];       /* the failed hops (7.4) */
    uint8_t  pkt[16 + MC_MAX_PAYLOAD];
    uint16_t pkt_len;
} mimeshi_pending;

/* Dedup entry with insertion order (14, layer 3): the insertion
 * counter makes the fixed-size cache a FIFO - the oldest entry is the
 * replacement victim when the cache is full. Measured under load: a
 * permanent "full = receive anyway" turned dedup off after 128
 * messages, and a routing ping-pong forwarded one message 166 times. */
typedef struct { uint16_t msg_id, src; uint16_t seq; } mimeshi_dedup;

/* 2-hop map entry (4.3, 10.1): a node we know about indirectly (from
 * table chunks) or directly. Geo in the 16-bit grid of section 5;
 * (0,0) = invalid. Age bookkeeping without a clock: arrived_tick is
 * the local tick counter of the last refresh; age_ticks is the on-air
 * age (in proof ticks, ~56 s) at that moment. Effective age =
 * (tick - arrived_tick) + age_ticks, in proof ticks; 255 (~4 h) =
 * expired (4.4). */
typedef struct {
    uint16_t id;
    uint16_t geo_lat, geo_lon;
    uint8_t  quality;        /* local measurement of this node's signal,
                                refreshed on every identifying reception
                                of it (snooping, chunks); routing-weight
                                material, never forwarded */
    uint8_t  adv_q;        /* entry class byte (both directions) exactly
                              as the origin advertised it in its star
                              (4.3 storage-as-received): reproduced
                              byte-exactly on every hop, hashed by the
                              content hash. 0 = not received yet. */
    uint16_t arrived_tick;
    uint8_t  age_ticks;
    uint8_t  star_tick;   /* star round (4.5): the origin's heartbeat
                            tick as it reached us - carried in the age
                            byte of the canonical star form. Separate
                            from age_ticks: age_ticks keeps its 4.3
                            meaning (travelled age for route weighing
                            and map eviction); star_tick is liveness.
                            0 = never seen via a star round (a star
                            from a plain 4.3 chunk has no tick yet). */
    uint8_t  alive;
    uint16_t star_fwd_tick;  /* star round (4.3): tick when this star
                                last entered the content queue (0 =
                                never; same 16-bit time base as
                                arrived_tick, wrap-exact) */
    uint8_t  star_fwd_hash;  /* star round (4.3): content hash of the
                                star as last CARRIED. Queuing is driven
                                by content: a star re-enters the queue
                                when its advertised content (hash over
                                the canonical bytes, tick excluded)
                                differs from this - not when a
                                bookkeeping tick merely moved */
    uint8_t  star_fwd_bits;  /* star round (4.3): carried-on-channel bits
                                (bit c = carried on channel c). A star
                                leaves the content queue only at 7 (all
                                three channels, 4.3 rotation); a content
                                change resets the bits. Set in one blow
                                by a full-spread (backlog) round */
    uint8_t  star_miss_ctr;  /* star round (4.5): missed-list damping -
                                fetch attempts for this star that yielded
                                no new content; reset on a real update */
    uint16_t star_wanted;    /* star round (4.3): local tick when a
                                neighbour last requested this star - an
                                explicit ask overrides the forward
                                pruning for a few intervals (the missed
                                list is the net for what the gradient
                                check gets wrong) */
    uint16_t star_tick_seen; /* liveness axis (4.4): local tick when this
                                star's origin heartbeat tick last ADVANCED
                                - content or a derivable match. Freshness
                                and liveness are independent: an echo keeps
                                the age fresh but cannot advance the tick;
                                a star whose tick stalls for
                                MC_STAR_DEAD_TICKS is first de-weighted in
                                routing (the age multiplier), then evicted
                                - a dead origin's star disappears everywhere
                                within the threshold, whatever the forwarded
                                copies claim */
} mimeshi_mapnode;

/* A link between two map nodes with the advertised quality class (3
 * bits) per direction (4.3). Endpoints normalized a < b; alive = 0
 * marks a free slot. */
typedef struct {
    uint16_t a, b;
    uint8_t  q;             /* (class a->b) << 4 | class b->a */
    uint8_t  alive;
} mimeshi_link;

struct mimeshi_node;
typedef struct mimeshi_node mimeshi_node;

/* ---------------- hardware interface -------------------------------- */
typedef void (*mimeshi_tx_fn)(mimeshi_node *n, uint8_t channel, const uint8_t *pkt,
                            uint16_t len, uint64_t airtime_us);
typedef void (*mimeshi_timer_fn)(mimeshi_node *n, uint64_t delay_us,
                               void (*cb)(mimeshi_node *, void *), void *ud);
typedef void (*mimeshi_switch_fn)(mimeshi_node *n, uint8_t channel, uint64_t s_us,
                                void (*done)(mimeshi_node *, void *), void *ud);
typedef uint32_t (*mimeshi_rand_fn)(mimeshi_node *n);

/* CAD, channel activity detection (2): the platform layer answers
 * whether the channel is free at this moment (carrier sense). The core
 * asks before every transmission that is not a direct reply: if busy,
 * it waits with random backoff on the channel and asks again. Ack1 is
 * the exception (7.1): it is a direct reply within the ongoing
 * exchange, the partner is already waiting there, so it goes without a
 * check. Not an RTS/CTS system against hidden nodes (open point 8):
 * CAD only avoids transmissions onto an occupied channel.
 * NULL = no CAD available (stub platform): transmit as before. */
typedef bool (*mimeshi_channel_free_fn)(mimeshi_node *n, uint8_t channel);

/* Star-round missed request (4.5): a star we do not have (or whose
 * digest fits nothing) as advertised by a neighbour. Travelled in the
 * missed area of our own round, addressed to that neighbour. The
 * damping (4.5): after 1-2 attempts without new content the request
 * falls back to fresh-beats-old weighing instead of re-asking every
 * interval. */
#define MC_STAR_MISS_MAX 12u
typedef struct {
    uint16_t nb;            /* the neighbour that advertised the star */
    uint16_t star;          /* the star id we are missing */
    uint8_t  tries;         /* attempts so far; dropped after 2 (4.5) */
    uint8_t  alive;
} mimeshi_star_miss;

/* ---------------- node ---------------------------------------------- */
struct mimeshi_node {
    uint16_t id;
    uint8_t  home_ch;                 /* id % MC_N_CHANNELS */
    uint16_t local_tbl_ctr;           /* internal table counter (glossary):
                                         drives push-window bundling (4.3);
                                         never transmitted */
    mimeshi_neighbor nbr[MC_MAX_NEIGHBORS];
    mimeshi_pending  pend[MC_MAX_PENDING];
    mimeshi_dedup    dedup[MC_MAX_DEDUP];
    uint64_t hello_interval_us;       /* adaptive: 30 s -> 1 h */
    uint64_t hello_jitter_us;         /* random phase per round; 0 = no jitter.
                                         Default MC_HELLO_JITTER_US; the
                                         simulation can set it before mimeshi_start */
    uint8_t  ka_idle[MC_N_CHANNELS];  /* silence counter per channel in
                                         keepalive ticks (4.1): every own
                                         identifying transmission on channel c
                                         sets it to 0; after 64 ticks
                                         (= T_keepalive, core.c) of silence the
                                         next proof list on c is pulled
                                         forward (4.2) */
    uint8_t  proof_wait[MC_N_CHANNELS];/* countdown to the flush moment
                                         per channel (4.2), staggered 20
                                         min apart; restarts at 1 h on
                                         every fire: the hourly cap */
    uint16_t geo_lat, geo_lon;        /* own geo in the 16-bit grid (5);
                                         (0,0) = none (yet) */
    uint32_t tick;                    /* local time base: proof ticks (~56 s)
                                         since start (4.4); starts at 1 */
    uint16_t dedup_seq;               /* FIFO insertion counter of the dedup cache (14) */
    uint8_t  tbl_dirty;               /* own-star content change pending (4.3) */
    uint8_t  star_round_sched;        /* star-round timer running (4.3) */
    uint16_t ev_taboo[256];          /* eviction taboo (4.3): the set of judged-
                                        and-rejected star IDs (admission losses
                                        and evictions) - a catalogue mismatch
                                        for a tabooed star creates NO miss
                                        request: the rejection was a policy
                                        choice, requesting it back is a cycle
                                        under pool pressure. No ring overwrite:
                                        a rejection stands until the star is
                                        actually stored again (the pool
                                        composition changes slowly). 256 =
                                        the map pool size: every possible
                                        rejection fits. */
    uint8_t  ev_taboo_idx;            /* ring write position */
    uint8_t  star_backlog;           /* 6e: content work remained after the last
                                        round (stars queued or not fully carried) -
                                        the quiet-hour acceleration halves the
                                        interval while this is set and the
                                        channels are quiet; cleared at rest */
    uint8_t  star_rot_ctr;           /* rotation counter (4.3): the round
                                        transmits on channel
                                        (home + ctr) mod 3 at rest, and
                                        on all three under backlog; the
                                        guaranteed own-star round (tick)
                                        fires when the rotation lands on
                                        the own home channel */
    mimeshi_star_miss star_miss[MC_STAR_MISS_MAX];  /* missed-list (4.5):
                                         bounded, damped, per neighbour */
    uint8_t  star_own_ctr;            /* star rounds since the own star last
                                         traveled in full; the guaranteed
                                         own-star round fires when this
                                         reaches the hour bound (4.5) */
    uint8_t  star_tick;               /* heartbeat tick of the own star (4.5):
                                         +1 at every guaranteed own-star
                                         round; rides in the age byte and
                                         drives the derivable digest */
    mimeshi_mapnode map[MC_MAP_NODES];  /* 2-hop map (10.1) */
    mimeshi_link   lnk[MC_MAX_LINKS];  /* links between map nodes (4.3) */
    uint8_t  trace;                   /* message tracing (measured runs):
                                         1 = log every send/receive/ack of
                                         DATA messages on this node. Set via
                                         mimeshi_set_trace; the simulator
                                         uses it for low-rate forensics runs. */
    mimeshi_tx_fn     tx;
    mimeshi_timer_fn  timer;
    mimeshi_switch_fn switch_ch;
    mimeshi_rand_fn   rand;
    mimeshi_channel_free_fn channel_free;  /* CAD (carrier sense); NULL = off */
    void *user;                       /* own data of platform/simulator */
};

/* ---------------- API ----------------------------------------------- */
/* Initializes the node (zeroing + callbacks). The id determines the home channel. */
void mimeshi_init(mimeshi_node *n, uint16_t id, mimeshi_tx_fn tx, mimeshi_timer_fn timer,
                mimeshi_switch_fn sw, mimeshi_rand_fn rnd,
                mimeshi_channel_free_fn cad, void *user);

/* Message tracing (forensics runs): 1 = this node logs every DATA
 * send/receive and every ack1/ack2 of its messages. Cheap when off (one
 * byte test per event). */
void mimeshi_set_trace(mimeshi_node *n, uint8_t on);

/* First hello round immediately, then startup rounds with random phase
 * (jitter). After the startup phase the hello no longer exists:
 * channel liveness is carried by traffic plus the proof list with the
 * keepalive role (4.1, 4.2). */
void mimeshi_start(mimeshi_node *n);

/* Unicast to dst. 0 = accepted, -1 = dst not in the map and no
 * usable dest geo (no flood, 10.2), -2 = no free pending slot,
 * -3 = payload too large. dest_lat/dest_lon: the destination geo in
 * the 16-bit grid of section 5 (0,0) = unknown - the client supplies
 * it; the source attaches it to the header when known (10.2), so
 * every hop can apply the bridge-node rule when the destination lies
 * outside its own graph. */
int  mimeshi_send_data(mimeshi_node *n, uint16_t dst, const uint8_t *payload,
                     uint16_t len, bool want_ack3,
                     uint16_t dest_lat, uint16_t dest_lon);

/* Own geo in the 16-bit grid of section 5 (0..65535 each); (0,0) =
 * invalid/none. Travel with every table chunk header (4.3). Call
 * before mimeshi_start(); a later call is a mutation (geo change)
 * and feeds the table push. */
void mimeshi_set_geo(mimeshi_node *n, uint16_t lat, uint16_t lon);

/* Packet received (from radio or simulator). ch = channel of reception. */
void mimeshi_rx(mimeshi_node *n, uint8_t channel, int8_t rssi_db,
              const uint8_t *pkt, uint16_t len);

/* ---- star canonicalisation and digest (4.5, proposal) --------------- */
/* Buffer bound for one serialised star: entry (9 B) + max 64 links
 * (3 B each) — the largest possible star (4.5: 64 links at the
 * neighbor table limit). */
#define MC_STAR_BUF (9 + 3 * 64)

/* Serialise the star of node `center` as known by this node: entry (id,
 * both quality classes, age, geo) + its links, each (peer id, classes
 * per direction), SORTED by peer id - the canonical byte form of 4.5.
 * age_off_out (optional) receives the offset of the age byte (the
 * heartbeat tick lives there). Returns the length, 0 = star unknown or
 * maxlen too small. */
uint16_t mimeshi_star_serialize(const mimeshi_node *n, uint16_t center,
                               uint8_t *buf, uint16_t maxlen,
                               uint16_t *age_off_out);

/* 1-byte content hash over the canonical star bytes EXCLUDING the age
 * byte (4.5). */
uint8_t mimeshi_star_content_hash(const uint8_t *star, uint16_t len,
                                  uint16_t age_off);

/* Catalogue digest (4.5): (content_hash + tick) mod 256 - derivable. */
uint8_t mimeshi_star_digest(uint8_t content_hash, uint8_t tick);

/* Heartbeat match (4.5): does `received` fit content_hash at
 * stored_tick + k, k in 0..k_max? Returns the tick advance (0 = same
 * tick, 1..k_max = advanced), -1 = no fit (content change or unknown). */
int mimeshi_star_tick_match(uint8_t received, uint8_t content_hash,
                           uint8_t stored_tick, uint8_t k_max);

#endif /* MCORE_H */
