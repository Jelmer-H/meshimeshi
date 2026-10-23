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
#define MC_MAX_PENDING   16
#define MC_MAX_DEDUP    128
#define MC_MAX_PAYLOAD  230
#define MC_MAP_NODES    256             /* bounded 2-hop map (13.2, 13.3) */
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
    uint8_t  adv_class;     /* last advertised quality class (3 bits,
                               hysteresis >= 2 steps, 4.3) */
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
    uint8_t  alt_left;      /* remaining alternative-hop series (7.4: at
                              most 3 neighbours per hop, so 2 after the
                              first choice) */
    uint8_t  excl_cnt;      /* failed next hops of THIS message (7.4:
                              cumulative exclusion per message, no
                              protocol field - the penalty 10.4 carries
                              the long-term lesson) */
    uint16_t msg_id, src, dst, prev_hop, next_hop;
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
    uint8_t  quality;        /* advertised quality of the link to its owner (4.3) */
    uint16_t arrived_tick;
    uint8_t  age_ticks;
    uint8_t  alive;
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
    uint8_t  chunk_nr;                /* next table chunk number (4.3) */
    uint16_t dedup_seq;               /* FIFO insertion counter of the dedup cache (14) */
    uint8_t  chunk_cursor;             /* table position where the next chunk
                                          continues: tables larger than one
                                          chunk cycle through the pushes (4.3) */
    uint8_t  tail_cursor;               /* map position where the next tail
                                          chunk continues (4.3): the 2-hop
                                          entries cycle through the
                                          periodic re-shares */
    uint8_t  tbl_dirty;               /* prefix mutation pending (4.3) */
    uint8_t  tbl_push_sched;          /* push window/period timer running */
    mimeshi_mapnode map[MC_MAP_NODES];  /* 2-hop map (10.1) */
    mimeshi_link   lnk[MC_MAX_LINKS];  /* links between map nodes (4.3) */
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

#endif /* MCORE_H */
