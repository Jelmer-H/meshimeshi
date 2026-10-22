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
#define MC_T_SYM_US      21600000000ull  /* 6 h: validity of confirmed symmetry (4.2) */
#define MC_SYM_MARGIN_US  7200000000ull  /* 2 h flush margin (4.2) */
#define MC_T_ACK2_US      7000000ull    /* ack2 wait window at the sender (7.3, proposal) */
#define MC_MAX_ROUTE_NBR  3             /* candidates per hop: 3 x 2 = 6 attempts (7.4) */
#define MC_RETRY_TX        2             /* transmit attempts per candidate (7.4) */
#define MC_ACK2_CHARGE     6             /* budget cost of a failed sub-route, charged at
                                            the node receiving the negative ack2 (7.4/9) */
#define MC_MAX_KNOWN     256            /* known map entries (10.1) */
#define MC_CHUNK_PERIOD_US 120000000ull /* 2 min: the chunk check tick (4.3) */
#define MC_CHUNK_REPEAT_TICKS 10         /* slow tail repetition: slices every
                                            ~20 min even in a stable net (4.3) */
#define MC_CHUNK_SLICES     2             /* slices per repetition in the stable phase (4.3) */
#define MC_CHUNK_MUT_MIN_TICKS 3          /* min ticks between mutation pushes (4.3): the
                                            short window still bundles, but a growing map
                                            cannot chain pushes back-to-back */
#define MC_CHUNK_ENTRIES 8              /* entries per chunk; with links an entry is
                                            up to 8 + 3*MC_CHUNK_LINKS B (4.3) */
#define MC_CHUNK_LINKS    10            /* links per entry (4.3): the links of the
                                            node follow directly after its entry. Must
                                            cover the real max node degree: a cap below
                                            it structurally limits the learned graph
                                            (measured: 5 cut off 19 of 40 nodes) */
#define MC_GEO_UNKNOWN   0x7fff         /* int16 sentinel: geo not known (10.2) */

/* ---------------- internal types (visible for fixed allocation) ---- */
typedef struct {
    uint16_t id;
    uint8_t  quality;      /* derived from RSSI, updated per hello/proof list */
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
    int16_t  geox, geoy;    /* position of the neighbor (10.2), from table
                              chunks; MC_GEO_UNKNOWN = not known */
    uint8_t  penalty;       /* local penalty on failures (10.4): +1 per failed
                              attempt, decays on success */
    uint8_t  alive;
    uint8_t  sym;           /* 1 = confirmed symmetric (hello-ack addressed
                               to us, proof list in which we appear, ack1
                               on our data, or table chunk naming us);
                               0 = only seen, one direction (4.2, 10.1) */
} mimeshi_neighbor;

/* Known map entry (10.1): node X, learned via the entry of another node.
 * The map has no hop limit: every chunk entry a node receives lands here,
 * wherever it came from. Capped at MC_MAX_KNOWN; on overflow the entries
 * with the best score stay -- quality toward us, freshness, and hub role
 * (how many of our entries travel via this one). The score is local and
 * cheap: computable from the stored entry without airtime. */
typedef struct {
    uint16_t id, via;       /* X, and the node whose chunk carried X */
    uint8_t  quality;       /* advertised quality of X at the sender */
    uint8_t  score;         /* eviction score (10.1): quality + freshness +
                                hub role, updated on insert/refresh */
    int16_t  geox, geoy;    /* position of X (10.2); MC_GEO_UNKNOWN = not known */
    uint8_t  alive;
    uint8_t  sym;           /* reserved: hop selection demands a confirmed
                              direct neighbour (10.1) */
    uint8_t  nlinks;        /* links of X as learned from chunks (4.3): the
                              node-centric entries carry their links, so the
                              receiver can compute paths (10.2), not just guess */
    uint16_t link[MC_CHUNK_LINKS];  /* the other endpoint of each link */
    uint8_t  lq[MC_CHUNK_LINKS];    /* link quality (both directions averaged) */
} mimeshi_2hop;

typedef struct {
    uint8_t  active, state;  /* state: 0 = attempts running, 1 = ack1 done,
                                waiting for ack2 (7.1/7.3) */
    uint8_t  tries_left, series;   /* attempts per neighbor / neighbor index
                                in the series (max MC_MAX_ROUTE_NBR, 7.4) */
    uint16_t msg_id, src, dst, prev_hop, next_hop, last_hop;
    uint8_t  budget_left;
    uint8_t  fwd;            /* 1 = forwarding record (7.2): after the own ack1,
                                ack2 goes to prev_hop */
    uint8_t  no_report;      /* 1 = failed-delivery report itself (10.5): never
                                generates a new report */
    int16_t  dst_geox, dst_geoy;   /* destination geo for greedy (10.2);
                                     MC_GEO_UNKNOWN = not known */
    uint8_t  pkt[14 + MC_MAX_PAYLOAD];  /* header incl. optional source geo (6.1) */
    uint16_t pkt_len;
} mimeshi_pending;

typedef struct { uint16_t msg_id, src; } mimeshi_dedup;

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

/* ---------------- node ---------------------------------------------- */
struct mimeshi_node {
    uint16_t id;
    uint8_t  home_ch;                 /* id % MC_N_CHANNELS */
    uint16_t local_tbl_ctr;           /* internal table counter (glossary):
                                         drives push-window bundling (4.3);
                                         never transmitted */
    int16_t  geox, geoy;              /* own position (10.2); MC_GEO_UNKNOWN =
                                         not set; the simulator sets it before
                                         start, firmware from GPS */
    mimeshi_neighbor nbr[MC_MAX_NEIGHBORS];
    mimeshi_2hop    known[MC_MAX_KNOWN]; /* known map (10.1), via chunks and snooping */
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
    uint32_t chunk_ctr_last;          /* local table counter at the last
                                         chunk push (4.3): mutation trigger */
    uint8_t  chunk_ticks;              /* ticks since the last push (4.3):
                                         the slow tail repetition counter */
    uint8_t  chunk_rr;                  /* round-robin cursor over the known map
                                         (4.3): every entry gets its turn */
    uint8_t  proof_wait[MC_N_CHANNELS];/* countdown to the flush moment
                                         per channel (4.2), staggered 20
                                         min apart; restarts at 1 h on
                                         every fire: the hourly cap */
    mimeshi_tx_fn     tx;
    mimeshi_timer_fn  timer;
    mimeshi_switch_fn switch_ch;
    mimeshi_rand_fn   rand;
    void *user;                       /* own data of platform/simulator */
};

/* ---------------- API ----------------------------------------------- */
/* Initializes the node (zeroing + callbacks). The id determines the home channel. */
void mimeshi_init(mimeshi_node *n, uint16_t id, mimeshi_tx_fn tx, mimeshi_timer_fn timer,
                mimeshi_switch_fn sw, mimeshi_rand_fn rnd, void *user);

/* First hello round immediately, then startup rounds with random phase
 * (jitter). After the startup phase the hello no longer exists:
 * channel liveness is carried by traffic plus the proof list with the
 * keepalive role (4.1, 4.2). */
void mimeshi_start(mimeshi_node *n);

/* Unicast to dst. 0 = accepted, -1 = dst not in the map and no geo known
 * (10.2: do not flood, wait for the map), -2 = no free pending slot,
 * -3 = payload too large. The destination geo comes from the own map. */
int  mimeshi_send_data(mimeshi_node *n, uint16_t dst, const uint8_t *payload,
                     uint16_t len, bool want_ack3);

/* Unicast to dst with an explicit destination geo from the client (11):
 * the end user points at a map (a city, an area) and gives a node ID.
 * The message travels greedy toward the geo; near the destination a
 * local node knows the target in its table and routes on the ID. The
 * geo needs not be exact. MC_GEO_UNKNOWN = no geo given: falls back to
 * the own map. Return codes as above. */
int  mimeshi_send_data_geo(mimeshi_node *n, uint16_t dst, int16_t geox, int16_t geoy,
                         const uint8_t *payload, uint16_t len, bool want_ack3);

/* Packet received (from radio or simulator). ch = channel of reception. */
void mimeshi_rx(mimeshi_node *n, uint8_t channel, int8_t rssi_db,
              const uint8_t *pkt, uint16_t len);

#endif /* MCORE_H */
