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
    uint8_t  alive;
    uint8_t  sym;           /* 1 = confirmed symmetric (hello-ack addressed
                               to us, proof list in which we appear, ack1
                               on our data, or table chunk naming us);
                               0 = only seen, one direction (4.2, 10.1) */
} mimeshi_neighbor;

typedef struct {
    uint8_t  active, tries_left, budget_left;
    uint16_t msg_id, src, dst, prev_hop, next_hop;
    uint8_t  pkt[10 + MC_MAX_PAYLOAD];
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

/* Clear-to-send (14, open point 8): the platform layer answers whether
 * the channel is free at this moment (carrier sense). The core asks
 * before every transmission that is not a direct reply: if busy, it
 * waits with random backoff on the channel and asks again. Ack1 is the
 * exception (7.1): it is a direct reply within the ongoing exchange,
 * the partner is already waiting there, so it goes without a check.
 * NULL = no carrier sense available (stub platform): transmit as before. */
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
    mimeshi_tx_fn     tx;
    mimeshi_timer_fn  timer;
    mimeshi_switch_fn switch_ch;
    mimeshi_rand_fn   rand;
    mimeshi_channel_free_fn channel_free;  /* carrier sense (CTS); NULL = off */
    void *user;                       /* own data of platform/simulator */
};

/* ---------------- API ----------------------------------------------- */
/* Initializes the node (zeroing + callbacks). The id determines the home channel. */
void mimeshi_init(mimeshi_node *n, uint16_t id, mimeshi_tx_fn tx, mimeshi_timer_fn timer,
                mimeshi_switch_fn sw, mimeshi_rand_fn rnd,
                mimeshi_channel_free_fn cts, void *user);

/* First hello round immediately, then startup rounds with random phase
 * (jitter). After the startup phase the hello no longer exists:
 * channel liveness is carried by traffic plus the proof list with the
 * keepalive role (4.1, 4.2). */
void mimeshi_start(mimeshi_node *n);

/* Unicast to dst. 0 = accepted, -1 = dst not in the map (no flood),
 * -2 = no free pending slot, -3 = payload too large. */
int  mimeshi_send_data(mimeshi_node *n, uint16_t dst, const uint8_t *payload,
                     uint16_t len, bool want_ack3);

/* Packet received (from radio or simulator). ch = channel of reception. */
void mimeshi_rx(mimeshi_node *n, uint8_t channel, int8_t rssi_db,
              const uint8_t *pkt, uint16_t len);

#endif /* MCORE_H */
