/*
 * Meshimeshi protocol core, internal configuration (config.h).
 * Values and status live in the design document; change them there first
 * (working rule: design -> core -> ctest + sim1.py grow along).
 * Shared by all core modules via internal.h; not part of the
 * public API (core.h).
 */
#ifndef MCORE_CONFIG_H
#define MCORE_CONFIG_H

#include <stdint.h>
#include <stdio.h>

#define MC_ACK_BACKOFF_CAP_TICKS 1440u     /* pace cap (5.3): 24 h in ticks
                                              (1 tick = 1 min) */
#define MC_STAR_LONELY_TICKS 5u            /* accelerated round interval while the
                                              node has no confirmed neighbour (5.3):
                                              5 min, full spread */
#define MC_MIN_US          60000000ull     /* 1 minute in us - the human time unit */
#define MC_KEEPALIVE_TICK_US MC_MIN_US      /* the time base without a clock: 1 tick = 1 min */
#define MC_KEEPALIVE_TICKS 60u             /* silence ticks = T_keepalive = 1 h (5.1) */
#define MC_HELLO_MAX_US    (MC_KEEPALIVE_TICKS * MC_KEEPALIVE_TICK_US)  /* 1 h stable (5.1); = T_keepalive */
#define MC_HELLO_JITTER_US 1000000ull      /* random phase per round (5.1, open point 3) */
#define MC_HELLO_ACK_WIN_US 3000000ull     /* max response time hello-acks: 3 s (2) */
#define MC_HELLO_ACK_JITTER_US (MC_HELLO_ACK_WIN_US / 16u)  /* ~187 ms, > t_ack + S (2) */
#define MC_BUDGET_INIT     32              /* initial value budget byte (9) */

/* Message tracing (forensics runs): prefix [msg] so the sim can grep
 * the trace apart from the always-on [core] lines. One byte test per
 * event - free when off. */
#define MTRACE(n, ...) do { if ((n)->trace) { \
    printf("[msg] node %u: ", (n)->id); printf(__VA_ARGS__); } } while (0)

/* ---------------- attempt schedule (8.4) -------------------------
 * One schedule per hop, no rebuild. Waits: a small fixed gap inside a
 * pass over the candidates (the next attempt goes to a DIFFERENT
 * node), an escalating gap at each pass boundary (the same node comes
 * back there - the damper belongs there). +-25% jitter on every gap:
 * two nodes that collided once must not retry in lock-step. With 1
 * candidate every attempt is a boundary; the pass table then runs
 * 0.8, 1.2, 1.8 s. Worst case total (6 attempts, 3 candidates):
 * ~6 s incl. attempt times - inside T_ack2 = 8 s (8.3). */
#define MC_ACK1_MARGIN_US   80000ull       /* processing + margin on top of t_data + t_ack (8.3) */
#define MC_T_ACK2_US        8000000ull    /* ack2 timeout (8.3, revised Oct 2026): 8 s - the attempt
                                   schedule of the next hop (8.4) completes in ~6 s worst case,
                                   so silence beyond 8 s means its forwarding truly stalled;
                                   measured against 15 s: fired 17x in 6h, all legitimate */
#define MC_ACK2_BUDGET_PENALTY 6u           /* budget units charged on the T_ack2 timeout (8.3) */
#define MC_S_SWITCH_US      5000ull        /* channel switch incl. settling (12.1) */
#define MC_SYM_QUALITY_MIN  13u            /* symmetry requires this quality (6.3):
                                               ~-102 dBm (decided Oct 2026, measured:
                                               at 14 the scenario kept 2 of 40 nodes
                                               without any routable link and lost ~10%
                                               of the unicasts to refusals; at 13
                                               every node has at least one confirmed
                                               neighbour and delivery reaches 98.9%.
                                               The retry series (8.4) absorb the
                                               weaker links' higher loss rate) */
#define MC_CAD_BACKOFF_US   20000ull       /* CAD backoff: random 0..19 ms (2) */
#define MC_CAD_MAX_TRIES    8u             /* ask at most 8x per transmission; then transmit anyway */

/* ---------------- airtime (SF7, 62.5 kHz, CR4/5) ----------------
 * Fixed values from the table in 12.1. The core passes the airtime to
 * tx() so the platform layer can schedule the transmit window; the
 * functions are deliberately simple (three classes) instead of an exact
 * LoRa airtime formula. 3 B packets (proof ack, and the legacy hello,
 *): 62 ms; hello-ack/proof list: 5-6 B and 3+2n B (10.2); ack1/ack2:
 * 7-8 B. */
static inline uint64_t t_ack_us(void)        { return 78000ull; }   /* ack1/ack2: 7 B */
static inline uint64_t t_hello_us(void)      { return 62000ull; }   /* 3 B packets */
static inline uint64_t t_hack_us(void)      { return 70000ull; }   /* hello-ack: 5 B (10.2) */
static inline uint64_t t_proof_us(void) { return 70000ull; }  /* proof list: 3+2n B,
                                                       n<=17 -> 37 B max; one
                                                       airtime class covers them
                                                       all (5.2) */
static inline uint64_t t_data_us(uint16_t l) { return l <= 40 ? 173000ull : l <= 150 ? 459000ull : 869000ull; }

#endif /* MCORE_CONFIG_H */
