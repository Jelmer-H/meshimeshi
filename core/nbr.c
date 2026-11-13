/*
 * Meshimeshi protocol core, nbr.c. See core/internal.h for the
 * module map; design references point into docs/design.md.
 */
#include "internal.h"
#include <string.h>
#include <stdio.h>

/* ---------------- keepalive counters (5.1) ----------------
 * Per channel the node counts how many keepalive ticks have passed since
 * it last "called" there itself — an identifying transmission on that
 * channel (hello-ack, ack1, ack2, data as prev_hop, ack3, star round,
 * proof list). ANN and FULL are anonymous and therefore do NOT reset.
 * Every own call sets the counter of that channel to 0; after
 * MC_KEEPALIVE_TICKS of silence exactly one proof list follows on that
 * channel (the keepalive, 5.2). The core has no clock: the tick timer
 * IS the time base. */
void ka_called(mimeshi_node *n, uint8_t ch) {
    if (ch < MC_N_CHANNELS) n->ka_idle[ch] = 0;
}

/* ---------------- neighbor table and dedup ---------------- */

/* Look up a neighbor by id. Linear search: MC_MAX_NEIGHBORS is small and
 * the table is only touched on reception or transmit choice. */
mimeshi_neighbor *nbr_find(mimeshi_node *n, uint16_t id) {
    for (int i = 0; i < MC_MAX_NEIGHBORS; i++) if (n->nbr[i].alive && n->nbr[i].id == id) return &n->nbr[i];
    return 0;
}

/* Quality byte from an RSSI measurement (5.2): halving of the normalized
 * RSSI leaves room for a future SNR component in the same byte. */
uint8_t quality_of(int8_t rssi) {
    return (uint8_t)(rssi < 0 ? (rssi + 128) >> 1 : 63);
}

/* Quality gate for symmetry confirmation (6.3): a link is used only
 * when it is above the quality threshold and works in both directions.
 * Each node can only measure the direction toward itself (the RSSI of
 * the received transmissions); the neighbour measures the opposite
 * direction itself and gates its own confirmation with the same rule.
 * The combined effect: a link is confirmed symmetric only when BOTH
 * directions are above the threshold. */
bool quality_ok(uint8_t q) { return q >= MC_SYM_QUALITY_MIN; }

/* Advertised quality class (6.1): the upper bits of the long-term
 * average, 3 bits (8 levels) in the prefix chunks. */
uint8_t adv_class_of(uint8_t q) { return (uint8_t)(q >> 3); }   /* 0..7 */

/* Hysteresis on the advertised link byte of the own star (6.1): each
 * direction nibble moves only when the fresh class is >= 2 steps from
 * the last published one - sub-threshold EWMA movement leaves the
 * advertised form untouched, so the content hash of the own star
 * stays stable while measurements drift. First publication takes the
 * fresh value. */
uint8_t adv_byte_hyst(uint8_t published, uint8_t fresh) {
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
uint8_t quality_from_class(uint8_t cls) { return (uint8_t)((cls << 3) | 4); }

/* EWMA on the long-term average (6.1): weight 1/32. */
uint8_t quality_ewma(uint8_t stored, uint8_t fresh) {
    return (uint8_t)(((uint16_t)stored * 31u + fresh) / 32u);
}


/* Add a neighbor (from a hello, hello-ack or proof list). Full = return 0;
 * the caller continues without the neighbor. A new entry starts as "seen":
 * one direction, sym = 0 (5.2). The local table counter feeds push-window
 * bundling (6.1); it never goes on air. */
mimeshi_neighbor *nbr_add(mimeshi_node *n, uint16_t id) {
    for (int i = 0; i < MC_MAX_NEIGHBORS; i++)
        if (!n->nbr[i].alive) {
            n->nbr[i].alive = 1; n->nbr[i].id = id;
            n->nbr[i].sym = 0; n->nbr[i].ack_try = 0;
            n->nbr[i].ack_spacing = 0; n->nbr[i].ack_next_tick = 0;
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
 * (each FORWARD is a fresh pending, and the ack1 rule of 8.4 kept the
 * loop fed). The recent 128 (msg-id, source) pairs always block. */
bool dedup_fresh(mimeshi_node *n, uint16_t msg_id, uint16_t src) {
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
