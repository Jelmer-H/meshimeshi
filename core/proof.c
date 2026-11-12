/*
 * Meshimeshi protocol core, proof.c. See core/internal.h for the
 * module map; design references point into docs/design.md.
 */
#include "internal.h"
#include <string.h>
#include <stdio.h>

/* ---------------- discovery and first evidence (5.3) ---------------- */

/* Collective proof ack (5.2, re-anchored by 5.3): one 3 B packet per
 * round that collected generalized acks, fired at the end of the ack
 * window. The responders sit on their home channels - the closure
 * reaches them there. Every responder of THIS round hears one small
 * packet instead of N individual confirmations; the responder-side
 * bookkeeping (acked_round on the neighbor) makes it exact: only
 * nodes that actually acked treat it as their closure. */
void proof_ack_fire(mimeshi_node *n, void *ud) {
    (void)ud;
    uint8_t p[3] = { PKT_PROOF_ACK, 0, 0 };
    wr16(p + 1, n->id);
    for (uint8_t c = 0; c < MC_N_CHANNELS; c++) {
        bound_send(n, c, p, 3, t_hello_us());             /* 3 B airtime */
        ka_called(n, c);
    }
}

/* Random phase per scheduled own call. Without jitter, nodes that
 * start up simultaneously (e.g. after a power outage) keep exactly
 * synchronous proof moments (5.1). Range in round_jitter_us so the
 * simulation can vary it per scenario. */
static uint64_t round_jitter(mimeshi_node *n) {
    return n->round_jitter_us ? (n->rand(n) % n->round_jitter_us) : 0;
}

/* Proof list for channel c (5.2): type, node-id, then the ids of the
 * direct neighbors at home on c whose reverse-direction evidence
 * ("last proven") expires within the flush margin. Reactionless by
 * design: no receiver ever replies. An empty list is the minimal
 * keepalive beacon (5.1). The packet is an own call on c and resets the
 * keepalive counter of c.
 *
 * Combined trigger (5.2): the flush check runs every 20 min rotating
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
    bound_send(n, c, p, (uint16_t)(3 + 2 * cnt), t_proof_us());
    ka_called(n, c);            /* own call: keepalive counter of c resets (5.1) */
}

static void proof_fire(mimeshi_node *n, void *ud) {
    proof_send(n, (uint8_t)(size_t)ud);
}

/* Flush check, rotating per channel (5.2). Each channel has its own
 * countdown in proof_wait[] ticks; the stagger (c x MC_PROOF_ROT_TICKS)
 * gives each channel one flush moment per hour, 20 minutes apart: no
 * self-congestion. Next to the countdown, a channel silent for
 * T_keepalive (ka_idle at max) is due NOW even without its slot -- the
 * keepalive trigger pulls the moment forward (5.1). The countdown
 * restarts at MC_KEEPALIVE_TICKS (= 1 h) on every fire: that is the cap,
 * one proof list per channel per hour maximum.
 */
#define MC_PROOF_ROT_TICKS (MC_KEEPALIVE_TICKS / MC_N_CHANNELS)  /* 20 min = 20 ticks (5.2) */


/* Death detection (6.2): every identifying reception refreshes
 * last_seen; per tick it ages. At T_expire (~4 h) the neighbour
 * is dead: the entry disappears from our table, and with it from our
 * own star - silence plus the round propagates the removal (6.2:
 * no tombstone, no negative delta). */
static void nbr_expire(mimeshi_node *n) {
    for (int i = 0; i < MC_MAX_NEIGHBORS; i++) {
        mimeshi_neighbor *nb = &n->nbr[i];
        if (!nb->alive) continue;
        if (nb->last_seen < 0xFFFF) nb->last_seen++;
        if (nb->last_seen >= MC_T_EXPIRE_TICKS) {
            nb->alive = 0; nb->sym = 0; nb->ack_try = 0;
            nb->quality = 0; nb->rev_quality = 0; nb->proven_age_us = 0;
            tbl_mark_dirty(n);              /* the own star loses the link (6.1) */
            printf("[core] node %u: neighbour %u expired after ~4 h of silence\n", n->id, nb->id);
        }
    }
}

void proof_tick(mimeshi_node *n, void *ud) {
    (void)ud;
    n->tick++;                                 /* local time base, 1 min per tick (6.2) */
    nbr_expire(n);                             /* death detection (6.2) */
    /* map aging (6.2): an entry nobody has refreshed anymore dies at
     * 2 x T_active (~24 h) - after that it is stale beyond every
     * cadence and only takes constellation space from fresher knowledge. */
    for (int i = 0; i < MC_MAP_NODES; i++) {
        if (!n->map[i].alive) continue;
        if (n->map[i].id == n->id) continue;   /* the own entry: alive by definition */
        if (nbr_find(n, n->map[i].id)) continue;           /* prefix work */
        /* liveness axis (6.2): a star whose origin tick has not advanced
         * for the threshold is dead, whatever the freshness of the
         * forwarded copies - the echo cannot fake a tick. Direct
         * neighbours live in the neighbor table (last_seen, 6.2). */
        if (n->tick - n->map[i].star_beat_seen >= MC_STAR_DEAD_TICKS) {
            printf("[core] node %u: star %u dead (beat stalled %lu ticks)\n",
                   n->id, n->map[i].id, (unsigned long)(n->tick - n->map[i].star_beat_seen));
            map_evict(n, &n->map[i]);
            continue;
        }
        if (map_age_ticks(n, &n->map[i]) >= 2 * MC_T_ACTIVE_TICKS)
            map_evict(n, &n->map[i]);
    }
    for (int i = 0; i < MC_MAX_NEIGHBORS; i++)          /* evidence ages per tick (5.2) */
        if (n->nbr[i].alive && n->nbr[i].proven_age_us < MC_T_SYM_US)
            n->nbr[i].proven_age_us += MC_KEEPALIVE_TICK_US;
    for (uint8_t c = 0; c < MC_N_CHANNELS; c++) {
        if (n->ka_idle[c] < MC_KEEPALIVE_TICKS) n->ka_idle[c]++;
        uint8_t due = n->proof_wait[c] == 0 || n->ka_idle[c] >= MC_KEEPALIVE_TICKS;
        if (due) {
            n->ka_idle[c] = 0;              /* reset on scheduling: one packet, no storm */
            n->proof_wait[c] = MC_KEEPALIVE_TICKS;        /* cap: next slot after 1 h */
            n->timer(n, round_jitter(n), proof_fire, (void *)(size_t)c);
        } else n->proof_wait[c]--;
    }
    n->timer(n, MC_KEEPALIVE_TICK_US, proof_tick, 0);
}

/* Deliver a hello-ack on the HOME channel of the acked sender (2): the
 * sender sits there outside its own exchanges and catches answers
 * within the response window. One exchange, then home.
 *
 * Directional format (6 B, 10.2): the destination is the acked sender.
 * Only because of that does the ack prove symmetry — the responder
 * received OUR identifying transmission (its ack is a reaction to it)
 * and its ack reached us (5.2, 6.3). That is the proof channel of the
 * generalized ack (5.3): available before the first unicast.
 * Eavesdroppers process the sender as "seen" at most; the state
 * version is dropped (6.1: no pull anymore). */
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
    ka_called(n, (uint8_t)(to % MC_N_CHANNELS));    /* own call on that channel (5.1) */
    /* three-way closure bookkeeping (5.2, Oct 2026): we acked this
     * neighbour's round - a later PROOF_ACK from it closes our side. */
    {
        mimeshi_neighbor *nb = nbr_find(n, to);
        if (nb) nb->acked_round = 1;
    }
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
 * Ack on demand (5.1, Oct 2026): a confirmed neighbor (sym, 5.2) gets
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
void hello_ack_wait(mimeshi_node *n, uint16_t from, int8_t rssi) {
    /* Generalized ack gate (5.3): called for every identifying
     * transmission of a neighbour. Quality gate (6.3), same threshold
     * as the confirmation: a reception below it can never become a
     * routing link, so an ack on it is pure airtime -- and around
     * boundary links it feeds the ack storm the pace gate prevents. */
    if (!quality_ok(quality_of(rssi))) return;
    mimeshi_neighbor *nb = nbr_find(n, from);
    if (nb) {
        if (nb->sym) {
            if (nb->ack_try >= 3) return;       /* courtesy used up (after 2) */
            nb->ack_try++;
        } else {
            /* time-based doubling backoff (5.3): the brake is independent
             * of the neighbour's transmission volume. After each ack that
             * does not close the three-way the spacing doubles, up to the
             * 24 h cap - a permanently one-way link costs one ack per day
             * at most, whatever the neighbour transmits. The fields are
             * not read for confirmed links, so no confirmation-site reset
             * is needed; an expired entry starts fresh (nbr_add). */
            if (n->tick < nb->ack_next_tick) return;
            uint16_t sp = nb->ack_spacing ? nb->ack_spacing : 1;
            nb->ack_next_tick = n->tick + sp;
            if (sp < MC_ACK_BACKOFF_CAP_TICKS) {
                sp = (uint16_t)(sp * 2);
                if (sp > MC_ACK_BACKOFF_CAP_TICKS) sp = MC_ACK_BACKOFF_CAP_TICKS;
            }
            nb->ack_spacing = sp;
        }
    }
    int norm = (rssi + 128) > 255 ? 255 : (rssi + 128);   /* 0..255 */
    uint32_t span = (uint32_t)(MC_HELLO_ACK_WIN_US - MC_HELLO_ACK_JITTER_US
                             - t_ack_us() - MC_S_SWITCH_US);
    uint32_t slot = (uint32_t)((255 - norm) * span / 255u);
    uint32_t jitter = n->rand(n) % (MC_HELLO_ACK_JITTER_US + 1u);
    pending_hack *ph = phack_get(from);
    if (ph) ph->quality = quality_of(rssi);      /* what we measured on the hello */
    n->timer(n, slot + jitter, hello_ack_fire, (void *)(size_t)from);
}

/* Stagger the flush moments (5.2): channel c is due for the first time
 * after c x 20 min, so the three channels lie 20 minutes apart. */
void proof_stagger(mimeshi_node *n) {
    for (uint8_t c = 0; c < MC_N_CHANNELS; c++)
        n->proof_wait[c] = (uint8_t)(c * MC_PROOF_ROT_TICKS);
}
