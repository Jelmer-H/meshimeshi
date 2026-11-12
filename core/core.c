/*
 * Meshimeshi — protocol core, implementation. See core.h (API + node struct)
 * and docs/design.md (the section references below point there).
 *
 * Building block 1 (link test): hello round, neighbor table, hello-acks with
 * RSSI spreading, unicast with ack1/ack2/ack3, retries, budget.
 * Forwarding, routing (7.1), T_ack2 window (8.3) and tablepull (6.1)
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
 *    the ack1 window on the exchange channel, 8.3). The simulator
 *    (sim/sim1.py, class Node) is the reference implementation of that
 *    platform contract.
 *  - Ack1 is the exception: it goes without a switch via tx() on the
 *    channel of the ongoing exchange, because the sender waits there (8.1).
 *
 * Logging: the printf lines are intended for the link test and simulator.
 * Firmware can damp them; later they become a platform callback.
 */
#include "core.h"
#include <string.h>
#include <stdio.h>
#include "internal.h"
#include <string.h>
#include <stdio.h>

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
        /* Startup beacon (3 B, 5.1): update or add the neighbor, then
         * schedule an ack. Quality is the RSSI measurement of this
         * reception; last_seen is still a stub — real aging policy (6.2)
         * follows. No state version on air anymore (6.1). */
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
        uint16_t dst0 = rd16(pkt + 1);
        if (dst0 == n->id) n->proof_ack_pend = 1;   /* OUR round collected an ack (5.2) */
        /* Directional ack (5 B): destination = the original hello
         * sender.
         *  - Addressed to US: the responder received our hello and
         *    this ack reached us — both directions worked at that
         *    moment: symmetry proven (5.2, 6.3). This is the proof
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
                uint8_t rq = pkt[5];     /* what the responder measured on OUR signal (10.2) */
                nb->rev_quality = rq ? quality_ewma(nb->rev_quality ? nb->rev_quality : rq, rq)
                                     : nb->rev_quality;
                /* the round's STRONGEST received evidence (5.2, the
                 * two-sided threshold): the PROOF_ACK fires only when
                 * the best received rq clears the gate - a round of
                 * weak acks confirms nothing and the sender stays
                 * silent, so BOTH sides end without symmetry: the
                 * agreement falls out of the shared threshold. */
                if (rq > n->proof_ack_best_rq) n->proof_ack_best_rq = rq;
            }
            /* the TWO-SIDED THRESHOLD (5.2, decided Oct 2026), final
             * form: the reception quality of THIS packet is the only
             * gate here. The responder's side of the threshold is
             * already enforced by HIS ack gate (hello_ack_wait: he
             * only acks a hello he received above the gate - a
             * received ack therefore implies his measurement of us
             * cleared it; no rq re-test needed). Our side: we only
             * confirm what we received strongly. Either side weak =
             * no confirmation anywhere - both nodes end WITHOUT
             * symmetry, in agreement. The rq itself plays NO role in
             * the gate: it IS the link quality measurement toward us
             * (rev_quality, refreshed above) and prices the edge in
             * the route cost. The stored nb->quality never gates
             * what the packet proved either - same role: route cost. */
            if (dst == n->id && quality_ok(quality_of(rssi))) {
                if (!nb->sym) tbl_nbr_added(n, from);  /* delta + table round (6.1) */
                nb->sym = 1; nb->ack_try = 0;          /* confirmed symmetric (5.2, 6.3) */
            }
        }
        /* no generalized ack on a received ack (5.3): the exchange
         * closes via the collective proof ack - and acking an ack
         * would ping-pong forever, because every received ack resets
         * the receiver's courtesy counter */
        break;
    }
    case PKT_PROOF_ACK: {
        /* The collective third step (5.2, decided Oct 2026): the hello
         * sender confirms all received hello-acks of its round at once.
         * Only a node that ACKED that round has an open agreement - for
         * it, this packet is the closure: our ack ARRIVED (the packet is
         * the proof), our direction works, and the sender drew its
         * symmetry conclusion. sym = 1 on this RECEIVED evidence: our
         * own quality measurement of the neighbour no longer blocks
         * what the packet proves (the 12/13-case: both directions
         * physically working, one point under the gate, link refused -
         * the two two-way handshakes could disagree). Our own
         * measurement keeps its role in the ROUTE COST (poorly heard
         * = expensive edge), never again as the gatekeeper of what
         * the neighbour just proved. */
        uint16_t from = rd16(pkt + 1);
        mimeshi_neighbor *nb = nbr_find(n, from);
        if (nb && nb->acked_round) {
            nb->acked_round = 0;                  /* the agreement is closed */
            nb->proven_age_us = 0;                /* reverse direction freshly proven */
            if (!nb->sym) tbl_nbr_added(n, from); /* delta + table round (6.1) */
            nb->sym = 1; nb->ack_try = 0;         /* confirmed symmetric (5.2) */
        }
        break;
    }
    case PKT_PROOF: {
        /* Proof list (5.2): type, node-id, n x neighbour-id. Reactionless:
         * never reply, whatever the content. Two effects:
         *  - WE appear in the list: the sender claims it hears us —
         *    both directions confirmed in a single reception (5.2),
         *    reverse evidence freshly refreshed.
         *  - We do not appear: snooping — liveness and quality for the
         *    sender, "seen" at most. No downgrade either: absence in
         *    one packet is no negative evidence (5.2, decided). */
        uint16_t from = rd16(pkt + 1);
        uint16_t ids = (uint16_t)((len - 3) / 2);
        mimeshi_neighbor *nb = nbr_find(n, from);
        if (!nb) nb = nbr_add(n, from);
        if (nb) {
            nb->last_seen = 0;
            nb->quality = quality_of(rssi);
        }
        /* appearing in the list = reverse evidence refreshed (5.2);
         * symmetry only above the quality threshold (6.3) */
        for (uint16_t i = 0; i < ids; i++) {
            if (rd16(pkt + 3 + 2 * i) == n->id && nb) {
                nb->proven_age_us = 0;
                /* his proof list naming us = he claims the link works both
                 * ways. The TWO-SIDED threshold (5.2, decided Oct 2026):
                 * his claim is one direction's evidence; our reception
                 * quality of THIS packet is the other. Both above the
                 * gate = mutual; a claim received weakly confirms
                 * nothing (and his own gates filter his side the same
                 * way - both end without symmetry, in agreement). */
                if (quality_ok(quality_of(rssi)) && !nb->sym) {
                    nb->sym = 1; nb->ack_try = 0; tbl_nbr_added(n, from);
                }
            }
        }
        hello_ack_wait(n, from, rssi);       /* generalized ack trigger (5.3):
                                                the list itself stays reactionless
                                                for its CONTENT - the sender's
                                                link state is the ack reason */
        break;
    }
    case PKT_ACK1: {
        /* our own outstanding transmission has arrived at the next hop:
         * free the pending record. The hop is done; ack2 arrives on its
         * own on our home channel (8.1). Matching on msg-id suffices in
         * building block 1 (ids are random per transmission); with
         * routing added, the source should be counted along.
         *
         * The sender of this ack1 received our message AND its ack
         * reached us: the link is hereby proven symmetric (5.2). This
         * is the first exact symmetry proof the core knows; the second
         * is the tablepull of the neighbor (6.1, building block 2). A
         * hello-ack proves no symmetry — unaddressed on a shared home
         * channel. */
        uint16_t msg_id = rd16(pkt + 1);
        hello_ack_wait(n, rd16(pkt + 5), rssi);   /* generalized ack trigger (5.3) */
        for (int i = 0; i < MC_MAX_PENDING; i++) {
            mimeshi_pending *p = &n->pend[i];
            if (p->active && p->msg_id == msg_id && !p->acked1) {
                p->acked1 = 1;                     /* the own hop succeeded (8.1) */
                mimeshi_neighbor *nb = nbr_find(n, rd16(pkt + 5));
                if (nb) {                               /* confirmed symmetric (5.2, 6.3) */
                    nb->quality = quality_of(rssi);     /* the ack1 itself is the measurement (its direction) */
                    nb->proven_age_us = 0;
                    if (nb->penalty) nb->penalty--;      /* recovery via ordinary traffic (8.5) */
                    if (len >= 8 && pkt[7]) {            /* what the acker measured on OUR data (10.2) */
                        nb->rev_quality = quality_ewma(nb->rev_quality ? nb->rev_quality : pkt[7], pkt[7]);
                    }
                    /* two-sided threshold, final form (5.2, Oct 2026): the
                     * ack1 arriving here IS the reverse direction worked;
                     * our reception quality of it is the measured gate.
                     * The rq byte (pkt[7]) is the link quality toward us,
                     * never a gate. */
                    if (quality_ok(quality_of(rssi))) {
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
                     * own home channel. T_ack2 (8 s) is the safety
                     * net (8.3). Exception (decided Oct 2026, the
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
        /* Ack2 from the next hop (7.1/8.2, revised Oct 2026). Positive
         * = two hops of progress: the next hop received AND its own hop
         * succeeded - the pending record closes. Negative = its
         * forwarding failed after every series; the spent budget
         * travels along (byte 7, 8.4): weigh the own alternatives
         * against what is left, stop when it is gone. */
        uint16_t msg_id = rd16(pkt + 1);
        hello_ack_wait(n, rd16(pkt + 5), rssi);   /* generalized ack trigger (5.3) */
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
                    schedule_fail(n, p);          /* exclusion + next plan (8.4) */
                }
                return;
            }
        }
        break;
    }
    case PKT_ACK3: {
        /* final destination has had the message delivered (8.8): routed
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
        /* THE DESTINATION SHORTCUT (decided Oct 2026): a data packet
         * routed toward another neighbour but received HERE, where WE
         * are the final destination, is DELIVERED on the spot. The
         * sender aimed at a relay; the physical reach brought it to
         * the right address anyway (same channel, audible) - refusing
         * it would be pure formality while the message sits one hop
         * from its goal. The delivery path below handles the dedup
         * (a second copy is ignored) and the ack1 goes to the actual
         * sender, exactly as a directly addressed delivery would.
         * This closes the forwarding loop: the last stretch becomes
         * 'any transmission the destination happens to hear'. */
        if (dst == n->id && rd16(pkt + 10) != n->id) {
            /* fall through into the delivery processing below: the
             * dedup check there gates double deliveries, the ack1
             * closes the sender's pending. */
        }
        /* next-hop addressing (10.1): only the hop the sender chose
         * processes the packet; other hearers on the home channel
         * snoop (liveness, quality) - a routed unicast must not
         * multiply. */
        else if (rd16(pkt + 10) != n->id) return;
        /* dedup gates the processing (no second delivery, no second
         * forward), but a retry STILL gets its ack1 (7.4, decided Oct
         * 2026): the most likely reason for the retry is a lost ack1,
         * not a lost data packet - silence would make the sender
         * declare the link dead and detour. */
        bool fresh = dedup_fresh(n, msg_id, src);
        if (!fresh) { ack1_send(n, msg_id, src, ch, rssi); return; }
        if (dst == n->id) {
            uint8_t poff = (flags & F_DEST_GEO) ? 16 : 12;   /* geo sits between header and payload (10.1) */
            printf("[core] node %u: DATA from %u: '%.*s\n", n->id, src, (int)(len - poff), (const char *)(pkt + poff));
            MTRACE(n, "DELIVERED msg %u from %u (budget %u, via %u)\n", msg_id, src, pkt[9], prev);
            ack1_send(n, msg_id, src, ch, rssi);       /* same channel; quality of the data travels along (8.1) */
            /* No ack2 from the destination: the positive ack2 of this
             * final hop goes out by the previous forwarder on OUR
             * ack1 (7.1, revised Oct 2026: two hops per ack2). */
            if (flags & F_ACK3_REQ) {
                /* ack3 as a routed message (10.2): DATA format, payload
                 * = the original msg-id, back toward the source - with
                 * routing, via the first hop computation. */
                uint16_t ah = route_next_hop(n, src);
                if (ah) {
                    uint8_t a[16] = { PKT_ACK3, 0,0, 0,0, 0,0, 0,0, 0, 0,0, 0,0, 0,0 };
                    wr16(a + 1, (uint16_t)n->rand(n)); wr16(a + 3, n->id);
                    wr16(a + 5, src); wr16(a + 7, n->id); a[9] = MC_BUDGET_INIT;
                    wr16(a + 10, ah); wr16(a + 12, msg_id);
                    bound_send(n, (uint8_t)(ah % MC_N_CHANNELS), a, 16, t_data_us(16));
                    ka_called(n, (uint8_t)(ah % MC_N_CHANNELS));   /* own call (5.1) */
                }
            }
            break;
        }
        /* forwarding (10.2, stateless per message): ack the previous
         * hop, then recompute the own first step toward the
         * destination. The budget bounds the chain (9); the next-hop
         * field in the header makes the chain unambiguous (10.1). */
        if (len < 12 || src == n->id) return;    /* malformed, or our own message echoed back */
        uint8_t budget = pkt[9];
        ack1_send(n, msg_id, src, ch, rssi);      /* the exchange of this hop (8.1) */
        if (budget == 0) return;                  /* budget spent: the chain stops here */
        uint16_t gl = 0, go = 0;
        if (flags & F_DEST_GEO) { gl = rd16(pkt + 12); go = rd16(pkt + 14); }   /* geo stays in the header (7.1) */
        /* prev-hop exclusion at the FIRST forward choice (decided Oct
         * 2026, measured under load): the hop the message came from
         * is not a candidate for the next one. The known graph is
         * local and incomplete, so two nodes can each other's best
         * first hop toward the destination (measured: 279<->285
         * bounced one message 166 times, kept alive by the dedup
         * pass-through, only broken by a 15 s T_ack2). The retry
         * machinery of 8.4 applies this exclusion from its second
         * series onward; here it prevents the two-hop loop from ever
         * starting. The source itself stays reachable as a fallback:
         * when the exclusion leaves no candidate, the hop simply does
         * not forward (the negative ack2 / failure report of 8.6
         * follows later). */
        {
            mimeshi_pending *p = 0;
            for (int i = 0; i < MC_MAX_PENDING; i++) if (!n->pend[i].active) { p = &n->pend[i]; break; }
            if (!p) {
                /* No slot (backpressure, 8.2): say NEGATIVE instead of
                 * dropping silently - the previous hop can try an
                 * alternative with its own retry plan (8.4) instead of
                 * burning its T_ack2 window on a hop that already let
                 * go of the message. The budget we pass back is the
                 * full remaining one minus this hop's unit: nothing was
                 * spent here. (The measured scale run: 554 of 573 lost
                 * messages ended exactly here - ACK1 sent, then the
                 * silent return - the dominant delivery failure.) */
                ack2_send(n, msg_id, src, prev, true,
                          (uint8_t)(budget > 1 ? budget - 1 : 0));
                return;
            }
            memset(p, 0, sizeof(*p));
            p->budget_left = (uint8_t)(budget - 1);

            /* Waypoint retired (Oct 2026 - see mimeshi_send_data): every
             * hop applies the distance-first bridge rule on the DESTINATION
             * geo directly. No traveling state; the header format keeps the
             * F_WAYPOINT_GEO flag defined but unused. */

            /* attempt schedule (8.4): up to 3 Dijkstra-best hops toward
             * dst; the collector never proposes the previous hop (the
             * 7.1 prev-hop exclusion is thereby part of every
             * candidate, not only the first). active goes on only
             * after the check - an unforwardable message must not
             * leak its pending slot. */
            p->cand_cnt = schedule_candidates(n, dst, gl, go, prev, 0, 0, p->cand);
            p->in_graph = r_idx(n, dst) >= 0 || nbr_find(n, dst) != 0;   /* Dijkstra-proven plan? (8.4) */
            if (n->hoptrace)
                hoptrace_dossier(n, dst, prev, gl, go, (uint8_t)(budget - 1),
                                 p->excl, 0, p->cand, p->cand_cnt,
                                 p->cand_cnt ? p->cand[0] : 0);
            if (p->cand_cnt == 0) {
                /* No routable path from here (7.2/8.6): NEGATIVE back,
                 * not silence - the previous hop retries via its own
                 * plan (8.4). Same budget arithmetic: nothing spent. */
                ack2_send(n, msg_id, src, prev, true,
                          (uint8_t)(budget > 1 ? budget - 1 : 0));
                return;
            }
            p->active = 1;
            p->msg_id = msg_id; p->src = src; p->dst = dst; p->prev_hop = prev;
            memcpy(p->pkt, pkt, len); p->pkt_len = len;
            p->pkt[7] = (uint8_t)(n->id & 0xFF); p->pkt[8] = (uint8_t)(n->id >> 8);  /* prev hop = us */
            printf("[core] node %u: FORWARD msg %u from %u to %u via %u\n", n->id, msg_id, src, dst, p->cand[0]);
            MTRACE(n, "FORWARD msg %u from %u to %u via %u (budget %u, plan %u cands)\n",
                   msg_id, src, dst, p->cand[0], p->budget_left, p->cand_cnt);
            pending_try(n, p);
        }
        break;
    }
    default: break;
    }
}

/* Message tracing (forensics runs, the --trace option of the sim):
 * every DATA send/receive and each ack of a traced node logs with a
 * [msg] prefix. One byte test per event - free when off. */
void mimeshi_set_trace(mimeshi_node *n, uint8_t on) { n->trace = on; }

void mimeshi_set_hoptrace(mimeshi_node *n, void *file) { n->hoptrace = file; }

/* ---------------- init/start ---------------- */

/* Zeroing + filling in the callbacks. The home channel follows from the
 * id (id mod 3, 2). round_jitter_us gets the firmware default; the
 * simulator may change it before mimeshi_start() to vary the range.
 * The node struct lives in core.h so callers can
 * reserve it statically — hence the zeroing here. */
void mimeshi_init(mimeshi_node *n, uint16_t id, mimeshi_tx_fn tx, mimeshi_timer_fn timer,
                mimeshi_switch_fn sw, mimeshi_rand_fn rnd,
                mimeshi_channel_free_fn cad, void *user) {
    memset(n, 0, sizeof(*n));
    n->id = id; n->home_ch = (uint8_t)(id % MC_N_CHANNELS);
    n->local_tbl_ctr = 1;
    n->round_jitter_us = MC_HELLO_JITTER_US;
    n->tx = tx; n->timer = timer; n->switch_ch = sw; n->rand = rnd; n->user = user;
    n->channel_free = cad;
}

/* First hello round immediately (fast map building, 5.1), then the
 * adaptive timer with random phase. Idempotent for the rest of the
 * core: after start the hello runs on the timer; do not repeat
 * mimeshi_start() itself. After the startup phase the hello stops; the
 * tick carries keepalive and symmetry flush from here (5.1, 5.2). */
/* Own geo in the 16-bit grid (5): (0,0) = none. A later call with a
 * different value is a mutation and feeds the table push (6.1). */
void mimeshi_set_geo(mimeshi_node *n, uint16_t lat, uint16_t lon) {
    if (lat == n->geo_lat && lon == n->geo_lon) return;
    n->geo_lat = lat; n->geo_lon = lon;
    tbl_mark_dirty(n);
}

void mimeshi_start(mimeshi_node *n) {
    proof_stagger(n);
    /* tick: time base for keepalive + symmetry flush (5.1, 5.2) */
    n->timer(n, MC_KEEPALIVE_TICK_US, proof_tick, 0);
    /* star round (6.1, 5.3): the uniform convergence cadence AND the
     * discovery mechanism. A fresh node is lonely (no confirmed
     * neighbour), so the first round rides the accelerated interval:
     * a random phase within it prevents synchronous rounds on
     * simultaneous startup - the old hello jitter, now on the star
     * round (5.1). */
    n->star_own_ctr = 0;
    n->star_beat = 0;
    n->star_round_sched = 1;
    n->timer(n, (n->rand(n) % MC_STAR_LONELY_TICKS) * MC_KEEPALIVE_TICK_US,
             star_round_tick, 0);
}
