/*
 * Meshimeshi protocol core, unicast.c. See core/internal.h for the
 * module map; design references point into docs/design.md.
 */
#include "internal.h"
#include <string.h>
#include <stdio.h>

/* ---------------- unicast + acks (7) ---------------- */

/* Ack1: immediately, on the channel of the ongoing data exchange — the
 * sender waits there for the answer (switch rule, 2). The sender-id
 * travels along so the sender knows who has been heard. */
void ack1_send(mimeshi_node *n, uint16_t msg_id, uint16_t src, uint8_t ch, int8_t rssi) {
    uint8_t p[8] = { PKT_ACK1, 0,0, 0,0, 0,0, 0 };
    wr16(p + 1, msg_id); wr16(p + 3, src); wr16(p + 5, n->id);
    p[7] = quality_of(rssi);              /* what we measured on the data (10.2) */
    n->tx(n, ch, p, 8, t_ack_us());       /* same channel as the data (switch rule) */
    ka_called(n, ch);                     /* own call on the data channel (5.1) */
    MTRACE(n, "ACK1 -> %u for msg %u (their quality %u)\n", src, msg_id, p[7]);
}

/* Ack2 to the previous hop, on ITS home channel (2, 8.1): the previous
 * hop listens there without its own window. Positive = the own next
 * hop has ack1'd, so the message is two hops along (7.1/8.2, revised
 * Oct 2026); negative = forwarding failed after every series, via
 * F_NEG_ACK2, with the spent budget of the message in byte 7 (8.4):
 * the previous hop sees what is left and can stop against an
 * exhausted budget. */
void ack2_send(mimeshi_node *n, uint16_t msg_id, uint16_t src, uint16_t prev,
                      bool neg, uint8_t budget_left) {
    uint8_t p[8] = { (uint8_t)(PKT_ACK2 | (neg ? F_NEG_ACK2 : 0)), 0,0, 0,0, 0,0, 0 };
    wr16(p + 1, msg_id); wr16(p + 3, src); wr16(p + 5, n->id);
    p[7] = budget_left;                       /* spent budget travels along (6.2, 8.4) */
    bound_send(n, (uint8_t)(prev % MC_N_CHANNELS), p, 8, t_ack_us()); /* HOME of previous hop */
    ka_called(n, (uint8_t)(prev % MC_N_CHANNELS));  /* own call there (5.1) */
    MTRACE(n, "ACK2 %s -> %u for msg %u (budget %u)\n", neg ? "NEG" : "pos", prev, msg_id, budget_left);
}

void pending_try(mimeshi_node *n, mimeshi_pending *p);
void schedule_fail(mimeshi_node *n, mimeshi_pending *p);

/* Candidate collector for the attempt schedule (8.4): every candidate is
 * the BEST HOP of a full Dijkstra run - not a ranking of own edges.
 * Candidate 1 is the computed route toward dst; candidate 2 is the
 * best route that does not start with candidate 1; candidate 3
 * likewise without the first two. So each candidate is a real,
 * computed alternative over the whole known graph, and the best
 * candidate always leads. Same admission rules as every route
 * (10.1/7.1): confirmed symmetric, above the quality gate, never
 * the previous hop (no bounce). */
uint8_t schedule_candidates(mimeshi_node *n, uint16_t dst,
                                uint16_t gl, uint16_t go,
                                uint16_t prev,
                                const uint16_t *excl, uint8_t excl_cnt,
                                uint16_t *cand) {
    uint16_t tmp[3];
    uint8_t cnt = 0;
    for (int round = 0; round < 3; round++) {
        /* prev goes into the Dijkstra exclusion itself, exactly like
         * the baseline first choice (7.1): a post-filter that drops
         * the result when it happens to be prev would discard the
         * whole plan when the best route leads back - the second-best
         * non-prev route exists but was never computed (measured:
         * silent hop drops, 115 T_ack2 timeouts at the sender). */
        uint8_t ec = 0;
        if (prev) tmp[ec++] = prev;
        for (uint8_t e = 0; e < excl_cnt && ec < 3; e++) tmp[ec++] = excl[e];
        for (uint8_t k = 0; k < cnt && ec < 3; k++) tmp[ec++] = cand[k];
        uint16_t nh = route_next_hop_geo_prev(n, dst, gl, go, tmp, ec, prev);
        if (!nh) break;
        cand[cnt++] = nh;
    }
    return cnt;
}

/* Total attempts of the schedule by its size (8.4): 1 candidate = 4
 * (A,A,A,A - the damper table then runs 0.8, 1.2, 1.8 s); 2 = 6
 * (A,B,A,B,A,B - each node 3x); 3 = 6 (A,B,C,A,B,C - each node 2x).
 * At most 4 toward the same node, at most 6 total. */
uint8_t schedule_attempts(const mimeshi_pending *p) {
    return p->cand_cnt == 1 ? 4 : 6;
}

/* Attempt schedule gaps (8.4): a small fixed gap inside a pass over
 * the candidates (the next attempt goes to a DIFFERENT node), an
 * escalating gap at each pass boundary (the same node comes back
 * there - the damper belongs there). */
#define PLAN_INTRA_MS       400u           /* A->B within a pass (different node) */
static const uint32_t PLAN_PASS_MS[3] = { 800, 1200, 1800 };  /* escalating per pass (same node) */

/* The wait before attempt try_idx+1 (8.4). The boundary gap applies
 * when THAT attempt starts a new pass - the same node comes back,
 * and it may be briefly busy on another channel, so the damper
 * belongs there; inside a pass the next attempt goes to a different
 * node, so the small fixed gap suffices. */
static uint64_t schedule_wait_us(mimeshi_node *n, const mimeshi_pending *p) {
    uint8_t next = p->try_idx + 1;                      /* attempt this wait leads to */
    uint8_t npass = next / p->cand_cnt;                 /* pass of that attempt */
    uint8_t boundary = (uint8_t)((next % p->cand_cnt) == 0 && npass > 0);
    uint32_t base = boundary ? PLAN_PASS_MS[npass > 3 ? 2 : npass - 1] : PLAN_INTRA_MS;
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
 * negative ack2, 8.4). */
static void ack1_timeout(mimeshi_node *n, void *ud) {
    mimeshi_pending *p = (mimeshi_pending *)ud;
    if (!p->active || p->acked1) return;
    if (p->try_idx < schedule_attempts(p) - 1 && p->budget_left > 0) {
        n->timer(n, schedule_wait_us(n, p), retry_fire, p);
        return;
    }
    schedule_fail(n, p);
}

/* The attempt schedule toward this destination is exhausted (8.4 -
 * no rebuild: the attempts ARE the retry space of the hop). All
 * candidates go into the cumulative exclusion together
 * and the message fails definitively with the negative ack2 to the
 * previous hop, the spent budget along. The previous hop decides with
 * its own plan and budget whether another route remains. */
void schedule_fail(mimeshi_node *n, mimeshi_pending *p) {
    printf("[core] node %u: schedule failed toward %u..%u (msg %u)\n",
           n->id, p->cand[0], p->cand[p->cand_cnt ? p->cand_cnt - 1 : 0], p->msg_id);
    for (uint8_t k = 0; k < p->cand_cnt && p->excl_cnt < 3; k++) {
        p->excl[p->excl_cnt++] = p->cand[k];      /* cumulative (8.4) */
        mimeshi_neighbor *fnb = nbr_find(n, p->cand[k]);
        if (fnb && fnb->penalty < 255) fnb->penalty++;   /* long-term lesson (8.5) */
    }
    p->active = 0;
    printf("[core] node %u: FAIL msg %u (schedule exhausted, budget %u)\n",
           n->id, p->msg_id, p->budget_left);
    if (p->prev_hop != n->id)
        ack2_send(n, p->msg_id, p->src, p->prev_hop, true, p->budget_left);
}

/* One transmit attempt: update the budget in the header (offset 9,
 * 6.1/9), the exchange toward next_hop, and schedule the ack1 timeout.
 * The timeout is (8.3) S + t_data + t_ack + margin, measured from the
 * moment of submission. Assumption: the platform layer delivers the
 * transmission quickly (queue short); if the switch takes longer, the
 * timeout fires too early and the attempt counts as failed. */
/* T_ack2 timeout (8.3, fixed 8 s): the next hop received (ack1 came)
 * but its own forwarding stayed silent beyond every series. The
 * silence counts as a failed series toward this hop: 6 units off the
 * budget, then the 8.4 structure takes over - next-best hop if
 * attempts remain, otherwise the definitive fail. */
void ack2_timeout(mimeshi_node *n, void *ud) {
    mimeshi_pending *p = (mimeshi_pending *)ud;
    if (!p->active || !p->acked1) return;
    printf("[core] node %u: T_ack2 timeout for msg %u (budget %u -> %u)\n",
           n->id, p->msg_id, p->budget_left,
           (uint8_t)(p->budget_left > MC_ACK2_BUDGET_PENALTY ? p->budget_left - MC_ACK2_BUDGET_PENALTY : 0));
    p->budget_left = (uint8_t)(p->budget_left > MC_ACK2_BUDGET_PENALTY ? p->budget_left - MC_ACK2_BUDGET_PENALTY : 0);
    p->acked1 = 0;
    p->try_idx = (uint8_t)(schedule_attempts(p) - 1);   /* the wait consumed the remaining attempts (8.3) */
    schedule_fail(n, p);
}

void pending_try(mimeshi_node *n, mimeshi_pending *p) {
    if (p->cand_cnt == 0) return;             /* no schedule: nothing to send (defensive) */
    /* Attempt order (8.4): round-robin over the candidates - two
     * attempts toward the same node are then always a full pass
     * apart, so the intra-pass wait can stay short. */
    p->next_hop = p->cand[p->try_idx % p->cand_cnt];   /* round-robin over the schedule (8.4) */
    p->pkt[9] = p->budget_left;
    wr16(p->pkt + 10, p->next_hop);           /* next hop in the header (10.1) */
    MTRACE(n, "TRY %u/%u msg %u via %u (budget %u)\n",
           (unsigned)(p->try_idx + 1), (unsigned)schedule_attempts(p), p->msg_id, p->next_hop, p->budget_left);
    bound_send(n, (uint8_t)(p->next_hop % MC_N_CHANNELS), p->pkt, p->pkt_len, t_data_us(p->pkt_len));
    ka_called(n, (uint8_t)(p->next_hop % MC_N_CHANNELS));   /* data as prev_hop (5.1) */
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
 * proven by the neighbor itself: via its tablepull (6.1) or via a
 * successful unicast with ack1. Routing (building block 2, 6.3) must
 * demand that confirmation before forwarding; until then this direct
 * sending is bounded by retries+budget and fails visibly on the ack1
 * timeout. */
/* Hop-decision dossier (deep forensics): the COMPLETE graph of the
 * deciding node at the moment of the decision - map, neighbor table,
 * link store - plus the candidate plan and the chosen first hop.
 * One JSON object per decision, JSONL. Called at the decision sites
 * (send + forward) when n->hoptrace is set (mimeshi_set_hoptrace).
 * The graph dump is the exact state the Dijkstra and the bridge rule
 * saw - no reconstruction from end-state dumps, no offset drift. */
void hoptrace_dossier(mimeshi_node *n, uint16_t dst, uint16_t prev,
                             uint16_t gl, uint16_t go, uint8_t budget,
                             const uint16_t *excl, uint8_t excl_cnt,
                             const uint16_t *cand, uint8_t cand_cnt,
                             uint16_t chosen) {
    FILE *f = (FILE *)n->hoptrace;
    if (!f) return;
    fprintf(f, "{\"node\":%u,\"dst\":%u,\"prev\":%u,\"budget\":%u,"
               "\"dst_geo\":[%u,%u],\"in_graph\":%d,\"excl\":[",
            n->id, dst, prev, budget, gl, go,
            (r_idx(n, dst) >= 0 || nbr_find(n, dst) != 0) ? 1 : 0);
    for (uint8_t e = 0; e < excl_cnt; e++)
        fprintf(f, "%s%u", e ? "," : "", excl[e]);
    fprintf(f, "],\"plan\":[");
    for (uint8_t k = 0; k < cand_cnt; k++)
        fprintf(f, "%s%u", k ? "," : "", cand[k]);
    fprintf(f, "],\"chosen\":%u,\"nbrs\":[", chosen);
    int first = 1;
    for (int i = 0; i < MC_MAX_NEIGHBORS; i++) {
        mimeshi_neighbor *nb = &n->nbr[i];
        if (!nb->alive) continue;
        fprintf(f, "%s{\"id\":%u,\"sym\":%u,\"q\":%u,\"pen\":%u}",
                first ? "" : ",", nb->id, nb->sym, nb->quality, nb->penalty);
        first = 0;
    }
    fprintf(f, "],\"map\":[");
    first = 1;
    for (int i = 0; i < MC_MAP_NODES; i++) {
        mimeshi_mapnode *m = &n->map[i];
        if (!m->alive) continue;
        fprintf(f, "%s{\"id\":%u,\"geo\":[%u,%u],\"aq\":%u,\"age\":%u,\"st\":%u}",
                first ? "" : ",", m->id, m->geo_lat, m->geo_lon,
                m->adv_q, m->age_ticks, m->star_beat);
        first = 0;
    }
    fprintf(f, "],\"links\":[");
    first = 1;
    for (int i = 0; i < MC_MAX_LINKS; i++) {
        mimeshi_link *l = &n->lnk[i];
        if (!l->alive) continue;
        fprintf(f, "%s{\"a\":%u,\"b\":%u,\"q\":%u}",
                first ? "" : ",", l->a, l->b, l->q);
        first = 0;
    }
    fprintf(f, "]}\n");
}

int mimeshi_send_data(mimeshi_node *n, uint16_t dst, const uint8_t *payload, uint16_t len, bool ack3,
                      uint16_t dest_lat, uint16_t dest_lon) {
    /* routing (7.1): the first hop follows from the own known graph -
     * a direct neighbour only when its link is confirmed symmetric
     * (6.3), otherwise the computation decides; a destination outside
     * the graph routes via the bridge-node rule when the client
     * supplied its geo, and is refused without geo (no flood). */
    if (len > MC_MAX_PAYLOAD) return -3;
    mimeshi_pending *p = 0;
    for (int i = 0; i < MC_MAX_PENDING; i++) if (!n->pend[i].active) { p = &n->pend[i]; break; }
    if (!p) return -2;
    memset(p, 0, sizeof(*p));
    p->active = 1;
    p->budget_left = MC_BUDGET_INIT;
    /* attempt schedule (8.4): up to 3 Dijkstra-best hops toward dst; the
     * route itself is the first candidate - or the send is refused. */
    p->cand_cnt = schedule_candidates(n, dst, dest_lat, dest_lon, n->id, 0, 0, p->cand);
    if (n->hoptrace)
        hoptrace_dossier(n, dst, n->id, dest_lat, dest_lon, p->budget_left,
                         p->excl, 0, p->cand, p->cand_cnt,
                         p->cand_cnt ? p->cand[0] : 0);
    if (p->cand_cnt == 0) return -1;           /* not in the graph: no flood (7.1) */
    uint16_t nh = p->cand[0];
    p->msg_id = (uint16_t)n->rand(n);
    p->src = n->id; p->dst = dst; p->prev_hop = n->id; p->next_hop = nh;
    uint8_t *k = p->pkt;
    bool geo = (dest_lat || dest_lon);
    /* Waypoint (7.1): an out-of-graph send attaches the heading - the geo
     * of the chosen bridge candidate - so the hops route toward its zone
     * with locally routable steps instead of every hop re-inventing a
     * bridge from possibly worse knowledge (the plateau finding: constellations
     * held candidates at 11 km median from the destination, but no hop
     * could ROUTE to them - toward the zone, every step is local). */
    /* Waypoint retired (Oct 2026): the heading was built against the
     * collapsed-geo plateau - an artifact of the sim's broken longitude
     * encoding, fixed since. With a healthy two-dimensional geo space the
     * distance-first bridge rule alone navigates; the waypoint's extra
     * state did not pay for its complexity (the 1347 trace: re-aims on
     * equal headings, chains still leaving the arrival zone). The flag
     * stays defined for format compatibility; nothing sets it. */
    k[0] = (uint8_t)(PKT_DATA | (ack3 ? F_ACK3_REQ : 0)
                     | (geo ? F_DEST_GEO : 0));
    wr16(k + 1, p->msg_id); wr16(k + 3, n->id); wr16(k + 5, dst); wr16(k + 7, n->id);
    k[9] = p->budget_left;                 /* header: budget at offset 9 (section 10.1) */
    wr16(k + 10, nh);                      /* next hop at offset 10 (10.1) */
    uint8_t off = 12;
    if (geo) {                              /* dest geo after the fixed header (6.1, 7.1) */
        wr16(k + 12, dest_lat); wr16(k + 14, dest_lon);
        off = 16;
    }
    memcpy(k + off, payload, len);
    p->pkt_len = (uint16_t)(off + len);
    MTRACE(n, "SEND msg %u to %u (budget %u, plan %u cands first %u)\n",
           p->msg_id, dst, p->budget_left, p->cand_cnt, p->cand[0]);
    pending_try(n, p);
    return 0;
}
