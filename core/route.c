/*
 * Meshimeshi protocol core, route.c. See core/internal.h for the
 * module map; design references point into docs/design.md.
 */
#include "internal.h"
#include <string.h>
#include <stdio.h>

/* ---------------- route computation (10.2, building block 2) ------
 * Stateless per message: no path in the header, every hop computes its
 * own first step on the own known graph. Cost model (decided, Oct
 * 2026): ETX per link = 1/p on the integer scale x16, one lookup table
 * indexed on the advertised quality class; the route cost is the sum
 * over the path, so the computation minimises the expected number of
 * transmissions. Age multiplier (7.1): an endpoint above half of
 * T_active makes the edge 1.5x as expensive, above T_active itself
 * 3x - stale knowledge loses against fresh. First step (hard
 * requirement, 6.3): only confirmed symmetric neighbours above the
 * quality threshold. */
#define MC_ROUTE_SCALE 16u                /* cost 1.0 (7.1) */
static const uint16_t mc_etx_class[8] = {
    0,                                /* class 0: not routable (7.1) */
    228, 145, 94, 64, 43, 29, 20      /* classes 1..7: cost x16 (10.2 table) */
};

/* Age multiplier on an edge endpoint: follows the effective age of the
 * map entry (6.2), in ticks (1 min). Returns 2 = 1.5x, 3 = 3x. */
static uint16_t route_age_mult(uint32_t age) {
    if (age >= MC_T_ACTIVE_TICKS) return 3;
    if (age >= MC_T_ACTIVE_TICKS / 2) return 2;
    return 0;
}

/* Liveness de-weight (6.2): a star whose origin tick has stalled for
 * more than half the liveness threshold costs like a half-stale entry
 * - the routes move away from possibly-dead knowledge before the
 * eviction takes it out entirely. */
static uint16_t route_live_mult(mimeshi_node *n, mimeshi_mapnode *m) {
    if (m->id == n->id) return 0;               /* the own star is alive by definition */
    if (n->tick - m->star_beat_seen < MC_STAR_DEAD_TICKS / 2) return 0;
    return 2;
}

static uint32_t mult_apply(uint32_t c, uint16_t m) {
    if (m == 3) return c * 3;
    if (m == 2) return c * 3 / 2;
    return c;
}

/* Cost of the own edge toward a confirmed neighbour: the measured
 * quality class (finer than the advertised classes), with the age
 * addition of the map entry when we have one. Penalty (8.5) follows
 * in a later building step. */
static uint32_t route_edge_own(mimeshi_node *n, mimeshi_neighbor *nb) {
    /* the AUTHORITATIVE class (decided Oct 2026): the star of the link
     * owner is the shared truth - the stored link class prices the edge,
     * the same information every neighbour holds. The private measurement
     * is only the fallback for a link the stars have not covered yet.
     * The store keeps the canonical order (lnk_add: a < b) with the
     * class in OUR direction: hi nibble = a->b when we are a, lo = b->a
     * when we are b. */
    uint8_t cls = 0;
    uint16_t us = n->id, them = nb->id;
    for (int j = 0; j < MC_MAX_LINKS && !cls; j++) {
        mimeshi_link *l = &n->lnk[j];
        if (!l->alive) continue;
        if ((l->a == us && l->b == them) || (l->a == them && l->b == us)) {
            uint16_t mn = us < them ? us : them;   /* canonical a (lnk_add) */
            cls = (us == mn) ? (uint8_t)(l->q >> 4) : (uint8_t)(l->q & 0x0F);
        }
    }
    if (!cls) cls = adv_class_of(nb->quality);   /* fallback: private measurement */
    uint32_t c = (cls < 8 && mc_etx_class[cls]) ? mc_etx_class[cls] : MC_ROUTE_SCALE;
    mimeshi_mapnode *m = map_find(n, nb->id);
    if (m) {
        c = mult_apply(c, route_age_mult(map_age_ticks(n, m)));
        {   uint16_t lm = route_live_mult(n, m);
            if (lm) c = mult_apply(c, lm); }
    }
    return c;
}

/* Dijkstra scratch: bounded like the map (11.2); static because the
 * core has no heap and is single-threaded per node (13b). */
#define MC_DIST_INF 0x7FFFFFFFu
static uint16_t route_next_hop_ex_prev(mimeshi_node *n, uint16_t dst,
                                  const uint16_t *excl, uint8_t excl_cnt,
                                  uint16_t prev);
static uint16_t route_bridge_node(mimeshi_node *n, uint16_t lat, uint16_t lon,
                                  const uint16_t *excl, uint8_t excl_cnt);
static uint32_t r_dist[MC_MAP_NODES];
static uint16_t r_prev[MC_MAP_NODES];   /* predecessor map slot; 0xFFFE = own edge */
static uint8_t  r_done[MC_MAP_NODES];

/* The first hop toward dst on the own known graph, or 0 when dst is
 * unreachable. The first step runs only over confirmed symmetric
 * neighbours above the quality threshold (6.3); further edges run
 * over the link store, direction of travel known, endpoints not
 * expired (6.2). A confirmed direct neighbour is its own first hop
 * even without a map slot: it is the destination itself.
 * `excl` (excl_cnt entries) are next hops that already failed for
 * THIS message: cumulative exclusion per series (8.4) - the
 * computation must not propose a hop the series has buried. */
static uint16_t route_next_hop_ex(mimeshi_node *n, uint16_t dst,
                                  const uint16_t *excl, uint8_t excl_cnt) {
    return route_next_hop_ex_prev(n, dst, excl, excl_cnt, 0);
}
static uint16_t route_next_hop_ex_prev(mimeshi_node *n, uint16_t dst,
                                  const uint16_t *excl, uint8_t excl_cnt,
                                  uint16_t prev) {
    (void)prev;    /* the prev-neighbourhood rule is removed (Oct 2026, the
                       6670 lesson - see the virtual-start comment below); prev
                       itself is excluded via the excl array, which every
                       caller fills (schedule_candidates puts it in first). The
                       parameter stays so the call sites that know prev read
                       clearly. */
    if (dst == n->id) return n->id;
    int di = r_idx(n, dst);
    /* A direct neighbour is NOT its own first hop by definition: its
     * own edge competes in the computation like every other edge, and
     * two good hops beat one bad hop (10.2, ETX). But a confirmed
     * neighbour without a map slot must still be able to WIN that
     * competition when it is the destination itself - otherwise a
     * fresh symmetric neighbour is unreachable until its chunk
     * arrives (measured: 'no candidate hop' losses under load). */
    mimeshi_neighbor *dnb = nbr_find(n, dst);
    int direct_ok = (dnb && dnb->sym && quality_ok(dnb->quality));
    if (di < 0) {
        if (direct_ok) {
            for (uint8_t e = 0; e < excl_cnt; e++) if (excl[e] == dst) return 0;
            return dst;                          /* the only edge toward it is the own one */
        }
        return 0;                                /* not in the graph: no flood (7.1) */
    }
    for (int i = 0; i < MC_MAP_NODES; i++) { r_dist[i] = MC_DIST_INF; r_prev[i] = 0xFFFF; r_done[i] = 0; }
    for (int i = 0; i < MC_MAX_NEIGHBORS; i++) {           /* own edges (virtual start) */
        mimeshi_neighbor *nb = &n->nbr[i];
        /* ONE TRUTH (decided Oct 2026, the forwarding-loop finding):
         * the virtual start admits CONFIRMED (sym) neighbours and
         * prices the own edge with the AUTHORITATIVE link class from
         * the star - the same information every neighbour holds in
         * its link store - instead of our private last-heard quality.
         * Private measurements made neighbours disagree: each
         * door-holder measured the destination just under the gate
         * (q 10-12), saw its star-published class-1 link, concluded
         * 'I cannot, but my neighbour can' - and they handed the
         * message to each other until the budget ran out. With the
         * star class, everyone prices the edge identically and
         * computes the same path. The quality gate itself falls:
         * sym (the two-sided threshold proof) is the admission, the
         * star class is the price - marginal links stay expensive
         * (class 1 = ETX 228), never invisible. */
        if (!nb->alive || !nb->sym) continue;
        int ex = 0;
        for (uint8_t e = 0; e < excl_cnt; e++) if (excl[e] == nb->id) { ex = 1; break; }
        if (ex) continue;                          /* failed for this message (8.4) */
        /* The prev-neighbourhood rule is REMOVED (decided Oct 2026, the
         * 6670 lesson): 'prev already considered its neighbours and chose us'
         * does not hold - prev's choice is one optimum under ITS cost model
         * (its link qualities, its age multipliers, its graph), and a
         * legitimate route from here may well begin with a hop into prev's
         * neighbourhood: our edge toward X can score better than prev's did,
         * or X's onward path better in OUR map. The measured cost: in a dense
         * cluster the last hops are each other's neighbours AND shared
         * neighbours of the destination - the rule filtered the delivery edge
         * at every forwarder (the 6670 packet circled a reachable destination
         * in its constellation until the sender gave up). The prev hop ITSELF stays
         * excluded (the original anti-ping-pong rule, via the excl array);
         * only the neighbourhood extension is gone. Triangle loops are then
         * possible again - the measurement decides whether the heuristic was
         * worth its price. */
        int bi = r_idx(n, nb->id);
        if (bi < 0) continue;                     /* no map slot: no further edges */
        uint32_t c = route_edge_own(n, nb);
        if (c < r_dist[bi]) { r_dist[bi] = c; r_prev[bi] = 0xFFFE; }
    }
    for (;;) {
        int u = -1;
        for (int i = 0; i < MC_MAP_NODES; i++)
            if (!r_done[i] && r_dist[i] < MC_DIST_INF && (u < 0 || r_dist[i] < r_dist[u])) u = i;
        if (u < 0 || u == di) break;              /* nothing left, or target settled */
        r_done[u] = 1;
        uint16_t mult_u = route_age_mult(map_age_ticks(n, &n->map[u]));
    {   uint16_t lm = route_live_mult(n, &n->map[u]);
        if (lm) mult_u = (uint16_t)(mult_u ? (mult_u * lm) / 2 * 2 : lm); }

        for (int j = 0; j < MC_MAX_LINKS; j++) {           /* relax the edges of map[u] */
            mimeshi_link *l = &n->lnk[j];
            if (!l->alive) continue;
            uint16_t from = n->map[u].id;
            if (l->a != from && l->b != from) continue;
            uint8_t cf = (l->a == from) ? (uint8_t)((l->q >> 4) & 0x0F) : (uint8_t)(l->q & 0x0F);
            if (cf == 0) continue;                          /* direction of travel unknown */
            uint32_t c = mc_etx_class[cf & 7];
            if (!c) continue;
            c = mult_apply(c, mult_u);                     /* stale endpoint u */
            uint16_t o = (l->a == from) ? l->b : l->a;
            int oi = r_idx(n, o);
            if (oi < 0 || r_done[oi]) continue;
            c = mult_apply(c, route_age_mult(map_age_ticks(n, &n->map[oi])));
            {   uint16_t lm = route_live_mult(n, &n->map[oi]);
                if (lm) c = mult_apply(c, lm); }
            if (r_dist[u] + c < r_dist[oi]) { r_dist[oi] = r_dist[u] + c; r_prev[oi] = (uint16_t)u; }
        }
    }
    if (r_dist[di] >= MC_DIST_INF) return 0;     /* unreachable in the graph */
    int cur = di;                                 /* walk back to the own edge */
    while (r_prev[cur] != 0xFFFE) {
        cur = r_prev[cur];
        if (cur == 0xFFFF) return 0;
    }
    return n->map[cur].id;
}


/* Plain variant: no exclusions (first choice, 7.1). */
uint16_t route_next_hop(mimeshi_node *n, uint16_t dst) {
    return route_next_hop_ex(n, dst, 0, 0);
}

/* First hop toward dst with, when known, the dest geo (7.1): the
 * graph decides when the destination lies in it; otherwise the
 * bridge-node rule routes toward the closest graph node near the
 * dest geo (15% candidate band, best route cost wins). geo (0,0)
 * = unknown: then a destination outside the graph is refused. */

uint16_t route_next_hop_geo_prev(mimeshi_node *n, uint16_t dst,
                                  uint16_t lat, uint16_t lon,
                                  const uint16_t *excl, uint8_t excl_cnt,
                                  uint16_t prev);
uint16_t route_next_hop_geo_prev(mimeshi_node *n, uint16_t dst,
                                  uint16_t lat, uint16_t lon,
                                  const uint16_t *excl, uint8_t excl_cnt,
                                  uint16_t prev) {
    (void)prev;    /* the prev-neighbourhood rule is removed; prev is excluded
                       via the excl array by the callers (schedule_candidates) */
    uint16_t nh = route_next_hop_ex_prev(n, dst, excl, excl_cnt, prev);
    if (nh) return nh;
    if (!lat && !lon) return 0;               /* no geo: no flood (7.1) */
    uint16_t bridge = route_bridge_node(n, lat, lon, excl, excl_cnt);
    if (!bridge) return 0;
    /* the route toward the bridge goes through the same computation as
     * the direct route (prev itself is excluded via the excl array,
     * 7.1); the old prev-neighbourhood rule is gone */
    return route_next_hop_ex_prev(n, bridge, excl, excl_cnt, prev);
}

/* Squared distance on the coarse 16-bit geo grid (5): comparison only,
 * so no sqrt. Longitude steps shrink with cos(latitude); at the NL
 * latitudes the factor is roughly 3/4 - folded into a fixed weight,
 * good enough to rank bridge-node candidates (7.1). */
/* Route cost toward dst on the own graph (7.1), or MC_DIST_INF when
 * unreachable. Shares the Dijkstra scratch with route_next_hop_ex: a
 * run settles every reachable node, so the cost of any candidate can
 * be read from r_dist afterwards. */
static uint32_t route_cost_to(mimeshi_node *n, uint16_t dst,
                              const uint16_t *excl, uint8_t excl_cnt) {
    if (!route_next_hop_ex(n, dst, excl, excl_cnt)) return MC_DIST_INF;
    int di = r_idx(n, dst);
    if (di < 0) return MC_DIST_INF;
    return r_dist[di];
}

/* Bridge-node rule (10.2, decided Oct 2026): the destination lies
 * outside the own graph but its geo is known. Candidates: all graph
 * nodes within 15% of the distance of the closest one (the margin
 * grows with the absolute distance - far from the destination there
 * is more freedom to pick a better path); among the candidates the
 * best route cost wins. Distance comparison on the square: 1.15^2 =
 * 1.3225, so the margin is +32.25% on d2. */
static uint16_t route_bridge_node(mimeshi_node *n, uint16_t lat, uint16_t lon,
                                  const uint16_t *excl, uint8_t excl_cnt) {
    /* Distance first, cost as tiebreak only (10.2, revised Oct 2026 - the
     * plateau finding). The old 15%-band + route-cost pass modelled a
     * future we cannot oversee: among geographically comparable
     * candidates it preferred the CHEAPEST - and the cheapest is
     * structurally the LOCAL one (few hops in the own graph),
     * cotangent to the destination, taking the message nowhere.
     * Measured (the 8h and 16h scale runs): the stranded out-of-graph
     * messages circled a flat plateau at ~130 km from their
     * destinations for every hop, while the constellation simulation showed the
     * median constellation holds a candidate at 16 km from the destination -
     * the knowledge was there, the cost pass kept choosing the
     * useless local step over it. The own entry is NEVER a candidate
     * (the self-loop fix stands). The rule now: the candidate CLOSEST
     * to the destination geo wins; the route cost decides only
     * between candidates at virtually the same distance (within ~5% on
     * the squared distance) - resolving true ties by quality, never
     * trading distance for cost. The local nodes near the destination
     * know better than our forward-thinking here. */
    /* REACHABILITY-FIRST (decided Oct 2026, the two hoptrace findings):
     * an unreachable candidate must NEVER win the bridge choice. The old
     * distance-first rule picked the closest constellation entry, and when the
     * Dijkstra then found no route toward it the WHOLE geo-route failed
     * (route_next_hop_geo_prev: if (!bridge) return 0) - while a slightly
     * farther but REACHABLE candidate stood ready. Measured (the two
     * trace-ones): spoor 1 - the source held the mast at dist2 6025 and
     * chose a LOCAL hop at 149364 because the geo-route failed; spoor 2 -
     * every near hop held dst itself at dist2 0 unreachable in the
     * confirmed graph, and the plans fell back to local wandering.
     *
     * The rule now: sort by distance as before, but a candidate only
     * sticks when a route toward it EXISTS (route_cost_to < INF). The
     * nearest unreachable candidate is skipped, not chosen; the first
     * reachable one on the distance ladder wins. The tiebreak (~5% on
     * the squared distance) stays for equal distances. This is one
     * Dijkstra per candidate - bounded by the constellation, the same cost the
     * old tiebreak already paid. */
    uint16_t pick = 0;
    uint32_t pick_d = 0, pick_cost = MC_DIST_INF;
    for (int i = 0; i < MC_MAP_NODES; i++) {
        if (!n->map[i].alive || !n->map[i].geo_lat) continue;   /* no geo: no candidate (5) */
        if (n->map[i].id == n->id) continue;     /* never the own entry (7.1) */
        uint32_t d = geo_dist2(n->map[i].geo_lat, n->map[i].geo_lon, lat, lon);
        if (!pick) {
            /* the first candidate only sticks when reachable */
            uint32_t c = route_cost_to(n, n->map[i].id, excl, excl_cnt);
            if (c >= MC_DIST_INF) continue;     /* unreachable: skip, not choose */
            pick = n->map[i].id; pick_d = d; pick_cost = c;
            continue;
        }
        if (d >= pick_d) continue;               /* farther than the pick: not a candidate */
        uint32_t c = route_cost_to(n, n->map[i].id, excl, excl_cnt);
        if (c >= MC_DIST_INF) continue;          /* unreachable: never wins */
        if (pick_d > d + d / 20u) {
            /* significantly closer AND reachable: wins outright */
            pick = n->map[i].id; pick_d = d; pick_cost = c;
        } else {
            /* within ~5%: a tie - the route cost decides */
            if (pick_cost == MC_DIST_INF)
                pick_cost = route_cost_to(n, pick, excl, excl_cnt);
            if (c < pick_cost) { pick = n->map[i].id; pick_d = d; pick_cost = c; }
        }
    }
    return pick;                                /* 0 when no reachable candidate at all */
}
