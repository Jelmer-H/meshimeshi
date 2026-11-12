/*
 * Meshimeshi protocol core, star_round.c. See core/internal.h for the
 * module map; design references point into docs/design.md.
 */
#include "internal.h"
#include <string.h>
#include <stdio.h>

/* ---- star round (5.3, proposal): the uniform convergence cadence --- */
/* One chunk per interval (starting value 15 min = 15 ticks of
 * 1 min), all three channels like the re-share round. Content first
 * (stars that changed since they were last forwarded, nearest hops
 * first, the own star at the front), catalogue in the remaining space
 * ((node id, digest) at 3 B per star). Hard bound: never more than
 * one chunk per node per interval, whatever happens (5.3). The own
 * star travels in full at least once per hour regardless of its
 * queue position (5.3): the guaranteed own-star round. */
#define MC_STAR_ROUND_TICKS   15u    /* 15 min interval in ticks (1 min each) */
#define MC_STAR_FLOOR_TICKS    5u     /* quiet-hour floor: 5 min (6e) - the interval
                                       may shorten to this when there is work and
                                       the channels are quiet */
#define MC_STAR_QUIET_TICKS    4u     /* a channel is quiet for the acceleration
                                       gate when its ka_idle counter stands at
                                       4+ ticks (4 min without any own
                                       identifying reception, 6e) */
#define MC_STAR_ROUNDS_PER_H  4u     /* 60 min / 15 min: own-star bound */
#define MC_STAR_CAT_RESERVE   30u     /* catalogue bytes reserved per round (5.3):
                                        content may fill the chunk, but the last
                                        30 B are for catalogue entries - the age
                                        maintenance that keeps the resting map
                                        alive must not be starved by a content
                                        backlog; 4 rounds/h x 30 B = 40 entries/h
                                        minimum on air */
#define F_STAR_CATALOGUE 0x10       /* chunk contains catalogue entries */
#define F_STAR_MISSED   0x20         /* chunk contains missed-star requests (5.3) */

/* Star age byte (5.3 heartbeat): the beat counter of the origin.
 * The round is also the beat driver of the own star: at the
 * guaranteed own-star round the beat advances and rides in the age
 * byte; neighbours derive the new digest without content. */
static uint8_t star_own_beat(mimeshi_node *n) { return n->star_beat; }


/* Star-forward usefulness (5.3, graph check): does forwarding the star
 * of `center` reach any neighbour that lies strictly FARTHER from the
 * center than we do? Hop distances over the known graph (map + link
 * store + our own edges), the same material routing uses - but pure
 * hop counting, no ETX: the question is reach, not cost.
 *
 * A BFS from the center over the routing-reachable edges gives the
 * distance of every known node. Our own distance: the minimum over
 * our neighbours (we are the virtual start edge, as in route_next_hop).
 * If no known neighbour lies strictly farther, the forwarding is an
 * echo back into the gradient: everyone we could reach got the star
 * from the same side we did. Unknown neighbours (no map slot, no
 * edges) count as FARTHER - absence of knowledge never silences the
 * forward (never prune on missing information, 5.3).
 *
 * Return 1 = useful (forward), 0 = useless (skip). */
static uint8_t star_dist[MC_MAP_NODES];     /* BFS distances, 0xFF = unreached */
static uint8_t star_fwd_useful(mimeshi_node *n, uint16_t center) {
    int ci = r_idx(n, center);
    if (ci < 0) return 1;                     /* center unknown to us: forward */
    for (int i = 0; i < MC_MAP_NODES; i++) star_dist[i] = 0xFF;

    /* BFS from the center. Edges: the symmetric threshold links of the
     * link store (both directions usable when the class is known) -
     * hop reach, not route cost, so undirected relaxation over both
     * nibbles with a non-zero class. */
    static uint16_t qbuf[MC_MAP_NODES];       /* BFS queue of map indices */
    uint16_t qh = 0, qt = 0;
    star_dist[ci] = 0;
    qbuf[qt++] = (uint16_t)ci;
    while (qh < qt) {
        uint16_t u = qbuf[qh++];
        if (star_dist[u] >= 0xFE) continue;
        uint16_t uid = n->map[u].id;
        for (int j = 0; j < MC_MAX_LINKS; j++) {
            if (!n->lnk[j].alive) continue;
            uint16_t o;
            if (n->lnk[j].a == uid) o = n->lnk[j].b;
            else if (n->lnk[j].b == uid) o = n->lnk[j].a;
            else continue;
            /* either direction known with a class = a hop for reach */
            uint8_t cf = (uint8_t)((n->lnk[j].q >> 4) & 0x0F);
            uint8_t cr = (uint8_t)(n->lnk[j].q & 0x0F);
            if (!cf && !cr) continue;         /* no measured direction: no hop */
            int oi = r_idx(n, o);
            if (oi < 0 || star_dist[oi] != 0xFF) continue;
            star_dist[oi] = (uint8_t)(star_dist[u] + 1);
            qbuf[qt++] = (uint16_t)oi;
        }
    }

    /* our distance: the best hop over our own edges (virtual start);
     * plus the trivial case: the center IS a direct neighbour (dist 1) */
    int my_dist = 0x7FFFFFFF;
    if (nbr_find(n, center) && nbr_find(n, center)->sym) my_dist = 1;
    for (int i = 0; i < MC_MAX_NEIGHBORS; i++) {
        mimeshi_neighbor *nb = &n->nbr[i];
        if (!nb->alive || !nb->sym) continue;
        int bi = r_idx(n, nb->id);
        if (bi < 0) continue;
        if (star_dist[bi] == 0xFF) continue;
        int d = star_dist[bi] + 1;
        if (d < my_dist) my_dist = d;
    }
    if (my_dist == 0x7FFFFFFF) return 1;       /* we are not on the graph of the
                                                  star: our neighbours are unknown
                                                  territory - forward */

    /* useful iff at least one neighbour lies STRICTLY farther */
    for (int i = 0; i < MC_MAX_NEIGHBORS; i++) {
        mimeshi_neighbor *nb = &n->nbr[i];
        if (!nb->alive || !nb->sym) continue;
        int bi = r_idx(n, nb->id);
        if (bi < 0) return 1;                 /* unknown neighbour: forward */
        if (star_dist[bi] == 0xFF) return 1;  /* not reached by the star's graph:
                                                 forward */
        if (star_dist[bi] > my_dist) return 1;/* strictly farther: reach to gain */
    }
    return 0;                                 /* everyone is closer or equal: echo */
}

/* Build and send one star-round chunk (5.3). Return value: number of
 * content stars carried (0 = the chunk still went out, catalogue-only). */

static int node_lonely(mimeshi_node *n);

static uint8_t star_round_send(mimeshi_node *n) {
    uint8_t p[230];
    uint16_t len = 10;
    p[0] = (uint8_t)PKT_STAR_ROUND;
    wr16(p + 1, n->id);
    p[3] = 0;                             /* catalogue count, filled below */
    p[4] = 0;                                   /* content star count, filled below */
    wr16(p + 5, n->geo_lat); wr16(p + 7, n->geo_lon);
    p[9] = 1;                                   /* firmware version (6.1 header shape) */

    /* --- content area: own star first, then nearest-first queue ---
     * The own star: the entry of n itself with its confirmed links,
     * serialised through the same canonical form. It is always a
     * candidate; with the own-star guarantee it goes in as soon as
     * the hour bound is reached, and its beat advances then. Under
     * the rotation (6.1) the guaranteed own-star round fires when the
     * rotation lands on the OWN HOME channel: the beat stays coupled
     * to the home channel, ~1x per hour at the 15-min starting value. */
    uint8_t carried = 0;
    uint8_t queued_left = 0;                /* stars packed but not yet fully carried */
    uint8_t rot_ch = (uint8_t)((n->home_ch + n->star_rot_ctr) % MC_N_CHANNELS);
    int home_round = (rot_ch == n->home_ch);
    /* guaranteed own-star round: every time the rotation lands on the
     * own home channel (~1x per 3 intervals = ~45 min at the 15-min
     * starting value - the once-per-hour bound of 6.1 stays honoured
     * as a lower bound). star_own_ctr counts home rounds since the
     * last guaranteed carry. */
    int own_due = home_round
                  && (uint8_t)(n->star_own_ctr + 1) >= (MC_STAR_ROUNDS_PER_H + 2) / 3;
    int own_star_changed = n->tbl_dirty;         /* prefix mutation pending = own star changed */
    if (own_due || own_star_changed) {
        /* The own star in the chunk entry layout: id, both classes,
         * tick, geo, links - via the canonical serialiser over the
         * own map entry. */
        uint8_t sbuf[MC_STAR_BUF]; uint16_t age_off;
        /* ensure the own map entry exists (geo, presence) */
        mimeshi_mapnode *om = map_find(n, n->id);
        if (!om) { om = map_add(n, n->id); }
        if (om) {
            om->geo_lat = n->geo_lat; om->geo_lon = n->geo_lon;
            uint16_t sl = mimeshi_star_serialize(n, n->id, sbuf, sizeof sbuf, &age_off);
            if (sl > 0 && len + sl <= 230) {
                if (own_due) { n->star_own_ctr = 0; n->star_beat++; }
                else n->star_own_ctr++;
                sbuf[age_off] = star_own_beat(n);   /* beat in the age byte (5.3) */
                memcpy(p + len, sbuf, sl);
                len = (uint16_t)(len + sl);
                carried++;
            }
        }
    } else {
        n->star_own_ctr++;
    }

    /* queue: stars whose CONTENT changed since they were last
     * carried, nearest hops first - prefix stars (direct neighbours)
     * before tail stars, in map order. The queue decision is the
     * content hash (5.3): a star goes back on the queue only when its
     * advertised content differs from the hash of its last carried
     * form; a mere age/refresh touch or sub-hysteresis EWMA movement
     * does NOT re-send it - the catalogue entry below is the cheap
     * confirmation. star_fwd_tick uses the same 16-bit time base as
     * arrived_tick, so wrap arithmetic is exact and no heuristic
     * guard is needed.
     *
     * No entry cap: stars pack until they do not fit the chunk - the
     * byte budget of the packet is the bound (5.3), not an entry
     * count inherited from the 6.1 chunk format. Thin stars pack many
     * per round (fast convergence where the air allows it), thick
     * stars few. What was carried is remembered per star
     * (star_fwd_tick + star_fwd_hash), so the next round simply
     * continues with what did not fit: a backlog drains over rounds,
     * bounded by the hard one-chunk-per-interval rule.
     *
     * One exception (5.3): the last MC_STAR_CAT_RESERVE bytes stay free
     * for catalogue entries. The catalogue is the heartbeat of the
     * resting network - without a guaranteed trickle of digests the
     * ages would run dry during a sustained content backlog and the
     * maintenance side of the round would starve. A star that does
     * not fit in the reserved space simply waits a round - the queue
     * drains, a round slower under pressure. */
    for (int pass = 0; pass < 2; pass++) {
        for (int i = 0; i < MC_MAP_NODES; i++) {
            mimeshi_mapnode *m = &n->map[i];
            if (!m->alive || m->id == n->id) continue;
            int is_prefix = nbr_find(n, m->id) != 0;
            if ((pass == 0) != (is_prefix != 0)) continue;   /* pass 0: prefix first */
            /* graph pruning (5.3): skip the star when every known
             * neighbour lies closer to (or level with) its center than
             * we do - the forward would be an echo back into the
             * gradient. Unknown neighbours never silence: no pruning
             * on missing information. The star stays in the map and
             * keeps its queue state; the missed list of a neighbour
             * that nevertheless wants it re-queues it explicitly. */
            /* pruning - unless a neighbour explicitly asked (6.1): a
             * fresh star_wanted overrides the gradient check for a few
             * intervals; the answer rides the next rounds. */
            int wanted = m->star_wanted != 0
                        && (uint32_t)(n->tick - m->star_wanted) < 2u * MC_STAR_ROUND_TICKS;
            if (!wanted && !star_fwd_useful(n, m->id)) continue;
            uint8_t sbuf[MC_STAR_BUF];
            uint16_t sl = mimeshi_star_serialize(n, m->id, sbuf, sizeof sbuf, 0);
            if (sl == 0 || len + sl > 230 - MC_STAR_CAT_RESERVE) continue;  /* full
                                          or would eat the catalogue reserve (6.1):
                                          stays queued for the next round */
            uint8_t h = mimeshi_star_content_hash(sbuf, sl, 3);
            if (m->star_fwd_tick != 0) {
                if (h == m->star_fwd_hash && (m->star_fwd_bits & 7) == 7)
                    continue;   /* unchanged and fully carried: done (6.1) */
                if (h != m->star_fwd_hash)
                    m->star_fwd_bits = 0;   /* content change: all channels again */
            }
            memcpy(p + len, sbuf, sl);
            len = (uint16_t)(len + sl);
            carried++;
            m->star_fwd_tick = n->tick;
            m->star_fwd_hash = h;
            queued_left++;                /* may stay queued: other channels pending */
        }
    }
    p[4] = carried;

    /* --- catalogue area: (node id, digest) in the remaining space ---
     * 3 B per star: what we have, in which version - the receiver
     * resets ages on a derivable match and prunes its own queue (5.3).
     * Own entry first (content hash of the own star + beat), then the
     * map, nearest first again. */
    uint16_t cat_cnt = 0;
    if (len + 3 <= 230) {
        uint8_t sbuf[MC_STAR_BUF];
        mimeshi_mapnode *om = map_find(n, n->id);
        if (om) {
            uint16_t sl = mimeshi_star_serialize(n, n->id, sbuf, sizeof sbuf, 0);
            if (sl > 0) {
                uint8_t h = mimeshi_star_content_hash(sbuf, sl, 3);
                wr16(p + len, n->id);
                p[len + 2] = mimeshi_star_digest(h, star_own_beat(n));
                len += 3; cat_cnt++;
            }
        }
    }
    /* Catalogue prioritisation (6.1, from the scenario-6 rest
     * measurement: the resting round carried ~40 triples of which most
     * confirmed knowledge that was fresh anyway). The limited space
     * goes to what is actually needed first; untriggered stars ride
     * only in the leftover space, in the old prefix-then-map order.
     * The selection re-runs every round: an entry that was skipped
     * keeps aging locally and enters a trigger band by itself, so
     * nothing is ever silently dropped.
     *
     * Trigger bands (checked per star, highest first):
     *  1. heartbeat due - the confirmation is older than one
     *     rotation cycle (3 intervals): the receiver's derivable
     *     match must keep arriving or the k-window drifts toward
     *     miss-requests; liveness detection depends on this.
     *  2. staleness - the effective age is above a third of the
     *     map-eviction bound: the age maintenance has to confirm it
     *     before the 24 h eviction (or route de-weighting) bites.
     *  3. fresh arrival - recently received as content, likely not
     *     yet confirmed by the neighbours' derivable matches: one
     *     catalogue triple settles the whole neighbourhood. */
    for (int pass = 0; pass < 2; pass++) {
        for (int i = 0; i < MC_MAP_NODES && len + 3 <= 230; i++) {
            mimeshi_mapnode *m = &n->map[i];
            if (!m->alive || m->id == n->id) continue;
            int is_prefix = nbr_find(n, m->id) != 0;
            if ((pass == 0) != (is_prefix != 0)) continue;
            uint8_t sbuf[MC_STAR_BUF];
            uint16_t sl = mimeshi_star_serialize(n, m->id, sbuf, sizeof sbuf, 0);
            if (sl == 0) continue;
            uint8_t h = mimeshi_star_content_hash(sbuf, sl, 3);
            /* beat of a foreign star: its forwarded age byte at arrival
             * (the origin's beat as it reached us); stored per map entry
             * as the low bits of arrived_tick are local - the beat
             * itself rides in the age byte of the star content, which
             * the digest derivation adds. Use the star's own age byte:
             * the serialised age IS the beat (6.1). */
            uint8_t beat = sbuf[3];
            wr16(p + len, m->id);
            p[len + 2] = mimeshi_star_digest(h, beat);
            len += 3; cat_cnt++;
        }
    }
    p[3] = (uint8_t)cat_cnt;               /* catalogue count in the header:
                                              the receiver knows where the
                                              catalogue ends and the missed
                                              area begins (the chunk-nr slot
                                              is free: star-round chunks are
                                              self-contained, no cursor) */
    if (cat_cnt) p[0] |= F_STAR_CATALOGUE;

    /* --- missed area (5.3): requests to specific neighbours ---
     * (neighbour id, n, star ids...) at 3 B + 2 B per star; the named
     * neighbour resets the last-carried state of those stars, so they
     * re-enter the content queue and travel with its next round. The
     * damping lives per entry: tries++ here, dropped after 2. */
    {
        uint16_t off = len;
        uint8_t carried_miss = 0;
        for (uint8_t i = 0; i < MC_STAR_MISS_MAX; i++) {
            if (!n->star_miss[i].alive) continue;
            if (n->star_miss[i].tries >= 2) {   /* damped out (5.3) */
                n->star_miss[i].alive = 0;
                continue;
            }
            if (!nbr_find(n, n->star_miss[i].nb)) {   /* neighbour gone (5.3):
                                                          the request could never
                                                          be answered - drop it */
                n->star_miss[i].alive = 0;
                continue;
            }
            /* group consecutive requests to the same neighbour: this
             * simple form packs one (nb, n=1, star) triple per entry;
             * the receiver accepts any grouping. */
            if (off + 5 > 230) break;           /* area full */
            wr16(p + off, n->star_miss[i].nb);
            p[off + 2] = 1;
            wr16(p + off + 3, n->star_miss[i].star);
            off += 5;
            n->star_miss[i].tries++;
            carried_miss++;
        }
        if (carried_miss) { p[0] |= F_STAR_MISSED; len = off; }
    }

    /* --- channel rotation with the backlog override (6.1) ---
     * At rest (no backlog: everything packed fits, nothing stays
     * queued) the round goes out on ONE channel: the rotation channel
     * (home + round counter) mod 3. The resting overhead is paid once
     * per interval instead of three times, the node is away from its
     * home channel at most one interval in three, and per channel only
     * the third of the nodes whose rotation it is transmits.
     * Under backlog (packed stars still await other channels, or the
     * queue did not fit) the round goes out on ALL THREE channels -
     * a backlog drains at full speed, exactly the old cadence: under
     * backlog every channel's throughput counts, and the full-spread
     * round sets all three carried bits in one blow. */
    {
        int backlog = (queued_left > 0) || (carried >= 8);
        /* lonely nodes spread in full (5.3): the accelerated round is the
         * discovery mechanism - neighbours on all home channels must hear
         * it, exactly like the old startup hello round */
        int spread = backlog || node_lonely(n);
        uint8_t mask = spread ? 7u : (uint8_t)(1u << rot_ch);
        for (uint8_t c = 0; c < MC_N_CHANNELS; c++) {
            if (!(mask & (1u << c))) continue;
            bound_send(n, c, p, len, t_data_us(len));
            ka_called(n, c);
        }
        /* carried-on-channel bits (6.1): a star leaves the queue only
         * when all three bits stand (or a content change resets them) */
        for (int i = 0; i < MC_MAP_NODES; i++) {
            mimeshi_mapnode *m = &n->map[i];
            if (!m->alive || m->id == n->id) continue;
            if (m->star_fwd_tick == n->tick) m->star_fwd_bits |= mask;
        }
        n->star_rot_ctr++;
        /* 6e: work remained when packed stars still await channels or the
         * queue did not fit - the quiet-hour acceleration reads this */
        n->star_backlog = (uint8_t)(queued_left > 0 || carried >= 8);
    }
    return carried;
}

/* Reception of a star-round chunk (5.3, receiver step).
 *
 * The CONTENT area carries stars in exactly the chunk entry layout of
 * 6.1 (id, classes, age, geo, links) - so the content processing goes
 * through the ordinary chunk entry path: a receiver that only
 * understands chunks still gets the full benefit of the content, and
 * the symmetry evidence (an owner naming us) keeps working (6.1).
 *
 * The CATALOGUE area carries (node id, digest) at 3 B: what the
 * sender has, in which version. For each entry about a star we know:
 *  - derivable match (digest = content_hash + beat + k, k <= window):
 *    the star is unchanged and its origin is alive; we advance the
 *    tick and RESET the travelled age of that star as received from
 *    this source (5.3): fresh-beats-old keeps working without a
 *    single content byte.
 *  - no fit: a real content change or a gap - the missed-list step
 *    (later sub-step of 5.3) takes this from here.
 *
 * We cannot see the sender's own star tick directly: the catalogue
 * entry for the sender's own star carries its digest, and a match
 * there also confirms the sender's liveness.
 *
 * The digest of a foreign star in OUR store: the content hash over
 * our canonical serialisation (age excluded) plus the beat we last
 * saw riding in the star's age byte - stored per map entry.
 */


/* Missed list (5.3): a catalogue mismatch (star unknown here, or a
 * digest that fits no tick) becomes a request to the ADVERTISER - not
 * a pull exchange: the request rides the own round, the answer rides
 * the neighbour's next round as ordinary content. Damped: at most 2
 * attempts per (neighbour, star); a star that arrives meanwhile clears
 * the entry. Bounded by MC_STAR_MISS_MAX; a full list drops the new
 * request (fresh-beats-old and the regular rounds remain the net). */
static void star_miss_add(mimeshi_node *n, uint16_t nb, uint16_t star) {
    if (nb == n->id) return;
    if (!nbr_find(n, nb)) return;              /* only direct neighbours (5.3:
                                                  channel-filtered by design) */
    for (uint8_t i = 0; i < MC_STAR_MISS_MAX; i++) {
        if (n->star_miss[i].alive && n->star_miss[i].nb == nb
            && n->star_miss[i].star == star) return;      /* already queued */
    }
    for (uint8_t i = 0; i < MC_STAR_MISS_MAX; i++) {
        if (!n->star_miss[i].alive) {
            n->star_miss[i].alive = 1;
            n->star_miss[i].nb = nb;
            n->star_miss[i].star = star;
            n->star_miss[i].tries = 0;
            return;
        }
    }
    /* full: drop silently - the damping rule keeps the list short in
     * practice; a persistent gap resurfaces via the next mismatch. */
}

/* A star we were missing arrived (content or derivable match): clear
 * every miss entry about it (5.3 damping: no re-request after fetch). */
static void star_miss_clear(mimeshi_node *n, uint16_t star) {
    for (uint8_t i = 0; i < MC_STAR_MISS_MAX; i++)
        if (n->star_miss[i].alive && n->star_miss[i].star == star)
            n->star_miss[i].alive = 0;
}


void star_round_rx(mimeshi_node *n, int8_t rssi, const uint8_t *pkt, uint16_t len) {
    if (len < 10) return;
    uint16_t owner = rd16(pkt + 1);      /* the advertiser: miss requests address it */

    /* Liveness (6.2, fixed Oct 2026, the warmup expiry finding): a star
     * round is an IDENTIFYING transmission of its advertiser - the id
     * is in the header, and every node sends one each round (~45 min).
     * A direct neighbour's own round therefore proves it is alive;
     * without this refresh the last_seen clock had no keepalive
     * trigger (hellos turn rare after startup, ACK1s only exist on
     * traffic, proofs are reactionless by design), and quiet
     * neighbour pairs expired at ~4 h on a living net - the
     * routing graph lost edges the knowledge graph still had. */
    if (owner != n->id) {
        mimeshi_neighbor *onb = nbr_find(n, owner);
        if (!onb && quality_ok(quality_of(rssi)))
            onb = nbr_add(n, owner);      /* discovery (5.3): a strongly heard
                                             round is the identifying
                                             transmission of a new neighbour;
                                             weak receptions add nothing */
        if (onb) {
            onb->last_seen = 0;
            onb->quality = quality_of(rssi);
        }
        hello_ack_wait(n, owner, rssi);   /* generalized ack trigger (5.3) */
    }

    /* --- content area: chunk entry layout, processed via chunk_rx ---
     * The header (10 B) is identical to a 6.1 chunk, and the entries
     * follow the same layout. chunk_rx() expects p[4] entries and
     * processes exactly that: reuse it wholesale by handing it the
     * same packet; the type nibble differs but chunk_rx does not
     * look at it.
     *
     * After the pass, the star semantics differ from a plain 6.1
     * chunk in exactly one field: the age byte of a star-round entry
     * carries the ORIGIN HEARTBEAT TICK (5.3), not the travelled age.
     * chunk_rx stored it into age_ticks; we move it to star_beat and
     * reset age_ticks to 0 - the star arrived NOW, its travelled age
     * starts here (6.2 semantics), and the beat is the liveness. */
    uint16_t cend = 10;                      /* end of the content area */
    for (uint8_t e = 0; e < pkt[4] && cend + 9 <= len; e++)
        cend = (uint16_t)(cend + 9 + 3 * pkt[cend + 8]);   /* walk the content stars */

chunk_rx_mode(n, rssi, pkt, cend, 1);   /* entries only: the catalogue and
                                               missed bytes are not entries */
    uint16_t rx_star[MC_CHUNK_MAX_ENTRIES];     /* content stars of THIS packet */
    uint8_t rx_star_n = 0;
    {
        uint16_t off = 10;
        for (uint8_t e = 0; e < pkt[4] && off + 9 <= cend; e++) {
            uint16_t eid = rd16(pkt + off);
            if (eid != n->id) {                    /* our own star is not in the map */
                mimeshi_mapnode *em = map_find(n, eid);
                if (em) {
                    /* fresh beats old (6.1): a form that lags more than
                     * the k-window behind the stored beat is a stale
                     * re-send - the newer knowledge stands, only the
                     * request is settled. */
                    int16_t d = (int16_t)((int16_t)em->star_beat - (int16_t)pkt[off + 3]);
                    if (!(em->adv_q
                          && (d > (int16_t)MC_STAR_K_MAX || d < -(int16_t)MC_STAR_K_MAX))) {
                        if ((int16_t)pkt[off + 3] != (int16_t)em->star_beat)
                            em->star_beat_seen = n->tick;   /* liveness: the origin beat advanced (6.2) */
                        em->star_beat = pkt[off + 3];  /* the beat as it traveled (6.1) */
                        em->age_ticks = 0;              /* arrived NOW: travelled age starts here */
                        em->arrived_tick = n->tick;
                    }
                }
                star_miss_clear(n, eid);            /* a missing star arrived:
                                                     its requests are settled (6.1) */
            }
            if (rx_star_n < MC_CHUNK_MAX_ENTRIES) rx_star[rx_star_n++] = eid;
            off += 9 + 3 * pkt[off + 8];
        }
    }

    /* --- catalogue area: after the content stars (5.3) ---
     * The number of content stars is p[4]; the entries follow the
     * chunk entry layout, so the catalogue starts after the last
     * content star. (n, digest) triples fill the rest. */
    uint16_t off = cend;                       /* the content walk ended here */
    if (!(pkt[0] & F_STAR_CATALOGUE)) { off = len; }
    uint8_t cat_n = pkt[3];                    /* catalogue count (header) */
    uint16_t cat_end = off + 3 * cat_n;
    if (cat_end > len) { cat_end = len; }      /* malformed: clip */
    for (; off + 3 <= cat_end; off += 3) {
        uint16_t sid = rd16(pkt + off);
        uint8_t digest = pkt[off + 2];
        if (sid == n->id) continue;              /* our own star: we are the origin */
        int in_content = 0;                       /* this packet already carried it */
        for (uint8_t i = 0; i < rx_star_n; i++)
            if (rx_star[i] == sid) { in_content = 1; break; }
        if (in_content) continue;                 /* fresh content beats the
                                                     catalogue mismatch: no
                                                     request (5.3) */
        mimeshi_mapnode *m = map_find(n, sid);
        if (!m) {
            /* the neighbour advertises a star we do not hold: either
             * a genuine gap (never had it) or OUR eviction (the constellation
             * recycle, 6.2). The taboo (6.1) sorts these out: an
             * evicted star was a policy choice - requesting it back
             * is the constellation-pressure cycle. Only genuine gaps ask. */
            /* A catalogue entry means the neighbour HOLDS the star (a
             * missed advertisement): a legitimate ask - ALSO at a
             * full constellation, because the answer is judged on arrival by
             * the admission competition (it may be better than what
             * we have; if not, it lands on the taboo, 6.1). The
             * taboo still blocks the already-judged. Link claims
             * never create requests: they carry no 'the neighbour has
             * this star' promise - only the catalogue does. */
            if (!ev_taboo_check(n, sid))
                star_miss_add(n, owner, sid);
            continue;
        }
        /* our version: content hash over our canonical bytes (age
         * excluded), beat = the heartbeat as it reached us. */
        uint8_t sbuf[MC_STAR_BUF];
        uint16_t sl = mimeshi_star_serialize(n, sid, sbuf, sizeof sbuf, 0);
        if (sl == 0) continue;
        uint8_t h = mimeshi_star_content_hash(sbuf, sl, 3);
        uint8_t our_beat = m->star_beat;        /* origin beat (5.3):
                                                   separate from age_ticks, which
                                                   keeps its 6.2 meaning */
        int k = mimeshi_star_beat_match(digest, h, our_beat, MC_STAR_K_MAX);
        if (k != -1) {
            /* derivable match (6.1): unchanged, origin alive. Advance
             * the beat and reset the travelled age - the knowledge is
             * confirmed without content. A star we were missing is
             * hereby settled: clear its miss entries. A lagging
             * advertisement (k > 16: a slower route) confirms the same
             * content: only the age refreshes, the newer stored beat
             * stands (the origin has already reached it). */
            if (k > 0 && k <= (int)MC_STAR_K_MAX) {
                m->star_beat = (uint8_t)(our_beat + (uint8_t)k);
                m->star_beat_seen = n->tick;   /* liveness: the origin beat advanced (6.2) */
            }
            m->age_ticks = 0;                   /* travelled age restarts (6.2) */
            m->arrived_tick = n->tick;          /* fresh as of NOW (6.2) */
            star_miss_clear(n, sid);
        } else {
            /* no tick fits: a real content change upstream, a hash
             * collision - or a star WE evicted (the neighbour still has
             * it). The eviction taboo (6.1) sorts this out: a star in
             * the taboo was our policy choice, requesting it back is
             * the constellation-pressure cycle - no request. Genuine gaps (not
             * in the taboo) ask as before. */
            /* No beat fits. Only a COMPLETE local form (star_beat set:
             * we once received the origin's full star) can be outdated
             * - a missed advertisement of a newer form: ask, the
             * answer is judged on arrival (6.2). A PRESENCE SHELL
             * (star_beat 0: only an ID from a link claim, no form)
             * mismatches every digest BY CONSTRUCTION - the shell never
             * carried the origin's links, so its bare bytes cannot
             * match the advertised full form. That mismatch is a
             * measurement artefact, not news: asking would cycle
             * forever (measured: the entire nofit storm, 39k over 3 h,
             * was shells). The origin's guaranteed own-star round
             * delivers the full form; from then the entry is a real
             * form and a future nofit is a genuine missed
             * advertisement. The taboo blocks the already-judged. */
            if (m->star_beat != 0 && !ev_taboo_check(n, sid))
                star_miss_add(n, owner, sid);
        }
    }

    /* --- missed area (5.3): requests addressed to US ---
     * A neighbour is missing a star (or saw a content change): reset
     * the last-carried state so the star re-enters the content queue
     * and travels with our next round - no exchange, no ack, the
     * regular cadence is the answer (5.3). */
    if (pkt[0] & F_STAR_MISSED) {
        uint16_t moff = cat_end;                /* where the catalogue ended */
        for (; moff + 3 <= len;) {
            uint16_t nb = rd16(pkt + moff);
            uint8_t cnt = pkt[moff + 2];
            moff += 3;
            if (moff + 2 * cnt > len) break;    /* malformed: ignore the rest */
            if (nb == n->id) {
                for (uint8_t i = 0; i < cnt; i++) {
                    uint16_t star = rd16(pkt + moff + 2 * i);
                    mimeshi_mapnode *m = map_find(n, star);
                    if (m) {
                        m->star_fwd_tick = 0;   /* as never carried: queued again */
                        m->star_fwd_hash = 0;
                        m->star_wanted = n->tick;   /* explicit ask: the
                                        pruning yields for a few
                                        intervals (6.1) */
                    }
                }
            }
            moff += 2 * cnt;
        }
    }
}

/* Round timer (6.1): per-node phase offset; the interval is the hard
 * bound - one chunk per node per interval.
 *
 * Quiet-hour acceleration (6e): when the last round left content work
 * (star_backlog) AND every channel has been quiet for MC_STAR_QUIET_TICKS
 * (no identifying reception - the ka_idle counters carry the CAD-free
 * occupancy measure), the next interval halves, down to the 5-min floor.
 * The convergence of a residual backlog then runs at up to 3x speed in
 * quiet periods without spending any airtime in busy ones: at occupancy
 * the full interval stands, and the acceleration self-clears at rest
 * (no backlog, no shortening). The hard bound is untouched - the round
 * still sends at most one chunk, just sooner. */
/* Lonely node (5.3): no confirmed neighbour. Drives the accelerated
 * full-spread star round - the replacement of the old startup hello
 * rounds. A fresh node is always lonely; the first confirmation ends
 * the acceleration (the normal rotation takes over). The cadence
 * itself doubles per unconfirmed round (star_lonely_ctr, 5.3) so the
 * acceleration decays even without confirmation. */
static int node_lonely(mimeshi_node *n) {
    for (int i = 0; i < MC_MAX_NEIGHBORS; i++)
        if (n->nbr[i].alive && n->nbr[i].sym) return 0;
    return 1;
}

void star_round_tick(mimeshi_node *n, void *ud) {
    (void)ud;
    n->star_round_sched = 0;
    if (n->tbl_dirty) { n->tbl_dirty = 0; }       /* the round carries mutations (6.1) */
    star_round_send(n);
    n->star_round_sched = 1;
    /* Three-way closure, re-anchored (5.3): the star round is the
     * identifying transmission that collects the generalized acks; at
     * the end of the ack window one COLLECTIVE proof ack confirms all
     * responders of this round - but only when acks actually arrived
     * (a silent round has no agreement to close and no airtime to
     * spend). Acks arriving after the window stay pending and close
     * with the next round. */
    if (n->proof_ack_pend) {
        n->proof_ack_pend = 0;
        n->proof_ack_best_rq = 0;
        n->timer(n, MC_HELLO_ACK_WIN_US + MC_HELLO_ACK_JITTER_US + t_ack_us(),
                 proof_ack_fire, 0);
    }
    /* lonely ladder (5.3): the acceleration is the discovery mechanism,
     * but it needs the old hello's doubling brake - a node that keeps
     * failing to confirm (the startup storm, or an isolated node whose
     * links never clear the quality gate) must not hammer full spread
     * at the top rate forever. The spacing doubles per round without a
     * confirmed neighbour, up to the normal cadence; the first
     * confirmation resets it. Measured without the brake: 5 isolated
     * nodes produced 7347 bound violations over 120 h at 480 nodes. */
    uint32_t iv;
    if (node_lonely(n)) {
        if (n->star_lonely_ctr == 0) n->star_lonely_ctr = MC_STAR_LONELY_TICKS;
        iv = n->star_lonely_ctr;
        if (n->star_lonely_ctr < MC_STAR_ROUND_TICKS) {
            uint16_t d = (uint16_t)(n->star_lonely_ctr * 2);
            n->star_lonely_ctr = (uint8_t)(d > MC_STAR_ROUND_TICKS
                                           ? MC_STAR_ROUND_TICKS : d);
        }
    } else {
        n->star_lonely_ctr = 0;      /* integrated: the ladder is done */
        iv = MC_STAR_ROUND_TICKS;
        if (n->star_backlog) {
            uint8_t quiet = 1;
            for (uint8_t c = 0; c < MC_N_CHANNELS; c++)
                if (n->ka_idle[c] < MC_STAR_QUIET_TICKS) { quiet = 0; break; }
            if (quiet && iv / 2 >= MC_STAR_FLOOR_TICKS) iv /= 2;
        }
    }
    n->timer(n, iv * MC_KEEPALIVE_TICK_US
             + (n->rand(n) % MC_KEEPALIVE_TICK_US), star_round_tick, 0);
}
