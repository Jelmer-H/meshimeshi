/*
 * Meshimeshi protocol core, map.c. See core/internal.h for the
 * module map; design references point into docs/design.md.
 */
#include "internal.h"
#include <string.h>
#include <stdio.h>

/* ---------------- 2-hop map and link store (6.1, 6.3) --------
 * The map holds every node we know: direct neighbours (alive in the
 * neighbor table) and 2-hop nodes from table chunks. The link store
 * holds the advertised links between map nodes (per direction a 3-bit
 * class). Both are bounded (11.2): full = the entry is dropped with a
 * message, never silently. */

/* Effective age of a map entry in full ticks (1 min each),
 * uncapped for internal comparisons (6.2); on air the age byte caps at
 * 255 (~4 h) - anything older advertises simply as expired. */
uint32_t map_age_ticks(mimeshi_node *n, mimeshi_mapnode *m) {
    return (uint32_t)m->age_ticks + (n->tick - m->arrived_tick);
}


/* Eviction of a map entry (6.2): the entry goes, and with it every
 * stored link that touches it - the link store stays consistent with
 * the map, and a link is only usable while both endpoints stand. */
/* Eviction taboo (6.1): the ring of recently evicted star IDs.
 * A catalogue mismatch for a tabooed star creates no miss request -
 * the eviction was a policy choice (constellation recycle or expiry), and
 * requesting the star back is a cycle under constellation pressure (measured:
 * 21,093 requests over 6 h on the islands pressure test, all cycle).
 * An ID that re-enters the map leaves the taboo (freshly learned
 * knowledge is wanted knowledge). */
#define MC_EV_TABOO 128u          /* the taboo set size (6.1): big enough to
                                     hold every star a small constellation evicts per
                                     cycle - at the 480-node scale the world
                                     minus the constellation is ~224, and the ring
                                     overwrite only retires the oldest */
static void ev_taboo_remember(mimeshi_node *n, uint16_t id) {
    /* no overwrite: a rejection stands until the star is stored
     * again (ev_taboo_forget). The array is the constellation size, so every
     * possible rejection fits; a duplicate remember is a no-op. */
    for (int i = 0; i < (int)(sizeof n->ev_taboo / sizeof n->ev_taboo[0]); i++) {
        if (n->ev_taboo[i] == id) return;         /* already judged */
        if (n->ev_taboo[i] == 0) { n->ev_taboo[i] = id; return; }
    }
}
int ev_taboo_check(mimeshi_node *n, uint16_t id) {
    /* The taboo is a memory of the admission competition, and the
     * competition runs at constellation pressure (6.2/6.1): at a NON-full constellation
     * the rejections are stale - the pressure that justified them is
     * gone. The free-slot policy (decided, Oct 2026):
     *  - 0 free slots: the taboo is active (the measured cycle
     *    protection, 50,893 blocked re-asks on the islands test);
     *  - 1-3 free slots: the taboo is TREATED AS EMPTY at the
     *    request gates (rejected stars may ask again - the answers
     *    fill the free slots; the physical list stands, so when the
     *    constellation refills, the old judgments apply again without a new
     *    judgment cycle). Temporary suboptimal admissions at the rim
     *    of our area are corrected by the complete forms that ride
     *    the origin's guaranteed rounds;
     *  - >= 4 free slots: the taboo is FLUSHED entirely - the pressure
     *    is gone, everyone competes fresh; new judgments come with
     *    the next full constellation.
     * Implemented here so every call site (both miss gates) shares
     * the same view. */
    int free = 0;
    for (int i = 0; i < MC_MAP_NODES; i++) {
        if (!n->map[i].alive) {
            free++;
            if (free >= 4) {
                for (int j = 0; j < (int)(sizeof n->ev_taboo / sizeof n->ev_taboo[0]); j++)
                    n->ev_taboo[j] = 0;
                n->ev_taboo_idx = 0;
                return 0;
            }
        }
    }
    if (free > 0) return 0;               /* 1-3 free: treated as empty, kept */
    for (int i = 0; i < (int)(sizeof n->ev_taboo / sizeof n->ev_taboo[0]); i++)
        if (n->ev_taboo[i] == id) return 1;
    return 0;
}
void ev_taboo_forget(mimeshi_node *n, uint16_t id) {
    for (int i = 0; i < (int)(sizeof n->ev_taboo / sizeof n->ev_taboo[0]); i++)
        if (n->ev_taboo[i] == id) n->ev_taboo[i] = 0;
}

void map_evict(mimeshi_node *n, mimeshi_mapnode *m) {
    /* Ownership-aware link cleanup (6.1, from the islands nofit
     * finding): evicting P may only remove the links P OWNS - the
     * per-link ownership bit (lnk_owner) decides. The links of OTHER
     * origins that merely name P as a peer are form material of the
     * OWNING origin's star: removing them broke the owner's
     * serialised form (storage-as-received violated), which
     * mismatched every catalogue digest forever - the 39k nofit
     * cycle measured under constellation pressure. A link to an evicted peer
     * stays as the owner's claim (routing skips dead endpoints
     * anyway); it leaves with its OWNER. */
    for (int i = 0; i < MC_MAX_LINKS; i++) {
        if (!n->lnk[i].alive) continue;
        if (n->lnk[i].a != m->id && n->lnk[i].b != m->id) continue;
        if (lnk_owner(n->lnk[i].a, n->lnk[i].b) == m->id)
            n->lnk[i].alive = 0;        /* P owned this link: it goes with P */
    }
    ev_taboo_remember(n, m->id);       /* the taboo: this was a choice (6.1) */
    m->alive = 0;
}

mimeshi_mapnode *map_find(mimeshi_node *n, uint16_t id) {
    for (int i = 0; i < MC_MAP_NODES; i++)
        if (n->map[i].alive && n->map[i].id == id) return &n->map[i];
    return 0;
}

int r_idx(mimeshi_node *n, uint16_t id);
uint32_t geo_dist2(uint16_t lat1, uint16_t lon1, uint16_t lat2, uint16_t lon2);
/* Constellation recycle at the cap (6.2, decided Oct 2026): the victim is
 * chosen by ROUTING VALUE, not freshness. Two axes:
 *  - hop distance from us to the star (closer stays: cheaper to
 *    re-confirm, more likely on real routes);
 *  - the star's total link length: the sum of the squared geo
 *    distances (geo_dist2) of every stored link the star takes part
 *    in - HIGHER is better (removing a hub fragments the map more
 *    than removing a leaf); the LOWEST total goes first.
 * Tie order: hop distance, then link length, then slot index. */
static uint8_t ev_hop[MC_MAP_NODES];       /* hop distance from the own node */

static void evict_hop_fill(mimeshi_node *n) {
    /* BFS from the OWN node over the own confirmed edges plus the link
     * store (hop reach, same relaxation as the forward pruning). */
    static uint16_t qbuf[MC_MAP_NODES];
    for (int i = 0; i < MC_MAP_NODES; i++) ev_hop[i] = 0xFF;
    uint16_t qh = 0, qt = 0;
    /* virtual start: our confirmed neighbours enter at distance 1 */
    for (int i = 0; i < MC_MAX_NEIGHBORS; i++) {
        mimeshi_neighbor *nb = &n->nbr[i];
        if (!nb->alive || !nb->sym) continue;
        int bi = r_idx(n, nb->id);
        if (bi < 0) continue;
        if (ev_hop[bi] != 0xFF) continue;
        ev_hop[bi] = 1;
        qbuf[qt++] = (uint16_t)bi;
    }
    int own_i = r_idx(n, n->id);
    if (own_i >= 0) { ev_hop[own_i] = 0; qbuf[qt++] = (uint16_t)own_i; }
    while (qh < qt) {
        uint16_t u = qbuf[qh++];
        uint16_t uid = n->map[u].id;
        for (int j = 0; j < MC_MAX_LINKS; j++) {
            if (!n->lnk[j].alive) continue;
            uint16_t o;
            if (n->lnk[j].a == uid) o = n->lnk[j].b;
            else if (n->lnk[j].b == uid) o = n->lnk[j].a;
            else continue;
            uint8_t cf = (uint8_t)((n->lnk[j].q >> 4) & 0x0F);
            uint8_t cr = (uint8_t)(n->lnk[j].q & 0x0F);
            if (!cf && !cr) continue;
            int oi = r_idx(n, o);
            if (oi < 0 || ev_hop[oi] != 0xFF) continue;
            ev_hop[oi] = (uint8_t)(ev_hop[u] + 1);
            qbuf[qt++] = (uint16_t)oi;
        }
    }
}

/* Total link length the star takes part in: the sum of geo_dist2 over
 * every alive stored link with this star as one endpoint (links whose
 * other endpoint has a geo; geo-less links count 0). */
static uint32_t evict_link_len(mimeshi_node *n, uint16_t id) {
    int mi = r_idx(n, id);
    uint16_t gl = 0, go = 0;
    if (mi >= 0) { gl = n->map[mi].geo_lat; go = n->map[mi].geo_lon; }
    uint32_t total = 0;
    for (int j = 0; j < MC_MAX_LINKS; j++) {
        if (!n->lnk[j].alive) continue;
        uint16_t o;
        if (n->lnk[j].a == id) o = n->lnk[j].b;
        else if (n->lnk[j].b == id) o = n->lnk[j].a;
        else continue;
        int oi = r_idx(n, o);
        if (oi < 0) continue;
        if (!n->map[oi].geo_lat || !gl) continue;   /* no geo: not computable */
        total += geo_dist2(gl, go, n->map[oi].geo_lat, n->map[oi].geo_lon);
    }
    return total;
}

/* map_add: store/refresh the entry for `id`. `admit_geo` carries the
 * star's OWN geo when this is a CONTENT arrival (chunk_rx read it from
 * the packet); 0 for presence shells (link claims) - shells never
 * compete for a slot, they are only IDs. */
mimeshi_mapnode *map_add_geo(mimeshi_node *n, uint16_t id,
                                    uint16_t glat, uint16_t glon);
mimeshi_mapnode *map_add(mimeshi_node *n, uint16_t id) {
    return map_add_geo(n, id, 0, 0);
}
mimeshi_mapnode *map_add_geo(mimeshi_node *n, uint16_t id,
                                    uint16_t glat, uint16_t glon) {
    mimeshi_mapnode *m = map_find(n, id);
    if (m) return m;
    for (int i = 0; i < MC_MAP_NODES; i++)
        if (!n->map[i].alive) {
            ev_taboo_forget(n, id);   /* re-entering the map ends the taboo (6.1) */
            n->map[i].alive = 1; n->map[i].id = id;
            n->map[i].star_beat_seen = n->tick;   /* liveness starts now (6.2):
                                                     no tick known yet, the first
                                                     advance (or the threshold)
                                                     decides from here */
            n->map[i].geo_lat = 0; n->map[i].geo_lon = 0;
            n->map[i].quality = 0; n->map[i].age_ticks = 255;
            n->map[i].arrived_tick = n->tick;
            return &n->map[i];
        }
    /* Constellation full (6.2, decided Oct 2026): ADMISSION COMPETITION - the
     * arriving star takes part. Score the newcomer (hop distance,
     * total link length) against the worst entry in the constellation:
     *  - the newcomer is the worst -> NOT stored, directly on the
     *    taboo (judged and rejected, 6.1): the constellation is untouched. A
     *    distant/thin star can no longer displace routing capital,
     *    and no half-filled shell of a rejected star circulates
     *    (the shell-mismatch cycle was the request storm of the
     *    islands pressure test).
     *  - the newcomer wins -> it takes the victim's slot; the victim
     *    goes on the taboo (map_evict).
     * Freshness is the EXPIRY trigger (24 h), never the recycle
     * choice: at a full constellation freshness is a rotation-timing lottery. */
    evict_hop_fill(n);
    int victim = -1;
    uint8_t v_hop = 0; uint32_t v_len = 0;
    int v_disc = 0;                        /* victim is NOT connected to our graph */
    for (int i = 0; i < MC_MAP_NODES; i++) {
        if (!n->map[i].alive) continue;
        /* direct neighbours never: they live in the prefix */
        if (nbr_find(n, n->map[i].id)) continue;
        uint8_t h = ev_hop[i];
        int disc = (h == 0xFF);            /* unreachable in our graph: a knowledge
                                              island - stored (first contact may
                                              arrive disconnected; the graph grows
                                              onto it later), but the FIRST OUT at
                                              the cap: routing capital beats
                                              stranded knowledge (decided Oct 2026).
                                              The link length never saves an island:
                                              a richly linked island is still an
                                              island - its links do not route. */
        if (h == 0xFF) h = 250;
        uint32_t l = evict_link_len(n, n->map[i].id);
        if (victim < 0
            || (disc && !v_disc)            /* an island always beats a connected entry */
            || (disc == v_disc && (h > v_hop || (h == v_hop && l < v_len)))) {
            victim = i; v_hop = h; v_len = l; v_disc = disc;
        }
    }
    if (victim < 0) {                            /* only direct neighbours in the map */
        printf("[core] node %u: map full, entry %u dropped\n", n->id, id);
        return 0;
    }
    /* The newcomer's score, honestly: a CONTENT arrival carries the
     * origin's own geo - the geo distance to OURS is the routing-
     * relevant measure for the head-to-head (physically near us is
     * near in hops too, whatever the current store happens to know).
     * Presence shells (glat 0) never compete: they have no form to
     * score, so at a full constellation they are not stored at all - a shell
     * is only an ID, losing it costs nothing. */
    if (!glat) {
        return 0;                       /* shell at a full constellation: not stored */
    }
    {
        int own_i = r_idx(n, n->id);
        if (own_i >= 0 && n->map[victim].geo_lat) {
            uint32_t n_dist = geo_dist2(glat, glon,
                                       n->map[own_i].geo_lat, n->map[own_i].geo_lon);
            uint32_t v_dist = geo_dist2(n->map[victim].geo_lat, n->map[victim].geo_lon,
                                        n->map[own_i].geo_lat, n->map[own_i].geo_lon);
            if (n_dist >= v_dist) {
                /* the newcomer is not closer than the victim: reject.
                 * Judged on its real form - the taboo holds this. */
                ev_taboo_remember(n, id);
                return 0;
            }
        }
    }
    map_evict(n, &n->map[victim]);
    ev_taboo_forget(n, id);   /* the newcomer is wanted - never tabooed (6.1) */
    n->map[victim].alive = 1; n->map[victim].id = id;
    n->map[victim].geo_lat = 0; n->map[victim].geo_lon = 0;
    n->map[victim].quality = 0; n->map[victim].age_ticks = 255;
    n->map[victim].arrived_tick = n->tick;
    return &n->map[victim];
}

/* Link between two map nodes, endpoints normalized a < b. */
static mimeshi_link *lnk_find(mimeshi_node *n, uint16_t a, uint16_t b) {
    if (a > b) { uint16_t t = a; a = b; b = t; }
    for (int i = 0; i < MC_MAX_LINKS; i++)
        if (n->lnk[i].alive && n->lnk[i].a == a && n->lnk[i].b == b) return &n->lnk[i];
    return 0;
}

static mimeshi_link *lnk_add(mimeshi_node *n, uint16_t a, uint16_t b) {
    mimeshi_link *l = lnk_find(n, a, b);
    if (l) return l;
    if (a > b) { uint16_t t = a; a = b; b = t; }
    for (int i = 0; i < MC_MAX_LINKS; i++)
        if (!n->lnk[i].alive) {
            n->lnk[i].alive = 1; n->lnk[i].a = a; n->lnk[i].b = b;
            return &n->lnk[i];
        }
    printf("[core] node %u: link store full, link %u-%u dropped\n", n->id, a, b);
    return 0;
}

/* Content stars per round packet (6.1): 8 x (9 B + links) fits 230 B. */

/* Table mutation feed (6.1): the local table counter marks the own
 * star as changed; the next round carries it as content. Nothing is
 * scheduled here - the round timer is the only send path (6.1 hard
 * bound). */
void tbl_mark_dirty(mimeshi_node *n) {
    n->tbl_dirty = 1;
}

/* Neighbour addition (6.1): the new neighbour is a content change of
 * the own star - the next round carries it. */
/* Neighbour confirmation (6.1): the own star gains the link - and the
 * newcomer acceleration (6.1) turns the queue for it: the forward
 * state of all prefix stars resets, so the stars of the direct
 * neighbours re-enter the content queue and travel with the coming
 * rounds. The old cadence answered a newcomer with a delta plus a
 * full prefix; the star-round translation is the queue reset - zero
 * extra bytes, the same stars that round anyway, only prioritised. */
void tbl_nbr_added(mimeshi_node *n, uint16_t id) {
    (void)id;
    tbl_mark_dirty(n);
    for (int i = 0; i < MC_MAP_NODES; i++) {
        mimeshi_mapnode *m = &n->map[i];
        if (!m->alive || m->id == n->id) continue;
        if (!nbr_find(n, m->id)) continue;         /* prefix stars only */
        m->star_fwd_tick = 0;                      /* as never carried */
        m->star_fwd_hash = 0;
    }
}

/* ---------------- table chunks (6.1) ----------------
 * Prefix chunk, node-centric and self-contained (6.1):
 *   header: type(1) owner-id(2) chunk-nr(1) n-entries(1)
 *           owner-geo(4) fw-version(1)                      = 10 B
 *   entry:  node-id(2) class(1) age(1) geo(4) link-count(1) = 9 B
 *   link:   endpoint-id(2) class-entry->endpoint(4 bits)
 *           class-endpoint->entry(4 bits)                   = 3 B
 * Only confirmed symmetric neighbours are advertised (decided:
 * no "heard only" in the chunks). The entry class is the owner's
 * measured quality of the link (EWMA average, 3 bits); the fading
 * asymmetry per direction (+-5 dB) is below the class width, so one
 * class stands for both directions of a direct link. The link
 * records under an entry carry the links between the entry-node and
 * our other confirmed neighbours, with the classes per direction as
 * they are known in the link store. */
/* Direction-aware access to the link store: nibbles keyed to the
 * sorted endpoints (a < b): high nibble = a->b, low = b->a. */
uint8_t lnk_class(mimeshi_node *n, uint16_t from, uint16_t to) {
    mimeshi_link *l = lnk_find(n, from, to);
    if (!l) return 0xFF;                        /* unknown */
    uint16_t a = from < to ? from : to;
    return (from == a) ? (uint8_t)((l->q >> 4) & 0x0F) : (uint8_t)(l->q & 0x0F);
}

void lnk_set_class(mimeshi_node *n, uint16_t from, uint16_t to, uint8_t cls) {
    if (cls > 7) return;                        /* unknown stays unknown */
    mimeshi_link *l = lnk_add(n, from, to);
    if (!l) return;
    uint16_t a = from < to ? from : to;
    uint8_t cur = (from == a) ? (uint8_t)(l->q >> 4) : (uint8_t)(l->q & 0x0F);
    if (cls == 0) {
        /* A zero in an advertisement is NOT a measurement - it is an
         * absent one (the owner has not measured/published that
         * direction yet, 6.1). Writing it would WIPE a class we
         * already learned through our own exchange (the hard-learned
         * lesson of the persistent zero-directions: every star round
         * re-published the not-yet-measured direction as 0 and reset
         * whatever the receiver knew). A zero never overwrites; the
         * direction simply keeps its current state. */
        return;
    }
    if (cls == 1 && cur == 0) {
        /* The INITIAL CLASS (decided Oct 2026): a class-1 advertisement
         * on a direction we have at 0 is often the owner's FIRST
         * published measurement of a link we know works (the proof
         * threshold guarantees it: no hello-ack or proof-ack exists
         * on a link whose direction failed the gate). Writing class 1
         * makes the direction IMMEDIATELY routable (ETX 228 - the
         * most expensive, honest for an unmeasured link) instead of
         * dead until our own measurement arrives. Real measurements
         * (2..7) always overwrite as before. */
        if (from == a) l->q = (uint8_t)((l->q & 0x0F) | (1 << 4));
        else           l->q = (uint8_t)((l->q & 0xF0) | 1);
        return;
    }
    if (from == a) l->q = (uint8_t)((l->q & 0x0F) | (cls << 4));
    else           l->q = (uint8_t)((l->q & 0xF0) | cls);
}

int r_idx(mimeshi_node *n, uint16_t id) {
    for (int i = 0; i < MC_MAP_NODES; i++)
        if (n->map[i].alive && n->map[i].id == id) return i;
    return -1;
}

/* Squared distance on the coarse 16-bit geo grid (5): comparison only,
 * so no sqrt. Longitude steps shrink with cos(latitude); at the NL
 * latitudes the factor is roughly 3/4 - folded into a fixed weight,
 * good enough to rank bridge-node candidates (7.1). */
uint32_t geo_dist2(uint16_t lat1, uint16_t lon1, uint16_t lat2, uint16_t lon2) {
    int32_t dlat = (int32_t)lat1 - (int32_t)lat2;
    int32_t dlon = ((int32_t)lon1 - (int32_t)lon2) * 3 / 4;
    return (uint32_t)(dlat * dlat + dlon * dlon);
}

/* Reception of a table chunk (6.1). Self-contained: every entry is
 * processed on its own against the own map and link store. The owner
 * naming US in an entry is reverse-direction evidence for the link
 * owner->us (6.3) — gated on quality like every confirmation. */
void chunk_rx(mimeshi_node *n, int8_t rssi, const uint8_t *pkt, uint16_t len) {
    chunk_rx_mode(n, rssi, pkt, len, 0);
}
void chunk_rx_mode(mimeshi_node *n, int8_t rssi, const uint8_t *pkt,
                          uint16_t len, int star_mode) {
    if (len < 10) return;
    uint16_t owner = rd16(pkt + 1);
    uint8_t n_ent = pkt[4];
    mimeshi_mapnode *om = map_add(n, owner);
    if (om) {
        om->geo_lat = rd16(pkt + 5); om->geo_lon = rd16(pkt + 7);
        om->quality = quality_of(rssi);       /* our measurement of owner->us */
        om->age_ticks = 0; om->arrived_tick = n->tick;
    }
    mimeshi_neighbor *nb = nbr_find(n, owner);
    if (!nb) nb = nbr_add(n, owner);
    if (nb) nb->quality = quality_ewma(nb->quality ? nb->quality : quality_of(rssi), quality_of(rssi));
    uint16_t off = 10;
    for (uint8_t e = 0; e < n_ent && off + 9 <= len; e++) {
        uint16_t eid = rd16(pkt + off);
        uint8_t qbyte = pkt[off + 2];
        uint8_t cls = qbyte >> 4;              /* origin->entry class (6.1) */
        uint8_t age = pkt[off + 3];
        uint16_t gl = rd16(pkt + off + 4), go = rd16(pkt + off + 6);
        uint8_t lcnt = pkt[off + 8];
        off += 9;
        if (eid == n->id) {                    /* the origin's own entry naming US: reverse evidence (6.3) */
            if (nb) {
                nb->proven_age_us = 0;
                /* the origin's entry class for us IS its measurement of
                 * our signal: our reverse quality, refreshed with every
                 * star that names us (10.2) — no extra airtime */
                uint8_t oq = quality_from_class(cls);
                nb->rev_quality = quality_ewma(nb->rev_quality ? nb->rev_quality : oq, oq);
                /* the TWO-SIDED THRESHOLD (5.2, final form): his star
                 * naming us is his direction's claim (his class for us
                 * becomes the link quality rev_quality, no gate); our
                 * reception of THIS star is the measured other
                 * direction - the only gate. Weakly received claims
                 * confirm nothing. */
                if (quality_ok(quality_of(rssi))) {
                    if (!nb->sym) tbl_nbr_added(n, owner);
                    nb->sym = 1; nb->ack_try = 0;
                }
            }
            /* our entry in the origin's star carries the origin's links,
             * ours included: consume them like every entry's links and
             * store them as received (the owner's advertised form of OUR
             * link is routing material, 6.1) - the link loop below runs
             * for this entry too. */
            for (uint8_t k = 0; k < lcnt && off + 3 <= len; k++) {
                uint16_t peer = rd16(pkt + off);
                uint8_t lb = pkt[off + 2];
                off += 3;
                if (peer == n->id || peer == owner) continue;  /* our own entry in
                    the origin's star names only us; a self-link does not exist */
                mimeshi_mapnode *pm = map_add(n, peer);  /* named by the link (6.3) */
                if (pm) { pm->age_ticks = age; pm->arrived_tick = n->tick; }
                lnk_set_class(n, eid, peer, (uint8_t)(lb >> 4));
                lnk_set_class(n, peer, eid, (uint8_t)(lb & 0x0F));
            }
        } else {
            mimeshi_mapnode *em = map_add_geo(n, eid, gl, go);   /* content arrival:
                the star's own geo rides in the packet, so the admission
                competition can score it honestly (6.2) */
            int stale = 0;
            if (em && star_mode) {
                /* fresh beats old on CONTENT (6.1): a form whose
                 * advertised entry byte differs from the stored one is
                 * only accepted when its beat is AHEAD of
                 * the stored beat - a differing form at an equal or
                 * older tick is a stale copy racing a newer one (a
                 * slow route, a missed-list answer) and must not roll
                 * the newer knowledge back. Identical content is
                 * always fine (an echo refreshes nothing but the
                 * age). Geo and presence still refresh - map
                 * material is never wasted. */
                int16_t ahead = (int16_t)((int16_t)age - (int16_t)em->star_beat);
                if (em->adv_q && qbyte != em->adv_q && ahead <= 0)
                    stale = 1;
            }
            if (em && !stale) {
                ev_taboo_forget(n, eid);   /* the complete form arrives as content:
                                              the rejection is over (6.1) - only
                                              here, not on presence shells */
                if (gl) { em->geo_lat = gl; em->geo_lon = go; }   /* (0,0) = none (5) */
                /* storage as received (6.1): the entry class byte is
                 * stored exactly as the origin advertised it - reproduced
                 * unchanged on every hop, hashed by the content hash.
                 * The local quality (our own measurement of the node's
                 * signal, refreshed below on any identifying reception)
                 * stays separate: routing material, never forwarded. */
                em->adv_q = qbyte;
                em->age_ticks = age; em->arrived_tick = n->tick;
            }
            /* the star's links: claims of the ORIGIN (eid), stored as
             * received (6.1) - the link store is the forwarded picture,
             * the advertised forms of the owning stars. Skipped for a
             * stale form: its link claims belong to the older copy. */
            if (!stale) {
                for (uint8_t k = 0; k < lcnt && off + 3 <= len; k++) {
                    uint16_t peer = rd16(pkt + off);
                    uint8_t lb = pkt[off + 2];
                    off += 3;
                    if (peer == n->id) {
                        /* the origin's (eid's) link to US: reverse-direction
                         * evidence (6.3) - the high nibble is the origin's
                         * measurement of our signal, the low nibble our class
                         * as it reported it. Only meaningful when the origin
                         * is a direct neighbour of ours (the claim reaches us
                         * possibly via a forwarder, 6.1). The link itself is
                         * stored as received like every other claim (6.1):
                         * the owner's advertised form of OUR link is routing
                         * material - the old cadence covered it in our own
                         * entry, the star round carries it only here. */
                        lnk_set_class(n, eid, peer, (uint8_t)(lb >> 4));
                        lnk_set_class(n, peer, eid, (uint8_t)(lb & 0x0F));
                        mimeshi_neighbor *enb = nbr_find(n, eid);
                        if (enb) {
                            enb->proven_age_us = 0;
                            uint8_t oq = quality_from_class((uint8_t)(lb >> 4));
                            enb->rev_quality = quality_ewma(enb->rev_quality ? enb->rev_quality : oq, oq);
                            /* two-sided threshold, final form (5.2, Oct 2026):
                             * the claim is his direction, our reception of
                             * the star is the other - the only gate */
                            if (quality_ok(quality_of(rssi))) {
                                if (!enb->sym) tbl_nbr_added(n, eid);
                                enb->sym = 1; enb->ack_try = 0;
                            }
                        }
                        continue;
                    }
                    mimeshi_mapnode *pm = map_add(n, peer);  /* named by the link (6.3) */
                    if (pm) {
                        /* the peer is exactly as fresh as the star that named
                         * it: the origin just advertised knowledge of it. */
                        pm->age_ticks = age; pm->arrived_tick = n->tick;
                    }
                    lnk_set_class(n, eid, peer, (uint8_t)(lb >> 4));
                    lnk_set_class(n, peer, eid, (uint8_t)(lb & 0x0F));
                }
            } else {
                off += 3 * lcnt;                /* skip the stale form's links */
            }
        }
    }
}
