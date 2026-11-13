/*
 * Meshimeshi protocol core, star.c. See core/internal.h for the
 * module map; design references point into docs/design.md.
 */
#include "internal.h"
#include <string.h>
#include <stdio.h>

/* ---- star canonicalisation (6.1) --------------------------------------- */
/* The star is the unit of sharing in the star-round concept: one node
 * entry plus the links it OWNS. Canonical form (6.1): links sorted by
 * neighbour id, so the star has a deterministic byte representation
 * independent of the internal table order - the basis for content
 * digests and the derivable heartbeat match. The entry layout follows
 * the chunk entry of 6.1 exactly (id, quality byte, age, geo), so a
 * star serialises to bytes a chunk receiver already understands. */
#define MC_STAR_MAX_LINKS 64
#define MC_STAR_MAX_BYTES (9 + 3 * MC_STAR_MAX_LINKS)

/* Link ownership (6.1): one bit of a CRC-8 over the normalised pair.
 * The owner of {A,B}: normalise a=min, b=max, hash the 4 bytes, bit 0
 * decides - set: a owns, else b. Both endpoints (and every receiver,
 * whatever the hop distance) compute the same owner from the pair
 * alone; the normalisation makes the rule symmetric by construction:
 * both sides hash the very same bytes, so no AB-vs-BA comparison and
 * no tie-break can arise. Measured fair split (~50/50), so each node
 * owns about half its links and star sizes even out across a
 * neighbourhood. Ownership is permanent (IDs and the hash are fixed):
 * a link's advertised form stays bound to exactly one star. */
static uint8_t lnk_pair_bit(uint16_t x, uint16_t y) {
    uint16_t a = x < y ? x : y, b = x < y ? y : x;
    uint8_t bytes[4];
    wr16(bytes, a); wr16(bytes + 2, b);
    uint8_t crc = 0;
    for (int i = 0; i < 4; i++) {
        crc ^= bytes[i];
        for (int k = 0; k < 8; k++)
            crc = (uint8_t)((crc & 0x80) ? (crc << 1) ^ 0x07 : crc << 1);
    }
    return (uint8_t)(crc & 1);
}
uint16_t lnk_owner(uint16_t x, uint16_t y) {
    uint16_t a = x < y ? x : y, b = x < y ? y : x;
    return lnk_pair_bit(x, y) ? a : b;
}

/* Canonical epilogue of a serialised star: sort the collected links by
 * peer id (the canonical form of 6.1 - deterministic bytes independent of
 * internal table order), check the byte budget, and pack the link area
 * after the 9-byte entry. Returns the total star length, or 0 when the
 * links do not fit maxlen. */
static uint16_t star_pack_links(uint8_t *buf, uint16_t maxlen,
                                uint16_t *ids, uint8_t *qc, uint16_t cnt) {
    for (uint16_t i = 1; i < cnt; i++) {         /* insertion sort by peer id */
        uint16_t id = ids[i]; uint8_t q = qc[i];
        uint16_t j = i;
        while (j > 0 && ids[j-1] > id) { ids[j] = ids[j-1]; qc[j] = qc[j-1]; j--; }
        ids[j] = id; qc[j] = q;
    }
    if (9 + 3 * cnt > maxlen) return 0;          /* does not fit */
    buf[8] = (uint8_t)cnt;                       /* link count, chunk entry layout (6.1) */
    for (uint16_t i = 0; i < cnt; i++) {
        wr16(buf + 9 + 3 * i, ids[i]);
        buf[9 + 3 * i + 2] = qc[i];
    }
    return (uint16_t)(9 + 3 * cnt);
}

/* Serialise the star of node `center` as known by n - storage as
 * received (6.1): the entry (id, both quality classes, tick, geo) and
 * every link OWNED by center, reproduced exactly as the origin
 * advertised them. For the OWN star (center == n->id) the classes of
 * the direct-neighbour links come from the neighbor table (the own
 * measurement and the peer's reported reverse quality, 5.2); for a
 * foreign star the entry class byte is adv_q as received and the
 * link bytes come from the link store as received - the form the
 * origin put on air, so the content hash is a property of the
 * origin, not of the observer (the 6a finding, resolved in design).
 * age_out receives the age byte position in buf (the beat
 * lives there, 6.1); the age byte itself is written but excluded from
 * the content hash (mimeshi_star_content_hash). Returns the total
 * length, or 0 if the star does not exist or does not fit maxlen. */
uint16_t mimeshi_star_serialize(mimeshi_node *n, uint16_t center,
                               uint8_t *buf, uint16_t maxlen,
                               uint16_t *age_off_out) {
    mimeshi_mapnode *m = map_find(n, center);
    if (!m) return 0;
    if (maxlen < 9) return 0;

    wr16(buf, center);
    if (center == n->id) {
        /* the OWN entry: the origin publishes itself. The entry byte
         * is the own link picture toward the strongest confirmed
         * neighbour (own measured class | the class that neighbour
         * reported about us, 5.2) - a stable, self-computed value.
         * Self-publication (6.1 storage-as-received): the form is
         * stored in the own adv_q, so the own catalogue digest is
         * computed over exactly the bytes the neighbours hold. */
        uint8_t q = 0;
        int best = -1;
        for (int i = 0; i < MC_MAX_NEIGHBORS; i++) {
            const mimeshi_neighbor *nb = &n->nbr[i];
            if (!nb->alive || !nb->sym) continue;
            if (best < 0 || nb->quality > n->nbr[best].quality) best = i;
        }
        if (best >= 0) {
            const mimeshi_neighbor *nb = &n->nbr[best];
            q = (uint8_t)((adv_class_of(nb->quality) << 4)
                          | adv_class_of(nb->rev_quality));
        }
        m->adv_q = adv_byte_hyst(m->adv_q, q);   /* hysteresis (6.1) */
        buf[2] = m->adv_q;
    } else if (m->adv_q) {
        buf[2] = m->adv_q;              /* entry classes exactly as the
                                           origin advertised them (6.1) */
    } else {
        /* never received the star itself, only map material: the local
           measurement of the node's signal as the known class */
        buf[2] = (uint8_t)((adv_class_of(m->quality) << 4) | 0);
    }
    buf[3] = m->star_beat;              /* beat in the age byte (6.1):
                                           the origin's liveness, not the
                                           travelled age - travelled age is
                                           local (6.2) and never travels in
                                           the canonical star form */
    wr16(buf + 4, m->geo_lat);
    wr16(buf + 6, m->geo_lon);
    if (age_off_out) *age_off_out = 3;

    /* links of the star, sorted by peer id (canonical form, 6.1).
     * OWNED share only (6.1): the link {center, peer} travels in
     * exactly one star - the owner's. The owner bit is derived from
     * the pair itself, so every observer packs the same share. */
    uint16_t ids[MC_STAR_MAX_LINKS];
    uint8_t qc[MC_STAR_MAX_LINKS];
    uint16_t cnt = 0;
    if (center == n->id) {
        /* the OWN star: the origin's own links come from the NEIGHBOR
         * TABLE, not the link store (6.1) - the origin is the source
         * of its own star, and at startup the store is still empty
         * (bootstrap: no star received yet, no link stored yet). Every
         * confirmed symmetric neighbour with the ownership bit on our
         * side travels, with the own measured class and the peer's
         * reported reverse class (5.2), published under hysteresis
         * (6.1): a >= 2-step class move re-publishes, sub-threshold
         * EWMA movement does not. */
        for (int i = 0; i < MC_MAX_NEIGHBORS && cnt < MC_STAR_MAX_LINKS; i++) {
            mimeshi_neighbor *pnb = &n->nbr[i];
            if (!pnb->alive || !pnb->sym) continue;
            if (lnk_owner(n->id, pnb->id) != n->id) continue;   /* owned share */
            uint8_t fresh = (uint8_t)((adv_class_of(pnb->quality) << 4)
                                      | adv_class_of(pnb->rev_quality));
            pnb->adv_class = adv_byte_hyst(pnb->adv_class, fresh);
            ids[cnt] = pnb->id;
            qc[cnt] = pnb->adv_class;
            cnt++;
        }
        return star_pack_links(buf, maxlen, ids, qc, cnt);
    }
    for (int i = 0; i < MC_MAX_LINKS && cnt < MC_STAR_MAX_LINKS; i++) {
        if (!n->lnk[i].alive) continue;
        uint16_t peer = 0;
        if (n->lnk[i].a == center)      peer = n->lnk[i].b;
        else if (n->lnk[i].b == center) peer = n->lnk[i].a;
        else continue;
        if (lnk_owner(center, peer) != center) continue;   /* owned share (6.1) */
        if (map_find(n, peer) == 0) continue;  /* link endpoints are map nodes */
        uint8_t cf = lnk_class(n, center, peer);
        if (cf == 0xFF) continue;
        uint8_t cr = lnk_class(n, peer, center);
        ids[cnt] = peer;
        qc[cnt] = (uint8_t)((cf << 4) | (cr == 0xFF ? 0 : cr));
        cnt++;
    }
    return star_pack_links(buf, maxlen, ids, qc, cnt);
}

/* 1-byte content hash over the canonical star bytes EXCLUDING the age
 * byte (5.3): the digest covers what is advertised (identity, links,
 * quality classes), not how old the advertisement is - silent EWMA
 * movement under the hysteresis does not change it, and the heartbeat
 * tick (added separately in the digest) can change freely. CRC-8/ATM
 * (poly 0x07, init 0): cheap, fixed table-free, good avalanche for the
 * small fixed-size input. */
uint8_t mimeshi_star_content_hash(const uint8_t *star, uint16_t len,
                                  uint16_t age_off) {
    uint8_t crc = 0;
    for (uint16_t i = 0; i < len; i++) {
        if (i == age_off) continue;              /* age excluded (5.3) */
        crc ^= star[i];
        for (int b = 0; b < 8; b++)
            crc = (uint8_t)((crc & 0x80) ? (crc << 1) ^ 0x07 : crc << 1);
    }
    return crc;
}

/* Digest of a star as carried in the catalogue (5.3):
 * digest = (content_hash + beat) mod 256 - derivable: a receiver that
 * has the star can compute the expected digest for beat + k for small
 * k, which is the heartbeat match. */
uint8_t mimeshi_star_digest(uint8_t content_hash, uint8_t beat) {
    return (uint8_t)(content_hash + beat);
}

/* Beat match (5.3): does `received` fit content_hash at
 * stored_beat + k for some k in 0..k_max? Returns the matched beat
 * advance (1..k_max), 0 for an exact same-beat match, or -1 when no
 * k fits (real content change, or a star we do not have). */
/* Beat match (6.1): the received digest fits when it was
 * computed from the same content hash at stored_beat + k, for small
 * k. The window is SYMMETRIC: a catalogue may lag behind our stored
 * tick (a slower route carried the newer tick here first, 6.1) - an
 * older advertisement with the same content hash still confirms the
 * content. Return convention: 0 = exact, 1..k_max = the advertisement
 * is AHEAD (advance the stored beat by k), 17..16+k_max = the
 * advertisement LAGS by (r - 16) (keep the stored beat, only the age
 * refreshes), -1 = no fit (a real content change, or an unknown
 * star). The two windows cannot collide with the sentinel. */
#define MC_TICK_LAG_BASE 16
int mimeshi_star_beat_match(uint8_t received, uint8_t content_hash,
                           uint8_t stored_beat, uint8_t k_max) {
    if (received == mimeshi_star_digest(content_hash, stored_beat)) return 0;
    for (uint8_t k = 1; k <= k_max; k++) {
        if (received == mimeshi_star_digest(content_hash,
                                            (uint8_t)(stored_beat + k)))
            return (int)k;
        if (received == mimeshi_star_digest(content_hash,
                                            (uint8_t)(stored_beat - k)))
            return (int)k + MC_TICK_LAG_BASE;
    }
    return -1;
}
