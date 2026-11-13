/*
 * Meshimeshi protocol core, packet types and formats (pkt.h).
 * Shared by all core modules via internal.h; the wire values of
 * the existing types are sacred (chunks are 4, data is 5, ack1 is 6,
 * star round 10).
 *
 * Type byte: low nibble = type, high nibble = flags (10.1). All
 * multi-byte fields little-endian (rd16/wr16). Layouts:
 *
 *   HELLO       3 B   type, node-id(2). DROPPED as a sent type
 *                     (5.3): no node sends it anymore. A received packet
 *                     of this type is still processed as an identifying
 *                     transmission of the sender (liveness, quality,
 *                     generalized ack).
 *   HELLO_ACK   6 B   type, destination-id(2) = the acked sender,
 *                     sender-id(2), rq(1) — directional: the address makes
 *                     the ack an exact symmetry proof (6.2; 2, 5.2, 5.3)
 *   PROOF       3+2n B type, node-id(2), n x neighbour-id(2), n <= 17 (5.2).
 *                     Reactionless keepalive + symmetry flush; an empty list
 *                     is the minimal keepalive beacon (type + node-id).
 *   PROOF_ACK   3 B   type, node-id(2) = the acked round's sender (5.2,
 *                     re-anchored by 5.3). The collective third step of the
 *                     three-way: once per round that collected acks,
 *                     scheduled at the ack window's end, the
 *                     sender confirms ALL received acks at once. A
 *                     responder that acked that round and hears this packet
 *                     closes its side of the symmetry agreement (sym = 1 on
 *                     this received evidence - the neighbour's own quality
 *                     measurement no longer blocks what the packet proves).
 *   DATA      12+n B  type+F_ACK3_REQ+F_DEST_GEO, msg-id(2), source(2),
 *                     destination(2), previous-hop(2), budget(1, offset 9),
 *                     next-hop(2, offset 10), [dest geo 4 B when F_DEST_GEO,
 *                     offset 12 (5, 7.1)], payload(n)  (10.1)
 *   ACK1        7 B   type, msg-id(2), source of the message(2), sender(2) (10.2)
 *   ACK2        7 B   same as ACK1; F_NEG_ACK2 makes it negative       (6.2, 8.2)
 *   ACK3       14 B   DATA format, payload 4 B: the original msg-id (6.2, 11)
 *
 * TABLE_PULL / TABLE_CHUNK (6.1) and FAIL_RPT (8.6) are reserved for
 * their future function and are ignored at the moment.
 */
#ifndef MCORE_PKT_H
#define MCORE_PKT_H

#include <stdint.h>

enum {
    PKT_HELLO = 1, PKT_HELLO_ACK, PKT_PROOF, PKT_TABLE_CHUNK,
    PKT_DATA, PKT_ACK1, PKT_ACK2, PKT_ACK3, PKT_FAIL_RPT
    /* The three-way closure (5.2, decided Oct 2026, re-anchored by 5.3):
     * the acked round sender's
     * COLLECTIVE confirmation, one packet per round after the ack
     * window - every responder of that round learns 'my ack arrived,
     * my direction works, the sender's symmetry conclusion is drawn'
     * in one airtime. Wire value 11: appended after PKT_STAR_ROUND (10,
     * a #define outside this enum) - the wire values of the existing
     * types are sacred (chunks are 4, data is 5, ack1 is 6, star round 10). */
};
#define PKT_STAR_ROUND 10  /* new packet type (5.3) */
#define PKT_PROOF_ACK 11
#define F_ACK3_REQ 0x10   /* requested by the source of the message (8.8) */
#define F_DEST_GEO 0x20   /* destination geo present: 4 B after the fixed
                             header (6.1, 7.1) - every hop can recompute
                             the bridge node toward an unknown destination */
#define F_WAYPOINT_GEO 0x40 /* waypoint geo present (10.2, decided Oct 2026): 4 B
                              after the dest geo (or after the fixed header when
                              no dest geo). The traveling heading of the best
                              bridge the chain has seen - never a required
                              destination: hops route toward its ZONE with
                              locally routable steps. A forwarder replaces it
                              only when its own bridge candidate lies strictly
                              closer to the destination (better-than, 5%
                              margin). The destination in the own graph always
                              wins over the heading. */

#define F_NEG_ACK2 0x40   /* next hop could not forward (8.2) */

/* little-endian reading/writing of 16-bit fields (10.1) */
static inline uint16_t rd16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static inline void wr16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }

#endif /* MCORE_PKT_H */
