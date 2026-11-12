/*
 * Meshimeshi protocol core, internal module interface (internal.h).
 * Module map (dependency order, no cycles):
 *   nbr.c        neighbour table, quality model, dedup, keepalive counters
 *   bound.c      the exchange: bound pool + switch rule (the only n->tx path
 *                besides ack1)
 *   proof.c      discovery evidence: proof lists, collective proof ack,
 *                generalized hello-ack, the 1-min tick
 *   map.c        2-hop map (constellation), link store, eviction/taboo,
 *                chunk ingestion
 *   star.c       star canonicalisation, digests, beat match
 *   star_round.c the uniform convergence cadence: round send/receive,
 *                catalogue, missed list
 *   route.c      ETX route computation (Dijkstra, bridge rule)
 *   unicast.c    unicast + ack1/ack2/ack3, attempt schedule, tracing
 *   core.c       reception dispatch, init/start (the public entry points)
 *
 * Everything declared here is internal to the core; the public API
 * stays in core.h. The simulator and firmware link all modules
 * together (CMakeLists.txt, README).
 */
#ifndef MCORE_INTERNAL_H
#define MCORE_INTERNAL_H

#include "core.h"
#include "config.h"
#include "pkt.h"

/* ---- shared module constants ---- */
#define MC_STAR_K_MAX 8u        /* beat-lag window (6.1, starting value) */
#define MC_STAR_DEAD_TICKS 360u  /* liveness threshold (6.2): a star whose origin
                                   tick has not advanced for 6 h of 1-min proof
                                   ticks is dead - de-weighted at half, evicted at the
                                   threshold, whatever the forwarded copies say */
#define MC_CHUNK_MAX_ENTRIES 8  /* content stars per round packet (6.1):
                                   8 x (9 B + links) fits 230 B */

/* ---- nbr.c ---- */
void ka_called(mimeshi_node *n, uint8_t ch);
mimeshi_neighbor *nbr_find(mimeshi_node *n, uint16_t id);
mimeshi_neighbor *nbr_add(mimeshi_node *n, uint16_t id);
uint8_t quality_of(int8_t rssi);
bool quality_ok(uint8_t q);
uint8_t adv_class_of(uint8_t q);
uint8_t adv_byte_hyst(uint8_t published, uint8_t fresh);
uint8_t quality_from_class(uint8_t cls);
uint8_t quality_ewma(uint8_t stored, uint8_t fresh);
bool dedup_fresh(mimeshi_node *n, uint16_t msg_id, uint16_t src);

/* ---- bound.c ---- */
void bound_send(mimeshi_node *n, uint8_t ch, const uint8_t *pkt, uint16_t len, uint64_t airtime);

/* ---- proof.c ---- */
void proof_ack_fire(mimeshi_node *n, void *ud);
void proof_tick(mimeshi_node *n, void *ud);
void proof_stagger(mimeshi_node *n);
void hello_ack_wait(mimeshi_node *n, uint16_t from, int8_t rssi);

/* ---- map.c ---- */
uint32_t map_age_ticks(mimeshi_node *n, mimeshi_mapnode *m);
int ev_taboo_check(mimeshi_node *n, uint16_t id);
void ev_taboo_forget(mimeshi_node *n, uint16_t id);
void map_evict(mimeshi_node *n, mimeshi_mapnode *m);
mimeshi_mapnode *map_find(mimeshi_node *n, uint16_t id);
mimeshi_mapnode *map_add(mimeshi_node *n, uint16_t id);
mimeshi_mapnode *map_add_geo(mimeshi_node *n, uint16_t id, uint16_t glat, uint16_t glon);
int r_idx(mimeshi_node *n, uint16_t id);
uint32_t geo_dist2(uint16_t lat1, uint16_t lon1, uint16_t lat2, uint16_t lon2);
uint8_t lnk_class(mimeshi_node *n, uint16_t from, uint16_t to);
void lnk_set_class(mimeshi_node *n, uint16_t from, uint16_t to, uint8_t cls);
void tbl_mark_dirty(mimeshi_node *n);
void tbl_nbr_added(mimeshi_node *n, uint16_t id);
void chunk_rx(mimeshi_node *n, int8_t rssi, const uint8_t *pkt, uint16_t len);
void chunk_rx_mode(mimeshi_node *n, int8_t rssi, const uint8_t *pkt,
                   uint16_t len, int star_mode);

/* ---- star.c ---- */
uint16_t lnk_owner(uint16_t x, uint16_t y);

/* ---- star_round.c ---- */
void star_round_rx(mimeshi_node *n, int8_t rssi, const uint8_t *pkt, uint16_t len);
void star_round_tick(mimeshi_node *n, void *ud);

/* ---- route.c ---- */
uint16_t route_next_hop(mimeshi_node *n, uint16_t dst);
uint16_t route_next_hop_geo_prev(mimeshi_node *n, uint16_t dst,
                                 uint16_t lat, uint16_t lon,
                                 const uint16_t *excl, uint8_t excl_cnt,
                                 uint16_t prev);

/* ---- unicast.c ---- */
void ack1_send(mimeshi_node *n, uint16_t msg_id, uint16_t src, uint8_t ch, int8_t rssi);
void ack2_send(mimeshi_node *n, uint16_t msg_id, uint16_t src, uint16_t prev,
               bool neg, uint8_t budget_left);
uint8_t schedule_candidates(mimeshi_node *n, uint16_t dst,
                           uint16_t gl, uint16_t go,
                           uint16_t prev,
                           const uint16_t *excl, uint8_t excl_cnt,
                           uint16_t *cand);
uint8_t schedule_attempts(const mimeshi_pending *p);
void schedule_fail(mimeshi_node *n, mimeshi_pending *p);
void pending_try(mimeshi_node *n, mimeshi_pending *p);
void ack2_timeout(mimeshi_node *n, void *ud);
void hoptrace_dossier(mimeshi_node *n, uint16_t dst, uint16_t prev,
                      uint16_t gl, uint16_t go, uint8_t budget,
                      const uint16_t *excl, uint8_t excl_cnt,
                      const uint16_t *cand, uint8_t cand_cnt, uint16_t chosen);

#endif /* MCORE_INTERNAL_H */
