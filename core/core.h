/*
 * Meshimeshi — protocolkern, publieke API (core.h)
 * Ontwerp: docs/ontwerp.md (sectie 13b: gedeelde kern voor firmware en simulator).
 *
 * De kern heeft geen eigen klok, radio of RNG: alles komt via de callbacks in
 * mimeshi_init(). De simulator levert ze (virtuele radio); op firmware levert de
 * platformlaag ze. Geen dynamische allocatie; alle buffers vast.
 *
 * De node-struct staat volledig in de header, zodat nodes statisch kunnen
 * worden gereserveerd (geen heap). Interne velden zijn read-only voor de
 * platformlaag.
 */
#ifndef MCORE_H
#define MCORE_H

#include <stdint.h>
#include <stdbool.h>

/* ---------------- configuratie (firmwareconstanten) ---------------- */
#define MC_N_CHANNELS    3
#define MC_MAX_NEIGHBORS 64
#define MC_MAX_PENDING   16
#define MC_MAX_DEDUP    128
#define MC_MAX_PAYLOAD  230

/* ---------------- interne typen (zichtbaar voor vaste allocatie) ---- */
typedef struct {
    uint16_t id;
    uint8_t  quality;      /* afgeleid van RSSI, bijgewerkt per hello/ack */
    uint8_t  ack_try;      /* teller per buur (4.1): elke identificerende
                              zending van de buur telt; ack-pogingen op de
                              tellerstanden 1, 2, 4, 8, 16, daarna gemiddeld
                              1 op 16; reset bij bevestiging of hernieuwd contact */
    uint32_t last_seen;
    uint16_t state_ver;    /* laatst geziene state-versie van deze buur (4.3) */
    uint8_t  alive;
    uint8_t  sym;           /* 1 = bevestigd symmetrisch (geslaagde unicast
                               met ack1 of tablepull van de buur); 0 =
                               alleen gezien, één richting (4.2, 10.1) */
} mimeshi_neighbor;

typedef struct {
    uint8_t  active, tries_left, budget_left;
    uint16_t msg_id, src, dst, prev_hop, next_hop;
    uint8_t  pkt[10 + MC_MAX_PAYLOAD];
    uint16_t pkt_len;
} mimeshi_pending;

typedef struct { uint16_t msg_id, src; } mimeshi_dedup;

struct mimeshi_node;
typedef struct mimeshi_node mimeshi_node;

/* ---------------- hardware-interface -------------------------------- */
typedef void (*mimeshi_tx_fn)(mimeshi_node *n, uint8_t channel, const uint8_t *pkt,
                            uint16_t len, uint64_t airtime_us);
typedef void (*mimeshi_timer_fn)(mimeshi_node *n, uint64_t delay_us,
                               void (*cb)(mimeshi_node *, void *), void *ud);
typedef void (*mimeshi_switch_fn)(mimeshi_node *n, uint8_t channel, uint64_t s_us,
                                void (*done)(mimeshi_node *, void *), void *ud);
typedef uint32_t (*mimeshi_rand_fn)(mimeshi_node *n);

/* ---------------- node ---------------------------------------------- */
struct mimeshi_node {
    uint16_t id;
    uint8_t  home_ch;                 /* id % MC_N_CHANNELS */
    uint16_t my_state_ver;
    mimeshi_neighbor nbr[MC_MAX_NEIGHBORS];
    mimeshi_pending  pend[MC_MAX_PENDING];
    mimeshi_dedup    dedup[MC_MAX_DEDUP];
    uint64_t hello_interval_us;       /* adaptief: 30 s -> 1 u */
    uint64_t hello_jitter_us;         /* random fase per ronde; 0 = geen jitter.
                                         Default MC_HELLO_JITTER_US; voor de
                                         simulatie in te stellen vóór mimeshi_start */
    uint8_t  ka_idle[MC_N_CHANNELS];  /* stillteteller per kanaal in keepaliveticks
                                         (4.1): elke eigen identificerende zending
                                         op kanaal c zet hem op 0; na 64 ticks
                                         (= T_keepalive, core.c) stilte volgt
                                         precies één hello op c */
    mimeshi_tx_fn     tx;
    mimeshi_timer_fn  timer;
    mimeshi_switch_fn switch_ch;
    mimeshi_rand_fn   rand;
    void *user;                       /* eigen data van platform/simulator */
};

/* ---------------- API ----------------------------------------------- */
/* Initialiseert de node (nuldekking + callbacks). id bepaalt het thuiskanaal. */
void mimeshi_init(mimeshi_node *n, uint16_t id, mimeshi_tx_fn tx, mimeshi_timer_fn timer,
                mimeshi_switch_fn sw, mimeshi_rand_fn rnd, void *user);

/* Eerste hello-ronde direct, daarna opstartrondes met random fase (jitter);
 * na de opstartfase bestaat de hello alleen nog als per-kanaal-keepalive (4.1). */
void mimeshi_start(mimeshi_node *n);

/* Unicast naar dst. 0 = geaccepteerd, -1 = dst niet in de kaart (geen flood),
 * -2 = geen pending-slot vrij, -3 = payload te groot. */
int  mimeshi_send_data(mimeshi_node *n, uint16_t dst, const uint8_t *payload,
                     uint16_t len, bool want_ack3);

/* Pakket ontvangen (van radio of simulator). ch = kanaal van ontvangst. */
void mimeshi_rx(mimeshi_node *n, uint8_t channel, int8_t rssi_db,
              const uint8_t *pkt, uint16_t len);

#endif /* MCORE_H */
