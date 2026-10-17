/*
 * Meshimeshi — protocolkern, implementatie. Zie core.h (API + node-struct)
 * en docs/ontwerp.md (sectieverwijzingen hieronder verwijzen daarnaartoe).
 *
 * Bouwsteen 1 (koppeltest): hello-ronde, neighbor table, hello-acks met
 * RSSI-spreiding, unicast met ack1/ack2/ack3, retries, budget.
 * Forwarding, routering (10.2), T_ack2-venster (7.3) en tablepull (4.3)
 * zijn bouwsteen 2 — waar dat gedrag nog ontbreekt staat dat vermeld.
 *
 * Architectuur (13b): de kern heeft geen klok, radio of RNG. Alles komt
 * binnen via de vier callbacks uit mimeshi_init(): tx (zenden), timer
 * (eenmalig tijdvenster), switch_ch (kanaalwissel met done-callback) en
 * rand (jitter, msg-id). Platformonafhankelijk en zonder dynamische
 * allocatie: alle buffers zitten vast in de node-struct uit core.h.
 *
 * Aanroepmodel:
 *  - Platform -> kern:  mimeshi_rx() per ontvangen pakket (kanaal + RSSI),
 *                       mimeshi_send_data() voor een unicast, mimeshi_start().
 *  - Kern -> platform:  tx() en switch_ch() voor elke zending, timer() voor
 *                       elke wachttijd, rand() voor elke willekeurige waarde.
 *  - Alle tijden in microseconden (uint64_t): de simulator rekent met gehele
 *    getallen, firmware met een lokale hardware-teller (13). Geen kloksync.
 *
 * Wisselregel (2), afgedwongen via bound_send():
 *  - Exchange = wissel naar het kanaal van de partner
 *    (MC_S_SWITCH_US), zenden zodra de radio op kanaal zit. De kern heeft
 *    geen expliciete "keer terug naar huis"-aanroep: de platformlaag keert
 *    na de exchange zelfstandig terug naar het thuiskanaal (bij
 *    data na het ack1-venster op het exchangekanaal, 7.3). De
 *    simulator (sim/sim1.py, class Node) is de referentie-implementatie
 *    van dat platformcontract.
 *  - Ack1 is de uitzondering: die gaat zonder wissel via tx() op het kanaal
 *    van de lopende exchange, want de verzender wacht daar (7.1).
 *
 * Logging: de printf-regels zijn bedoeld voor koppeltest en simulator.
 * Firmware kan ze dempen; later worden ze een platform-callback.
 */
#include "core.h"
#include <string.h>
#include <stdio.h>

/* ---------------- configuratie (firmwareconstanten) ----------------
 * Waarden en status staan in het ontwerpdocument; wijzig ze daar eerst
 * (werkregel: ontwerp -> kern -> ctest + sim1.py laten meegroeien). */
#define MC_HELLO_START_US  30000000ull     /* 30 s na opstarten (4.1) */
#define MC_HELLO_MAX_US    3600000000ull   /* 1 u stabiel (4.1) */
#define MC_HELLO_JITTER_US 1000000ull      /* random fase per ronde (4.1, open punt 3) */
#define MC_HELLO_ACK_WIN_US 3000000ull     /* max responstijd hello-acks: 3 s (2) */
#define MC_HELLO_ACK_JITTER_US (MC_HELLO_ACK_WIN_US / 16u)  /* ~187 ms, > t_ack + S (2) */
#define MC_BUDGET_INIT     32              /* beginwaarde budgetbyte (9) */
#define MC_RETRY_TX         2              /* zendpogingen per buur per serie (7.4) */
#define MC_ACK1_MARGIN_US   80000ull       /* verwerking + marge bovenop t_data + t_ack (7.3) */
#define MC_S_SWITCH_US      5000ull        /* kanaalwissel incl. settling (14.1) */

/* ---------------- airtime (SF7, 62,5 kHz, CR4/5) ----------------
 * Vaste waarden uit de tabel in 14.1. De kern geeft de airtime mee aan
 * tx() zodat de platformlaag het zendvenster kan plannen; de functies
 * zijn bewust eenvoudig (drie klassen) in plaats van een exacte
 * lora-airtime-formule. */
static uint64_t t_ack_us(void)        { return 78000ull; }
static uint64_t t_hello_us(void)      { return 70000ull; }
static uint64_t t_data_us(uint16_t l) { return l <= 40 ? 173000ull : l <= 150 ? 459000ull : 869000ull; }

/* ---------------- packettypen en formaten ----------------
 * Type-byte: lage nibble = type, hoge nibble = flags (6.1). Alle
 * multi-bytevelden little-endian (rd16/wr16). Layouts:
 *
 *   HELLO       5 B   type, node-id(2), state-versie neighbor table(2)  (4.1, 6.3)
 *   HELLO_ACK   7 B   type, bestemmings-id(2) = hello-verzender,
 *                     afzender-id(2), state-versie(2) — directioneel:
 *                     het adres maakt de ack tot exact symmetriebewijs
 *                     (78 ms airtime; 2, 4.2)
 *   DATA      10+n B  type+F_ACK3_REQ, msg-id(2), bron(2), doel(2),
 *                     vorige-hop(2), budget(1, offset 9), payload(n)  (6.1)
 *   ACK1        7 B   type, msg-id(2), bron van het bericht(2), afzender(2) (6.2)
 *   ACK2        7 B   idem ACK1; F_NEG_ACK2 zet het negatief          (6.2, 7.2)
 *   ACK3       14 B   DATA-formaat, payload 4 B: het originele msg-id (6.2, 11)
 *
 * TABLE_PULL / TABLE_CHUNK (4.3) en FAIL_RPT (10.5) zijn gereserveerd
 * voor hun toekomstige functie en worden op dit moment genegeerd.
 */
enum {
    PKT_HELLO = 1, PKT_HELLO_ACK, PKT_TABLE_PULL, PKT_TABLE_CHUNK,
    PKT_DATA, PKT_ACK1, PKT_ACK2, PKT_ACK3, PKT_FAIL_RPT
};
#define F_ACK3_REQ 0x10   /* gevraagd door de bron van het bericht (11) */
#define F_NEG_ACK2 0x40   /* volgende hop kon niet doorsturen (7.2) */

/* little-endian lezen/schrijven van 16-bits velden (6.1) */
static uint16_t rd16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static void wr16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }

/* ---------------- neighbor table en dedup ---------------- */

/* Buur opzoeken op id. Lineair zoeken: MC_MAX_NEIGHBORS is klein en de
 * tabel wordt alleen geraakt bij ontvangst of verzendkeuze. */
static mimeshi_neighbor *nbr_find(mimeshi_node *n, uint16_t id) {
    for (int i = 0; i < MC_MAX_NEIGHBORS; i++) if (n->nbr[i].alive && n->nbr[i].id == id) return &n->nbr[i];
    return 0;
}

/* Kwaliteitsbyte uit een RSSI-meting (4.2): halvering van de genormaliseerde
 * RSSI laat ruimte voor een toekomstige SNR-component in dezelfde byte. */
static uint8_t quality_of(int8_t rssi) {
    return (uint8_t)(rssi < 0 ? (rssi + 128) >> 1 : 63);
}

/* Buur toevoegen (uit een hello of hello-ack). Vol = 0 teruggeven; de
 * aanroeper gaat daarmee door zonder de buur. Een nieuwe entry begint als
 * "gezien": één richting, sym = 0 (4.2). my_state_ver verhoogt bij elke
 * mutatie: de hello draagt die versie mee, zodat buren zien dat een
 * tablepull nodig is (4.3). */
static mimeshi_neighbor *nbr_add(mimeshi_node *n, uint16_t id) {
    for (int i = 0; i < MC_MAX_NEIGHBORS; i++)
        if (!n->nbr[i].alive) {
            n->nbr[i].alive = 1; n->nbr[i].id = id;
            n->nbr[i].sym = 0; n->nbr[i].ack_try = 0;
            n->my_state_ver++;
            return &n->nbr[i];
        }
    return 0;
}

/* Dedup op (msg-id, bron). false = duplicaat, niet opnieuw verwerken.
 * msg-id 0 geldt als leeg slot: elk bericht met id 0 wordt dus door-
 * gelaten; msg-ids komen uit de RNG (16 bit) en mogen 0 zijn — bekend
 * en acceptabel voor bouwsteen 1, gaat mee in de dedup-hardening.
 * Volle cache = ontvang maar; LRU-vervanging komt later (14, laag 3). */
static bool dedup_fresh(mimeshi_node *n, uint16_t msg_id, uint16_t src) {
    for (int i = 0; i < MC_MAX_DEDUP; i++)
        if (n->dedup[i].msg_id == msg_id && n->dedup[i].src == src) return false;
    for (int i = 0; i < MC_MAX_DEDUP; i++) if (n->dedup[i].msg_id == 0) {
        n->dedup[i].msg_id = msg_id; n->dedup[i].src = src; return true;
    }
    return true; /* volle cache: ontvang maar (LRU komt later) */
}

/* ---------------- exchange (wisselregel, sectie 2) ---------
 * bound_send() is de enige weg voor alles wat op een vreemd kanaal
 * verstuurd moet worden: hello's, hello-acks, ack2, ack3, data. De kern
 * kopieert het pakket in een vaste pool en vraagt de platformlaag om te
 * wisselen; de done-callback (bound_fire) zendt zodra de radio op kanaal
 * zit en maakt het poolslot daarna vrij. De platformlaag moet wissels
 * per node serialiseren (één radio) en na de zending thuiskomen.
 *
 * De pool heeft evenveel slots als pending-records: elk actief bericht
 * houdt hooguit één exchange tegelijk aan, want een retry start
 * pas na de ack1-timeout. Geen heap (13b); len == 0 markeert een vrij slot.
 * Op firmware draait één node per proces, dus MC_MAX_PENDING volstaat;
 * de simulator deelt de kern tussen alle nodes van het scenario — bouw de
 * .so dan met -DMC_BOUND_POOL=256 (zie README).
 */
#ifndef MC_BOUND_POOL
#define MC_BOUND_POOL MC_MAX_PENDING
#endif
typedef struct { uint8_t ch; uint16_t len; uint64_t airtime; uint8_t pkt[10 + MC_MAX_PAYLOAD]; } bound_ctx;
static bound_ctx bctx[MC_BOUND_POOL]; /* pool per zending; klein en vast */
static uint32_t bctx_drops;          /* volle pool = defect, nooit stil */

static void bound_fire(mimeshi_node *n, void *ud) {
    bound_ctx *c = (bound_ctx *)ud;
    n->tx(n, c->ch, c->pkt, c->len, c->airtime);
    c->len = 0; /* slot vrij */
}

/* Zenden op kanaal ch na een wissel. Laat de zending vallen als de pool
 * vol is — met de juiste poolgrootte is dat een defect, dus met teller
 * en melding, nooit stil. */
static void bound_send(mimeshi_node *n, uint8_t ch, const uint8_t *pkt, uint16_t len, uint64_t airtime) {
    for (int i = 0; i < MC_BOUND_POOL; i++) {
        if (bctx[i].len == 0) {
            bctx[i].ch = ch; bctx[i].len = len; bctx[i].airtime = airtime;
            memcpy(bctx[i].pkt, pkt, len);
            n->switch_ch(n, ch, MC_S_SWITCH_US, bound_fire, &bctx[i]);
            return;
        }
    }
    bctx_drops++;
    printf("[core] node %u: bound-pool vol, zending vervalt (teller %u)\n", n->id, bctx_drops);
}

/* ---------------- hello (4.1) ---------------- */

/* Eén hello-ronde: hetzelfde pakket op alle drie de kanalen, in vaste
 * volgorde. Redenen (2): buren zitten willekeurig op alle thuiskanalen,
 * en de vaste volgorde geeft responders een voorspelbaar moment waarop
 * de verzender weer op zijn eigen kanaal luistert — hun hello-acks
 * gaan daarheen. Kosten: 3 x 5 B met adaptief interval. */
static void hello_round(mimeshi_node *n) {
    uint8_t p[5] = { PKT_HELLO, 0,0,0,0 };
    wr16(p + 1, n->id); wr16(p + 3, n->my_state_ver);
    for (uint8_t c = 0; c < MC_N_CHANNELS; c++) {          /* vaste volgorde */
        bound_send(n, c, p, 5, t_hello_us());
    }
    /* na de ronde is de node thuis; hello-acks komen daar (binnen het venster) */
}

/* Random fase per ronde. Zonder jitter geldt (4.1): nodes die tegelijk
 * opstarten (bijv. na stroomuitval) houden exact synchrone rondes —
 * de intervallen verdubbelen synchroon — en horen elkaar nooit, want
 * tijdens de eigen ronde is een node van huis. Range in hello_jitter_us
 * zodat de simulatie hem per scenario kan doormeten (open punt 3). */
static uint64_t hello_jitter(mimeshi_node *n) {
    return n->hello_jitter_us ? (n->rand(n) % n->hello_jitter_us) : 0;
}

/* Ronde draaien, interval verdubbelen (30 s -> 60 s -> ... -> 1 u) en de
 * volgende ronde plannen. De verdubbeling is per (4.1): langer aanwezige
 * nodes storen minder. De jitter zit op de timer, niet op de ronde zelf. */
static void hello_tick(mimeshi_node *n, void *ud) {
    (void)ud;
    hello_round(n);
    if (n->hello_interval_us < MC_HELLO_MAX_US) {
        n->hello_interval_us *= 2;
        if (n->hello_interval_us > MC_HELLO_MAX_US) n->hello_interval_us = MC_HELLO_MAX_US;
    }
    n->timer(n, n->hello_interval_us + hello_jitter(n), hello_tick, 0);
}

/* Hello-ack afgeven op het THUISkanaal van de hello-verzender (2): de
 * verzender is na zijn ronde daar en vangt alle antwoorden binnen het
 * responstijdvenster op. Eén exchange, daarna thuis.
 *
 * Directioneel formaat (7 B): de bestemming is de hello-verzender. Alleen
 * daardoor bewijst de ack symmetrie — de responder ontving ónze hello
 * (zijn ack is een reactie daarop) en zijn ack bereikte ons (4.2, 10.1).
 * Dat is het bewijskanaal van het hello-proces: beschikbaar vóór de eerste
 * unicast. De eigen state-versie reist mee voor de pull-beslissing (4.3);
 * afluisteraars verwerken de afzender als minimale hello ("gezien"). */
static void hello_ack_fire(mimeshi_node *n, void *ud) {
    uint16_t to = (uint16_t)(uintptr_t)(size_t)ud;
    uint8_t p[7] = { PKT_HELLO_ACK, 0,0, 0,0, 0,0 };
    wr16(p + 1, to);                       /* bestemmeling: de hello-verzender */
    wr16(p + 3, n->id);
    wr16(p + 5, n->my_state_ver);
    bound_send(n, (uint8_t)(to % MC_N_CHANNELS), p, 7, t_ack_us()); /* naar THUIS van verzender */
}

/* Wachttijd en besluit voor de hello-ack, na ontvangst van een hello:
 *
 * Sterk eerst (Besloten), met slot + jitter:
 *   - RSSI bepaalt het SLOT (de rang): de sterkste responder zendt in
 *     het eerste deel van het venster.
 *   - De jitter is groter dan de airtime van een ack (okt 2026, ~187 ms
 *     t.o.v. 78 ms + wissel): bij het oude venster/64 (47 ms) overlapten
 *     acks met exact gelijke rang zeker. Nu spreidt de jitter
 *     gelijkwaardige responders wél uit.
 *   - De slotberekening reserveert jitter + S + t_ack aan het einde van
 *     het venster, zodat elke ack (incl. wissel en airtime) binnen het
 *     ontvangstvenster van de verzender valt.
 *   - Prijs: de rangordening is "zacht" — responders binnen ~16 dB
 *     kunnen van volgorde wisselen. Botsingen voorkomen weegt zwaarder
 *     dan strikte volgorde (2).
 *
 * Ack op behoefte (4.1, okt 2026): een bevestigde buur (sym, 4.2) krijgt
 * nog twee courtesy acks na onze eigen bevestiging en daarna geen
 * enkele meer — snooping onderhoudt liveness, versie en kwaliteit al.
 * De twee extra acks zijn nodig omdat bevestiging vrijwel altijd
 * asymmetrisch verloopt: als één richting eerst slaagt, stoppen de acks
 * van die kant, terwijl de búúr zijn bewijs nog uit onze ack moet
 * halen (in de simulatie: capture verloor N's richting terwijl de
 * tegenzijde al bevestigd was — zonder courtesy verhongert dat
 * bewijs tot een eventuele tablepull).
 * Een onbevestigde buur blijf in de tabel, maar de ack-kans dunnet uit
 * met de tijd: elke GEHOORDE zending van de buur telt (teller per buur),
 * en alleen de tellerstanden 1, 2, 4, 8, 16 leveren een poging, daarna
 * een druppel van gemiddeld 1 op 16 zendingen. Zo lost een ack-storm rond een toren-
 * site zichzelf op: de druk op het venster daalt, de overgebleven
 * pogingen gaan ván door, geslaagde bewijzen stoppen de acks (na de
 * courtesy) volledig — en eenzijdige links kosten hooguit een
 * druppel. Reset van de teller bij bevestiging of hernieuwd contact
 * (nbr_add); tellen bij zendingen, niet bij pogingen — anders raken
 * verloren pogingen vast zonder derde kans.
 */
static void hello_ack_wait(mimeshi_node *n, uint16_t from, int8_t rssi) {
    mimeshi_neighbor *nb = nbr_find(n, from);
    int ack = 1;                                 /* onbekende zender: altijd ack */
    if (nb) {
        if (nb->sym) {
            if (nb->ack_try >= 3) return;       /* courtesy op (na 2) */
            nb->ack_try++;
        } else {
            if (nb->ack_try < 255) nb->ack_try++;   /* teller per buur */
            uint32_t c = nb->ack_try;
            ack = c <= 16 && (c & (c - 1)) == 0;    /* pogingen op tellerstand 1,2,4,8,16 */
            if (!ack && c > 16) ack = (n->rand(n) % 16u) == 0;  /* druppel: gemiddeld 1 op 16 zendingen */
        }
    }
    if (!ack) return;
    int norm = (rssi + 128) > 255 ? 255 : (rssi + 128);   /* 0..255 */
    uint32_t span = (uint32_t)(MC_HELLO_ACK_WIN_US - MC_HELLO_ACK_JITTER_US
                             - t_ack_us() - MC_S_SWITCH_US);
    uint32_t slot = (uint32_t)((255 - norm) * span / 255u);
    uint32_t jitter = n->rand(n) % (MC_HELLO_ACK_JITTER_US + 1u);
    n->timer(n, slot + jitter, hello_ack_fire, (void *)(size_t)from);
}

/* ---------------- unicast + acks (7) ---------------- */

/* Ack1: direct, op het kanaal van de lopende data-uitwisseling — de
 * verzender wacht daar op het antwoord (wisselregel, 2). De afzender-
 * id gaat mee zodat de verzender weet wie gehoord heeft. */
static void ack1_send(mimeshi_node *n, uint16_t msg_id, uint16_t src, uint8_t ch) {
    uint8_t p[7] = { PKT_ACK1, 0,0, 0,0, 0,0 };
    wr16(p + 1, msg_id); wr16(p + 3, src); wr16(p + 5, n->id);
    n->tx(n, ch, p, 7, t_ack_us());       /* zelfde kanaal als de data (wisselregel) */
}

/* Ack2 naar de vorige hop, op DIENS thuiskanaal (2, 7.1): de vorige hop
 * luistert daar zonder eigen venster. Positief = ontvangen (niet per se
 * doorgestuurd, 7.2); negatief = doorsturen mislukt, via F_NEG_ACK2. */
static void ack2_send(mimeshi_node *n, uint16_t msg_id, uint16_t src, uint16_t prev, bool neg) {
    uint8_t p[7] = { (uint8_t)(PKT_ACK2 | (neg ? F_NEG_ACK2 : 0)), 0,0, 0,0, 0,0 };
    wr16(p + 1, msg_id); wr16(p + 3, src); wr16(p + 5, n->id);
    bound_send(n, (uint8_t)(prev % MC_N_CHANNELS), p, 7, t_ack_us()); /* THUIS van vorige hop */
}

static void pending_try(mimeshi_node *n, mimeshi_pending *p);

/* Ack1 uitgebleven (7.3/7.4). structuur van de retries:
 *  - tries_left > 0:  volgende poging richting DEZELFDE volgende hop,
 *                      tries en budget zakken met 1 per poging;
 *  - tries op:         nieuwe pogingserie (richting een andere buur,
 *                      zodra routering er is — 10.2), budget checkt
 *                      eerst of er nog ruimte is;
 *  - budget op:        bericht laten vallen. Het echte failed delivery
 *                      report (10.5, routed bericht naar de bron) komt
 *                      met bouwsteen 2; nu alleen de melding op stdout. */
static void ack1_timeout(mimeshi_node *n, void *ud) {
    mimeshi_pending *p = (mimeshi_pending *)ud;
    if (!p->active) return;
    if (p->tries_left > 0) {
        p->tries_left--; p->budget_left--;
        pending_try(n, p);
    } else {
        p->tries_left = MC_RETRY_TX;      /* volgende pogingserie */
        if (p->budget_left == 0) {
            p->active = 0;
            printf("[core] node %u: FAIL msg %u (budget op, geen ack1)\n", n->id, p->msg_id);
        } else {
            p->budget_left--;
            pending_try(n, p);            /* buurkeuze: echte kern ETX-gestuurd (10.2) */
        }
    }
}

/* Eén zendpoging: budget in de header bijwerken (offset 9, 6.1/9), de
 * exchange richting next_hop, en de ack1-timeout plannen.
 * De timeout is (7.3) S + t_data + t_ack + marge, gemeten vanaf het
 * moment van opgeven. Aanname: de platformlaag levert de zending snel
 * af (wachtrij kort); duurt de wissel langer, dan valt de timeout te
 * vroeg en telt de poging als mislukt. */
static void pending_try(mimeshi_node *n, mimeshi_pending *p) {
    p->pkt[9] = p->budget_left;
    bound_send(n, (uint8_t)(p->next_hop % MC_N_CHANNELS), p->pkt, p->pkt_len, t_data_us(p->pkt_len));
    n->timer(n, MC_S_SWITCH_US + t_data_us(p->pkt_len) + t_ack_us() + MC_ACK1_MARGIN_US, ack1_timeout, p);
}

/* Unicast opgeven. Retourcodes (core.h): 0 = geaccepteerd, -1 = bestemming
 * niet in de neighbor table (10.2: niet flooden, afwachten tot de kaart ze
 * kent), -2 = geen pending-slot vrij, -3 = payload te groot.
 * Bouwsteen 1 kiest als volgende hop de bestemming zelf (directe buur);
 * vanaf bouwsteen 2 kiest een router hier de beste hop op de 2-hop-kaart.
 *
 * Eénrichtingsbewijs: elke neighbor-table-entry ontstond door een ontvangst
 * (hello of hello-ack) en bewijst dus alleen de richting buur->wij. De
 * tegenrichting (wij->buur) wordt pas bewezen door de buur zelf: via zijn
 * tablepull (4.3) of via een geslaagde unicast met ack1. Routering
 * (bouwsteen 2, 10.1) moet die bevestiging eisen vóór doorsturen; tot
 * die tijd is dit directe sturen begrensd door retries+budget en faalt
 * het zichtbaar op de ack1-timeout. */
int mimeshi_send_data(mimeshi_node *n, uint16_t dst, const uint8_t *payload, uint16_t len, bool ack3) {
    if (!nbr_find(n, dst)) return -1;          /* niet in kaart: niet flooden (10.2) */
    if (len > MC_MAX_PAYLOAD) return -3;
    mimeshi_pending *p = 0;
    for (int i = 0; i < MC_MAX_PENDING; i++) if (!n->pend[i].active) { p = &n->pend[i]; break; }
    if (!p) return -2;
    memset(p, 0, sizeof(*p));
    p->active = 1; p->tries_left = MC_RETRY_TX; p->budget_left = MC_BUDGET_INIT;
    p->msg_id = (uint16_t)n->rand(n);
    p->src = n->id; p->dst = dst; p->prev_hop = n->id; p->next_hop = dst;
    uint8_t *k = p->pkt;
    k[0] = (uint8_t)(PKT_DATA | (ack3 ? F_ACK3_REQ : 0));
    wr16(k + 1, p->msg_id); wr16(k + 3, n->id); wr16(k + 5, dst); wr16(k + 7, n->id);
    k[9] = p->budget_left;                 /* header: budget op offset 9 (sectie 6.1) */
    memcpy(k + 10, payload, len);
    p->pkt_len = (uint16_t)(10 + len);
    pending_try(n, p);
    return 0;
}

/* ---------------- ontvangst (platform -> kern) ----------------
 * Elk ontvangen pakket komt hier binnen, met het kanaal van ontvangst
 * (bepaalt waar ack1 terug moet) en de RSSI (bepaalt het hello-ack-slot).
 * Pakketten voor een andere bestemming zijn doorstuurwerk en worden
 * bewust genegeerd tot bouwsteen 2. */
void mimeshi_rx(mimeshi_node *n, uint8_t ch, int8_t rssi, const uint8_t *pkt, uint16_t len) {
    (void)len;
    uint8_t type = pkt[0] & 0x0F, flags = pkt[0] & 0xF0;
    switch (type) {
    case PKT_HELLO: {
        /* buur bijwerken of toevoegen, daarna ack plannen. quality is de
         * RSSI-meting van deze ontvangst; de state-versie van de buur wordt
         * bijgehouden voor toekomstige tablepulls (4.3); last_seen is
         * nu nog een stub — echt verouderingsbeleid (4.4) volgt. */
        uint16_t from = rd16(pkt + 1);
        mimeshi_neighbor *nb = nbr_find(n, from);
        if (!nb) nb = nbr_add(n, from);
        if (nb) {
            nb->last_seen = 0;
            nb->quality = quality_of(rssi);
            nb->state_ver = rd16(pkt + 3);
        }
        hello_ack_wait(n, from, rssi);
        break;
    }
    case PKT_HELLO_ACK: {
        /* Directionele ack (7 B): bestemming = de oorspronkelijke
         * hello-verzender.
         *  - Aan óns gericht: de responder ontving onze hello en deze ack
         *    bereikte ons — beide richtingen werkten op dat moment:
         *    symmetrie bewezen (4.2, 10.1). Dit is het bewijskanaal van
         *    het hello-proces, beschikbaar vóór de eerste unicast.
         *  - Gericht aan een ander (snooping, 2): de afzender wordt als
         *    minimale hello verwerkt — id, state-versie (rechtstreeks van
         *    de bron) en de hier gemeten kwaliteit — maar blijft "gezien":
         *    eenrichtingsbewijs, geen symmetrie. De ack triggert geen
         *    nieuwe reactie: daarvoor dient het type-byte. */
        uint16_t dst = rd16(pkt + 1), from = rd16(pkt + 3);
        mimeshi_neighbor *nb = nbr_find(n, from);
        if (!nb) nb = nbr_add(n, from);
        if (nb) {
            nb->last_seen = 0;
            nb->quality = quality_of(rssi);
            nb->state_ver = rd16(pkt + 5);
            if (dst == n->id) { nb->sym = 1; nb->ack_try = 0; }   /* bevestigd symmetrisch (4.2) */
        }
        break;
    }
    case PKT_ACK1: {
        /* onze eigen uitstaande zending is bij de volgende hop aangekomen:
         * pending-record vrijmaken. De hop is af; ack2 komt vanzelf op ons
         * thuiskanaal (7.1). Matchen op msg-id volstaat in bouwsteen 1
         * (ids zijn random per zending); met routering erbij hoort de
         * bron erin mee te tellen.
         *
         * De afzender van deze ack1 ontving ons bericht én zijn ack bereikte
         * ons: de link is hiermee bewezen symmetrisch (4.2). Dit is het
         * eerste exacte symmetriebewijs dat de kern kent; het tweede is de
         * tablepull van de buur (4.3, bouwsteen 2). Een hello-ack bewijst
         * geen symmetrie — onadresseerd op een gedeeld thuiskanaal. */
        uint16_t msg_id = rd16(pkt + 1);
        for (int i = 0; i < MC_MAX_PENDING; i++) {
            mimeshi_pending *p = &n->pend[i];
            if (p->active && p->msg_id == msg_id) {
                p->active = 0;                     /* hop gelukt; ack2 komt vanzelf op ons thuiskanaal */
                mimeshi_neighbor *nb = nbr_find(n, rd16(pkt + 5));
                if (nb) { nb->sym = 1; nb->ack_try = 0; }  /* bevestigd symmetrisch (4.2, 10.1) */
                printf("[core] node %u: ACK1 voor msg %u\n", n->id, msg_id);
                return;
            }
        }
        break;
    }
    case PKT_ACK2: {
        /* bevestiging van de volgende hop richting de vorige (7.2). Nu
         * alleen zichtbaar; het T_ack2-venster en de pending-afhandeling
         * op de verzender (7.3) horen bij bouwsteen 2. */
        printf("[core] node %u: ACK2 %s voor msg %u\n", n->id,
               (flags & F_NEG_ACK2) ? "NEGATIEF" : "positief", rd16(pkt + 1));
        break;
    }
    case PKT_ACK3: {
        /* eindbestemming heeft het bericht afgeleverd gekregen (11):
         * routed terug naar de bron, hier nog zonder routeringslogica. */
        printf("[core] node %u: ACK3 — afgeleverd bij bestemming\n", n->id);
        break;
    }
    case PKT_DATA: {
        uint16_t msg_id = rd16(pkt + 1), src = rd16(pkt + 3), dst = rd16(pkt + 5), prev = rd16(pkt + 7);
        if (dst != n->id) return;                  /* forwarding: volgende bouwsteen */
        /* dedup vóór de ack's: een retry van dezelfde hop mag niet
         * nogmaals ack1/ack2/ack3 opleveren. */
        if (!dedup_fresh(n, msg_id, src)) return;
        printf("[core] node %u: DATA van %u: '%.*s'\n", n->id, src, (int)(len - 10), (const char *)(pkt + 10));
        ack1_send(n, msg_id, src, ch);             /* zelfde kanaal */
        ack2_send(n, msg_id, src, prev, false);    /* thuiskanaal vorige hop */
        if (flags & F_ACK3_REQ) {
            /* ack3 als routed bericht (6.2): DATA-formaat, payload = het
             * originele msg-id (4 B incl. reservering), terug naar de
             * vorige hop — tot er echte routering is (bouwsteen 2). */
            uint8_t a[14] = { PKT_ACK3, 0,0, 0,0, 0,0, 0,0, 0, 0,0, 0,0 };
            wr16(a + 1, (uint16_t)n->rand(n)); wr16(a + 3, n->id);
            wr16(a + 5, src); wr16(a + 7, n->id); a[9] = MC_BUDGET_INIT; wr16(a + 10, msg_id);
            bound_send(n, (uint8_t)(prev % MC_N_CHANNELS), a, 14, t_data_us(14));
        }
        break;
    }
    default: break;
    }
}

/* ---------------- init/start ---------------- */

/* Nuldekkking + callbacks invullen. Het thuiskanaal volgt uit het id
 * (id mod 3, 2). hello_jitter_us krijgt de firmwaredefault; de simulator
 * mag hem vóór mimeshi_start() aanpassen om de range te doormeten
 * (open punt 3). De node-struct staat in core.h zodat callers hem
 * statisch kunnen reserveren — vandaar nuldekking hier. */
void mimeshi_init(mimeshi_node *n, uint16_t id, mimeshi_tx_fn tx, mimeshi_timer_fn timer,
                mimeshi_switch_fn sw, mimeshi_rand_fn rnd, void *user) {
    memset(n, 0, sizeof(*n));
    n->id = id; n->home_ch = (uint8_t)(id % MC_N_CHANNELS);
    n->my_state_ver = 1; n->hello_interval_us = MC_HELLO_START_US;
    n->hello_jitter_us = MC_HELLO_JITTER_US;
    n->tx = tx; n->timer = timer; n->switch_ch = sw; n->rand = rnd; n->user = user;
}

/* Eerste hello-ronde direct (snelle kaartopbouw, 4.1), daarna de
 * adaptieve timer met random fase. Idempotent voor de rest van de kern:
 * na start draait de hello op de timer; mimeshi_start() zelf niet
 * herhalen. */
void mimeshi_start(mimeshi_node *n) {
    hello_round(n);
    /* random fase: voorkomt synchrone rondes bij gelijktijdig opstarten (4.1) */
    n->timer(n, n->hello_interval_us + hello_jitter(n), hello_tick, 0);
}
