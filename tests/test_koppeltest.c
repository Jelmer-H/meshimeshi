/*
 * test_koppeltest.c — scenario 1 (koppeltest) direct op de C-kern.
 *
 * Doel (ontwerp 14.1): controleren dat de kern de juiste pakketformaten
 * maakt, de wisselregel volgt (hello-ronde over 3 kanalen, ack1 op het
 * datakanaal, ack2 naar het thuiskanaal van de vorige hop) en het
 * unicast-pad met ack's doorloopt.
 *
 * Deze test gebruikt een mini-eventqueue in plaats van de Python-simulator:
 * hij toetst de kern zelfstandig, de simulator blijft de second opinion.
 */
#include <stdio.h>
#include <string.h>
#include <assert.h>

#include "core/core.h"

/* --- mini virtuele radio ---------------------------------------------- */
#define MAXEV 256
typedef struct { uint64_t at; mimeshi_node *n; void (*cb)(mimeshi_node *, void *); void *ud; int live; } ev;
static ev q[MAXEV]; static int qn;
static uint64_t now;

static void sched(mimeshi_node *n, uint64_t delay, void (*cb)(mimeshi_node *, void *), void *ud) {
    assert(qn < MAXEV);
    q[qn] = (ev){ now + delay, n, cb, ud, 1 }; qn++;
}
static void pump_until(uint64_t until) {
    for (;;) {
        int best = -1;
        for (int i = 0; i < qn; i++)
            if (q[i].live && q[i].at <= until && (best < 0 || q[i].at < q[best].at)) best = i;
        if (best < 0) return;
        now = q[best].at; q[best].live = 0;
        ev e = q[best]; e.cb(e.n, e.ud);
    }
}

/* --- vastleggen wat de kern doet ---------------------------------------- */
static int tx_count, hello_seen, switch_count;
static uint8_t last_type, last_ch;

static void vrtx(mimeshi_node *n, uint8_t ch, const uint8_t *pkt, uint16_t len, uint64_t air) {
    (void)n; (void)len; (void)air;
    tx_count++; last_type = pkt[0]; last_ch = ch;
    if ((pkt[0] & 0x0F) == 1) hello_seen++;           /* PKT_HELLO */
    /* afleveren aan de andere node in de test is niet nodig: we toetsen
       de kern-uitgang, niet de ronddraaiende keten (dat doet sim1.py). */
}
static void vrswitch(mimeshi_node *n, uint8_t ch, uint64_t s, void (*done)(mimeshi_node *, void *), void *ud) {
    (void)n; (void)ch; (void)s;
    switch_count++;
    if (done) sched(n, s, done, ud);
}
static uint32_t vrrand(mimeshi_node *n) { (void)n; return 42u; }

int main(void) {
    static mimeshi_node A, B;
    qn = 0; now = 0;

    /* Thuiskanaal = id mod 3 (sectie 2). 0x0102 = 258 -> thuis 0,
     * 0x0100 = 256 -> thuis 1: precies de kanalen die deze test verwacht. */
    mimeshi_init(&A, 0x0102, vrtx, sched, vrswitch, vrrand, 0);
    mimeshi_init(&B, 0x0100, vrtx, sched, vrswitch, vrrand, 0);
    assert(A.home_ch == 0 && B.home_ch == 1);

    mimeshi_start(&A);
    mimeshi_start(&B);

    /* Assert 1: de hello-ronde zendt op alle drie de kanalen (per node 3). */
    pump_until(1000000);
    assert(hello_seen == 6);

    /* Assert 2: unicast naar een onbekende bestemming weigeren (10.2: niet flooden). */
    assert(mimeshi_send_data(&A, 0x0BEE, (const uint8_t *)"x", 1, false) == -1);

    /* Assert 3: na een ontvangen hello is B buur van A en omgekeerd.
     * Elke hello wordt hier geinjecteerd op het thuiskanaal van de ontvanger,
     * zoals de radio hem echt zou afleveren. */
    uint8_t helloB[5] = { 1, 0x00, 0x01, 1, 0 };      /* PKT_HELLO van 0x0100 */
    mimeshi_rx(&A, 0, -55, helloB, 5);
    uint8_t helloA[5] = { 1, 0x02, 0x01, 1, 0 };       /* PKT_HELLO van 0x0102 */
    mimeshi_rx(&B, 1, -60, helloA, 5);
    pump_until(1000000);

    /* hello-ack van A richting B: exchange naar B's thuiskanaal (1). */
    assert(mimeshi_send_data(&B, 0x0102, (const uint8_t *)"hallo mesh", 10, true) == 0);
    pump_until(100000); /* wissel (5 ms) + tx; niet tot de ack1-timeout */
    /* B zendt data op A's thuiskanaal 0. */
    assert(last_ch == 0 && (last_type & 0x0F) == 5);

    /* Assert 4: een via hello geleerde buur staat op "gezien": één richting,
     * nog geen symmetrie bewezen (4.2). B kende A uit de geinjecteerde hello. */
    {
        int seen = 0;
        for (int i = 0; i < MC_MAX_NEIGHBORS; i++)
            if (B.nbr[i].alive && B.nbr[i].id == 0x0102) seen = B.nbr[i].sym;
        assert(seen == 0);
    }

    /* Assert 5: een ontvangen ack1 bevestigt de symmetrie van de afzender
     * (4.2): de buur ontving ons bericht en zijn ack bereikte ons. B's
     * unicast draagt msg-id 42 (vrrand levert 42); A's ack1 komt binnen. */
    uint8_t ack1[7] = { 6, 42,0, 0x00,0x01, 0x02,0x01 };   /* ACK1: msg 42, bron B, afzender A */
    mimeshi_rx(&B, 1, -70, ack1, 7);                       /* B thuis op kanaal 1 */
    {
        int sym_ok = 0;
        for (int i = 0; i < MC_MAX_NEIGHBORS; i++)
            if (B.nbr[i].alive && B.nbr[i].id == 0x0102) sym_ok = B.nbr[i].sym;
        assert(sym_ok == 1);
    }

    /* Assert 6: hello-acks zijn directioneel (7 B, sectie 2). Alleen een
     * aan óns gerichte ack bewijst symmetrie — de responder ontving onze
     * hello en zijn ack bereikte ons. Een afgeluisterde ack (voor een
     * ander) zet de afzender hooguit op "gezien" (4.2). */
    uint8_t hack_voor_a[7] = { 2, 0x02,0x01, 0x00,0x01, 1,0 };  /* van B, voor A */
    mimeshi_rx(&A, 0, -70, hack_voor_a, 7);
    {
        int sym_b = 0;
        for (int i = 0; i < MC_MAX_NEIGHBORS; i++)
            if (A.nbr[i].alive && A.nbr[i].id == 0x0100) sym_b = A.nbr[i].sym;
        assert(sym_b == 1);
    }
    uint8_t hack_voor_c[7] = { 2, 0x05,0x01, 0x07,0x01, 1,0 };  /* van 0x0107, voor 0x0105: snooping */
    mimeshi_rx(&A, 0, -70, hack_voor_c, 7);
    {
        int gezien = -1;
        for (int i = 0; i < MC_MAX_NEIGHBORS; i++)
            if (A.nbr[i].alive && A.nbr[i].id == 0x0107) gezien = A.nbr[i].sym;
        assert(gezien == 0);              /* gezien, niet bevestigd */
    }

    printf("koppeltest: OK (tx=%d, hellos=%d, switches=%d)\n", tx_count, hello_seen, switch_count);
    return 0;
}
