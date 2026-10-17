/*
 * host_radio.c — hardware-laag voor ontwikkeling op een werkstation (stub).
 *
 * De echte platforms (nrf52/, esp32/) vervangen alleen deze laag; de kern
 * blijft ongewijzigd (ontwerp 13b). Deze stub logt de vier callbacks op stdout
 * zodat de koppeltest ook zonder Python draaibaar is.
 */
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include "core/core.h"

void host_tx(mimeshi_node *n, uint8_t ch, const uint8_t *pkt, uint16_t len, uint64_t airtime) {
    (void)n;
    printf("[host] tx kanaal %u: type %02X (%u B, %llu ms)\n",
           ch, pkt[0], len, (unsigned long long)(airtime / 1000));
}

void host_timer(mimeshi_node *n, uint64_t delay_us, void (*cb)(mimeshi_node *, void *), void *ud) {
    (void)n; (void)delay_us; (void)cb; (void)ud;
    /* stub: echte timing zit in de simulator, niet hier */
}

void host_switch(mimeshi_node *n, uint8_t ch, uint64_t s_us, void (*done)(mimeshi_node *, void *), void *ud) {
    (void)n; (void)ch;
    if (done) done(n, ud);
    (void)s_us;
}

uint32_t host_rand(mimeshi_node *n) {
    (void)n;
    return (uint32_t)rand();
}
