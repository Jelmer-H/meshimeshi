/*
 * host_radio.c — hardware layer for development on a workstation (stub).
 *
 * The real platforms (nrf52/, esp32/) only replace this layer; the core
 * stays unchanged (design 13b). This stub logs the four callbacks to
 * stdout so the link test also runs without Python.
 */
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include "core/core.h"

void host_tx(mimeshi_node *n, uint8_t ch, const uint8_t *pkt, uint16_t len, uint64_t airtime) {
    (void)n;
    printf("[host] tx channel %u: type %02X (%u B, %llu ms)\n",
           ch, pkt[0], len, (unsigned long long)(airtime / 1000));
}

void host_timer(mimeshi_node *n, uint64_t delay_us, void (*cb)(mimeshi_node *, void *), void *ud) {
    (void)n; (void)delay_us; (void)cb; (void)ud;
    /* stub: real timing lives in the simulator, not here */
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

bool host_busy(mimeshi_node *n, uint8_t ch) {
    (void)n; (void)ch;
    return false;   /* stub: real CAD lives in the SX1262 driver */
}
