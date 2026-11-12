/*
 * Meshimeshi protocol core, bound.c. See core/internal.h for the
 * module map; design references point into docs/design.md.
 */
#include "internal.h"
#include <string.h>
#include <stdio.h>

/* ---------------- exchange (switch rule, section 2) ---------
 * bound_send() is the only path for anything that must be sent on a
 * foreign channel: hellos, hello-acks, ack2, ack3, data. The core copies
 * the packet into a fixed pool and asks the platform layer to switch;
 * the done-callback (bound_fire) transmits as soon as the radio is on
 * channel and then frees the pool slot. The platform layer must
 * serialize switches per node (one radio) and return home after the
 * transmission.
 *
 * The pool has as many slots as pending records: every active message
 * holds at most one exchange at a time, because a retry only starts
 * after the ack1 timeout. No heap (13b); len == 0 marks a free slot.
 * On firmware one node runs per process, so MC_MAX_PENDING suffices;
 * the simulator shares the core between all nodes of the scenario —
 * build the .so with -DMC_BOUND_POOL=256 in that case (see README).
 */
#ifndef MC_BOUND_POOL
#define MC_BOUND_POOL MC_MAX_PENDING
#endif
typedef struct { uint8_t ch; uint8_t cad_tries; uint16_t len; uint64_t airtime; uint8_t pkt[16 + MC_MAX_PAYLOAD]; } bound_ctx;
static bound_ctx bctx[MC_BOUND_POOL]; /* pool per transmission; small and fixed */
static uint32_t bctx_drops;          /* full pool = defect, never silent */

static void bound_fire(mimeshi_node *n, void *ud) {
    bound_ctx *c = (bound_ctx *)ud;
    /* CAD, channel activity detection (2): after the switch the radio
     * is on channel and can sense the carrier. Busy = wait with random
     * backoff on the channel and ask again; the pool slot stays
     * reserved, so a busy channel delays but never drops the
     * transmission. After MC_CAD_MAX_TRIES the transmission goes
     * anyway: a permanently busy channel must not block the node
     * forever (hello round, proof list). Ack1 never comes through here:
     * it is a direct reply and goes without a switch and without a
     * check via tx() (8.1). Not RTS/CTS against hidden nodes (8). */
    if (n->channel_free && c->cad_tries < MC_CAD_MAX_TRIES &&
        !n->channel_free(n, c->ch)) {
        c->cad_tries++;
        n->timer(n, 1 + (n->rand(n) % MC_CAD_BACKOFF_US), bound_fire, c);
        return;
    }
    n->tx(n, c->ch, c->pkt, c->len, c->airtime);
    c->len = 0; /* slot free */
}

/* Transmit on channel ch after a switch. Drop the transmission if the
 * pool is full — with the right pool size that is a defect, so with
 * counter and message, never silent. */
void bound_send(mimeshi_node *n, uint8_t ch, const uint8_t *pkt, uint16_t len, uint64_t airtime) {
    for (int i = 0; i < MC_BOUND_POOL; i++) {
        if (bctx[i].len == 0) {
            bctx[i].ch = ch; bctx[i].cad_tries = 0; bctx[i].len = len; bctx[i].airtime = airtime;
            memcpy(bctx[i].pkt, pkt, len);
            n->switch_ch(n, ch, MC_S_SWITCH_US, bound_fire, &bctx[i]);
            return;
        }
    }
    bctx_drops++;
    printf("[core] node %u: bound-pool full, transmission dropped (counter %u)\n", n->id, bctx_drops);
}
