/*
 * test_koppeltest.c — scenario 1 (link test) directly on the C core.
 *
 * Goal (design 14.1): verify that the core produces the right packet
 * formats, follows the switch rule (hello round over 3 channels, ack1 on
 * the data channel, ack2 to the home channel of the previous hop) and
 * runs through the unicast path with acks.
 *
 * This test uses a mini event queue instead of the Python simulator:
 * it tests the core stand-alone, the simulator remains the second
 * opinion. Since the per-channel keepalive (4.1) it also verifies that:
 * after the startup rounds exactly one hello follows per silent channel,
 * and an own call on a channel resets only that counter.
 */
#include <stdio.h>
#include <string.h>
#include <assert.h>

#include "core/core.h"

/* --- mini virtual radio ---------------------------------------------- */
#define MAXEV 1024
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

/* --- record what the core does ---------------------------------------- */
static int tx_count, hello_seen, switch_count;
static uint8_t last_type, last_ch;
static int hello_cnt_A[3], hello_cnt_B[3];   /* hellos per channel per node (keepalive, 4.1) */

static void vrtx(mimeshi_node *n, uint8_t ch, const uint8_t *pkt, uint16_t len, uint64_t air) {
    (void)len; (void)air;
    tx_count++; last_type = pkt[0]; last_ch = ch;
    if ((pkt[0] & 0x0F) == 1) {              /* PKT_HELLO */
        hello_seen++;
        if (n->id == 0x0102) hello_cnt_A[ch]++;
        else if (n->id == 0x0100) hello_cnt_B[ch]++;
    }
    /* delivery to the other node in the test is not needed: we test
       the core output, not the circulating chain (sim1.py does that). */
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

    /* Home channel = id mod 3 (section 2). 0x0102 = 258 -> home 0,
     * 0x0100 = 256 -> home 1: exactly the channels this test expects. */
    mimeshi_init(&A, 0x0102, vrtx, sched, vrswitch, vrrand, 0);
    mimeshi_init(&B, 0x0100, vrtx, sched, vrswitch, vrrand, 0);
    assert(A.home_ch == 0 && B.home_ch == 1);

    mimeshi_start(&A);
    mimeshi_start(&B);

    /* Assert 1: the hello round transmits on all three channels (3 per node). */
    pump_until(1000000);
    assert(hello_seen == 6);

    /* Assert 2: refuse a unicast to an unknown destination (10.2: no flooding). */
    assert(mimeshi_send_data(&A, 0x0BEE, (const uint8_t *)"x", 1, false) == -1);

    /* Assert 3: after a received hello, B is a neighbor of A and vice
     * versa. Each hello is injected here on the home channel of the
     * receiver, just as the radio would really deliver it. */
    uint8_t helloB[5] = { 1, 0x00, 0x01, 1, 0 };      /* PKT_HELLO from 0x0100 */
    mimeshi_rx(&A, 0, -55, helloB, 5);
    uint8_t helloA[5] = { 1, 0x02, 0x01, 1, 0 };       /* PKT_HELLO from 0x0102 */
    mimeshi_rx(&B, 1, -60, helloA, 5);
    pump_until(1000000);

    /* hello-ack from A toward B: exchange to B's home channel (1). */
    assert(mimeshi_send_data(&B, 0x0102, (const uint8_t *)"hello mesh", 10, true) == 0);
    pump_until(100000); /* switch (5 ms) + tx; not up to the ack1 timeout */
    /* B transmits data on A's home channel 0. */
    assert(last_ch == 0 && (last_type & 0x0F) == 5);

    /* Assert 4: a neighbor learned via hello stands on "seen": one
     * direction, symmetry not yet proven (4.2). B knew A from the
     * injected hello. */
    {
        int seen = 0;
        for (int i = 0; i < MC_MAX_NEIGHBORS; i++)
            if (B.nbr[i].alive && B.nbr[i].id == 0x0102) seen = B.nbr[i].sym;
        assert(seen == 0);
    }

    /* Assert 5: a received ack1 confirms the symmetry of the sender
     * (4.2): the neighbor received our message and its ack reached us.
     * B's unicast carries msg-id 42 (vrrand yields 42); A's ack1
     * arrives. */
    uint8_t ack1[7] = { 6, 42,0, 0x00,0x01, 0x02,0x01 };   /* ACK1: msg 42, source B, sender A */
    mimeshi_rx(&B, 1, -70, ack1, 7);                       /* B home on channel 1 */
    {
        int sym_ok = 0;
        for (int i = 0; i < MC_MAX_NEIGHBORS; i++)
            if (B.nbr[i].alive && B.nbr[i].id == 0x0102) sym_ok = B.nbr[i].sym;
        assert(sym_ok == 1);
    }

    /* Assert 6: hello-acks are directional (7 B, section 2). Only an
     * ack addressed to US proves symmetry — the responder received our
     * hello and its ack reached us. An overheard ack (for someone else)
     * puts the sender at most on "seen" (4.2). */
    uint8_t hack_for_a[7] = { 2, 0x02,0x01, 0x00,0x01, 1,0 };  /* from B, for A */
    mimeshi_rx(&A, 0, -70, hack_for_a, 7);
    {
        int sym_b = 0;
        for (int i = 0; i < MC_MAX_NEIGHBORS; i++)
            if (A.nbr[i].alive && A.nbr[i].id == 0x0100) sym_b = A.nbr[i].sym;
        assert(sym_b == 1);
    }
    uint8_t hack_for_c[7] = { 2, 0x05,0x01, 0x07,0x01, 1,0 };  /* from 0x0107, for 0x0105: snooping */
    mimeshi_rx(&A, 0, -70, hack_for_c, 7);
    {
        int seen = -1;
        for (int i = 0; i < MC_MAX_NEIGHBORS; i++)
            if (A.nbr[i].alive && A.nbr[i].id == 0x0107) seen = A.nbr[i].sym;
        assert(seen == 0);              /* seen, not confirmed */
    }

    /* --- keepalive (4.1): after the startup rounds (0-3810 s; the
     * interval doubling happens after the round, so the last round
     * sits on the 1920 s interval and the stop only comes at the 1-hour
     * interval) the hello only survives as per-channel keepalive:
     * exactly one hello after T_keepalive of silence on a channel, and
     * an own call resets only THAT channel. */
    pump_until(4500000000ull);            /* past all startup rounds */
    int rA[3] = { hello_cnt_A[0], hello_cnt_A[1], hello_cnt_A[2] };
    int rB[3] = { hello_cnt_B[0], hello_cnt_B[1], hello_cnt_B[2] };
    /* identifying traffic for A on channel 1 (ack1 + ack2 there):
     * resets only channel 1 of A; 0 and 2 keep counting from the round
     * at 3810 s. */
    uint8_t data[11] = { 5, 7,0, 0x00,0x01, 0x02,0x01, 0x00,0x01, 32, 'x' };
    mimeshi_rx(&A, 1, -60, data, 11);
    pump_until(7500000000ull);            /* ~1 h of silence: channels 0 and 2 of A (since 3810 s) */
    assert(hello_cnt_A[0] == rA[0] + 1);       /* keepalive after ~1 h of silence */
    assert(hello_cnt_A[2] == rA[2] + 1);
    assert(hello_cnt_A[1] == rA[1]);           /* reset by traffic on channel 1 */
    assert(hello_cnt_B[0] == rB[0] + 1 && hello_cnt_B[1] == rB[1] + 1
        && hello_cnt_B[2] == rB[2] + 1);       /* B silent: all three channels */
    pump_until(8200000000ull);            /* A's channel 1: silent since 4500 s */
    assert(hello_cnt_A[1] == rA[1] + 1);       /* after all exactly one keepalive */
    assert(hello_cnt_B[0] == rB[0] + 1);       /* B gets no second: counter resets on the call */

    printf("link-test: OK (tx=%d, hellos=%d, switches=%d)\n", tx_count, hello_seen, switch_count);
    return 0;
}
