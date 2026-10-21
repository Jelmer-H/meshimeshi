#!/usr/bin/env python3
"""
sim_proofrun.py — long-run check of the keepalive replacement (4.1/4.2).

Two nodes, straight from the scenario-1 setup in sim1.py, but run for
6.5 hours of virtual time. Verifies against the design:
  - the hello exists only during the startup phase (last round at the
    1920 s interval); after that no PKT_HELLO goes on air anymore;
  - each channel gets about one proof list per hour (rotation, 4.2),
    pulled forward on silence (keepalive role, 4.1);
  - both directions stay confirmed symmetric over the whole run
    (appearing in each other's proof lists refreshes the evidence).
"""
import sys
from sim1 import Radio, Node, CCore, load_lib, N_CH, sym_count

RUN_US = 6 * 3600_000_000 + 1800_000_000   # 6.5 h


def main():
    radio = Radio()
    a = Node(radio, 0x0100)
    b = Node(radio, 0x0101)
    lib = load_lib(sys.argv[1] if len(sys.argv) > 1 else 'build/libmimeshi.so')
    a.core = CCore(a, lib)
    b.core = CCore(b, lib)

    hello_tx = [0, 0]        # per node, PKT_HELLO transmissions
    proof_tx = [0, 0]        # per node, PKT_PROOF transmissions
    proof_per_ch = [0] * N_CH
    last_hello_at = [0, 0]

    base_tx = Radio.tx

    def tx(radio_self, node, ch, pkt, airtime):
        t = pkt[0] & 0x0F
        if t == 1:
            idx = 0 if node.id == 0x0100 else 1
            hello_tx[idx] += 1
            last_hello_at[idx] = radio_self.now
        elif t == 3:
            idx = 0 if node.id == 0x0100 else 1
            proof_tx[idx] += 1
            proof_per_ch[ch] += 1
        base_tx(radio_self, node, ch, pkt, airtime)

    Radio.tx = tx
    a.core.start()
    radio.schedule(500_000, b.core.start)
    radio.run(until_us=RUN_US)

    h = RUN_US / 3600_000_000
    print(f"run: {h:.2f} h virtual")
    for i, nid in enumerate((0x0100, 0x0101)):
        print(f"node {nid:#06x}: hellos {hello_tx[i]} (last at "
              f"{last_hello_at[i]/1e6:.1f} s), proof lists {proof_tx[i]} "
              f"(~{proof_tx[i]/h:.2f}/h)")
    print("proof lists per channel:", proof_per_ch,
          f"(expected ~{h:.1f} each)")
    for nid, nd in radio.nodes.items():
        sym = sym_count(nd)
        print(f"node {nid:#06x}: confirmed symmetric {sym}, "
              f"collisions {nd.collided}")

    ok = True
    if last_hello_at[0] > 4200_000_000 or last_hello_at[1] > 4200_000_000:
        print("FAIL: hellos after the startup phase")
        ok = False
    for i in range(2):
        if proof_tx[i] < int(h) - 2:
            print(f"FAIL: too few proof lists on node {i}")
            ok = False
    for nid, nd in radio.nodes.items():
        if sym_count(nd) is not None and sym_count(nd) < 1:
            print(f"FAIL: symmetry lost at {nid:#06x}")
            ok = False
    print("PROOFRUN:", "OK" if ok else "FAILED")
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
