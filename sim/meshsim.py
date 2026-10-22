#!/usr/bin/env python3
"""
meshsim.py — Meshimeshi simulator on simpy, with a scenario registry.

The engine here is a simpy.Environment (time in microseconds, integers),
with exactly the same Radio interface as sim1.Radio. Everything that is
not a clock — radio model per node (Node), channel model (Wire), C core
via ctypes (CCore) — is reused from sim1.py. The link test via this
script must give byte-for-byte the same output as sim1.py; that is the
engine check between the hand-written event queue and simpy.

Scenarios are registered with @scenario("name") and run with:
  .venv/bin/python sim/meshsim.py <name> [-q] [--jitter US]
New scenarios: add a function with an (mk, jitter) signature, where
mk(node) builds the C protocol core (CCore; there is no other core) and
jitter is the max hello phase jitter in microseconds (None = core value).
"""
import argparse, os, sys

import random

import simpy

from sim1 import (radio_log, Node, CCore, load_lib, Wire, sym_count, nbr_counts,
                  load_nodes_csv, project, linkbudget, RX_MIN_DBM, DutyLedger, N_CH)

class Radio:  # simpy version of the virtual radio; interface equal to sim1.Radio
    def __init__(self):
        self.env = simpy.Environment(initial_time=0)
        self.nodes = {}        # id -> Node
        self.airtime = [0]*N_CH
        self.hello_air = [0]*N_CH   # airtime of hellos (type 1) per channel
        self.ack_air = [0]*N_CH     # airtime of hello-acks (type 2) per channel
        self.duty = DutyLedger()
        self.wire = Wire()
    @property
    def now(self):
        return self.env.now
    def schedule(self, delay_us, fn, arg=None):
        ev = self.env.timeout(int(delay_us))
        # callbacks receive the event; the scenario code has no use for it
        ev.callbacks.append((lambda _e: fn()) if arg is None else (lambda _e: fn(arg)))
    def run(self, until_us=None):
        """Runs up to and including until_us (or empty), and leaves the clock
        on the last event — just like sim1.Radio.run."""
        if until_us is None:
            while self.env.peek() < float('inf'):
                self.env.step()
        else:
            while self.env.peek() <= until_us:
                self.env.step()
    def tx(self, node, ch, pkt, airtime):
        radio_log(f"[{self.now/1e6:8.3f}s] node {node.id:#06x} transmits type {pkt[0]:#04x} "
                  f"({len(pkt)} B, {airtime//1000} ms) on channel {ch}")
        self.airtime[ch] += airtime
        self.wire.begin(self.now, ch, node.id, airtime)
        ptype = pkt[0] & 0x0F
        if ptype == 1: self.hello_air[ch] += airtime
        elif ptype == 2: self.ack_air[ch] += airtime
        self.duty.add(node.id, ch, self.now, airtime)
        for r in self.nodes.values():
            if r is node: continue
            r.schedule_recv(airtime, ch, bytes(pkt), node.id)   # audibility is checked by the receiver itself

# ---------------- scenarios ---------------------------------------------

SCENARIOS = {}   # name -> (fn, description)

def scenario(name, desc=""):
    def deco(fn):
        SCENARIOS[name] = (fn, desc)
        return fn
    return deco

def out(*a):
    print(*a, flush=True)

def set_jitter(core, jitter_us):
    """Sets the max hello phase jitter on the core (before start). None or
    a negative value leaves the core value in place (MC_HELLO_JITTER_US = 1 s)."""
    if jitter_us is not None and jitter_us >= 0:
        core.mn.hello_jitter_us = jitter_us
    return core

def report(radio):
    out("\n=== Measurements ===")
    for c in range(N_CH):
        out(f"channel {c}: airtime {radio.airtime[c]/1e6:.3f} s, "
            f"lost receptions {radio.wire.collided[c]}")
    for nid, nd in radio.nodes.items():
        sym = sym_count(nd)
        out(f"node {nid:#06x}: home-deafness {nd.away_total/1e6:.3f} s "
            f"({100*nd.away_total/max(radio.now,1):.2f}% of the time), "
            f"hellos heard {nd.hello_rx}, missed {nd.hello_missed}, "
            f"collisions {nd.collided}"
            + (f", confirmed symmetric {sym}" if sym is not None else ""))

@scenario("link-test", "2 nodes: hellos, unicast A->B and B->A with ack1/ack2/ack3")
def link_test(mk, jitter):
    """Same scenario as sim1.py: also serves as the equivalence check of the engines."""
    radio = Radio()
    a = Node(radio, 0x0100)   # home channel 1
    b = Node(radio, 0x0101)   # home channel 2
    a.core = set_jitter(mk(a), jitter); b.core = set_jitter(mk(b), jitter)

    a.core.start()
    radio.schedule(500_000, b.core.start)   # start offset: no overlap between
    radio.run(until_us=1_500_000)           # the first rounds (see sim1.py)

    out("\n--- unicast A -> B (A knows B from the hello round) ---")
    a.core.send_data(0x0101, "hello mesh", ack3=True)
    radio.run(until_us=3_000_000)          # hello-ack from A shows B the map

    out("\n--- unicast B -> A (after hello-ack: B knows A) ---")
    b.core.send_data(0x0100, "reply message", ack3=True)
    radio.run(until_us=50_000_000)         # second hello rounds (30 s + jitter)
    report(radio)

@scenario("hello-interval", "2 nodes, 95 s: adaptive hello interval 30 s -> 60 s -> ...")
def hello_interval(mk, jitter):
    radio = Radio()
    a = Node(radio, 0x0100)
    b = Node(radio, 0x0101)
    a.core = set_jitter(mk(a), jitter); b.core = set_jitter(mk(b), jitter)
    a.core.start()
    radio.schedule(500_000, b.core.start)
    radio.run(until_us=95_000_000)         # rounds 2 (30 s) and 3 (60 s) + jitter
    report(radio)

@scenario("jitter-sweep", "2 nodes, 1000 s: round sync against the jitter range (open point 3)")
def jitter_sweep(mk, _jitter):
    """Measures per jitter range how many hellos a node hears and misses
    from the other (own round / away from home). Preparation for scenario 2;
    the CLI jitter stands outside this, the sweep determines it itself."""
    out(f"{'jitter':>9s} {'A hears':>8s} {'A misses':>8s} {'B hears':>8s} {'B misses':>8s}")
    for j in (0, 100_000, 500_000, 1_000_000, 5_000_000):
        radio = Radio()
        a = Node(radio, 0x0100); b = Node(radio, 0x0101)
        a.core = set_jitter(mk(a), j); b.core = set_jitter(mk(b), j)
        a.core.start()
        radio.schedule(137_000, b.core.start)
        radio.run(until_us=1_000_000_000) # rounds up to ~930 s (30-60-120-240-480 s)
        out(f"{j/1e6:>9.3f} {a.hello_rx:>8d} {a.hello_missed:>8d} "
            f"{b.hello_rx:>8d} {b.hello_missed:>8d}")

@scenario("newcomer", "stable R1/R2 + newcomer at 5 s: who proves symmetry and when")
def newcomer(mk, jitter):
    """Demonstrates the proof asymmetry after startup (4.2/4.3): the
    newcomer N gets its symmetry proofs within seconds (acks on its own
    round, addressed to it); the responders R1/R2 then still have N as
    "seen". Their reverse-direction proof only comes via the hello
    process — N acks their round 2 at ~30 s — or later via the startup
    pull (4.3, building block 2). N only exists from t=5 s, so round 1
    of R1/R2 is genuinely missed."""
    radio = Radio()
    r1 = Node(radio, 0x0100)   # home 1
    r2 = Node(radio, 0x0101)   # home 2
    r1.core = set_jitter(mk(r1), jitter); r2.core = set_jitter(mk(r2), jitter)
    # Heterogeneous links (more realistic than the default -60): responders
    # with different RSSI ranks land in different slots, so acks to the
    # same receiver do not structurally collide (see design 2, note).
    radio.wire.links[(0x0100, 0x0101)] = -70   # R1 -> R2
    radio.wire.links[(0x0101, 0x0100)] = -66   # R2 -> R1
    radio.wire.links[(0x0100, 0x0102)] = -57   # R1 -> N
    radio.wire.links[(0x0102, 0x0100)] = -55   # N -> R1
    radio.wire.links[(0x0101, 0x0102)] = -68   # R2 -> N
    radio.wire.links[(0x0102, 0x0101)] = -58   # N -> R2
    nn = [None]
    def start_newcomer():
        nn[0] = Node(radio, 0x0102)   # home 0
        nn[0].core = set_jitter(mk(nn[0]), jitter)
        nn[0].core.start()
    def symstatus(label):
        out(f"[sym] {label}: R1={sym_count(r1)} R2={sym_count(r2)} N={sym_count(nn[0])}")
    r1.core.start()
    radio.schedule(500_000, r2.core.start)
    radio.schedule(5_000_000, start_newcomer)
    radio.schedule(10_000_000, lambda: symstatus("t=10 s, after acks on the newcomer"))
    radio.schedule(40_000_000, lambda: symstatus("t=40 s, after round 2 (hello process)"))
    radio.schedule(100_000_000, lambda: symstatus("t=100 s, after round 3"))
    radio.run(until_us=100_000_000)
    report(radio)


@scenario("newcomer-traffic", "local-net + traffic: newcomer joins a loaded running network")
def newcomer_traffic(mk, jitter):
    """Variant on the traffic scenario (14.1 scenario 2, plus the risk
    from 4.1): the whole net runs with unicast load; a newcomer with
    its own links joins at --join-at seconds and has to discover the
    net under that load. Measures per newcomer: the time to the first
    confirmed symmetric neighbor (startup hellos + hello-acks), the
    time to full confirmation, how much of the running load it missed
    (missed hellos), and whether its arrival disturbs the running
    delivery (ratio before/after). CLI: --nodes --duration --rate
    --payload --join-at (seconds, default 5400 = 1.5 h)."""
    nodes_list = load_nodes_csv(_ARGS.nodes)
    duration_us = _ARGS.duration * 3_600_000_000
    rate_per_h, payload_b = _ARGS.rate, _ARGS.payload
    join_at_us = _ARGS.join_at * 1_000_000
    radio = Radio()
    radio.wire.open = False
    lat0 = sum(n['lat'] for n in nodes_list) / len(nodes_list)
    lon0 = sum(n['lon'] for n in nodes_list) / len(nodes_list)
    pos = {n['id']: (*project(n['lat'], n['lon'], lat0, lon0), n['h']) for n in nodes_list}
    # one extra id at the centre of the map: the newcomer, with a few
    # links into the net (fading via the same rule as the rest).
    nid = max(n['id'] for n in nodes_list) + 1
    pos[nid] = (0.0, 0.0, 25)
    ids = [n['id'] for n in nodes_list] + [nid]
    links = {}
    for a in ids:
        for b in ids:
            if a == b: continue
            raw = linkbudget(pos[a], pos[b])
            if raw is None: continue
            fade = ((a * 31 + b * 17) % 11) - 5
            r = raw + fade
            if r >= RX_MIN_DBM:
                links[(a, b)] = max(-128, min(120, round(r)))
    radio.wire.links = links
    nodes = []
    for i, nd in enumerate(nodes_list):
        kn = Node(radio, nd['id'])
        kn.core = set_jitter(mk(kn), jitter)
        nodes.append(kn)
        radio.schedule(i * 250_000, kn.core.start)

    newcomer = [None]
    def start_newcomer():
        kn = Node(radio, nid)
        kn.core = set_jitter(mk(kn), jitter)
        kn.core.start()
        newcomer[0] = kn
        out(f"[newcomer] node {nid:#06x} starts at t={radio.now/1e6:.0f} s")
    radio.schedule(join_at_us, start_newcomer)

    # delivery bookkeeping (as in the traffic scenario), split around
    # the join moment to see whether the newcomer disturbs the run.
    attempted = [0, 0]; delivered = [0, 0]      # [before, after]
    def phase(): return 0 if radio.now <= join_at_us else 1
    def rx(self, arg):
        ch, rssi, pkt = arg
        if pkt and (pkt[0] & 0x0F) == 5 and len(pkt) > 10:
            dst = pkt[5] | (pkt[6] << 8)
            if dst == self.n.id:
                p = phase()
                src = pkt[3] | (pkt[4] << 8)
                mid = pkt[1] | (pkt[2] << 8)
                key = (mid, src)
                if not hasattr(rx, '_seen'): rx._seen = set()
                if key not in rx._seen:
                    rx._seen.add(key)
                    delivered[p] += 1
                else:
                    pass
        base_rx_global(self, arg)
    base_rx_global = CCore.rx
    CCore.rx = rx

    payload = bytes(range(payload_b % 221))[:payload_b] if payload_b else b'x' * 8
    rng = random.Random(2027)
    interval = 3_600_000_000 // max(rate_per_h, 1)
    def gen():
        cand = [kn for kn in nodes if sym_count(kn)]
        if newcomer[0] and newcomer[0] not in cand and sym_count(newcomer[0]):
            cand.append(newcomer[0])           # from confirmation the newcomer also sends
        if cand:
            kn = rng.choice(cand)
            peers = [e.id for e in kn.core.mn.nbr if e.alive and e.sym]
            if peers:
                rc = kn.core.send_data(rng.choice(peers), bytes(payload), ack3=False)
                if rc == 0: attempted[phase()] += 1
        radio.schedule(interval, gen)
    radio.schedule(600_000_000, gen)         # from 10 min: load before the join

    # newcomer milestones: time per new confirmed-neighbor level
    milestones = []          # (level, t_us); one entry per level
    def check():
        kn = newcomer[0]
        if kn:
            s = sym_count(kn) or 0
            while s > len(milestones):
                milestones.append(radio.now)
                out(f"[newcomer] confirmed neighbors {len(milestones)} "
                    f"at t={radio.now/1e6:.0f} s "
                    f"({(radio.now-join_at_us)/1e6:.0f} s after start)")
        radio.schedule(1_000_000, check)
    radio.schedule(join_at_us + 1_000_000, check)
    radio.run(until_us=duration_us)

    out("\n=== Measurements ===")
    for i, lbl in enumerate(("before the join", "after the join")):
        att, dlv = attempted[i], delivered[i]
        out(f"unicast {lbl}: attempts {att}, delivered {dlv} "
            f"({100*dlv/max(att,1):.1f}%)")
    kn = newcomer[0]
    if kn:
        s = sym_count(kn) or 0
        t1 = (milestones[0]-join_at_us)/1e6 if milestones else None
        full = (milestones[-1]-join_at_us)/1e6 if milestones else None
        out(f"newcomer {kn.id:#06x}: first confirmed after {t1 if t1 is None else f'{t1:.0f} s'}, "
            f"now {s} confirmed, missed hellos {kn.hello_missed}, "
            f"collisions {kn.collided}")
    out(f"running nodes: confirmed symmetric avg "
        f"{sum(nbr_counts(kn)[1] for kn in nodes)/len(nodes):.1f}")
    usage = {(n, c): radio.duty.usage(n, c, radio.now) for n in ids for c in range(N_CH)}
    top = sorted(usage.values(), reverse=True)[:5]
    out(f"duty (sliding hour, limit 360 s): max {top[0]/1e6:.1f} s, "
        f"top 5 {[f'{u/1e6:.1f}' for u in top]} s, "
        f"above limit: {sum(1 for u in usage.values() if u > 360_000_000)}")

def grid_geo(lat, lon):
    """16-bit grid encoding of section 5: 180 degrees over 65536 steps
    for latitude, 360 for longitude (offset +32768)."""
    la = int((lat + 90.0) / 180.0 * 65536)
    lo = int((lon + 180.0) / 360.0 * 65536) + 32768
    return max(1, min(65535, la)), max(1, min(65535, lo))

_ARGS = None   # CLI context for scenarios (local-net reads --nodes/--duration)

@scenario("local-net", "GPS node list (position+height) -> link budget; hello/ack measurement")
def local_net(mk, jitter):
    """Design 14.1 scenario 2: 30-50 nodes at real density. Measures
    hello and ack airtime (startup vs stable), ack collisions, and the
    seen/confirmed ratio (symmetry), plus duty cycle (10%/hour)."""
    nodes_list = load_nodes_csv(_ARGS.nodes)
    duration_us = _ARGS.duration * 3_600_000_000
    radio = Radio()
    radio.wire.open = False    # closed world: only link-budget pairs exist
    lat0 = sum(n['lat'] for n in nodes_list) / len(nodes_list)
    lon0 = sum(n['lon'] for n in nodes_list) / len(nodes_list)
    pos = {n['id']: (*project(n['lat'], n['lon'], lat0, lon0), n['h']) for n in nodes_list}
    nodes = []
    for i, nd in enumerate(nodes_list):
        kn = Node(radio, nd['id'])
        kn.core = set_jitter(mk(kn), jitter)
        kn.core.set_geo(*grid_geo(nd['lat'], nd['lon']))   # published geo (5, 4.3)
        nodes.append(kn)
        radio.schedule(i * 250_000, kn.core.start)   # never start synchronously
    ids = [n['id'] for n in nodes_list]
    for a in ids:
        for b in ids:
            if a == b: continue
            raw = linkbudget(pos[a], pos[b])
            if raw is None: continue
            fade = ((a * 31 + b * 17) % 11) - 5     # deterministic per direction
            r = raw + fade
            if r >= RX_MIN_DBM:
                radio.wire.links[(a, b)] = max(-128, min(120, round(r)))
    pairs = {(a, b) if a < b else (b, a) for (a, b) in radio.wire.links}
    bidir = sum(1 for (a, b) in pairs
                if (a, b) in radio.wire.links and (b, a) in radio.wire.links)
    degree = {i: 0 for i in ids}
    for (a, b) in pairs:
        degree[a] += 1; degree[b] += 1
    out(f"{len(nodes_list)} nodes from {_ARGS.nodes}")
    out(f"topology: {len(pairs)} links ({bidir} in both directions), "
        f"average degree {sum(degree.values())/len(ids):.1f}, "
        f"min {min(degree.values())}, max {max(degree.values())}")
    phase = {}
    def phase_border():   # after the startup rounds (last at ~1890 s); after that only keepalive (4.1)
        phase.update(air=radio.airtime[:], hello=radio.hello_air[:],
                     ack=radio.ack_air[:], t=radio.now)
    radio.schedule(3_700_000_000, phase_border)
    radio.run(until_us=duration_us)
    out("\n=== Measurements ===")
    t1 = phase.get('t', radio.now)
    out(f"phase 1: startup 0-{t1/1e6:.0f} s | phase 2: stable "
        f"{t1/1e6:.0f}-{radio.now/1e6:.0f} s (last event at {radio.now/1e6:.0f} s)")
    for c in range(N_CH):
        a1 = phase['air'][c] if phase else 0
        h1 = phase['hello'][c] if phase else 0
        k1 = phase['ack'][c] if phase else 0
        a2, h2, k2 = radio.airtime[c]-a1, radio.hello_air[c]-h1, radio.ack_air[c]-k1
        stable_us = max(radio.now - t1, 1)
        out(f"channel {c}: startup {a1/1e6:6.1f} s air (hello {h1/1e6:5.1f}, ack {k1/1e6:5.1f}) | "
            f"stable {a2/1e6:6.1f} s — hello {100*h2/stable_us:.2f}% and "
            f"ack {100*k2/stable_us:.2f}% of channel time")
    stats = [nbr_counts(kn) for kn in nodes]
    seen = [s[0] for s in stats]; conf = [s[1] for s in stats]
    out(f"neighbors per node: seen avg {sum(seen)/len(seen):.1f} (min {min(seen)}, max {max(seen)}), "
        f"confirmed symmetric avg {sum(conf)/len(conf):.1f}")
    out(f"nodes without confirmed neighbor: {sum(1 for b in conf if b == 0)} of {len(conf)}")
    maps = [kn.core.map_size() for kn in nodes]
    links = [kn.core.link_count() for kn in nodes]
    geos = [sum(1 for m in kn.core.mn.map if m.alive and m.geo_lat) for kn in nodes]
    out(f"2-hop map: avg {sum(maps)/len(maps):.1f} nodes (min {min(maps)}, max {max(maps)}), "
        f"links avg {sum(links)/len(links):.1f}, "
        f"entries with geo avg {sum(geos)/len(geos):.1f}")
    out(f"collisions: {sum(radio.wire.collided)} lost receptions, "
        f"{sum(kn.hello_missed for kn in nodes)} missed hellos")
    usage = {(n, c): radio.duty.usage(n, c, radio.now) for n in ids for c in range(N_CH)}
    top = sorted(usage.values(), reverse=True)[:5]
    out(f"duty (sliding hour window, limit 10% = 360 s): max {top[0]/1e6:.1f} s, "
        f"top 5: {[f'{u/1e6:.1f}' for u in top]} s, "
        f"above the limit: {sum(1 for u in usage.values() if u > 360_000_000)}")

@scenario("collision", "3 nodes: synchronous hellos collide; capture at >= 6 dB difference")
def collision(mk, jitter):
    """Power outage case (design 4.1): A and B share a home channel and
    start exactly synchronously (jitter 0) — their hellos overlap on all
    three channels. Round 1: equal strength, so C loses both (collision).
    Round 2: B is >= 6 dB stronger at C, so C captures B's hello and
    loses A's."""
    radio = Radio()
    a = Node(radio, 0x0102)   # home 0
    b = Node(radio, 0x0105)   # home 0 — same home channel as A
    c = Node(radio, 0x0103)   # home 1 — hears A and B on channel 1
    a.core = set_jitter(mk(a), 0); b.core = set_jitter(mk(b), 0)
    c.core = set_jitter(mk(c), jitter)
    a.core.start(); b.core.start()   # exactly simultaneous: the 4.1 edge case
    radio.schedule(500_000, c.core.start)
    out("\n--- round 1: equal strength (default -60) -> collisions at C ---")
    radio.run(until_us=1_000_000)
    radio.wire.links[(a.id, c.id)] = -66
    radio.wire.links[(b.id, c.id)] = -60
    out("\n--- round 2 (30 s): B >= 6 dB stronger -> C captures B, loses A ---")
    radio.run(until_us=31_000_000)
    report(radio)


@scenario("traffic", "local-net + periodic unicast between confirmed symmetric neighbor pairs")
def traffic(mk, jitter):
    """Design 14.1 scenario 2 with workload: after the startup phase,
    nodes send unicaasts to confirmed symmetric neighbors (10.1) at a
    fixed request rate. Measures delivery ratio, retries/airtime under
    load, collisions, home-deafness and duty against the 10% limit, and
    whether the resting load (proof lists) still fits next to it.
    CLI: --nodes --duration (hours) --rate (unicasts per hour per node),
    --payload (bytes)."""
    nodes_list = load_nodes_csv(_ARGS.nodes)
    duration_us = _ARGS.duration * 3_600_000_000
    rate_per_h = _ARGS.rate
    payload_b = _ARGS.payload
    radio = Radio()
    radio.wire.open = False
    lat0 = sum(n['lat'] for n in nodes_list) / len(nodes_list)
    lon0 = sum(n['lon'] for n in nodes_list) / len(nodes_list)
    pos = {n['id']: (*project(n['lat'], n['lon'], lat0, lon0), n['h']) for n in nodes_list}
    nodes = []
    for i, nd in enumerate(nodes_list):
        kn = Node(radio, nd['id'])
        kn.core = set_jitter(mk(kn), jitter)
        nodes.append(kn)
        radio.schedule(i * 250_000, kn.core.start)
    ids = [n['id'] for n in nodes_list]
    for a in ids:
        for b in ids:
            if a == b: continue
            raw = linkbudget(pos[a], pos[b])
            if raw is None: continue
            fade = ((a * 31 + b * 17) % 11) - 5
            r = raw + fade
            if r >= RX_MIN_DBM:
                radio.wire.links[(a, b)] = max(-128, min(120, round(r)))
    out(f"{len(nodes_list)} nodes from {_ARGS.nodes}, "
        f"{_ARGS.duration} h, {rate_per_h} unicast/h per node, payload {payload_b} B")

    # delivery bookkeeping: msg-id -> (src, dst, attempts) via dedup at rx
    attempted = [0]
    delivered = [0]     # DATA receptions at the destination (dedup-fresh)
    retries_seen = [0]  # duplicate receptions = retries of the same hop

    base_rx = CCore.rx
    def rx(self, arg):
        ch, rssi, pkt = arg
        if pkt and (pkt[0] & 0x0F) == 5 and len(pkt) > 10:
            dst = pkt[5] | (pkt[6] << 8)
            if dst == self.n.id:          # only the destination counts
                src = pkt[3] | (pkt[4] << 8)
                mid = pkt[1] | (pkt[2] << 8)
                key = (mid, src)
                if not hasattr(rx, '_seen'): rx._seen = set()
                if key not in rx._seen:
                    rx._seen.add(key)
                    delivered[0] += 1
                else:
                    retries_seen[0] += 1
        base_rx(self, arg)
    CCore.rx = rx

    payload = bytes(range(payload_b % 221))[:payload_b] if payload_b else b'x' * 8

    # traffic generator: every interval a random node sends to a random
    # confirmed symmetric neighbor (10.1); skipped if none yet (startup).
    # The last interval before the end of the run stays free: a send at
    # the very end would get no transmit window inside the simulated
    # time and show up as a false loss.
    rng = random.Random(2026)
    interval = 3_600_000_000 // max(rate_per_h, 1)
    def gen():
        if radio.now + interval > duration_us: return   # no send in the last interval
        cand = []
        for kn in nodes:
            if sym_count(kn):
                cand.append(kn)
        if cand:
            kn = rng.choice(cand)
            peers = [e.id for e in kn.core.mn.nbr if e.alive and e.sym]
            if peers:
                data = bytes(payload)
                kn.core.send_data(rng.choice(peers), data, ack3=False)
                attempted[0] += 1
        radio.schedule(interval, gen)
    radio.schedule(600_000_000, gen)         # from 10 min: first confirmed pairs exist
    radio.run(until_us=duration_us)

    out("\n=== Measurements ===")
    att, dlv = attempted[0], delivered[0]
    out(f"unicast attempts {att}, delivered {dlv} "
        f"({100*dlv/max(att,1):.1f}% of attempts, retries seen {retries_seen[0]})")
    for c in range(N_CH):
        out(f"channel {c}: airtime {radio.airtime[c]/1e6:.3f} s, "
            f"lost receptions {radio.wire.collided[c]}")
    stats = [nbr_counts(kn) for kn in nodes]
    conf = [s[1] for s in stats]
    out(f"confirmed symmetric avg {sum(conf)/len(conf):.1f}, "
        f"nodes without confirmed neighbor: {sum(1 for b in conf if b == 0)}")
    away = [kn.away_total for kn in nodes]
    out(f"home-deafness: avg {sum(away)/len(away)/1e6:.1f} s, max {max(away)/1e6:.1f} s "
        f"({100*max(away)/max(radio.now,1):.2f}% of the time)")
    usage = {(n, c): radio.duty.usage(n, c, radio.now) for n in ids for c in range(N_CH)}
    top = sorted(usage.values(), reverse=True)[:5]
    out(f"duty (sliding hour, limit 360 s): max {top[0]/1e6:.1f} s, "
        f"top 5 {[f'{u/1e6:.1f}' for u in top]} s, "
        f"above limit: {sum(1 for u in usage.values() if u > 360_000_000)}")

# ---------------- CLI -------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(description="Meshimeshi scenarios on simpy")
    ap.add_argument('scenario', nargs='?', default='link-test',
                    help="scenario name (default: link-test)")
    ap.add_argument('--list', action='store_true', help="show available scenarios")
    ap.add_argument('--so', default=os.path.join(os.path.dirname(__file__), '..', 'build', 'libmimeshi.so'))
    ap.add_argument('-q', '--quiet', action='store_true', help='no radio logs, only [core] lines')
    ap.add_argument('--jitter', type=int, default=None, metavar='US',
                    help='max hello phase jitter in microseconds (default: core value, 1 s)')
    ap.add_argument('--nodes', default=os.path.join(os.path.dirname(__file__), 'nodes', 'utrecht40.csv'),
                    help='node list (id,lat,lon,height_m,name) for local-net')
    ap.add_argument('--duration', type=int, default=8, metavar='HOURS',
                    help='simulation duration in hours (local-net/traffic, default 8)')
    ap.add_argument('--rate', type=int, default=6, metavar='PERHOUR',
                    help='unicasts per hour per node (traffic, default 6)')
    ap.add_argument('--payload', type=int, default=20, metavar='BYTES',
                    help='unicast payload size in bytes (traffic/newcomer-traffic, default 20)')
    ap.add_argument('--join-at', type=int, default=5400, metavar='SECONDS',
                    help='newcomer start time in seconds (newcomer-traffic, default 5400)')
    args = ap.parse_args()
    global _ARGS
    _ARGS = args

    if args.list:
        for name, (_, desc) in SCENARIOS.items():
            out(f"{name:16s} {desc}")
        return
    if args.scenario not in SCENARIOS:
        ap.error(f"unknown scenario '{args.scenario}' (choose from: {', '.join(SCENARIOS)})")
    if not os.path.exists(args.so):
        out(f"libmimeshi.so not found ({args.so}); build with gcc (see README).")
        sys.exit(1)
    # let printf of the C core appear directly after the Python prints
    import ctypes
    try:
        libc = ctypes.CDLL(None)
        libc.setbuf(ctypes.c_void_p.in_dll(libc, "stdout"), None)
    except OSError:
        pass

    import sim1
    if args.quiet:
        sim1.VERBOSE = False   # switch off radio logs; [core] lines remain

    lib = load_lib(args.so)
    mk = lambda nd: CCore(nd, lib)

    fn, desc = SCENARIOS[args.scenario]
    out(f"=== Scenario '{args.scenario}': {desc} (C core, SF7, 62.5 kHz) ===")
    fn(mk, args.jitter)

if __name__ == '__main__':
    main()
