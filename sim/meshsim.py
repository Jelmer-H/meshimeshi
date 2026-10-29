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
import argparse, ctypes, math, os, sys

import random

import simpy

from sim1 import (radio_log, Node, CCore, load_lib, Wire, sym_count, nbr_counts,
                  load_nodes_csv, project, linkbudget, RX_MIN_DBM, DutyLedger, N_CH,
                  StarStats)

class Radio:  # simpy version of the virtual radio; interface equal to sim1.Radio
    def __init__(self):
        self.env = simpy.Environment(initial_time=0)
        self.nodes = {}        # id -> Node
        self.airtime = [0]*N_CH
        self.hello_air = [0]*N_CH   # airtime of hellos (type 1) per channel
        self.star_air = [0]*N_CH    # airtime of star rounds (type 10) per channel (4.5)
        self.star_stats = StarStats()  # star-round measurement (4.5, scenario 6)
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
        elif ptype == 10: self.star_air[ch] += airtime
        if ptype == 10: self.star_stats.tx(self.now, node.id, ch, pkt, airtime)
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

def star_report(radio):
    """Star-round measurements (4.5, scenario 6): per-node round counts,
    the three areas (content stars / catalogue entries / missed requests),
    airtime per channel and the hard-bound check."""
    st = radio.star_stats
    s = st.summary()
    out("\n=== Star-round measurements (4.5) ===")
    out(f"nodes with rounds: {s['nodes']}, rounds: {s['rounds']}, "
        f"content stars: {s['content']}, catalogue entries: {s['catalogue']}, "
        f"missed requests: {s['missed']}")
    for c in range(N_CH):
        out(f"channel {c}: star-round airtime {radio.star_air[c]/1e6:.3f} s")
    viol = st.hard_bound_violations()
    out(f"hard-bound violations (rounds closer than one interval): {len(viol)}")
    for v in viol[:5]:
        out(f"  node {v[0]:#06x}: rounds at {v[1]/1e6:.1f}s and {v[2]/1e6:.1f}s")

@scenario("star-round", "2 nodes with the star round enabled: first rounds, "
                        "own star, catalogue and heartbeat tick (4.5, 6a warm-up)")
def star_round(mk, jitter):
    radio = Radio()
    a = Node(radio, 0x0100)
    b = Node(radio, 0x0101)
    a.core = set_jitter(mk(a), jitter); b.core = set_jitter(mk(b), jitter)
    a.core.start()
    radio.schedule(500_000, b.core.start)
    radio.run(until_us=2 * 3600 * 1_000_000)      # 2 h: own-star guarantee x2
    star_report(radio)
    # heartbeat probe: b's view of a's star
    present, tick, age, eff = b.core.star_state(0x0100)
    out(f"b's view of a's star: present={present} tick={tick} "
        f"age={age} effective_age={eff}")
    present, tick, age, eff = a.core.star_state(0x0101)
    out(f"a's view of b's star: present={present} tick={tick} "
        f"age={age} effective_age={eff}")

@scenario("star-line", "10 nodes in a line, 4 km spacing, only adjacent "
                       "neighbours audible: tick propagation, remote liveness "
                       "and duplicate work over 9 hops (4.5, 6a multi-hop)")
def star_line(mk, jitter):
    """Scenario 6a multi-hop (4.5). A line of 10 nodes at 10 m height,
    4 km apart: adjacent pairs at ~-107 dBm (well above the -110
    threshold), two hops (8 km) at ~-113 dBm - inaudible. So every node
    has exactly one symmetric neighbour on each side (ends: one).

    Measures over ~6 simulated hours (with the 15-min round: ample
    room for a tick of node 0's own star to reach node 9 in 9 hops):
      - tick propagation: the star_tick of node 0's star as seen by
        every node, hop by hop (expect: latest tick, within the
        k-window, arrived within ~hop x interval);
      - remote liveness: the effective age of node 0's star everywhere
        (expect: small - refreshed by the catalogue, not drifting
        toward the 24 h eviction);
      - duplicate work: how often node 0's star traveled as CONTENT
        per link (expect ~1 per hop: the content hash suppresses
        re-sends, the graph check suppresses echoes);
      - hard bound: one round per node per interval, all nodes."""
    N = 10
    SPACING = 4000.0            # metres; height 10 m
    H = 10.0
    duration_h = _ARGS.duration if getattr(_ARGS, 'duration', None) else 6
    duration_us = duration_h * 3_600_000_000

    radio = Radio()
    radio.wire.open = False     # closed world: only adjacent pairs
    # positions: line along the x-axis; ids 0x0100..0x0109
    pos = {}
    nodes = []
    lat0, lon0 = 52.09, 5.07    # around Utrecht - the grid_geo encoding needs real lat/lon
    for i in range(N):
        nid = 0x0100 + i
        # exact x in metres along the line:
        x, y = SPACING * i, 0.0
        pos[nid] = (x, y, H)
        kn = Node(radio, nid)
        kn.core = set_jitter(mk(kn), jitter)
        # real geo for the 16-bit grid: walk east ~4 km per step
        d_lon = SPACING / (111320.0 * math.cos(math.radians(lat0)))
        kn.core.set_geo(*grid_geo(lat0, lon0 + d_lon * i))
        nodes.append(kn)
    # links: EXPLICIT, not from the link budget. A controlled 6a
    # topology: every adjacent pair bidirectionally at -95 dBm (well
    # above the -110 threshold, comfortably below the +14 capture
    # zone of the quality gate), no other pairs - the line is exactly
    # one neighbour per side. The ID-dependent fade of local-net would
    # cut some directions below the threshold at 4 km (measured: the
    # 0x0101->0x0102 direction lands at -112 dBm and the line breaks
    # in half); the radio model itself is exercised by the other
    # scenarios - here the subject is the star round, not the link.
    for i in range(N - 1):
        a, b = 0x0100 + i, 0x0100 + i + 1
        radio.wire.links[(a, b)] = -95
        radio.wire.links[(b, a)] = -95
    # non-adjacent pairs: simply no link entries in the closed world -
    # the line has exactly N-1 bidirectional edges.

    for i, kn in enumerate(nodes):
        radio.schedule(i * 250_000, kn.core.start)   # never start synchronously
    radio.run(until_us=duration_us)

    star_report(radio)

    # hash-chain diagnosis: does the digest each node derives for node 0's
    # star match what its upstream neighbour would advertise?
    out("\n--- node 0's star: per-node content hash and tick ---")
    for i in range(1, N):
        core = nodes[i].core
        present, tick, age, eff = core.star_state(0x0100)
        if not present:
            out(f"node {0x0100+i:#06x}: not present")
            continue
        buf = (ctypes.c_uint8 * 300)()
        slb = core.lib
        sl = slb.mimeshi_star_serialize(ctypes.byref(core.mn), 0x0100, buf, 300, None)
        h = slb.mimeshi_star_content_hash(buf, sl, 3)
        out(f"node {0x0100+i:#06x}: hash={h:#04x} tick={tick} "
            f"digest={slb.mimeshi_star_digest(h, tick):#04x} slen={sl}")

    # tick propagation: node 0's star as seen by every node
    out("\n--- tick of node 0x0100's star per node (heartbeat propagation) ---")
    for i in range(1, N):
        nid = 0x0100 + i
        core = nodes[i].core
        present, tick, age, eff = core.star_state(0x0100)
        out(f"node {nid:#06x} ({i} hops): present={present} tick={tick} "
            f"age={age} effective_age={eff}")

    # duplicate work: how often node 0's star traveled as content per hop
    out("\n--- duplicate work: content carries of node 0x0100's star ---")
    st = radio.star_stats
    # count via the per-node content sum is aggregate; per-link counting
    # needs the receiver side: report the aggregate and the per-node rounds
    out(f"(aggregate: content stars total {st.summary()['content']}, "
        f"rounds {st.summary()['rounds']}, missed requests {st.summary()['missed']})")

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
    ids_set = set(n['id'] for n in nodes_list)
    ghost = [m.id for kn in nodes for m in kn.core.mn.map if m.alive and m.id not in ids_set]
    out(f"ghost map entries (not in the node list): {len(ghost)}, sample: {[hex(g) for g in ghost[:8]]}")
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


@scenario("star-rest", "scenario 6c (4.3): utrecht40 at rest, one node silenced mid-run - "
                         "rest behaviour, freshness and remote liveness (tick stall and death)")
def star_rest(mk, jitter):
    """Design 4.3/4.4 scenario 6c. The uniform cadence at rest:
      - rest behaviour: content drains to ~0, the catalogue carries
        the maintenance (fill measured per 6-h block via star_stats);
      - freshness: the effective age of the map entries must stay far
        from the 24 h eviction while the origins live;
      - remote liveness (the piece the old 4.4 lacked): one node is
        silenced at half-time; its star's tick stops advancing and the
        liveness axis (star_tick_seen, 4.4) must degrade the star
        EVERYWHERE within the liveness threshold - first de-weighted,
        then evicted, at every hop distance, not just at the direct
        neighbours.
    Reports per node: presence/tick/effective age of the silenced
    node's star, at silencing time and every 6 h after."""
    nodes_list = load_nodes_csv(_ARGS.nodes)
    duration_h = _ARGS.duration if getattr(_ARGS, 'duration', None) else 48
    duration_us = duration_h * 3_600_000_000
    radio = Radio()
    radio.wire.open = False
    lat0 = sum(n['lat'] for n in nodes_list) / len(nodes_list)
    lon0 = sum(n['lon'] for n in nodes_list) / len(nodes_list)
    pos = {n['id']: (*project(n['lat'], n['lon'], lat0, lon0), n['h']) for n in nodes_list}
    nodes = []
    for i, nd in enumerate(nodes_list):
        kn = Node(radio, nd['id'])
        kn.core = set_jitter(mk(kn), jitter)
        kn.core.set_geo(*grid_geo(nd['lat'], nd['lon']))
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
    out(f"{len(nodes_list)} nodes from {_ARGS.nodes}, duration {duration_h} h")

    # the silenced node: pick a mid-degree node (not a hub, not a leaf)
    degree = {i: 0 for i in ids}
    for (a, b) in radio.wire.links:
        degree[a] += 1
    victim = sorted(ids, key=lambda i: abs(degree[i] - 19))[0]
    out(f"victim: {victim:#06x} (degree {degree[victim]}) - silenced at "
        f"t = {duration_h//2} h")
    victim_links = {k: v for k, v in radio.wire.links.items() if k[0] == victim}

    def report_state(label):
        out(f"\n--- {label} ---")
        present = 0; ticks = []; ages = []
        for kn in nodes:
            if kn.id == victim: continue
            p, t, a, eff = kn.core.star_state(victim)
            if p:
                present += 1; ticks.append(t); ages.append(eff)
        if present:
            out(f"victim's star present at {present}/{len(nodes)-1} nodes, "
                f"tick min..max {min(ticks)}..{max(ticks)}, "
                f"eff age avg {sum(ages)/len(ages):.0f} ticks "
                f"(max {max(ages)}, eviction at {2*771})")
        else:
            out("victim's star present nowhere (evicted everywhere)")

    # silence the victim at half-time: cut all its links (it goes deaf
    # AND mute - the wire no longer carries anything to or from it)
    def silence():
        for k in list(radio.wire.links):
            if k[0] == victim or k[1] == victim:
                del radio.wire.links[k]
        out(f"[t = {duration_h//2} h] {victim:#06x} silenced (links cut)")
    radio.schedule((duration_h // 2) * 3_600_000_000, silence)

    # snapshots: presence trend of the victim's star after silencing
    def snap(label):
        present = sum(1 for kn in nodes if kn.id != victim
                      and kn.core.star_state(victim)[0])
        out(f"[snapshot {label}] victim star present at {present}/{len(nodes)-1}")
    for frac in (0.51, 0.6, 0.7, 0.8, 0.9):
        ts = int(frac * duration_us)
        label = f"t = {frac*duration_h:.1f} h"
        radio.schedule(ts, (lambda l=label: (lambda: snap(l)))())

    report_state("state at start (pre-run)")
    radio.run(until_us=duration_us)
    report_state(f"state at end (t = {duration_h} h)")
    star_report(radio)

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

@scenario("star-islands", "isolation test: two 25-node islands, cross-island delivery "
                            "- where exactly do out-of-graph messages strand?")
def star_islands(mk, jitter):
    """Targeted small-scale test for the OUT-OF-GRAPH delivery problem:
    two dense islands of 25 nodes each, connected by a handful of
    border links. All traffic crosses the border (source in island A,
    destination in island B), so every message depends on:
      - the source finding a bridge toward the destination's geo
      - the border hop finding a route onward
      - the receiving island routing internally
    Per message we log the full hop chain and the failure point, so a
    failure reads as: source-refused / no-bridge / stranded-at-hop-N /
    delivered. The islands are 4x2 tiles of a mini-grid: dense
    internally, ~10 km apart, 3-6 border pairs."""
    import math as _math
    N_PER = 25
    duration_h = _ARGS.duration if getattr(_ARGS, 'duration', None) else 8
    duration_us = duration_h * 3_600_000_000
    warmup_h = _ARGS.warmup
    warmup_us = warmup_h * 3_600_000_000
    interval_us = _ARGS.interval * 1_000_000
    radio = Radio()
    radio.wire.open = False
    # island A: grid around (0,0); island B: same grid around (+12 km, 0)
    nodes = []
    ids = []
    def mk_positions(cx_m, cy_m, base_id):
        out = []
        for i in range(N_PER):
            gx = (i % 5) * 900            # 900 m grid spacing
            gy = (i // 5) * 900
            out.append((base_id + i, cx_m + gx, cy_m + gy, 20.0))
        return out
    A = mk_positions(0, 0, 0x0100)
    B = mk_positions(15000, 0, 0x0200)    # 15 km east, 20 m masts:
    # 150 border pairs only - thin enough to force real bridging
    ids_a = [p[0] for p in A]; ids_b = [p[0] for p in B]
    ids = ids_a + ids_b
    lat0, lon0 = 52.09, 5.07
    def geo_of(x_m, y_m):
        return grid_geo(lat0 + y_m/111000, lon0 + x_m/(111000*0.615))
    for (nid, x, y, h) in A + B:
        kn = Node(radio, nid)
        kn.core = set_jitter(mk(kn), jitter)
        kn.core.set_geo(*geo_of(x, y))
        nodes.append(kn)
        radio.schedule((nid & 0xFF) * 250_000, kn.core.start)
    # links: all pairs within an island via linkbudget on local metres;
    # border pairs: the east edge of A to the west edge of B
    posm = {p[0]: (p[1], p[2], p[3]) for p in A + B}
    for a in ids:
        for b in ids:
            if a == b: continue
            same = ((a in ids_a) == (b in ids_b))
            pa = posm[a]; pb = posm[b]
            raw = linkbudget(pa, pb)
            if raw is None: continue
            fade = ((a * 31 + b * 17) % 11) - 5
            r = raw + fade
            if r >= RX_MIN_DBM:
                radio.wire.links[(a, b)] = max(-128, min(120, round(r)))
    deg = {i: 0 for i in ids}
    for (a, b) in radio.wire.links: deg[a] += 1
    border = [(a,b) for (a,b) in radio.wire.links if a in ids_a and b in ids_b]
    out(f"two islands x{N_PER}: {len(radio.wire.links)} directed links, "
        f"degree avg {sum(deg.values())/len(ids):.1f}, border pairs: {len(border)}")

    # cross-island traffic: source in A, destination in B (always out of
    # the local graph at the source, geo from the client)
    rng = random.Random(2026)
    node_geo = {nid: geo_of(posm[nid][0], posm[nid][1]) for nid in ids}
    payload = b'x' * 20
    att = [0]; dlv = [0]; refused = [0]
    msg_hops = {}      # msg-id -> [hops...] (for the strand analysis)
    base_rx = CCore.rx
    def rx(self, arg):
        ch, rssi, pkt = arg
        if pkt and (pkt[0] & 0x0F) == 5 and len(pkt) > 12:
            mid = pkt[1] | (pkt[2] << 8)
            nh = pkt[10] | (pkt[11] << 8)
            msg_hops.setdefault(mid, []).append((self.n.id, nh))
        base_rx(self, arg)
    CCore.rx = rx
    def gen():
        if radio.now >= warmup_us:
            src = rng.choice(ids_a)
            dst = rng.choice(ids_b)
            rc = src_node(src).core.send_data(dst, payload, ack3=False,
                                              geo=node_geo[dst])
            if rc == 0: att[0] += 1
            else: refused[0] += 1
        if radio.now + interval_us <= duration_us:
            radio.schedule(interval_us, gen)
    def src_node(nid):
        return next(kn for kn in nodes if kn.id == nid)
    if interval_us:
        radio.schedule(warmup_us, gen)

    radio.run(until_us=duration_us)
    out(f"\n=== Islands test ({duration_h} h) ===")

    # strand analysis: per msg, did it reach its destination?
    # msg_hops[mid] = [(node-id, next-hop), ...] per DATA reception
    # A message DELIVERED: its destination appears as a receiver.
    # delivered set from the rx wrapper: dst field is in the header,
    # so also track dst per mid.
    msg_dst = {}
    base_send = CCore.send_data
    # find the destination per msg id from the pending records at send
    # (simpler: re-derive from the send log we keep below)
    # -- the analysis below uses the rx hop chains + the dst per mid
    deliveries = set()
    for mid, hops in msg_hops.items():
        # every DATA reception recorded (node, next-hop): if the
        # receiving node IS the next hop target of the previous entry
        # chain end, or the node equals the dst, it delivered
        pass
    # track dst per mid via a send-side hook: wrap send_data to note
    # (msg-id, dst) - the pending record is readable right after the call
    def send_wrap(self, dst, data, ack3=True, geo=None):
        rc = base_send(self, dst, data, ack3=ack3, geo=geo)
        if rc == 0:
            for p in self.mn.pend:
                if p.active and p.dst == dst:
                    msg_dst[p.msg_id] = dst
                    break
        return rc
    CCore.send_data = send_wrap
    # note: send_wrap installed AFTER the run - re-installed for a
    # second short phase is not needed; instead reconstruct dsts from
    # the send log we recorded: att/refused used the actual pend records
    # at send time; for the analysis we re-run the bookkeeping from the
    # logged chains alone (dst = the last next-hop of a delivered chain).
    delivered = 0
    stranded = {}
    for mid, hops in msg_hops.items():
        # a hop chain: the message delivered when some receiver node's
        # recorded next-hop equals its own id (dst received it) OR the
        # chain's last next-hop was the dst and the dst appears as a
        # receiver
        rcv = {nid for (nid, nh) in hops}
        last_nh = hops[-1][1]
        if last_nh in rcv or (len(hops) > 1 and hops[-1][0] == last_nh):
            delivered += 1
        else:
            # stranded: count by the LAST node that had it
            last_node = hops[-1][0]
            stranded[last_node] = stranded.get(last_node, 0) + 1
    out(f"cross-island: attempts {att[0]}, refused-at-send {refused[0]}")
    out(f"hop chains seen: {len(msg_hops)}, delivered {delivered} "
        f"({100*delivered/max(len(msg_hops),1):.1f}%)")
    out(f"stranded at last holder: " + ", ".join(
        f"{nid:#06x} x{c}" for nid, c in sorted(stranded.items(), key=lambda kv: -kv[1])[:10]))
    # which nodes are the stranding points? (their island + border
    # position tells the story)
    star_report(radio)

@scenario("star-recover", "recovery test: pool pressure, then a mass death frees the "
                          "pool - do the tabooed stars come back in?")
def star_recover(mk, jitter):
    """The free-slot taboo policy (4.3), the recovery measurement: two
    25-node islands with MC_MAP_NODES small enough for permanent pool
    pressure (the --so build decides), all B-island stars tabooed at
    island A. At half time, HALF of island B dies (links cut): the
    liveness axis evicts them network-wide within ~6 h... for this
    test the probe is short, so we die-and-shrink the pool faster:
    the death frees pool slots at A (the dead stars' entries leave
    the pool within the liveness threshold). Once >= 4 slots free,
    the taboo flushes and A re-learns the SURVIVING B stars. The
    probes measure: A's knowledge of the surviving B nodes before,
    right after the death (pool still draining), and after recovery."""
    N_PER = 25
    duration_h = _ARGS.duration if getattr(_ARGS, 'duration', None) else 10
    duration_us = duration_h * 3_600_000_000
    radio = Radio()
    radio.wire.open = False
    def mk_positions(cx_m, cy_m, base_id):
        out = []
        for i in range(N_PER):
            out.append((base_id + i, cx_m + (i % 5) * 900, cy_m + (i // 5) * 900, 20.0))
        return out
    A = mk_positions(0, 0, 0x0100)
    B = mk_positions(15000, 0, 0x0200)
    ids_a = [p[0] for p in A]; ids_b = [p[0] for p in B]
    ids = ids_a + ids_b
    lat0, lon0 = 52.09, 5.07
    def geo_of(x_m, y_m):
        return grid_geo(lat0 + y_m/111000, lon0 + x_m/(111000*0.615))
    nodes = []
    posm = {p[0]: (p[1], p[2], p[3]) for p in A + B}
    for (nid, x, y, h) in A + B:
        kn = Node(radio, nid)
        kn.core = set_jitter(mk(kn), jitter)
        kn.core.set_geo(*geo_of(x, y))
        nodes.append(kn)
        radio.schedule((nid & 0xFF) * 250_000, kn.core.start)
    for a in ids:
        for b in ids:
            if a == b: continue
            raw = linkbudget(posm[a], posm[b])
            if raw is None: continue
            fade = ((a * 31 + b * 17) % 11) - 5
            r = raw + fade
            if r >= RX_MIN_DBM:
                radio.wire.links[(a, b)] = max(-128, min(120, round(r)))
    out(f"two islands x{N_PER}, {duration_h} h run")

    survivors_b = ids_b[:12]          # the first 12 of B survive
    doomed = [i for i in ids_b if i not in survivors_b]
    def probe_fn(label):
        out(f"[probe {label}] A-nodes with a surviving B star in the map: "
            f"{sum(1 for kn in nodes if kn.id in ids_a and any(m.alive and m.id in set(survivors_b) for m in kn.core.mn.map))}/{len(ids_a)}")
    # schedule the probes via the helper
    for (t, lbl) in [(3*3_600_000_000, "t=3h (pressure)"),
                    (5*3_600_000_000, "t=5h (after death)"),
                    (8*3_600_000_000, "t=8h (recovery)"),
                    (duration_us - 60_000_000, "end")]:
        radio.schedule(t, (lambda l=lbl: (lambda: probe_fn(l)))())
    def kill():
        for k in list(radio.wire.links):
            if k[0] in doomed or k[1] in doomed:
                del radio.wire.links[k]
        out(f"[death] t={radio.now/3.6e9:.1f} h: {len(doomed)} of B's nodes silenced")
    radio.schedule(4 * 3_600_000_000, kill)

    radio.run(until_us=duration_us)
    out(f"\n=== Recovery test ({duration_h} h) ===")
    star_report(radio)

@scenario("star-scale", "scale test: 480 nodes (4x3 tiles of utrecht40) - rest "
                        "convergence and long-distance delivery")
def star_scale(mk, jitter):
    """Scale scenario (post-6): the utrecht40 set tiled 4x3 with ~3 km
    gaps - 480 nodes over ~180x250 km, every tile internally dense,
    tens of links across every tile border. Two phases in one run:
      - convergence at rest: the probes sample how many of the 479
        OTHER nodes an average node has in its map, and the link
        coverage - the whole-network picture building up (target:
        full-graph coverage as measured on 40 nodes, in the 24 h
        order, not weeks);
      - delivery: after the warmup, a unicast every --interval s
        between random node PAIRS ACROSS THE WHOLE WORLD (multi-hop,
        2-15 hops expected) - the real verdict at scale.
    Reports per phase: map coverage %, link count, delivery ratio,
    latency, airtime, duty, hard bound, missed requests."""
    nodes_list = load_nodes_csv(_ARGS.nodes)
    duration_h = _ARGS.duration if getattr(_ARGS, 'duration', None) else 24
    duration_us = duration_h * 3_600_000_000
    warmup_h = _ARGS.warmup
    warmup_us = warmup_h * 3_600_000_000
    interval_us = _ARGS.interval * 1_000_000
    radio = Radio()
    radio.wire.open = False
    lat0 = sum(n['lat'] for n in nodes_list) / len(nodes_list)
    lon0 = sum(n['lon'] for n in nodes_list) / len(nodes_list)
    pos = {n['id']: (*project(n['lat'], n['lon'], lat0, lon0), n['h']) for n in nodes_list}
    nodes = []
    for i, nd in enumerate(nodes_list):
        kn = Node(radio, nd['id'])
        kn.core = set_jitter(mk(kn), jitter)
        kn.core.set_geo(*grid_geo(nd['lat'], nd['lon']))
        nodes.append(kn)
        radio.schedule(i * 250_000, kn.core.start)
    ids = [n['id'] for n in nodes_list]
    idset = set(ids)
    print(f"building {len(ids)}x{len(ids)} link table...")
    for a in ids:
        for b in ids:
            if a == b: continue
            raw = linkbudget(pos[a], pos[b])
            if raw is None: continue
            fade = ((a * 31 + b * 17) % 11) - 5
            r = raw + fade
            if r >= RX_MIN_DBM:
                radio.wire.links[(a, b)] = max(-128, min(120, round(r)))
    deg = {i: 0 for i in ids}
    for (a, b) in radio.wire.links:
        deg[a] += 1
    out(f"{len(nodes_list)} nodes from {_ARGS.nodes}, duration {duration_h} h "
        f"(warmup {warmup_h} h)")
    out(f"topology: {len(radio.wire.links)} directed links, avg degree "
        f"{sum(deg.values())/len(ids):.1f}, min {min(deg.values())}, max {max(deg.values())}")

    # convergence probes: sampled (full 480x480 scans are too slow)
    rng = random.Random(2026)
    probe_sample = rng.sample(nodes, 24)     # 24 observer nodes
    def probe():
        known = []; links = []
        for kn in probe_sample:
            known.append(sum(1 for m in kn.core.mn.map if m.alive))
            links.append(sum(1 for l in kn.core.mn.lnk if l.alive))
        out(f"[probe t={radio.now/3.6e9:.2f} h] map avg "
            f"{sum(known)/len(known):.1f}/{len(ids)} ({100*sum(known)/len(known)/len(ids):.1f}%), "
            f"links avg {sum(links)/len(links):.1f}")
        if radio.now + 60*60_000_000 < duration_us:
            radio.schedule(60 * 60_000_000, probe)
    radio.schedule(warmup_us, probe)
    for fh in (2, 4, 8, 12, 18):
        if warmup_h < fh < duration_h:
            radio.schedule(fh * 3_600_000_000, probe)

    # delivery after warmup: random pairs across the whole world
    node_geo = {n['id']: grid_geo(n['lat'], n['lon']) for n in nodes_list}
    payload = b'x' * 20
    # delivery split: IN-GRAPH (destination in the source's map) vs
    # OUT-OF-GRAPH (the bridge-node rule, 10.2: destination geo from
    # the client, route toward the nearest known node) - the geo-based
    # delivery question of the scale run
    att_in = [0]; dlv_in = [0]; att_out = [0]; dlv_out = [0]; refused = [0]
    base_rx = CCore.rx
    def rx(self, arg):
        ch, rssi, pkt = arg
        if pkt and (pkt[0] & 0x0F) == 5 and len(pkt) > 10:
            dst = pkt[5] | (pkt[6] << 8)
            if dst == self.n.id:
                src = pkt[3] | (pkt[4] << 8)
                mid = pkt[1] | (pkt[2] << 8)
                key = (mid, src)
                if not hasattr(rx, '_seen'): rx._seen = set()
                if key not in rx._seen:
                    rx._seen.add(key)
                    if key in getattr(rx, '_bymode', {}):
                        if rx._bymode[key] == 'in': dlv_in[0] += 1
                        else: dlv_out[0] += 1
                    else:
                        dlv_in[0] += 1   # fallback: counted as in-graph
        base_rx(self, arg)
    CCore.rx = rx
    def gen():
        if radio.now >= warmup_us:
            kn = rng.choice(nodes)
            dst = rng.choice(ids)
            if dst != kn.id:
                in_graph = any(m.alive and m.id == dst for m in kn.core.mn.map)
                rc = kn.core.send_data(dst, payload, ack3=False, geo=node_geo[dst])
                if rc == 0:
                    if not hasattr(rx, '_bymode'): rx._bymode = {}
                    # record the msg id per mode (dedup key = (mid, src))
                    for p in kn.core.mn.pend:
                        if p.active and p.dst == dst:
                            rx._bymode[(p.msg_id, kn.id)] = 'in' if in_graph else 'out'
                            break
                    if in_graph: att_in[0] += 1
                    else: att_out[0] += 1
                else:
                    refused[0] += 1
        if radio.now + interval_us <= duration_us:
            radio.schedule(interval_us, gen)
    if interval_us:
        radio.schedule(warmup_us, gen)

    radio.run(until_us=duration_us)
    out(f"\n=== Scale scenario ({len(nodes_list)} nodes, {duration_h} h) ===")
    ai, di, ao, do_ = att_in[0], dlv_in[0], att_out[0], dlv_out[0]
    if ai: out(f"unicast IN-GRAPH: attempts {ai}, delivered {di} ({100*di/ai:.1f}%)")
    if ao: out(f"unicast OUT-OF-GRAPH (geo/bridge): attempts {ao}, delivered {do_} "
               f"({100*do_/ao:.1f}%)")
    # map pool occupancy: MC_MAP_NODES is 256, the world is 480 - how full,
    # and how many nodes run AT the cap (eviction pressure)
    pools = sorted(kn.core.map_size() for kn in nodes)
    at_cap = sum(1 for p in pools if p >= 256)
    out(f"map pool: avg {sum(pools)/len(pools):.1f}/256, p50 {pools[len(pools)//2]}, "
        f"max {pools[-1]}, at the 256 cap: {at_cap}/{len(nodes)}")
    out(f"refused (destination not in the graph yet / no slot): {refused[0]} "
        f"({100*refused[0]/max(ai+ao+refused[0],1):.1f}% of offers)")
    # LOCAL occupancy: the channel is shared per receiver (a packet
    # occupies only where it is audible) - the global counter reads as
    # if the whole world shared one cell. Report per-node local busy
    # share on the home channel, the honest capacity number.
    occs = []
    for kn in nodes:
        occs.append(radio.wire.occupancy(kn.id, kn.core.mn.home_ch, duration_us))
    occs.sort()
    out(f"local occupancy (home channel, per node): avg {100*sum(occs)/len(occs):.2f}%, "
        f"p50 {100*occs[len(occs)//2]:.2f}%, max {100*occs[-1]:.2f}%")
    usage = {(kn.id, c): radio.duty.usage(kn.id, c, radio.now)
             for kn in nodes for c in range(N_CH)}
    top = sorted(usage.values(), reverse=True)[:5]
    out(f"duty (sliding hour, limit 360 s): max {top[0]/1e6:.1f} s, "
        f"above limit: {sum(1 for u in usage.values() if u > 360_000_000)}")
    ghosts = sum(1 for kn in nodes for m in kn.core.mn.map
                 if m.alive and m.id not in idset)
    out(f"ghost entries: {ghosts}")
    star_report(radio)

@scenario("star-dense", "scenario 6f (4.3): dense-cluster stress - 64 fully meshed "
                          "nodes on one home channel, the shared-channel bound")
def star_dense(mk, jitter):
    """Design 4.3 scenario 6f. The per-node hard bound (one chunk per
    interval) is NOT a per-channel bound: a dense single-channel
    cluster measures the AGGREGATE channel occupancy. 64 nodes, all
    mutually in range (a fully meshed tower cluster), all at home on
    the same channel (ids chosen so id mod 3 is equal), so:
      - every round of every node lands on the same channel;
      - every receiver processes 63 neighbours' stars + catalogues;
      - the stars are thick (each node owns ~half of its 63 links).
    Measures: aggregate occupancy per channel against the 10% duty
    budget, convergence of a mutation wave at this density, the
    map/link/miss bookkeeping at 64x the per-node information, ghost
    integrity, hard bound. If the budget breaks, open point 1
    (interval scaling with density) gets its measurement here."""
    N = _ARGS.nodes_n if hasattr(_ARGS, 'nodes_n') and _ARGS.nodes_n else 64
    duration_h = _ARGS.duration if getattr(_ARGS, 'duration', None) else 12
    duration_us = duration_h * 3_600_000_000
    radio = Radio()
    radio.wire.open = False
    # one cluster: everyone within ~2 km of a centre, 30 m masts - all
    # pairs audible; ids all with the same home channel (mod 3 == 0)
    lat0, lon0 = 52.09, 5.07
    nodes = []
    ids = []
    for i in range(N):
        nid = 0x0300 + 3*i          # id mod 3 == 0 for all: one home channel
        ids.append(nid)
    pos = {}
    import math as _math
    for i, nid in enumerate(ids):
        ang = 2 * _math.pi * i / N
        r = 400 + (i % 7) * 100     # 400-1000 m ring
        lat = lat0 + r * _math.cos(ang) / 111000
        lon = lon0 + r * _math.sin(ang) / (111000 * _math.cos(_math.radians(lat0)))
        pos[nid] = (r * _math.cos(ang), r * _math.sin(ang), 30.0)
    for i, nid in enumerate(ids):
        kn = Node(radio, nid)
        kn.core = set_jitter(mk(kn), jitter)
        gl, go = grid_geo(lat0 + pos[nid][1] / 111000,
                          lon0 + pos[nid][0] / (111000 * _math.cos(_math.radians(lat0))))
        kn.core.set_geo(gl, go)
        nodes.append(kn)
        radio.schedule(i * 250_000, kn.core.start)
    for a in ids:
        for b in ids:
            if a == b: continue
            raw = linkbudget(pos[a], pos[b])
            if raw is None: continue
            fade = ((a * 31 + b * 17) % 11) - 5
            r = raw + fade
            if r >= RX_MIN_DBM:
                radio.wire.links[(a, b)] = max(-128, min(120, round(r)))
    deg = {i: 0 for i in ids}
    for (a, b) in radio.wire.links:
        deg[a] += 1
    out(f"{N} nodes, fully meshed cluster, all home on channel {ids[0] % 3}, "
        f"duration {duration_h} h")
    out(f"topology: {len(radio.wire.links)} directed links, "
        f"degree min {min(deg.values())}, max {max(deg.values())}")

    # a mutation wave at half time: a quarter of the nodes moves
    rng = random.Random(2026)
    mut_ids = rng.sample(ids, max(1, N // 4))
    post_geo = {}
    def wave():
        for nid in mut_ids:
            for kn in nodes:
                if kn.id == nid:
                    kn.core.set_geo((kn.core.mn.geo_lat + 37) % 65536,
                                    (kn.core.mn.geo_lon + 41) % 65536)
        for nid in mut_ids:
            for kn in nodes:
                if kn.id == nid:
                    post_geo[nid] = (kn.core.mn.geo_lat, kn.core.mn.geo_lon)
        out(f"[wave] t={radio.now/3.6e9:.2f} h: {len(mut_ids)} nodes moved")
        radio.schedule(30 * 60_000_000, probe)
    def probe():
        conv = 0; total = 0
        for nid in mut_ids:
            gl, go = post_geo[nid]
            for kn in nodes:
                if kn.id == nid: continue
                m = next((m for m in kn.core.mn.map if m.alive and m.id == nid), None)
                if m is None: continue
                total += 1
                if m.geo_lat == gl and m.geo_lon == go:
                    conv += 1
        out(f"[probe t={radio.now/3.6e9:.2f} h] post-wave geo visible: "
            f"{conv}/{total} ({100*conv/max(total,1):.1f}%)")
        if radio.now + 30*60_000_000 < duration_us:
            radio.schedule(30 * 60_000_000, probe)
    radio.schedule(duration_us // 2, wave)

    radio.run(until_us=duration_us)
    out(f"\n=== Dense-cluster scenario 6f ({N} nodes, {duration_h} h) ===")
    st = radio.star_stats.summary()
    out(f"rounds: {st['rounds']}, per node/h: {st['rounds']/N/duration_h:.1f}")
    for c in range(N_CH):
        frac = radio.airtime[c] / duration_us * 100 if duration_us else 0
        out(f"channel {c}: airtime {radio.airtime[c]/1e6:.1f} s = {frac:.2f}% of the run")
    usage = {(kn.id, c): radio.duty.usage(kn.id, c, radio.now)
             for kn in nodes for c in range(N_CH)}
    top = sorted(usage.values(), reverse=True)[:5]
    out(f"duty (sliding hour, limit 360 s): max {top[0]/1e6:.1f} s, "
        f"above limit: {sum(1 for u in usage.values() if u > 360_000_000)}")
    # map and link load + ghost integrity
    maps = [kn.core.map_size() for kn in nodes]
    links = [kn.core.link_count() for kn in nodes]
    ghosts = sum(1 for kn in nodes for m in kn.core.mn.map
                 if m.alive and m.id not in set(ids))
    out(f"map: avg {sum(maps)/len(maps):.1f} (min {min(maps)}, max {max(maps)}), "
        f"links avg {sum(links)/len(links):.1f}, ghost entries: {ghosts}")
    star_report(radio)

@scenario("star-quiet", "scenario 6e (4.3): quiet-hour acceleration - a backlog "
                          "drains at up to 3x speed in quiet hours, day/night profile")
def star_quiet(mk, jitter):
    """Design 4.3 scenario 6e. Day/night traffic profile on utrecht40:
      - day (traffic on): a unicast every --interval s keeps the
        channels busy - the acceleration gate must stay closed;
      - night (quiet): no unicast; just before the night starts, a
        wave of geo mutations (--rate x a burst factor) creates a
        real content backlog;
      - the measurement: how much of the backlog drains during the
        quiet hours, at which round rate (the rounds per hour per
        node - the acceleration shows as up to 3 rounds/h instead of
        4/h... at 15-min interval 4/h, at the 5-min floor 12/h), and
        the airtime: quiet-hour convergence must stay bounded (the
        duty share criterion).
    The star_stats rounds counter gives the round rate; the
    convergence measure is the time until all nodes see the mutated
    stars' new form (tick advanced, hash changed, map fresh)."""
    nodes_list = load_nodes_csv(_ARGS.nodes)
    duration_h = _ARGS.duration if getattr(_ARGS, 'duration', None) else 12
    duration_us = duration_h * 3_600_000_000
    interval_us = _ARGS.interval * 1_000_000
    radio = Radio()
    radio.wire.open = False
    lat0 = sum(n['lat'] for n in nodes_list) / len(nodes_list)
    lon0 = sum(n['lon'] for n in nodes_list) / len(nodes_list)
    pos = {n['id']: (*project(n['lat'], n['lon'], lat0, lon0), n['h']) for n in nodes_list}
    nodes = []
    for i, nd in enumerate(nodes_list):
        kn = Node(radio, nd['id'])
        kn.core = set_jitter(mk(kn), jitter)
        kn.core.set_geo(*grid_geo(nd['lat'], nd['lon']))
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
    # day = first half (unicast on), night = second half (quiet)
    night_start_us = duration_us // 2
    out(f"{len(nodes_list)} nodes, {duration_h} h: day 0-{duration_h//2} h "
        f"(unicast every {_ARGS.interval} s), night {duration_h//2}-{duration_h} h (quiet)")

    # day traffic
    node_geo = {n['id']: grid_geo(n['lat'], n['lon']) for n in nodes_list}
    rng = random.Random(2026)
    payload = b'x' * 20
    attempted = [0]
    def gen():
        if radio.now < night_start_us:
            kn = rng.choice(nodes)
            dst = rng.choice(ids)
            if dst != kn.id and kn.core.send_data(dst, payload, ack3=False,
                                                  geo=node_geo[dst]) == 0:
                attempted[0] += 1
            radio.schedule(interval_us, gen)
    radio.schedule(interval_us, gen)

    # the backlog wave: just before the night, move a third of the nodes
    n_mut = max(1, len(nodes) // 3)
    mut_ids = rng.sample([kn.id for kn in nodes], n_mut)
    t_wave = night_start_us - 10 * 60_000_000   # 10 min before the night
    def wave():
        for nid in mut_ids:
            for kn in nodes:
                if kn.id == nid:
                    kn.core.set_geo((kn.core.mn.geo_lat + 37) % 65536,
                                    (kn.core.mn.geo_lon + 41) % 65536)
        out(f"[wave] t={radio.now/3.6e9:.2f} h: {n_mut} nodes moved "
            f"(content backlog created)")
    radio.schedule(t_wave, wave)

    # convergence probe: a mutated star is CONVERGED at an observer
    # when the observer's map entry carries the post-wave geo (the
    # mutation IS a geo move - the sharpest possible criterion)
    post_geo = {}     # nid -> (lat, lon) after the wave
    def probe():
        conv = 0; total = 0
        for nid in mut_ids:
            gl, go = post_geo[nid]
            for kn in nodes:
                if kn.id == nid: continue
                m = next((m for m in kn.core.mn.map if m.alive and m.id == nid), None)
                if m is None: continue
                total += 1
                if m.geo_lat == gl and m.geo_lon == go:
                    conv += 1
        out(f"[probe t={radio.now/3.6e9:.2f} h] post-wave geo visible: "
            f"{conv}/{total} ({100*conv/max(total,1):.1f}%)")
        if radio.now + 30*60_000_000 < duration_us:
            radio.schedule(30 * 60_000_000, probe)
    # capture the post-wave geos right after the wave fires
    def wave_probe():
        for nid in mut_ids:
            for kn in nodes:
                if kn.id == nid:
                    post_geo[nid] = (kn.core.mn.geo_lat, kn.core.mn.geo_lon)
        probe()
    radio.schedule(night_start_us, wave_probe)

    radio.run(until_us=duration_us)
    out(f"\n=== Quiet-hour scenario 6e ({duration_h} h) ===")
    out(f"day unicast attempts: {attempted[0]}")
    st = radio.star_stats.summary()
    out(f"rounds total: {st['rounds']}, per node/h: "
        f"{st['rounds']/len(nodes)/duration_h:.1f}")
    usage = {(kn.id, c): radio.duty.usage(kn.id, c, radio.now)
             for kn in nodes for c in range(N_CH)}
    top = sorted(usage.values(), reverse=True)[:5]
    out(f"duty (sliding hour, limit 360 s): max {top[0]/1e6:.1f} s, "
        f"above limit: {sum(1 for u in usage.values() if u > 360_000_000)}")
    star_report(radio)

@scenario("star-churn", "scenario 6d (4.3/4.4): churn - geo mutations, nodes "
                           "joining and leaving, reboot/reincarnation, delivery under churn")
def star_churn(mk, jitter):
    """Design 4.3/4.4 scenario 6d. Churn on utrecht40:
      - geo mutations: --rate per hour, a random node moves (its own
        star's content hash changes - the pure mutation-convergence
        measurement: time until the new form is visible network-wide);
      - join/leave: --churn = percent of nodes per day that leave
        (links cut - the network must detect the death via the liveness
        axis) and later return (links restored - short absence within
        the liveness threshold must resume seamlessly; long absence
        rides the newcomer path);
      - delivery: a unicast every --interval s between random nodes
        throughout, so the real verdict (delivery ratio) is measured
        under churn, not just the bookkeeping.
    Reports: mutation convergence per hop band, missed-requests per
    node per hour, liveness wave (per hop band) for the leavers,
    delivery ratio, airtime, hard bound."""
    nodes_list = load_nodes_csv(_ARGS.nodes)
    duration_h = _ARGS.duration if getattr(_ARGS, 'duration', None) else 24
    duration_us = duration_h * 3_600_000_000
    warmup_h = _ARGS.warmup
    warmup_us = warmup_h * 3_600_000_000
    mut_rate = getattr(_ARGS, 'rate', 0)          # geo mutations per hour
    churn_pct = getattr(_ARGS, 'churn', 0.0)      # percent of nodes per day leave+return
    interval_us = _ARGS.interval * 1_000_000
    radio = Radio()
    radio.wire.open = False
    lat0 = sum(n['lat'] for n in nodes_list) / len(nodes_list)
    lon0 = sum(n['lon'] for n in nodes_list) / len(nodes_list)
    pos = {n['id']: (*project(n['lat'], n['lon'], lat0, lon0), n['h']) for n in nodes_list}
    nodes = []
    for i, nd in enumerate(nodes_list):
        kn = Node(radio, nd['id'])
        kn.core = set_jitter(mk(kn), jitter)
        kn.core.set_geo(*grid_geo(nd['lat'], nd['lon']))
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
    out(f"{len(nodes_list)} nodes, {duration_h} h (warmup {warmup_h} h), "
        f"mutations {mut_rate}/h, churn {churn_pct}%/day")

    # hop bands from the topology (BFS over the wire links, undirected)
    def hop_bands():
        import collections
        adj = collections.defaultdict(set)
        for (a, b) in radio.wire.links:
            adj[a].add(b)
        bands = {}
        for src in ids:
            dist = {src: 0}
            q = collections.deque([src])
            while q:
                u = q.popleft()
                for v in adj[u]:
                    if v not in dist:
                        dist[v] = dist[u] + 1
                        q.append(v)
            bands[src] = dist
        return bands
    bands = hop_bands()

    # geo mutations: a random node shifts position (a real content
    # change of its own star: the hash changes, the tick does not)
    rng = random.Random(2026)
    mut_log = []      # (t_us, node_id)
    n_mut = int(mut_rate * duration_h) if mut_rate else 0
    for i in range(n_mut):
        t = warmup_us + rng.random() * (duration_us - warmup_us) if duration_us > warmup_us else 0
        if t < warmup_us: continue
        kn = rng.choice(nodes)
        mut_log.append((int(t), kn.id))
        def _mut(kn=kn):
            lat, lon = kn.core.mn.geo_lat, kn.core.mn.geo_lon
            kn.core.set_geo((lat + 37) % 65536, (lon + 41) % 65536)
            out(f"[mut] t={(radio.now/3.6e9):.2f} h: node {kn.id:#06x} moved")
        radio.schedule(max(t - radio.now, 0), _mut)

    # join/leave: churn_pct% of the nodes per day leave (links cut)
    # and return after an absence (half short < liveness, half long)
    leave_events = []
    import math
    n_leave = int(math.ceil(len(nodes) * churn_pct / 100.0 * duration_h / 24.0))
    if churn_pct > 0 and n_leave == 0:
        n_leave = 1 if duration_h >= 3 else 0   # a short run still shows one cycle
    for i in range(n_leave):
        t = warmup_us + rng.random() * max(duration_us - warmup_us, 1)
        kn = rng.choice(nodes)
        long_absence = (i % 2 == 0)          # half long (reincarnation-scale), half short
        leave_events.append((int(t), kn, long_absence))
    cut = {}
    for (t, kn, long_abs) in leave_events:
        abs_h = 8 if long_abs else 0.5      # short: within the 6 h liveness threshold
        def _leave(kn=kn, abs_h=abs_h):
            cut[kn.id] = {k: v for k, v in radio.wire.links.items()
                         if k[0] == kn.id or k[1] == kn.id}
            for k in list(cut[kn.id]):
                del radio.wire.links[k]
            out(f"[leave] t={(radio.now/3.6e9):.2f} h: node {kn.id:#06x} "
                f"leaves (absence {abs_h} h)")
        def _return(kn=kn, abs_h=abs_h):
            radio.wire.links.update(cut.get(kn.id, {}))
            out(f"[return] t={(radio.now/3.6e9):.2f} h: node {kn.id:#06x} returns")
        radio.schedule(max(t - radio.now, 0), _leave)
        radio.schedule(max(t + abs_h * 3_600_000_000 - radio.now, 0), _return)

    # delivery throughout: a unicast every --interval s (after warmup)
    node_geo = {n['id']: grid_geo(n['lat'], n['lon']) for n in nodes_list}
    payload = b'x' * 20
    attempted = [0]; delivered = [0]
    def gen():
        if radio.now >= warmup_us:
            kn = rng.choice(nodes)
            dst = rng.choice(ids)
            if dst != kn.id and kn.core.send_data(dst, payload, ack3=False,
                                                  geo=node_geo[dst]) == 0:
                attempted[0] += 1
        radio.schedule(interval_us, gen)
    if interval_us:
        radio.schedule(interval_us, gen)

    # delivery bookkeeping: DATA receptions at the destination,
    # dedup per (msg-id, src) - the traffic-scenario style
    delivered = [0]
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
        base_rx(self, arg)
    CCore.rx = rx

    radio.run(until_us=warmup_us + duration_us)

    out(f"\n=== Churn scenario 6d ({duration_h} h, mut {mut_rate}/h, churn {churn_pct}%/day) ===")
    out(f"mutations: {len(mut_log)}, leave/return events: {len(leave_events)}")
    out(f"unicast attempts {attempted[0]}, delivered {delivered[0]} "
        f"(100.0% = {attempted[0] and 100.0*delivered[0]/attempted[0]:.1f}%)")
    star_report(radio)
    out(f"mutations logged at: " + ", ".join(
        f"t={t/3.6e9:.2f}h {nid:#06x}" for t, nid in mut_log[:12]))
    out(f"leaves at: " + ", ".join(
        f"t={t/3.6e9:.2f}h {kn.id:#06x}{'(long)' if la else '(short)'}"
        for (t, kn, la) in leave_events[:12]))

@scenario("load", "24 h warmup (convergence), then a unicast every N seconds from a random node to a random node")
def load(mk, jitter):
    """Heavy-load routing test (10.2, steps 1-3): after a warmup in
    which the network converges (neighbour tables, map, link store),
    a message goes out every --interval seconds - source and
    destination both random, so most messages are routed over
    multiple hops. Measures delivery ratio, refused sends per reason
    (-1 not in the graph, -2 no pending slot), retries, latency at
    the last hop, airtime, collisions, home-deafness and duty against
    the 10% limit.
    CLI: --nodes --warmup (h, default 24) --duration (h, the load
    phase, default 6) --interval (s, default 10) --payload (bytes)."""
    nodes_list = load_nodes_csv(_ARGS.nodes)
    warmup_us = _ARGS.warmup * 3_600_000_000
    duration_us = _ARGS.duration * 3_600_000_000
    interval_us = _ARGS.interval * 1_000_000
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
        f"warmup {_ARGS.warmup} h, load {_ARGS.duration} h, "
        f"one unicast per {_ARGS.interval} s, payload {payload_b} B")

    # delivery bookkeeping on the raw bytes, before the core's own
    # next-hop filter and dedup: a DATA reception at the destination
    # counts once per (msg-id, source); further receptions of the same
    # pair are last-hop retries.
    attempted = [0]; refused_graph = [0]; refused_slot = [0]
    delivered = [0]; retries_seen = [0]
    sent_at = {}
    first_rx = {}
    base_rx = CCore.rx
    def rx(self, arg):
        ch, rssi, pkt = arg
        if pkt and (pkt[0] & 0x0F) == 5 and len(pkt) > 12:
            dst = pkt[5] | (pkt[6] << 8)
            if dst == self.n.id:
                src = pkt[3] | (pkt[4] << 8)
                mid = pkt[1] | (pkt[2] << 8)
                key = (mid, src)
                if not hasattr(rx, '_seen'): rx._seen = set()
                if not hasattr(rx, '_times'): rx._times = {}
                rx._times[key] = rx._times.get(key, 0) + 1
                if key not in rx._seen:
                    rx._seen.add(key)
                    delivered[0] += 1
                    t0 = sent_at.get(key)
                    if t0 is not None:
                        first_rx[key] = (radio.now - t0) / 1e6
                else:
                    retries_seen[0] += 1
        base_rx(self, arg)
    CCore.rx = rx

    # msg-id bookkeeping: read the pending right after a successful send
    base_send = CCore.send_data
    def send_probe(self, dst, data, ack3=True, geo=None):
        rc = base_send(self, dst, data, ack3=ack3, geo=geo)
        if rc == 0:
            for p in self.mn.pend:
                if p.active and p.dst == dst:
                    sent_at[(p.msg_id, p.src)] = radio.now
                    break
        elif rc == -1:
            refused_graph[0] += 1
            if not hasattr(send_probe, '_ref'): send_probe._ref = {}
            key = (self.n.id, dst)
            send_probe._ref[key] = send_probe._ref.get(key, 0) + 1
        elif rc == -2: refused_slot[0] += 1
        return rc
    CCore.send_data = send_probe

    payload = bytes(range(1, payload_b + 1))[:payload_b] if payload_b else b'x' * 8
    rng = random.Random(2026)
    node_geo = {n['id']: grid_geo(n['lat'], n['lon']) for n in nodes_list}

    def gen():
        # no offer that would fall in the last interval (measured
        # before: a send at the very end gets no airtime and shows up
        # as a false loss)
        if radio.now + interval_us <= warmup_us + duration_us and radio.now >= warmup_us:
            kn = rng.choice(nodes)
            dst = rng.choice(ids)
            if dst != kn.id:
                # assumption (10.2): the end user supplies the correct
                # destination geo with the message - the bridge-node
                # rule can route even outside the own graph
                rc = kn.core.send_data(dst, bytes(payload), ack3=False, geo=node_geo[dst])
                attempted[0] += 1
                if not hasattr(gen, '_att'): gen._att = {}
                gen._att[kn.id] = gen._att.get(kn.id, 0) + 1
                if rc != 0:
                    if not hasattr(gen, '_ref'): gen._ref = {}
                    gen._ref[(kn.id, rc)] = gen._ref.get((kn.id, rc), 0) + 1
        radio.schedule(interval_us, gen)
    radio.schedule(interval_us, gen)
    radio.run(until_us=warmup_us + duration_us)

    out("\n=== Measurements (load phase) ===")
    att, dlv = attempted[0], delivered[0]
    out(f"unicast attempts {att}, refused: dst not in graph {refused_graph[0]}, "
        f"no pending slot {refused_slot[0]}; delivered {dlv} "
        f"({100*dlv/max(att,1):.1f}% of attempts, last-hop retries seen {retries_seen[0]})")
    if refused_graph[0] and hasattr(send_probe, '_ref'):
        ref = send_probe._ref
        pairs = sorted(ref.items(), key=lambda kv: -kv[1])[:10]
        out(f"'dst not in graph' by (source, destination): "
            f"{sum(ref.values())} over {len(ref)} pairs; top {len(pairs)}:")
        for (s, d), c in pairs:
            out(f"  {s} -> {d}: {c}x")
    if hasattr(gen, '_att') and hasattr(CCore.rx, '_times'):
        per_src = {}
        for (mid, s), t in CCore.rx._times.items():
            per_src[s] = per_src.get(s, 0) + 1
        rows = []
        for nid, a in sorted(gen._att.items()):
            d = per_src.get(nid, 0)
            rows.append((d / max(a, 1), nid, a, d, gen._ref.get((nid, -1), 0)))
        rows.sort()
        out("per source: attempts / delivered / refused-not-in-graph (worst delivery first)")
        for r, nid, a, d, rf in rows[:8]:
            out(f"  node {nid}: {a} attempts, {d} delivered ({100*r:.0f}%), refused {rf}")
    if delivered and hasattr(CCore.rx, '_times'):
        times = list(CCore.rx._times.values())
        multi = [t for t in times if t > 1]
        if multi:
            dist = {}
            for t in multi: dist[t] = dist.get(t, 0) + 1
            out(f"arrived more than once at the destination: {len(multi)} of {len(times)} "
                f"delivered messages ({100*len(multi)/len(times):.1f}%), "
                f"receptions per duplicated message: {dist}")
    if first_rx:
        ls = sorted(first_rx.values())
        out(f"latency (last hop delivery): median {ls[len(ls)//2]:.1f} s, "
            f"p95 {ls[max(0,int(0.95*len(ls))-1)]:.1f} s, max {ls[-1]:.1f} s")
    for c in range(N_CH):
        out(f"channel {c}: airtime {radio.airtime[c]/1e6:.3f} s, "
            f"lost receptions {radio.wire.collided[c]}")
    stats = [nbr_counts(kn) for kn in nodes]
    conf = [s2[1] for s2 in stats]
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
    ap.add_argument('--warmup', type=int, default=24, metavar='HOURS',
                    help='convergence warmup before the load phase (load, default 24)')
    ap.add_argument('--nodes-n', type=int, default=64, metavar='COUNT',
                    help="scenario 6f: the dense-cluster size (default 64)")
    ap.add_argument('--churn', type=float, default=0.0, metavar='PCT_DAY',
                    help="scenario 6d: percent of the nodes per day that leave and later return")
    ap.add_argument('--interval', type=int, default=10, metavar='SECONDS',
                    help='seconds between unicasts (load, default 10)')
    ap.add_argument('--trace', action='store_true',
                    help='message tracing: every node logs its DATA sends/receives '
                         'and ack1/ack2 with a [msg] prefix (forensics runs; pair '
                         'with a low --interval to keep the log readable)')
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
    if args.trace:
        sim1.TRACE_MSGS = True

    lib = load_lib(args.so)
    mk = lambda nd: CCore(nd, lib)

    fn, desc = SCENARIOS[args.scenario]
    out(f"=== Scenario '{args.scenario}': {desc} (C core, SF7, 62.5 kHz) ===")
    fn(mk, args.jitter)

if __name__ == '__main__':
    main()
