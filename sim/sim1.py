#!/usr/bin/env python3
"""
sim1.py — Meshimeshi simulator, scenario 1 (link test: 2 nodes).

Architecture (design 14.1): event-driven discrete simulation; the protocol
core is C and is called via ctypes (default build/libmimeshi.so, built with
  gcc -O2 -fPIC -shared core/core.c -o build/libmimeshi.so).
The C core is the only protocol implementation (design 13b): this file is
exclusively platform layer — clock, radio, RNG — and contains no protocol
logic. The simpy-based simulator (meshsim.py) reuses Node/Wire/CCCore here.

Radio model (switch rule, design 2):
- Every node has ONE radio. Switch requests are serialized per node
  (FIFO); a transmission only starts when the radio is free.
- An exchange = switch (S) + transmission (airtime) + optionally the
  ack1 window (data on a foreign channel), then automatically back to
  the home channel. The transmission belonging to a switch follows
  directly after the switch.
- A node only hears a packet on its current channel, and not while it
  is switching or transmitting itself.

Measurements: airtime per channel, home-deafness per node, delivery outcome.
"""
import argparse, ctypes, heapq, math, os, random, sys
from collections import deque

T_HELLO = 62_000          # us, hello 3 B (6.3)
T_HACK  = 70_000          # hello-ack 5 B (6.2)
T_ACK   = 78_000          # ack1/ack2 7 B (6.2)
T_DATA  = lambda l: 173_000 if l <= 40 else 459_000 if l <= 150 else 869_000
S_SWITCH = 5_000          # channel switch incl. settling (design 14.1)
ACK1_MARGIN = 80_000      # margin on the ack1 wait time, equal to core.c
N_CH = 3
HOME = lambda nid: nid % N_CH
RSSI = -60                # link quality default; settable per pair (Wire.links)
CAPTURE_DB = 6            # margin at which the strongest wins (design 2/14.1)

VERBOSE = True             # False: suppress radio logs (tx/hears-nothing)

def out(*a):
    print(*a, flush=True)

def radio_log(*a):
    if VERBOSE:
        print(*a, flush=True)

class Wire:
    """Channel model shared by both engines: ongoing transmissions per
    channel, link RSSI per (sender, receiver), collision and capture
    (design 2 and 14.1: two overlapping transmissions on the same channel
    collide, unless the strongest wins with >= CAPTURE_DB margin)."""
    def __init__(self):
        self.chan = [[] for _ in range(N_CH)]   # (start, end, sender-id)
        self.links = {}                          # (sender-id, receiver-id) -> dBm
        self.collided = [0]*N_CH               # lost receptions per channel
        self.open = True    # open world: unknown pairs are received with the
                           # default RSSI (small scenarios without a range model);
                           # False = closed world: only pairs in links exist
                           # (link budget from positions, local-net)
    def busy(self, now, ch):
        """CAD, channel activity detection (design 2): is there an
        ongoing transmission on the channel at time now? Overlap of the
        own transmission is impossible (the node serializes itself), so
        this says something about the OTHERS."""
        return any(s < now < e for s, e, _ in self.chan[ch])
    def link(self, tx_id, rx_id):
        if self.open:
            return self.links.get((tx_id, rx_id), RSSI)
        return self.links.get((tx_id, rx_id))   # None = no link
    def begin(self, now, ch, tx_id, airtime):
        self.chan[ch] = [w for w in self.chan[ch] if w[1] > now]
        self.chan[ch].append((now, now + airtime, tx_id))
        # local occupancy (scale runs): the channel is a SHARED MEDIUM
        # PER RECEIVER - a transmission occupies only where it is
        # audible. Book the airtime on the AUDIBLE receivers' local
        # channel, not globally (the global airtime[c] counter is the
        # sum over the whole world and reads as if one packet occupies
        # the channel everywhere - which is only true at 40-node scale).
        if not hasattr(self, 'occ'):
            self.occ = {}          # (rx-id, ch) -> busy us
        for rx in self.audible(tx_id, ch):
            self.occ[(rx, ch)] = self.occ.get((rx, ch), 0) + airtime

    def audible(self, tx_id, ch):
        """Receivers that can hear tx_id on any channel (their home or
        an exchange): iterates the link table - at 480 nodes this is
        the pair list, ~degree entries, cheap enough per packet."""
        if not hasattr(self, '_adj'):
            self._adj = {}
            for (a, b) in self.links:
                self._adj.setdefault(a, []).append(b)
        return self._adj.get(tx_id, [])

    def occupancy(self, rx_id, ch, span_us):
        """Fraction [0..1] of span_us that rx_id's local channel ch
        was busy with audible transmissions."""
        return self.occ.get((rx_id, ch), 0) / max(span_us, 1)
    def check(self, now, rx_id, tx_id, ch, start, end):
        """Disturbance of the reception [start, end) on channel ch.
        None = clean; otherwise (reason, disturbing sender).
        Cleanup may only happen AFTER the window under review: at the
        delivery check (at tx-end), transmissions that overlap this
        window must still be in there, even if they themselves are
        already past."""
        self.chan[ch] = [w for w in self.chan[ch] if w[1] > start]
        s = self.link(tx_id, rx_id)
        for s0, e0, t0 in self.chan[ch]:
            if t0 == tx_id or e0 <= start or s0 >= end:
                continue
            s1 = self.link(t0, rx_id)
            if s1 is None:
                continue                     # disturbance out of range does not interfere
            if s1 >= s + CAPTURE_DB:
                return ("capture by", t0)
            if s1 > s - CAPTURE_DB:
                return ("collision with", t0)
        return None

class DutyLedger:
    """Transmit time per (node, channel) in a sliding window — the 10%
    duty per channel (design 2). Records all transmissions of every Radio."""
    def __init__(self, window_us=3_600_000_000):
        self.window = window_us
        self.log = {}                       # (node-id, channel) -> deque[(t, us)]
    def add(self, nid, ch, nu, us):
        d = self.log.setdefault((nid, ch), deque())
        d.append((nu, us))
        while d and d[0][0] <= nu - self.window:
            d.popleft()
    def usage(self, nid, ch, nu):
        return sum(us for t, us in self.log.get((nid, ch), ()) if t > nu - self.window)

# ---------------- node list and link budget (scenario layer) -----------
# Radio model (Proposal; NL calibration: Open, see design 14.1):
#   reception = TX - FSPL(d) - 2 x clutter(h), bounded by the radio horizon.
# clutter(h) = 30 - 14 log10(1+h) dB: low masts lose a lot to the
# environment, high masts look over it. Per direction a fading offset of
# +-5 dB so links are asymmetric, as in practice.
TX_DBM = 27.0          # ERP in the 869.4-869.65 MHz sub-band (500 mW)
RX_MIN_DBM = -110.0    # reception threshold incl. fade margin
FSPL_1KM_DB = 91.2    # free-space loss at 869 MHz over 1 km

def load_nodes_csv(path):
    """Node list with GPS position and height. Format per line:
    id,lat,lon,height_m,name — '#' is a comment."""
    nodes = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            d = [v.strip() for v in line.split(',')]
            nodes.append({'id': int(d[0], 0), 'lat': float(d[1]),
                          'lon': float(d[2]), 'h': float(d[3]),
                          'name': d[4] if len(d) > 4 else ''})
    return nodes

def project(lat, lon, lat0, lon0):
    """Local flat projection in meters around (lat0, lon0): distances
    within a region are small enough for an approximation with cos(lat0)."""
    x = math.radians(lon - lon0) * 6371000.0 * math.cos(math.radians(lat0))
    y = math.radians(lat - lat0) * 6371000.0
    return x, y

def linkbudget(pa, pb):
    """Raw reception strength in dBm from a to b (positions in meters,
    heights in meters), without fading and without threshold.
    None = link physically impossible: radio horizon (refraction counted
    in, d <= 4.12 (sqrt(h1) + sqrt(h2)) km)."""
    d_km = math.hypot(pa[0] - pb[0], pa[1] - pb[1]) / 1000.0
    if d_km <= 1e-3:
        return None
    if d_km > 4.12 * (math.sqrt(pa[2]) + math.sqrt(pb[2])):
        return None
    fspl = FSPL_1KM_DB + 20.0 * math.log10(d_km)
    clutter = (30.0 - 14.0 * math.log10(1.0 + pa[2])) + \
              (30.0 - 14.0 * math.log10(1.0 + pb[2]))
    return TX_DBM - fspl - clutter

class StarStats:
    """Star-round measurement (4.5, scenario 6): per transmitted chunk the
    content star count (p[4]), the catalogue count (p[3]) and the missed
    entries - the three areas of the round. Aggregates per node:
    rounds, content stars, catalogue entries, missed requests, and the
    byte split, plus the hard-bound check (max rounds per node per
    interval of 16 proof ticks)."""
    INTERVAL_US = 16 * 56000000          # 15 min (4.5 starting value)
    def __init__(self):
        self.per_node = {}                # id -> dict of counters
        self.air_total = 0
    def tx(self, now, nid, ch, pkt, airtime):
        d = self.per_node.setdefault(nid, dict(rounds=0, content=0, catalogue=0,
                                                missed=0, content_b=0, cat_b=0,
                                                miss_b=0, times=[]))
        # one round transmits on all three channels back to back (~180 ms
        # apart): count the ROUND once, not per channel. A second cluster
        # within an interval would be a real violation.
        if d["times"] and now - d["times"][-1] < 3_200_000:      # < 3.2 s: same round
            pass                                   # (a full-spread round spans 3
        else:                                       # channels: ~3.9 s air total)
            d["rounds"] += 1
            d["times"].append(now)
        d["content"] += pkt[4]
        d["content_b"] += 0              # filled below from the entry walk
        cat = pkt[3] if (pkt[0] & 0x10) else 0
        d["catalogue"] += cat
        d["cat_b"] += 3 * cat
        # missed area: after header + content + catalogue
        if pkt[0] & 0x20:
            off = 10
            for _ in range(pkt[4]):
                off += 9 + 3 * pkt[off + 8]
            off += 3 * cat
            while off + 3 <= len(pkt):
                nb = pkt[off] | (pkt[off+1] << 8)
                cnt = pkt[off+2]
                d["missed"] += cnt
                d["miss_b"] += 3 + 2 * cnt
                off += 3 + 2 * cnt
        self.air_total += airtime
    def hard_bound_violations(self):
        """Rounds per node per interval: more than 1 in an interval
        violates the one-chunk-per-interval bound (4.5). Returns a
        list of (nid, t1, t2) violation tuples."""
        out = []
        iv = self.INTERVAL_US
        for nid, d in self.per_node.items():
            times = sorted(d["times"])
            for a, b in zip(times, times[1:]):
                if b - a < iv - 2 * 56000000:      # 2 ticks slack for jitter
                    out.append((nid, a, b))
        return out
    def summary(self):
        rounds = sum(d["rounds"] for d in self.per_node.values())
        content = sum(d["content"] for d in self.per_node.values())
        catalogue = sum(d["catalogue"] for d in self.per_node.values())
        missed = sum(d["missed"] for d in self.per_node.values())
        nodes = len(self.per_node)
        return dict(nodes=nodes, rounds=rounds, content=content,
                    catalogue=catalogue, missed=missed,
                    air_total_us=self.air_total)

class Radio:  # virtual radio + clock
    def __init__(self):
        self.now = 0
        self.q = []          # (at, seq, fn, arg)
        self.nodes = {}      # id -> Node
        self.airtime = [0]*N_CH
        self.hello_air = [0]*N_CH   # airtime of hellos (type 1) per channel
        self.ack_air = [0]*N_CH     # airtime of hello-acks (type 2) per channel
        self.proof_air = [0]*N_CH   # airtime of proof lists (type 3) per channel
        self.chunk_air = [0]*N_CH   # airtime of table chunks (type 4) per channel
        self.star_air = [0]*N_CH    # airtime of star rounds (type 10) per channel (4.5)
        self.star_stats = StarStats()  # star-round measurement (4.5, scenario 6)
        self.duty = DutyLedger()
        self.wire = Wire()
        self.seq = 0
    def schedule(self, delay_us, fn, arg=None):
        self.seq += 1
        heapq.heappush(self.q, (self.now + delay_us, self.seq, fn, arg))
    def run(self, until_us=None):
        while self.q:
            at, _, fn, arg = self.q[0]              # peek: do not pop before the until-check
            if until_us and at > until_us: return False
            heapq.heappop(self.q)
            self.now = at
            fn() if arg is None else fn(arg)
            if until_us is None and not self.q: return True
        return True
    def tx(self, node, ch, pkt, airtime):
        radio_log(f"[{self.now/1e6:8.3f}s] node {node.id:#06x} transmits type {pkt[0]:#04x} "
                  f"({len(pkt)} B, {airtime//1000} ms) on channel {ch}")
        self.airtime[ch] += airtime
        ptype = pkt[0] & 0x0F
        if ptype == 1: self.hello_air[ch] += airtime
        elif ptype == 2: self.ack_air[ch] += airtime
        elif ptype == 3: self.proof_air[ch] += airtime
        elif ptype == 4: self.chunk_air[ch] += airtime
        elif ptype == 10:
            self.star_air[ch] += airtime
            self.star_stats.tx(self.now, node.id, ch, pkt, airtime)
        self.duty.add(node.id, ch, self.now, airtime)
        self.wire.begin(self.now, ch, node.id, airtime)
        for r in self.nodes.values():
            if r is node: continue
            r.schedule_recv(airtime, ch, bytes(pkt), node.id)   # audibility is checked by the receiver itself

class Node:
    """Simulator side of a node: radio timing around the protocol core."""
    def __init__(self, radio, nid):
        self.radio, self.id = radio, nid
        self.core = None
        self.ch = HOME(nid)        # current channel
        self.busy_until = 0        # radio busy (switch/transmission/ack window) until
        self.deaf_until = 0        # deaf (switch or own transmission) until
        self.away_total = 0        # us outside the own home channel
        self._away_since = None
        self.hello_rx = 0          # heard hellos (measurement: round sync, open point 3)
        self.hello_missed = 0      # missed hellos on the own channel (away/busy/collision)
        self.collided = 0          # receptions lost to collision or capture
        self._swq = deque()        # queued switch requests
        self._drain_at = None
        self.rng = random.Random(nid)
        radio.nodes[nid] = self

    # ---- channel state and measurement --------------------------------
    def _set_ch(self, ch):
        home = HOME(self.id)
        if ch != self.ch:
            if ch == home:
                if self._away_since is not None:
                    self.away_total += self.radio.now - self._away_since
                    self._away_since = None
            elif self._away_since is None:
                self._away_since = self.radio.now
        self.ch = ch

    # ---- serialized radio ----------------------------------------------
    def _when_free(self, job):
        """Run the switch job as soon as the radio is free (now or after
        the ongoing busy period). Transmissions belong to their own
        switch and follow directly, so only switches need to queue."""
        if not self._swq and self.radio.now >= self.busy_until:
            job()
        else:
            self._swq.append(job)
            self._plan_drain()

    def _plan_drain(self):
        if self._swq and self._drain_at is None:
            self._drain_at = self.busy_until
            self.radio.schedule(max(self.busy_until - self.radio.now, 0), self._drain)

    def _drain(self):
        self._drain_at = None
        if self.radio.now < self.busy_until:   # busy period grew (transmission after switch)
            self._plan_drain(); return
        if self._swq:
            self._swq.popleft()()
        self._plan_drain()

    def switch(self, ch, s_us, done=None):
        def job():
            end = self.radio.now + s_us
            self.busy_until = end
            self.deaf_until = max(self.deaf_until, end)
            def settled():
                self._set_ch(ch)
                if done: done()
                self._plan_drain()
            self.radio.schedule(s_us, settled)
        self._when_free(job)

    def tx(self, ch, pkt, airtime, wait=None):
        """Transmission on the current channel; wait>0 keeps the node on
        the channel for the return. Default: data gets an ack1 window
        (t_ack + margin, design 7.3) on the exchange channel."""
        if wait is None:
            wait = T_ACK + ACK1_MARGIN if (pkt[0] & 0x0F) == 5 else 0
        start = max(self.radio.now, self.busy_until)
        end = start + airtime
        self.busy_until = end + wait
        self.deaf_until = max(self.deaf_until, end)
        home = HOME(self.id)
        if ch != home:
            self.radio.schedule(end + wait - self.radio.now, lambda: self._set_ch(home))
        self.radio.schedule(start - self.radio.now,
                            lambda: self.radio.tx(self, ch, pkt, airtime))
        self._plan_drain()

    def bound_send(self, ch, pkt, airtime, wait=None):
        """Bound transmission: switch, transmit, (ack window), back home."""
        self.switch(ch, S_SWITCH, lambda: self.tx(ch, pkt, airtime, wait))

    # ---- reception -------------------------------------------------------
    def schedule_recv(self, delay, ch, pkt, tx_id):
        def deliver():
            hello = (pkt[0] & 0x0F) == 1
            start = self.radio.now - delay
            rssi = self.radio.wire.link(tx_id, self.id)
            if rssi is None:
                return    # out of range (closed world): no reception, no count
            # the whole packet must have been heard: on the channel, and the
            # own transmission/switch must be past before the start of this packet
            if self.ch == ch and start >= self.deaf_until:
                disturbance = self.radio.wire.check(self.radio.now, self.id, tx_id, ch, start, self.radio.now)
                if disturbance:
                    reason, by = disturbance
                    self.collided += 1
                    self.radio.wire.collided[ch] += 1
                    if hello: self.hello_missed += 1
                    radio_log(f"[{self.radio.now/1e6:8.3f}s] node {self.id:#06x} verliest "
                              f"type {pkt[0]:#04x} on channel {ch} ({reason} {by:#06x})")
                    return
                if hello: self.hello_rx += 1
                self.core.rx((ch, rssi, pkt)); return
            if ch == HOME(self.id) or self.ch == ch:
                if hello: self.hello_missed += 1
                reason = ("busy with own transmission" if start < self.deaf_until
                          else "away from home (exchange)")
                radio_log(f"[{self.radio.now/1e6:8.3f}s] node {self.id:#06x} hoort niets "
                          f"on channel {ch} ({reason})")
        self.radio.schedule(delay, deliver)

# ---------------------------------------------------------------------------
# C core via ctypes — the only protocol implementation (design 13b).
# Struct layout verified against sizeof/offsetof of core.h
# (neighbor 24, pending 256, mapnode 12, link 6, node 18528 B).
# ---------------------------------------------------------------------------
MC_MAX_NEIGHBORS, MC_MAX_PENDING, MC_MAX_DEDUP, MC_MAX_PAYLOAD = 64, 64, 128, 230

class CNeighbor(ctypes.Structure):
    _fields_ = [("id", ctypes.c_uint16), ("quality", ctypes.c_uint8),
                ("rev_quality", ctypes.c_uint8),
                ("ack_try", ctypes.c_uint8), ("last_seen", ctypes.c_uint32),
                ("proven_age_us", ctypes.c_uint64), ("alive", ctypes.c_uint8),
                ("sym", ctypes.c_uint8), ("adv_class", ctypes.c_uint8),
                ("penalty", ctypes.c_uint8)]

class CPending(ctypes.Structure):
    _fields_ = [("active", ctypes.c_uint8), ("tries_left", ctypes.c_uint8),
                ("budget_left", ctypes.c_uint8), ("acked1", ctypes.c_uint8),
                ("cand_cnt", ctypes.c_uint8), ("try_idx", ctypes.c_uint8),
                ("excl_cnt", ctypes.c_uint8),
                ("msg_id", ctypes.c_uint16), ("src", ctypes.c_uint16),
                ("dst", ctypes.c_uint16), ("prev_hop", ctypes.c_uint16),
                ("next_hop", ctypes.c_uint16),
                ("in_graph", ctypes.c_uint8), ("pad0", ctypes.c_uint8),   # alignment: C pads to 4 before cand[3]
                ("cand", ctypes.c_uint16 * 3),
                ("excl", ctypes.c_uint16 * 3),
                ("pkt", ctypes.c_uint8 * (16 + MC_MAX_PAYLOAD)),
                ("pkt_len", ctypes.c_uint16)]

class CDedup(ctypes.Structure):
    _fields_ = [("msg_id", ctypes.c_uint16), ("src", ctypes.c_uint16), ("seq", ctypes.c_uint16)]

MC_MAP_NODES, MC_MAX_LINKS = 256, 1536

class CMapNode(ctypes.Structure):
    _fields_ = [("id", ctypes.c_uint16),
                ("geo_lat", ctypes.c_uint16), ("geo_lon", ctypes.c_uint16),
                ("quality", ctypes.c_uint8),   # local measurement of the node's signal
                ("adv_q", ctypes.c_uint8),      # entry class byte as the origin advertised (4.3)
                ("arrived_tick", ctypes.c_uint16),
                ("age_ticks", ctypes.c_uint8),
                ("star_tick", ctypes.c_uint8),      # origin heartbeat tick (4.3)
                ("alive", ctypes.c_uint8),
                ("star_fwd_tick", ctypes.c_uint16), # queue entry tick (4.3)
                ("star_fwd_hash", ctypes.c_uint8),  # content hash as last carried (4.3)
                ("star_fwd_bits", ctypes.c_uint8),  # carried-on-channel bits (4.3 rotation)
                ("star_miss_ctr", ctypes.c_uint8),  # missed-list damping (4.3)
                ("star_wanted", ctypes.c_uint16),    # tick of the last explicit ask (4.3)
                ("star_tick_seen", ctypes.c_uint16)]  # local tick of the last origin-tick advance (4.4 liveness)

class CLink(ctypes.Structure):
    _fields_ = [("a", ctypes.c_uint16), ("b", ctypes.c_uint16),
                ("q", ctypes.c_uint8), ("alive", ctypes.c_uint8)]

class CNode(ctypes.Structure):
    pass

CB = ctypes.CFUNCTYPE(None, ctypes.POINTER(CNode), ctypes.c_void_p)   # core callback (timer/switch-done)
CAD_FN = ctypes.CFUNCTYPE(ctypes.c_bool, ctypes.POINTER(CNode), ctypes.c_uint8)
TX_FN = ctypes.CFUNCTYPE(None, ctypes.POINTER(CNode), ctypes.c_uint8,
                         ctypes.POINTER(ctypes.c_uint8), ctypes.c_uint16, ctypes.c_uint64)
TIMER_FN = ctypes.CFUNCTYPE(None, ctypes.POINTER(CNode), ctypes.c_uint64,
                            ctypes.c_void_p, ctypes.c_void_p)
SWITCH_FN = ctypes.CFUNCTYPE(None, ctypes.POINTER(CNode), ctypes.c_uint8, ctypes.c_uint64,
                             ctypes.c_void_p, ctypes.c_void_p)
RAND_FN = ctypes.CFUNCTYPE(ctypes.c_uint32, ctypes.POINTER(CNode))

class CStarMiss(ctypes.Structure):
    _fields_ = [("nb", ctypes.c_uint16), ("star", ctypes.c_uint16),
                ("tries", ctypes.c_uint8), ("alive", ctypes.c_uint8)]

CNode._fields_ = [
    ("id", ctypes.c_uint16), ("home_ch", ctypes.c_uint8), ("local_tbl_ctr", ctypes.c_uint16),
    ("nbr", CNeighbor * MC_MAX_NEIGHBORS), ("pend", CPending * MC_MAX_PENDING),
    ("dedup", CDedup * MC_MAX_DEDUP), ("hello_interval_us", ctypes.c_uint64),
    ("hello_jitter_us", ctypes.c_uint64),
    ("ka_idle", ctypes.c_uint8 * 3),   # keepalive counters per channel (4.1)
    ("proof_wait", ctypes.c_uint8 * 3),  # flush countdown per channel (4.2)
    ("geo_lat", ctypes.c_uint16), ("geo_lon", ctypes.c_uint16),  # own geo grid (5)
    ("tick", ctypes.c_uint32),          # local time base in proof ticks (4.4)
    ("dedup_seq", ctypes.c_uint16),
    ("tbl_dirty", ctypes.c_uint8),             # own-star content change pending (4.3)
    ("star_round_sched", ctypes.c_uint8),     # star-round timer running (4.3)
    ("star_backlog", ctypes.c_uint8),         # content work remained (6e acceleration)
    ("star_rot_ctr", ctypes.c_uint8),         # rotation counter (4.3)
    ("ev_taboo", ctypes.c_uint16 * 256),      # eviction taboo (4.3): judged-and-rejected set
    ("ev_taboo_idx", ctypes.c_uint8),         # taboo ring write position
    ("star_miss", CStarMiss * 12),           # missed list (4.3, MC_STAR_MISS_MAX)
    ("star_own_ctr", ctypes.c_uint8),         # home rounds since the last own-star carry (4.3)
    ("star_tick", ctypes.c_uint8),            # own-star heartbeat tick (4.3)
    ("map", CMapNode * MC_MAP_NODES),   # 2-hop map (10.1)
    ("lnk", CLink * MC_MAX_LINKS),      # link store (4.3)
    ("trace", ctypes.c_uint8),                 # message tracing (forensics runs)
    ("tx", TX_FN), ("timer", TIMER_FN), ("switch_ch", SWITCH_FN), ("rand", RAND_FN),
    ("channel_free", CAD_FN), ("user", ctypes.c_void_p),
]

NODES = {}   # address of mimeshi_node -> Python Node

def _node_of(nptr):
    return NODES[ctypes.addressof(nptr.contents)]

@TX_FN
def _tx(nptr, ch, pkt, length, airtime):
    _node_of(nptr).tx(ch, ctypes.string_at(pkt, length), airtime)

@TIMER_FN
def _timer(nptr, delay, cb, ud):
    fire = ctypes.cast(cb, CB)
    nd = _node_of(nptr)
    nd.radio.schedule(delay, lambda: fire(nptr, ud))

@SWITCH_FN
def _switch(nptr, ch, s_us, done, ud):
    nd = _node_of(nptr)
    if done:
        fire = ctypes.cast(done, CB)
        nd.switch(ch, s_us, lambda: fire(nptr, ud))
    else:
        nd.switch(ch, s_us, None)

@RAND_FN
def _rand(nptr):
    return _node_of(nptr).rng.getrandbits(32)

@CAD_FN
def _channel_free(nptr, ch):
    return not _node_of(nptr).radio.wire.busy(_node_of(nptr).radio.now, ch)

def load_lib(path):
    lib = ctypes.CDLL(path)
    lib.mimeshi_init.argtypes = [ctypes.POINTER(CNode), ctypes.c_uint16, TX_FN,
                                 TIMER_FN, SWITCH_FN, RAND_FN, CAD_FN, ctypes.c_void_p]
    lib.mimeshi_set_trace.argtypes = [ctypes.POINTER(CNode), ctypes.c_uint8]
    lib.mimeshi_start.argtypes = [ctypes.POINTER(CNode)]
    lib.mimeshi_rx.argtypes = [ctypes.POINTER(CNode), ctypes.c_uint8, ctypes.c_int8,
                               ctypes.POINTER(ctypes.c_uint8), ctypes.c_uint16]
    lib.mimeshi_send_data.argtypes = [ctypes.POINTER(CNode), ctypes.c_uint16,
                                      ctypes.c_char_p, ctypes.c_uint16, ctypes.c_bool,
                                      ctypes.c_uint16, ctypes.c_uint16]
    lib.mimeshi_send_data.restype = ctypes.c_int
    lib.mimeshi_set_geo.argtypes = [ctypes.POINTER(CNode), ctypes.c_uint16, ctypes.c_uint16]
    lib.mimeshi_star_serialize.argtypes = [ctypes.POINTER(CNode), ctypes.c_uint16,
                                           ctypes.POINTER(ctypes.c_uint8), ctypes.c_uint16,
                                           ctypes.c_void_p]
    lib.mimeshi_star_serialize.restype = ctypes.c_uint16
    lib.mimeshi_star_content_hash.argtypes = [ctypes.POINTER(ctypes.c_uint8), ctypes.c_uint16,
                                              ctypes.c_uint16]
    lib.mimeshi_star_content_hash.restype = ctypes.c_uint8
    return lib

class CCore:
    """Protocol core in C; every node has its own mimeshi_node (no shared
    state). The four callbacks run via the simulator above."""
    def __init__(self, node, lib):
        self.n, self.lib = node, lib
        globals().setdefault('ALL_NODES', []).append(node)
        self.mn = CNode()
        NODES[ctypes.addressof(self.mn)] = node
        lib.mimeshi_init(ctypes.byref(self.mn), node.id, _tx, _timer, _switch, _rand, _channel_free, None)
        if globals().get('TRACE_MSGS'):
            lib.mimeshi_set_trace(ctypes.byref(self.mn), 1)
    def start(self):
        self.lib.mimeshi_start(ctypes.byref(self.mn))
    STAR_PROBE = None   # (rx_id, tx_id): if set, prints type-10 rx (debug hook)
    def rx(self, arg):
        ch, rssi, pkt = arg
        if CCore.STAR_PROBE and (pkt[0] & 0x0F) == 10:
            want_rx, want_tx = CCore.STAR_PROBE
            if want_tx is None or self.n.id == want_rx:
                off = 10
                cent = []
                for _ in range(pkt[4]):
                    cent.append(hex(pkt[off] | (pkt[off+1] << 8)))
                    off += 9 + 3 * pkt[off + 8]
                cat_end = off + 3 * pkt[3]
                ids = []
                while off + 3 <= min(cat_end, len(pkt)):
                    ids.append(hex(pkt[off] | (pkt[off+1] << 8)))
                    off += 3
                print(f"[probe] rx {self.n.id:#06x}: "
                      f"owner {pkt[1] | (pkt[2]<<8):#06x} content={cent} catalogue={ids}",
                      flush=True)
        buf = (ctypes.c_uint8 * len(pkt)).from_buffer_copy(pkt) if pkt else (ctypes.c_uint8 * 1)()
        self.lib.mimeshi_rx(ctypes.byref(self.mn), ch, rssi, buf, len(pkt))
    def set_geo(self, lat, lon):
        self.lib.mimeshi_set_geo(ctypes.byref(self.mn), lat, lon)
    def star_state(self, star_id):
        """(present, star_tick, age_ticks, effective_age) of a star in the
        map - the 4.5 liveness/age measurement probes."""
        for m in self.mn.map:
            if m.alive and m.id == star_id:
                eff = m.age_ticks + (self.mn.tick - m.arrived_tick)
                return True, m.star_tick, m.age_ticks, eff
        return False, 0, 0, 0
    def miss_count(self):
        return sum(1 for e in self.mn.star_miss if e.alive)
    def map_size(self):
        return sum(1 for m in self.mn.map if m.alive)
    def link_count(self):
        return sum(1 for l in self.mn.lnk if l.alive)
    def send_data(self, dst, data, ack3=True, geo=None):
        """geo: (lat, lon) in the 16-bit grid of section 5 - the
        client-supplied destination geo (10.2); None = unknown."""
        payload = data.encode() if isinstance(data, str) else bytes(data)
        gl, go = geo if geo else (0, 0)
        rc = self.lib.mimeshi_send_data(ctypes.byref(self.mn), dst, payload, len(payload), ack3, gl, go)
        if rc != 0:
            out(f"[sim] node {self.n.id:#06x}: transmission refused (rc={rc})")
        return rc

def sym_count(node):
    """Number of neighbors with confirmed symmetric traffic (nb.sym,
    4.2/10.1). Only visible on the C core (the status lives in the node
    struct)."""
    core = node.core
    if isinstance(core, CCore):
        return sum(1 for e in core.mn.nbr if e.alive and e.sym)
    return None

def nbr_counts(node):
    """(neighbor count seen, count confirmed symmetric) according to the core."""
    core = node.core
    if isinstance(core, CCore):
        e = core.mn.nbr
        return (sum(1 for x in e if x.alive), sum(1 for x in e if x.alive and x.sym))
    return None, None

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--so', default=os.path.join(os.path.dirname(__file__), '..', 'build', 'libmimeshi.so'))
    args = ap.parse_args()
    if not os.path.exists(args.so):
        out(f"libmimeshi.so not found ({args.so}); build with gcc (see README).")
        sys.exit(1)
    # let printf of the C core appear directly after the Python prints
    try:
        libc = ctypes.CDLL(None)
        libc.setbuf(ctypes.c_void_p.in_dll(libc, "stdout"), None)
    except OSError:
        pass

    radio = Radio()
    a = Node(radio, 0x0100)   # home channel 1
    b = Node(radio, 0x0101)   # home channel 2
    lib = load_lib(args.so)
    a.core = CCore(a, lib); b.core = CCore(b, lib)

    out("=== Scenario 1: link test (2 nodes, SF7, 62.5 kHz) ===")
    a.core.start()
    radio.schedule(500_000, b.core.start)   # real nodes never start on the same
    # microsecond; 0.5 s keeps the first rounds free of overlap so the
    # unicast at 1.5 s can lean on a real neighbor table (overlap between
    # rounds is measured by the sweeps below).
    radio.run(until_us=1_500_000)           # first hello round + acks

    out("\n--- unicast A -> B (A knows B from the hello round) ---")
    a.core.send_data(0x0101, "hello mesh", ack3=True)
    radio.run(until_us=3_000_000)          # hello-ack from A shows B the map

    out("\n--- unicast B -> A (after hello-ack: B knows A) ---")
    b.core.send_data(0x0100, "reply message", ack3=True)
    radio.run(until_us=50_000_000)         # second hello rounds (30 s + jitter)

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

if __name__ == '__main__':
    main()
