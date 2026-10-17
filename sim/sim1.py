#!/usr/bin/env python3
"""
sim1.py — Meshimeshi simulator, scenario 1 (koppeltest: 2 nodes).

Architectuur (ontwerp 14.1): event-gestuurde discrete simulatie; de protocolkern
is C en wordt via ctypes aangeroepen (default build/libmimeshi.so, gemaakt met
  gcc -O2 -fPIC -shared core/core.c -o build/libmimeshi.so).
De C-kern is de enige protocolimplementatie (ontwerp 13b): dit bestand is
uitsluitend platformlaag — klok, radio, RNG — en bevat geen protocollogica.
De simulator op simpy (meshsim.py) hergebruikt Node/Wire/CCCore hier.

Radiomodel (wisselregel, ontwerp 2):
- Elke node heeft EEN radio. Wisselverzoeken worden per node geserialiseerd
  (FIFO); een zending start pas als de radio vrij is.
- Een gebonden uitwisseling = wissel (S) + zending (airtime) + eventueel
  ack1-venster (data op een vreemd kanaal), daarna automatisch terug naar het
  thuiskanaal. De zending die bij een wissel hoort volgt direct op de wissel.
- Een node hoort een pakket alleen op zijn huidige kanaal, en niet terwijl
  hij zelf aan het wisselen of zenden is.

Metingen: airtime per kanaal, thuis-doofheid per node, leveringsuitkomst.
"""
import argparse, ctypes, heapq, math, os, random, sys
from collections import deque

T_HELLO = 70_000          # us, SF7/62.5k/CR4_5
T_ACK   = 78_000
T_DATA  = lambda l: 173_000 if l <= 40 else 459_000 if l <= 150 else 869_000
S_SWITCH = 5_000          # kanaalwissel incl. settling (ontwerp 14.1)
ACK1_MARGIN = 80_000      # marge op de ack1-wachttijd, gelijk aan core.c
N_CH = 3
HOME = lambda nid: nid % N_CH
RSSI = -60                # linkkwaliteit default; per paar in te stellen (Wire.links)
CAPTURE_DB = 6            # marge waarbij de sterkste wint (ontwerp 2/14.1)

VERBOSE = True             # False: radiologs (tx/hoort-niets) onderdrukken

def out(*a):
    print(*a, flush=True)

def radio_log(*a):
    if VERBOSE:
        print(*a, flush=True)

class Wire:
    """Kanaalmodel gedeeld door beide motoren: lopende zendingen per kanaal,
    link-RSSI per (zender, ontvanger), botsing en capture (ontwerp 2 en 14.1:
    twee overlappende zendingen op hetzelfde kanaal botsen, tenzij de sterkste
    met >= CAPTURE_DB marge wint)."""
    def __init__(self):
        self.chan = [[] for _ in range(N_CH)]   # (start, eind, zender-id)
        self.links = {}                          # (zender-id, ontvanger-id) -> dBm
        self.collided = [0]*N_CH               # verloren ontvangsten per kanaal
        self.open = True    # open wereld: onbekende paren ontvangen met de
                           # default-RSSI (kleine scenario's zonder bereikmodel);
                           # False = gesloten wereld: alleen paren in links
                           # bestaan (linkbudget uit posities, lokaal-net)
    def link(self, tx_id, rx_id):
        if self.open:
            return self.links.get((tx_id, rx_id), RSSI)
        return self.links.get((tx_id, rx_id))   # None = geen link
    def begin(self, now, ch, tx_id, airtime):
        self.chan[ch] = [w for w in self.chan[ch] if w[1] > now]
        self.chan[ch].append((now, now + airtime, tx_id))
    def check(self, now, rx_id, tx_id, ch, start, end):
        """Verstoring van de ontvangst [start, end) op kanaal ch.
        None = schoon; anders (reden, verstorende zender).
        Opruimen mag pas ná het te beoordelen venster: bij de aflevercontrole
        (op tx-eind) moeten zendingen die dit venster overlappen er nog in
        staan, ook al zijn ze zelf al voorbij."""
        self.chan[ch] = [w for w in self.chan[ch] if w[1] > start]
        s = self.link(tx_id, rx_id)
        for s0, e0, t0 in self.chan[ch]:
            if t0 == tx_id or e0 <= start or s0 >= end:
                continue
            s1 = self.link(t0, rx_id)
            if s1 is None:
                continue                     # verstoring buiten bereik stoort niet
            if s1 >= s + CAPTURE_DB:
                return ("capture door", t0)
            if s1 > s - CAPTURE_DB:
                return ("botsing met", t0)
        return None

class DutyLedger:
    """Zendtijd per (node, kanaal) in een schuivend venster — de 10% duty
    per kanaal (ontwerp 2). Registreert alle zendingen van elke Radio."""
    def __init__(self, venster_us=3_600_000_000):
        self.venster = venster_us
        self.log = {}                       # (node-id, kanaal) -> deque[(t, us)]
    def add(self, nid, ch, nu, us):
        d = self.log.setdefault((nid, ch), deque())
        d.append((nu, us))
        while d and d[0][0] <= nu - self.venster:
            d.popleft()
    def gebruik(self, nid, ch, nu):
        return sum(us for t, us in self.log.get((nid, ch), ()) if t > nu - self.venster)

# ---------------- knooppuntlijst en linkbudget (scenariolaag) -----------
# Radiomodel (Voorstel; NL-kalibratie: Open, zie ontwerp 14.1):
#   ontvangst = TX - FSPL(d) - 2 x clutter(h), begrensd door de radiohorizon.
# clutter(h) = 30 - 14 log10(1+h) dB: lage masten verliezen veel aan
# omgeving, hoge masten kijken er overheen. Per richting een fading-offset
# van +-5 dB zodat links asymmetrisch zijn, zoals in de praktijk.
TX_DBM = 27.0          # ERP in de 869,4-869,65 MHz sub-band (500 mW)
RX_MIN_DBM = -110.0    # ontvangstdrempel incl. fade-marge
FSPL_1KM_DB = 91.2    # vrije-ruimte verlies op 869 MHz over 1 km

def load_nodes_csv(pad):
    """Knooppuntlijst met GPS-positie en hoogte. Formaat per regel:
    id,lat,lon,hoogte_m,naam — '#' is commentaar."""
    nodes = []
    with open(pad) as f:
        for regel in f:
            regel = regel.strip()
            if not regel or regel.startswith('#'):
                continue
            d = [v.strip() for v in regel.split(',')]
            nodes.append({'id': int(d[0], 0), 'lat': float(d[1]),
                          'lon': float(d[2]), 'h': float(d[3]),
                          'naam': d[4] if len(d) > 4 else ''})
    return nodes

def projecteer(lat, lon, lat0, lon0):
    """Lokale vlakprojectie in meters rond (lat0, lon0): afstanden binnen
    een regio zijn klein genoeg voor een benadering met cos(lat0)."""
    x = math.radians(lon - lon0) * 6371000.0 * math.cos(math.radians(lat0))
    y = math.radians(lat - lat0) * 6371000.0
    return x, y

def linkbudget(pa, pb):
    """Ruwe ontvangststerkte in dBm van a naar b (posities in meters,
    hoogte in meters), zonder fading en zonder drempel.
    None = link fysiek onmogelijk: radiohorizon (breking meegerekend,
    d <= 4,12 (wortel(h1) + wortel(h2)) km)."""
    d_km = math.hypot(pa[0] - pb[0], pa[1] - pb[1]) / 1000.0
    if d_km <= 1e-3:
        return None
    if d_km > 4.12 * (math.sqrt(pa[2]) + math.sqrt(pb[2])):
        return None
    fspl = FSPL_1KM_DB + 20.0 * math.log10(d_km)
    clutter = (30.0 - 14.0 * math.log10(1.0 + pa[2])) + \
              (30.0 - 14.0 * math.log10(1.0 + pb[2]))
    return TX_DBM - fspl - clutter

class Radio:  # virtuele radio + klok
    def __init__(self):
        self.now = 0
        self.q = []          # (at, seq, fn, arg)
        self.nodes = {}      # id -> Node
        self.airtime = [0]*N_CH
        self.hello_air = [0]*N_CH   # airtime van hello's (type 1) per kanaal
        self.ack_air = [0]*N_CH     # airtime van hello-acks (type 2) per kanaal
        self.duty = DutyLedger()
        self.wire = Wire()
        self.seq = 0
    def schedule(self, delay_us, fn, arg=None):
        self.seq += 1
        heapq.heappush(self.q, (self.now + delay_us, self.seq, fn, arg))
    def run(self, until_us=None):
        while self.q:
            at, _, fn, arg = self.q[0]              # peek: niet poppen vóór de until-check
            if until_us and at > until_us: return False
            heapq.heappop(self.q)
            self.now = at
            fn() if arg is None else fn(arg)
            if until_us is None and not self.q: return True
        return True
    def tx(self, node, ch, pkt, airtime):
        radio_log(f"[{self.now/1e6:8.3f}s] node {node.id:#06x} zendt type {pkt[0]:#04x} "
                  f"({len(pkt)} B, {airtime//1000} ms) op kanaal {ch}")
        self.airtime[ch] += airtime
        ptype = pkt[0] & 0x0F
        if ptype == 1: self.hello_air[ch] += airtime
        elif ptype == 2: self.ack_air[ch] += airtime
        self.duty.add(node.id, ch, self.now, airtime)
        self.wire.begin(self.now, ch, node.id, airtime)
        for r in self.nodes.values():
            if r is node: continue
            r.schedule_recv(airtime, ch, bytes(pkt), node.id)   # hoorbaarheid checkt de ontvanger zelf

class Node:
    """Simulator-zijde van een node: radio-timing rondom de protocolkern."""
    def __init__(self, radio, nid):
        self.radio, self.id = radio, nid
        self.core = None
        self.ch = HOME(nid)        # huidig kanaal
        self.busy_until = 0        # radio bezet (wissel/zending/ack-venster) tot
        self.deaf_until = 0        # doof (wissel of eigen zending) tot
        self.away_total = 0        # us buiten het eigen thuiskanaal
        self._away_since = None
        self.hello_rx = 0          # gehoorde hellos (meting: ronde-sync, open punt 3)
        self.hello_missed = 0      # gemiste hellos op het eigen kanaal (weg/bezig/botsing)
        self.collided = 0          # ontvangsten verloren door botsing of capture
        self._swq = deque()        # wachtende wisselverzoeken
        self._drain_at = None
        self.rng = random.Random(nid)
        radio.nodes[nid] = self

    # ---- kanaaltoestand en meting --------------------------------------
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

    # ---- geserialiseerde radio ------------------------------------------
    def _when_free(self, job):
        """Voer de wissel-job uit zodra de radio vrij is (nu of na de lopende
        bezetting). Zendingen horen bij de eigen wissel en volgen direct,
        dus alleen wissels hoeven in de wachtrij."""
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
        if self.radio.now < self.busy_until:   # bezetting liep op (zending na wissel)
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
        """Zending op het huidige kanaal; wait>0 houdt de node op het kanaal
        voor de terugkeer. Standaard: data krijgt een ack1-venster
        (t_ack + marge, ontwerp 7.3) op het uitwisselingskanaal."""
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
        """Gebonden zending: wissel, zenden, (ack-venster), terug naar huis."""
        self.switch(ch, S_SWITCH, lambda: self.tx(ch, pkt, airtime, wait))

    # ---- ontvangst -------------------------------------------------------
    def schedule_recv(self, delay, ch, pkt, tx_id):
        def deliver():
            hello = (pkt[0] & 0x0F) == 1
            start = self.radio.now - delay
            rssi = self.radio.wire.link(tx_id, self.id)
            if rssi is None:
                return    # buiten bereik (gesloten wereld): geen ontvangst, geen telling
            # het hele pakket moet gehoord zijn: op het kanaal, en de eigen
            # zending/wissel moet vóór de start van dit pakket voorbij zijn
            if self.ch == ch and start >= self.deaf_until:
                stooring = self.radio.wire.check(self.radio.now, self.id, tx_id, ch, start, self.radio.now)
                if stooring:
                    reden, door = stooring
                    self.collided += 1
                    self.radio.wire.collided[ch] += 1
                    if hello: self.hello_missed += 1
                    radio_log(f"[{self.radio.now/1e6:8.3f}s] node {self.id:#06x} verliest "
                              f"type {pkt[0]:#04x} op kanaal {ch} ({reden} {door:#06x})")
                    return
                if hello: self.hello_rx += 1
                self.core.rx((ch, rssi, pkt)); return
            if ch == HOME(self.id) or self.ch == ch:
                if hello: self.hello_missed += 1
                reden = ("bezig met eigen zending" if start < self.deaf_until
                         else "van huis (gebonden uitwisseling)")
                radio_log(f"[{self.radio.now/1e6:8.3f}s] node {self.id:#06x} hoort niets "
                          f"op kanaal {ch} ({reden})")
        self.radio.schedule(delay, deliver)

# ---------------------------------------------------------------------------
# C-kern via ctypes — de enige protocolimplementatie (ontwerp 13b).
# Structlayout is geverifieerd tegen sizeof/offsetof van core.h
# (neighbor 12, pending 256, node 5440 B).
# ---------------------------------------------------------------------------
MC_MAX_NEIGHBORS, MC_MAX_PENDING, MC_MAX_DEDUP, MC_MAX_PAYLOAD = 64, 16, 128, 230

class CNeighbor(ctypes.Structure):
    _fields_ = [("id", ctypes.c_uint16), ("quality", ctypes.c_uint8),
                ("ack_try", ctypes.c_uint8), ("last_seen", ctypes.c_uint32),
                ("state_ver", ctypes.c_uint16), ("alive", ctypes.c_uint8),
                ("sym", ctypes.c_uint8)]

class CPending(ctypes.Structure):
    _fields_ = [("active", ctypes.c_uint8), ("tries_left", ctypes.c_uint8),
                ("budget_left", ctypes.c_uint8), ("msg_id", ctypes.c_uint16),
                ("src", ctypes.c_uint16), ("dst", ctypes.c_uint16),
                ("prev_hop", ctypes.c_uint16), ("next_hop", ctypes.c_uint16),
                ("pkt", ctypes.c_uint8 * (10 + MC_MAX_PAYLOAD)),
                ("pkt_len", ctypes.c_uint16)]

class CDedup(ctypes.Structure):
    _fields_ = [("msg_id", ctypes.c_uint16), ("src", ctypes.c_uint16)]

class CNode(ctypes.Structure):
    pass

CB = ctypes.CFUNCTYPE(None, ctypes.POINTER(CNode), ctypes.c_void_p)   # kern-callback (timer/switch-done)
TX_FN = ctypes.CFUNCTYPE(None, ctypes.POINTER(CNode), ctypes.c_uint8,
                         ctypes.POINTER(ctypes.c_uint8), ctypes.c_uint16, ctypes.c_uint64)
TIMER_FN = ctypes.CFUNCTYPE(None, ctypes.POINTER(CNode), ctypes.c_uint64,
                            ctypes.c_void_p, ctypes.c_void_p)
SWITCH_FN = ctypes.CFUNCTYPE(None, ctypes.POINTER(CNode), ctypes.c_uint8, ctypes.c_uint64,
                             ctypes.c_void_p, ctypes.c_void_p)
RAND_FN = ctypes.CFUNCTYPE(ctypes.c_uint32, ctypes.POINTER(CNode))

CNode._fields_ = [
    ("id", ctypes.c_uint16), ("home_ch", ctypes.c_uint8), ("my_state_ver", ctypes.c_uint16),
    ("nbr", CNeighbor * MC_MAX_NEIGHBORS), ("pend", CPending * MC_MAX_PENDING),
    ("dedup", CDedup * MC_MAX_DEDUP), ("hello_interval_us", ctypes.c_uint64),
    ("hello_jitter_us", ctypes.c_uint64),
    ("tx", TX_FN), ("timer", TIMER_FN), ("switch_ch", SWITCH_FN), ("rand", RAND_FN),
    ("user", ctypes.c_void_p),
]

NODES = {}   # adres van mimeshi_node -> Python-Node

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

def load_lib(path):
    lib = ctypes.CDLL(path)
    lib.mimeshi_init.argtypes = [ctypes.POINTER(CNode), ctypes.c_uint16, TX_FN,
                                 TIMER_FN, SWITCH_FN, RAND_FN, ctypes.c_void_p]
    lib.mimeshi_start.argtypes = [ctypes.POINTER(CNode)]
    lib.mimeshi_rx.argtypes = [ctypes.POINTER(CNode), ctypes.c_uint8, ctypes.c_int8,
                               ctypes.POINTER(ctypes.c_uint8), ctypes.c_uint16]
    lib.mimeshi_send_data.argtypes = [ctypes.POINTER(CNode), ctypes.c_uint16,
                                      ctypes.c_char_p, ctypes.c_uint16, ctypes.c_bool]
    lib.mimeshi_send_data.restype = ctypes.c_int
    return lib

class CCore:
    """Protocolkern in C; elke node heeft zijn eigen mimeshi_node (geen
    gedeelde state). De vier callbacks lopen via de simulator hierboven."""
    def __init__(self, node, lib):
        self.n, self.lib = node, lib
        self.mn = CNode()
        NODES[ctypes.addressof(self.mn)] = node
        lib.mimeshi_init(ctypes.byref(self.mn), node.id, _tx, _timer, _switch, _rand, None)
    def start(self):
        self.lib.mimeshi_start(ctypes.byref(self.mn))
    def rx(self, arg):
        ch, rssi, pkt = arg
        buf = (ctypes.c_uint8 * len(pkt)).from_buffer_copy(pkt) if pkt else (ctypes.c_uint8 * 1)()
        self.lib.mimeshi_rx(ctypes.byref(self.mn), ch, rssi, buf, len(pkt))
    def send_data(self, dst, text, ack3=True):
        payload = text.encode()
        rc = self.lib.mimeshi_send_data(ctypes.byref(self.mn), dst, payload, len(payload), ack3)
        if rc != 0:
            out(f"[sim] node {self.n.id:#06x}: zending geweigerd (rc={rc})")
        return rc

def sym_count(node):
    """Aantal buren met bevestigd symmetrisch verkeer (nb.sym, 4.2/10.1).
    Alleen zichtbaar op de C-kern (de status woont in de node-struct)."""
    core = node.core
    if isinstance(core, CCore):
        return sum(1 for e in core.mn.nbr if e.alive and e.sym)
    return None

def nbr_counts(node):
    """(aantal buren gezien, aantal bevestigd symmetrisch) volgens de kern."""
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
        out(f"libmimeshi.so niet gevonden ({args.so}); bouw met gcc (zie README).")
        sys.exit(1)
    # printf van de C-kern direct na de Python-prints laten verschijnen
    try:
        libc = ctypes.CDLL(None)
        libc.setbuf(ctypes.c_void_p.in_dll(libc, "stdout"), None)
    except OSError:
        pass

    radio = Radio()
    a = Node(radio, 0x0100)   # thuis kanaal 1
    b = Node(radio, 0x0101)   # thuis kanaal 2
    lib = load_lib(args.so)
    a.core = CCore(a, lib); b.core = CCore(b, lib)

    out("=== Scenario 1: koppeltest (2 nodes, SF7, 62,5 kHz) ===")
    a.core.start()
    radio.schedule(500_000, b.core.start)   # echte nodes starten nooit op dezelfde
    # microseconde; 0,5 s houdt de eerste rondes vrij van overlap zodat de
    # unicast op 1,5 s op een echte buurtabel kan steunen (overlap tussen
    # rondes meten de sweeps onderaan).
    radio.run(until_us=1_500_000)           # eerste hello-ronde + acks

    out("\n--- unicast A -> B (A kent B uit de hello-ronde) ---")
    a.core.send_data(0x0101, "hallo mesh", ack3=True)
    radio.run(until_us=3_000_000)          # hello-ack van A laat B de kaart zien

    out("\n--- unicast B -> A (na hello-ack: B kent A) ---")
    b.core.send_data(0x0100, "terugbericht", ack3=True)
    radio.run(until_us=50_000_000)         # tweede hello-rondes (30 s + jitter)

    out("\n=== Metingen ===")
    for c in range(N_CH):
        out(f"kanaal {c}: airtime {radio.airtime[c]/1e6:.3f} s, "
            f"verloren ontvangsten {radio.wire.collided[c]}")
    for nid, nd in radio.nodes.items():
        sym = sym_count(nd)
        out(f"node {nid:#06x}: thuis-doofheid {nd.away_total/1e6:.3f} s "
            f"({100*nd.away_total/max(radio.now,1):.2f}% van de tijd), "
            f"hellos gehoord {nd.hello_rx}, gemist {nd.hello_missed}, "
            f"botsingen {nd.collided}"
            + (f", symmetrisch bevestigd {sym}" if sym is not None else ""))

if __name__ == '__main__':
    main()
