#!/usr/bin/env python3
"""
meshsim.py — Meshimeshi simulator op simpy, met scenario-register.

De engine hier is een simpy.Environment (tijd in microseconden, gehele
getallen), met exact dezelfde Radio-interface als sim1.Radio. Alles wat geen
klok is — radiomodel per node (Node), kanaalmodel (Wire), C-kern via ctypes
(CCore) — wordt hergebruikt uit sim1.py. De koppeltest via dit script
moet byte-voor-byte dezelfde uitvoer geven als sim1.py; dat is de
motorcheck tussen de handgeschreven eventwachtrij en simpy.

Scenario's worden geregistreerd met @scenario("naam") en draaien met:
  .venv/bin/python sim/meshsim.py <naam> [-q] [--jitter US]
Nieuwe scenario's: voeg een functie toe met (mk, jitter)-handtekening, waar
mk(node) de C-protocolkern maakt (CCore; er is geen andere kern) en jitter
de max hello-fase-jitter in microseconden (None = kernwaarde).
"""
import argparse, os, sys

import simpy

from sim1 import (radio_log, Node, CCore, load_lib, Wire, sym_count, nbr_counts,
                  load_nodes_csv, projecteer, linkbudget, RX_MIN_DBM, DutyLedger, N_CH)

class Radio:  # simpy-versie van de virtuele radio; interface gelijk aan sim1.Radio
    def __init__(self):
        self.env = simpy.Environment(initial_time=0)
        self.nodes = {}        # id -> Node
        self.airtime = [0]*N_CH
        self.hello_air = [0]*N_CH   # airtime van hello's (type 1) per kanaal
        self.ack_air = [0]*N_CH     # airtime van hello-acks (type 2) per kanaal
        self.duty = DutyLedger()
        self.wire = Wire()
    @property
    def now(self):
        return self.env.now
    def schedule(self, delay_us, fn, arg=None):
        ev = self.env.timeout(int(delay_us))
        # callbacks krijgen het event mee; de scenario-code heeft daar niets aan
        ev.callbacks.append((lambda _e: fn()) if arg is None else (lambda _e: fn(arg)))
    def run(self, until_us=None):
        """Draait tot en with until_us (of leeg), en laat de klok op de laatste
        gebeurtenis staan — net als sim1.Radio.run."""
        if until_us is None:
            while self.env.peek() < float('inf'):
                self.env.step()
        else:
            while self.env.peek() <= until_us:
                self.env.step()
    def tx(self, node, ch, pkt, airtime):
        radio_log(f"[{self.now/1e6:8.3f}s] node {node.id:#06x} zendt type {pkt[0]:#04x} "
                  f"({len(pkt)} B, {airtime//1000} ms) op kanaal {ch}")
        self.airtime[ch] += airtime
        self.wire.begin(self.now, ch, node.id, airtime)
        ptype = pkt[0] & 0x0F
        if ptype == 1: self.hello_air[ch] += airtime
        elif ptype == 2: self.ack_air[ch] += airtime
        self.duty.add(node.id, ch, self.now, airtime)
        for r in self.nodes.values():
            if r is node: continue
            r.schedule_recv(airtime, ch, bytes(pkt), node.id)   # hoorbaarheid checkt de ontvanger zelf

# ---------------- scenario's ---------------------------------------------

SCENARIOS = {}   # naam -> (fn, beschrijving)

def scenario(name, desc=""):
    def deco(fn):
        SCENARIOS[name] = (fn, desc)
        return fn
    return deco

def out(*a):
    print(*a, flush=True)

def set_jitter(core, jitter_us):
    """Stelt de max hello-fase-jitter in op de kern (vóór start). None of een
    negatieve waarde laat de kernwaarde staan (MC_HELLO_JITTER_US = 1 s)."""
    if jitter_us is not None and jitter_us >= 0:
        core.mn.hello_jitter_us = jitter_us
    return core

def report(radio):
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

@scenario("koppeltest", "2 nodes: hello's, unicast A->B en B->A met ack1/ack2/ack3")
def koppeltest(mk, jitter):
    """Zelfde scenario als sim1.py: dient ook als equivalentiecheck van de engines."""
    radio = Radio()
    a = Node(radio, 0x0100)   # thuis kanaal 1
    b = Node(radio, 0x0101)   # thuis kanaal 2
    a.core = set_jitter(mk(a), jitter); b.core = set_jitter(mk(b), jitter)

    a.core.start()
    radio.schedule(500_000, b.core.start)   # startoffset: geen overlap tussen
    radio.run(until_us=1_500_000)           # de eerste rondes (zie sim1.py)

    out("\n--- unicast A -> B (A kent B uit de hello-ronde) ---")
    a.core.send_data(0x0101, "hallo mesh", ack3=True)
    radio.run(until_us=3_000_000)          # hello-ack van A laat B de kaart zien

    out("\n--- unicast B -> A (na hello-ack: B kent A) ---")
    b.core.send_data(0x0100, "terugbericht", ack3=True)
    radio.run(until_us=50_000_000)         # tweede hello-rondes (30 s + jitter)
    report(radio)

@scenario("hello-interval", "2 nodes, 95 s: adaptief hello-interval 30 s -> 60 s -> ...")
def hello_interval(mk, jitter):
    radio = Radio()
    a = Node(radio, 0x0100)
    b = Node(radio, 0x0101)
    a.core = set_jitter(mk(a), jitter); b.core = set_jitter(mk(b), jitter)
    a.core.start()
    radio.schedule(500_000, b.core.start)
    radio.run(until_us=95_000_000)         # rondes 2 (30 s) en 3 (60 s) + jitter
    report(radio)

@scenario("jitter-sweep", "2 nodes, 1000 s: ronde-sync tegen de jitter-range (open punt 3)")
def jitter_sweep(mk, _jitter):
    """Meet per jitter-range hoeveel hellos een node van de ander hoort en
    mist (eigen ronde / van huis). Voorbereiding op scenario 2; de CLI-jitter
    staat hierbuiten, de sweep bepaalt hem zelf."""
    out(f"{'jitter':>9s} {'A hoort':>8s} {'A mist':>7s} {'B hoort':>8s} {'B mist':>7s}")
    for j in (0, 100_000, 500_000, 1_000_000, 5_000_000):
        radio = Radio()
        a = Node(radio, 0x0100); b = Node(radio, 0x0101)
        a.core = set_jitter(mk(a), j); b.core = set_jitter(mk(b), j)
        a.core.start()
        radio.schedule(137_000, b.core.start)
        radio.run(until_us=1_000_000_000) # rondes t/m ~930 s (30-60-120-240-480 s)
        out(f"{j/1e6:>9.3f} {a.hello_rx:>8d} {a.hello_missed:>7d} "
            f"{b.hello_rx:>8d} {b.hello_missed:>7d}")

@scenario("nieuwkomer", "stabiele R1/R2 + nieuwkomer op 5 s: wanneer bewijst wie symmetrie")
def nieuwkomer(mk, jitter):
    """Demonstreert de bewijsasymmetrie na opstart (4.2/4.3): de nieuwkomer
    N krijgt zijn symmetriebewijzen binnen seconden (acks op zijn eigen
    ronde, aan hem gericht); de responders R1/R2 staan N daarna nog als
    "gezien". Hun tegenrichtingbewijs komt pas via het hello-proces —
    N ackt hun ronde 2 op ~30 s — of straks via de opstartpull (4.3,
    bouwsteen 2). N bestaat pas vanaf t=5 s, zodat ronde 1 van R1/R2
    écht gemist wordt."""
    radio = Radio()
    r1 = Node(radio, 0x0100)   # thuis 1
    r2 = Node(radio, 0x0101)   # thuis 2
    r1.core = set_jitter(mk(r1), jitter); r2.core = set_jitter(mk(r2), jitter)
    # Heterogene links (realistischer dan de default -60): responders met
    # verschillende RSSI-rang komen in verschillende slots, zodat acks aan
    # dezelfde ontvanger niet structureel botsen (zie ontwerp 2, randje).
    radio.wire.links[(0x0100, 0x0101)] = -70   # R1 -> R2
    radio.wire.links[(0x0101, 0x0100)] = -66   # R2 -> R1
    radio.wire.links[(0x0100, 0x0102)] = -57   # R1 -> N
    radio.wire.links[(0x0102, 0x0100)] = -55   # N -> R1
    radio.wire.links[(0x0101, 0x0102)] = -68   # R2 -> N
    radio.wire.links[(0x0102, 0x0101)] = -58   # N -> R2
    nn = [None]
    def start_nieuwkomer():
        nn[0] = Node(radio, 0x0102)   # thuis 0
        nn[0].core = set_jitter(mk(nn[0]), jitter)
        nn[0].core.start()
    def symstatus(label):
        out(f"[sym] {label}: R1={sym_count(r1)} R2={sym_count(r2)} N={sym_count(nn[0])}")
    r1.core.start()
    radio.schedule(500_000, r2.core.start)
    radio.schedule(5_000_000, start_nieuwkomer)
    radio.schedule(10_000_000, lambda: symstatus("t=10 s, na acks op de nieuwkomer"))
    radio.schedule(40_000_000, lambda: symstatus("t=40 s, na ronde 2 (hello-proces)"))
    radio.schedule(100_000_000, lambda: symstatus("t=100 s, na ronde 3"))
    radio.run(until_us=100_000_000)
    report(radio)

_ARGS = None   # CLI-context voor scenario's (lokaal-net leest --nodes/--duur)

@scenario("lokaal-net", "GPS-nodelijst (positie+hoogte) -> linkbudget; hello/ack-meting")
def lokaal_net(mk, jitter):
    """Ontwerp 14.1 scenario 2: 30-50 nodes op echte dichtheid. Meet
    hello- en ack-airtime (opstart vs stabiel), ack-botsingen, en de
    verhouding gezien/bevestigd (symmetrie), plus duty-cycle (10%/uur)."""
    lijst = load_nodes_csv(_ARGS.nodes)
    duur_us = _ARGS.duur * 3_600_000_000
    radio = Radio()
    radio.wire.open = False    # gesloten wereld: alleen linkbudget-paren bestaan
    lat0 = sum(n['lat'] for n in lijst) / len(lijst)
    lon0 = sum(n['lon'] for n in lijst) / len(lijst)
    pos = {n['id']: (*projecteer(n['lat'], n['lon'], lat0, lon0), n['h']) for n in lijst}
    nodes = []
    for i, nd in enumerate(lijst):
        kn = Node(radio, nd['id'])
        kn.core = set_jitter(mk(kn), jitter)
        nodes.append(kn)
        radio.schedule(i * 250_000, kn.core.start)   # nooit synchroon opstarten
    ids = [n['id'] for n in lijst]
    for a in ids:
        for b in ids:
            if a == b: continue
            raw = linkbudget(pos[a], pos[b])
            if raw is None: continue
            fade = ((a * 31 + b * 17) % 11) - 5     # deterministisch per richting
            r = raw + fade
            if r >= RX_MIN_DBM:
                radio.wire.links[(a, b)] = max(-128, min(120, round(r)))
    paren = {(a, b) if a < b else (b, a) for (a, b) in radio.wire.links}
    bidir = sum(1 for (a, b) in paren
                if (a, b) in radio.wire.links and (b, a) in radio.wire.links)
    graad = {i: 0 for i in ids}
    for (a, b) in paren:
        graad[a] += 1; graad[b] += 1
    out(f"{len(lijst)} nodes uit {_ARGS.nodes}")
    out(f"topologie: {len(paren)} links ({bidir} in beide richtingen), "
        f"gemiddelde graad {sum(graad.values())/len(ids):.1f}, "
        f"min {min(graad.values())}, max {max(graad.values())}")
    fase = {}
    def fasegrens():   # na de opstartrondes (laatste op ~1890 s); daarna alleen keepalive (4.1)
        fase.update(air=radio.airtime[:], hello=radio.hello_air[:],
                    ack=radio.ack_air[:], t=radio.now)
    radio.schedule(3_700_000_000, fasegrens)
    radio.run(until_us=duur_us)
    out("\n=== Metingen ===")
    t1 = fase.get('t', radio.now)
    out(f"fase 1: opstart 0-{t1/1e6:.0f} s | fase 2: stabiel "
        f"{t1/1e6:.0f}-{radio.now/1e6:.0f} s (laatste gebeurtenis op {radio.now/1e6:.0f} s)")
    for c in range(N_CH):
        a1 = fase['air'][c] if fase else 0
        h1 = fase['hello'][c] if fase else 0
        k1 = fase['ack'][c] if fase else 0
        a2, h2, k2 = radio.airtime[c]-a1, radio.hello_air[c]-h1, radio.ack_air[c]-k1
        stabiel_us = max(radio.now - t1, 1)
        out(f"kanaal {c}: opstart {a1/1e6:6.1f} s lucht (hello {h1/1e6:5.1f}, ack {k1/1e6:5.1f}) | "
            f"stabiel {a2/1e6:6.1f} s — hello {100*h2/stabiel_us:.2f}% en "
            f"ack {100*k2/stabiel_us:.2f}% van de kanaaltijd")
    stats = [nbr_counts(kn) for kn in nodes]
    gez = [s[0] for s in stats]; bef = [s[1] for s in stats]
    out(f"buren per node: gezien gem {sum(gez)/len(gez):.1f} (min {min(gez)}, max {max(gez)}), "
        f"bevestigd symmetrisch gem {sum(bef)/len(bef):.1f}")
    out(f"nodes zonder bevestigde buur: {sum(1 for b in bef if b == 0)} van {len(bef)}")
    out(f"botsingen: {sum(radio.wire.collided)} verloren ontvangsten, "
        f"{sum(kn.hello_missed for kn in nodes)} gemiste hellos")
    gebruik = {(n, c): radio.duty.gebruik(n, c, radio.now) for n in ids for c in range(N_CH)}
    top = sorted(gebruik.values(), reverse=True)[:5]
    out(f"duty (schuivend uurvenster, grens 10% = 360 s): max {top[0]/1e6:.1f} s, "
        f"hoogste 5: {[f'{u/1e6:.1f}' for u in top]} s, "
        f"boven de grens: {sum(1 for u in gebruik.values() if u > 360_000_000)}")

@scenario("botsing", "3 nodes: synchrone hellos botsen; capture bij >= 6 dB verschil")
def botsing(mk, jitter):
    """Stroomuitval-geval (ontwerp 4.1): A en B delen een thuiskanaal en
    starten exact synchroon (jitter 0) — hun hellos overlappen op alle drie
    de kanalen. Ronde 1: gelijke sterkte, dus C verliest beide (botsing).
    Ronde 2: B is bij C >= 6 dB sterker, dus C vangt B's hello (capture)
    en verliest A's."""
    radio = Radio()
    a = Node(radio, 0x0102)   # thuis 0
    b = Node(radio, 0x0105)   # thuis 0 — zelfde thuiskanaal als A
    c = Node(radio, 0x0103)   # thuis 1 — hoort A en B op kanaal 1
    a.core = set_jitter(mk(a), 0); b.core = set_jitter(mk(b), 0)
    c.core = set_jitter(mk(c), jitter)
    a.core.start(); b.core.start()   # exact gelijktijdig: het 4.1-randgeval
    radio.schedule(500_000, c.core.start)
    out("\n--- ronde 1: gelijke sterkte (default -60) -> botsingen bij C ---")
    radio.run(until_us=1_000_000)
    radio.wire.links[(a.id, c.id)] = -66
    radio.wire.links[(b.id, c.id)] = -60
    out("\n--- ronde 2 (30 s): B >= 6 dB sterker -> C vangt B, verliest A ---")
    radio.run(until_us=31_000_000)
    report(radio)

# ---------------- CLI -------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(description="Meshimeshi-scenario's op simpy")
    ap.add_argument('scenario', nargs='?', default='koppeltest',
                    help="scenario-naam (standaard: koppeltest)")
    ap.add_argument('--list', action='store_true', help="beschikbare scenario's tonen")
    ap.add_argument('--so', default=os.path.join(os.path.dirname(__file__), '..', 'build', 'libmimeshi.so'))
    ap.add_argument('-q', '--quiet', action='store_true', help='geen radio-logs, alleen [core]-regels')
    ap.add_argument('--jitter', type=int, default=None, metavar='US',
                    help='max hello-fase-jitter in microseconden (default: kernwaarde, 1 s)')
    ap.add_argument('--nodes', default=os.path.join(os.path.dirname(__file__), 'nodes', 'utrecht40.csv'),
                    help='knooppuntlijst (id,lat,lon,hoogte_m,naam) voor lokaal-net')
    ap.add_argument('--duur', type=int, default=8, metavar='UUR',
                    help='simulatieduur in uren (lokaal-net, standaard 8)')
    args = ap.parse_args()
    global _ARGS
    _ARGS = args

    if args.list:
        for name, (_, desc) in SCENARIOS.items():
            out(f"{name:16s} {desc}")
        return
    if args.scenario not in SCENARIOS:
        ap.error(f"onbekend scenario '{args.scenario}' (kies uit: {', '.join(SCENARIOS)})")
    if not os.path.exists(args.so):
        out(f"libmimeshi.so niet gevonden ({args.so}); bouw met gcc (zie README).")
        sys.exit(1)
    # printf van de C-kern direct na de Python-prints laten verschijnen
    import ctypes
    try:
        libc = ctypes.CDLL(None)
        libc.setbuf(ctypes.c_void_p.in_dll(libc, "stdout"), None)
    except OSError:
        pass

    import sim1
    if args.quiet:
        sim1.VERBOSE = False   # radiologs uitzetten; [core]-regels blijven

    lib = load_lib(args.so)
    mk = lambda nd: CCore(nd, lib)

    fn, desc = SCENARIOS[args.scenario]
    out(f"=== Scenario '{args.scenario}': {desc} (C-kern, SF7, 62,5 kHz) ===")
    fn(mk, args.jitter)

if __name__ == '__main__':
    main()
