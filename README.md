# Meshimeshi — protocolkern voor landelijke LoRa-mesh (NL, ~3000 nodes)

Protocolkern in C voor repeaters/companions + Python-simulator. Zie
`docs/ontwerp.md` voor het volledige ontwerpdocument (statussen per onderdeel).

## Status (okt 2026)

- **Bouwsteen 1 compleet**: hello (adaptief + jitter), directionele hello-acks
  met snooping, symmetrie-status per buur (gezien/bevestigd), unicast met
  ack1/2/3, retries, budget, botstings- en capturemodel, GPS/hoogte-linkbudget.
  Scenario's tot en met `lokaal-net` (40 nodes uit een nodelijst).
- **In voorbereiding**: hello-op-behoefte in de kern (4.1), slot × 3/n
  (open punt 3), bouwsteen 2: tablepull, veroudering, routering (10).
- Alle besluiten en metingen staan met rationale in `docs/ontwerp.md`;
  de werkvolgorde in 14.1.

## Structuur

```
meshimeshi/
├── core/            # protocolkern: platformonafhankelijk, geen dynamische allocatie
│   ├── core.h       #   API + hardware-interface (tx / timer / wissel / RNG)
│   └── core.c       #   hello, neighbor table, acks, unicast, budget, wisselregel
├── sim/             # simulator (event-gestuurde virtuele radio, SF7/62,5 kHz)
│   ├── sim1.py      #   platformlaag (klok/radio/RNG) + koppeltest-scenario
│   └── meshsim.py   #   simpy-motor + scenario-register (hergebruikt sim1.py)
├── platform/        # hardware-lagen: alleen de vier callbacks implementeren
│   ├── host/        #   werkstation (stub, voor tests)
│   ├── nrf52/       #   RAK4631 e.d. (nog te bouwen)
│   └── esp32/       #   ESP32-S3 (nog te bouwen)
├── tests/           # C-tests op de kern zelf (ctest)
└── docs/            # ontwerpdocument
```

## Bouwen (Linux werkstation)

```bash
cmake -B build
cmake --build build
ctest --test-dir build            # koppeltest op de C-kern

# of zonder cmake:
gcc -O2 -fPIC -shared -DMC_BOUND_POOL=256 core/core.c -o build/libmimeshi.so -I.
gcc -O2 -Wall -Wextra -Werror -I. tests/test_koppeltest.c core/core.c -o build/test_koppeltest
# (de pool-vlag: de simulator deelt de C-kern tussen alle nodes van het
#  scenario; firmware bouwt zonder en krijgt de compacte pool)

# simulator tegen de gecompileerde kern:
python3 sim/sim1.py                # koppeltest-scenario, gebruikt build/libmimeshi.so

# meerdere scenario's via simpy (eenmalig):
python3 -m venv .venv && .venv/bin/pip install simpy
.venv/bin/python sim/meshsim.py --list          # beschikbare scenario's
.venv/bin/python sim/meshsim.py koppeltest     # zelfde run als sim1.py (motorcheck)
.venv/bin/python sim/meshsim.py hello-interval -q
.venv/bin/python sim/meshsim.py jitter-sweep   # hello-sync vs jitter-range (open punt 3)
.venv/bin/python sim/meshsim.py botsing       # botsing + capture (6 dB-marge, ontwerp 14.1)
.venv/bin/python sim/meshsim.py lokaal-net    # 40 nodes uit sim/nodes/*.csv (GPS+hoogte)
.venv/bin/python sim/meshsim.py lokaal-net --nodes sim/nodes/utrecht40.csv --duur 8
.venv/bin/python sim/meshsim.py hello-interval --jitter 5000000   # jitter range variëren
```

## Werkregels (uit ontwerp 13b)

- De kern is één C-bibliotheek; firmware én simulator gebruiken dezelfde bron.
- Protocollogica bestaat uitsluitend in de C-kern: Python is alleen platform-
  en simulatielaag (klok, radio, RNG), geen tweede protocolimplementatie.
- Geen dynamische geheugenallocatie in de kern; vaste buffers.
- Alleen de platformlaag vervangt de vier hardware-callbacks.
- Elke protocolwijziging eerst in het ontwerpdocument, dan in de kern, dan
  beide testen (ctest + sim1.py) laten groeien.
