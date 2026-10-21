# Meshimeshi — protocol core for a rural LoRa mesh (NL, ~3000 nodes)

Protocol core in C for repeaters/companions + Python simulator. See
`docs/design.md` for the full design document (status per component).

## Status (Oct 2026)

- **Building block 1 complete**: hello (adaptive startup rounds + jitter,
  then per-channel keepalive), directional hello-acks
  with snooping, symmetry status per neighbor (seen/confirmed), unicast
  with ack1/2/3, retries, budget, collision and capture model, GPS/height
  link budget.
  Scenarios up to and including `local-net` (40 nodes from a node list).
- **In preparation**: ack-on-demand in the core (4.1), slot × 3/n
  (open point 3), building block 2: tablepull, aging, routing (10).
- All decisions and measurements are documented with rationale in
  `docs/design.md`; the work order in 14.1.

## Structure

```
meshimeshi/
├── core/            # protocol core: platform-independent, no dynamic allocation
│   ├── core.h       #   API + hardware interface (tx / timer / switch / RNG)
│   └── core.c       #   hello, neighbor table, acks, unicast, budget, switch rule
├── sim/             # simulator (event-driven virtual radio, SF7/62.5 kHz)
│   ├── sim1.py      #   platform layer (clock/radio/RNG) + link-test scenario
│   └── meshsim.py   #   simpy engine + scenario registry (reuses sim1.py)
├── platform/        # hardware layers: only implement the four callbacks
│   ├── host/        #   workstation (stub, for tests)
│   ├── nrf52/       #   RAK4631 etc. (to be built)
│   └── esp32/       #   ESP32-S3 (to be built)
├── tests/           # C tests on the core itself (ctest)
└── docs/            # design document
```

## Building (Linux workstation)

```bash
cmake -B build
cmake --build build
ctest --test-dir build            # link test on the C core

# or without cmake:
gcc -O2 -fPIC -shared -DMC_BOUND_POOL=256 core/core.c -o build/libmimeshi.so -I.
gcc -O2 -Wall -Wextra -Werror -I. tests/test_link_test.c core/core.c -o build/test_link_test
# (the pool flag: the simulator shares the C core between all nodes of the
#  scenario; firmware builds without it and gets the compact pool)

# simulator against the compiled core:
python3 sim/sim1.py                # link-test scenario, uses build/libmimeshi.so

# multiple scenarios via simpy (one-time setup):
python3 -m venv .venv && .venv/bin/pip install simpy
.venv/bin/python sim/meshsim.py --list          # available scenarios
.venv/bin/python sim/meshsim.py link-test     # same run as sim1.py (engine check)
.venv/bin/python sim/meshsim.py hello-interval -q
.venv/bin/python sim/meshsim.py jitter-sweep   # hello sync vs jitter range (open point 3)
.venv/bin/python sim/meshsim.py collision       # collision + capture (6 dB margin, design 14.1)
.venv/bin/python sim/meshsim.py local-net    # 40 nodes from sim/nodes/*.csv (GPS+height)
.venv/bin/python sim/meshsim.py local-net --nodes sim/nodes/utrecht40.csv --duration 8
.venv/bin/python sim/meshsim.py hello-interval --jitter 5000000   # vary the jitter range
```

## Working rules (from design 13b)

- The core is one C library; firmware and simulator use the same source.
- Protocol logic exists exclusively in the C core: Python is only the
  platform and simulation layer (clock, radio, RNG), not a second protocol
  implementation.
- No dynamic memory allocation in the core; fixed buffers.
- Only the platform layer replaces the four hardware callbacks.
- Every protocol change first in the design document, then in the core,
  then grow both tests (ctest + sim1.py).
