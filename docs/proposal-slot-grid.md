# Proposal: receiver-anchored retry slots for unicast

**Status: proposal — deliberately NOT part of `docs/design.md` yet.**
This document captures the full design discussion (slot grid, learning
counters, parameters, weak points) so it can be evaluated and measured
before anything is integrated into the protocol core.

References like "§12.3" point to sections of the current `docs/design.md`.

---

## 1. Problem statement

Under sustained load, unicast delivery degrades through collisions:

- **Data-vs-data on first attempts.** Two senders pick (nearly) the same
  moment to start their exchange towards the same receiver. CAD cannot
  see transmitters that are hidden from each other (open point 8), so
  these collisions are not prevented today.
- **Retry collisions.** The retry series of §8.4 spaces retries of one
  sender, but two senders running the same schedule can collide again
  and again.
- **Data-vs-ack1.** A third sender's CAD window misses a short 70 ms
  ack1; the arriving data packet is lost at the receiver.

Measured baseline (README, Oct 2026): 81.7% delivery for random-to-random
unicast under sustained load on `utrecht40`; the remainder is largely
collision loss that CAD alone does not remove.

## 2. Design principles carried over from the discussion

1. **No new roles.** There is no anchor node and no election. Every node
   derive1 its own timing from signals that are already being sent.
2. **No global clock.** The only synchronisation source is the reception
   time of star rounds, which every neighbour hears anyway. (A multi-hop
   phase grid in the "power grid" style was considered and parked; it is
   not required for this proposal.)
3. **Worst case = today.** A node that has no anchor, no counters, or no
   slots falls back to plain ALOHA + CAD. The proposal may only add
   behaviour, never remove it.
4. **Consensus by computation, not negotiation.** Where nodes must agree
   (slot numbering), they compute the same answer from the same view;
   there is no negotiation traffic.

## 3. Mechanism

### 3.1 Receiver-anchored grid

- Each node derives a local superframe from the **last heard star round
  of the receiver** it wants to reach. Because exchanging nodes are on
  the receiver's home channel anyway, this anchor is available to every
  potential sender at no extra cost.
- Superframe: `S = 8` slots. Slot duration is sized for one data packet
  plus acknowledgement: data (100 B, 459 ms) + ack1 (70 ms) + guard
  ≈ **0.6 s** per slot; one superframe ≈ **4.8 s** (§12.3 airtimes).
- The slot index of a sender towards a receiver is
  `hash(sender_id) mod S` — deterministic, no communication. Hash
  collisions between two senders to the same receiver are resolved by
  the learning layer below, not by extra signalling.

### 3.2 Scope: retries only

- **First attempts stay ALOHA + CAD** (§2). Their latency profile is the
  protocol's core value and is not traded away.
- **Retries of unicast** are placed on the grid: instead of the pure
  timeout of the §8.4 series, the retry waits for the next occurrence of
  the sender's slot in the receiver's superframe.
- Everything else (ack1, ack2, ICUs, proof lists, announcements, star
  rounds) is untouched. Star rounds remain the synchronisation pulses;
  no new packet types are introduced.

### 3.3 Learning layer: per-neighbour, per-slot failure counters

- For each neighbour, the sender keeps **S one-byte collision counters**, 
  one per slot. A failed exchange increments the counter of the slot it
  used. Every 15 minutes (one star-round interval, no extra timer needed: 
  decay on the round boundary) each counter is halved, so old damage
  fades and the counters stay small.
- On collision, the sender re-picks its slot **at random among the 4
  slots with the lowest counters** (exploration), rather than
  deterministically. This keeps the population moving away from
  contested slots without synchronising them onto one "best" slot.
- Memory cost: 8 one-byte slots per neighbour, capped at 64 neighbours
  (the neighbour-table limit), i.e. **512 bytes of RAM worst case**.
- Counters double as a **passive congestion indicator**: their sum per
  neighbour is a locally observed load signal, available for free and
  usable as input for the link-pressure cost that open point 15 asks
  for. It measures contention *experienced*, not queue depth at the
  neighbour — the two are complementary, not the same signal.

### 3.4 Interaction with existing mechanisms

| Mechanism | Interaction |
|---|---|
| §8.4 retry series | The spacing schedule stays; the grid only decides *when* in the superframe the retry may fire. |
| CAD (§2) | Unchanged; slots reduce who is transmitting at the same time, CAD still guards the final check. |
| Capture effect (§2) | Unchanged; slot separation reduces how often it is needed. |
| Star round jitter | The per-round random phase (§5.1) makes slot boundaries fuzzy at multi-hop distance. If this proves measurable, a deterministic per-node offset can replace the random phase (open question). |
| Hidden nodes | Partially addressed: counters learn contested slots even when the interferer is invisible, because the cost shows up as loss regardless of who caused it. |
| Companions | Out of scope for now; companion traffic may move to a separate channel/SF (see open questions). |

### 3.5 Phase measurement from every packet

Every disciplined transmission doubles as a phase observation, free of
charge: a packet's arrival time minus its known airtime reconstructs the
sender's transmit start, and comparing that against the expected slot
position yields an offset measurement. The measurement rate therefore
scales with traffic — the busier the network, the denser the timing
evidence — and no new packet format is required.

Rules for using these observations:

- **Only disciplined traffic counts.** First attempts under ALOHA are
  unpositioned by design and are excluded from the phase estimate; star
  rounds and slot-placed retries are the primary sources.
- **Robust filtering.** Each node keeps a short history of offsets per
  neighbour; the phase estimate follows the median (or a similarly
  robust statistic), so a single noisy sample — or a node with a bad
  crystal — cannot pull anyone along.
- **Self-reinforcing.** The more traffic runs on the grid, the denser
  the sync evidence becomes: discipline produces the very signal that
  keeps the whole network locked to one rhythm.

### 3.6 Placement rule: align to the slot's end

Exchanges are aligned to the **end** of their slot, not the start: the
transmission is placed so that data plus acknowledgement finish at the
slot boundary. This turns packet length into the within-slot arbiter:

- The longest packet in a slot starts first and claims the slot.
- Shorter contenders do their CAD while the longer transmission is
  already on the air, hear it busy, and defer. With start-alignment,
  all contenders see a free channel and start simultaneously — CAD is
  defeated by the simultaneity itself. End-alignment restores CAD as
  the effective arbiter *within* a slot, complementing the grid that
  arbitrates *between* slots.
- **Equal-length residue.** Two same-length packets still start
  together. The existing jitter (§5.1), natural clock drift, and the
  learning layer (§3.3) catch this residue: a chronically deferred
  node moves to another slot.
- **Ack placement.** The exchange (data + ack1) ends at the slot
  boundary; the ack1 remains a CAD-exempt direct reply inside the
  slot.

## 4. Parameters

| Parameter | Default | Notes |
|---|---|---|
| Slots per superframe `S` | 8 | Raise to 16/32 if measurements show residual slot collisions under load. |
| Slot duration | ~0.6 s (data 100 B + ack + guard) | Follows from §12.3; recompute for other frame sizes. |
| Counter width | 1 byte per (neighbour, slot) | Saturating; halved every 15 min on the star-round boundary. |
| Exploration set | 4 lowest counters | Keeps escaping collided slots cheap without full reshuffles. |
| Fallback | — | No fresh anchor for the receiver → behave exactly as today. |

## 5. Known weak points (from the discussion)

1. **Counters measure only failures**; they are noisy where loss has
   other causes (fading, home deafness). The decay keeps such damage
   temporary.
2. **8 slots is a lower bound, not a comfort.** With more concurrent
   senders to one receiver than slots, residual collisions are
   unavoidable; the learning layer then only spreads them fairly.
3. **Attribution is implicit.** A sender never knows *who* it collided
   with, only that a slot is contested. This is sufficient for slot
   avoidance but says nothing about the cause.
4. **Latency, not airtime, is the currency.** A retry may wait up to one
   superframe (~4.8 s) for its slot. First attempts are deliberately
   kept out of the grid to protect the low-latency path.
5. **Anchor freshness.** A sender that has not heard the receiver's star
   round recently has a stale grid anchor; the fallback is today's
   behaviour, which is safe but not improved.

## 6. Open questions

1. Slot count sweep: is 8 enough under the `load` scenario, or is 16/32
   the sweet spot?
2. Does the grid apply per channel (three independent rasters) or
   across channels?
3. Interaction with a future demand-proportional extension (queue-depth
   driven extra slots) — deliberately deferred.
4. Deterministic per-node phase offset instead of random per-round
   jitter, if multi-hop grid alignment is ever wanted.

## 7. Measurement plan (gate before any core work)

Before any integration:

1. Extend scenario `load` with a **collision breakdown**: per traffic
   type (data / ack1 / ICU / proof) and per attempt number (first
   attempt vs. retries).
2. Only if first-attempt collisions dominate (as assumed), the grid
   helps; if retries dominate, tune §8.4 spacing instead.
3. Then implement the grid as a compile-time optional layer behind a
   flag, and A/B it against the current behaviour on `utrecht40` and the
   denser scenarios.
