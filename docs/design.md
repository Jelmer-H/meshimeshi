# Meshimeshi: Design Document

**Status:** working document. Parts of the design are validated in simulation (utrecht40 scenarios).
**Last updated:** 26 October 2026

This document describes the current design of Meshimeshi, a LoRa mesh network. It states what the protocol does and why.

Each section carries a status: **Decided** (chosen, unless simulation shows otherwise), **Proposal** (worked out, not yet settled), or **Open** (still to be decided or measured).

---

## Glossary

- **Node:** any device on the mesh. Either a **repeater** or a **companion**.
- **Repeater:** a fixed, always-on node that forwards traffic for others.
- **Companion:** a mobile end device connected to a phone over BLE. It has one fixed repeater as its home base. It listens but does not forward.
- **Home channel:** each node listens by default on one of three LoRa channels, chosen as `node_id mod 3`. Other nodes send to it on that channel.
- **Exchange:** a short, bounded interaction in which a node briefly switches to another node's home channel and then returns home.
- **Home deafness:** the time a node spends off its own home channel, during which it misses traffic addressed to it. This is the main simulation metric for channel switching.
- **Identifying transmission:** any transmission that carries the sender's node ID (star rounds, proof lists, acks, data).
- **Star:** a node's advertised entry plus the links it owns. The unit of sharing (section 6.1).
- **Star round:** the periodic broadcast of one chunk containing stars, a catalogue of digests, and requests for missing stars (section 6.1).
- **Proof list:** a short packet naming neighbours whose reverse-direction evidence is about to expire (section 5.2).
- **Confirmed symmetric / seen:** the two states of a neighbour relationship. *Seen* means one direction has been heard. *Confirmed symmetric* means the neighbour itself has proven the other direction. Routing only uses confirmed links.
- **ICU (I see you):** a reply to an identifying transmission that confirms the link in the other direction (section 5.3).
- **ack1 / ack2 / ack3:** the per-hop acknowledgement, the two-hop progress acknowledgement, and the end-to-end acknowledgement (section 8).
- **ANN / FULL:** an announcement that a large thread message exists, and the message itself (section 9.5).
- **Budget:** a counter in the header limiting how many forwards and retries a message may use (section 8.7).
- **Pending record:** the small record a forwarder keeps for a message until its next hop's outcome is known (section 8.2).
- **Bridge node:** the known node closest to a destination that is outside the constellation (section 7.1).
- **Beat:** a star's heartbeat counter (mod 256), increased once per hour by its owner (section 6.1).
- **Tick:** one minute of local time. Used for local timers only; ticks never travel on air.
- **Galaxy:** the complete network: all nodes and links, including those a given node never learns about. No node holds all of it.
- **Constellation:** a node's local knowledge: the stars it has received, with their age and confirmation (section 6.1).
- **Thread:** a conversation started by one node and limited to a geographic radius (section 9).

---

## 1. Goals and priorities

| Priority | Goal | Status |
|---|---|---|
| 1 | Reliability: messages arrive, and when a link fails a fallback route is chosen quickly | Decided |
| 2 | Scale: works nationwide (about 3000 nodes, nearly all of them repeaters) | Decided |
| 3 | Zero configuration: the user does not have to set anything up | Decided |
| 4 | Low airtime, without sacrificing reliability | Decided |
| 5 | Speed over long distances is secondary | Decided |
| 6 | New users see recent messages quickly | Decided |
| 7 | Privacy: geo is published at limited precision | Decided |

---

## 2. Radio

| Parameter | Value | Status |
|---|---|---|
| Modulation | LoRa | Decided |
| Spreading factor | SF7 as baseline. SF8 occurs in the Dutch data and is still to be examined (open point 9) | Proposal |
| Bandwidth | **62.5 kHz**. At 125 kHz, channels 100 kHz apart overlap. 62.5 kHz gives about 3 dB more sensitivity and matches the existing Dutch LoRa network on this band (869.618 MHz, 62.5 kHz, SF7/CR5). The data rate halves to about 2.7 kbps at SF7. | Decided |
| Coding rate | 4/5 | Proposal |
| Home channels | **869.44 / 869.50 / 869.56 MHz**. Spaced 60 kHz apart, they fit within the 869.4–869.65 MHz sub-band. Channel 4 (869.62 MHz) is not used, because it lies at the band edge. | Decided |
| Home channel of a node | `node_id mod 3` | Decided |
| Duty cycle | 10% airtime per channel per node | Decided |
| Collision model | Overlapping packets on the same channel collide, unless the stronger one wins by a sufficient margin (capture effect) | Proposal |

### 2.1 Channel use and switching

Each node listens continuously on its home channel. It switches channels only for an **exchange** with a partner whose packet size and timeout are known. It returns home immediately afterwards. A node waiting for something of unknown duration stays on, or returns to, its home channel and receives the reply there.

- **Direct replies** (ack1) are sent on the channel where the exchange takes place. The sender is already there.
- **ICUs** are sent on the home channel of the node that triggered them (section 5.3).
- **Star rounds** are broadcast on the sender's channel(s) without an exchange, without a switch and without an ack. Neighbours that are at home on that channel snoop the packet.
- **Proof lists** are sent on one channel at a time, only to neighbours at home on that channel.
- **Announcements (ANN)** go out on all three channels in a fixed order.
- **Unicast data:** the sender switches to the receiver's home channel, transmits, waits there for ack1, and returns. Per hop, the sender makes at most two switches (out and back). The receiver makes none.

**Spreading by RSSI.** Strong neighbours answer first and weak neighbours later. The waiting time falls with the received strength of the packet, in discrete slots. A random jitter of about 187 ms (window/16) inside each slot prevents neighbours with almost equal signal from answering at the same moment. The ranking is therefore soft: responders within about 16 dB may swap order. This was accepted, because avoiding collisions matters more.

**Carrier sense (CAD).** Before every transmission that is not a direct reply, the sender checks whether the channel is free, after switching when the radio is actually on that channel. If the channel is busy, it waits a random backoff of 0–20 ms and checks again, at most eight times. After that it transmits anyway, so a permanently busy channel can never block a node. The pool slot stays reserved while waiting, so a busy channel delays a packet but never drops it.

**Ack1 is exempt from CAD.** It is a direct reply inside an ongoing exchange. The partner is waiting on the channel, and a long wait would exceed the sender's window.

CAD avoids transmitting onto an occupied channel. It does not solve hidden-node collisions (open point 8).

**ICU gate.** A transmission leads to an ICU only when all three conditions hold (see also section 5.3):

1. **Need:** the link is *seen* rather than *confirmed symmetric*, or a courtesy ack is still pending.
2. **Quality:** the received quality is at or above `MC_SYM_QUALITY_MIN` (section 6.3).
3. **Pace and spacing:** the per-neighbour backoff allows it (section 5.3), and the RSSI slot with jitter spreads the responders.

---

## 3. Node types

| Type | Description | Status |
|---|---|---|
| Repeater | Fixed, always on, forwards traffic | Decided |
| Companion | Mobile end device. Connects to a phone over BLE and has one fixed repeater as home base. Listens continuously, does not forward. | Decided |

**Repeaters** mainly run on solar power with a small battery. WiFi is off by default and can be switched on on request or at fixed times. Continuous listening is the default, because it costs little on a solar repeater. Example: a SenseCAP Solar P1-Pro (5 W panel, 13,400 mAh) uses about 110–175 mAh over a 16-hour winter night, roughly 1% of the battery. Without sun, a repeater lasts several weeks. Battery capacity therefore does not constrain the protocol. Very small repeaters without solar power are an exception.

**Companions** listen continuously, so they receive threads in range immediately. Listening costs battery but no airtime. There is no sleep mode.

- Own messages go via the home repeater.
- After moving out of range or restarting, a companion retrieves missed threads from its home repeater's thread cache, as soon as a received message shows a predecessor it is missing.
- A companion may miss an ack2 when it has just moved out of range. It then confirms through the repeater's thread cache.
- A direct message to an unreachable companion produces a failed delivery report ("end device not reachable").

The two types differ in published geo (section 4): repeaters publish the full grid, companions coarsen it before publishing.

---

## 4. Geo encoding

| Property | Value | Status |
|---|---|---|
| Range | Worldwide | Decided |
| Latitude | 16 bits over 180°, about **306 m** per step | Decided |
| Longitude | 16 bits over 360°. At most about 612 m per step at the equator, about 376 m in the Netherlands (52° N). | Decided |
| Total size | **4 bytes**, no flag bits | Decided |
| Invalid geo | Sentinel (0, 0), which decodes to a point in the South Pole/date-line corner where no node stands | Decided |

The grid is about 306 × 376 m in the Netherlands, which is enough for routing: the bridge rule, the Gabriel graph and the radius work at kilometre scale. Coarse geo also protects privacy. Companions can coarsen their published geo further by zeroing low bits before publication. The level does not travel on air. Companions keep their precise GPS position locally for display.

**Privacy.** Geo is public, because relays need it to route. Encryption would break routing. The 306 m grid is the protection built into the protocol. Coarsening before publication is the second layer.

---

## 5. Neighbours and discovery

### 5.1 Discovery and keepalive

There is no periodic hello. Discovery and liveness are carried by three mechanisms:

- **Star rounds** make nodes visible to each other. A node that has no confirmed neighbour sends its star round in full spread (see the lonely-node rule in section 5.3).
- **ICUs** provide the first evidence that a link works in both directions (section 5.3).
- **Proof lists** keep established links alive without a per-neighbour reply (section 5.2).

**Keepalive per channel.** For each channel, a node records when it was last addressed there. Every identifying transmission on that channel counts: star rounds, acks, proof lists, data with a previous-hop field. ANN and FULL do not count, because they are anonymous.

When a channel has been quiet for `T_keepalive` (1 hour), the next proof-list moment on that channel is brought forward. An empty proof list is sent as a minimal packet (type + node ID). At the edge of the network, a proof list therefore acts as a small keepalive beacon.

**Startup.** Nodes that start simultaneously, for example after a regional power outage, could keep synchronous rounds and never hear each other. Random phases and jitter in the round timer prevent this.

### 5.2 Neighbour table and symmetry

Each node advertises only its own neighbour table. For each neighbour:

| Field | Size | Meaning |
|---|---|---|
| Node ID | 2 bytes | |
| Incoming quality | 1 byte | Measured by the node itself (RSSI/SNR) |
| Age | local only | Not transmitted. Freshness is kept locally (section 6.2). |
| Geo | 4 bytes | Optional. (0,0) means no geo. |

Each direction of a link is measured by its receiver. Asymmetry is therefore visible in the table.

**Symmetry status** is kept locally per neighbour:

- **Seen (0):** only the neighbour→us direction has been measured.
- **Confirmed symmetric (1):** the neighbour itself has proven the us→neighbour direction. This confirmation can come from four sources:
  - (a) an ICU addressed to us,
  - (b) a successful unicast exchange with ack1,
  - (c) a star round from the neighbour in which we appear,
  - (d) a proof list from the neighbour in which we appear.

An overheard ack addressed to someone else does not prove symmetry. Routing selects next hops only among confirmed neighbours.

**Long-term symmetry.** Confirmation expires. For each neighbour, a node records when the reverse evidence was last refreshed. Without maintenance, a link on which one side only ever acks would age out even though it is healthy. The **proof list** solves this:

- When the evidence for a neighbour is older than `T_sym − 2 h` (`T_sym` = 6 hours), the neighbour enters the next proof list for its channel.
- A proof list names the neighbours (on that channel, at home there) that the sender still hears. A receiver that appears in it refreshes its evidence and does not react. A receiver that does not appear in it treats the sender as snooping material only.
- Each channel has one flush moment per node per hour (rotation every 20 minutes, `slot mod 3`), so a node sends at most one proof list per channel per hour. The keepalive (section 5.1) can pull a moment forward, but the cap stands.
- A proof list costs 3 + 2n bytes (n ≤ 17 IDs). The entries bundle into one packet, so hubs send fewer packets.
- After a regional silence, many entries expire together. Every node has its own rotation and confirmation timeline, so the wave spreads itself out.

**Quality gate on confirmation.** A confirmation makes a link symmetric only when the measured quality of that reception is above `MC_SYM_QUALITY_MIN`. A link is confirmed only when both directions are above the threshold. A weak link stays *seen*: liveness and quality are stored, but routing never selects it. The proof list only names confirmed links, so weak links are never advertised to third parties as working.

### 5.3 ICU and backoff

Without a hello beacon, the **ICU** is the only confirmation path for a new or lapsed link. It replies to any identifying transmission of a neighbour: star rounds, proof lists, ack1 and ack2.

**Exceptions:**

- **Data** is answered by ack1 alone. Every received data packet produces one, and it carries the same quality byte.
- **A received ICU** does not trigger a new ack. The exchange closes through the collective proof ack (below). Acking an ack would ping-pong forever.

**Format.** A directional 6-byte ack: type, destination ID (the triggering sender), sender ID, and the quality byte the responder measured on the trigger.

**Gate.** A ICU is sent only when the ICU gate (section 2.1) is open: the need, quality and spacing conditions hold. The pace condition is a time-based backoff per neighbour:

- After each ack that does not close the three-way agreement, the next ack waits twice as long: 1, 2, 4, 8, … 1024 minutes, capped at 1440 minutes (24 hours).
- A confirmed link does not read the backoff, so no reset is needed at confirmation.
- An entry that expires (`T_expire`) starts at the bottom again. A link that really disappeared and returns recovers quickly. A permanently one-way link stays at the cap.
- After the own confirmation, two more *courtesy acks* follow at short spacing, then silence. Courtesy acks are needed because confirmation usually happens asymmetrically.

This backoff is independent of how much traffic the neighbour sends, so a busy hub cannot pull more than one ack per day from a one-way link.

**Lonely nodes.** A node with no confirmed neighbour sends its star round in full spread (all three channels), at a doubling interval: 5 minutes, then 10 minutes, and then the normal 15-minute cap. The interval resets on the first confirmation. The doubling is necessary: a node with links that never clear the quality gate would otherwise keep hammering the channels at the flat 5-minute rate.

**Proof ack.** Once per round that collected acks, the sender sends one small collective proof ack. It closes the responders' side of the agreement, and one packet confirms all responders at once. This keeps the ack airtime bounded.

**Accepted trade-offs:**

- A repaired one-way link can wait up to 24 hours for confirmation. Snooping shows *seen* and fresh quality at once, but the closure waits for the backoff.
- Acks are best-effort. An ack that is lost is not retried. The neighbour's next transmission is the next chance.
- Two mutually unknown nodes that both have confirmed neighbours elsewhere learn about each other through normal star rounds, which is fast enough for the common newcomer case.

---

## 6. Star rounds and the constellation

### 6.1 Star rounds: sharing the constellation

**The star** is the unit of sharing. A star contains:

- one **entry** for the owner: node ID, quality class per direction, beat, geo (9 bytes), and
- one **link record** for every link the owner owns: peer ID and a class byte per direction (3 bytes each), sorted by peer ID.

A star is therefore not the full picture around the owner. Every link is owned by exactly one endpoint, so every link travels in exactly one star. **A node's constellation is the union of all stars it knows.** Sorting by peer ID makes a star a canonical byte sequence.

**Link ownership** is decided by a hash over the pair. For a link {A, B}, normalise `a = min(A, B)` and `b = max(A, B)`. Compute `h = CRC-8` over the four bytes of `(a, b)`. If bit 0 of `h` is set, `a` owns the link; otherwise `b` owns it. Every node computes the owner from the pair alone, so no protocol field and no negotiation is needed. The split is fair (about half the links per node), so no node becomes the structurally large one. A simpler "lowest ID owns" rule would concentrate all links of a dense hub on one node. Ownership is fixed, so a link's advertised form always belongs to one star.

**Storage as received.** A node stores links and entries exactly as the owner advertised them, and forwards them unchanged. Its own measurements go into the neighbour table, not the link store. This makes the bytes of a star a property of its origin: serialising a star on any hop reproduces the origin's bytes exactly. The content hash therefore agrees across the network.

**Beat, fingerprint and liveness.** Three separate mechanisms keep a star alive in the network:

- **Beat.** Each star carries a beat counter (mod 256). The owner increments it once per hour, in its own star round. The beat travels unchanged through the network, so a higher beat means a newer version of that star.
- **Fingerprint (digest).** The content hash is a CRC-8 over the canonical bytes of a star, excluding the beat byte. It covers identity, links and quality classes, but not how old the advertisement is. The **catalogue digest** is `digest = (content_hash + beat) mod 256`. A receiver holding a star computes the expected digest for `beat + k` (k ≤ 8). A match confirms that the star is unchanged, without any content bytes. This is the **derivable match**.
- **Liveness.** A receiver stores, per origin, the local time at which it last saw that origin's beat advance. Repeats of the same beat do not reset this timer. Section 6.2 describes how liveness is used.

**The round.** Each node sends at most one star-round chunk per interval. The default interval is **15 minutes**. This is a hard bound: no matter how many changes happen, one node sends at most one chunk per interval (about 869 ms of airtime per 15 minutes, roughly 0.1% of a channel).

- **Rotation at rest.** When the queue fits in one chunk, the round goes out on one channel, `(home + round_counter) mod 3`. Each node cycles through its three channels. The overhead of the round is paid once per interval. Fewer transmissions also mean less home deafness and fewer collisions.
- **Full spread under backlog.** When content is left over after packing, the round goes out on all three channels. A backlog drains at the full speed of the three channels.
- **Carried bits per channel.** A star leaves the queue only after it has travelled on all three channels. A mutation therefore reaches full neighbour coverage within three intervals at rest (45 minutes per hop at the default interval), and within one interval under backlog.
- **Own star on the home channel.** The guaranteed own-star round goes out on the home channel. Its other two channels follow in the next rounds.

**Packing order.** A chunk contains three areas, in this fixed order:

1. **Content area:** complete stars in the entry layout. The receiver processes them through the normal entry path, so they build the constellation, the link store and symmetry evidence. An owner that names us confirms the link at no extra airtime.
2. **Catalogue area:** `(node ID, digest)` for each star the sender holds, 3 bytes each. The catalogue reserves the last 30 bytes of every chunk, so a content backlog can never starve the age maintenance. The catalogue starts with the own star, then the constellation.
3. **Missed area:** `(neighbour ID, count, star IDs)` requests for specific stars from direct neighbours.

The content queue is prioritised:

1. The own star (on any mutation immediately, otherwise by the guaranteed own-star round).
2. Stars whose content hash has changed since they were last carried. An age refresh or a small measurement change does not re-send a star. The catalogue is the cheap confirmation.
3. Everything else that fits, nearest hops first.

**Graph pruning.** Before a queued star is packed, the forwarder runs a breadth-first search over its own constellation from the star's centre. A forward is useful only when at least one confirmed neighbour lies farther from the centre than the forwarder itself. If every neighbour lies closer or level, forwarding would echo the star back, so it is skipped. Unknown neighbours always count as farther, so a lack of knowledge never silences a forward. The missed list catches whatever this coarse check gets wrong.

**The receiver.**

- **Content stars** use the normal entry processing with **fresh-beats-old**: a star whose advertised bytes differ from the stored bytes is accepted only if its beat is ahead of the stored beat. A differing form at an equal or older beat is a stale copy and is ignored. An identical form is always fine.
- **Catalogue entries** are checked per `(ID, digest)`:
  - **Derivable match** (k ≤ 8): the star is unchanged. The stored beat advances if the advertisement is ahead, and the age is refreshed in either case. Freshness is confirmed without a single content byte.
  - **No match, star unknown here:** a gap. The star is requested through the missed list.
  - **No match, star known here:** a real change upstream (or a hash collision). The star is requested through the missed list.
  - A star that arrived as content in the same packet is never requested. Fresh content beats the catalogue mismatch.

**Self-publication of the own entry.** The origin publishes its own entry byte as the classes of its links toward its strongest confirmed neighbour, with hysteresis (a change of at least two class steps re-publishes). The origin's catalogue digest is computed over exactly the bytes its neighbours hold.

**The missed list.** A request rides the requester's own round. The addressed neighbour resets its forward state for the star, so the star re-enters its content queue and travels with its next round. There is no exchange and no ack. Rules:

- at most 12 entries per round, no duplicates,
- only direct neighbours can be addressed,
- at most two attempts per (neighbour, star), after which fresh-beats-old and the regular rounds take over,
- entries for neighbours that have disappeared are dropped at packing time,
- an arrived star clears its requests.

**Eviction taboo.** A catalogue mismatch for a star that was deliberately evicted or rejected (section 6.2) creates **no** request. Requesting it back would start a cycle: the star arrives, displaces something else, is evicted again, and is requested again. The pool policy decides what we want to know. The missed list serves only real gaps. The taboo is local and costs no airtime. Its free-slot rules:

- **0 free slots:** the taboo is active.
- **1–3 free slots:** the taboo is treated as empty at the request gates, so rejected stars may be requested again.
- **4 or more free slots:** the taboo is cleared entirely.

**Link claims never create requests.** A star that is named only as a link peer is stored as a *presence shell*: an ID without a full form. Its bare form cannot match a catalogue digest, so a mismatch there is not a meaningful signal. The origin's own star round delivers the full form.

**Explicit requests override pruning.** When an addressee receives a request, it forwards the requested star for a few intervals even if the graph pruning would skip it. The missed list is thereby the safety net for what the coarse check gets wrong.

**Newcomer acceleration.** When a node confirms a new neighbour, it resets the forward state of all stars in its prefix. These stars re-enter the content queue and travel with the coming rounds, without waiting for requests. This costs no extra bytes, because the same stars would travel anyway. It only changes their priority.

**Constellation and round parameters (starting values, to sweep in simulation):** interval 15 min; quiet-hour floor 5 min; own-star guarantee at least once per hour; rotation with full spread under backlog; catalogue reserve 30 bytes; digest 1 byte; k-window 8; missed-list cap 12 entries, 2 attempts; publication hysteresis 2 class steps; pruning override after an explicit request 2 intervals.

**Quiet-hour acceleration.** When a node has content waiting and every channel has been quiet for about four minutes, the round interval halves, down to 5 minutes. At occupancy the 15-minute interval stands. The acceleration clears itself at rest. The hard bound still holds, because the node sends at most one chunk per round.

### 6.2 Ageing and death

**Age is local; confirmation resets it.** No age is transmitted. Every confirmation (received content, or a derivable catalogue match) sets `confirmed_at = now`. Effective age = `now − confirmed_at`, measured in minutes.

**Three independent axes:**

1. **Freshness (routing weight, continuous).** Old knowledge weighs more in routing but is never refused (section 7.1).
2. **Eviction (memory, hard).** A foreign star whose effective age reaches about 24 hours is evicted. Direct neighbours are never evicted; they live in the neighbour table. Evicting a star removes only the links it owns. Links that other origins name as peers stay until their own origins leave.
3. **Liveness (remote death detection).** A dead origin stops advancing its beat, but its last star keeps circulating and keeps matching catalogues. Freshness alone would keep it alive forever. Therefore each constellation entry stores the local time when its beat last advanced. A star whose beat has not advanced for **6 hours** is first de-weighted in routing (half the threshold) and then evicted, regardless of how fresh its age looks through echoes.

**Pool recycle when the constellation is full.** When the constellation pool (256 entries) is full and a new star arrives, the victim is chosen by routing value, not by freshness:

- **Primary: hop distance** from the own node, computed by breadth-first search over the constellation. Closer stars are kept.
- **Secondary: total link length** (sum of geo distances of the links the star takes part in). Higher is better, because removing a well-connected hub fragments the graph more than removing a leaf. The lowest total is evicted first.
- **Tie:** slot index, which keeps the behaviour deterministic.

Freshness triggers expiry, but it never decides who is recycled. At full capacity, freshness depends on the timing of echoes and is effectively random.

**Admission competition.** The arriving star is scored against the worst entry in the pool first. If the newcomer is itself the worst, it is not stored. It goes straight onto the eviction taboo, and the pool is unchanged. Only a newcomer that wins takes the victim's slot. This keeps the pool stable under pressure. It also prevents a cycle in which every arrival evicts another star and leaves half-filled shells behind.

**Death of a direct neighbour.** Direct neighbours expire after `T_expire` = 4 hours of silence. The consequences travel through the stars:

- the links the neighbour owned disappear from its own star,
- the links it did not own disappear from the owning peers' stars,
- the next rounds carry these changes to the network, within minutes for direct neighbours and in roughly hop × interval for distant ones.

No tombstone and no negative message are needed for this. Silence plus the round is the message.

**The star round is a liveness pulse.** Hellos are gone, ACK1s only exist on shared traffic, and proof lists are reactionless. Without a pulse, quiet neighbour pairs would expire on a healthy network. Every node's star round is an identifying transmission, and it keeps flowing on a quiet network. A received round therefore refreshes the sender's `last_seen`, at no extra airtime.

**Tombstones** serve one purpose: a living neighbour that was removed because the neighbour table is full (section 7.2). This is independent of the star round.

**Open: hop-scaled liveness.** The 6-hour liveness threshold is flat across the network, but the evidence differs with distance. A direct neighbour hears the origin itself, while a distant node only sees the echo trail. A threshold that depends on the hop distance could be better. The distance is already known locally from the breadth-first search (open point 19).

### 6.3 Constellation and links

- Each node's constellation is built from its direct neighbours and everything it learns from star rounds: nodes with ID, geo, quality and beat, plus the links between them. The constellation holds up to 256 nodes and a fixed number of links (section 11.2). It has no hop limit: any entry counts, wherever its origin lies.
- Links are used only if they are above the quality threshold and work in both directions.
- **Threshold:** `MC_SYM_QUALITY_MIN` = 13 (about −102 dBm). At this value, every node in the test scenario had at least one confirmed neighbour, the average number of confirmed links was 4.8, and delivery reached 98.9% under load. Lower values gave no further structural gain.
- Each direction is proven by its own receiver. Routing selects only confirmed links.

---

## 7. Routing

### 7.1 Choosing the next hop

Routing is **stateless per message**. The path is not in the header. Each hop computes its own next step over its own constellation. A route therefore repairs itself at every hop when the constellation changes. There are no route errors, route invalidations or stale paths in flight. Loop protection comes from the budget (section 8.7) and the dedup cache (section 9.3).

**Previous hop exclusion.** At the first forward choice, the hop the message came from is not a candidate. Two nodes that are each other's best first hop would otherwise bounce a message indefinitely. The exclusion applies at the first choice and in every retry plan. It does not extend to the previous hop's neighbours. Those are legitimate routes, and excluding them removed the delivery edge in dense clusters.

**Shortest route (Dijkstra).** The computation runs per message on the own constellation:

- **The first step must be a confirmed direct neighbour.** Only the neighbour table provides first steps.
- **Further edges** come from the link store. A link is used only when both directions are above the threshold and both endpoints are fresh enough. Expired entries are constellation material, not route material.
- **Cost per edge** is the expected number of transmissions, `1/p`, with `p` the per-transmission success probability. The cost is a lookup table on the quality class (scaled ×16, so cost 1.0 = 16):

  | Class | 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 |
  |---|---|---|---|---|---|---|---|---|
  | p | 0.80 | 0.55 | 0.37 | 0.25 | 0.17 | 0.11 | 0.07 | — |
  | Cost (×16) | 20 | 29 | 43 | 64 | 94 | 145 | 228 | not routable |

  The tail's coarse classes map onto the same scale. Directly measured links use the measurement. Everything beyond the own node uses the class table. These values are provisional and will be calibrated by simulation.
- **Age multiplier.** Edges are weighted by the effective age of their endpoints: below about 6 hours, the plain cost; from 6 hours, ×1.5; beyond about 12 hours (constellation material only), ×3. Stale knowledge loses to fresh knowledge inside the computation.
- **Penalties** (section 7.2) are local additions to the own edges only.

The result is one next hop. A route that is locally suboptimal on paper is accepted: age weighting already discounts stale information, and route errors would cost more airtime than they save.

**Destination outside the constellation: the bridge rule.** When the destination is not in the constellation but its geo is known, the forwarder uses the bridge rule:

1. **Find the bridge.** Among the known constellation nodes, find the node closest to the destination geo. Only nodes that have a route toward them count. Unreachable nodes stay in the constellation at low priority, because they may become connected later, but they never take part in routing. Distance comes first, route cost only breaks ties (within about 5% on squared distance). The own node is never a candidate. Without a reachable bridge, the answer is no route.
2. **Route toward the bridge.** Dijkstra toward the bridge decides the next hop. That path may start sideways or even backwards, for example around a lake or along a dam. This solves the void problems of pure greedy routing as long as there is any path material toward the bridge.

Every hop recomputes this. As soon as the destination (or a node that knows it) enters the constellation, the normal shortest-route rule takes over.

**Destination unknown** (no ID in the constellation, no geo): do not flood. Wait until the constellation contains the destination, or return a negative report.

**Selection is distance-first.** A cost-first choice among candidates kept messages on local plateaus, because it preferred a nearby local step over a candidate closer to the destination.

### 7.2 Neighbour table limit

The neighbour table holds at most 64 entries. When it is full, the weakest neighbours are removed. On hub sites, a **tombstone** blocks a removed neighbour from reappearing immediately through other tables. A tombstone exists only for this purpose.

---

## 8. Unicast

### 8.1 Forwarding with a double ack

Example: A → B → C → D.

1. A switches to B's home channel, transmits, and receives ack1 there. A then returns home.
2. B tries C, at most as described in section 8.4. If that fails, B tries D.
3. Once B has received ack1 from its own next hop, B sends **ack2** to A on A's home channel. A positive ack2 therefore means: the message has come two hops (B received it, and B's next hop acknowledged it).
4. D sends an ack3 to the source if one was requested. That is a routed message.

**One-hop case.** When the next hop is the final destination, the source closes its pending record on the destination's ack1. There is no second hop to wait for.

### 8.2 Meaning of ack2 and responsibility per hop

- **Positive:** the next hop received the message and its own next hop succeeded. Two hops of progress per ack2. Deeper progress is the responsibility of the hop that holds it.
- **Negative:** the next hop could not forward the message (its schedule is exhausted, or the message has no routable path, or it has no free pending slot). The spent budget travels back. The previous hop immediately tries its next-best alternative.

A forwarding node does not store the message. It stores a **pending record** `(message ID, previous hop, next hop, spent budget)` in a fixed buffer of 16–32 slots, first in first out. The record lives until the hop is settled: a positive ack2, a negative ack2, or the timeout at the previous hop. After that only the dedup entry remains.

**Deliberate trade-off:** after a positive ack2, the source cannot fall back to the previous hop for a later failure. Late failures reach the source through the routed failure report (section 8.6), or they never arrive when the ack3 is missing.

### 8.3 Timeouts

| Timeout | Value | Status |
|---|---|---|
| ack1 | `t_data + t_ack + processing + margin`, measured from the end of the own transmission | Decided |
| ack2 | **`T_ack2` = 8 s.** It covers the worst case of the next hop's attempt schedule (about 6 s) plus the ack2 transmission. | Decided |

- If the next hop itself fails, no ack1 arrives. The sender notices at the ack1 timeout, which is the fast signal.
- `T_ack2` is the safety net for slow failures, where the next hop received the message but its forwarding stalled. A negative ack2 normally arrives well before the timeout.

### 8.4 Retries: the attempt schedule

Each hop builds one schedule of at most six transmission attempts when it submits or forwards a message. The schedule is not rebuilt afterwards.

- **Candidates (up to three).** Candidate 1 is the best computed route (section 7.1). Candidate 2 is the best route that does not start with candidate 1. Candidate 3 is the same, excluding the first two. Each is a real computed alternative. The previous hop is excluded from the start.
- **Round-robin attempts.** One candidate gives four attempts (A, A, A, A). Two give six (A, B, A, B, A, B). Three give six (A, B, C, A, B, C). At most four attempts go to the same node, and at most six in total.
- **Waits.** Inside a pass, the next attempt goes to a different node after a fixed gap of 400 ms. At a pass boundary, the same node comes back after 800, 1200 and 1800 ms, each with ±25% jitter. The jitter prevents two nodes that collided once from retrying in lock-step.
- **Timing.** Everything completes within about 6 s in the worst case, and within 10 s with maximal CAD backoffs. Measured last-hop latency at the 95th percentile was 1.1 s.
- **Exhausted schedule.** The hop fails definitively. All its candidates go into the message's exclusion, and a negative ack2 goes to the previous hop with the spent budget. The previous hop decides with its own schedule whether another route exists.
- **Retry reception is acked.** A duplicate that arrives again gets an ack1 on the channel it arrived on, because the most likely cause is a lost ack1, not a lost data packet. The dedup cache still prevents the duplicate from being processed again.

### 8.5 Penalties and recovery

The penalty is a small counter per direct neighbour, stored next to the quality values.

- **ack1 timeout after both attempts:** penalty +1.
- **Negative ack2 from the neighbour:** no penalty. The link works; the problem lies deeper, and the neighbour applies its own penalties there.
- **Successful ack1:** penalty −1, floor 0. Traffic drives recovery. Links without traffic are probed (open point 15).
- The penalty is applied in route computation as a cost increase on the own edge, proportional to the counter.
- **Hysteresis.** Penalties only take effect from a value of 2, so one lost ack1 (for example, a collision) does not push traffic away. A penalised link is retried for routing only after at least one minute since its last failure.

### 8.6 Failed delivery report

- A node that cannot forward a message after all its attempts sends a negative report to the source.
- The report is a routed message. It uses the normal routing toward the source and the source geo from the failed message's header.
- **Payload:** message ID of the failed message (2 bytes), cause (1 byte: no ack1 after retries, no candidate, budget exhausted, destination left the constellation), optional reporter geo (4 bytes).
- It has its own budget of 16 and a flag that prevents it from triggering another report.
- A report does not improve reliability. It informs the client.

### 8.7 Budget

- One byte in the base header, starting value **32**.
- Each forward costs 1. Each retry costs 1, deducted by the sender before retransmitting.
- At zero, the node stops forwarding and reports negatively.
- Every forward reduces the budget, so loops are bounded.
- The budget is a total per message. Retries do not multiply along the path.

A shorter budget for threads is an open option (open point 20).

### 8.8 Clients and ack3

- Ack3 is a client function. Routers forward it only as routed data.
- Ack3 is sent only when requested by the flag.
- Broadcast and thread messages do not request ack3.

---

## 9. Threads and regional chat

### 9.1 Concept

- A thread is a conversation started by one node, the **starter**.
- The starter's geo is the centre. The radius is derived from local density.
- Replies stay within the radius. A relay forwards a reply only if it lies inside the radius.
- There are no fixed cells. This suits areas with very different densities.
- Thread ID = hash of starter ID and a counter or timestamp, so threads do not interfere.

### 9.2 Radius

- The radius is the distance to the k-th nearest node around the starter, with `k ≈ 15`.
- Dense areas get small radii, sparse areas large ones.
- The radius is encoded in 1 byte on a logarithmic scale, from about 500 m to about 60 km.
- The upper bound is a firmware constant, starting at 40 km, and is not carried in the packet. Relays reject radii above their firmware cap. Increases roll out in two steps: first all nodes accept the new cap, then starters use it.
- The radius is computed once per thread and carried in the header.

### 9.3 Spreading

- **Dominant pruning.** A receiver forwards only if it has neighbours the sender does not reach. It uses the sender's neighbour table, which it already has from the star rounds.
- **Waiting time and suppression.** A candidate forwarder waits a random time that gets shorter the more new neighbours it can reach. If it hears a retransmission, it recalculates its remaining coverage. If nothing remains, it stays silent. Suppression applies only to channels the node listens to. Duplicates on other channels are handled by the dedup cache.
- **Dedup.** A fixed-size FIFO cache of `(thread ID, message ID)` pairs. When full, the oldest entry is evicted.

### 9.4 Recovery and sequence

- A FULL message carries the ID of its predecessor (2 bytes, 0 for the root).
- A node that receives a FULL whose predecessor it lacks requests that predecessor from the sender. If that fails, it asks another neighbour that holds it. This repeats up to the root.
- **Tail loss.** If the last message of a thread is missed and nothing follows, the original sender repeats its ANN once after a random delay of a few minutes. Nodes that already have the FULL suppress the repeat.
- **After a missed ANN:** every forwarder re-announces its own forward on all three channels. A FULL sent on a forwarder's home channel is heard directly by nodes on that channel. A later message whose predecessor is missing triggers a unicast request to its source.
- **Newcomers** ask one neighbour for its recent threads after first contact. This is event-driven, not periodic, and it uses unicast, not flooding.

### 9.5 Announcement and large message

Large messages are sent once, to save airtime:

1. The sender transmits an ANN on all three channels in a fixed order, with a short pause between them.
2. It then sends the FULL on its own home channel.
3. A node that hears an ANN but not yet the FULL waits a short random time. If the FULL arrives in that time, it stops. Otherwise it switches to the announcer's channel and listens.
4. The fetch timeout is `T_full = T_ANN + t_full + margin`.
5. After a timeout, the node tries at most two other ANN sources. After that, sequence recovery takes over.

### 9.6 Storage and load

- Each node keeps threads within its radius in a fixed amount of memory. When memory is full, the thread with the oldest local activity counter is dropped (LRU). No clock is needed.
- A maximum number of messages per minute applies per thread, and a minimum wait applies per sender.
- Chat is best-effort. Thread messages do not request ack3.

---

## 10. Packet formats

### 10.1 Base header (data)

| Field | Size |
|---|---|
| Type + flags | 1 byte |
| Message ID | 2 bytes |
| Source ID | 2 bytes |
| Destination ID | 2 bytes |
| Previous hop ID | 2 bytes |
| Budget | 1 byte (section 8.7) |
| Next hop ID | 2 bytes |
| **Total** | **12 bytes** |

**Next hop ID.** A unicast travels on the next hop's home channel, where other confirmed neighbours hear it too. Without a next-hop field, every hearer would forward the message, and each hop would multiply it into a retransmission storm. Only the named next hop processes the packet. Other hearers use it as snooping material. Retries rewrite this field, so each hop chooses its own next hop without state.

**Optional fields** (present when the matching flag is set):

- **Source geo** (4 bytes): lets the destination reply with an ack3 or a failure report.
- **Destination geo** (4 bytes): the source attaches it whenever it is known. Every hop needs it for the bridge rule.
- **Waypoint geo** (4 bytes): defined for format compatibility. The flag is reserved for format compatibility, and no current node sets it.

**Flags.** Ack3 requested, source geo present, negative ack2, broadcast, destination geo present, waypoint present. The first four live in the type byte. A flag extension byte follows the fixed header when any geo flag or a higher flag is set.

### 10.2 Acks

| Type | Size | Direction | Channel |
|---|---|---|---|
| ack1 | 8 bytes: base identity + quality byte (the RSSI quality the acker measured on the data) | Next hop → previous hop | Same channel as the exchange |
| ack2 | 8 bytes: base identity + budget byte. The negative variant carries the **spent budget**, so the previous hop can weigh its alternatives. | Next hop → previous hop | Home channel of the previous hop |
| ack3 | Routed message, 4-byte payload | Destination → source | Via routing |
| ICU | 6 bytes: type, destination ID, sender ID, quality byte | Responder → trigger sender | Home channel of the trigger sender |
| Proof list | 3 + 2n bytes, n ≤ 17 | Node → neighbours at home on that channel | One channel, at most once per hour |

### 10.3 Announcement (ANN)

| Field | Size |
|---|---|
| Type + flags (includes the channel of the FULL, 2 bits) | 1 byte |
| Thread ID | 2 bytes |
| Message ID | 2 bytes |
| **Total** | **5 bytes** |

### 10.4 Large thread message (FULL)

| Field | Size |
|---|---|
| Type + flags | 1 byte |
| Thread ID | 2 bytes |
| Message ID | 2 bytes |
| Root geo (same encoding as section 4) | 4 bytes |
| Radius (logarithmic scale) | 1 byte |
| Predecessor message ID (0 for the root) | 2 bytes |
| Payload | n bytes |

### 10.5 Star round chunk

| Field | Size |
|---|---|
| Type + flags (catalogue, missed) | 1 byte |
| Owner ID | 2 bytes |
| Catalogue count | 1 byte |
| Content star count | 1 byte |
| Owner geo | 4 bytes |
| Firmware version | 1 byte |
| Content, catalogue and missed areas | as described in section 6.1 |

---

## 11. Hardware and implementation

### 11.1 Target platforms

| Platform | RAM | Flash | Radio | Note |
|---|---|---|---|---|
| nRF52840 (e.g. RAK4631) | 256 KB | 1 MB | SX1262 | BLE uses part of the RAM |
| ESP32-S3 | 512 KB | 4–16 MB | SX1262 | BLE and possibly WiFi |
| ESP32 (classic) | 520 KB | 4 MB or more | SX1276 | Many boards have no PSRAM |

### 11.2 Memory budget

| Component | nRF52840 | ESP32-S3 |
|---|---|---|
| Available for the protocol | 48 KB | 128 KB |
| Thread cache (LRU) | 24 KB | 64 KB |
| Dedup cache (512 entries of 4 bytes) | 2 KB | 2 KB |
| Neighbour table and constellation (256 nodes of 12 bytes) | 3 KB | 3 KB |
| Link list (1536 links of 5 bytes) | 7.5 KB | 7.5 KB |
| Free | 0 | 4 KB |
| Queues and transmit buffers | Remainder | Remainder |

### 11.3 Design rules

- **Language:** C or C++ for the firmware. Python is for simulation only. Timing (ack timeouts, CAD, channel switches) needs predictable handling.
- **One shared protocol core.** Hello-free discovery, neighbour table, acks, routing, pruning and caches live in a single C/C++ library. The simulator calls the same core through cffi or ctypes, so simulation cannot drift from the firmware.
- **Hardware layer.** Radio, timers, RNG and flash sit behind a thin interface. Only that layer changes per platform.
- **No dynamic allocation.** All buffers are fixed and bounded. Heap fragmentation on 256 KB is a real risk.
- **Integer geo arithmetic** where possible, using the 16-bit values from section 4. The east-west axis scales with cos(latitude). In the Netherlands, a fixed factor of about 154/250 (cos 52°) is sufficient.
- **Random numbers** come from the hardware RNG, for backoff and jitter.
- **Time.** Local timers count in **ticks of one minute**. Ticks never travel on air: only beats travel, so per-node drift does not matter.
- **Bounded constellation.** The number of nodes is capped, so that the graph, radius and route computations stay within the compute budget.

---

## 12. Simulation

### 12.1 Layers

1. Radio and channel: airtime, path loss, capture effect, duty cycle.
2. Neighbour table, discovery and symmetry.
3. Data, ack1, ack2, retries and budget.
4. Routing: constellation, bridge rule, failed reports.
5. Gaps and sparse areas (IJsselmeer pattern).
6. Threads: radius, pruning, ANN/FULL, recovery.
7. Churn: outages and returns.

### 12.2 Measurements per run

- Delivery ratio per message (ack3 where requested, otherwise ack2).
- Failure causes: collision, no neighbour, budget exhausted, gap.
- Airtime and duty cycle per channel.
- Budget consumption per message.
- Latency (informational).
- Duplicate work in threads: FULL transmissions per node.
- Proof lists per node per channel, and the share of empty (keepalive-only) lists.
- Home deafness per node.

**Topology.** Real repeater locations where available, otherwise synthetic layouts with the same density per region. Water is excluded, for example via OpenStreetMap.

**Radio model.** Link budget per direction: reception = 27 dBm ERP − free-space path loss at 869 MHz − 2 × clutter, where clutter(h) = 30 − 14·log10(1 + h) dB. Radio horizon d ≤ 4.12 · (√h₁ + √h₂) km. Reception succeeds above −110 dBm with fade margin. A deterministic fading offset of ±5 dB per direction makes links asymmetric, as in practice. Roughly calibrated: 20–40 m masts reach about 12–14 km, and 300–375 m masts cover a province.

### 12.3 Airtime reference (SF7, 62.5 kHz, CR4/5)

| Packet | Airtime |
|---|---|
| Symbol time | 2.048 ms |
| ANN (5 B) and ack1/ack2 (5 B) | 70 ms |
| Data (100 B) | 459 ms |
| FULL or chunk (200 B) | 869 ms |

Channel switch time: 5–10 ms (open). Duty cycle: 10% per node per channel over a 1-hour window.

### 12.4 Scenarios

1. **Link test:** two nodes, one unicast. Checks that core and simulator agree on packet sizes, airtime and ack order.
2. **Local network:** 30–50 nodes at one density, from a node list with GPS position and height. Measures ack collisions, proof lists per channel and steady-state ack load.
3. **Density limit:** 200, 500, 1000 and 3000 nodes in a synthetic Dutch distribution. Measures airtime per channel, duty peaks and delivery.
4. **Sparse area and gap:** IJsselmeer/Flevoland pattern. Radius, failed reports.
5. **Churn:** nodes drop out and return. Tombstone behaviour and recovery.
6. **Star rounds (acceptance).** Sub-scenarios 6a to 6f test the star-round design on correctness, cold start, steady state, churn, quiet-hour acceleration and density.
   - **6a. Correctness:** canonical sorting, digest derivation, beat advance, wrap at 256, missed-list behaviour, queue priorities.
   - **6b. Cold start:** convergence from empty within the backlog bound, airtime per channel, no starvation.
   - **6c. Steady state:** catalogue-dominated rounds at rest, beat propagation hop by hop, freshness of the constellation, and remote liveness after a node goes silent.
   - **6d. Churn:** mutation latency across the network, bounded missed-list load, and reboot behaviour (with and without a persisted table).
   - **6e. Quiet-hour acceleration:** the interval shortens on low occupancy with a floor of 5 minutes, and the duty share stays bounded.
   - **6f. Dense cluster:** 64 fully meshed nodes on one channel. Occupancy and convergence. See open point 1.

**Key results**

- *Churn sweep (utrecht40, 0–20% of nodes per day):* at 2–5% per day (the design band), delivery was 96–97% with about 3.3 missed requests per node per hour. At 20% per day delivery was 93.6%. Failures were schedule exhaustion, not graph loss. No hard-bound violations occurred anywhere.
- *Dense cluster (64 nodes, one channel):* the shared channel carried 7.9% occupancy, convergence within 30 minutes, constellation integrity 64/64, zero hard-bound violations.
- *Liveness (48 h run, one node silenced at 24 h):* the dead star disappeared from all 39 remaining nodes within about six hours.
- *Long rest (24 h):* about 1.0% airtime per channel, delivery 100%, confirmed symmetric neighbours about 7 per node.

**Acceptance criteria.** The star-round design is accepted if all six criteria hold:

1. Routed delivery stays at 95% or higher in every sub-scenario.
2. Airtime per channel at rest stays well below the 10% duty cycle.
3. Convergence is at least as fast: cold start within the backlog bound, and mutations network-wide within hop distance × interval.
4. No starvation: every star is forwarded within the fairness bound.
5. No storms: missed-list requests stay bounded under churn and reboot.
6. The hard bound holds in every run: never more than one chunk per node per interval.

**Parameters to sweep:** interval (15 min), beat period (1 h), k-window (8), digest width (1 byte), missed-list cap (12) and attempt limit (1–2), quiet-hour floor (5 min).

**Starting point.** Scenarios 1 and 2 need no thread logic. Section 9 becomes relevant from scenario 3 on. The first scenarios test the radio and ack choices (62.5 kHz, switching rule, `T_ack2`).

---

## 13. Open points

| # | Question | Why it matters |
|---|---|---|
| 1 | **Interval scaling with channel occupancy.** A dense single-channel cluster of 64 nodes reached 7.9% occupancy, against a 10% budget, with little margin for user traffic. Doubling the interval to 30 minutes at about 64 neighbours on a channel would bring it to about 4%. To be calibrated. | Density limit |
| 2 | Radius: bounds and bootstrap while the constellation is small. The upper bound (40 km) needs testing. | Thread size |
| 3 | Waiting-time slots and the timing of repeated ANNs | Duplicate work and collisions |
| 4 | Budget 32: enough, or too generous? | Worst-case traffic |
| 5 | Companion precision: default coarsening level | Privacy and accuracy |
| 6 | Star round tuning: `T_sym` (6 h), the flush margin (2 h), and `T_keepalive : T_expire = 1 : 4` (1 h / 4 h). To test in the churn and long-run scenarios. | Route stability versus airtime |
| 7 | Chunk size (about 200 bytes) and fragmentation | Packet design |
| 8 | Hidden-node collisions. CAD reduces losses by about 19% in a traffic test, but cannot see nodes outside the partner's range. A real RTS/CTS handshake stays on the shelf until hidden-node loss proves heavy. | Only if hidden-node loss is high |
| 9 | SF8 nodes in the Dutch data: range and airtime | Radio model |
| 10 | SF8 fallback for remote nodes (estimated 25–30% more range at about twice the airtime). A receiver listens on one SF, so the choice applies per link or per region. | Radio design |
| 11 | Companion battery capacity and listening profile (charging source, battery level) | Companion behaviour |
| 12 | Companion design: BLE protocol to the phone, coupling to the home repeater, behaviour out of range. Companions have no node ID. Addressing uses the home repeater and a slot number. | Companion is largely unspecified |
| 13 | Node ID as geo tag plus random suffix. Disadvantage: the ID changes when the node moves, and location is always in the packet. | Neighbour table size |
| 14 | Local maintenance during quiet periods: recalculating the Gabriel graph, route recomputation, tombstone cleanup, probes to penalised links, and SF8 measurements. Floods, broad pulls and changes to the keepalive are not allowed. | Maintenance without extra network load |
| 15 | Link pressure: carry the pending-slot occupancy in ack1 and apply a short-lived cost penalty to heavily loaded hops, so routes steer around them before drops happen. | Load-aware routing |
| 16 | ICUs under load: ack collisions on ANN/FULL triggers and the ranking spread there | Resting traffic |
| 17 | Hop-scaled liveness threshold (section 6.2) | Detection latency versus route flapping |
| 18 | Death wave per hop band: measure the extinction of a dying star near and far from the origin | Liveness calibration |
| 19 | Hop distance is available locally from the breadth-first search. Its use in the liveness threshold is open (section 6.2). | Liveness calibration |
| 20 | Shorter budget for threads, with the effect on delivery and cost | Budget and thread size |

---

## 14. Alternatives considered and rejected

These approaches were considered and set aside. They are listed so the reasoning is not lost.

| Idea | Reason |
|---|---|
| Cost-first choice among candidates (15% candidate band) | Kept messages on local plateaus, because it preferred a nearby local step over a candidate closer to the destination |
| Gossip routing as the main route | Link-state over a constellation is used instead |
| Distance-vector routing | The constellation gives each node a full picture of the links instead |
| Fixed geographic cells for chat | Density varies too much |
| Shared chat channels (#public, #test) | Broadcast storm |
| One channel per chat region | Threads cover this without a channel per region |
| Flooding as fallback | Does not scale |
| A path in the packet header | Costs more than it gives |
| Encrypted geo | Relays need it to route |
| Repeater storage for companions | Companions retrieve threads from their repeater's cache |
| Repeater WiFi on by default | Off by default, because of power consumption on solar |
| Full neighbour list in the keepalive | Airtime is superlinear in packet size, and routing needs quality, geo and direction |
| Flag bit "confirm this" with a reply rule | The proof list gives the same symmetry evidence without replies or ack chances |
| Hello beacon with startup rounds | The star round for lonely nodes and the ICU discover neighbours without hellos |
| Delta chunks and a periodic table pull | The star round carries mutations as content changes of the own star, and the catalogue covers ageing |
| Negative delta messages for dead neighbours | Silence plus the star round is enough |
| Waypoints for routing | Distance-first bridge rule navigates without them |
| Perimeter routing for unicast | The bridge rule routes through the graph itself |
