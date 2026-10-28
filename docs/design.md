# Meshimeshi: Design Document (revised)

**Status:** working document, simulated (building blocks 1 and 2 on utrecht40)
**Language:** English
**Last updated:** 26 October 2026 (4.3 rotation implemented: one channel per interval at rest, full spread under backlog, carried bits per channel, own-star guarantee on the home-channel round — measured utrecht40: rest 24 h 1.0% per channel (was 2.3%), home deafness halved, delivery 100%, map integrity 40/40; scenario 6 continues)

This document records the design decisions discussed so far. Each item carries a status:

- **Decided:** chosen, unless the simulation shows otherwise.
- **Proposal:** worked out, not yet settled.
- **Open:** still to be decided or measured.
- **Later:** deliberately postponed.

---

## Reading guide and glossary

This document uses a number of terms that build on each other. Read this glossary first. Each entry says where the concept is explained in full. The sections are not in strict dependency order, so use the glossary and the cross-references when a term is unfamiliar.

### Network and nodes

- **Node:** any device on the mesh. There are two kinds: repeaters and companions.
- **Repeater:** a fixed node that is always on and forwards messages for others. (Section 3.)
- **Companion:** a mobile end device, connected to a phone via BLE, with one fixed repeater as its home base. It listens but does not forward. (Section 3.)
- **Home channel:** each node has one of three LoRa channels (`node_id mod 3`) that it listens on by default. Other nodes send to it when they want to reach it. (Section 2.)
- **Exchange:** a short, bounded interaction in which a node briefly switches to another node's home channel, with a known packet size and timeout, and then returns home. (Section 2.)
- **Duty cycle:** the share of airtime a node may use per channel (10%). (Section 2.)
- **Home deafness:** the time a node spends off its own home channel, during which it misses traffic addressed to it there. This is the main simulation metric for channel switching. (Section 2.)

### Messages and acknowledgements

- **Hello:** a 3-byte beacon that announces a node's presence: type (1), node ID (2). (Section 4.1.)
- **Keepalive:** the role of the proof list (4.2): when a channel has been quiet for a while, the next scheduled proof list on that channel is the keepalive. A separate keepalive hello does not exist. (Sections 4.1, 4.2.)
- **Hello ack:** a reply to a hello (or to other identifying traffic) that confirms the link in the other direction. (Sections 2, 4.2.)
- **ack1:** confirms that the next hop received a data packet. Sent on the same channel as the exchange. (Section 7.)
- **ack2:** sent back to the previous hop once the next hop has received the message (positive) or failed to forward it (negative). (Section 7.2.)
- **ack3:** an end-to-end confirmation from destination to source, sent only when requested. (Sections 6.2, 11.)
- **ANN (announcement):** a short 5-byte message that tells the network a large thread message is available. (Section 8.5.)
- **FULL:** the large thread message itself, which is fetched after an ANN. (Sections 6.5, 8.5.)
- **Courtesy ack:** after a node has confirmed a link, it sends two more hello acks so that the other side can complete its own evidence, then stays silent. (Section 4.1.)
- **Star round:** the uniform convergence cadence (4.3): one chunk per interval carrying content stars, a catalogue of digests and missed-star requests. The star is the unit of sharing: one node entry plus the links it owns. (Section 4.3.)
- **Proof list:** a small packet with the node IDs of neighbours whose reverse-direction evidence expires soon. (Section 4.2.)
- **Failed delivery report:** a routed message sent back to the source when a message could not be forwarded after all retries. (Section 10.5.)
- **Budget:** a counter in the header that limits how many forwards and retries a message may use. (Section 9.)
- **Pending record:** the small record a forwarding node keeps for a message until its ack2 window closes. (Section 7.2.)
- **Dedup cache:** a cache of recently seen message IDs, used to discard duplicates. (Section 8.3.)

### Neighbours, links and routing

- **Neighbor table:** the list of direct neighbours a node advertises, with quality and age for each. (Section 4.2.)
- **Local table counter (not on air):** an internal counter of table mutations. Under the star round (4.3) it marks the own star as changed; nothing of it is transmitted — freshness is purely local (confirmation resets it, 4.4). (Section 4.3.)
- **Link:** a radio connection between two nodes. Each direction is measured separately. (Section 4.2.)
- **"Seen" versus "confirmed symmetric":** the two states of a neighbour relationship. "Seen" means one direction has been heard. "Confirmed symmetric" means the other direction has been proven by the neighbour itself. Routing uses only confirmed links. (Sections 4.2, 10.1.)
- **Ack eligibility:** the three conditions (need, pace, spacing) that decide whether a received transmission leads to an ack at all. (Section 2.)
- **Known graph (map):** the network picture a node builds from its own direct neighbours and everything that travels along in the star rounds (4.3): nodes with ID, geo, quality and heartbeat tick, plus the links between them. Bounded at 256 nodes and a fixed link store. No longer a 2-hop horizon: entries arrive from every chunk, whatever the hop distance of their source. (Section 10.1.)
- **Neighbour table limit:** the maximum size of a node's neighbor table (64). When it is full, the weakest neighbours are removed. (Section 4.1, 10.4.)
- **Tombstone:** a short-lived block on a neighbour that was removed because of the neighbour table limit, so that it does not reappear immediately through other tables. (Section 4.4.)
- **RSSI / SNR:** received signal strength and signal-to-noise ratio, the radio measurements used to rank links and responders. (Sections 2, 4.2.)
- **ETX-like quality:** a link cost based on the expected number of transmissions needed. Used to pick the best route. (Section 10.2.)

### Geography

- **Geo:** a 4-byte position (latitude and longitude at a deliberately coarse 306 m grid). (Section 5.)
- **Bridge node:** the graph node closest to a destination that lies outside the own graph; the intermediate target of the geo+Dijkstra rule. All nodes within 15% of its distance join as candidates. (Section 10.2.)
- **Gabriel graph:** a way of deciding which links are needed to connect nodes around a gap, based on geometry. (Section 10.3.)
- **Perimeter routing (GPSR):** lapsed for unicast — the bridge-node rule routes around gaps through the graph itself. (Section 10.3.)
- **kNN radius:** the radius of a thread, set by the distance to the k-th nearest node around the starter. (Section 8.2.)
- **Waypoint:** an intermediate geographic point that a source attaches to steer a message; it travels as an optional geo in the data header (6.1) together with the destination geo, and replaces the dest geo in the bridge-node rule until a hop passes it. (Section 10.2.)

### Threads

- **Thread:** a conversation started by one node, limited to a geographic radius around the starter. (Section 8.1.)
- **Starter:** the node that starts a thread and thereby sets its area. (Section 8.1.)
- **Predecessor:** the previous message in a thread, referenced by ID so that gaps can be detected. (Section 8.4.)

### Timers and parameters

- **`T_ack2`:** how long a node keeps a pending record and how long a sender waits for ack2. (Section 7.3.)
- **`T_full`:** the timeout for fetching a FULL after an ANN. (Section 8.5.)
- **`T_keepalive`:** how long a channel may be quiet before the next proof list on that channel is sent as keepalive (1 hour). (Sections 4.1, 4.2.)
- **`T_sym`:** how long a confirmed symmetry remains valid before it must be refreshed (6 hours). (Section 4.2.)
- **`T_expire`:** how long a neighbour may be silent before it is considered dead (4 hours). (Section 4.4.)
- **`k`:** the number of nearest nodes used to determine a thread radius (about 15). (Section 8.2.)

### Other

- **CAD:** channel activity detection, a LoRa feature that tells whether a channel is busy without decoding a packet. (Section 12, point 15.)
- **Capture effect:** when two packets overlap, the stronger one may still be received if it is sufficiently stronger. (Section 14.3.)
- **BLE:** Bluetooth Low Energy, the link between a companion and its phone. (Section 3.)

---

## 1. Goals and priorities

| Priority | Goal | Status |
|---|---|---|
| 1 | Reliability: messages arrive, and when a link fails a fallback route is chosen quickly | Decided |
| 2 | Scale: works nationwide (about 3000 nodes, nearly 3000 of them repeaters) | Decided |
| 3 | Zero config: the user does not have to set anything up | Decided |
| 4 | Low airtime, without sacrificing reliability | Decided |
| 5 | Speed over long distances is secondary | Decided |
| 6 | New users see recent messages quickly | Decided |
| 7 | Privacy: geo published at limited precision | Decided |

---

## 2. Radio

| Parameter | Value | Status |
|---|---|---|
| Modulation | LoRa | Decided |
| Spreading factor | SF7 as the baseline. SF8 also occurs in the Dutch data and must be examined in simulation | Proposal |
| Bandwidth | **62.5 kHz** (Oct 2026). Reasons: (a) at 125 kHz, channels 100 kHz apart overlap, and the 250 kHz sub-band fits only two clean channels; (b) 3 dB extra sensitivity, which means more range, smaller gaps and fewer retries; (c) the same settings as the existing Dutch LoRa network on this band (869.618 MHz, 62.5 kHz, SF7/CR5). Cost: the data rate halves (about 2.7 kbps at SF7). All timeouts and slot parameters are calculated for this. | Decided |
| Coding rate | 4/5 | Proposal |
| Home channels | **869.44 / 869.50 / 869.56 MHz** (Oct 2026). At 62.5 kHz, three channels spaced 60 kHz apart fit comfortably within the 869.4–869.65 MHz sub-band. The old plan (869.45/55/65 at 125 kHz) was dropped because those channels overlapped. Channel 4 (±869.62 MHz) was considered but rejected because of the band edge; it may be revisited later. | Decided |
| Home channel of a node | `node_id mod 3` | Decided |
| Duty cycle | 10% per channel | Decided |
| Collision model | Two overlapping packets on the same channel collide, unless the stronger one wins with sufficient margin (capture effect) | Proposal |

### Channel use and the switching rule

Each node listens continuously on its own home channel. It switches channels only for an **exchange** with a known partner (known packet size and a short, known timeout), and then returns to its home channel immediately. If a node is waiting for something of unknown or long duration, it stays on, or returns to, its own home channel; the reply arrives there by itself.

- **Direct replies on the same channel:** ack1 for data is awaited on the channel where the exchange takes place. The sender is already there, and the receiver is at home or has already switched.
- **Hello acks:** these go to the home channel of the hello sender. After a hello, several neighbours respond, and their replies are spread out over time. The keepalive hello no longer exists (4.1): its role is carried by the reactionless proof list (4.2), so there is no ack channel question for a keepalive (open point 18 dropped).
- The sender transmits the hello on all three channels in a fixed order and then returns to its home channel. Responders reply there, within the maximum response time. That response time is also the sender's receive window.
- **Spreading by RSSI (decided):** strong neighbours respond first, weak neighbours later. The waiting time falls as the received strength of the hello rises, in discrete slots. RSSI determines the slot; a random jitter determines the position within the slot. Without jitter, neighbours with almost identical RSSI get the same waiting time and collide. Strong neighbours provide the most valuable link-quality information, so their acks arrive first.
- **Jitter above the ack airtime (decided, Oct 2026):** the jitter within a slot is window/16 ≈ 187 ms. That is well above the airtime of an ack (70 ms) plus the switch. The old window/64 (≈ 47 ms) let acks of equal rank collide; this was demonstrated in simulation. Now acks collide only by chance. The cost: the ranking becomes "soft". Responders that differ by less than about 16 dB may swap order. This was accepted, because avoiding collisions is the most important goal.
- **Unicast data:** the sender switches to the home channel of the receiver, transmits, waits there for ack1, and returns. Per hop the sender makes at most two switches (out and back), and the receiver makes none.
- **CAD, channel activity detection (decided, Oct 2026):** before every transmission that is not a direct reply, the sender asks the radio whether the channel is free (CAD/carrier sense) — after the switch, when the radio is actually on the channel. Busy means: wait on the channel with a short random backoff (0–20 ms) and ask again, at most 8 times; after that the transmission goes anyway, so a permanently busy channel can never block a node (hello round, proof list). The pool slot stays reserved during the wait, so a busy channel delays but never drops the packet. **Ack1 is exempt** (7.1): it is a direct reply within the ongoing exchange, the partner is waiting there, and its timing window is short — asking again would risk exactly the collision it is meant to prevent, and would delay the answer beyond the sender's window. The platform layer provides CAD via a callback; without one (stub platform) the core transmits as before. CAD is *not* an RTS/CTS system against hidden nodes (open point 8): it only avoids transmissions onto an occupied channel; a disturbing node beyond the partner's range stays invisible to carrier sense.
- **Ack2:** the next hop switches to the home channel of the previous hop, transmits, and returns directly. One exchange.
- **Table chunks (push, 4.3):** broadcast on the own home channel, in the fixed cadence. No exchange, no switch, no ack; neighbours snoop along.
- **Proof list (4.2):** a small packet with the node IDs of neighbours whose reverse-direction evidence expires soon. Only the sub-list of neighbours that are at home on that channel; each channel has one flush moment per node per hour (20-minute rotation), so at most one proof-list packet per channel per hour. The packet is an own call on that channel and resets its keepalive counter (4.1). A receiver that appears in it refreshes its evidence and does not react; a receiver that does not appear in it processes the sender the way snooping does (liveness, quality, "seen").
- **Announcements (ANN):** on all three channels in a fixed order. A FULL fetch is an exchange (`T_full`), so the fetcher switches to the channel of the announcer and returns.
- **Shared chat channels** (#public, #test) are dropped.
- **Simulation metric:** the time per node spent outside its own home channel, the "home deafness". Every moment outside its own channel is missed reception on its own channel.
- **Ack eligibility (decided, Oct 2026):** a transmission leads to an ack only when three independent conditions are met:
  1. **Need:** the link is at "seen" rather than "confirmed symmetric", or a courtesy ack is still pending. Stale reverse evidence is not an ack reason: that is maintained by the proof list (4.2). At rest this condition is false, which limits the number of acks even when a neighbour transmits a lot.
  2. **Pace:** the counter per neighbour (section 4.1) and the cooldown after an own ack.
  3. **Spacing:** the RSSI-based ranking with jitter (see above), so that acks from several responders do not collide.

---

## 3. Node types

| Type | Description | Status |
|---|---|---|
| Repeater | Fixed node, always on, forwards traffic | Decided |
| Companion | Mobile end device. Connects via BLE to the phone and has one fixed repeater as its home base. Listens continuously and does not forward. | Decided |

**Repeater.** Mainly runs on solar power with a small battery. WiFi is off by default and can be switched on on request or at fixed times. No memory is reserved for companions.

**Energy (repeaters, decided).** Continuous listening is not a problem for solar repeaters and is therefore the default. Example calculation (SenseCAP Solar P1-Pro: 5 W panel, 4 × 3350 mAh = 13,400 mAh): the SX1262 draws about 4.6 mA in receive, and the whole system in listening mode about 7–11 mA. A 16-hour winter night costs 110–175 mAh, about 1% of the battery. Without sun, a repeater therefore lasts several weeks. Conclusion: battery capacity does not constrain the protocol. For very small repeaters without solar power there is still a lower limit.

**Companion.** Listens continuously, so that it receives threads in range immediately. Listening costs battery but no airtime, so it does not burden the mesh. There is no sleep mode. After being out of range, or after a restart, the companion retrieves missed threads from its home repeater's thread cache, as soon as a received message shows a predecessor it is missing. No extra state is needed for this.

- Own messages go via the home repeater.
- A companion may miss an ack2 if it has just moved out of range. It then confirms via the repeater's thread cache.
- Direct messages to a companion that is out of range produce a failed delivery report with the message "end device not reachable".

The distinction between the two types follows from the published geo (section 5): repeaters publish the full grid, while companions coarsen it before publishing.

---

## 4. Hello and neighbor table

### 4.1 Hello

- **Size:** 3 bytes. Contents: type (1), node ID (2). Dropped (with it the state version): the version-triggered table fetch no longer exists — the star round (4.3) carries mutations unsolicited; freshness is local: every confirmation resets it (4.4), and liveness travels as the star's heartbeat tick (4.3). A geo change no longer needs a version increment; the geo travels in the table chunks.
- **Frequency (startup phase, adaptive):** at startup every 30 s, then doubling per round (30 → 60 → 120 → 240 → 480 → 960 → 1920 s). At the 1-hour interval the round stops; after that the hello no longer exists (the keepalive role goes to the proof list, below). Nodes that have been present longer therefore disturb less and less during startup.
- **Random phase and jitter (decided, coupling test Oct 2026):** each hello is scheduled with a random delay. Without jitter, nodes that start at the same time (for example after a power outage in a region) keep synchronous rounds, and never hear each other, because during their own round they are on a different channel. The solution now lives in the protocol core.
- **Per-channel keepalive via the proof list (decided: replaces the keepalive hello):** the keepalive hello is dropped. Its role goes to the proof list (4.2): every identifying transmission on a channel refreshes that channel's liveness, and when a channel has been quiet for `T_keepalive`, the next proof list on that channel follows immediately, even if the flush margin has not yet expired. The proof list is reactionless by design (4.2), so the ack channel question of the keepalive hello (old open point 18) lapses entirely.
  - For each channel c, a node records when it was last "addressed" there. Every identifying transmission on c counts (hello, hello-ack, ack1, ack2, data as `prev_hop`, table chunks). ANN and FULL do not count, because they are anonymous.
  - When the timer has expired (`T_keepalive`, starting value 1 hour), the next proof-list moment on c is pulled forward: the node sends its proof list for channel c, even if no entry has reached the flush margin. An empty sub-list becomes a minimal proof-list packet (type + node ID) — in the edge of the network the proof list thereby degenerates into a mini-hello.
  - A proof list on c is itself an own call on c and resets the keepalive timer of c (4.2).
  - During the startup phase the round across all three channels still applies (30 s interval). After that the hello disappears entirely; channel liveness rests on traffic plus the proof list.
  - Benefit: nationwide, the hello is the largest load. With 3000 nodes and a 1-hour interval, that is about 5.8% per channel, and the hello acks come on top of that. The proof list bundles the keepalive with work that has to happen anyway: the airtime for table maintenance (4.2) is reused, and on a channel that is quiet only for the keepalive the hello and its acks disappear completely. Fewer switches: a node whose stale neighbours are on its own home channel alone sends without any switch.
  - Newcomers do not suffer from this: their startup hellos provide discovery, and their first table push (4.3) provides both sides their symmetry evidence.
  - Risks: discovery between two nodes that are both still unknown is postponed until the keepalive moment of the proof list.
  - To measure: proof-list packets per hour per node per channel, airtime per channel, and discovery latency of newcomers. `T_keepalive` is a parameter (starting value 1 hour, linked to `T_expire`, 4.4); it is now the lower bound on the proof-list cadence per channel. History: first measurement after implementation of the keepalive hello (local-net, 40 nodes, 8 h): stable hello airtime 0.09% → 0.06% per channel, missed hellos 4690 → 3582, confirmed symmetric on average 6.9 → 8.4. Measurement after the proof list took over the keepalive (local-net, utrecht40, 40 nodes, same topology, old vs. new core): 8 h — stable hello 0.06% → 0.01% per channel, stable ack 0.11–0.13% → 0.01–0.03%, collisions 14,105 → 12,171, missed hellos 3,582 → 2,588, confirmed symmetric 8.4 → 9.1, duty max 0.4 → 0.1 s per hour; 24 h — stable hello 0.00% and ack 0.00–0.01% per channel, confirmed symmetric 9.1 (unchanged over three full `T_sym` cycles), 0 nodes without a confirmed neighbour, duty max 0.1 s. Conclusion: the resting network is carried by the reactionless proof list alone; the evidence density rises slightly instead of falling.
- **Hello ack on demand (proposal, Oct 2026; minimal form implemented):** unconfirmed links stay stored, but the chance of an ack decreases over time. Every identifying transmission from the neighbour increments a counter (hellos and home-channel transmissions, section 2; retries count as one). Only counter values 1, 2, 4, 8 and 16 produce an ack attempt. After that, on average one ack per 16 transmissions still occurs, with a minimum time threshold per neighbour. This means that a permanently asymmetric link on a busy hub does not cause unbounded ack traffic.
  - **Quality gate (decided, Oct 2026):** a hello below the quality threshold (10.1, `MC_SYM_QUALITY_MIN`) gets no ack at all, from the first transmission on. The link can never become a routing link (both confirmations would be blocked by the same gate), so an ack on it is pure airtime — and around boundary links it feeds exactly the ack storm this scheme prevents. The sender still lands in the table ("seen", liveness and quality stored); the counter does not count a weak hello: it is no evidence of a usable link.
  - After our own confirmation, two more "courtesy acks" follow, and then definitive silence. Snooping (section 2) keeps liveness and quality in order by then.
  - Courtesy acks are needed because confirmation almost always happens asymmetrically: when one direction succeeds first, those acks stop, while the neighbour still has to retrieve its evidence from our ack.
  - The counter resets on confirmation and on renewed contact. Whether a transmission leads to an ack is determined by ack eligibility (section 2).
  - Not yet implemented: the cooldown per neighbour and the minimum time threshold on the drop.
  - Dropped (Oct 2026): acks on an own version change, and the reset on `T_stale`. These have been replaced by evidence ageing with `T_sym` (4.2). Dropped (now): the state version itself — no mechanism pulls a table anymore (4.3), so nothing on air needs a version.
  - Motivation (40-node scenario): tower sites trigger an ack storm on their home channel every round. With the scheme, the steady-state ack load falls from 0.31–0.37% to 0.12–0.17% per channel. Collisions fall from 25,435 to 15,734 (−38%), and missed hellos from 5,296 to 4,690. The density of evidence actually rises (average confirmed from 6.2 to 6.9), because the remaining attempts go through more often.
  - Hub sites see hundreds of neighbours. There the neighbour table limit of 64 acts as a brake, provided it is stable (top-N on strength with hysteresis, 10.4).
- A new node quickly gets a map thanks to the short startup intervals.
- **Scope (decided): the hello exists only during the startup phase.** It is a discovery-and-evidence mechanism for nodes that do not yet appear in anyone's table: the round over all three channels reaches neighbours whose home channel is unknown, and the hello acks addressed to the newcomer are its only self-evidence before it has traffic of its own. From the moment the adaptive interval reaches 1 hour, the hello no longer exists: the resting traffic is carried by data and its acks, the table push (4.3), the proof list with the keepalive role (4.2), and the generalised hello ack on home-channel transmissions (section 2, open point 16). Two nodes that start simultaneously (regional power outage) still find each other via the random phase and jitter of the startup rounds; no other mechanism covers that case.
- Thread information is not included in the hello (see 8.4).

**Status:** decided, except for the exact intervals (open).

### 4.2 Neighbor table

Each node advertises only its own neighbor table. For each neighbour:

| Field | Size | Explanation |
|---|---|---|
| Node ID | 2 bytes | |
| Incoming quality | 1 byte | Measured by the node itself (RSSI/SNR, averaged over hellos and hello acks). An overheard ack is also a valid measurement of the incoming link. |
| Age | 1 byte | Local bookkeeping in the neighbour entry; 255 = expired (4.4). Age itself no longer travels on air — the star's tick byte is liveness, freshness is local confirmation (4.4). |
| Geo | 4 bytes | Optional. (0,0) means: no geo (section 5). |

Each direction of a link is measured by the receiver of that direction. Asymmetry is therefore visible.

**Symmetry status (decided, Oct 2026).** Besides the advertised fields, each node keeps track locally, per neighbour, of whether symmetric traffic has been proven:

- **0 = seen:** only the neighbour→us direction has been measured.
- **1 = confirmed symmetric.**

Confirmation comes exclusively from the neighbour itself:

- (a) a hello ack addressed to us, the evidence channel of the hello process. This is available before the first unicast.
- (b) a successful unicast exchange with ack1.
- (c) a table chunk from the neighbour in which we appear (4.3), with its own measurement of our signal.
- (d) a proof list of the neighbour in which we appear (below).

An overheard hello ack addressed to someone else does not prove symmetry. Routing chooses the next hop only via confirmed symmetric neighbours (10.1). Direct replies in exchanges are always allowed (4.3). Advertising this status in the neighbor table happens via the table chunks (4.3): a chunk containing our entry is itself already the reverse-direction evidence.

**Long-term symmetry (proposal, Oct 2026).** The confirmed status is not valid forever. For each neighbour, a node records how old the reverse-direction evidence is ("last proven"). It is refreshed by a hello ack addressed to us, a successful unicast with ack1, a table chunk from the neighbour in which we appear (4.3), or by appearing in its proof list.

Without maintenance there is a deadlock: the neighbour that acks first again (the drop, 4.1) only refreshes its receiver. The sender learns nothing from its own transmission, so its evidence ages indefinitely, even though the link is healthy.

The solution is the **proof list**: a small packet with the node IDs of neighbours whose reverse-direction evidence expires soon. The working principle: evidence about the direction neighbour→us can only come from US, because we are the only source of their unmeasurable direction, and vice versa. When a node publishes "I hear X", then at X, in a single reception, the direction X→node is confirmed (the claim of the node itself) and the direction node→X is freshly measured (the reception itself). No reply is needed: each side maintains the other via its own lists, independently and symmetrically.

- **Trigger.** For each neighbour, the node records when it last confirmed it (hello ack addressed to it, ack1 on its data, a table chunk in which it appeared, or an earlier list) — that is the own transmission log, so exactly locally computable. When the confirmation age exceeds `T_sym − 2 h`, the neighbour enters the list. The margin gives every link two more flush moments before expiry; at `T_sym` the link degrades to "seen", and routing (10.1) then no longer selects it.
- **Combined trigger with the keepalive (4.1).** The proof list also carries the per-channel keepalive: when a channel has been quiet for `T_keepalive`, the next proof-list moment on that channel is pulled forward, even if no entry has reached the flush margin. In a quiet network the two triggers coincide — one packet instead of a keepalive hello plus a list. In an active network the channel stays fresh from traffic and only the `T_sym` trigger remains. A missing self-entry in a received table chunk of a direct neighbour is deliberately not treated as negative evidence: chunks have no ack, so absence is no proof, and nothing breaks by doing nothing. Optimisation (a downgrade rule with coverage over at least two prefix rounds of the neighbour) can follow later if scenario measurements show damage from asymmetric links that stay routed; without a state version on air, such coverage would have to come from a monotone chunk counter or reception timestamps, which do not exist on air today.
- **Cadence and cap.** Flush check every 20 minutes, rotating per channel (`slot mod 3`). Each channel thereby has one flush moment per hour: at most one proof-list packet per channel per hour per node, the same frequency as the old keepalive hello (4.1). A node's three channels lie 20 minutes apart, so no self-congestion. The keepalive trigger (4.1) can pull a moment forward, but the cap of one packet per channel per hour stands.
- **Channel.** Only the sub-list of neighbours that are at home on that channel; every receiver listens at home. The packet is an own call on that channel and resets the keepalive counter (4.1). An empty sub-list (no confirmed neighbours at home on that channel, nothing to flush) is sent as a minimal packet with type + node ID — the keepalive beacon for that channel (4.1).
- **Reactionless.** Appearing in the list refreshes the evidence; not appearing in the list is snooping ("seen", liveness, quality). No reception triggers a reply; the ack-on-request regression stays cut off, without flag semantics or a reply rule.
- **Cost.** Per link per `T_sym` period two list entries, one per side — with loose entries as much air as the earlier flag-bit solution (2 × 78 ms), but the entries bundle (2 through 17 IDs in one packet of the 82–165 ms class), the schedule plans itself (no dependence on ack chances) and on hubs the packet count drops. Almost only in a quiet network, because with traffic, ack1s and table chunks usually refresh the evidence sooner.
- **Mass events.** After regional silence, many proofs expire in sync. The wave spreads itself out: every node has its own rotation and its own confirmation ages (the last traffic per link, no shared clock), and the little lists bundle what expires simultaneously.
- **Parameters.** `T_sym` starting value 6 hours (to be tested in scenario 5); flush margin 2 hours; cadence 20 minutes (rotation). First long-run measurement (local-net, utrecht40, 24 h, combined keepalive trigger implemented): confirmed symmetric per node 9.1, unchanged over three full `T_sym` cycles; no node lost its evidence; resting load per channel hello 0.00% and ack 0.00–0.01%. Note: the rotation is staggered per node but not correlated per region; lost receptions rose slightly with duration (12,171 at 8 h → 14,204 at 24 h), so in a nationwide rollout the flush moments may concentrate on a dense channel — to keep an eye on in scenario 5.

**Status:** symmetry status (seen/confirmed) decided; proof list proposal, to be tested in scenario 5.

### 4.3 Star rounds: table sharing as a uniform cadence

**Status: decided in design (Oct 2026), replacing the chunk-push cadance of the old 4.3/4.4. The core implementation is complete and tested at unit level, including the four design additions of the convergence review (Oct 2026): link ownership (per-link hash bit), storage-as-received (advertised forms stored and forwarded untouched, local measurements confined to the neighbor table), newcomer acceleration (forward-state reset of the prefix stars on neighbour confirmation) and the hash over the origin's advertised form. The measured 6a cycle (2481 miss requests / 6 h) is gone at the source: the star-line scenario now converges — the residual requests are a startup transient in the first hour (forms settle after the first measurements), steady state is quiet. The old chunk-push path is fully removed from the code (delta chunks, bundling windows, periodic prefix re-share, tail cursor, F_NEG — the star round is the only cadence, started by `mimeshi_start`). The rotation with the backlog override (Oct 2026) brought the resting airtime under the old cadence's level: measured utrecht40 24 h rest — 853-884 s per channel (1.0%) vs 746-749 s (0.9%) on main, with delivery 100% (0 retries), confirmed symmetric 7.1 vs 5.0, map 40/40 with zero ghost entries, home deafness halved; star-line (9 hops): 126 missed requests over 4 h (a startup transient; the pre-rotation arm: 117 — the miss answers ride the rotation now). The replacement is measured in simulation scenario 6 against the current main (two builds, fixed seeds) before it is merged. The old 4.3/4.4 text moves to section 15.**

**The star (unit of sharing).** The star of node X is the unit of sharing: one node entry (node ID, quality class per direction, tick byte, geo — the 9 B entry layout that a chunk receiver already understands) plus every link that X **owns** (below), each 3 B (peer ID, class byte per direction), **sorted by peer ID**. A star is therefore *not* the complete link picture around X: it is the entry plus X's owned share of the links. Every link stands under exactly one endpoint, so every link travels inside exactly one star; **the known graph is the union of all known stars** — the link store, not any single star, is the graph. The sorted order makes the star a canonical byte sequence, independent of internal table order.

**Link ownership (decided, Oct 2026): one bit of a hash over the normalised pair.** The owner of a link {A, B}: normalise `a = min(A, B)`, `b = max(A, B)`, compute `h = CRC-8 over the 4 bytes of (a, b)`, and let bit 0 decide — `h & 1` set: `a` owns the link, otherwise `b`. Both endpoints — and every receiver, whatever the hop distance — compute the owner from the pair alone: no protocol field, no owner bit in the store, no negotiation. The normalisation makes the rule symmetric by construction: both sides hash the very same bytes, so no AB-vs-BA comparison and no tie-break can ever arise (a comparison of two ordered hashes would collide for roughly 1 in 250 pairs and need a tie-break; the unordered form has no such case at all). Measured, the split is fair: bit 0 falls on either side ~50% of pairs, so each node owns about half its links, star sizes even out across the neighbourhood, and no site is structurally the fat one — the problem of the simpler "lowest ID owns" rule, which would concentrate all links of a dense neighbourhood on one node (a fully meshed 64-hub would put all ~2,000 links on the lowest-ID site, against the one-chunk packet bound). Ownership is permanent (IDs and the hash are fixed), so a link's advertised form stays bound to exactly one star — the precondition for the content hash below.

**Storage: as received (decided, Oct 2026).** The link store holds links **exactly as the owner advertised them** — the class byte per direction as it travelled in the owner's star — and nothing else. Local measurements do not write to the link store: a direct neighbour's own two directions live in the neighbor table (`quality`, `rev_quality`, 4.2), which is where routing reads the first hop anyway; the link store is the *forwarded* picture. Likewise the entry class byte of a map node is stored as received, not refreshed by "fresher-looking" forwarded classes. The consequence: serialising the star of X (entry bytes as received + all links owned by X, bytes as received) is a pure read that reproduces the origin's advertised form **byte-exactly on every hop**. The content hash is thereby a property of the origin, not of the observer — the per-observer hash divergence measured in scenario 6a disappears at the source. Two stores, two questions: the neighbor table answers "what do I measure myself", the link store answers "what did the origins advertise".

**Content hash and digest.** Over the canonical bytes, excluding the tick byte: a 1-byte content hash (CRC-8). The hash covers what is advertised (identity, links, quality classes), not how old the advertisement is — sub-hysteresis EWMA movement does not change it. The catalogue digest of a star is `digest = (content_hash + tick) mod 256`: **derivable**. A receiver that has the star can compute the expected digest for its own hash at `tick + k` for small k (starting value: k ≤ 8) — that is the heartbeat match.

**The heartbeat tick.** The tick byte of a star is the origin's heartbeat: the origin increments it at every guaranteed own-star round (once per hour, starting values below), and the tick rides along in the star form, hop by hop, through the catalogue. Every star thereby has network-wide liveness, separate from the local death detection of direct neighbours (4.4). Wrap at 256 is harmless within the k-window: ticks compare modulo 256.

**The round (one packet type: STAR_ROUND).** Every node broadcasts at most one chunk per interval (starting value: 15 min = 16 proof ticks) — **channel rotation with a backlog override (decided, Oct 2026, from the scenario-6 airtime measurement)**:

- **Rotation at rest.** A round with thin content (no backlog: the queue fits) transmits on ONE channel: the rotation channel `(home + round_counter) mod 3`, so a node cycles home → home+1 → home+2 and back. The neighbours at home on that channel receive the full round; the resting overhead (header, catalogue, heartbeat) is paid once per interval instead of three times — measured (utrecht40, 24 h rest): 2.3% per channel → ~0.8%, below the old cadence's 0.9%. Fewer transmissions also mean less home deafness (the node is away from its own channel at most one interval in three) and fewer collisions (per channel only the third of the nodes whose rotation it is).
- **Full spread under backlog.** A round whose content area is full — or whose queue did not fit — transmits on ALL THREE channels, exactly as the original cadence: a backlog drains at today's speed, because under backlog every channel's throughput counts. The rule is local (does anything remain queued after packing?) and needs no measurement.
- **Carried per channel.** The content hash gate (below: a star re-sends only on content change) is per CHANNEL now, not per transmission: a star leaves the queue only when it has travelled on all three channels (3 carried bits per star). A mutation therefore needs up to three intervals (45 min at the 15-min starting value) for full neighbour coverage — the price of the rotation, paid only at rest; under backlog the full-spread round sets all three bits in one interval, exactly today's behaviour. The guaranteed own-star round (tick +1) stays on the OWN HOME channel; the own star rides the other two channels in the following rounds.
- **Coverage cadence.** Consequence: a content star reaches its per-channel community every 15 min, the full neighbourhood within 3 intervals (45 min per hop at rest, 15 min under backlog); the catalogue and tick rotate along and stay on the 15-min cadence. Miss-list requests name the channel-independent neighbour; the answer rides the addressee's rotation, worst case a few intervals.

- **Catalogue prioritisation (measured, Oct 2026, not adopted).** A triggered fill (heartbeat due after a rotation cycle, staleness above a third of the eviction bound, fresh arrival) plus a small untriggered quota was measured on utrecht40 rest: no airtime win (291-298 s vs 237-248 s per channel, 6 h) - the heartbeat band collects most stars at once, so the catalogue stays full anyway and the plain fill is simpler and cheaper. Open: with a much wider heartbeat band (e.g. 2 h) the resting catalogue would thin out; the k-window and the liveness detection would have to be re-measured against it. The plain fill stands: prefix first, then map, everything that fits. type/flags byte (`F_STAR_CATALOGUE 0x10`, `F_STAR_MISSED 0x20`), owner ID, **catalogue count** (the old chunk-nr slot: the receiver knows where the catalogue ends and the missed area begins), content star count, owner geo, firmware version. Three payload areas in fixed order:

1. **Content area** — complete stars in the chunk entry layout, self-contained processable via the ordinary chunk entry path (map build, link store, and symmetry evidence: an owner naming us confirms the link, 10.1, at no extra airtime).
2. **Catalogue area** — `(node ID, digest)` at 3 B per star: what the sender has, in which version.
3. **Missed area** — `(neighbour ID, n, star IDs...)`: requests to one specific neighbour.

**Hard airtime bound (constructive, not statistical).** The round timer is the only send path: never more than one chunk per node per interval (~869 ms per 15 min ≈ 0.1% of a channel), whatever happens. A mutation storm, a buggy firmware, a mass reboot — the queue absorbs, the round degrades to fewer stars per chunk, the bound holds.

**Packing order (fixed priority).**
1. **The own star first.** On a mutation immediately (`tbl_dirty`), otherwise via the **guaranteed own-star round**: at least once per hour, regardless of queue position and how full the content area is. The same rule drives the heartbeat: at the guaranteed round the origin's tick advances (+1). One rule, two guarantees.
2. **Content queue**: stars whose advertised content changed since they were last carried. Per star the node remembers `star_fwd_tick` (16-bit, when last carried) and `star_fwd_hash` (the content hash as last carried). Re-send only when the hash differs — an age refresh or sub-hysteresis measurement does NOT re-send; the catalogue is the cheap confirmation. **No entry cap**: stars pack until they do not fit the chunk; what does not fit stays queued and drains over the following rounds, inside the same hard bound.
3. **Catalogue reserve**: the last 30 B of every round are reserved for catalogue entries — a sustained content backlog cannot starve the age maintenance. At 4 rounds/hour that is a minimum of 40 digests per hour on air. The catalogue itself: own star first, then the map, prefix before tail.
4. **Missed requests** in the remaining space.

**Graph pruning of forwards.** Before a queued star is packed, the forwarder runs a BFS over its own known graph (map + link store + own edges; pure hop counting) from the star's center. The forward is useful only if at least one confirmed neighbour lies strictly farther from the center than the forwarder itself; if every known neighbour lies closer or level, the forward would be an echo back into the gradient and is skipped — the star stays in the map and keeps its queue state. Safety rules: unknown neighbours (no map slot, not reached) always count as "farther" — absence of knowledge never silences a forward — and the missed list is the net for whatever the coarse check gets wrong. Zero bytes on air; one BFS per queued star per round, millisecond order on the MCU. A reactive second layer (damping on overheard catalogue advertisements of the same content version) is deliberately not built: scenario 6 measures the duplicate work first.

**The receiver.** Content stars go through the ordinary chunk entry processing (above), with **fresh-beats-old on content**: a form whose advertised bytes differ from the stored ones is accepted only when its tick is AHEAD of the stored tick — a differing form at an equal or older tick is a stale copy racing a newer one (a slow route, a missed-list answer) and must not roll the newer knowledge back; its link claims are skipped with it. An identical form is always fine (an echo refreshes nothing but the age). The star-round post-pass per entry: tick byte → `star_tick`, travelled age restarts at 0 (`age_ticks = 0`), `arrived_tick = now`, and the star's miss entries clear. The catalogue, per `(ID, digest)`:
- **Derivable match** (symmetric k ≤ 8): the star is unchanged — the advertisement may be ahead (advance `star_tick` by k, restart the travelled age, clear the miss entries) or lag behind (a slower route carried the newer tick here first; the stored tick stands, only the age refreshes). Freshness confirmed without a single content byte: Trickle in 3 bytes, and the heartbeat of the resting network.
- **No fit, star unknown here**: a gap — onto the missed list.
- **No fit, star known**: a real content change upstream (or a hash collision) — onto the missed list as well.
- A star that arrived as content in the same packet never gets a request: fresh content beats the catalogue mismatch.

**Self-publication of the own entry (implemented with storage-as-received).** The origin of a star is itself the source of its own entry byte: the classes of the link picture toward its strongest confirmed neighbour, published under the same hysteresis as every link byte (a ≥ 2-step class move re-publishes, sub-threshold EWMA movement does not) and stored in its own `adv_q` — the origin's catalogue digest is computed over exactly the bytes its neighbours hold. This closes the last asymmetry of storage-as-received: the origin is an observer of its own advertised form, like everyone else.

**The missed list (requests in-band).** A request rides the requester's own round in the missed area; the named neighbour resets the last-carried state (`star_fwd_tick = 0`, `star_fwd_hash = 0`) so the stars re-enter the content queue and travel with its next regular round. No exchange, no ack, no channel switch; the answer rides the cadence, bounded by the same hard bound. Rules: at most 12 entries, no duplicates, only direct neighbours as addressees, at most 2 attempts per (neighbour, star) — then fresh-beats-old and the regular rounds remain the net; entries to a neighbour that has disappeared from the table are dropped at pack time; an arrived star clears its entries. **Eviction taboo (decided, Oct 2026, from the islands pool-pressure test):** a catalogue mismatch for a star we DELIBERATELY evicted (pool recycle, 4.4) is NOT a miss - it is a policy choice, and requesting it back is a perpetuum mobile under pool pressure: the star arrives, displaces something else, gets evicted again, and the next catalogue mismatch re-creates the request with a fresh attempt counter. Measured (two 25-node islands, MC_MAP_NODES=32): 21,093 miss requests over 6 h against 18 at full pools - a thousandfold, all from this cycle. The rule: a small ring buffer of recently evicted star IDs (starting value 32 entries); a mismatch for a star in the taboo creates NO request. The taboo is purely local, costs no airtime, and aligns the request service with the storage policy: the pool policy decides what we WANT to know, and the missed list serves only genuine gaps - stars we never had. An evicted star returns through the regular rounds (the catalogues keep confirming the neighbours' copies) or through a route need (a message toward it makes the pool policy re-evaluate), not through self-inflicted requests. **An explicit ask overrides the pruning** (implemented): the addressee records the request per star, and for a few intervals the gradient check yields — the star travels as content even when every known neighbour lies closer to or level with its center. The missed list is thereby truly the net for what the coarse check gets wrong, including the echo cases the BFS cannot see.

**What replaces what.** Under the star round there are no delta chunks, no bundling windows, no periodic prefix re-share, no tail cursor and no negative deltas: a mutation is a content change of the own star and travels with the next round; a dead neighbour disappears from the own star (the links it owned and the entry) the same way, and the links it did not own disappear from the owning peers' stars on their next round. Hello/proof-list mechanics (4.1, 4.2) are untouched — the round leans on them (last_seen aging, symmetry evidence).

**Resolved in design (Oct 2026, from the scenario 6a finding):** the measured open point — the content hash of a star differing per observer (2481 miss requests over 6 h on the 10-node line) — had two causes, both now designed out at the source: (1) the serializer mixed locally gathered and received knowledge into one form, and (2) a link could travel in the star of *both* endpoints. The storage rule above removes (1) — the advertised form is stored and forwarded untouched — and the ownership rule removes (2) — a link has exactly one owner-star. A "recently arrived" window on miss creation stays in reserve; with byte-exact stars it should not be needed, to be confirmed in scenario 6a.

**Newcomer acceleration (decided, Oct 2026).** The old prefix advertisement carried the full neighbourhood to every neighbour, a newcomer included; under the star round a newcomer sees a catalogue full of digests it cannot match. The miss list covers that, but rides the requester's own cadence and its caps (12 entries, 2 attempts) — a fresh neighbour of a 64-hub would otherwise need hours to collect the full picture. The rule: **on confirmation of a new neighbour, the node resets the forward state (`star_fwd_tick = 0`, `star_fwd_hash = 0`) of all prefix stars** — the stars of its direct neighbours re-enter the content queue and travel with the coming rounds, unsolicited. Zero extra bytes on air (the same stars that round anyway, only prioritised), bounded by the hard one-chunk-per-interval rule; the guaranteed own-star round already carries the own star. This is the star-round translation of the old "delta on addition": a newcomer matters, so the queue turns for it.

**Parameters (starting values, to sweep in scenario 6):** interval 15 min; own-star guarantee on the home-channel round (~1× per 45 min at the 15-min interval, the 1×/hour bound honoured as a lower bound); rotation (home + round counter) mod 3, full spread under backlog; catalogue reserve 30 B; k-window 8 (symmetric: ahead and lagging); missed cap 12, attempts 2; digest 1 byte; publication hysteresis 2 class steps; pruning override after an explicit ask: 2 intervals.

### 4.4 Ageing and death

**Status: decided in design (Oct 2026), replacing the old 4.4. Fully implemented and measured (scenario 6c, Oct 2026): local ageing, last_seen for direct neighbours, and the tick-liveness axis below (star_tick_seen, de-weight at half the threshold, eviction at ~6 h of tick stall). Measured utrecht40 48 h with one node silenced at t=24 h: the tick stalls everywhere at the silencing moment, the de-weight starts at ~4.8 h, and the dead origin's star is gone from ALL 39 nodes by ~6 h after silence (0/39 present at t=30 h) - versus the old 4.4 arm which kept the dead node in 37/39 maps after 24 h of silence, and the pre-liveness star arm which showed it falsely fresh (eff age ~2 ticks) forever through echo matches. Freshness and liveness are now truly independent axes.**

**The principle: age is local, confirmation resets it.** Under the star round, no age travels on air (the tick byte is liveness, not age). Freshness is purely local: every confirmation — content received, or a derivable catalogue match — sets `arrived_tick = now`. Effective age = `tick − arrived_tick`, in proof ticks (~56 s). This is cleaner than the old sum of travelled age plus local ticks (one clock, not two), and it removes any dependence on senders counting honestly.

**Three axes of expiry.**

1. **Freshness axis (routing weight, continuous).** The effective age feeds the existing route weighting (`route_age_mult`): old knowledge weighs heavier but is never refused. Unchanged from 4.3, only the computation is simpler.

2. **Eviction axis (memory, hard).** A map entry (foreign star) whose effective age reaches ~24 h (2 × T_active, the existing bound) is evicted. Direct neighbours are never evicted — they live in the neighbor table.

   **Pool recycle at the cap (decided, Oct 2026, from the 480-node scale run).** When the map pool is FULL and a new star arrives (the scale reality: MC_MAP_NODES = 256 against a 480-node world), the victim is chosen by **routing value, not freshness**:
   - **Primary: hop distance** to the star (BFS over the own known graph, the same distance field the forward pruning already computes). Closer stars are kept: they are cheaper to re-confirm and more likely on real routes.
   - **Secondary: total link length** - the sum of the geo distances of every stored link this star takes part in (all links between map nodes whose endpoints both carry a geo, where this star is one endpoint). HIGHER is better: removing a well-connected hub from the map fragments the graph more than removing a leaf; the star with the LOWEST total link length is the first victim.
   - Tie order: hop distance first (a hop-2 star with a small link sum stays over a hop-5 star with a large one), link length second, slot index last (deterministic).
   The old least-fresh-first rule fails at scale: freshness at a full pool is a rotation-timing lottery (derivably matched echoes refresh arbitrary entries), so it evicted routing capital at random. Freshness remains the expiry trigger (the 24 h bound above) but never the recycle choice. Rationale measured: at 480 nodes the pools sit at the cap within hours and thrash - each arriving star evicts a random other, and the re-arrival of the evicted star evicts the next: the map churns without learning.

3. **Liveness axis (implemented, measured Oct 2026: remote death detection).** A dead origin stops ticking, but its last star keeps circulating: forwarders advertise it in their catalogues with the old tick, every receiver matches derivably (k small) and resets the age. Without an extra rule a dead origin would look fresh forever. Therefore: per map entry, `star_tick_seen` (16-bit local tick of when the entry's tick last advanced). A star whose tick has not advanced for N tick periods (starting value: 6 hours) is treated as "origin or chain dead": first de-weighted in routing (same multiplier as old age), then evicted — regardless of how fresh its age looks through echoes. Freshness and liveness are independent axes: confirmation is trust, tick advance is life.

**Death detection per object.**

- **Direct neighbour:** unchanged mechanics (4.1/4.2): last_seen ages per proof tick, at `T_expire` (~4 h) the entry disappears. The consequences now travel via the stars: the links the neighbour owned disappear from its own star, the links it did not own from the owning peers' stars → the next rounds carry the change → the network adapts within minutes (direct) to hop × interval (far). **No tombstone, no negative delta**: silence plus the round is the message, exactly as the old 4.4 decided, but without the F_NEG packet.
- **Remote node (map entry):** the liveness axis above. This is the network-wide death detection the old 4.4 lacked — remote entries used to age out blindly at 24 h; now a dead origin's star degrades within ~6 h after its last tick, everywhere.

- **Open: hop-scaled liveness (later, 6d follow-up).** The liveness threshold is flat (~6 h) everywhere, but the evidence quality differs by distance: a 1-hop neighbour loses the origin's own transmissions directly, a far observer only the echo trail. The extinction of a dying star could therefore run FASTER near the origin (strong local silence) and slower far away — or conversely, the threshold could grow with hop distance (each hop adds propagation delay to the last genuine tick) — with no new protocol field: the hop distance is locally known from the map (the BFS the pruning already computes). To be measured in scenario 6d: the death wave front per hop band, and whether a hop-scaled threshold beats the flat one on route flapping vs. detection latency.
- **The tombstone** keeps exactly one task: removal by the neighbor table limit on hub sites (10.4) — a living neighbour that keeps transmitting. That mechanic is untouched and orthogonal to the star round.

**What falls away with the replacement:** travelled age on air; the age byte as an age carrier (it is the tick now); F_NEG negative deltas; the "2 × T_active for remote entries" blind spot (replaced by the liveness axis, which is earlier and targeted).


## 5. Geo encoding

| Property | Value | Status |
|---|---|---|
| Range | Worldwide | Decided |
| Latitude | 16 bits: 180° divided over about 65,536 steps, ≈ 0.002747° ≈ **306 m** per step | Decided (Oct 2026) |
| Longitude | 16 bits: 360° divided over about 65,536 steps. **Maximum ≈ 612 m per step at the equator** (worst case), ≈ 376 m in the Netherlands (52° N). Shrinks with cos(latitude). | Decided (Oct 2026) |
| Total size | **4 bytes** (16 + 16 bits, no flag byte) | Decided (Oct 2026) |
| Invalid geo | Sentinel (0, 0). This decodes to (−90°, −180°), the South Pole/date-line corner, where no node ever stands. A reserved or cleared entry is therefore automatically "no geo". | Decided (Oct 2026) |

**Change from the previous form (5 bytes: 18 + 18 bits + 4 flag bits).** Geo becomes 4 bytes, per neighbor table entry and in the optional source geo of every data header. The grid becomes four times coarser: 306 × 376 m in the Netherlands instead of 76 × 90 m. This is enough for all network purposes. The bridge-node rule (10.2), the Gabriel graph and the kNN radius work at kilometre scale, and coarser geo is better for privacy (goal 7). For display purposes, a companion keeps its precise GPS locally. The earlier goal of "about 100 m" has therefore been abandoned for the network function. The FULL root geo (6.5) already used this format.

**Precision levels are dropped as a protocol field.** There are no flag bits any more. Anyone who wants to coarsen (companions: 1 km or 10 km) sets the low bits to zero before publication. The level does not travel with the packet and cannot be inferred by receivers. This is acceptable: routing does not need it, and the display shows the published value anyway.

**Privacy.** The geo is public, because every receiver can read the packet. Encryption would break routing, because relays need the geo. The 306 m grid is the only protection in the protocol itself. Coarsening before publication is the second layer, on the client side.

---

## 6. Packet headers

### 6.1 Base header (data)

| Field | Size |
|---|---|
| Type + flags | 1 byte |
| Message ID | 2 bytes |
| Source ID | 2 bytes |
| Destination ID | 2 bytes |
| Previous hop ID | 2 bytes |
| Budget | 1 byte (section 9), at offset 9 of the header |
| Next hop ID | 2 bytes, at offset 10 (decided Oct 2026, see below) |
| **Total** | **12 bytes** |

**Next hop ID (decided, Oct 2026).** A unicast travels on the home channel of the next hop, but other confirmed neighbours sit on the same home channel and hear every packet there (snooping, 4.3). Without a next-hop field a forwarder cannot tell whether a heard DATA is addressed to it: every snooping candidate would forward toward the destination, and a routed message multiplies per hop into a retransmission storm. The next hop is the one hop the sender chose (10.2); only that node processes the packet, as delivery candidate or forwarder. Other hearers treat it as snooping material (liveness, quality), exactly like table chunks. The retry and alternative-hop series (7.4) rewrite the field in place - stateless per message: each hop chooses its own next hop.

Optional, present when the flag is set:

- **Source geo** (4 bytes, section 5). Needed when the source falls outside the destination's graph and the destination must send an ack3 or a failed delivery report (10.5) back.
- **Destination geo** (4 bytes, section 5; decided, Oct 2026, see 10.2). Needed so that every hop can route greedily toward a destination that lies outside its own known graph. The source attaches it when known; a forwarder keeps it standing in the header.
- **Waypoint geo** (4 bytes, section 5; proposal, 10.2): an intermediate point that replaces the dest geo for the greedy rule until a hop passes it.

**Flags in the type byte (proposal):** ack3 requested, source geo present, ack2 negative, broadcast, destination geo present, waypoint geo present. Six flags do not fit in four bits: the type byte gets a **flag extension byte** after the fixed header when any of the geo flags is set (dest geo, waypoint, or a flag beyond bit 3). The first four flags stay in the type byte; the extension byte carries the geo flags. Alternatively, one of the two reserved types can become a 'long header' type. Choice: open; the extension byte is the working assumption.

### 6.2 Acks

| Type | Size | Direction | Channel |
|---|---|---|---|
| ack1 | 8 bytes: idem + quality byte — the RSSI quality the acker measured on the data; the sender learns the reverse direction of the link in the same exchange (decided, Oct 2026) | Next hop → previous hop | Same channel as the data exchange; the sender waits there according to the switching rule |
| ack2 | 8 bytes: idem + budget byte — the negative variant (F_NEG_ACK2) carries the **spent budget** of the message (decided, Oct 2026, 7.4): the previous hop sees how much of the 32 is left and can weigh its own alternatives against it (stop rather than try another hop against an exhausted budget). The positive variant carries the budget byte as reserved/0. | Next hop → previous hop | Home channel of the previous hop |
| ack3 | Routed message, payload 4 bytes | Destination → source | Via routing |
| hello ack | 6 bytes, directional: type, destination ID (the triggered sender), sender ID, quality byte — the RSSI quality the responder measured on the hello (decided, Oct 2026) | Responder → triggered sender | During a hello round: home channel of the hello sender. For a home-channel transmission of the trigger (ANN, FULL, or ack1 on its own channel): that channel itself, because the triggering sender listens there by definition. |
| proof list | 3 + 2n bytes (n neighbour IDs, n ≤ 17): type, node ID (2), then n × 2 B | Node → all neighbours at home on that channel | Only the sub-list of neighbours at home on channel c; at most one per channel per hour (4.2). In the list = evidence refreshed, no reaction; not in the list = snooping. |

Ack1 contains the identity of the original message and the sender of ack1.

### 6.3 Hello

3 bytes, see section 4.1.

### 6.4 Announcement (ANN)

| Field | Size |
|---|---|
| Type + flags | 1 byte (including the channel of the FULL, 2 bits) |
| Thread ID | 2 bytes |
| Message ID | 2 bytes |
| **Total** | **5 bytes** |

### 6.5 Large thread message (FULL)

| Field | Size |
|---|---|
| Type + flags | 1 byte |
| Thread ID | 2 bytes |
| Message ID | 2 bytes |
| Root geo | 4 bytes (centre of the thread, same encoding as section 5) |
| Radius | 1 byte (logarithmic scale) |
| Predecessor message ID | 2 bytes (0 for the root) |
| Payload | n bytes |

**Status:** proposal, see section 8.

---

## 7. Message and ack process (unicast)

### 7.1 Forwarding with a double ack

Example: A → B → C → D.

1. A switches to the home channel of B, transmits, and receives ack1 there on the same channel. Then A returns to its own home channel.
2. B tries C, at most twice. No ack1 from C: B tries D, likewise at most twice.
3. **As soon as B has received ack1 from its own next hop, it sends ack2 to A, on the home channel of A** (decided, Oct 2026, replacing the earlier "on reception" reading of 7.2). The positive ack2 therefore means: the message has come two hops — B received it AND B's own hop succeeded. B does not wait for deeper progress; C's forwarding is C's own responsibility within its retry space (7.4).
4. A then knows the message has come two hops.
5. D sends an ack3 to the source, if requested. That is a routed message.

**Status:** decided (revised Oct 2026: ack2 goes out on the ack1 of the own next hop).

**One-hop closing piece (decided, Oct 2026, measured under load).** When the own next hop IS the final destination, the source closes its pending record on the ack1 of the destination: there is no second hop whose ack2 the source would wait for. Waiting for `T_ack2` anyway burned 6 budget units per delivered one-hop unicast and drove retry series through unrelated neighbours (measured: hundreds of `T_ack2` timeouts on messages that had already been delivered). The destination therefore needs no ack2 of its own; its ack1 is the end confirmation.

### 7.2 Meaning of ack2

- **Positive (revised, Oct 2026):** the next hop has received the message **and its own hop has succeeded** — the ack2 goes out on the ack1 of the next hop's own next hop (7.1). Two hops of progress per ack2. Deeper progress is not covered; the hop after that carries its own responsibility within its retry space (7.4), so no ack2 has to wait for the whole chain.
- **Negative:** the next hop could not forward the message (all series exhausted, or source-is-next-hop, 7.4). The spent budget travels along (6.2). The sender immediately recomputes and tries the next-best hop; no report about the report.

**Responsibility per hop (bounded backtrack).** The node that receives the message does not store the message. It stores only a **pending record** `(message ID, previous hop, next hop, spent budget)` in a fixed buffer of, for example, 16–32 slots (FIFO: when full, the oldest is dropped). That record lives until its own hop is settled: positive ack2 (sent on the own next hop's ack1), negative ack2 (sent when every series failed), or the `T_ack2` timeout at the previous hop. After that, only the dedup cache entry remains.

- If forwarding fails after the retries, it tries another neighbour (backtrack within the window).
- If that also fails, or if the next hop goes down after the window, it sends a failed delivery report to the source, as a routed message. That report does not itself generate a new report.

**Deliberate cost:** after a positive ack2, the source can no longer fall back to the previous hop if the failure occurs later and deeper. The only signal is then the failure report or the absence of ack3. There is therefore one return channel for late failures: the routed failure report.

**Status:** decided (positive meaning revised Oct 2026: two hops per ack2; negative carries the spent budget). Late negative notifications reach the source via the routed failure report (see 10.5).

### 7.3 Timeouts

| Timeout | Value | Status |
|---|---|---|
| ack1 (short) | `t_data + t_ack + processing + margin`, measured from the end of the own transmission | Decided (principle) |
| ack2 (fixed) | **`T_ack2 = 8 s`** (revised Oct 2026 with the attempt schedule of 7.4: the schedule completes in ~6 s worst case, so silence beyond 8 s means the next hop's forwarding truly stalled). After the timeout the sender treats the silence like a failed attempt toward this neighbour: budget units go off (7.4), and the message fails for this hop with a negative ack2 upstream | Decided |

- `T_ack2` does not depend on the path length, because the sender only looks at its own next hop. The fixed 8 s covers the worst case of the next hop's own attempt schedule: up to 6 attempts with escalating waits inside ~6 s (7.4), plus the ack2 transmission itself.
- If the next hop itself fails, no ack1 comes. The sender notices this at the ack1 timeout — that is the fast signal; `T_ack2` is only about the *slow* failure (next hop received, but its own forwarding stalls beyond every retry).
- A negative ack2 arrives well before the timeout in every normal failure case; the timeout is the safety net, not the primary mechanism.

### 7.4 Retries

**Attempt schedule (revised Oct 2026, measured under load).** One hop gets ONE schedule of at most 6 transmission attempts, built once at submission/forwarding - no rebuild afterward. The schedule is table-driven and thereby fully readable in the code (candidate table, wait table, one round-robin function).

- **Candidates: up to 3, each from a full Dijkstra run.** Candidate 1 is the computed route (10.2); candidate 2 is the best route that does not start with candidate 1; candidate 3 likewise without the first two. Every candidate is thereby a real, computed alternative over the whole known graph, and the best candidate always leads. The previous hop is part of the Dijkstra exclusion itself from the first round - a post-filter on the result would discard the whole plan when the best route leads back, while the second-best non-prev route exists but was never computed (measured: silent hop drops, five times the T_ack2 firings at the sender).
- **Attempts round-robin over the candidates: 1 candidate = 4 attempts (A,A,A,A); 2 = 6 (A,B,A,B,A,B); 3 = 6 (A,B,C,A,B,C).** At most 4 toward the same node, at most 6 total - both limits hold structurally, without separate counters.
- **Waits: small fixed gap inside a pass, escalating gap at each pass boundary.** The next attempt within a pass goes to a DIFFERENT node, so no damper needed: a fixed 400 ms (+-25% jitter). At a pass boundary the same node comes back - there the damper belongs: 800, 1200, 1800 ms (+-25% jitter), strictly escalating. With 1 candidate every attempt is a boundary; the pass table then runs 0.8, 1.2, 1.8 s. The jitter keeps two nodes that collided once from retrying in lock-step.
- **Everything inside ~6 s worst case, 10 s including maximal CAD backoffs.** Measured on the load run: last-hop latency max 3.9 s (was 28.0 s with the old series), p95 1.1 s.
- **Schedule exhausted: definitive fail for this hop.** All candidates go into the cumulative exclusion together (per message, no protocol field; the penalty 10.4 carries the long-term lesson), and a negative ack2 goes to the previous hop with the spent budget - the previous hop decides with its own schedule and budget whether another route remains. No rebuild: the 6 attempts ARE the retry space of the hop.
- **Retry reception is acked (decided, Oct 2026):** a received transmission that is a duplicate (dedup hit) gets an ack1 anyway, on the channel it arrived on. The most likely reason for the retry is a lost ack1, not a lost data packet; silence would make the sender declare the link dead and detour. Dedup still gates the processing: a duplicate is not delivered or forwarded again.

**Status:** decided (attempt schedule: Dijkstra candidates, round-robin attempts, pass-boundary waits, definitive fail; retry acking).

---

## 8. Threads and regional chat

### 8.1 Concept

- A thread is a conversation started by one starter.
- The starter determines the area. The centre is its own geo, and the radius is derived from density.
- Replies stay within the starter's radius. A relay forwards a reply only if it is inside that radius.
- There are no fixed cells. This suits areas with very different densities.
- A thread has its own identity: thread ID = hash of starter ID and a counter or timestamp. Threads therefore do not disturb each other.

**Status:** decided.

### 8.2 Radius determination

- The radius is the distance to the k-th nearest node in the starter's map, with `k ≈ 15`.
- Dense areas get a small radius, sparse areas a large one.
- The radius is encoded in 1 byte on a logarithmic scale, from about 500 m to about 60 km. Higher values fall to the maximum code.
- The lower bound is not critical: a radius that is too small harms no one. The upper bound does. It is a firmware constant with a starting value of 40 km, not a value in the packet.
- Relays reject a radius above the cap of their firmware version. An increase is rolled out in phases: first all nodes accept the new cap, then starters use it.
- The radius is calculated once per thread and then sent along in the header.

**Status:** decided for the method (kNN). Open: exact k and the bounds.

### 8.3 Spreading within the radius

**Dominant pruning.** A receiver forwards only if it has neighbours that the sender does not reach. For this it uses the neighbor table of the sender, which it already has via 4.3. There is therefore no extra header field for the neighbour list.

**Waiting time and suppression.** A candidate forwarder waits a random time that becomes shorter the more new neighbours it can reach. If it hears a retransmission from someone else, it recalculates its coverage using that sender's neighbour list. If neighbours remain that none of the heard senders covers, it waits and transmits for that remainder. If nothing remains, it stays silent. Suppression applies only to the channels the node itself listens to. Duplicates on other channels are caught by the dedup cache.

**Dedup.** A cache on `(thread ID, message ID)` catches repeats.

**Status:** proposal.

### 8.4 Recovery and sequence numbers

- FULL messages contain the message ID of their predecessor (2 bytes, 0 for the root).
- A node that receives FULL M with a predecessor P that it does not have requests P via unicast from the sender of M. If that fails, it requests P from another neighbour that has P. This repeats up to the root.
- **Tail loss:** if the last message of a thread was missed and nothing follows, nobody notices. The original sender therefore repeats its ANN once after a random delay of a few minutes. Nodes that already have the FULL suppress this. It costs one ANN of 5 bytes per message.
- **Recovery after a missed ANN:**
  - (a) every forwarder re-announces its own forward on all three channels, so that a message passes nodes on each channel several times;
  - (b) a FULL transmitted on the home channel of a forwarder is heard directly by nodes on that channel, without an ANN;
  - (c) a later message whose predecessor is missing triggers a unicast request to the source of that message, with a switch to its channel.
- When first contacting a neighbour, newcomers ask that neighbour for its recent threads. This is event-driven, not periodic, so it costs nothing in the hello.
- Thread information is not included in the hello. There is no periodic digest comparison.

**Status:** proposal. Open: the delay of the repeated ANN and the cache size.

### 8.5 Announcement and large message (ANN / FULL)

To save airtime, the large message is sent only once:

1. The sender of a FULL first sends an ANN, on all three channels in a fixed order, with a short pause `g` in between.
2. Then it sends the FULL on its own home channel.
3. A node that hears an ANN and does not yet have the FULL waits a short random time `f`. If it has heard the FULL within that time, it stops. If not, it switches to the channel of the announcer and listens for the FULL.
4. The timeout for that fetch is `T_full = T_ANN + t_full + margin`.
5. On timeout, it tries at most two other ANN sources. After that, sequence recovery takes over.

The waiting time for senders uses the same logic as in 8.3.

**Parameters to simulate:** `W_base`, `J`, `g`, `F`, `margin`, and the maximum number of fetch attempts (currently 2).

**Status:** proposal. Parameters: open.

### 8.6 Newcomers

- Each node stores threads within its radius in a fixed amount of memory. There is no time limit. When memory is full, the thread with the oldest last activity is dropped (LRU).
- Last activity is tracked with a local counter, not with a clock. No time synchronisation is needed.
- A new node asks one neighbour for that cache after its first hello. This is done via unicast, not via flooding.
- A reply without a root: the node asks a neighbour for the root.

**Status:** decided (principle: fixed memory size with LRU). Cache size: open.

### 8.7 Load limiting

- A maximum number of messages per minute may apply per thread.
- A waiting time between messages applies per sender.
- Chat is best-effort. Thread messages do not need ack3.

**Status:** proposal.

---

## 9. Budget (replaces TTL)

- One byte in the base header, starting value **32**.
- Each forward costs 1. Each retry costs 1, deducted by the sender before retransmitting.
- At zero, the node stops forwarding and reports this negatively.
- Loops are excluded, because every forward subtracts at least 1.
- The budget is a total per message. Retries therefore do not multiply along the path.

**Status:** decided. The value 32 must be tested in simulation.

### 9.1 Short TTL for threads

Threads could use a shorter limit, for example three bits. That provides some extra room in the flags, at most one byte of gain.

**Status:** proposal. A path in the header is not needed, because it costs more than it gives.

---

## 10. Routing

### 10.1 Map

- Per node: the known graph built from the own direct neighbours plus everything that arrives via the table chunks (4.3): nodes as ID, geo, age and quality, and the links between them with the advertised quality class per direction. Bounded at 256 map nodes and a fixed link store (13.2, 13.3). There is no hop horizon anymore: an entry counts, wherever its source lies. Beyond the graph, a node knows nothing.
- Links are used only if they are above a quality threshold and work in both directions.
- **Quality gate on confirmation (decided, Oct 2026):** a confirmation (hello ack addressed to us, proof list naming us, ack1 on our data) makes a link symmetric only when the measured quality of that reception is above the threshold. Each node can only measure the direction toward itself; the neighbour applies the same gate to its own direction. The combined effect: a link is confirmed symmetric only when BOTH directions are above the threshold — the core value is `MC_SYM_QUALITY_MIN`. A weak link stays "seen": liveness and quality remain stored, the proof refreshes the reverse evidence, but routing never selects it. The proof list only sends confirmed links (4.2), so weak links are also never advertised to third parties as working.
- **Gate value (revised Oct 2026, measured under load):** `MC_SYM_QUALITY_MIN` = 13 (~−102 dBm). At 14 (~−100 dBm) the utrecht40 scenario kept 2 of 40 nodes without a single routable link and ~10% of the unicasts were refused as 'dst not in graph'; at 13 every node has at least one confirmed neighbour, the average rises from 3.5 to 4.8 confirmed links, and delivery reaches 98.9% (load run, 24 h warmup + 6 h). The retry series (7.4) absorb the higher loss rate of the weaker links: T_ack2 timeouts and FAILs stayed at the same level, duty stayed far under the limit. Below 13 there is no further structural gain in the scenario (only thicker retry traffic over bad links).
- Each direction must be proven by the receiver of that direction:
  - A hello or an overheard hello ack proves only sender→receiver ("seen").
  - The reverse direction comes exclusively from the neighbour itself: via a hello ack addressed to us, via its table chunks (4.3), or via a successful unicast with ack1 ("confirmed").
- Routing chooses the next hop only via confirmed links. Direct replies in exchanges are exempt from this requirement (4.3).

**Status:** decided.

### 10.2 Choosing the next hop

Routing is **stateless per message**: the path is not in the header, every hop recomputes its own first step on its own known graph (10.1). A route therefore repairs itself at every hop when a table mutation arrives; there is no route invalidation, no route error propagation, and no stale path in flight. Loop-freedom does not come from the route computation but from the two existing mechanisms: the budget (9) bounds the total number of forwards and retries per message, and the dedup cache (8.3) discards repeats.

**Dedup cache replacement (decided, Oct 2026, measured under load).** The dedup cache is a fixed-size FIFO of the most recent (msg-id, source) pairs; when full, the oldest entry is evicted. The earlier "full cache = receive anyway" pass-through turned dedup off completely after 128 messages, and a two-hop routing ping-pong could then forward one message more than 160 times (each forward is a fresh pending, and the retry-ack1 rule of 7.4 keeps the loop fed). FIFO replacement bounds the duplicate work structurally, without aging bookkeeping on the entries.

**Prev-hop exclusion at the first forward choice (decided, Oct 2026, measured under load).** At the first route computation for a forwarded message, the hop the message came from is not a candidate for the next one. The known graph is local and incomplete, so two nodes can be each other's best first hop toward the destination (measured: 279<->285 bounced one message 166 times before a 15 s `T_ack2` broke the loop). The exclusion does not travel in the header - the previous hop is already in the packet (offset 7) - and costs nothing extra. The retry machinery of 7.4 applies it from its second series onward; here it prevents the two-hop loop from starting at all. When the exclusion leaves no candidate, the hop does not forward and the failure becomes visible through the negative ack2 / the failure report (10.5).

| Situation | Rule | Status |
|---|---|---|
| Destination in the known graph | Shortest route on ETX-like quality, computed on the known graph (Dijkstra, below) | Decided |
| Destination outside the graph, dest geo known | **Bridge-node rule (decided, Oct 2026):** combine geo and Dijkstra — see below | Decided |
| Destination unknown (ID only, no dest geo) | Do not flood. Wait until the graph contains the destination, or return a negative report | Decided |

**Shortest route on the known graph (worked out, Oct 2026).** The hop computation is a Dijkstra over the known graph with the own node as source and the destination as target. Edges:

- The **first step must be a confirmed direct neighbour** (10.1): only edges from the own node run over the neighbor table, and only over links that are confirmed symmetric. This is the hard requirement; everything after the first hop is knowledge, not authority.
- **Further edges run over the link store** (4.3): advertised links between map nodes, with the advertised quality class per direction. A link is only used when both directions are above the quality threshold and the effective age (4.4) of both endpoints is below expiry; expired entries are map material, not route material.
- **Cost per edge: an ETX-like value from the quality class.** The cost of one link is the expected number of transmissions: cost = 1/p, with p the per-transmission success probability of the link. The route cost is the sum over the path, so Dijkstra minimises the expected total airtime of the message. The quality travels on air only as a class (prefix 3 bits = 8 classes, tail 2 bits = 4 classes), so the cost is a lookup table indexed on the class. Directly measured links (own neighbor table with `quality` and `rev_quality`) are more precise than the advertised classes; the own edges therefore use the measurement, everything beyond it the class table.
- **Cost table (decided, provisional, Oct 2026 — to be calibrated by the load simulation).** Start values, integer scale ×16 (cost 1.0 = 16), per class step p falls by roughly ×0.66 (cost ×1.5):

  | class | 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 |
  |---|---|---|---|---|---|---|---|---|
  | p | 0.80 | 0.55 | 0.37 | 0.25 | 0.17 | 0.11 | 0.07 | — |
  | cost (x16) | 20 | 29 | 43 | 64 | 94 | 145 | 228 | not routable |

  The tail's coarse 2-bit classes map onto the same scale at classes 2, 4, 5 and 7. Class 0 (never reported / below threshold) is not routable.
- **Penalty (10.4) is a local addition on the own edges only:** the cost toward a penalised neighbour rises, so the route computation avoids the neighbour without a protocol field.

The result of the computation is one next hop; the message carries no more than that. Hops deeper in the graph recompute with their own, fresher knowledge. A route that is locally suboptimal on paper (for example through a slightly older tail entry) is accepted: the age weighing (4.4) already discounts stale information, and the alternative — route errors and path state — costs more airtime than it saves.

**Destination outside the graph: the bridge-node rule (decided, Oct 2026).** When the destination is not in the own known graph but its geo is known, geo and Dijkstra cooperate in two steps:

1. **Bridge node:** find the graph node closest to the destination geo (distance in the 16-bit grid of section 5). All nodes within **15% of that minimum distance** join the candidate set (decided: the margin grows with the absolute distance, so far from the destination there is more freedom to pick a better path; at 30 km that is 4.5 km of play along the ideal line).
2. **Dijkstra decides:** among the candidate set, the next hop follows from the shortest-route computation above — the candidate with the best route cost wins. The next hop itself does **not** have to lie closer to the destination: it lies on the shortest path *toward the bridge node*, and that path may start sideways or even backwards (around a lake, along a dam). Every void-perimeter problem of pure greedy routing disappears here, as long as there is any path material toward the bridge node in the graph.

Every hop recomputes: as soon as the destination (or a node that knows it) enters someone's graph, the shortest-route rule takes over — the bridge node is per-hop and local, so no header field and no state. This replaces the earlier pure-greedy rule and its waypoint/perimeter fallback for the routed case; perimeter routing (10.3) lapses for unicast and remains only as a possible later tool.

**Destination geo in the header (decided).** The bridge-node rule needs the dest geo at every hop, and only the source reliably has it. The base header (6.1) therefore gets a second optional geo: **destination geo, 4 bytes, flag**, next to the existing source geo. **The source attaches the dest geo whenever it is known** — not only when the destination lies outside its own graph: the source cannot know what the hops further on still have in their graphs. A forwarder keeps the dest geo standing in the header. Without a dest geo, a destination outside the graph is unreachable: wait or negative report.

**Waypoints** remain possible (proposal): a waypoint replaces the dest geo in the bridge-node rule until a hop passes it; 8 bytes total (waypoint + dest geo together).

**Age in the route cost (decided, Oct 2026).** Stale knowledge must lose against fresh knowledge inside the computation, not only by the receiver-side weighing of 4.4. Edges therefore get an age multiplier on the effective age (4.4) of their endpoints: under half of `T_active` (~6 h) the plain class cost, above that ×1.5, and above `T_active` itself (12–24 h, map material only) ×3 — as a last resort, only when nothing fresher exists. The first-step symmetry requirement (10.1) applies unchanged. The multiplier boundaries follow `T_active` (12 h), not `T_expire` (4 h, neighbor-table only). Parameters to be measured in simulation.

**Computation budget (decided, Oct 2026).** The route computation runs **per message**, not per tick with a cache: 256 map nodes and ~1.5 k links are small enough for an MCU, and per-message computation reacts to table mutations immediately. The bridge-node search is a linear scan over the map (256 distance computations), not a second Dijkstra. A route cache over all destinations remains a quiet-period option (open point 15) if simulation shows CPU problems — not before.

**Status:** bridge-node rule, 15% margin, dest-geo-in-header, age multiplier, per-message computation and the provisional cost table decided (Oct 2026). Cost table values, age multiplier boundaries and the waypoint margin are to be calibrated by the load simulation. Perimeter routing (10.3) lapses for unicast.

### 10.3 Perimeter routing (lapsed for unicast)

- Basis: GPSR-like perimeter mode.
- **Lapsed for unicast (decided, Oct 2026):** the bridge-node rule (10.2) routes around gaps through the graph itself — Dijkstra finds the path toward the bridge node even when it starts sideways or backwards, so the void problem that perimeter was meant to solve no longer exists for routed unicast. Perimeter mode is retained only as a possible later tool for other purposes (for example broadcast spreading), not for the unicast path.
- The Gabriel graph remains useful as a local computation on the known links (a link between u and v remains if no witness w lies within the circle with diameter uv).

**Status:** lapsed for unicast. Revisit only if simulation shows a gap the bridge-node rule cannot route around (no path material at all toward the bridge node).

### 10.4 Penalty and recovery

- A failed transmission increases the penalty of the link. Consecutive failures increase the penalty further. Success lets it decrease slowly.
- The penalty is local. A shared "suspect" field could be added later, as information and not as a decision.
- Probing: periodically send a request to a penalised neighbour, so that recovery becomes visible.
- Hysteresis and a minimum waiting time prevent oscillation.

**Worked out (Oct 2026).** The penalty is a small counter per direct neighbour, next to quality and rev_quality in the neighbor table. Events:

- **ack1 timeout after both attempts** toward the neighbour: penalty +1.
- **Negative ack2 from the neighbour** (itself reached, forwarding failed): no penalty — the link works, the problem lies deeper; the route computation of the neighbour already applies its own penalty there.
- **Successful ack1**: penalty −1 per success, floor 0. Recovery is thereby driven by ordinary traffic; a separate probe (open point 15) is only needed for links without traffic.
- The penalty is applied at route computation (10.2) as a cost increase on the own edge toward that neighbour, proportional to the counter: a small integer multiplier on the ETX cost of the link.
- **Hysteresis:** the route computation applies the penalty only from penalty ≥ 2, so a single missed ack1 (one collision) does not yet push traffic away; and a penalised link is only probed again for routing after a minimum waiting time (order of magnitude one proof-tick, ~1 min) since the last failure. Both parameters are firmware constants and simulation parameters.

**Status:** worked out (mechanism decided, parameters open). Implementation follows with the routing core.

### 10.5 Failed delivery report

- A node that has not forwarded after all retries sends a negative report back to the source.
- The report is a routed message, not a single hop. It uses the normal routing to the source, with the source geo if available.
- Contents: message ID, node ID of the reporter, cause, and optionally the geo of the reporter.
- The report has its own small budget, for example half of the standard budget.
- It does not improve reliability. It is information for the client.
- A node keeps a pending record until `T_ack2` of its own hop or until the retry sequence has finished. After that, only the dedup cache remains. The message itself is not stored.
- The report carries a flag that prevents it from triggering a report of its own.

**Worked out (Oct 2026).** The report reuses the base header (6.1): source = reporter, destination = original source, with the original source geo from the header of the failed message (the source attaches its geo for exactly this purpose). Payload: message ID of the failed message (2 B), cause (1 B: no ack1 after retries, no candidate hop, budget exhausted, destination left the graph), reporter geo optional (4 B). The report carries budget 16 (half of 9) and the no-report flag. It routes normally (10.2), greedy on the source geo when the original source lies outside the reporter's graph.

**Status:** decided. The type is now reserved; the implementation comes with the routing core.

---

## 11. Clients and ack3

- Ack3 is a client function. Routers forward it only as routed data.
- Ack3 is sent only when the flag is set.
- Broadcast and thread messages do not request ack3.

**Status:** decided.

---

## 12. Open points

| # | Question | Why it matters |
|---|---|---|
| 2 | Radius based on local density, target about 100 nodes. kNN works only on an almost complete local picture, so while the graph is small, a bootstrap on link distances applies. The upper bound (starting value 40 km) is a firmware constant that still needs to be tested in simulation. The lower bound is not critical. | Determines the size of threads |
| 3 | Waiting time W in ranks: W = slot × 3/n, with n the number of new neighbours of the candidate. Jitter J less than half a slot, so that the order between ranks stays clean. Still to test: slot size, `g`, `F`, margin. | Determines duplicate work and collisions |
| 4 | Budget 32: enough or too generous? | Limits worst-case traffic |
| 5 | Companion precision: coarsen the published geo before publication; the levels no longer travel. Which level is the default still to be decided. | Privacy and accuracy |
| 8 | RTS/CTS to combat hidden-node collisions: not wanted for now (Oct 2026). The cheap first step is there — CAD, channel activity detection (section 2): carrier sense before every non-reply transmission, ack1 exempt. First measurement (traffic scenario, utrecht40, 2 h, 30 unicast/h, all pairs, 1 seed): lost receptions 11,446 → 9,248 (−19%), confirmed-symmetric neighbours per node 5.7 → 7.8. Residual loss includes hidden nodes that CAD cannot see. A real RTS/CTS exchange (request/clear handshake over the air) stays on the shelf until measurements show that hidden-node loss is heavy. | Only needed if hidden-node collisions cause heavy loss |
| 9 | SF8 nodes in the Dutch data: which range and which airtime. | Radio model |
| 10 | Chunk size of tables (about 200 bytes). | Fragmentation |
| 11 | Battery capacity: for solar repeaters this is settled (continuous listening is the default). The question shifts to companions: battery capacity and lifetime with continuous listening, and whether companions can choose a listening profile based on charging source or battery level. | Companion behaviour, not the protocol |
| 12 | Companion design: BLE protocol to the phone, coupling with the home repeater, and behaviour when out of range (parked, on the to-do list). Companions do not get a node ID. Addressing goes via the home repeater and a slot number, so that joining and leaving causes no changes in the neighbor tables. | Companion is still largely unspecified |
| 13 | Node ID as geo tag plus random suffix: not yet implemented. Disadvantage: the ID changes when the node moves, headers become longer, and the location is always in the packet. | The saving lies mainly in the neighbor table |
| 14 | SF8 fallback for remote nodes. A receiver listens on one SF, so the choice applies per link or per region, not per node. Estimated gain: 25–30% range at about 2× airtime. | Radio design, possibly per region |
| 15 | Local load measurement (channel occupancy via CAD, own duty cycle, ack failure rate) to schedule maintenance in quiet moments. Tasks: (a) local computation without airtime: recalculate the Gabriel graph, recompute routes, clean up tombstones, compress the dedup cache; (b) small unicast tasks: fetch routing information from map neighbours, refresh neighbours' neighbor tables; (c) probes to penalised links, and measuring quality in both directions; (d) measurement work for the SF8 question. Not allowed in quiet periods: floods, broad pulls, and anything that changes the hello. Knowledge from neighbours gets an age and a source, and weighs more heavily the more recent it is. This way the graph fills beyond the directly shared set, without one node having to fetch everything. | Maintenance without extra network load |
| 16 | Trigger generalisation (section 2): the hello ack follows every identifying home-channel transmission of a neighbour. To be measured in scenario 2: the steady-state ack load must not exceed the schedule (0.12–0.17% per channel). Also: ack collisions on ANN/FULL triggers and the rank spreading there. | Determines whether the generalised ack carries the resting traffic alongside the reactionless proof list (4.2) |
| 17 | `T_sym` and the flush margin of the proof list (4.2; starting values 6 hours / 2 hours), and the ratio `T_keepalive : T_expire = 1 : 4` (starting values 1 hour / 4 hours, 4.4). | Route stability versus airtime; to be tested in scenario 5 |
| 18 | Dropped: the keepalive hello no longer exists (4.1); its role is carried by the reactionless proof list (4.2), so there is no ack channel to decide on. | Formerly: ack channel for a keepalive hello on channel c |

---

## 13. Hardware and implementation

### 13.1 Target platforms

| Platform | RAM | Flash | Radio | Note |
|---|---|---|---|---|
| nRF52840 (e.g. RAK4631) | 256 KB | 1 MB | SX1262 | BLE uses part of the RAM |
| ESP32-S3 | 512 KB | 4 to 16 MB | SX1262 | BLE and possibly WiFi |
| ESP32 (classic) | 520 KB | 4 MB or more | SX1276 | Many boards have no PSRAM |

### 13.2 Memory budget (proposal)

| Component | nRF52840 | ESP32-S3 |
|---|---|---|
| Available for the protocol | 48 KB | 128 KB |
| Thread cache (LRU, section 8.6) | 24 KB | 64 KB |
| Dedup cache (512 entries of 4 bytes) | 2 KB | 2 KB |
| Neighbor table and known graph (256 nodes of 12 bytes) | 3 KB | 3 KB |
| Link list (1536 links of 5 bytes, local format) | 7.5 KB | 7.5 KB |
| Free (chunk reassembly lapses with the self-contained chunks, 4.3) | 0 | 4 KB |
| Queues and transmit buffers | remainder | remainder |

### 13.3 Design rules

- **Language:** firmware in C or C++. Python is for simulation only. The chip has too little RAM for an interpreter, and the timing (ack timeouts, CAD, channel switches) requires predictable handling via interrupts.
- **Shared protocol core:** the core (hello, neighbor table, acks, routing, pruning, caches) is a single C/C++ library. The simulator calls that core via cffi or ctypes. This way the simulation does not run on its own implementation, which could drift from the firmware.
- **Hardware layer:** radio, timers, RNG and flash sit behind a thin interface. Only that layer is replaced per platform.
- **No dynamic memory allocation:** all buffers are fixed and bounded, in line with the budgets above. Heap fragmentation on 256 KB is a real risk.
- **Geo calculations with integers** where possible, using the 16-bit values from section 5. The east-west axis scales with cos(latitude). Within a region that is a constant, so a fixed fraction suffices (for example 154/250 ≈ cos 52° for the Netherlands).
- **Random numbers** from the hardware RNG for backoff and jitter.
- **One radio abstraction** for SX126x and SX127x. The modem parameters are the same.
- **Time** via a local counter and a hardware timer. No clock is needed.
- **Bounded known graph:** the number of nodes is capped, so that the Gabriel calculation, the kNN radius and route computation stay within the computing budget.

**Status:** budgets decided. Open: whether a BLE companion on the same chip competes with the protocol for RAM.

---

## 14. Simulation

### 14.1 Layers

1. Radio and channel: airtime, path loss, capture effect, duty cycle.
2. Hello and neighbor table.
3. Data, ack1, ack2, retries and budget.
4. Routing: map, bridge-node rule (geo+Dijkstra), failed reports.
5. Gaps and perimeter (IJsselmeer).
6. Threads: radius, pruning, ANN/FULL, recovery.
7. Churn: outages and returns.

### 14.2 Measurements

- Delivery ratio per message (ack3 where requested, otherwise ack2).
- Cause of failure: collision, no neighbour, budget exhausted, gap.
- Airtime per channel and duty cycle.
- Budget consumption per message.
- Latency (for information).
- Duplicate work in threads: ratio of FULL transmissions to the number of nodes.
- Proof-list packets per node per channel (both triggers: `T_sym` flush and `T_keepalive` pulled forward, 4.1/4.2), and share of empty (keepalive-only) lists.

**Topology:** real repeater locations where available, otherwise synthetic with the same density per region. Water is excluded (for example via OpenStreetMap).

**Calibration:** the SNR thresholds of the Dutch map (poor below −2.5 dB, good above 3.5 dB) and the measured channel occupancy.

### 14.3 Build plan (proposal)

**Architecture.** Event-driven discrete simulation in Python. The shared C protocol core (13.3) is called via ctypes or cffi. Each simulated node has its own instance of the core, with no shared state. The simulator supplies the clock, the radio and the RNG to the hardware interface of the core. Radio events: `tx_start`, `tx_end`, `rx_begin`, `collision`. Time in microseconds, to avoid rounding errors in long runs.

**Layer 1: radio and channel (build first).**

| Quantity | Model | Value (SF7, 62.5 kHz, CR4/5) |
|---|---|---|
| t_symbol | 2^SF / BW | 2.048 ms |
| hello (3 B) | LoRa airtime | 62 ms |
| ANN (5 B) | idem | 70 ms |
| ack1 / ack2 / hello ack (5 B) | idem | 70 ms |
| data (100 B) | idem | 459 ms |
| FULL / chunk (200 B) | idem | 869 ms |
| Hello round (3 channels + pauses `g`) | 3 × 62 ms + 2g | about 186 ms + pauses |
| Successful reception | SNR = signal − noise + path loss (path, antennas); thresholds −2.5 / +3.5 dB | NL calibration |
| Collision | Same channel and overlapping time. Capture effect: the stronger wins with a margin of about 6 dB (to be calibrated). | Proposal |
| Channel switch `S` | Fixed, for example 5–10 ms including frequency settling | Open |
| Duty cycle | Per node per channel, 10%, window of 1 hour | Decided |

**Radio model (proposal, Oct 2026; NL calibration open).** Scenarios with more than a few nodes use a node list with GPS position and height (`id,lat,lon,height_m,name`). The link budget per direction:

- reception = TX (27 dBm ERP) − FSPL(869 MHz) − 2 × clutter(h), with clutter(h) = 30 − 14 · log10(1 + h) dB;
- radio horizon: d ≤ 4.12 · (√h1 + √h2) km, including refraction;
- reception succeeds above −110 dBm, with fade margin included;
- per direction a deterministic fading offset of ±5 dB, so that links are asymmetric as they are in practice.

Roughly calibrated: masts of 20–40 m reach about 12–14 km; tall masts (300–375 m) cover a province.

**Layers 2–3: hello, table, data and acks.** The core runs the real state machines: adaptive hello interval, RSSI-driven spreading of hello acks, switching rule, ack1/ack2 with retries (3 neighbours × 2 attempts), pending records with `T_ack2`, and budget 32. On top of the core, the simulator measures home deafness per node, and failed unicast transmissions by cause (collision, no ack1, neighbour gone).

**Scenarios (in increasing order):**

1. **Link test:** 2 nodes, hello, table push and one unicast. Checks that the C core and the simulator agree on packet sizes, airtimes and ack order.
2. **Local network:** 30–50 nodes at one density, from a node list with GPS position and height. Measures hello airtime and ack collisions on hello acks (RSSI spreading), proof-list packets per node per channel (both triggers), and the steady-state ack load on generalised triggers (ANN/FULL) against the schedule (0.12–0.17%).
3. **Density limit:** 200, 500, 1000 and 3000 nodes, synthetic Dutch distribution. Airtime per channel, duty cycle peaks, and delivery ratio.
4. **Sparse area and gap:** IJsselmeer/Flevoland pattern. Radius k, waypoints, failed reports.
5. **Churn:** nodes drop out and return (winter, faults). Tombstone behaviour and recovery.
6. **Star rounds (4.3/4.4, replacement):** the uniform convergence cadence as the replacement of the chunk-push cadance. **A/B over two builds, not two code paths**: the star-round build (the PR branch, integral replacement — no gates) against the chunk-push build (current main, with its existing utrecht40 measurements: rest 0.26-0.55% chunk airtime, sym gone in 4-6 h, map empty in 24-30 h). Same simulator, same topology files, same seeds, same metrics — the comparison is apples-to-apples and stricter than two paths in one binary, because no shared code can behave differently per arm. Gate: the replacement merges only when scenario 6 succeeds in full (see acceptance criteria); until then the old cadance keeps running on main. Sub-scenarios, each building on the previous:
   - **6a. Unit-level correctness** (utrecht40, short): star canonicalisation and sorting; digest derivation `(content_hash + tick) mod 256`; tick advance and derivable match; wrap at 256 within the k-window; missed-list request, counter reset and reply via the regular round; queue priorities (own star front, nearest hops first, tick-lag and mutation ordering). Multi-hop first finding (10-node line, Oct 2026): the tick reaches hop 2 but the hop-3 chain stalls with 2481 missed requests over 6 h — cause analysis: the content hash differs per observer (each hop knows a different link set around the star's centre), making digest matching brittle over chains, and the miss list re-creates cleared entries every round. **The fix (hash over the origin's advertised form, and/or a recently-arrived window on miss creation) is a precondition for the rest of scenario 6.** Checked against the 4.3 text point by point.
   - **6b. Cold start** (utrecht40 + a 300-node synthetic set): full-table convergence time from empty (target: within one backlog bound, order 7-8 h at 15-min interval; hard bound never violated: max one chunk per node per interval, measured); airtime per channel; queue fairness (no tail starvation: every map star forwarded at least once within X intervals, X to be fixed after the first run).
   - **6c. Steady state and heartbeat** (utrecht40, 48 h rest): catalogue share of the round at rest (target: rounds almost purely catalogue); tick propagation hop-by-hop (target: derivable matches dominate, content re-sends at rest near zero); age freshness of the map versus the current core; remote liveness: silence a node, measure how its star's tick stalls across the network versus the old 4.4's 4-6 h local-only detection.
   - **6d. Churn** (utrecht40 + synthetic, 1-3% per day): mutation latency from local detection to network-wide (target: hop distance x interval); missed-list load and damping under sustained churn (no request storms: missed requests per node per hour bounded); reboot/reincarnation: kill-and-restore a node with a persisted table (epoch unchanged) — convergence within a few intervals; same node with a lost table — falls back to the newcomer pull, no missed-list flood.

      **Measured (Oct 2026, star arm, utrecht40, 22 h + 2 h warmup, 1 geo-mutation/h, background unicast every 60 s; the churn sweep --churn 0/2/5/10/20 % of the nodes per day, half of the leavers returning after 0.5 h (within the liveness threshold), half after 8 h (reincarnation-scale)):**
      | churn/day | delivery | missed requests | per node/h | airtime/channel | liveness evictions | hard bound |
      |---|---|---|---|---|---|---|
      | 0% | 99.9% (1289/1290) | 1132 | 1.3 | 928 s (~1.1%) | 10 (baseline) | 0 |
      | 2% | 96.7% (1245/1287) | 2921 | 3.3 | 1035 s (~1.2%) | 61 | 0 |
      | 5% | 96.3% (1236/1283) | 2946 | 3.4 | 1064 s (~1.2%) | 68 | 0 |
      | 10% | 95.6% (1225/1281) | 3301 | 3.8 | 1115 s (~1.3%) | 61 | 0 |
      | 20% | 93.6% (1198/1280) | 3845 | 4.4 | 1161 s (~1.4%) | 151 | 0 |

      Readings: the design band (1-3%/day) sits at 96-97% delivery with missed requests around 3.3/node/h - the damping holds, no request storms, no hard-bound violations anywhere in the sweep. The failure budget up to 20%/day degrades gracefully (93.6%, ~4.4 requests/node/h) - the mechanism saturates softly: failures are 'schedule exhausted' (the attempt schedule gives up after its alternatives), not graph loss. Airtime rises ~25% from rest to 20%/day (the death/return traffic rides the rounds), staying well under the duty budget. The liveness evictions scale with the churn (the returning-after-8h nodes were correctly declared dead network-wide and re-enter via the newcomer path). Open: the hop-scaled liveness threshold (4.4) against this data; and the FAIL causes of the 2%+ runs (route flapping around freshly-returned nodes) are the natural next measurement.
   - **6e. Quiet-hour acceleration** (utrecht40, day/night traffic profile): interval shortening on low occupancy (floor 5 min); duty-cycle share stays bounded; convergence of the residual backlog during quiet hours.

      **Implemented and measured (Oct 2026).** The round timer halves the interval when the last round left content work (star_backlog: packed stars awaiting channels, or a queue that did not fit) AND every channel has been quiet for MC_STAR_QUIET_TICKS (~4 min without any identifying reception - the ka_idle counters carry the occupancy measure, no new mechanism). Floor: 5 min (12 rounds/h). At occupancy the full 15-min interval stands; the acceleration self-clears at rest (no backlog, no shortening); the hard bound is untouched - at most one chunk per round, just sooner.
      Measured (utrecht40, day/night profile: unicast every 60 s by day, quiet by night, a 13-node geo-mutation wave just before nightfall, 507 star-views to converge): post-wave form visible at 22.7% of observers at nightfall, 92.5% after 30 min, **100% within 1 h of quiet hours** - the residual backlog drains at up to 3 rounds/h/node in the quiet phase (vs 4/h at the fixed interval; the fixed-interval baseline from 6d is ~45 min per hop at rest, so the same wave would take several hours). Round rate 3.9/node/h average over the whole day/night cycle, duty max 1.8 s/h per node-channel (limit 360), hard bound 0 violations. Day-traffic keeps the gate closed (ka_idle stays low under unicast traffic) - no acceleration airtime in busy hours.
   - **6f. Dense-cluster stress:** the per-node hard bound is not a per-channel bound — a dense single-channel cluster (n = 64 fully meshed) measures aggregate channel occupancy; if it exceeds the duty-cycle budget, the interval scaling (open point 1) gets measured here before adoption.

      **Measured (Oct 2026).** 64 fully meshed nodes (all pairs audible, 30 m masts, one ring), all at home on ONE channel, 12 h, a 16-node geo-mutation wave at half time (1,008 star-views to converge). Results: the shared home channel carries **7.94%** occupancy (the other two, rotation guests only: 6.1%) - under the 10% duty budget, with the arithmetic exactly as predicted (64 nodes x 3.9 rounds/h x ~0.5 s air); per-node duty max 3.5 s/h (limit 360); convergence 100% within 30 min of the wave (one hop - the rotation cycle); map 64/64 with 1,506 links/node, zero ghost entries; missed requests 36 total; hard bound 0 violations. The margin: ~20% under the budget WITHOUT user traffic and without quiet-hour acceleration - a real tower with traffic and a nightly backlog can exceed it. This is the measured input for open point 1: the interval should scale with the local channel occupancy (e.g. doubling to 30 min at ~64 neighbours-on-channel halves the 7.94% to a safe ~4%); the mechanism is a local parameter change (the round timer's interval), to be calibrated in a follow-up sweep. All other behaviour (convergence, integrity, damping) IMPROVES at density - occupancy is the only scaling axis.

   **Acceptance criteria (all must hold before anything in 4.3 is replaced):** (1) delivery ratio of routed traffic not worse than the 4.3 arm in every sub-scenario; (2) airtime per channel at rest not higher than the 4.3 arm's 0.26-0.55%; (3) convergence at least as fast: cold start within the backlog bound, mutations network-wide within (hop distance x interval); (4) no starvation: every star forwarded within the fairness bound; (5) no storms: missed-list requests bounded under churn and reboot; (6) the hard bound holds in every run: never more than one content chunk per node per interval. **Parameters to sweep:** interval (15 min starting value), tick period (1 h), k-window (8), digest width (1 B), missed-list cap (~12) and attempt limit (1-2), floor interval (5 min).

**Measurement report per run:** delivery ratio (ack3 or ack2), failure causes, airtime and duty cycle per channel, home deafness, budget consumption, latency, and duplicate work in threads.

**Starting point:** scenarios 1 and 2 can run without thread logic. Section 8 comes into play at scenario 3 and beyond. This gives an early working base that rigorously tests the choices for radio and acks (62.5 kHz, switching rule, `T_ack2`).

---

## 15. Changed and dropped ideas

| Idea | Status | Reason |
|---|---|---|
| Gossip routing as the main route | Dropped | Chose link-state with a map |
| Distance-vector | Dropped | Replaced by the map |
| Fixed geographic cells for chat | Dropped | Density varies too much |
| Shared chat channels (#public, #test) | Dropped | Broadcast storm |
| Channel per chat region | Dropped | Replaced by threads |
| Flood as fallback | Dropped | Does not scale |
| Path in the header | Dropped | Costs more than it gives |
| Single-hop ack for failure reports | Replaced | Routed report is better |
| Complete tablepull mechanism (pull request, repair pull, ack per chunk, startup pull as symmetry closer) | Replaced (Oct 2026) | Push to all neighbours: the information is the same for everyone, a hub transmits its table once instead of n times. Chunks are themselves symmetry evidence (same principle as the proof list, 4.2); recovery runs via the regularity of the cadence, not via requests |
| Encrypted geo | Dropped | Relays need the geo |
| Repeater WiFi on by default | Dropped | Power consumption on solar; now off by default |
| Repeater storage for companions | Dropped | Companions retrieve threads from their repeater's thread cache |
| "Hello on demand", rules 1 and 2 (stale-neighbour and version triggers) | Replaced (Oct 2026) | Snooping and the generalised hello ack cover liveness. The per-channel keepalive goes via the proof list (4.1, 4.2). |
| Hello ack only after a hello | Replaced (Oct 2026) | Generalised to every identifying home-channel transmission, with the ack eligibility condition and cooldown; at the time including a re-confirmation flag, later replaced itself (row above) |
| Flag bit "confirm this" + reply rule | Replaced (Oct 2026) | The proof list (4.2) maintains the evidence without a flag, without a reply and without dependence on ack chances; same air per link, fewer packets on hubs |
| Full neighbour ID list in the keepalive | Dropped (Oct 2026) | Airtime is superlinear (5 B = 70 ms, 133 B = 459 ms) and the packet goes to everyone on every channel every hour; the "last hour" filter filters nothing, because through the keepalive every living neighbour is by definition heard within the last hour; routing needs quality/geo/direction. The delta variant (proof list) covers the maintenance; the table chunks (4.3) cover the full information |
| Removal by silence with tombstone | Replaced (Oct 2026) | Silence is death detection (4 hours, 4.4) without a tombstone. The tombstone stays for neighbor table limit churn on hub sites (10.4). |
| Chunk-push table sharing: bundling windows, F_DELTA/F_NEG delta chunks, periodic prefix re-share, tail cursor, travelled age on air, per-entry age encoding (the old 4.3/4.4) | Replaced (Oct 2026, pending scenario 6) | The star round (4.3) subsumes the cadance: one chunk per interval carries content (mutation = content change of the own star, no delta packets), the catalogue is the age maintenance (no re-share needed), the missed list is the repair (no repair pull), and the heartbeat tick replaces travelled age — liveness and freshness separate (4.4). The old text (chunk format, three-speed cadence, age encoding, tombstone discussion) is archived in the git history of this document at the commit that introduced sections 4.3/4.4 "Star rounds" and "Ageing and death"; the scenario-6 measurements of the old cadance remain the A/B baseline (main). |
