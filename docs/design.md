# Meshimeshi: Design Document (revised)

**Status:** working document, not yet simulated
**Language:** English
**Last updated:** 15 October 2026

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
- **Table chunks:** a part of a node's own neighbor table, broadcast unsolicited on the home channel. (Section 4.3.)
- **Proof list:** a small packet with the node IDs of neighbours whose reverse-direction evidence expires soon. (Section 4.2.)
- **Failed delivery report:** a routed message sent back to the source when a message could not be forwarded after all retries. (Section 10.5.)
- **Budget:** a counter in the header that limits how many forwards and retries a message may use. (Section 9.)
- **Pending record:** the small record a forwarding node keeps for a message until its ack2 window closes. (Section 7.2.)
- **Dedup cache:** a cache of recently seen message IDs, used to discard duplicates. (Section 8.3.)

### Neighbours, links and routing

- **Neighbor table:** the list of direct neighbours a node advertises, with quality and age for each. (Section 4.2.)
- **Local table counter (not on air):** an internal counter that a node uses to bundle table mutations within the push window. It is not transmitted; freshness on air is carried by the per-entry age encoding (4.4). (Section 4.3.)
- **Link:** a radio connection between two nodes. Each direction is measured separately. (Section 4.2.)
- **"Seen" versus "confirmed symmetric":** the two states of a neighbour relationship. "Seen" means one direction has been heard. "Confirmed symmetric" means the other direction has been proven by the neighbour itself. Routing uses only confirmed links. (Sections 4.2, 10.1.)
- **Ack eligibility:** the three conditions (need, pace, spacing) that decide whether a received transmission leads to an ack at all. (Section 2.)
- **2-hop map:** the set of nodes a node knows about: its direct neighbours with full tables, and their neighbours by ID, geo and link. (Section 10.1.)
- **Neighbour table limit:** the maximum size of a node's neighbor table (64). When it is full, the weakest neighbours are removed. (Section 4.1, 10.4.)
- **Tombstone:** a short-lived block on a neighbour that was removed because of the neighbour table limit, so that it does not reappear immediately through other tables. (Section 4.4.)
- **RSSI / SNR:** received signal strength and signal-to-noise ratio, the radio measurements used to rank links and responders. (Sections 2, 4.2.)
- **ETX-like quality:** a link cost based on the expected number of transmissions needed. Used to pick the best route. (Section 10.2.)

### Geography

- **Geo:** a 4-byte position (latitude and longitude at a deliberately coarse 306 m grid). (Section 5.)
- **Gabriel graph:** a way of deciding which links are needed to connect nodes around a gap, based on geometry. (Section 10.3.)
- **Perimeter routing (GPSR):** a fallback for routing around a gap when no neighbour is closer to the destination. (Section 10.3.)
- **kNN radius:** the radius of a thread, set by the distance to the k-th nearest node around the starter. (Section 8.2.)
- **Waypoint:** an intermediate geographic point that a source attaches to steer a message. (Section 10.2.)

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

- **Size:** 3 bytes. Contents: type (1), node ID (2). Dropped (with it the state version): the version-triggered table fetch no longer exists — the table push (4.3) carries mutations unsolicited, and freshness on air is carried by the per-entry age encoding (4.4). A geo change no longer needs a version increment; the geo travels in the table chunks.
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
| Age | 1 byte | Minutes since the neighbour was last heard. 255 = 4 hours or older, hence expired (4.4). |
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

### 4.3 Sharing tables (push)

- **Push, no pull (decided, Oct 2026).** The table information is the same for all neighbours, so the owner shares its table unasked: every chunk is a broadcast on its own home channel, in the fixed cadence (below). Neighbours snoop along. There is no pull request, no repair pull and no ack on chunks; missed chunks are not requested but return by themselves at the next mutation or the next round.
- **Table chunks are themselves symmetry evidence (decided, Oct 2026).** When a node publishes "I hear X" (its entry for X with quality), then at X, in a single reception, the direction X→node is confirmed (the claim of the node) and the direction node→X is freshly measured (the reception itself) — the same working principle as the proof list (4.2). The startup pull thereby lapses as symmetry closer: with a new node, its startup hellos provide the acks addressed to it (its own evidence), and its first table push provides the responders their counter-evidence. The route via the hello process (the newcomer acks a later home-channel transmission of the responder, section 2) remains standing as an extra source.
- **Forwarding versus direct replies (decided, Oct 2026).** The symmetry requirement applies to choosing a next hop for data (routing, 10.1). A direct reply within an exchange (ack1, ack2) may also be sent by the responder if it has the partner only as "seen". The received request proves the partner→responder direction, the partner is already waiting on the channel, and the ack1 on the reply supplies the missing evidence afterwards. Refusing would block the startup of new nodes.
- The table header contains the absolute geo of the owner and a firmware version of 1 byte. The version thus travels with every table push, without extra bytes per packet.
- The table also contains the links between the owner's neighbours. The owner already knows these from its neighbours' tables, so no extra pull is needed. Per link, the node IDs of both endpoints travel along (absolute IDs, no indices; see the chunk format below), with the quality in both directions. This way a node knows all links within its 2-hop set, and the Gabriel graph around a gap becomes complete. Nodes three hops away are unknown by default. They only come in through maintenance during quiet periods (section 12, point 15).
- Sharing tables from second-hop nodes does **not** happen (that information already travels along via the tables of direct neighbours). Strategically expanding the routing table during quiet periods can be considered later.
- At startup, the first table chunks are scheduled with a random delay, so that they do not collide with the hello round and other newcomers.
- Tables larger than one packet go in chunks of about 200 bytes, with sequence numbers.

**Chunk format (Oct 2026: self-contained, node-centric chunks).** Every chunk is processable in its entirety without information from other chunks; the order does not matter and the receiver does not need to know whether and when the rest comes.

- **Node-centric.** A chunk contains a small number of complete node entries (8 B: node ID, quality, age, geo), and directly after each entry the links of that node: `(neighbour ID 2 B, quality bits per direction)` — width depending on the chunk type, see the quality width below. Every link stands under exactly one endpoint (the owner chooses), so every link travels once per table.
- **Absolute IDs, no indices.** Link references are node IDs, not positions in the sender's table. No mapping between the tables of sender and receiver is needed, and no temporary scratch buffer or reassembly: every chunk is processed directly against the own table (256 nodes, 13.2).
- **Chunk header** (about 8 B): type, chunk number, number of nodes, and the owner geo with firmware version (4.3). The header repeats per chunk, so every chunk is self-contained. Dropped: the state version — no receiver pulls a table, and freshness travels per entry via the age encoding (4.4).
- **Quality measurement (decided, Oct 2026): long-term average per direction.** The first measurement initialises the stored value; every subsequent reception updates it with an EWMA: `stored = (31 × stored + new value) / 32` (weight 1/32, a window of roughly half an hour of traffic). Small quality excursions are thereby damped automatically. There is no age per link: the age of a link is the age of its source, and that travels along per node entry.
- **Reverse quality via the acks (decided, Oct 2026).** The hello ack and ack1 carry a quality byte: what the acker measured on the acked message. The sender of the original thereby learns the reverse direction of its own link immediately, without waiting for a chunk. A chunk entry also names our reverse class (its measurement of us) in its low nibble — refreshed with every chunk that names us. The chunk entry byte thereby carries both directions of the owner→neighbour link; a receiver of the chunk sees both, and the owner's own reverse knowledge comes from the acks. Measured (utrecht40, 6 h, no traffic): reverse quality known for 91% of confirmed links after 5 minutes, 100% after 6 hours; agrees with the neighbour's own measurement in 142/142 cases within one class.
- **Quality on air (decided, Oct 2026): hybrid width per chunk.** The advertised link quality is the upper bits of the long-term average: 3 bits per direction in the prefix chunks (direct neighbours and their mutual threshold links — the links where fine-grained route choices are determined; 8 levels), 2 bits per direction in the tail chunks (2-hop links serve greedy direction and Gabriel witnesses, not hop selection; 4 classes per the NL calibration 14.2: bad below −2.5 dB SNR, moderate, good above +3.5 dB, excellent). The chunk header gets a flag bit for the quality width, so every chunk stays independently decodable. With 3 bits per direction (prefix) a link is 22 bits wide; with 2 bits (tail) 20 bits — both packed on a byte boundary (8 links per 22 B and 4 links per 9 B respectively). Budget: the full table thereby stays around 30 to 31 chunks, within the 32.
- **Advertisement hysteresis (decided, Oct 2026): share only on difference ≥ 2.** A change in the advertised quality is only passed on when the new advertised value differs from the previous one by at least 2 steps. The average may swash over a class boundary without triggering a single packet; the advertisement hysteresis is independent of the measurement hysteresis for the neighbor table limit (10.4) — two dampers with two goals.
- **Status flags do not travel along.** Seen/confirmed, the quality threshold and Gabriel membership are derived: the owner only sends links it has measured in both directions, and every receiver calculates threshold and Gabriel graph itself from class and geos (10.1, 10.3).
- **Broadcast per chunk.** Every chunk is its own broadcast on the own home channel; no exchange, no ack, no switch. Snooping processes it as liveness + quality for the sender (the chunk is an own call and resets the keepalive counter of that channel, 4.1).
- **Loss (decided: no repair pull).** A missed chunk is not requested. Recovery consists of the regularity of the push itself: the prefix is shared at the next direct-neighbour mutation or when the window expires (the whole prefix again), the tail is periodically repeated in full in the cadence. Because every chunk is self-contained processable, there is no order or reassembly dependence and no extra state. The price: the receiver does not know THAT a chunk was missed (no ack), so the age of its stored copy is the only measure — older information loses automatically against fresh.
- **Prefix: the routing core first (decided, Oct 2026).** The fixed table layout starts with the direct neighbours and their mutual threshold links (above the quality threshold, 10.1). The first k chunks contain that routing core (typically 8 to 16 neighbours in one chunk; a hub with 64 neighbours has a prefix of several chunks). Chunks remain independently processable; the prefix property is about the layout of the sender. A receiver with only the prefix can already route fully; the tail is best-effort.
- **Cadence (proposal, Oct 2026): three speeds, driven by the kind of mutation.** The distinction between direct neighbours and other nodes determines how fast a change is shared:
  1. **Prefix on direct-neighbour mutation (fastest):** new neighbour, death detection (`T_expire`), limit churn (tombstone), or a quality-class change that survives the hysteresis (10.4) — the prefix follows directly behind it, within the short window. These are the events that touch routing: a new or dead direct neighbour changes the possible hop selection for everyone in the area. Delaying there is withholding route information.
  2. **Prefix otherwise:** mutations that only affect the edge (new 2-hop node at a neighbour, geo change) follow the normal short window (at most once per 5 to 10 min).
  3. **Tail:** exclusively the rising cadence (below).
  Mutations within the window bundle automatically, because only the current state goes on air — intermediate states do not exist on air. The kind of mutation is exactly known locally (the node knows itself what it just changed), so no protocol field is needed. The local table counter (glossary) drives this bundling; it is not on air, because no trigger has to be visible on air anymore: there is no pull, and a receiver cannot miss a chunk knowingly (no ack). The tail is shared more slowly, with a pause between chunks that rises with the number of direct neighbours n (per doubling of n the pause doubles): the marginal airtime cost of a chunk rises with n, most table mutations sit at hub sites, and no node has to do a busyness measurement (open point 15). Guideline (Oct 2026): order of magnitude 32 hours before a tower site (n = 64, about 30 chunks) has shared its full table — base pause about 8 min, doubling per bucket (n ≤ 8 → 8 min, ≤ 16 → 16 min, ≤ 32 → 32 min, ≤ 64 → 64 min per tail chunk). Parameters: window length, base pause and bucket boundaries — to be measured in scenario 2.
- **The slowness is sharing delay, not aging at the source (Oct 2026).** The knowledge of the sender is fresh at the moment of sending: it hears its own neighbours directly, and its 2-hop edge is supplied to it by the prefixes of its neighbours (every 5 to 10 min) plus everything it snoops. A tower with many neighbours snoops more sources and thereby on average has a *fresher* edge than a small node; the rising pause only delays the sharing, not the knowledge. The receiver can subtract the sharing delay locally, because the sender's n is in its prefix. Every receiver computes: actual age = travelled age (sender timestamp, 4.4) + expected sharing delay f(n). No protocol field, no clock, no synchronisation — only the bucket table in the firmware. Tail entries are thereby not filtered away but weighed: routing uses them for direction (greedy, Gabriel), not for hop selection; the symmetry requirement (10.1) already restricts hop selection to confirmed direct neighbours anyway.
- **Budget.** Full table: 256 nodes × 8 B plus links (prefix 22 bits, tail 20 bits, see the quality width above) plus headers ≈ 5.7 to 6.0 KB, so 28 to 31 chunks of the 200 B class (869 ms), within the 32 of the memory budget. A full push round of a tower (about 30 chunks) is thereby about 26 s of air per 32 hours — negligible. The receiver does not store links it already has; only the difference counts.

**Status:** decided. Implemented (Oct 2026): prefix chunks — node entries (id, quality class, age, geo) with their advertised links, self-contained per chunk, a neighbour ADDITION first sends a tiny delta chunk (F_DELTA, one entry, 19-20 B, 173 ms - a new neighbour has no links in our store yet), immediately on confirmation; other mutations and the full-prefix consistency rounds follow after an adaptive bundling window (Oct 2026): small tables (<= 8 chunks) 15 min + jitter — fast convergence where the airtime allows it; larger tables 1 h + jitter — sustained churn on a dense hub stays under 1% of a channel even for a fully meshed 64-hub (0.75%); mutations inside the window keep bundling) — all chunks back to back; the periodic re-share runs every ~6 h (per-node phase offset), so every chunk comes by at least a few times per day at rest (all three channels in fixed order, starting at (home + 1) mod 3 so the round ends on the own home channel - no leftover switch, and same-channel neighbours can react immediately; the hello-round reason: confirmed neighbours sit on all home channels; a chunk on the own home channel alone would keep the cross-channel links — exactly what Gabriel and route choice need — invisible). Only confirmed symmetric neighbours travel along as ENTRIES (decided: no "heard only" in the chunks). Link endpoints beyond the 2-hop set are learned as ordinary map nodes (id, geo, quality once heard directly): "3-hop" in the sym graph does not mean out of radio range, and map material is never wasted. Their link quality toward us is simply not routing information until the link itself is confirmed (10.1). The receiver builds the 2-hop map and link store from the entries; a chunk naming us confirms our link to the owner, gated on quality like every confirmation (10.1). Tail chunks implemented: the 2-hop map as full entries (F_TAIL flag in the type byte, same format), one tail chunk per periodic re-share via the tail cursor, so the full tail passes in a few days at rest; the airtime at rest is unchanged because the tail cycles through the re-share moments. Measurements (utrecht40): rest 48 h - full graph: node coverage 100%, link coverage 100% (incl. all B-C links between 2-hop nodes); links per node in the store 363.5 avg (8 h); chunk airtime 0.262% of a channel; duty max 0.1 s/h. Rest 6 h — chunk airtime 0.27% of a channel (was 2.18% with the 10-min re-share; the whole-prefix mutation push + 6-h re-share cut it by a factor 8), total rest airtime 0.55%, duty max 0.1 s/h, node coverage of the 2-hop set 100%. Worst case for all direct neighbours in chunks: 64 confirmed neighbours take 3 chunks without mutual links, ~11 at 30% meshing, ~31 fully meshed (4.3 budget).

### 4.4 Expiry, tombstones and stale entries

**Death detection (proposal, Oct 2026).** For each direct neighbour, a node locally records when it last heard it. Every identifying transmission that is overheard counts. The clock is local, so no synchronisation is needed. After `T_expire` (starting value 4 hours) without reception, the neighbour is considered dead and the entry disappears, without a tombstone.

- `T_expire` is at least three to four times `T_keepalive` (4.1). A living, quiet node sends at least its proof list on each channel once per hour (4.2, combined trigger). Four hours therefore means four missed keepalive moments, on top of all traffic that is overheard.
- In a mass outage (for example a power failure in a region), hundreds of entries expire at the same time. Every mutation feeds the local table counter, and the push wave that follows must stay spread out through the existing random delay and the cadence (4.3).
- The age per entry remains the measure of what a receiver trusts.

**Age encoding (proposal, Oct 2026).** The age byte per entry encodes minutes; 255 = 4 hours or older, hence expired. For direct neighbours (prefix chunks), receivers thereby filter out expired entries automatically. For tail entries the rule is weighing, not discarding (4.3): link qualities of expired entries are not used for routing, but the node information (ID, geo) stays standing as map material.

**Why death detection needs no tombstone.** A dead neighbour disappears from every advertised table no later than `T_expire`, because every owner applies the same rule. The age encoding ensures that receivers ignore expired entries. A neighbour that lives only in the 2-hop map (Y hears him, we do not) is not a zombie but legitimate information with a fresh age. A tombstone covering the whole table would wrongly block him.

**Tombstone, limited to its actual task (Oct 2026).** The tombstone exists for removal by the neighbor table limit on hub sites (10.4). That affects a living neighbour that keeps transmitting. Without a tombstone, the neighbour would return within the keepalive interval, the neighbor table limit would oscillate, and every mutation would cause a push wave.

- The tombstone blocks entries that arrive indirectly via tables. A direct hello from the node itself lifts it.
- Duration: 5 times the maximum keepalive interval, so 5 hours at `T_keepalive` = 1 hour.
- Removal through silence does **not** use a tombstone.
- The number of tombstones is limited by the memory budget.

**Status:** proposal for `T_expire` and the age encoding (to be tested in scenario 5). Tombstone duration decided.

---

## 5. Geo encoding

| Property | Value | Status |
|---|---|---|
| Range | Worldwide | Decided |
| Latitude | 16 bits: 180° divided over about 65,536 steps, ≈ 0.002747° ≈ **306 m** per step | Decided (Oct 2026) |
| Longitude | 16 bits: 360° divided over about 65,536 steps. **Maximum ≈ 612 m per step at the equator** (worst case), ≈ 376 m in the Netherlands (52° N). Shrinks with cos(latitude). | Decided (Oct 2026) |
| Total size | **4 bytes** (16 + 16 bits, no flag byte) | Decided (Oct 2026) |
| Invalid geo | Sentinel (0, 0). This decodes to (−90°, −180°), the South Pole/date-line corner, where no node ever stands. A reserved or cleared entry is therefore automatically "no geo". | Decided (Oct 2026) |

**Change from the previous form (5 bytes: 18 + 18 bits + 4 flag bits).** Geo becomes 4 bytes, per neighbor table entry and in the optional source geo of every data header. The grid becomes four times coarser: 306 × 376 m in the Netherlands instead of 76 × 90 m. This is enough for all network purposes. Greedy routing (10.2), the Gabriel graph and the kNN radius work at kilometre scale, and coarser geo is better for privacy (goal 7). For display purposes, a companion keeps its precise GPS locally. The earlier goal of "about 100 m" has therefore been abandoned for the network function. The FULL root geo (6.5) already used this format.

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
| **Total** | **10 bytes** |

Optional: source geo (4 bytes, section 5), present when the flag is set. This is needed when the source falls outside the destination's map and the destination must send an ack3 back.

**Flags in the type byte (proposal):** ack3 requested, source geo present, ack2 negative, broadcast.

### 6.2 Acks

| Type | Size | Direction | Channel |
|---|---|---|---|
| ack1 | 8 bytes: idem + quality byte — the RSSI quality the acker measured on the data; the sender learns the reverse direction of the link in the same exchange (decided, Oct 2026) | Next hop → previous hop | Same channel as the data exchange; the sender waits there according to the switching rule |
| ack2 | 7 bytes | Next hop → previous hop | Home channel of the previous hop |
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
3. As soon as B has received ack1 from D, it immediately sends ack2 to A, on the home channel of A.
4. A then knows the message has come two hops.
5. D sends an ack3 to the source, if requested. That is a routed message.

**Status:** decided.

### 7.2 Meaning of ack2

- **Positive:** the next hop has **received** the message, not necessarily forwarded it. This was a deliberate choice, so that ack2 does not have to wait for the whole chain.
- **Negative:** the next hop could not forward the message. A can then choose a different route immediately.

**Responsibility per hop (bounded backtrack).** The node that receives the message does not store the message. It stores only a **pending record** `(message ID, previous hop, next hop, counter)` in a fixed buffer of, for example, 16–32 slots (FIFO: when full, the oldest is dropped). That record lives until `T_ack2` of its own hop expires or the retry sequence has finished. After that, only the dedup cache entry remains.

- If forwarding fails after the retries, it tries another neighbour (backtrack within the window).
- If that also fails, or if the next hop goes down after the window, it sends a failed delivery report to the source, as a routed message. That report does not itself generate a new report.

**Deliberate cost:** after a positive ack2, the source can no longer fall back to the previous hop if the failure occurs later and deeper. The only signal is then the failure report or the absence of ack3. There is therefore one return channel for late failures: the routed failure report.

**Status:** decided. Late negative notifications reach the source via the routed failure report (see 10.5).

### 7.3 Timeouts

| Timeout | Formula | Status |
|---|---|---|
| ack1 (short) | `t_data + t_ack + processing + margin`, measured from the end of the own transmission | Decided (principle) |
| ack2 at A | `T_ack2 = 6 × (t_data + T1) + 5 × B_max + S + t_ack + m` | Proposal |

- `T_ack2` does not depend on the path length, because A only looks at B. `T_ack2` is also the lifetime of the pending record at B: one timer with two functions (timeout at A, and the window for backtrack at B).
- `S` is the channel switch from the channel of the next hop back to the home channel of A.
- If B itself fails, no ack1 comes. A notices this at the ack1 timeout.
- If B exhausts all six attempts, it immediately sends a negative ack2.

### 7.4 Retries

- At most three neighbours per hop.
- At most two transmission attempts per neighbour.
- Ack1 received but no ack2: switch to the next neighbour.
- Negative ack2: move to the next neighbour immediately, without waiting.
- Between attempts to the same neighbour, a random wait longer than the airtime of the next packet.

**Status:** decided.

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

- Per node: the direct neighbours with full table, and the neighbours of neighbours as ID, geo and link. That is the 2-hop map. Beyond that, a node knows nothing.
- Links are used only if they are above a quality threshold and work in both directions.
- **Quality gate on confirmation (decided, Oct 2026):** a confirmation (hello ack addressed to us, proof list naming us, ack1 on our data) makes a link symmetric only when the measured quality of that reception is above the threshold. Each node can only measure the direction toward itself; the neighbour applies the same gate to its own direction. The combined effect: a link is confirmed symmetric only when BOTH directions are above the threshold — the core value is `MC_SYM_QUALITY_MIN`, quality_of(−100 dBm), 10 dB above the reception threshold including fade margin. A weak link stays "seen": liveness and quality remain stored, the proof refreshes the reverse evidence, but routing never selects it. The proof list only sends confirmed links (4.2), so weak links are also never advertised to third parties as working.
- Each direction must be proven by the receiver of that direction:
  - A hello or an overheard hello ack proves only sender→receiver ("seen").
  - The reverse direction comes exclusively from the neighbour itself: via a hello ack addressed to us, via its table chunks (4.3), or via a successful unicast with ack1 ("confirmed").
- Routing chooses the next hop only via confirmed links. Direct replies in exchanges are exempt from this requirement (4.3).

**Status:** decided.

### 10.2 Choosing the next hop

| Situation | Rule | Status |
|---|---|---|
| Destination in the 2-hop map | Shortest route on ETX-like quality | Decided |
| Destination outside the map, geo known | Greedy: the neighbour with the best quality that is closer to the destination | Decided |
| No neighbour closer (gap, "void") | Fallback: waypoint or perimeter (see 10.3) | Proposal |
| Destination unknown (ID only) | Do not flood. Wait until the map contains the destination, or return a negative report | Decided |

**Waypoints.** The source can attach an intermediate point, for example the geo of a repeater on the Afsluitdijk. This is stateless and cheap.

**Status:** proposal.

### 10.3 Perimeter routing (later)

- Basis: GPSR-like perimeter mode.
- The Gabriel graph is calculated on the known links. A link between u and v remains if there is no witness w within the circle with diameter uv.
- Witnesses almost always lie within the 2-hop map, because the circle falls within radio range.
- A packet in perimeter mode carries ten extra bytes: the geo of the perimeter's starting point and the last face change.
- Limitation: links between two-hop nodes are not known. The perimeter around a gap can therefore be incomplete. This must be tested in simulation, for example around the IJsselmeer.

**Status:** later. First waypoints and failed delivery reports. Perimeter only if the simulation requires it.

### 10.4 Penalty and recovery

- A failed transmission increases the penalty of the link. Consecutive failures increase the penalty further. Success lets it decrease slowly.
- The penalty is local. A shared "suspect" field could be added later, as information and not as a decision.
- Probing: periodically send a request to a penalised neighbour, so that recovery becomes visible.
- Hysteresis and a minimum waiting time prevent oscillation.

**Status:** proposal. Parameters: open.

### 10.5 Failed delivery report

- A node that has not forwarded after all retries sends a negative report back to the source.
- The report is a routed message, not a single hop. It uses the normal routing to the source, with the source geo if available.
- Contents: message ID, node ID of the reporter, cause, and optionally the geo of the reporter.
- The report has its own small budget, for example half of the standard budget.
- It does not improve reliability. It is information for the client.
- A node keeps a pending record until `T_ack2` of its own hop or until the retry sequence has finished. After that, only the dedup cache remains. The message itself is not stored.
- The report carries a flag that prevents it from triggering a report of its own.

**Status:** decided. The type is now reserved; the implementation can come later.

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
| 2 | Radius based on local density, target about 100 nodes. kNN works only on an almost complete 2-hop map, so while the map is small, a bootstrap on link distances applies. The upper bound (starting value 40 km) is a firmware constant that still needs to be tested in simulation. The lower bound is not critical. | Determines the size of threads |
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
| 15 | Local load measurement (channel occupancy via CAD, own duty cycle, ack failure rate) to schedule maintenance in quiet moments. Tasks: (a) local computation without airtime: recalculate the Gabriel graph, recompute routes, clean up tombstones, compress the dedup cache; (b) small unicast tasks: fetch routing information from 2-hop neighbours, refresh neighbours' neighbor tables; (c) probes to penalised links, and measuring quality in both directions; (d) measurement work for the SF8 question. Not allowed in quiet periods: floods, broad pulls, and anything that changes the hello. Knowledge from neighbours gets an age and a source, and weighs more heavily the more recent it is. This way the map's knowledge reaches beyond two hops, without one node having to fetch everything. | Maintenance without extra network load |
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
| Neighbor table and 2-hop map (256 nodes of 12 bytes) | 3 KB | 3 KB |
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
- **Bounded 2-hop map:** the number of nodes is capped, so that the Gabriel calculation and the kNN radius stay within the computing budget.

**Status:** budgets decided. Open: whether a BLE companion on the same chip competes with the protocol for RAM.

---

## 14. Simulation

### 14.1 Layers

1. Radio and channel: airtime, path loss, capture effect, duty cycle.
2. Hello and neighbor table.
3. Data, ack1, ack2, retries and budget.
4. Routing: map, greedy, failed reports.
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
