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

- **Hello:** a 5-byte beacon that announces a node's presence and the state version of its neighbor table. (Section 4.1.)
- **Keepalive:** a periodic hello that a node sends on a channel when that channel has been quiet for a while, so that neighbours do not forget it. (Section 4.1.)
- **Hello ack:** a reply to a hello (or to other identifying traffic) that confirms the link in the other direction. (Sections 2, 4.2.)
- **ack1:** confirms that the next hop received a data packet. Sent on the same channel as the exchange. (Section 7.)
- **ack2:** sent back to the previous hop once the next hop has received the message (positive) or failed to forward it (negative). (Section 7.2.)
- **ack3:** an end-to-end confirmation from destination to source, sent only when requested. (Sections 6.2, 11.)
- **ANN (announcement):** a short 5-byte message that tells the network a large thread message is available. (Section 8.5.)
- **FULL:** the large thread message itself, which is fetched after an ANN. (Sections 6.5, 8.5.)
- **Courtesy ack:** after a node has confirmed a link, it sends two more hello acks so that the other side can complete its own evidence, then stays silent. (Section 4.1.)
- **Tablepull:** a request for a neighbour's neighbor table, sent in chunks. (Section 4.3.)
- **Failed delivery report:** a routed message sent back to the source when a message could not be forwarded after all retries. (Section 10.5.)
- **Budget:** a counter in the header that limits how many forwards and retries a message may use. (Section 9.)
- **Pending record:** the small record a forwarding node keeps for a message until its ack2 window closes. (Section 7.2.)
- **Dedup cache:** a cache of recently seen message IDs, used to discard duplicates. (Section 8.3.)

### Neighbours, links and routing

- **Neighbor table:** the list of direct neighbours a node advertises, with quality and age for each. (Section 4.2.)
- **State version:** a counter that increases whenever a node's neighbor table changes. A changed version triggers a table fetch. (Section 4.1.)
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
- **`T_keepalive`:** how long a channel may be quiet before a keepalive hello is sent (1 hour). (Section 4.1.)
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

- **Direct replies on the same channel:** ack1 for data and acks for tablepull chunks are awaited on the channel where the exchange takes place. The sender is already there, and the receiver is at home or has already switched.
- **Hello acks:** these go to the home channel of the hello sender. After a hello, several neighbours respond, and their replies are spread out over time. For a keepalive hello on channel c there are two options (open point 18):
  - (a) the sender stays on c for a short time and the responders reply there without switching. **Preferred**, because the responders then do not need to switch, and the snooping benefit of the acks falls exactly on the intended audience of the hello.
  - (b) the acks switch to the home channel of the sender.
- The sender transmits the hello on all three channels in a fixed order and then returns to its home channel. Responders reply there, within the maximum response time. That response time is also the sender's receive window.
- **Spreading by RSSI (decided):** strong neighbours respond first, weak neighbours later. The waiting time falls as the received strength of the hello rises, in discrete slots. RSSI determines the slot; a random jitter determines the position within the slot. Without jitter, neighbours with almost identical RSSI get the same waiting time and collide. Strong neighbours provide the most valuable link-quality information, so their acks arrive first.
- **Jitter above the ack airtime (decided, Oct 2026):** the jitter within a slot is window/16 ≈ 187 ms. That is well above the airtime of an ack (78 ms) plus the switch. The old window/64 (≈ 47 ms) let acks of equal rank collide; this was demonstrated in simulation. Now acks collide only by chance. The cost: the ranking becomes "soft". Responders that differ by less than about 16 dB may swap order. This was accepted, because avoiding collisions is the most important goal.
- **Unicast data:** the sender switches to the home channel of the receiver, transmits, waits there for ack1, and returns. Per hop the sender makes at most two switches (out and back), and the receiver makes none.
- **Ack2:** the next hop switches to the home channel of the previous hop, transmits, and returns directly. One exchange.
- **Tablepulls:** unicast to the home channel of the partner. This is an exchange, so switching is allowed.
- **Announcements (ANN):** on all three channels in a fixed order. A FULL fetch is an exchange (`T_full`), so the fetcher switches to the channel of the announcer and returns.
- **Shared chat channels** (#public, #test) are dropped.
- **Simulation metric:** the time per node spent outside its own home channel, the "home deafness". Every moment outside its own channel is missed reception on its own channel.
- **Ack eligibility (decided, Oct 2026):** a transmission leads to an ack only when three independent conditions are met:
  1. **Need:** the link is at "seen" rather than "confirmed symmetric", the reverse-direction evidence is stale (after `T_sym`), or a courtesy ack is still pending. At rest this condition is false, which limits the number of acks even when a neighbour transmits a lot.
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

- **Size:** 5 bytes. Contents: type (1), node ID (2), state version of the neighbor table (2). A geo change increments the state version; there is no separate geo version.
- **Frequency (adaptive):** at startup every 30 s, then rising to 5 minutes, and eventually 1 hour in a stable state. Nodes that have been present longer therefore send less often.
- **Random phase and jitter (decided, coupling test Oct 2026):** each hello is scheduled with a random delay. Without jitter, nodes that start at the same time (for example after a power outage in a region) keep synchronous rounds, and never hear each other, because during their own round they are on a different channel. The solution now lives in the protocol core.
- **Per-channel keepalive (proposal, Oct 2026; to be tested in scenarios 2 and 5):** the hello serves two purposes: keeping known neighbours fresh, and discoverability for unknown nodes. Liveness can also come from other traffic. Every received transmission refreshes `last_seen` of the sender: the sender appears in `prev_hop` for data/ack3 and in `afzender` for ack1, ack2 and hello-ack. `src` never counts, because a forwarded message does not prove that the original source is in range.
  - For each channel c, a node records when it was last "addressed" there. Every identifying transmission on c counts (hello, hello-ack, ack1, ack2, data as `prev_hop`, pull traffic). ANN and FULL do not count, because they are anonymous.
  - When the timer has expired (`T_keepalive`, starting value 1 hour), exactly one hello is sent on c.
  - During the startup phase the round across all three channels still applies (30 s interval). After that the hello exists only as a keepalive.
  - Benefit: nationwide, the hello is the largest load. With 3000 nodes and a 1-hour interval, that is about 5.8% per channel, and the hello acks come on top of that. The round shrinks to the channels that are needed: fewer switches, and a node whose stale neighbours are on its own home channel alone sends without any switch.
  - Newcomers do not suffer from this: their startup hellos provide discovery, and the response to them (hello ack plus startup pull, 4.3) gives both sides their symmetry evidence.
  - Risks: discovery between two nodes that are both still unknown is postponed until the keepalive, and version propagation becomes dependent on existing traffic.
  - To measure: hellos per hour per node per channel, hello and ack airtime per channel, discovery latency of newcomers, and the delay in version refresh. `T_keepalive` is a parameter (starting value 1 hour, linked to `T_expire`, 4.4).
- **Hello ack on demand (proposal, Oct 2026; minimal form implemented):** unconfirmed links stay stored, but the chance of an ack decreases over time. Every identifying transmission from the neighbour increments a counter (hellos and home-channel transmissions, section 2; retries count as one). Only counter values 1, 2, 4, 8 and 16 produce an ack attempt. After that, on average one ack per 16 transmissions still occurs, with a minimum time threshold per neighbour. This means that a permanently asymmetric link on a busy hub does not cause unbounded ack traffic.
  - After our own confirmation, two more "courtesy acks" follow, and then definitive silence. Snooping (section 2) keeps liveness, version and quality in order by then.
  - Courtesy acks are needed because confirmation almost always happens asymmetrically: when one direction succeeds first, those acks stop, while the neighbour still has to retrieve its evidence from our ack.
  - The counter resets on confirmation and on renewed contact. Whether a transmission leads to an ack is determined by ack eligibility (section 2).
  - Not yet implemented: the cooldown per neighbour and the minimum time threshold on the drop.
  - Dropped (Oct 2026): acks on an own version change, and the reset on `T_stale`. These have been replaced by evidence ageing with `T_sym` (4.2).
  - Motivation (40-node scenario): tower sites trigger an ack storm on their home channel every round. With the scheme, the steady-state ack load falls from 0.31–0.37% to 0.12–0.17% per channel. Collisions fall from 25,435 to 15,734 (−38%), and missed hellos from 5,296 to 4,690. The density of evidence actually rises (average confirmed from 6.2 to 6.9), because the remaining attempts go through more often.
  - Hub sites see hundreds of neighbours. There the neighbour table limit of 64 acts as a brake, provided it is stable (top-N on strength with hysteresis, 10.4).
- A new node quickly gets a map thanks to the short startup intervals.
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
- (c) later, a tablepull from the neighbour in which we appear (4.3), with its own measurement of our signal.

An overheard hello ack addressed to someone else does not prove symmetry. Routing chooses the next hop only via confirmed symmetric neighbours (10.1). Direct replies in exchanges are always allowed (4.3). Advertising this status in the neighbor table comes with the tablepull of building block 2.

**Long-term symmetry (proposal, Oct 2026).** The confirmed status is not valid forever. For each neighbour, a node records how old the reverse-direction evidence is ("last proven"). It is refreshed by a hello ack addressed to us, a successful unicast with ack1, or a tablepull from the neighbour in which we appear.

Without maintenance there is a deadlock: the neighbour that acks first again (the drop, 4.1) only refreshes its receiver. The sender learns nothing from its own transmission, so its evidence ages indefinitely, even though the link is healthy.

The solution is the flag "confirm this" in the hello ack (section 2). When the evidence age reaches `T_sym/2`, a node sends its next hello ack with the flag set. The receiver replies exactly once; its own evidence is already fresh from the reception, and without the flag it would stay silent.

- Cost: two packets of 7 bytes per link per period. This occurs almost only in a quiet network, because with traffic, ack1s and tablepulls usually refresh the evidence sooner.
- The margin between `T_sym/2` and `T_sym` absorbs lost exchanges. At `T_sym` the link degrades to "seen", and routing (10.1) then no longer selects it.
- Parameter: `T_sym`, starting value 6 hours, to be tested in scenario 5.

**Status:** decided.

### 4.3 Fetching tables

- A node fetches the neighbor table only from direct neighbours, and only when the state version in the hello changes. First contact (no stored table yet for that neighbour) counts as a version change.
- **Pulls complete the symmetry (decided, Oct 2026).** When a new node starts up, its hello round provides the symmetry evidence for itself (the acks addressed to it, sections 2 and 4.2). At the responders, the newcomer remains "seen" afterwards. Their reverse-direction evidence arises by two routes:
  - (a) from the hello process: the newcomer acks the responder's next own round. With a stable responder this can take up to one hello interval (at most 1 hour). With the generalised hello ack (section 2) this often happens sooner, because every home-channel transmission of the responder is an ack trigger for the newcomer.
  - (b) from the startup pull: the puller confirms every chunk with ack1. The responder sees its own transmission arriving and marks the newcomer as symmetric after the first chunk ack. Besides building the map, this is also the fast symmetry closer, and an additional reason not to postpone the startup pull.
- **Forwarding versus direct replies (decided, Oct 2026).** The symmetry requirement applies to choosing a next hop for data (routing, 10.1). A direct reply within an exchange (ack1, pull chunks, ack2) may also be sent by the responder if it has the partner only as "seen". The received request proves the partner→responder direction, the partner is already waiting on the channel, and the ack1s on the reply supply the missing evidence afterwards. Refusing would block the startup of new nodes.
- The table header contains the absolute geo of the owner and a firmware version of 1 byte. The version thus travels with every tablepull, without extra bytes per packet.
- The table also contains the links between the owner's neighbours. The owner already knows these from its neighbours' tables, so no extra pull is needed. Per link: two index bytes within the neighbour list, and the quality in both directions. This way a node knows all links within its 2-hop set, and the Gabriel graph around a gap becomes complete. Nodes three hops away are unknown by default. They only come in through maintenance during quiet periods (section 12, point 15).
- Pulls of tables from second-hop nodes do **not** happen by default. Strategically expanding the routing table during quiet periods can be considered later.
- At startup, requests are sent with a random delay, so that replies do not collide on the home channel.
- Tables larger than one packet are sent in chunks of about 200 bytes, with sequence numbers. Missing chunks are requested again. Each chunk receives an ack1.

**Status:** decided.

### 4.4 Expiry, tombstones and stale entries

**Death detection (proposal, Oct 2026).** For each direct neighbour, a node locally records when it last heard it. Every identifying transmission that is overheard counts. The clock is local, so no synchronisation is needed. After `T_expire` (starting value 4 hours) without reception, the neighbour is considered dead and the entry disappears, without a tombstone.

- `T_expire` is at least three to four times `T_keepalive` (4.1). A living, quiet node sends at least its keepalive hello once per hour. Four hours therefore means four missed keepalive rounds, on top of all traffic that is overheard.
- In a mass outage (for example a power failure in a region), hundreds of entries expire at the same time. Every mutation increments the state version, and the pull wave that follows must stay spread out through the existing random delay (4.3).
- The age per entry remains the measure of what a receiver trusts.

**Age encoding (proposal, Oct 2026).** The age byte per entry encodes minutes; 255 = 4 hours or older, hence expired. Receivers of tablepulls thereby filter out expired entries automatically.

**Why death detection needs no tombstone.** A dead neighbour disappears from every advertised table no later than `T_expire`, because every owner applies the same rule. The age encoding ensures that receivers ignore expired entries. A neighbour that lives only in the 2-hop map (Y hears him, we do not) is not a zombie but legitimate information with a fresh age. A tombstone covering the whole table would wrongly block him.

**Tombstone, limited to its actual task (Oct 2026).** The tombstone exists for removal by the neighbor table limit on hub sites (10.4). That affects a living neighbour that keeps transmitting. Without a tombstone, the neighbour would return within the keepalive interval, the neighbor table limit would oscillate, and every mutation would cause a version increment and a pull wave.

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
| ack1 | 7 bytes | Next hop → previous hop | Same channel as the data exchange; the sender waits there according to the switching rule |
| ack2 | 7 bytes | Next hop → previous hop | Home channel of the previous hop |
| ack3 | Routed message, payload 4 bytes | Destination → source | Via routing |
| hello ack | 7 bytes, directional: type (including the "confirm this" flag, 4.2), destination ID (the triggered sender), sender ID, state version (section 2) | Responder → triggered sender | During a hello round: home channel of the hello sender. For a home-channel transmission of the trigger (ANN, FULL, or ack1 on its own channel): that channel itself, because the triggering sender listens there by definition. |

Ack1 contains the identity of the original message and the sender of ack1.

### 6.3 Hello

5 bytes, see section 4.1.

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
- Each direction must be proven by the receiver of that direction:
  - A hello or an overheard hello ack proves only sender→receiver ("seen").
  - The reverse direction comes exclusively from the neighbour itself: via a hello ack addressed to us, via its tablepull (4.3), or via a successful unicast with ack1 ("confirmed").
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
| 8 | RTS/CTS: experiment with and without. | Only needed if hidden-node collisions cause heavy loss |
| 9 | SF8 nodes in the Dutch data: which range and which airtime. | Radio model |
| 10 | Chunk size of tables (about 200 bytes). | Fragmentation |
| 11 | Battery capacity: for solar repeaters this is settled (continuous listening is the default). The question shifts to companions: battery capacity and lifetime with continuous listening, and whether companions can choose a listening profile based on charging source or battery level. | Companion behaviour, not the protocol |
| 12 | Companion design: BLE protocol to the phone, coupling with the home repeater, and behaviour when out of range (parked, on the to-do list). Companions do not get a node ID. Addressing goes via the home repeater and a slot number, so that joining and leaving causes no changes in the neighbor tables. | Companion is still largely unspecified |
| 13 | Node ID as geo tag plus random suffix: not yet implemented. Disadvantage: the ID changes when the node moves, headers become longer, and the location is always in the packet. | The saving lies mainly in the neighbor table |
| 14 | SF8 fallback for remote nodes. A receiver listens on one SF, so the choice applies per link or per region, not per node. Estimated gain: 25–30% range at about 2× airtime. | Radio design, possibly per region |
| 15 | Local load measurement (channel occupancy via CAD, own duty cycle, ack failure rate) to schedule maintenance in quiet moments. Tasks: (a) local computation without airtime: recalculate the Gabriel graph, recompute routes, clean up tombstones, compress the dedup cache; (b) small unicast tasks: fetch routing information from 2-hop neighbours, refresh neighbours' neighbor tables; (c) probes to penalised links, and measuring quality in both directions; (d) measurement work for the SF8 question. Not allowed in quiet periods: floods, broad pulls, and anything that changes the hello. Knowledge from neighbours gets an age and a source, and weighs more heavily the more recent it is. This way the map's knowledge reaches beyond two hops, without one node having to fetch everything. | Maintenance without extra network load |
| 16 | Trigger generalisation (section 2): the hello ack follows every identifying home-channel transmission of a neighbour. To be measured in scenario 2: the steady-state ack load must not exceed the schedule (0.12–0.17% per channel). Also: ack collisions on ANN/FULL triggers and the rank spreading there. | Determines whether the keepalive hello and the generalised ack together carry the resting traffic |
| 17 | `T_sym` for re-confirming symmetry (4.2, starting value 6 hours), and the ratio `T_keepalive : T_expire = 1 : 4` (starting values 1 hour / 4 hours, 4.4). | Route stability versus airtime; to be tested in scenario 5 |
| 18 | Ack channel for a keepalive hello on channel c: (a) the sender stays on c for a short window, responders do not switch (preferred, section 2); or (b) the current rule: acks switch to the home channel of the sender. | (a) concentrates home deafness on one node per keepalive hello; (b) spreads switches across all responders |

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
| Neighbor table and 2-hop map (about 200 nodes of 12 bytes) | 3 KB | 3 KB |
| Link list (about 1000 links of 5 bytes) | 5 KB | 5 KB |
| Chunk reassembly (one per neighbour) | 4 KB | 8 KB |
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
- Keepalive hellos per node per channel, delay in version refresh, and re-confirmation exchanges (`T_sym`, 4.2).

**Topology:** real repeater locations where available, otherwise synthetic with the same density per region. Water is excluded (for example via OpenStreetMap).

**Calibration:** the SNR thresholds of the Dutch map (poor below −2.5 dB, good above 3.5 dB) and the measured channel occupancy.

### 14.3 Build plan (proposal)

**Architecture.** Event-driven discrete simulation in Python. The shared C protocol core (13.3) is called via ctypes or cffi. Each simulated node has its own instance of the core, with no shared state. The simulator supplies the clock, the radio and the RNG to the hardware interface of the core. Radio events: `tx_start`, `tx_end`, `rx_begin`, `collision`. Time in microseconds, to avoid rounding errors in long runs.

**Layer 1: radio and channel (build first).**

| Quantity | Model | Value (SF7, 62.5 kHz, CR4/5) |
|---|---|---|
| t_symbol | 2^SF / BW | 2.048 ms |
| hello / ANN (5 B) | LoRa airtime | 70 ms |
| ack1 / ack2 / hello ack (7 B) | idem | 78 ms |
| data (100 B) | idem | 459 ms |
| FULL / chunk (200 B) | idem | 869 ms |
| Hello round (3 channels + pauses `g`) | 3 × 70 ms + 2g | about 210 ms + pauses |
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

1. **Coupling test:** 2 nodes, hello, tablepull and one unicast. Checks that the C core and the simulator agree on packet sizes, airtimes and ack order.
2. **Local network:** 30–50 nodes at one density, from a node list with GPS position and height. Measures hello airtime and ack collisions on hello acks (RSSI spreading), keepalive hellos per node per channel, and the steady-state ack load on generalised triggers (ANN/FULL) against the schedule (0.12–0.17%).
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
| Tablepull from second-hop nodes | Dropped (default) | Too much traffic; possibly off-peak later |
| Encrypted geo | Dropped | Relays need the geo |
| Repeater WiFi on by default | Dropped | Power consumption on solar; now off by default |
| Repeater storage for companions | Dropped | Companions retrieve threads from their repeater's thread cache |
| "Hello on demand", rules 1 and 2 (stale-neighbour and version triggers) | Replaced (Oct 2026) | Snooping and the generalised hello ack cover liveness and version. The per-channel keepalive (4.1) remains. |
| Hello ack only after a hello | Replaced (Oct 2026) | Generalised to every identifying home-channel transmission, with the ack eligibility condition, cooldown and re-confirmation flag (sections 2 and 4.2) |
| Removal by silence with tombstone | Replaced (Oct 2026) | Silence is death detection (4 hours, 4.4) without a tombstone. The tombstone stays for neighbor table limit churn on hub sites (10.4). |
