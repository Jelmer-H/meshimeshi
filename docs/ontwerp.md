# Meshimeshi: ontwerpdocument
**Status:** werkdocument, nog niet gesimuleerd
**Taal:** Nederlands
**Laatst bijgewerkt:** 17 oktober 2026

Dit document legt de ontwerpkeuzes vast die tot nu toe zijn besproken. Elk onderdeel krijgt een status:

- **Besloten:** gekozen, tenzij de simulatie anders laat zien.
- **Voorstel:** uitgewerkt, nog niet vastgesteld.
- **Open:** nog te beslissen of te meten.
- **Later:** bewust uitgesteld.

---

## 1. Doelen en prioriteiten

| Prioriteit | Doel | Status |
|---|---|---|
| 1 | Betrouwbaarheid: berichten komen aan, en bij uitval wordt snel een alternatieve route gekozen | Besloten |
| 2 | Schaal: werkt op landelijke schaal (ongeveer 3000 nodes, waarvan bijna 3000 repeaters) | Besloten |
| 3 | Zero config: de gebruiker hoeft niets in te stellen | Besloten |
| 4 | Lage airtime, zonder dat dit ten koste gaat van betrouwbaarheid | Besloten |
| 5 | Snelheid op lange afstand is secundair | Besloten |
| 6 | Nieuwe gebruikers zien snel recente berichten | Besloten |
| 7 | Privacy: geo met beperkte precisie | Besloten |

---

## 2. Radio

| Parameter | Waarde | Status |
|---|---|---|
| Modulatie | LoRa | Besloten |
| Spreading factor | SF7 als basis. In de Nederlandse data komt ook SF8 voor; dat moet in de simulatie worden onderzocht | Voorstel |
| Bandbreedte | **62,5 kHz** (okt 2026). Redenen: (a) bij 125 kHz overlappen kanalen op 100 kHz afstand elkaar, en passen er in de sub-band van 250 kHz maar twee schone kanalen; (b) 3 dB extra gevoeligheid, dus meer bereik, kleinere gaten en minder retries; (c) dezelfde instellingen als het bestaande Nederlandse LoRa-netwerk op deze band (869,618 MHz, 62,5 kHz, SF7/CR5). Prijs: de datarate halveert (ongeveer 2,7 kbps bij SF7). Alle time-outs en slotparameters zijn hierop berekend. | Besloten |
| Coding rate | 4/5 | Voorstel |
| Thuiskanalen | **869,44 / 869,50 / 869,56 MHz** (okt 2026). Met 62,5 kHz passen drie kanalen met 60 kHz tussenruimte ruim binnen de sub-band 869,4–869,65 MHz. Het oude plan (869,45/55/65 bij 125 kHz) is vervallen omdat die kanalen elkaar overlapten. Kanaal 4 (±869,62 MHz) is overwogen maar afgevallen vanwege de bandrand; mogelijk later opnieuw bekijken. | Besloten |
| Thuiskanaal van een node | `node_id mod 3` | Besloten |
| Duty cycle | 10% per kanaal | Besloten |
| Botsingsmodel | Twee overlappende pakketten op hetzelfde kanaal botsen, tenzij het sterkste met voldoende marge wint (capture effect) | Voorstel |

### Kanaalgebruik en wisselregel

Elke node luistert continu op zijn eigen thuiskanaal. Hij wisselt alleen van kanaal voor een **exchange** met een bekende partner (bekende pakketgrootte en korte, bekende timeout), en keert daarna meteen terug naar zijn thuiskanaal. Wacht een node op iets van onbekende of lange duur, dan blijft hij op of gaat hij naar zijn eigen thuiskanaal; het antwoord komt daar vanzelf binnen.

- **Gerichte antwoorden op hetzelfde kanaal:** ack1 op data wordt afgewacht op het kanaal waar de uitwisseling plaatsvindt. De verzender is daar al, en de ontvanger is thuis of heeft al gewisseld.
- **Hello-acks:** deze gaan naar het thuiskanaal van de hello-verzender. Na een hello reageren meerdere buren, en die reacties worden over de tijd gespreid. Bij een keepalive-hello op kanaal c zijn er twee opties (open punt 18):
  - (a) de verzender blijft een korte tijd op c en de responders antwoorden daar zonder te wisselen. **Voorkeur**, omdat responders dan niet hoeven te wisselen en de snooping-bonus van de acks precies bij de doelgroep van de hello valt.
  - (b) de acks wisselen naar het thuiskanaal van de verzender.
- De verzender stuurt de hello op alle drie de kanalen in vaste volgorde en keert daarna terug naar zijn thuiskanaal. Responders antwoorden daar, binnen de maximale responstijd. Die responstijd is tegelijk het ontvangstvenster van de verzender.
- **Spreiding via RSSI (besloten):** sterke buren reageren eerst, zwakke buren later. De wachttijd daalt met de ontvangststerkte van de hello, in discrete slots. RSSI bepaalt het slot, een random jitter bepaalt de plek binnen het slot. Zonder jitter krijgen buren met vrijwel gelijke RSSI dezelfde wachttijd en botsen ze met elkaar. Sterke buren leveren de waardevolste kwaliteitsinformatie, dus die komen het eerst binnen.
- **Jitter boven de ack-airtime (besloten, okt 2026):** de jitter binnen een slot is venster/16 ≈ 187 ms. Dat is ruim boven de airtime van een ack (78 ms) plus de wissel. Het oude venster/64 (≈ 47 ms) liet acks met gelijke rang botsen; dat is in de simulatie aangetoond. Nu botsen acks hooguit toevallig. De prijs: de rangordening is "zacht". Responders die minder dan ongeveer 16 dB uit elkaar liggen kunnen van volgorde wisselen. Dat is geaccepteerd, want voorkomen van botsingen is het belangrijkste doel.
- **Unicast data:** de verzender wisselt naar het thuiskanaal van de ontvanger, zendt, wacht daar op ack1 en keert terug. Per hop maximaal twee wissels voor de verzender (heen en terug), en nul voor de ontvanger.
- **Ack2:** de volgende hop wisselt naar het thuiskanaal van de vorige hop, zendt, en keert direct terug. Eén exchange.
- **Tabelchunks (push, 4.3):** broadcast op het eigen thuiskanaal, in de vaste kadans. Geen exchange, geen wissel, geen ack; buren snoopen mee.
- **Bewijslijst (4.2):** een klein pakket met de node-ID's van buren waarvan het tegenrichtingbewijs binnenkort afloopt. Alleen de sub-lijst van buren die op dát kanaal thuis zijn; elk kanaal heeft per node één flush-moment per uur (20-minutenrotatie), dus maximaal één bewijslijst-pakket per kanaal per uur. Het pakket is een eigen roep op dat kanaal en reset de keepaliveteller ervan (4.1). Een ontvanger die erin staat, verversst zijn bewijs en reageert niet; een ontvanger die er niet in staat, verwerkt de zender zoals snooping dat doet (liveness, versie, kwaliteit, "gezien").
- **Aankondigingen (ANN):** op alle drie de kanalen in vaste volgorde. Een FULL-fetch is een exchange (`T_full`), dus de fetcher wisselt naar het kanaal van de aankondiger en keert terug.
- **Gedeelde chatkanalen** (#public, #test) vervallen.
- **Simulatiemeting:** de tijd per node buiten het eigen thuiskanaal, de "thuis-doofheid". Elk moment buiten het eigen kanaal is gemiste ontvangst op het eigen kanaal.
- **Ack eligibility (besloten, okt 2026):** een zending leidt alleen tot een ack als aan drie onafhankelijke voorwaarden is voldaan:
  1. **Need:** de link staat op "gezien" in plaats van "bevestigd symmetrisch", of er staat een courtesy ack open. Verouderd tegenbewijs is geen ack-reden: dat onderhoudt de bewijslijst (4.2). In rust is deze voorwaarde niet waar, en dat beperkt het aantal acks, ook als een buur veel zendt.
  2. **Pace:** de teller per buur (sectie 4.1) en de cooldown na een eigen ack.
  3. **Spacing:** de rangorde op RSSI met jitter (zie hierboven), zodat acks van meerdere responders elkaar niet raken.

---

## 3. Nodetypen

| Type | Beschrijving | Status |
|---|---|---|
| Repeater | Vaste node, altijd aan, stuurt door | Besloten |
| Companion | Mobiel eindapparaat. Verbindt via BLE met de telefoon en heeft één vaste repeater als thuisbasis. Luistert continu en stuurt niet door. | Besloten |

**Repeater.** Werkt vooral op zonne-energie met een kleine accu. WiFi staat standaard uit en kan op aanvraag of op vaste tijden aan. Er wordt geen geheugen gereserveerd voor companions.

**Energie (repeaters, besloten).** Continu luisteren is voor zonne-repeaters geen probleem en is daarom de standaard. Rekenvoorbeeld (SenseCAP Solar P1-Pro: 5 W paneel, 4 × 3350 mAh = 13.400 mAh): de SX1262 verbruikt ongeveer 4,6 mA in ontvangst, het hele systeem in luisterstand ongeveer 7–11 mA. Een winternacht van 16 uur kost 110–175 mAh, ongeveer 1% van de accu. Zonder zon houdt een repeater het dus enkele weken vol. Conclusie: de accucapaciteit beperkt het protocol niet. Voor zeer kleine repeaters zonder zon geldt nog wel een lagere grens.

**Companion.** Luistert continu, zodat hij threads in bereik direct ontvangt. Luisteren kost accu maar geen airtime, dus het belast het mesh niet. Er is geen slaapstand. Na het uit bereik zijn, of na een herstart, haalt de companion gemiste threads op bij zijn thuis-repeater uit de threadcache, zodra een ontvangen bericht een voorganger laat zien die hij mist. Daar is geen extra state voor nodig.

- Eigen berichten gaan via de thuis-repeater.
- Ack2 kan een companion missen als hij net uit bereik is. Hij bevestigt dan via de threadcache van de repeater.
- Directe berichten aan een companion die uit bereik is, leveren een failed delivery report op met de melding "eindapparaat niet bereikbaar".

Het onderscheid tussen beide typen volgt uit de gepubliceerde geo (sectie 5): repeaters publiceren het volledige raster, companions schalen die vóór publicatie terug.

---

## 4. Hello en neighbor table

### 4.1 Hello

- **Grootte:** 5 bytes. Inhoud: type (1), node-ID (2), state-versie van de neighbor table (2). Een geowijziging verhoogt de state-versie; er is geen aparte geo-versie.
- **Frequentie (opstartfase, adaptief):** bij opstarten elke 30 s, daarna verdubbend per ronde (30 → 60 → 120 → 240 → 480 → 960 → 1920 s). Op het 1-uursinterval stopt de ronde; daarna bestaat de hello alleen nog als per-kanaal-keepalive (hierna). Langer aanwezige nodes storen tijdens de opstart dus steeds minder.
- **Random fase en jitter (besloten, koppeltest okt 2026):** elke hello wordt met een willekeurige vertraging gepland. Zonder jitter houden nodes die tegelijk opstarten (bijvoorbeeld na een stroomuitval in een regio) synchrone rondes, en horen ze elkaar nooit, omdat ze tijdens hun eigen ronde op een ander kanaal zitten. De oplossing zit nu in de protocolkern.
- **Per-kanaal-keepalive (voorstel, okt 2026; minimale vorm geïmplementeerd; te toetsen in scenario 2 en 5):** de hello dient twee doelen: de versheid van bekende buren, en ontdekbaarheid voor onbekende nodes. Liveness kan ook uit ander verkeer komen. Elke ontvangen zending verversen `last_seen` van de zender: de zender staat in `prev_hop` bij data/ack3 en in `afzender` bij ack1, ack2 en hello-ack. `src` telt nooit, want een doorgestuurd bericht bewijst niet dat de oorspronkelijke bron in bereik is.
  - Per kanaal c houdt een node bij wanneer hij daar voor het laatst "geroepen" heeft. Elke identificerende zending op c telt mee (hello, hello-ack, ack1, ack2, data als `prev_hop`, tabelchunks). ANN en FULL tellen niet, omdat die anoniem zijn.
  - Is de timer verstreken (`T_keepalive`, startwaarde 1 uur), dan volgt precies één hello op c.
  - In de opstartfase blijft de ronde over alle drie de kanalen bestaan (30 s-interval). Daarna bestaat de hello alleen nog als keepalive.
  - Voordeel: op landelijke schaal is de hello de grootste belasting. Bij 3000 nodes en een interval van 1 uur is dat ongeveer 5,8% per kanaal, en de hello-acks komen daar nog bovenop. De ronde krimpt tot de benodigde kanalen: minder wissels, en een node wiens verouderde buren alleen op het eigen thuiskanaal zitten, zendt zonder enige wissel.
  - Nieuwkomers hebben hier geen nadeel van: hun opstart-hellos leveren de ontdekking, en hun eerste tabelpush (4.3) levert beide kanten hun symmetriebewijs.
  - Risico's: ontdekking tussen twee nog onbekende nodes schuift door tot de keepalive, en versieverspreiding wordt afhankelijk van aanwezig verkeer.
  - Te meten: hello's per uur per node per kanaal, hello- en ack-airtime per kanaal, ontdekkingslatentie van nieuwkomers, en vertraging van versie-verversing. `T_keepalive` is een parameter (startwaarde 1 uur, gekoppeld aan `T_expire`, 4.4). Eerste meting na implementatie (lokaal-net, 40 nodes, 8 u): stabiele hello-lucht 0,09% → 0,06% per kanaal, gemiste hellos 4690 → 3582, bevestigd symmetrisch gem 6,9 → 8,4 — een node die op een kanaal ackte (eigen roep), hoeft daar geen keepalive-hello meer te sturen.
- **Hello-ack op behoefte (voorstel, okt 2026; minimale vorm geïmplementeerd):** onbevestigde links blijven opgeslagen, maar de kans op een ack neemt af met de tijd. Elke identificerende zending van de buur verhoogt een teller (hellos en thuis-zendingen, sectie 2; retries tellen als één). Alleen de tellerstanden 1, 2, 4, 8 en 16 leveren een ack-poging op. Daarna komt er gemiddeld nog één ack per 16 zendingen, met een minimale tijdsdrempel per buur. Zo levert een blijvend asymmetrische link op een drukke hub geen onbegrensd ackverbruik op.
  - Na eigen bevestiging volgen nog twee "courtesy acks" en daarna definitieve stilte. Snooping (sectie 2) houdt liveness, versie en kwaliteit dan al op orde.
  - Courtesy acks zijn nodig omdat bevestiging bijna altijd asymmetrisch verloopt: als één richting eerst slaagt, stoppen die acks, terwijl de buur zijn bewijs nog uit onze ack moet halen.
  - Teller reset bij bevestiging en bij hernieuwd contact. Of een zending tot een ack leidt, bepaalt de ack eligibility (sectie 2).
  - Nog niet geïmplementeerd: de cooldown per buur en de minimale tijdsdrempel op de druppel.
  - Vervallen (okt 2026): acks bij een eigen versiewijziging, en de reset op `T_stale`. Die zijn vervangen door de bewijsveroudering met `T_sym` (4.2).
  - Motivatie (40-node-scenario): toren-sites ontstaken per ronde een ack-storm op hun thuiskanaal. Met het schema daalt de stabiele ackbelasting van 0,31–0,37% naar 0,12–0,17% per kanaal. Botsingen dalen van 25.435 naar 15.734 (−38%), en gemiste hellos van 5.296 naar 4.690. De bewijsdichtheid stijgt juist (gemiddeld bevestigd van 6,2 naar 6,9), omdat de overgebleven pogingen vaker doorgaan.
  - Hub-sites zien honderden buren. Daar werkt de neighbor table limit van 64 als rem, mits die stabiel is (top-N op sterkte met hysterese, 10.4).
- Een nieuwe node krijgt snel een kaart dankzij de korte startintervallen.
- Threadinformatie komt niet in de hello (zie 8.4).

**Status:** besloten, behalve de exacte intervallen (open).

### 4.2 Neighbor table

Elke node adverteert alleen zijn eigen neighbor table. Per buur geldt:

| Veld | Grootte | Toelichting |
|---|---|---|
| Node-ID | 2 bytes | |
| Inkomende kwaliteit | 1 byte | Gemeten door de node zelf (RSSI/SNR, gemiddeld over hellos en hello-acks). Ook een afgeluisterde ack is een geldige meting van de inkomende link. |
| Leeftijd | 1 byte | Minuten sinds de buur voor het laatst is gehoord. 255 = 4 uur of ouder, dus verlopen (4.4). |
| Geo | 4 bytes | Optioneel. (0,0) betekent: geen geo (sectie 5). |

Elke richting van een link wordt door de ontvanger van die richting gemeten. Asymmetrie is daarmee zichtbaar.

**Symmetriestatus (besloten, okt 2026).** Naast de geadverteerde velden houdt elke node per buur lokaal bij of symmetrisch verkeer bewezen is:

- **0 = gezien:** alleen de richting buur→ons is gemeten.
- **1 = bevestigd symmetrisch.**

De bevestiging komt uitsluitend van de buur zelf:

- (a) een aan ons gerichte hello-ack, het bewijskanaal van het hello-proces. Dit is beschikbaar vóór de eerste unicast.
- (b) een geslaagde unicast-uitwisseling met ack1.
- (c) een tabelchunk van de buur waarin wij staan (4.3), met zijn eigen meting van ons signaal.
- (d) een bewijslijst van de buur waarin wij staan (hierna).

Een aan een ander gerichte, afgeluisterde hello-ack bewijst geen symmetrie. Routering kiest de volgende hop alleen via bevestigd symmetrische buren (10.1). Directe antwoorden in exchanges zijn altijd toegestaan (4.3). Het meeadverteren van deze status in de neighbor table gebeurt via de tabelchunks (4.3): een chunk met onze entry is zelf al het tegenrichtingbewijs.

**Symmetrie op de lange termijn (voorstel, okt 2026).** De bevestigde status is niet eeuwig geldig. Per buur houdt een node bij hoe oud het tegenrichtingbewijs is ("laatst bewezen"). Dat wordt ververst door een aan ons gerichte hello-ack, een geslaagde unicast met ack1, een tabelchunk van de buur waarin wij staan (4.3), of doordat wij in zijn bewijslijst staan.

Zonder onderhoud ontstaat er een impasse: de buur die als eerste opnieuw ackt (de druppel, 4.1), ververst alleen zijn ontvanger. De zender leert niets van zijn eigen zending, dus zijn bewijs veroudert eeuwig, terwijl de link gezond is.

De oplossing is de **bewijslijst**: een klein pakket met de node-ID's van buren waarvan het tegenrichtingbewijs binnenkort afloopt. Het werkingsprincipe: een bewijs over de richting buur→ons kan alleen van óns komen, want wij zijn de enige bron van hun onmeetbare richting, en omgekeerd. Publiceert een node "ik hoor X", dan is bij X in één ontvangst de richting X→node bevestigd (de claim van de node zelf) en de richting node→X vers gemeten (de ontvangst zelf). Er is geen antwoord nodig: elke kant onderhoudt de andere via zijn eigen lijsten, onafhankelijk en symmetrisch.

- **Trigger.** Per buur houdt de node bij wanneer hij die voor het laatst heeft bevestigd (aan hem gerichte hello-ack, ack1 op zijn data, een tabelchunk waarin hij stond, of een eerdere lijst) — dat is de eigen zendlog, dus exact lokaal berekenbaar. Loopt de bevestigingsleeftijd boven `T_sym − 2 u`, dan komt de buur in de lijst. De marge geeft elke link nog twee flush-momenten vóór verval; bij `T_sym` degradeert de link naar "gezien", en routering (10.1) kiest hem dan niet meer.
- **Kadans en kap.** Flush-check elke 20 minuten, roterend per kanaal (`slot mod 3`). Daarmee heeft elk kanaal één flush-moment per uur: maximaal één bewijslijst-pakket per kanaal per uur per node, dezelfde frequentie als de keepalive zelf (4.1). De drie kanalen van een node liggen 20 minuten uit elkaar, dus geen zelf-opstopping.
- **Kanaal.** Alleen de sub-lijst van buren die op dát kanaal thuis zijn; elke ontvanger luistert thuis. Het pakket is een eigen roep op dat kanaal en reset de keepaliveteller (4.1).
- **Reactieloos.** In de lijst staan verversst het bewijs; niet in de lijst staan is snooping ("gezien", liveness, versie, kwaliteit). Geen enkele ontvangst triggert een reactie; de ack-op-op-regress blijft afgesneden, zonder vlagsemantiek of antwoordregel.
- **Kosten.** Per link per `T_sym`-periode twee lijst-entries, een per kant — bij losse entries evenveel lucht als de eerdere vlagbit-oplossing (2 × 78 ms), maar de entries bundelen (2 t/m 17 ID's in één pakket van de 173 ms-klasse), het schema plant zichzelf (geen afhankelijkheid van ack-kansen) en op hubs daalt het aantal pakketten. Vrijwel alleen in een stil netwerk, want bij verkeer verversen ack1's en tabelchunks het bewijs meestal eerder.
- **Massa-gebeurtenissen.** Na regionale stilte lopen veel bewijzen synchroon af. De golf spreidt vanzelf: elke node heeft zijn eigen rotatie en zijn eigen bevestigings-leeftijden (het laatste verkeer per link, geen gedeelde klok), en de lijstjes bundelen wat tegelijk afloopt.
- **Parameters.** `T_sym` startwaarde 6 uur (te toetsen in scenario 5); flush-marge 2 uur; kadans 20 minuten (rotatie).

**Status:** symmetriestatus (gezien/bevestigd) besloten; bewijslijst voorstel, te toetsen in scenario 5.

### 4.3 Tabel delen (push)

- **Push, geen pull (besloten, okt 2026).** De tabelinformatie is voor alle buren hetzelfde, dus de eigenaar deelt zijn tabel ongevraagd: elke chunk is een broadcast op zijn eigen thuiskanaal, in de vaste kadans (hierna). Buren snoopen mee. Er is geen pull-request, geen reparatie-pull en geen ack op chunks; gemiste chunks worden niet opgevraagd maar komen bij de volgende mutatie of de volgende ronde vanzelf terug.
- **Tabelchunks zijn zelf symmetriebewijs (besloten, okt 2026).** Publiceert een node "ik hoor X" (zijn entry voor X met kwaliteit), dan is bij X in één ontvangst de richting X→node bevestigd (de claim van de node) en de richting node→X vers gemeten (de ontvangst zelf) — hetzelfde werkingsprincipe als de bewijslijst (4.2). Daarmee vervalt de opstartpull als symmetrie-afmaker: bij een nieuwe node leveren zijn opstart-hello's de aan hem gerichte acks (zijn eigen bewijs), en zijn eerste tabelpush levert de responders hun tegenbewijs. De weg via het hello-proces (de nieuwkomer ackt een latere thuis-zending van de responder, sectie 2) blijft als extra bron overeind.
- **Doorsturen versus direct antwoorden (besloten, okt 2026).** De symmetrie-eis geldt voor het kiezen van een volgende hop voor data (routering, 10.1). Een direct antwoord in een exchange (ack1, ack2) mag de responder ook sturen als hij de partner alleen als "gezien" heeft. Het ontvangen verzoek bewijst de richting partner→responder, de partner wacht al op het kanaal, en de ack1 op het antwoord levert het ontbrekende bewijs alsnog. Weigeren zou de opstart van nieuwe nodes blokkeren.
- De tabel-header bevat de absolute geo van de eigenaar en een firmwareversie van 1 byte. De versie reist zo mee bij elke tabelpush, zonder extra bytes per pakket.
- De tabel bevat ook de links tussen de buren van de eigenaar. Die kent de eigenaar al uit de tabellen van zijn buren, dus er is geen extra pull nodig. Per link reizen de node-ID's van beide eindpunten mee (absolute ID's, geen indices; zie het chunkformaat hierna), met de kwaliteit in beide richtingen. Zo kent een node alle links binnen zijn 2-hop-verzameling, en wordt de Gabriel-graaf rond een gat compleet. Nodes op drie hops zijn standaard onbekend. Ze komen alleen binnen via onderhoud in rustige periodes (sectie 12, punt 15).
- Delen van tabellen van tweede-hopnodes gebeurt **niet** (die informatie reist al mee via de tabellen van directe buren). Strategisch uitbreiden van de routetabel in rustige periodes kan later worden overwogen.
- Bij het opstarten worden de eerste tabelchunks met een willekeurige vertraging gepland, zodat ze niet met de hello-ronde en andere nieuwkomers botsen.
- Tabellen groter dan één pakket gaan in chunks van circa 200 bytes, met volgnummers.

**Chunkformaat (okt 2026): zelfstandige, node-centrische chunks.** Elke chunk is in zijn geheel verwerkbaar zonder informatie uit andere chunks; de volgorde doet er niet toe en de ontvanger hoeft niet te weten of en wanneer de rest komt.

- **Node-centrisch.** Een chunk bevat een klein aantal volledige node-entries (8 B: node-ID, kwaliteit, leeftijd, geo), en direct na elke entry de links van die node: `(buur-ID 2 B, kwaliteitsbits per richting)` — breedte afhankelijk van de chunksoort, zie de kwaliteitsbreedte hieronder. Elke link staat onder precies één eindpunt (de eigenaar kiest), dus elke link reist één keer per tabel.
- **Absolute ID's, geen indices.** Link-verwijzingen zijn node-ID's, geen posities in de tabel van de verzender. Daarmee is geen mapping tussen de tabellen van verzender en ontvanger nodig, en is geen tijdelijke scratch-buffer of reassembly nodig: elke chunk wordt direct verwerkt tegen de eigen tabel (256 nodes, 13.2).
- **Chunkheader** (circa 10 B): type, state-versie, chunknummer, aantal nodes, en de eigenaar-geo met firmwareversie (4.3). De header herhaalt per chunk, zodat elke chunk zelfstandig is.
- **Kwaliteitsmeting (besloten, okt 2026): langdurig gemiddelde per richting.** De eerste meting initialiseert de opgeslagen waarde; elke volgende ontvangst updatet hem met een EWMA: `opgeslagen = (31 × opgeslagen + nieuwe waarde) / 32` (gewicht 1/32, een venster van ruwweg een half uur verkeer). Kleine kwaliteitsuitslagen worden zo vanzelf gedempt. Leeftijd per link bestaat niet: de leeftijd van een link is de leeftijd van zijn bron, en die reist per node-entry mee.
- **Kwaliteit op air (besloten, okt 2026): hybride breedte per chunk.** De geadverteerde linkkwaliteit is de bovenste bits van het langdurige gemiddelde: 3 bits per richting in de prefix-chunks (directe buren en hun onderlinge drempel-links — de links waar fijnkorreling routekeuzes bepaalt; 8 niveaus), 2 bits per richting in de staart-chunks (2-hop-links dienen greedy-richting en Gabriel-witnesses, geen hopselectie; 4 klassen volgens de NL-kalibratie 14.2: slecht onder −2,5 dB SNR, matig, goed boven +3,5 dB, uitstekend). De chunkheader krijgt een vlagbit voor de kwaliteitsbreedte, zodat elke chunk zelfstandig decodeerbaar blijft. Met 3 bits per richting (prefix) wordt een link 22 bits breed; met 2 bits (staart) 20 bits — beide gepakt op bytegrens (resp. 8 links per 22 B en 4 links per 9 B). Budget: de volle tabel blijft daarmee rond 30 tot 31 chunks, binnen de 32.
- **Advertentie-hysterese (besloten, okt 2026): pas delen bij verschil ≥ 2.** Een aanpassing van de geadverteerde kwaliteit wordt alleen doorgegeven als de nieuwe geadverteerde waarde minimaal 2 stappen van de vorige verschilt. Het gemiddelde mag over een klassegrens zwabberen zonder één pakket te triggeren; de advertentie-hysterese is onafhankelijk van de meet-hysterese voor de neighbor table limit (10.4) — twee dempers met twee doelen.
- **Statusvlaggen reizen niet mee.** Gezien/bevestigd, de kwaliteitsdrempel en het Gabriel-memberschap zijn afgeleid: de eigenaar stuurt alleen links die hij in beide richtingen gemeten heeft, en elke ontvanger berekent drempel en Gabriel-graaf zelf uit klasse en geo's (10.1, 10.3).
- **Broadcast per chunk.** Elke chunk is een eigen broadcast op het eigen thuiskanaal; geen exchange, geen ack, geen wissel. Snooping verwerkt hem als liveness + versie + kwaliteit voor de zender (de chunk is een eigen roep en reset de keepaliveteller van dat kanaal, 4.1).
- **Verlies (besloten: geen reparatie-pull).** Een gemiste chunk wordt niet opgevraagd. Herstel bestaat uit de regelmaat van de push zelf: de prefix wordt gedeeld bij de eerstvolgende directe-buurmutatie of bij afloop van het venster (de hele prefix opnieuw), de staart wordt periodiek volledig herhaald in de kadans. Omdat elke chunk zelfstandig verwerkbaar is, is er geen volgorde- of reassembly-afhankelijkheid en geen extra staat. De prijs: de ontvanger weet niet dát een chunk is gemist (geen ack), dus de leeftijd van zijn opgeslagen kopie is de enige maatstaf — oudere informatie verliest vanzelf van verse.
- **Prefix: de routekern eerst (besloten, okt 2026).** De vaste tabelindeling begint met de directe buren en hun onderlinge drempel-links (boven de kwaliteitsdrempel, 10.1). De eerste k chunks bevatten die routekern (typisch 8 tot 16 buren in één chunk; een hub met 64 buren heeft een prefix van enkele chunks). Chunks blijven zelfstandig verwerkbaar; de prefix-eigenschap gaat over de indeling van de verzender. Een ontvanger met alleen de prefix kan al volwaardig routeren; de staart is best-effort.
- **Kadans (voorstel, okt 2026): drie snelheden, gestuurd door de soort mutatie.** Het onderscheid tussen directe buren en andere nodes bepaalt hoe snel een aanpassing wordt gedeeld:
  1. **Prefix op mutatie van directe buren (snelste):** nieuwe buur, dooddetectie (`T_expire`), limit-churn (tombstone), of een kwaliteitsklasse-wissel die de hysterese overleeft (10.4) — de prefix gaat er direct achteraan, binnen het korte venster. Dit zijn de gebeurtenissen die routering raken: een nieuwe of dode directe buur verandert de mogelijke hopselectie voor iedereen in de omgeving. Vertraging daar is route-informatie achterhouden.
  2. **Prefix overig:** mutaties die alleen de rand raken (nieuwe 2-hop-node bij een buur, geo-wijziging) volgen het normale korte venster (maximaal een keer per 5 tot 10 min).
  3. **Staart:** uitsluitend de oplopende kadans (hierna).
  Mutaties binnen het venster bundelen vanzelf, want alleen de huidige versie gaat de lucht in — tussenversies bestaan niet on air. De soort mutatie is lokaal exact bekend (de node weet zelf wat hij zojuist veranderde), dus er is geen protocolveld voor nodig. De staatversie in de hello (4.1) blijft de enige aan de lucht zichtbare trigger. De staart wordt langzamer gedeeld, met een pauze tussen chunks die oploopt met het aantal directe buren n (per verdubbeling van n verdubbelt de pauze): de marginale luchtkosten van een chunk stijgen met n, de meeste tabelmutaties zitten bij hub-sites, en geen enkele node hoeft een drukte-meting te doen (open punt 15). Richtwaarde (okt 2026): ordegrootte 32 uur voordat een toren-site (n = 64, circa 30 chunks) zijn volledige tabel heeft gedeeld — basispauze circa 8 min, verdubbelend per bucket (n ≤ 8 → 8 min, ≤ 16 → 16 min, ≤ 32 → 32 min, ≤ 64 → 64 min per staart-chunk). Parameters: vensterlengte, basispauze en bucketgrenzen — te meten in scenario 2.
- **De traagheid is deel-vertraging, geen veroudering bij de bron (okt 2026).** De kennis van de zender is op het moment van verzenden vers: zijn eigen buren hoort hij direct, en zijn 2-hop-rand wordt hem aangeleverd door de prefixen van zijn buren (elke 5 tot 10 min) plus alles wat hij snoopt. Een toren met veel buren snoopt meer bronnen en heeft daardoor gemiddeld een *versere* rand dan een kleine node; de oplopende pauze vertraagt alleen het delen, niet de kennis. De ontvanger kan de deel-vertraging lokaal aftoppen, want n van de zender staat in diens prefix. Elke ontvanger rekent: werkelijke leeftijd = meegereisde leeftijd (zender-tijdstip, 4.4) + verwachte deel-vertraging f(n). Geen protocolveld, geen klok, geen synchronisatie — alleen de bucket-tabel in de firmware. Staart-entries worden daardoor niet weggefilterd maar gewogen: routering gebruikt ze voor richting (greedy, Gabriel), niet voor hopselectie; de symmetrie-eis (10.1) beperkt hopselectie toch al tot bevestigde directe buren.
- **Budget.** Volle tabel: 256 nodes × 8 B plus links (prefix 22 bits, staart 20 bits, zie de kwaliteitsbreedte hierboven) plus headers ≈ 5,7 tot 6,0 KB, dus 28 tot 31 chunks van de 200 B-klasse (869 ms), binnen de 32 van het geheugenbudget. Een volle push-ronde van een toren (circa 30 chunks) is daarmee circa 26 s lucht per 32 uur — verwaarloosbaar. De ontvanger slaat links die hij al heeft niet dubbel op; alleen het verschil telt.

**Status:** besloten.

### 4.4 Verval, tombstones en verouderde entries

**Dooddetectie (voorstel, okt 2026).** Per directe buur houdt een node lokaal bij wanneer hij die voor het laatst heeft gehoord. Elke afgeluisterde identificerende zending telt mee. De klok is lokaal, dus er is geen synchronisatie nodig. Na `T_expire` (startwaarde 4 uur) zonder ontvangst is de buur dood en verdwijnt de entry, zonder tombstone.

- `T_expire` is minimaal drie tot vier keer `T_keepalive` (4.1). Een levende, stille node zendt minstens zijn keepalive-hello per uur. Vier uur betekent dus vier gemiste keepalive-rondes, bovenop alle verkeer dat wordt afgeluisterd.
- Bij massa-uitval (bijvoorbeeld een stroomuitval in een regio) vervallen honderden entries tegelijk. Elke mutatie verhoogt de state-versie, en de push-golf die volgt moet gespreid blijven via de bestaande random vertraging en de kadans (4.3).
- De leeftijd per entry blijft de maatstaf voor wat een ontvanger vertrouwt.

**Leeftijdscodering (voorstel, okt 2026).** Het leeftijd-byte per entry codeert minuten; 255 = 4 uur of ouder, dus verlopen. Voor directe buren (prefix-chunks) filteren ontvangers verlopen entries weg. Voor staart-entries geldt het wegen, niet het weggooien (4.3): linkkwaliteiten van verlopen entries worden niet voor routering gebruikt, maar de node-informatie (ID, geo) blijft als kaartmateriaal staan.

**Waarom dooddetectie geen tombstone nodig heeft.** Een dode buur verdwijnt uiterlijk na `T_expire` uit elke geadverteerde tabel, omdat elke eigenaar dezelfde regel toepast. De leeftijdscodering zorgt dat ontvangers verlopen entries negeren. Een buur die alleen in de 2-hop-kaart leeft (Y hoort hem, wij niet) is geen zombie maar legitieme informatie met een verse leeftijd. Een tabelbrede tombstone zou hem ten onrechte blokkeren.

**Tombstone, beperkt tot zijn echte taak (okt 2026).** De tombstone is er voor verwijdering door de neighbor table limit op hub-sites (10.4). Die treft een levende buur die blijft zenden. Zonder tombstone stapt de buur binnen het keepalive-interval weer terug, oscilleert de neighbor table limit, en veroorzaakt elke mutatie een versiestijging en een push-golf.

- De tombstone blokkeert entries die indirect via tabellen binnenkomen. Een directe hello van de node zelf heft hem op.
- Duur: 5 keer het maximale keepalive-interval, dus 5 uur bij `T_keepalive` = 1 uur.
- Verwijdering door stilte gebruikt géén tombstone.
- Het aantal tombstones is begrensd door het geheugenbudget.

**Status:** voorstel voor `T_expire` en de leeftijdscodering (te toetsen in scenario 5). Tombstone-duur besloten.

---

## 5. Geo-encoding

| Eigenschap | Waarde | Status |
|---|---|---|
| Bereik | Wereldwijd | Besloten |
| Breedtegraad | 16 bits: 180° verdeeld over ca. 65.536 stappen, ≈ 0,002747° ≈ **306 m** per stap | Besloten (okt 2026) |
| Lengtegraad | 16 bits: 360° verdeeld over ca. 65.536 stappen. **Maximaal ≈ 612 m per stap op de evenaar** (slechtste geval), ≈ 376 m in Nederland (52° NB). Krimpt met cos(breedtegraad). | Besloten (okt 2026) |
| Totale grootte | **4 bytes** (16 + 16 bits, geen flagbyte) | Besloten (okt 2026) |
| Ongeldige geo | Sentinel (0, 0). Dat decodeert naar (−90°, −180°), de zuidpool/datumlijn-hoek, waar nooit een node staat. Een gereserveerde of gewiste entry is dus automatisch "geen geo". | Besloten (okt 2026) |

**Wijziging ten opzichte van de vorige vorm (5 bytes: 18 + 18 bits + 4 flagbits).** De geo wordt 4 bytes, per neighbor table entry en in de optionele bron-geo van elke data-header. Het raster wordt vier keer zo grof: 306 × 376 m in Nederland in plaats van 76 × 90 m. Dat is genoeg voor alle netwerkdoelen. Greedy-routering (10.2), de Gabriel-graaf en de kNN-straal werken op kilometerschaal, en grovere geo is beter voor privacy (doel 7). Voor weergaven houdt een companion zijn precieze GPS lokaal bij. Het oude doel van "ca. 100 m" is daarmee losgelaten voor de netwerkfunctie. De FULL-root-geo (6.5) gebruikte dit formaat al.

**Precisieniveaus vervallen als protocolveld.** Er zijn geen flagbits meer. Wie wil terugschalen (companions: 1 km of 10 km), zet de lage bits vóór publicatie op nul. Het niveau reist niet mee en is voor ontvangers niet af te leiden. Dat is acceptabel: routering heeft het niet nodig, en de weergave toont de gepubliceerde waarde toch al.

**Privacy.** De geo is publiek, omdat elke ontvanger het pakket kan lezen. Versleutelen breekt de routering, want relays hebben de geo nodig. Het raster van 306 m is de enige bescherming in het protocol zelf. Terugschalen vóór publicatie is de tweede laag, aan de clientkant.

---

## 6. Pakketheaders

### 6.1 Basisheader (data)

| Veld | Grootte |
|---|---|
| Type + flags | 1 byte |
| Message-ID | 2 bytes |
| Bron-ID | 2 bytes |
| Bestemming-ID | 2 bytes |
| Vorige hop-ID | 2 bytes |
| Budget | 1 byte (sectie 9), op offset 9 van de header |
| **Totaal** | **10 bytes** |

Optioneel: bron-geo (4 bytes, sectie 5), aanwezig als de flag is gezet. Dat is nodig als de bron buiten de kaart van de bestemming valt en de bestemming een ack3 terug moet sturen.

**Flags in het type-byte (voorstel):** ack3 gevraagd, bron-geo aanwezig, ack2 negatief, broadcast.

### 6.2 Acks

| Type | Grootte | Richting | Kanaal |
|---|---|---|---|
| ack1 | 7 bytes | Volgende hop → vorige hop | Zelfde kanaal als de data-uitwisseling; de verzender wacht daar volgens de wisselregel |
| ack2 | 7 bytes | Volgende hop → vorige hop | Thuiskanaal van de vorige hop |
| ack3 | Routed bericht, payload 4 bytes | Bestemming → bron | Via routering |
| hello-ack | 7 bytes, directioneel: type, bestemmings-id (de getriggerde zender), afzender-id, state-versie (sectie 2) | Responder → getriggerde zender | Bij een hello-ronde: thuiskanaal van de hello-verzender. Bij een thuis-zending van de trigger (ANN, FULL, of ack1 op het eigen kanaal): dat kanaal zelf, want de trigger-zender luistert daar per definitie. |
| bewijslijst | 7 + 2n bytes (n buur-ID's, n ≤ 17): type, node-ID (2), state-versie (2), daarna n × 2 B | Node → alle buren die op dat kanaal thuis zijn | Alleen de sub-lijst van buren die op kanaal c thuis zijn; max één per kanaal per uur (4.2). In de lijst staan = bewijs ververst, geen reactie; niet in de lijst = snooping. |

Ack1 bevat de identiteit van het oorspronkelijke bericht en de afzender van ack1.

### 6.3 Hello

5 bytes, zie sectie 4.1.

### 6.4 Aankondiging (ANN)

| Veld | Grootte |
|---|---|
| Type + flags | 1 byte (incl. kanaal van de FULL, 2 bits) |
| Thread-ID | 2 bytes |
| Message-ID | 2 bytes |
| **Totaal** | **5 bytes** |

### 6.5 Groot thread-bericht (FULL)

| Veld | Grootte |
|---|---|
| Type + flags | 1 byte |
| Thread-ID | 2 bytes |
| Message-ID | 2 bytes |
| Root-geo | 4 bytes (middelpunt van de thread, zelfde codering als sectie 5) |
| Straal | 1 byte (logaritmische schaal) |
| Voorganger message-ID | 2 bytes (0 bij de root) |
| Payload | n bytes |

**Status:** voorstel, zie sectie 8.

---

## 7. Bericht- en ackproces (unicast)

### 7.1 Doorsturen met dubbele ack

Voorbeeld: A → B → C → D.

1. A wisselt naar het thuiskanaal van B, zendt en ontvangt ack1 daar op hetzelfde kanaal. Daarna keert A terug naar zijn eigen thuiskanaal.
2. B probeert C, maximaal twee keer. Geen ack1 van C: B probeert D, eveneens maximaal twee keer.
3. Zodra B ack1 van D heeft ontvangen, stuurt hij direct ack2 naar A, op het thuiskanaal van A.
4. A weet dan dat het bericht twee hops is gekomen.
5. D stuurt een ack3 naar de bron, als die is gevraagd. Dat is een routed bericht.

**Status:** besloten.

### 7.2 Betekenis van ack2

- **Positief:** de volgende hop heeft het bericht **ontvangen**, niet per se doorgestuurd. Dat is bewust gekozen, zodat ack2 niet op de hele keten hoeft te wachten.
- **Negatief:** de volgende hop kon het bericht niet doorsturen. A kan dan direct een andere route kiezen.

**Verantwoordelijkheid per hop (begrensde backtrack).** De node die het bericht ontvangt, bewaart het bericht niet. Hij bewaart alleen een **pending-record** `(message-ID, vorige hop, volgende hop, teller)` in een vaste buffer van bijvoorbeeld 16–32 plekken (FIFO: als vol, dan vervalt de oudste). Dat record leeft tot `T_ack2` van de eigen hop verstrijkt of de retry-reeks is afgerond. Daarna blijft alleen de dedup-cache-entry over.

- Lukt doorsturen niet na de retries, dan probeert hij een andere buur (backtrack binnen het venster).
- Lukt dat ook niet, of valt de volgende hop uit ná het venster, dan stuurt hij een failed delivery report naar de bron, als routed bericht. Dat rapport genereert zelf geen nieuw rapport.

**Bewuste prijs:** na een positieve ack2 kan de bron niet meer op de vorige hop terugvallen als de fout later en dieper optreedt. Het enige signaal is dan het failure report of het uitblijven van ack3. Er is dus één terugkanaal voor late fouten: het routed failure report.

**Status:** besloten. Late negatieve meldingen komen via het routed failure report bij de bron (zie 10.5).

### 7.3 Timeouts

| Timeout | Formule | Status |
|---|---|---|
| ack1 (kort) | `t_data + t_ack + verwerking + marge`, gemeten vanaf het einde van de eigen zending | Besloten (principe) |
| ack2 bij A | `T_ack2 = 6 × (t_data + T1) + 5 × B_max + S + t_ack + m` | Voorstel |

- `T_ack2` hangt niet af van de padlengte, want A kijkt alleen naar B. `T_ack2` is ook de levensduur van het pending-record bij B: één timer met twee functies (timeout bij A, en het venster voor backtrack bij B).
- `S` is de kanaalwissel van het kanaal van de volgende hop terug naar het thuiskanaal van A.
- Valt B zelf uit, dan komt er geen ack1. A merkt dat op de ack1-timeout.
- Verliest B alle zes pogingen, dan stuurt hij direct een negatieve ack2.

### 7.4 Retries

- Maximaal drie buren per hop.
- Per buur maximaal twee zendpogingen.
- Ack1 ontvangen maar ack2 niet: wissel naar de volgende buur.
- Negatieve ack2: direct naar de volgende buur, zonder te wachten.
- Tussen pogingen naar dezelfde buur een willekeurige wachttijd, langer dan de airtime van het volgende pakket.

**Status:** besloten.

---

## 8. Threads en regionale chat

### 8.1 Concept

- Een thread is een gesprek dat door één starter is begonnen.
- De starter bepaalt het gebied. Het middelpunt is zijn eigen geo, en de straal wordt afgeleid van de dichtheid.
- Replies blijven binnen de straal van de starter. Een relay stuurt een reply alleen door als hij binnen die straal zit.
- Er zijn geen vaste cellen. Dat past bij gebieden met sterk verschillende dichtheden.
- Een thread heeft een eigen identiteit: thread-ID = hash van starter-ID en een teller of tijdstempel. Threads verstoren elkaar daardoor niet.

**Status:** besloten.

### 8.2 Straalbepaling

- De straal is de afstand tot de k-de dichtstbijzijnde node in de kaart van de starter, met `k ≈ 15`.
- Dichte gebieden krijgen een kleine straal, dunne gebieden een grote.
- De straal wordt in 1 byte gecodeerd op logaritmische schaal, van ca. 500 m tot ca. 60 km. Hogere waarden vallen af op de maximale code.
- De ondergrens is niet kritisch: een te kleine straal schaadt niemand. De bovengrens wel. Die is een firmwareconstante met startwaarde 40 km, geen waarde in het pakket.
- Relays wijzen een straal boven de cap van hun firmwareversie af. Een verhoging gaat gefaseerd: eerst accepteren alle nodes de nieuwe cap, daarna gebruiken starters hem.
- De straal wordt één keer per thread berekend en daarna in de header meegestuurd.

**Status:** besloten voor de methode (kNN). Open: exacte k en de grenzen.

### 8.3 Verspreiding binnen de straal

**Dominant pruning.** Een ontvanger herzendt alleen als hij buren heeft die de zender niet bereikt. Daarvoor gebruikt hij de neighbor table van de zender, die hij al heeft via 4.3. Er is dus geen extra headerveld voor de buurlijst.

**Wachttijd en onderdrukking.** Een kandidaat-herzender wacht een willekeurige tijd die korter wordt naarmate hij meer nieuwe buren kan bereiken. Hoort hij een herzending van iemand anders, dan herberekent hij zijn dekking met de buurlijst van die zender. Blijven er buren over die door geen van de gehoorde zenders gedekt zijn, dan wacht hij en zendt hij voor die rest. Is er niets meer over, dan zwijgt hij. Onderdrukking geldt alleen voor de kanalen die de node zelf afluistert. Duplicaten op andere kanalen vangt de dedup-cache op.

**Dedup.** Een cache op `(thread-ID, message-ID)` vangt herhalingen op.

**Status:** voorstel.

### 8.4 Herstel en sequentienummers

- FULL-berichten bevatten het message-ID van hun voorganger (2 bytes, 0 voor de root).
- Een node die FULL M ontvangt met een voorganger P die hij niet heeft, vraagt P op via unicast bij de zender van M. Bij uitval vraagt hij P bij een andere buur die P heeft. Dat herhaalt zich tot de root.
- **Staartverlies:** als het laatste bericht van een thread is gemist en er volgt niets meer, merkt niemand dat. De oorspronkelijke zender herhaalt daarom zijn ANN één keer na een willekeurige vertraging van enkele minuten. Nodes die FULL al hebben, onderdrukken dat. Dat kost één ANN van 5 bytes per bericht.
- **Herstel na een gemiste ANN:**
  - (a) elke herzender kondigt zijn eigen forward opnieuw aan op alle drie de kanalen, zodat een bericht meerdere keren langs nodes op elk kanaal komt;
  - (b) een FULL die op het thuiskanaal van een herzender wordt uitgezonden, wordt direct gehoord door nodes op dat kanaal, zonder ANN;
  - (c) een later bericht waarvan de voorganger ontbreekt, triggert een unicast-opvraag bij de bron van dat bericht, met wisseling naar diens kanaal.
- Nieuwkomers vragen bij het eerste contact met een buur de recente threads van die buur op. Dat is gebeurtenisgestuurd, niet periodiek, dus het kost niets in de hello.
- Threadinformatie komt niet in de hello. Er is geen periodieke digestvergelijking.

**Status:** voorstel. Open: de vertraging van de herhaalde ANN en de cachegrootte.

### 8.5 Aankondiging en groot bericht (ANN / FULL)

Om airtime te besparen, wordt het grote bericht maar één keer verstuurd:

1. De zender van een FULL stuurt eerst een ANN, op alle drie de kanalen in vaste volgorde, met een korte tussenpauze `g`.
2. Daarna stuurt hij de FULL op zijn eigen thuiskanaal.
3. Een node die een ANN hoort en de FULL nog niet heeft, wacht een korte willekeurige tijd `f`. Heeft hij de FULL in die tijd gehoord, dan stopt hij. Zo niet, dan schakelt hij naar het kanaal van de aankondiger en luistert op de FULL.
4. De timeout voor die fetch is `T_full = T_ANN + t_full + marge`.
5. Bij timeout probeert hij maximaal twee andere ANN-bronnen. Daarna is het sequentieherstel aan zet.

De wachttijd voor zenders gebruikt dezelfde logica als in 8.3.

**Parameters om te simuleren:** `W_base`, `J`, `g`, `F`, `marge`, en het maximum aantal fetch-pogingen (nu 2).

**Status:** voorstel. Parameters: open.

### 8.6 Nieuwkomers

- Elke node bewaart threads binnen zijn straal in een vaste hoeveelheid geheugen. Er is geen tijdslimiet. Is het geheugen vol, dan vervalt de thread met de oudste laatste activiteit (LRU).
- Laatste activiteit wordt bijgehouden met een lokale teller, niet met een klok. Er is dus geen tijdsynchronisatie nodig.
- Een nieuwe node vraagt na zijn eerste hello één buur om die cache. Dat gaat via unicast, niet via flooding.
- Een reply zonder root: de node vraagt de root op bij een buur.

**Status:** besloten (principe: vaste geheugengrootte met LRU). Cachegrootte: open.

### 8.7 Belastingbegrenzing

- Per thread kan een maximum aantal berichten per minuut gelden.
- Per zender geldt een wachttijd tussen berichten.
- Chat is best-effort. Voor threadberichten is ack3 niet nodig.

**Status:** voorstel.

---

## 9. Budget (vervangt TTL)

- Eén byte in de basisheader, beginwaarde **32**.
- Elke doorsturing kost 1. Elke retry kost 1, afgetrokken door de verzender voor het opnieuw zenden.
- Bij nul stuurt de node niet meer door en meldt hij dat negatief.
- Lussen zijn uitgesloten, want elke doorsturing haalt minimaal 1 af.
- Het budget is een totaal per bericht. De pogingen vermenigvuldigen zich dus niet over het pad.

**Status:** besloten. De waarde 32 moet in de simulatie worden getoetst.

### 9.1 Korte TTL voor threads

Threads kunnen met een kortere limiet werken, bijvoorbeeld drie bits. Dat levert wat extra ruimte in de flags op, hooguit één byte winst.

**Status:** voorstel. Een pad in de header is niet nodig, want dat kost meer dan het oplevert.

---

## 10. Routering

### 10.1 Kaart

- Per node: de directe buren met volledige tabel, en de buren van de buren als ID, geo en link. Dat is de 2-hop-kaart. Verder is een node niet bekend.
- Links worden alleen gebruikt als ze boven een kwaliteitsdrempel liggen en in beide richtingen werken.
- Elke richting moet door de ontvanger van die richting zijn bewezen:
  - Een hello of afgeluisterde hello-ack bewijst alleen zender→ontvanger ("gezien").
  - De tegenrichting komt uitsluitend van de buur zelf: via een aan ons gerichte hello-ack, via zijn tabelchunks (4.3), of via een geslaagde unicast met ack1 ("bevestigd").
- Routering kiest de volgende hop alleen via bevestigde links. Directe antwoorden in exchanges zijn van die eis vrijgesteld (4.3).

**Status:** besloten.

### 10.2 Keuze van de volgende hop

| Situatie | Regel | Status |
|---|---|---|
| Bestemming in de 2-hop-kaart | Kortste route op ETX-achtige kwaliteit | Besloten |
| Bestemming buiten de kaart, geo bekend | Greedy: de buur met de beste kwaliteit die dichter bij de bestemming ligt | Besloten |
| Geen buur dichterbij (gat, "void") | Terugval: waypoint of perimeter (zie 10.3) | Voorstel |
| Bestemming onbekend (alleen ID) | Niet flooden. Wachten tot de kaart de bestemming bevat, of een negatief rapport teruggeven | Besloten |

**Waypoints.** De bron kan een tussenpunt meesturen, bijvoorbeeld de geo van een repeater op de Afsluitdijk. Dat is stateless en goedkoop.

**Status:** voorstel.

### 10.3 Perimeter-routing (later)

- Basis: GPSR-achtige perimeter-modus.
- De Gabriel-graaf wordt berekend op de bekende links. Een link tussen u en v blijft bestaan als er geen witness w binnen de cirkel met diameter uv ligt.
- Witnesses liggen vrijwel altijd binnen de 2-hop-kaart, want de cirkel valt binnen het radiobereik.
- Een pakket in perimeter-modus draagt tien bytes extra: de geo van het startpunt van de perimeter en de laatste faceovergang.
- Beperking: links tussen twee-hop-nodes zijn niet bekend. De perimeter rond een gat kan dus onvolledig zijn. Dat moet in de simulatie worden getoetst, bijvoorbeeld rond het IJsselmeer.

**Status:** later. Eerst waypoints en failed delivery reports. Perimeter alleen als de simulatie dat vraagt.

### 10.4 Penalty en herstel

- Een mislukte zending verhoogt de penalty van de link. Opeenvolgende fouten vergroten de penalty. Succes laat hem langzaam dalen.
- De penalty is lokaal. Een gedeeld "verdacht"-veld kan later worden toegevoegd, als informatie en niet als besluit.
- Probing: periodiek een verzoek naar een gepenaliseerde buur, zodat herstel zichtbaar wordt.
- Hysterese en een minimale wachttijd voorkomen oscillatie.

**Status:** voorstel. Parameters: open.

### 10.5 Failed delivery report

- Een node die na alle retries niet heeft doorgestuurd, stuurt een negatief rapport terug naar de bron.
- Het rapport is een routed bericht, geen single-hop. Het gebruikt de normale routering naar de bron, met de bron-geo als die beschikbaar is.
- Inhoud: message-ID, node-ID van de melder, oorzaak, en optioneel de geo van de melder.
- Het rapport heeft een eigen klein budget, bijvoorbeeld de helft van het standaardbudget.
- Het verbetert de betrouwbaarheid niet. Het is informatie voor de client.
- Een node houdt een pending-record aan tot `T_ack2` van de eigen hop of tot de retry-reeks is afgerond. Daarna blijft alleen de dedup-cache over. Het bericht zelf wordt niet bewaard.
- Het rapport krijgt een vlag die voorkomt dat het zelf een rapport oproept.

**Status:** besloten. Het type wordt nu gereserveerd, de implementatie kan later.

---

## 11. Clients en ack3

- Ack3 is een clientfunctie. Routers sturen het alleen door als routed data.
- Ack3 wordt alleen gestuurd als de flag is gezet.
- Broadcast- en threadberichten vragen geen ack3.

**Status:** besloten.

---

## 12. Open punten

| # | Vraag | Waarom het telt |
|---|---|---|
| 2 | Straal op basis van lokale dichtheid, doel ca. 100 nodes. kNN werkt alleen op een vrijwel volledige 2-hop-kaart, dus zolang de kaart klein is geldt een bootstrap op linkafstanden. De bovengrens (startwaarde 40 km) is een firmwareconstante die nog in de simulatie moet worden getoetst. De ondergrens is niet kritisch. | Bepaalt de grootte van threads |
| 3 | Wachttijd W in rangen: W = slot × 3/n, met n het aantal nieuwe buren van de kandidaat. Jitter J kleiner dan de helft van een slot, zodat de volgorde tussen rangen zuiver blijft. Nog te toetsen: slotgrootte, `g`, `F`, marge. | Bepaalt dubbelwerk en botsingen |
| 4 | Budget 32: genoeg of te ruim? | Begrenst worstcase-verkeer |
| 5 | Companion-precisie: terugschalen van de gepubliceerde geo vóór publicatie; de niveaus reizen niet meer mee. Te bepalen welk niveau standaard is. | Privacy en nauwkeurigheid |
| 8 | RTS/CTS: experiment met en zonder. | Alleen nodig als hidden-node-botsingen veel verlies geven |
| 9 | SF8-nodes in de Nederlandse data: welk bereik en welke airtime. | Radiomodel |
| 10 | Chunkgrootte van tabellen (circa 200 bytes). | Fragmentatie |
| 11 | Accucapaciteit: voor zonne-repeaters is dit afgehandeld (continu luisteren is standaard). De vraag verschuift naar companions: accucapaciteit en levensduur bij continu luisteren, en of companions een luisterprofiel kunnen kiezen op basis van laadbron of accustand. | Companiongedrag, niet het protocol |
| 12 | Uitwerking companion: BLE-protocol naar de telefoon, koppeling met de thuis-repeater, en gedrag bij uit bereik zijn (geparkeerd, op de todo-lijst). Companions krijgen geen node-ID. Adressering gaat via de thuis-repeater en een slotnummer, zodat aan- en afmelden geen wijzigingen in de neighbor tables veroorzaakt. | Companion is nog grotendeels niet uitgewerkt |
| 13 | Node-ID als geo-tag plus random suffix: nog niet doorgevoerd. Nadeel: het ID verandert bij verplaatsing, headers worden langer, en de locatie zit altijd in het pakket. | Besparing zit vooral in de neighbor table |
| 14 | SF8-terugval voor afgelegen nodes. Een receiver luistert op één SF, dus de keuze geldt per link of per regio, niet per node. Geschatte winst: 25–30% bereik tegen ongeveer 2× airtime. | Radioontwerp, mogelijk per regio |
| 15 | Lokale drukte-meting (kanaalbezetting via CAD, eigen duty-cycle, faalpercentage van acks) om onderhoud te plannen in rustige momenten. Taken: (a) lokaal rekenwerk zonder airtime: Gabriel-graaf herberekenen, routes opnieuw uitrekenen, tombstones opruimen, dedup-cache comprimeren; (b) kleine unicast-taken: routeinformatie van 2-hop-buren ophalen, neighbor tables van buren verversen; (c) probes naar gepenaliseerde links, en kwaliteit in beide richtingen meten; (d) meetwerk voor de SF8-vraag. Niet toegestaan in rustige periodes: floods, brede pulls, en alles wat de hello verandert. Kennis uit buren krijgt een leeftijd en een bron, en weegt zwaarder naarmate ze recenter is. Zo reikt de kennis van de kaart verder dan twee hops, zonder dat één node alles hoeft op te halen. | Onderhoud zonder extra netwerkbelasting |
| 16 | Triggergeneralisatie (sectie 2): de hello-ack volgt op elke identificerende thuis-zending van een buur. Te meten in scenario 2: de stabiele ackbelasting mag niet boven het schema uitkomen (0,12–0,17% per kanaal). Ook: ackbotsingen bij ANN/FULL-triggers en de rangspreiding daar. | Bepaalt of keepalive-hello en gegeneraliseerde ack samen het rustverkeer dragen |
| 17 | `T_sym` en de flush-marge van de bewijslijst (4.2; startwaarden 6 uur / 2 uur), en de verhouding `T_keepalive : T_expire = 1 : 4` (startwaarden 1 uur / 4 uur, 4.4). | Routestabiliteit tegenover airtime; te toetsen in scenario 5 |
| 18 | Ackkanaal bij een keepalive-hello op kanaal c: (a) de verzender blijft op c in een kort venster, responders wisselen niet (voorkeur, sectie 2); of (b) de huidige regel: acks wisselen naar het thuiskanaal van de verzender. | (a) concentreert de thuis-doofheid bij één node per keepalive-hello; (b) verspreidt wissels over alle responders |

---

## 13. Hardware en implementatie

### 13.1 Doelplatforms

| Platform | RAM | Flash | Radio | Opmerking |
|---|---|---|---|---|
| nRF52840 (bijv. RAK4631) | 256 KB | 1 MB | SX1262 | BLE gebruikt een deel van het RAM |
| ESP32-S3 | 512 KB | 4 tot 16 MB | SX1262 | BLE en eventueel WiFi |
| ESP32 (klassiek) | 520 KB | 4 MB of meer | SX1276 | Veel boards hebben geen PSRAM |

### 13.2 Geheugenbudget (voorstel)

| Onderdeel | nRF52840 | ESP32-S3 |
|---|---|---|
| Beschikbaar voor het protocol | 48 KB | 128 KB |
| Threadcache (LRU, sectie 8.6) | 24 KB | 64 KB |
| Dedup-cache (512 entries van 4 bytes) | 2 KB | 2 KB |
| Neighbor table en 2-hop-kaart (256 nodes van 12 bytes) | 3 KB | 3 KB |
| Linklijst (1536 links van 5 bytes, lokaal formaat) | 7,5 KB | 7,5 KB |
| Vrij (chunk-reassembly vervalt met de zelfstandige chunks, 4.3) | 0 | 4 KB |
| Wachtrijen en zendbuffers | rest | rest |

### 13.3 Ontwerpregels

- **Taal:** firmware in C of C++. Python is alleen voor de simulatie. Op de chip is er te weinig RAM voor een interpreter, en de timing (ack-timeouts, CAD, kanaalwissels) vraagt om voorspelbare afhandeling via interrupts.
- **Gedeelde protocolkern:** de kern (hello, neighbor table, acks, routering, pruning, caches) is één C/C++-bibliotheek. De simulator roept die kern aan via cffi of ctypes. Zo draait de simulatie niet op een eigen implementatie die uit de pas kan lopen met de firmware.
- **Hardwarelaag:** radio, timers, RNG en flash zitten achter een dunne interface. Per platform wordt alleen die laag vervangen.
- **Geen dynamische geheugenallocatie:** alle buffers zijn vast en begrensd, conform de budgetten hierboven. Heapfragmentatie op 256 KB is een reëel risico.
- **Geo-berekeningen met gehele getallen** waar dat kan, met de 16-bits waarden uit sectie 5. De oost-westas schaalt met cos(breedtegraad). Binnen een regio is dat een constante, dus volstaat een vast breukgetal (bijvoorbeeld 154/250 ≈ cos 52° voor Nederland).
- **Willekeurige getallen** van de hardware-RNG voor backoff en jitter.
- **Eén radioabstractie** voor SX126x en SX127x. De modemparameters zijn gelijk.
- **Tijd** via een lokale teller en een hardware-timer. Er is geen klok nodig.
- **Begrensde 2-hop-kaart:** het aantal nodes wordt begrensd, zodat de Gabriel-berekening en de kNN-straal binnen het rekenbudget blijven.

**Status:** budgetten besloten. Open: of een BLE-companion op dezelfde chip concurreert met het protocol om RAM.

---

## 14. Simulatie

### 14.1 Lagen

1. Radio en kanaal: airtime, padverlies, capture effect, duty cycle.
2. Hello en neighbor table.
3. Data, ack1, ack2, retries en budget.
4. Routering: kaart, greedy, failed reports.
5. Gaten en perimeter (IJsselmeer).
6. Threads: straal, pruning, ANN/FULL, herstel.
7. Churn: uitval en terugkomst.

### 14.2 Metingen

- Delivery ratio per bericht (ack3 waar gevraagd, anders ack2).
- Oorzaak van falen: botsing, geen buur, budget op, gat.
- Airtime per kanaal en duty cycle.
- Budgetverbruik per bericht.
- Latentie (ter informatie).
- Dubbelwerk bij threads: verhouding FULL-zendingen tot het aantal nodes.
- Keepalive-hello's per node per kanaal, vertraging van versie-verversing, en bewijslijst-pakketten per kanaal per uur (`T_sym`, 4.2).

**Topologie:** echte repeaterlocaties als die beschikbaar zijn, anders synthetisch met dezelfde dichtheid per regio. Water wordt uitgesloten (bijvoorbeeld via OpenStreetMap).

**Kalibratie:** de SNR-grenzen van de Nederlandse kaart (slecht onder −2,5 dB, goed boven 3,5 dB) en de gemeten kanaalbezetting.

### 14.3 Bouwplan (voorstel)

**Architectuur.** Event-gestuurde discrete simulatie in Python. De gedeelde C-protocolkern (13.3) wordt via ctypes of cffi aangeroepen. Elke gesimuleerde node heeft een eigen instantie van de kern, zonder gedeelde state. De simulator levert de klok, de radio en de RNG aan de hardware-interface van de kern. Radio-events: `tx_start`, `tx_end`, `rx_begin`, `collision`. Tijd in microseconden, om afrondingsfouten bij lange runs te vermijden.

**Laag 1: radio en kanaal (eerst bouwen).**

| Grootheid | Model | Waarde (SF7, 62,5 kHz, CR4/5) |
|---|---|---|
| t_symbol | 2^SF / BW | 2,048 ms |
| hello / ANN (5 B) | LoRa-airtime | 70 ms |
| ack1 / ack2 / hello-ack (7 B) | idem | 78 ms |
| data (100 B) | idem | 459 ms |
| FULL / chunk (200 B) | idem | 869 ms |
| Hello-ronde (3 kanalen + pauzes `g`) | 3 × 70 ms + 2g | ca. 210 ms + pauzes |
| Ontvangst geslaagd | SNR = signaal − ruis + padverlies (pad, antennes); drempels −2,5 / +3,5 dB | NL-kalibratie |
| Botsing | Zelfde kanaal en overlappende tijd. Capture effect: de sterkste wint bij een marge van ca. 6 dB (te kalibreren). | Voorstel |
| Kanaalwissel `S` | Vast, bijvoorbeeld 5–10 ms inclusief frequentie-settling | Open |
| Duty cycle | Per node per kanaal, 10%, venster van 1 uur | Besloten |

**Radiomodel (voorstel, okt 2026; NL-kalibratie open).** Scenario's met meer dan enkele nodes gebruiken een knooppuntlijst met GPS-positie en hoogte (`id,lat,lon,hoogte_m,naam`). Het linkbudget per richting:

- ontvangst = TX (27 dBm ERP) − FSPL(869 MHz) − 2 × clutter(h), met clutter(h) = 30 − 14 · log10(1 + h) dB;
- radiohorizon: d ≤ 4,12 · (√h1 + √h2) km, met breking meegerekend;
- ontvangst lukt boven −110 dBm, met fade-marge ingerekend;
- per richting een deterministische fading-offset van ±5 dB, zodat links asymmetrisch zijn zoals in de praktijk.

Grof gekalibreerd: masten van 20–40 m reiken ongeveer 12–14 km, hoge masten (300–375 m) dekken een provincie.

**Lagen 2–3: hello, tabel, data en acks.** De kern draait de echte state machines: adaptief hello-interval, RSSI-gestuurde spreiding van hello-acks, wisselregel, ack1/ack2 met retries (3 buren × 2 pogingen), pending-records met `T_ack2`, en budget 32. De simulator meet bovenop de kern de thuis-doofheid per node, en mislukte unicast-zendingen naar oorzaak (botsing, geen ack1, buur weg).

**Scenario's (oplopend):**

1. **Koppeltest:** 2 nodes, hello, tabelpush en één unicast. Controleert dat de C-kern en de simulator het eens zijn over pakketgroottes, airtimes en ackvolgorde.
2. **Lokaal net:** 30–50 nodes op één dichtheid, uit een knooppuntlijst met GPS-positie en hoogte. Meet de hello-airtime en ackbotsingen bij de hello-acks (RSSI-spreiding), keepalive-hello's per node per kanaal, en de stabiele ackbelasting bij gegeneraliseerde triggers (ANN/FULL) ten opzichte van het schema (0,12–0,17%).
3. **Dichtheidsgrens:** 200, 500, 1000 en 3000 nodes, synthetische Nederlandse verspreiding. Airtime per kanaal, pieken in de duty cycle, en delivery ratio.
4. **Dun gebied en gat:** IJsselmeer/Flevoland-patroon. Straal-k, waypoints, failed reports.
5. **Churn:** nodes vallen uit en komen terug (winters, storingen). Tombstone-gedrag en herstel.

**Meetrapport per run:** delivery ratio (ack3 of ack2), faaloorzaken, airtime en duty cycle per kanaal, thuis-doofheid, budgetverbruik, latentie, en dubbelwerk bij threads.

**Uitgangspunt:** scenario 1 en 2 kunnen zonder threadlogica. Sectie 8 komt pas in beeld bij scenario 3 en daarna. Zo is er vroeg een werkende basis die de keuzes voor radio en acks (62,5 kHz, wisselregel, `T_ack2`) hard toetst.

---

## 15. Gewijzigde en vervallen ideeën

| Idee | Status | Reden |
|---|---|---|
| Gossip-routing als hoofdroute | Vervallen | Gekozen voor link-state met kaart |
| Distance-vector | Vervallen | Vervangen door de kaart |
| Vaste geografische cellen voor chat | Vervallen | Dichtheid verschilt te sterk |
| Gedeelde chatkanalen (#public, #test) | Vervallen | Broadcast-storm |
| Kanaal per chatregio | Vervallen | Vervangen door threads |
| Flood als terugval | Vervallen | Schaalt niet |
| Pad in de header | Vervallen | Kost meer dan het oplevert |
| Enkelvoudige hop-ack voor failure reports | Vervangen | Routed rapport is beter |
| Volledig tablepull-mechanisme (pull-request, reparatie-pull, ack per chunk, opstartpull als symmetrie-afmaker) | Vervangen (okt 2026) | Push naar alle buren: de informatie is voor iedereen hetzelfde, een hub zendt zijn tabel één keer in plaats van n keer. Chunks zijn zelf symmetriebewijs (zelfde principe als de bewijslijst, 4.2); herstel loopt via de regelmaat van de kadans, niet via opvragen |
| Versleutelde geo | Vervallen | Relays hebben de geo nodig |
| Repeater-WiFi standaard aan | Vervallen | Stroomverbruik op zonne-energie; nu standaard uit |
| Repeater-opslag voor companions | Vervallen | Companions halen threads op uit de threadcache van hun repeater |
| "Hello op behoefte", regels 1 en 2 (stale-buur- en versietrigger) | Vervangen (okt 2026) | Snooping en de gegeneraliseerde hello-ack dekken liveness en versie. Over blijft de per-kanaal-keepalive (4.1). |
| Hello-ack alleen na een hello | Vervangen (okt 2026) | Gegeneraliseerd naar elke identificerende thuis-zending, met de ack eligibility-conditie en cooldown; destijds inclusief een herbevestigingsvlag, later zelf vervangen (rij hierboven) |
| Verwijdering door stilte mét tombstone | Vervangen (okt 2026) | Stilte is dooddetectie (4 uur, 4.4) zonder tombstone. De tombstone blijft voor neighbor table limit churn op hub-sites (10.4). |
| Vlagbit "bevestig dit" + antwoordregel | Vervangen (okt 2026) | De bewijslijst (4.2) onderhoudt de bewijzen zonder vlag, zonder antwoord en zonder afhankelijkheid van ack-kansen; zelfde lucht per link, minder pakketten op hubs |
| Volle buur-ID-lijst in de keepalive | Vervallen (okt 2026) | Lucht is superlineaal (5 B = 70 ms, 133 B = 459 ms) en het pakket gaat elk uur elk kanaal naar iedereen; het "afgelopen uur"-filter filtert niets, want door de keepalive is elke levende buur per definitie binnen het laatste uur gehoord; routering heeft kwaliteit/geo/richting nodig. De deltavariant (bewijslijst) dekt het onderhoud; de tabelchunks (4.3) dekken de volledige informatie |