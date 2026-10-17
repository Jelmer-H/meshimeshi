
# Meshimeshi: ontwerpdocument

**Status:** werkdocument, nog niet gesimuleerd
**Taal:** Nederlands
**Laatst bijgewerkt:** 15 oktober 2026

Dit document legt de concepten vast die tot nu toe zijn besproken. Elk onderdeel heeft een status:

- **Besloten:** gekozen, tenzij de simulatie anders laat zien
- **Voorstel:** uitgewerkt, nog niet vastgesteld
- **Open:** nog te beslissen of te meten
- **Later:** bewust uitgesteld

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
| Spreading factor | SF7 als basis. In de Nederlandse data komt ook SF8 voor, dat moet in de simulatie | Voorstel |
| Bandbreedte | **62,5 kHz** (Besloten, okt 2026). Redenen: (a) bij 125 kHz overlappen drie kanalen op 100 kHz afstand elkaar — in de 250 kHz brede sub-band passen maar 2 schone kanalen van 125 kHz; (b) +3 dB gevoeligheid t.o.v. 125 kHz: meer links, kleinere voids, minder retries; (c) gelijke instellingen als het bestaande Nederlandse LoRa-netwerk op deze band (869,618 MHz, 62,5 kHz, SF7/CR5). Prijs: datarate halveert t.o.v. 125 kHz (±2,7 kbps bij SF7); alle time-outs en slotparameters zijn hierop berekend |
| Coding rate | 4/5 | Voorstel |
| Thuiskanalen | **869,44 / 869,50 / 869,56 MHz** (Besloten, okt 2026). Bij 62,5 kHz BW passen drie kanalen met 60 kHz spacing en ruime marge tot de bandranden binnen de sub-band 869,4–869,65 MHz. Oud plan (869,45/55/65 bij 125 kHz) vervallen: kanalen overlapten elkaar daar bij 25 kHz. Kanaal 4 op ±869,62 was overwogen (past exact zonder guard) maar afgevallen vanwege de bandrand; kan later heroverwogen worden |
| Thuiskanaal van een node | `node_id mod 3` | Besloten |
| Duty cycle | 10% per kanaal | Besloten |
| Collision-model | Twee overlappende pakketten op hetzelfde kanaal botsen, tenzij het sterkste met voldoende marge wint (capture effect) | Voorstel |

**Kanaalgebruik en wisselregel** (Besloten, principe)

- Elke node luistert continu op zijn eigen thuiskanaal.
- **Wisselregel:** een node wisselt alleen van kanaal voor een **gebonden uitwisseling** met een bekende partner (bekende pakketgrootte, korte bekende timeout), en keert direct daarna terug naar zijn thuiskanaal. Voor wachten van onbekende of lange duur blijft of gaat de node naar zijn eigen thuiskanaal; het antwoord komt daar vanzelf binnen.
- **Directe antwoorden op hetzelfde kanaal:** ack1 op data en acks op tabelpull-chunks worden afgewacht op het kanaal waar de uitwisseling loopt. De verzender is daar al; de ontvanger is thuis of al gewisseld.
- **Hello-acks: op het thuiskanaal van de hello-verzender.** Na een hello kunnen meerdere buren reageren, en die reacties worden over de tijd gespreid: **bij een vloer-hello op kanaal c (4.1) zijn er twee opties (open punt 18)** — (a) de verzender blijft op c gedurende een kort venster en de responders antwoorden daar zonder enige wissel; (b) deze regel, met acks via een wissel naar het thuiskanaal van de verzender. Voorkeur: (a), want nul wissels voor de responders, en de snooping-bonus van de acks valt op c — precies bij de doelgroep van de hello.
  - De verzender stuurt de hello op alle drie de kanalen in vaste volgorde en keert daarna terug naar zijn thuiskanaal. Responders sturen hun antwoord daar, binnen de maximale responstijd.
  - De maximale responstijd geldt als ontvangstvenster: de verzender vangt antwoorden op zijn thuiskanaal op tot die voorbij is.
  - **Spreiding via RSSI-afhankelijke wachttijd (Besloten): sterke buren reageren eerst, zwakke buren later.** De wachttijd is een dalende functie van de ontvangststerkte van de hello, in discrete slotten: RSSI bepaalt het slot (de rang), en een random jitter bepaalt de plek binnen het slot. Reden: RSSI is een analoge waarde die per ontvangst fluctueert (fading, interferentie); buren met vrijwel gelijke RSSI krijgen zonder jitter nagenoeg dezelfde wachttijd en botsen juist met elkaar. Sterke buren leveren de waardevolste kwaliteit-informatie voor de buurtabel en de tabellewin (4.2), en die komt zo het eerst binnen. De totale wachttijd blijft binnen de responstijd van de verzender.
    **Jitter boven de ack-airtime (Besloten, okt 2026, vervangt venster/64):** de jitter binnen het slot is venster/16 ≈ 187 ms, ruim boven de airtime van een ack (78 ms) plus de wissel (S). Bij het oude venster/64 ≈ 47 ms overlapten acks met exact gelijke rang zeker — aangetoond in de simulatie (nieuwkomer-scenario, uniforme links); nu botsen ze hooguit bij toeval. De slotberekening reserveert jitter + S + t_ack aan het einde van het venster, zodat elke ack — inclusief wissel en airtime — binnen het ontvangstvenster van de verzender valt. Prijs: de rangordening is voortaan "zacht": responders die minder dan ~16 dB uit elkaar liggen kunnen van volgorde wisselen. Geaccepteerd: botsingen voorkomen is het primaire doel, en "sterk eerst" blijft in hoofdzaken waar. Hoeveel gelijke-rang acks in de praktijk voorkomen meet scenario 2; voor verdere dichtheidsspreiding blijft slot × 3/n op de lijst (open punt 3).
  - **Formaat hello-ack (Besloten, okt 2026, vervangt het eerdere 5-byteformaat): 7 bytes, directioneel** — type (1), bestemmings-id (2, de zender van de gehoorde zending — voor de triggergeneralisatie hieronder alleen de hello-verzender), afzender-id (2), state-versie van de eigen buurtabel (2). Reden: doorsturen van unicast vereist vooraf bewezen symmetrie (10.1), en dat bewijs moet uit het hello-proces komen — niet pas uit de eerste unicast, die dan zelf al op een onbewezen link zou vertrekken. Alleen een aan ons gerichte ack bewijst beide richtingen: de responder ontving ónze hello (zijn ack is een reactie daarop) en zijn ack bereikte ons. De eerdere experimenten (7 B met 4 opvulnullen, daarna 5 B symmetrisch met de hello) deelden hetzelfde gebrek: geen bestemming, dus was een ack op een gedeeld thuiskanaal niet te onderscheiden van een antwoord aan een andere node en kon hij geen symmetrie bewijzen. Prijs van de adresbytes: één extra symboolbereik (78 t.o.v. 70 ms airtime); de state-versie reist mee en het botstingsvenster blijft één ack per responder.
  - **Hello-ack-snooping (Besloten, okt 2026): een ack gericht aan een ander is voor elke luisteraar verwerkbaar.** Elke node die zo'n ack hoort, verwerkt de afzender als een minimale hello: node-ID, state-versie en de lokaal gemeten ontvangstkwaliteit (4.2) gaan in de eigen buurtabel. De versie-informatie van een buur vernieuwt daardoor bij luisteraars vaak eerder dan op de volgende eigen hello-ronde van die buur, en de pull-beslissing (4.3) kan vroeger vallen. De snooper weet dat de ack niet voor hem is, dus de afzender blijft "gezien" (één richting). De ack zelf triggert geen nieuwe reactie — daarvoor dient het type-byte.
    **Waarom dit geen onterechte link-perceptie geeft:** een succesvolle ontvangst ís het bewijs van een inkomende link — een luisteraar registreert uitsluitend wat hij zelf meet (kwaliteit uit de eigen RSSI) of wat rechtstreeks van de bron komt (state-versie). Uit de ack valt niets af te leiden over de link waar hij op antwoordt (A↔B); de 2-hop-kaart wordt uitsluitend via tabelpulls (4.3) gevuld, met de tabel van de eigenaar als autoriteit. Randjes om in de gaten te houden: monsters uit snooping zijn gecorreleerd (bursts na andermans ronde, sterke buren acken eerder), en zwakke verre buren vullen de tabel — afzonderlijk echte metingen, gefilterd door de kwaliteitsdrempel bij gebruik en later door veroudering (4.4).
    **Tegenrichting (okt 2026, expliciet):** geen enkele ontvangst zegt uit zichzelf iets over de richting van onszelf naar de zender. Bewijs voor de tegenrichting kan uitsluitend van de buur zelf komen, in drie vormen: (a) een aan óns gerichte hello-ack — de aangewezen weg, want het hello-proces draait toch al en levert het bewijs vóór de eerste unicast; (b) een geslaagde unicast-uitwisseling met ack1; (c) straks een tabelpull van de buur waarin wij staan (4.3), met zijn eigen meting van ons signaal. Een aan een ander gerichte (afgeluisterde) ack blijft "gezien". Routering gebruikt uitsluitend bevestigde links (10.1): zien plaatst een buur in de tabel, pas het bewijs van de tegenrichting maakt hem een route.
  - **Vlagbit "bevestig dit" in het type-byte (Voorstel, okt 2026):** het formaat blijft 7 bytes. De zender kan met de vlag om een antwoord vragen — de periodieke herbevestiging van symmetrie (4.2). Alleen een ack mét vlag kan een antwoord oproepen; een ontvangen ack zónder vlag ververst het bewijs van de ontvanger (beide richtingen, zie hierboven) en triggert zelf nooit een reactie. Daarmee is elke ack-op-op-regress afgesneden: het verzoek is expliciet, niet afleidbaar uit een ontvangst.
  - **Triggergeneralisatie (Voorstel, okt 2026): de hello-ack volgt op elke geschikte zending van een buur, niet alleen op een hello.** Elke zending van X op zijn eigen thuiskanaal is een ack-kans, want X is daar per definitie thuis en luisteraar (wisselregel hierboven): de hello-ronde, de ANN-ronde, een FULL, en een ack1 van X op zijn eigen kanaal (X was ontvanger en wisselt niet). Niet ackbaar: zendingen van X als verstuurder van een gebonden uitwisseling (data, ack2) — X zit dan op het kanaal van de ander, en op een lopende uitwisseling wordt niet meegestuurd. Gesniffde data ververst wél de liveness en kwaliteit van X; alleen de ack-trigger vervalt. ANN en FULL zijn noch trigger noch zieling: die headers dragen geen node-ID (6.4/6.5). Anoniem is een privacy-keuze; prijs is dat thread-verkeer geen enkele waarneembaarheid oplevert, dus een node die alleen in threads praat vuurt gewoon zijn vloer-hello (4.1).
    **Beperkingssysteem, drie onafhankelijke poorten:** (1) nood-gate — er wordt alleen geackt bij nood: de link staat op "gezien" in plaats van bevestigd, het tegenbewijs is verouderd (T_sym, 4.2), of er staat hoffelijkheid open. In rust staat deze poort dicht; dát remt het aantal acks, niet het aantal triggers. (2) tempo-gate — het zielingsschema (1, 2, 4, 8, 16, daarna gemiddeld 1 op 16; 4.1) telt voortaan alle identificerende zendingen van de buur, retries daarvan als één zieling (dedup); na een eigen ack geldt een cooldown per buur. (3) spreiding — rang op RSSI + jitter zoals hierboven, want een FULL of ANN-ronde op een hub-site is voortaan ook een ack-moment; bij een thuis-zending van de trigger bestaat geen ontvangstvenster en dient de spreiding alleen tegen botsingen tussen ackers onderling.
  - **Responstijd en verblijf (Besloten):** de maximale responstijd geldt als venster: de verzender blijft op zijn thuiskanaal tot die voorbij is. Daarna blijft hij gewoon thuis — een node is immers standaard op zijn thuiskanaal — tot hij zelf iets te zenden heeft. Speciale "terugkeer"-logica is niet nodig; de hello-ronde is de enige situatie met een begrensd ontvangstvenster.
- Unicast data: de verzender wisselt naar het thuiskanaal van de ontvanger, zendt, wacht ack1 daar, en keert terug. Per hop maximaal twee wissels (heen en terug) voor de verzender; nul voor de ontvanger.
- Ack2: de volgende hop wisselt naar het thuiskanaal van de vorige hop, zendt, en keert direct terug. Eén gebonden transmissie.
- Hello: op **alle drie de kanalen**, in vaste volgorde (net als ANN). Reden: buren zitten willekeurig op alle drie de thuiskanalen; een hello op alleen het eigen kanaal maakt de buurtabel incompleet en breekt de 2-hop-kaart. De vaste volgorde geeft responders een voorspelbaar moment waarop de verzender weer op zijn thuiskanaal luistert. Antwoorden gaan daarheen (zie hello-acks hierboven). De extra kosten zijn klein (5 bytes × 3, met adaptief interval). **Rol in rust (Voorstel, okt 2026):** de vaste ronde geldt nog voor de opstartfase; daarna vervangt de per-kanaal-vloer (4.1) haar — een node zendt alleen nog op kanalen waarop hij langer dan T_vloer niet geroepen heeft, en de vaste volgorde komt te vervallen.
- Tabelpulls zijn unicast naar het thuiskanaal van de partner: gebonden uitwisseling, dus wisselen is toegestaan.
- Gedeelde chatkanalen zoals #public en #test vervallen.
- Aankondigingen (ANN, zie sectie 8) gaan op alle drie de kanalen, in vaste volgorde. FULL-fetch (8.5) is gebonden (`T_full`), dus de fetcher wisselt naar het kanaal van de aankondiger en keert terug.
- **Simulatormeting:** tijd per node buiten het eigen thuiskanaal ("thuis-doofheid"), naast airtime en duty cycle. Elk moment van kanaal is gemiste ontvangst op het eigen kanaal.

---

## 3. Nodetypen

| Type | Beschrijving | Status |
|---|---|---|
| Repeater | Vaste node, altijd aan, stuurt door | Besloten |
| Companion | Mobiel eindapparaat. Verbindt via BLE met de telefoon en heeft Ã©Ã©n vaste repeater als thuisbasis. Luistert continu en stuurt niet door. | Besloten |

**Repeater.** Werkt voornamelijk op zonne-energie met een kleine accu. WiFi-toegang staat standaard uit en kan op aanvraag of op vaste tijden aan. Er wordt geen geheugen gereserveerd voor companions.

**Energie (repeaters, besloten).** Continu luisteren is voor zonne-repeaters geen probleem en is daarmee de standaard. Rekenvoorbeeld (SenseCAP Solar P1-Pro: 5 W paneel, 4×3350 mAh = 13.400 mAh): de SX1262 trekt ~4,6 mA in RX; het hele systeem in luisterstand ~7–11 mA. Een winters nacht van 16 uur kost ~110–175 mAh (ca. 1% van de accu); zonder zon houdt het een repeater het enkele weken vol. Conclusie: accucapaciteit begrenst het protocol niet. Voor zeer kleine repeaters zonder zon geldt nog wel een lagere grens.

**Companion.** Luistert continu, zodat hij threads in bereik direct ontvangt. Luisteren kost accu, geen airtime, dus het belast het mesh niet. Er is geen slaapstand. Na het uit bereik zijn, of na een herstart, haalt de companion gemiste threads op bij zijn thuis-repeater uit de threadcache, zodra een ontvangen bericht een voorganger laat zien die hij mist. Daar is geen extra state voor nodig. Eigen berichten gaan via de thuis-repeater. Ack2 kan een companion missen als hij net uit bereik is; die bevestigt hij dan via de threadcache van de repeater. Directe berichten aan een companion die uit bereik is, leveren een failed delivery report op met de melding "eindapparaat niet bereikbaar".

Het onderscheid tussen beide typen volgt uit de gepubliceerde geo (sectie 5): repeaters publiceren het volledige raster, companions schalen vóór publiceren terug.

---

## 4. Hello en buurtabel

### 4.1 Hello

- Grootte: 5 bytes. Inhoud: type (1), node-ID (2), state-versie van de buurtabel (2). Een geowijziging verhoogt de state-versie, er is geen aparte geo-versie.
- Frequentie: adaptief. Bij opstarten 30 s, daarna oplopend naar 5 min, en uiteindelijk 1 uur in stabiele toestand. Langer aanwezige nodes sturen minder vaak.
- **Random fase/jitter op de hello-timer (Besloten, koppeltest oktober 2026):** elke hello wordt met een random vertraging gepland. Zonder jitter geldt: nodes die tegelijk opstarten (bijv. na stroomuitval in een regio) houden exact synchrone rondes — de intervallen verdubbelen synchroon — en horen elkaar nooit, want tijdens de eigen ronde is een node op een ander kanaal. De koppeltest liet dit zien; de oplossing staat nu in de protocolkern.
- **Per-kanaalvloer, opvolger van "hello op behoefte" (Voorstel, okt 2026 — te toetsen in scenario 2 en 5):** de hello dient twee doelen: versheid van bekende buren en ontdekkbaarheid voor onbekende. Liveness kan ook uit ander verkeer: elke ontvangen zending ververst `last_seen` van de zender — de zender staat in `prev_hop` bij data/ack3 en in `afzender` bij ack1/ack2/hello-ack; `src` telt nooit (een doorgestuurd bericht bewijst niet dat de originele bron in bereik is). De eigen state-versie reist al mee op de hello-ack (sectie 2) en komt er bij op de data-header (+2 bytes, meestal binnen hetzelfde symboolbereik; ack1/ack2 blijven 7 B — 9 B springt een bereik over). Daarmee degradeert de hello tot een per-kanaalvloer (okt 2026; de regels (1) en (2) van het eerdere voorstel vervallen): per kanaal c houdt een node bij wanneer hij daar voor het laatst "geroepen" heeft — elke identificerende zending op c telt (hello, hello-ack, ack1/ack2, data als `prev_hop`, pull-verkeer); ANN en FULL tellen niet, want anoniem (6.4/6.5). Is de timer verstreken (T_vloer, startwaarde 1 u), dan volgt precies één hello op c. De opstartfase houdt de ronde over alle drie de kanalen (30 s-interval); daarna bestaat de hello alleen nog als vloer. Versiepropagatie sleept daardoor maximaal één vloer-interval achter — de versie reist op de vloer-hello, de hello-ack (sectie 2) en de data-header — wat geaccepteerd is tegenover de vervaltermijn van 4 u (4.4) en de zelfcorrigerende routering. De timer reset op elke zending, dus actieve nodes desynchroniseren vanzelf; de random jitter (hierboven) blijft nodig voor stille regio's. Winst: op landelijke schaal is de hello de grootste rust-lucht (bij 3000 nodes en 1-uursinterval ≈ 5,8% per kanaal aan hellos, en de hello-acks zijn daar nog eens enkele malen bovenop), en de ronde krimpt naar de benodigde kanalen: minder wissels, en een node wiens verouderde buren alleen op het eigen thuiskanaal zitten zendt zonder enkele wissel. Nieuwkomers worden niet geschaad: hun opstart-hello's leveren de ontdekking, en de reactie daarop (hello-ack + opstartpull, 4.3) geeft beide kanten hun symmetriebewijs zonder dat de oude node hello'te. Risico's: onderlinge ontdekking van twee nog-onbekende nodes schuift door tot de vloer, en versiepropagatie wordt afhankelijk van aanwezig verkeer. Te meten: hello's/uur per node per kanaal, hello- en ack-airtime per kanaal, ontdekkingslatentie van nieuwkomers, versie-verversingsvertraging; T_vloer is parameter (startwaarde 1 u, gekoppeld aan T_expire, 4.4).
- **Hello-ack op behoefte (Voorstel, okt 2026 — responderkant van de per-kanaalvloer; minimale vorm geïmplementeerd in de kern):** onbevestigde links blijven opgeslagen, maar de ack-kans dunnet uit met de tijd: elke identificerende zending van de buur verhoogt een zichtingteller (hello's én thuis-zendingen, sectie 2; retries tellen als één zieling — dedup), en alleen zichtingen 1, 2, 4, 8, 16 leveren een ack-poging, daarna een druppel van gemiddeld 1 op 16 zichtingen met een minimale tijdsvloer per buur, zodat een blijvend asymmetrische link op een drukke hub geen onbegrensd ack-verbruik oplevert. Na eigen bevestiging (sym, 4.2) volgen nog twee hoffelijkheids-acks en dan definitief stilte — snooping (sectie 2) onderhoudt liveness, versie en kwaliteit al. De hoffelijkheid is noodzakelijk omdat bevestiging bijna altijd asymmetrisch verloopt: slaagt één richting eerst, dan stoppen die acken, terwijl de buur zijn bewijs nog uit onze ack moet halen (in de simulatie verloor capture precies die ene richting). Teller-reset bij bevestiging en hernieuwd contact. Of een zieling tot een ack leidt, bepaalt de nood-gate (sectie 2). Nog niet geïmplementeerd (bouwsteen 2): de cooldown per buur en de tijdsvloer op de druppel. Vervallen (okt 2026): acken bij eigen versiewijziging (de versie reist op de vloer-hello, de hello-ack en de data-header) en de reset op T_stale (vervangen door de bewijsveroudering met T_sym, 4.2). Motivatie en meting (40-node-scenario): toren-sites ontsteken per ronde een ack-storm op hun thuiskanaal; met het schema daalt de stabiele ack-lucht van 0,31-0,37% naar 0,12-0,17% per kanaal, botsingen van 25.435 naar 15.734 (-38%) en gemiste hellos van 5.296 naar 4.690, terwijl de bewijsdichtheid juist stijgt (bevestigd gem 6,2 -> 6,9) omdat de overgebleven pogingen vaker door gaan. Hub-sites zien daarnaast honderden buren: de tabel-kap van 64 werkt daar als rem, mits de kap stabiel is (top-N op sterkte met hysterese, 10.4) — churn op de kap zou de state-versie en daarmee de pull-frequentie laten oplaaien (4.3).
- Een nieuwe node krijgt snel een kaart dankzij de korte startintervallen.
- Threadinformatie komt niet in de hello (zie 8.4).

**Status:** Besloten, behalve de exacte intervallen (Open).

### 4.2 Buurtabel

Elke node adverteert alleen zijn eigen buurtabel. Per buur:

| Veld | Grootte | Toelichting |
|---|---|---|
| Node-ID | 2 bytes | |
| Inkomende kwaliteit | 1 byte | Gemeten door de node zelf (RSSI/SNR, gemiddeld over meerdere hellos en hello-acks; ook een afgeluisterde ack is een geldige meting van de inkomende link, zie sectie 2) |
| Leeftijd | 1 byte | Minuten sinds de buur voor het laatst is gehoord; 255 = 4 uur of ouder, verlopen (4.4) |
| Geo | 4 bytes (16 bits per as, sectie 5; (0,0) = geen geo) | Optioneel |

Elke richting van een link wordt door de ontvanger gemeten. Asymmetrie is daarmee zichtbaar.

**Symmetrie-status (Besloten, okt 2026):** naast de geadverteerde velden houdt elke node per buur lokaal bij of symmetrisch verkeer bewezen is: 0 = gezien (alleen de richting buur→wij gemeten), 1 = bevestigd symmetrisch. De bevestiging komt uitsluitend van de buur zelf: (a) een aan óns gerichte hello-ack — het bewijskanaal van het hello-proces, beschikbaar vóór de eerste unicast; (b) een geslaagde unicast-uitwisseling met ack1; (c) straks een tabelpull van de buur waarin wij staan (4.3), met zijn eigen meting van ons signaal. Een aan een ander gerichte (afgeluisterde) hello-ack bewijst geen symmetrie. Routering kiest de volgende hop uitsluitend via bevestigd-symmetrische buren (10.1); directe antwoorden in gebonden uitwisselingen zijn altijd toegestaan (4.3). Het meeadverteren van de status in de buurtabel — zodat 2-hop-nodes asymmetrie van hun buren zien — komt met de tabelpull van bouwsteen 2.

**Symmetrie op de lange termijn (Voorstel, okt 2026):** de bevestigde-status is niet eeuwig. Per buur houdt een node de leeftijd van het tegenrichtingbewijs bij ("laatst bewezen"), ververst door elke aan óns gerichte hello-ack, een geslaagde unicast met ack1, of de tabelpull van de buur waarin wij staan. Zonder onderhoud ontstaat er een impasse: de buur die als eerste opnieuw ackt (de druppel, 4.1), ververst alleen zijn ontvanger — de zender zelf leert niets van zijn eigen zending, dus diens bewijs verouderdt eeuwig terwijl de link gezond is. Oplossing: de vlagbit "bevestig dit" in de hello-ack (sectie 2). Bij bewijs-leeftijd T_sym/2 zendt een node zijn volgende hello-ack mét vlag; de ontvanger antwoordt precies één keer — zijn eigen bewijs is door de ontvangst alweer vers, zonder vlag zou hij zwijgen. Kosten: twee pakketten van 7 bytes per link per periode, vrijwel alleen in een stil netwerk, want bij verkeer verversen ack1's en tabelpulls het bewijs meestal eerder. De marge tussen T_sym/2 en T_sym vangt verloren uitwisselingen op; bij T_sym degradeert de link naar "gezien" en kiest routering (10.1) hem niet meer. Parameter: T_sym (startwaarde 6 u, te toetsen in scenario 5).

**Status:** Besloten.

### 4.3 Tabelophalen

- Een node haalt de buurtabel alleen op bij zijn directe buren, en alleen als de state-versie in de hello verandert. Eerste contact (nieuwe node, nog geen tabel opgeslagen van die buur) telt als versiewijziging.
- **Pulls completeren de symmetrie bij de responder (Besloten, okt 2026):** wanneer een nieuwe node opstart, levert zijn hello-ronde hémzelf de symmetriebewijzen (de aan hem gerichte acks, sectie 2/4.2); bij de responders staat de nieuwkomer daarna nog als "gezien". Hun tegenrichtingbewijs ontstaat langs twee wegen: (a) uit het hello-proces — de nieuwkomer ackt de volgende eigen ronde van de responder, wat bij een stabiele responder tot één hello-interval (max. 1 uur) kan duren; (b) uit de opstartpull hieronder — de puller bevestigt elke chunk met ack1, dus de responder ziet zijn eigen zending aankomen en markeert de nieuwkomer symmetrisch na de eerste chunk-ack. Weg (b) is daarmee naast kaartopbouw ook de snelle symmetrie-afmaker, en een extra reden om de opstartpull niet uit te stellen. (Okt 2026: met de gegeneraliseerde hello-ack, sectie 2, valt weg (a) vaak eerder — elke thuis-zending van de responder is voor de nieuwkomer een ack-trigger.)
- **Doorsturen versus direct antwoorden (Besloten, okt 2026):** het symmetrie-eis geldt het kíezen van een volgende hop voor data (routering, 10.1). Een direct antwoord in een gebonden uitwisseling — ack1, pullchunks, ack2 — mag de responder sturen ook al heeft hij de partner slechts "gezien": het ontvangen verzoek bewijst de richting partner→responder, de partner wacht al op het kanaal (wisselregel, sectie 2), en de ack1's op het antwoord leveren alsnog het ontbrekende bewijs. Weigeren zou de opstart van nieuwe nodes blokkeren: zonder te antwoorden kan de responder zijn eigen bewijs nooit verdienen.
- De tabel-header bevat de absolute geo van de eigenaar en een firmwareversie van 1 byte. De versie reist zo mee bij elke tabelpull, zonder extra bytes per pakket. Er is geen versiebit in de pakketheader.
- De tabel bevat ook de links tussen de buren van de eigenaar. Die kent de eigenaar al uit de tabellen van zijn buren, dus er is geen extra pull nodig. Per link: twee index-bytes binnen de buurlijst en de kwaliteit in beide richtingen. Zo kent een node alle links binnen zijn 2-hop-verzameling, en wordt de Gabriel-graaf rond een gat compleet. Nodes op drie hops zijn standaard onbekend. Die komen alleen binnen via onderhoud in rustige periodes (sectie 16), en dan komen hun links mee in dezelfde tabelpulls.
- Pulls van tabellen van tweede-hopnodes gebeuren **niet** standaard. Strategisch uitbreiden van de routetabel in rustige periodes kan later worden overwogen.
- Bij opstarten worden verzoeken met een willekeurige vertraging verstuurd, zodat de antwoorden niet op het thuiskanaal botsen.
- Tabellen die groter zijn dan een pakket worden in chunks van circa 200 bytes verstuurd, met volgnummers. Ontbrekende chunks worden opnieuw opgevraagd. Elke chunk krijgt een ack1.

**Status:** Besloten.

### 4.4 Verval, tombstones en verouderde entries

**Dooddetectie (Voorstel, okt 2026):** per directe buur houdt een node lokaal bij wanneer hij hem voor het laatst gehoord heeft — elke gesniffde identificerende zending telt; de klok is lokaal, geen synchronisatie nodig. Langer dan T_expire (startwaarde 4 u) geen ontvangst: de buur is dood, de entry verdwijnt — zonder tombstone. Koppeling: T_expire is minstens 3 tot 4 keer T_vloer (4.1), want een levende stille node zendt minstens zijn vloer-hello per uur; 4 u betekent vier gemiste vloer-rondes bovenop alle traffic-snooping. Massa-gebeurtenissen (stroomuitval in een regio) laten honderden entries synchroon vervallen: elke mutatie verhoogt de state-versie, en de pull-golf die volgt moet met de bestaande random vertraging (4.3) gespreid blijven. De leeftijd per entry blijft de maatstaf voor wat een ontvanger vertrouwt.

**Leeftijdscodering (Voorstel, okt 2026):** het leeftijd-byte per geadverteerde entry codeert minuten; 255 = 4 uur of ouder, verlopen. Ontvangers van tabelpulls filteren verlopen entries daardoor vanzelf weg.

**Waarom dooddetectie geen tombstone nodig heeft:** een dode buur verdwijnt uiterlijk T_expire uit élke geadverteerde tabel (elke eigenaar past dezelfde regel toe), en de leeftijdscodering laat ontvangers verlopen entries negeren. Een buur die alleen in de 2-hop-kaart leeft (Y hoort hem, wij niet) is geen zombie maar legitieme informatie met een verse leeftijd — een tabelbrede tombstone zou hem onterecht blokkeren.

**Tombstone, herbeperkt tot zijn echte taak (okt 2026):** verwijdering wegens de tabel-kap op hub-sites (10.4) treft een lévende buur die blijft zenden. Zonder tombstone stapt hij binnen het vloer-interval terug, oscilleert de kap, en veroorzaakt elke mutatie een versiestijging en een pull-golf. Voor die reden behoudt de tombstone met hysterese zijn functie: hij blokkeert entries die indirect via tabellen binnenkomen, en een directe hello van de node zelf heft hem op. De duur is 5 keer het maximale vloer-interval, dus 5 uur bij T_vloer = 1 u. Verwijdering door stilte gebruikt géén tombstone.

**Status:** Voorstel voor T_expire en de leeftijdscodering (te toetsen in scenario 5). Tombstone-duur besloten. Het aantal tombstones is begrensd door het geheugenbudget.

---

## 5. Geo-encoding

| Eigenschap | Waarde | Status |
|---|---|---|
| Bereik | Wereldwijd | Besloten |
| Breedtegraad | 16 bits: 180° in ca. 65.536 stappen ≈ 0,002747° ≈ **306 m** per stap | Besloten (okt 2026) |
| Lengtegraad | 16 bits: 360° in ca. 65.536 stappen ≈ 0,005493°; **max. ≈ 612 m per stap op de evenaar** (slechtste geval), ≈ 376 m in Nederland (52° breedte); krimpt met cos(breedtegraad) | Besloten (okt 2026) |
| Totale grootte | **4 bytes** (16 + 16 bits, geen flagbyte) | Besloten (okt 2026) |
| Geo ongeldig | Sentinel (0, 0): decodeert naar (−90°, −180°), de zuidpool/datumlijn-hoek — nergens een node. Voordeel: een gereserveerde of gewiste entry is automatisch "geen geo" | Besloten (okt 2026) |

**Wijziging t.o.v. de eerdere 5-byte-vorm (18 + 18 bits + 4 flagbits):** de geo krimpt van 5 naar 4 bytes — per buurtabelentry en in de optionele bron-geo van elke data-header. Het raster wordt 4× grover: 306 × 376 m in NL i.p.v. 76 × 90 m. Dat is voldoende voor alle netwerkdoeleinden: greedy-routering (10.2), de Gabriel-graaf en de kNN-straal spelen op kilometerschaal, en coarser is gunstiger voor privacy (doel 7). Voor gebruikersweergave houdt een companion zijn eigen precieze GPS lokaal bij; de gepubliceerde geo is het ruwe raster. Voor de netwerkfunctie is het oude "ca. 100 m"-precisiedoel daarmee losgelaten; de FULL-root-geo (6.5) gebruikte dit formaat al.

**Precisieniveaus vervallen als protocolveld.** Er zijn geen flagbits meer; wie wil terugschalen (companions: 1 km of 10 km) kwantiseert vóór het publiceren door de lage bits op nul te zetten. Het niveau reist niet mee en is voor ontvangers niet afleidbaar. Acceptabel: routering heeft het niet nodig, en weergave toont de gepubliceerde waarde toch al.

**Privacy:** de geo is publiek, omdat elke ontvanger het pakket kan lezen. Versleutelen breekt de routering, want relays hebben de geo nodig. Het 306 m-raster is de enige bescherming in het protocol zelf; terugschalen vóór publiceren is de tweede laag, aan de kant van de client.

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
| Budget | 1 byte (zie sectie 9), op offset 9 van de header |
| **Totaal** | **10 bytes** |

Optioneel: bron-geo (4 bytes, sectie 5), aanwezig als de flag is gezet. Dat is nodig als de bron buiten de kaart van de bestemming valt en de bestemming ack3 terug moet sturen.

**Flags in het type-byte (voorstel):** ack3 gevraagd, bron-geo aanwezig, ack2 negatief, broadcast.

### 6.2 Acks

| Type | Grootte | Verzender â ontvanger | Kanaal |
|---|---|---|---|
| ack1 | 7 bytes | Volgende hop → vorige hop | Zelfde kanaal als de data-uitwisseling; de verzender wacht daar volgens de wisselregel (sectie 2) |
| ack2 | 7 bytes | Volgende hop â vorige hop | Thuiskanaal van de vorige hop |
| ack3 | Routed bericht, payload 4 bytes | Bestemming â bron | Via routering |
| hello-ack | 7 bytes, directioneel: type (incl. vlagbit "bevestig dit", 4.2), bestemmings-id (getriggerde zender), afzender-id, state-versie (sectie 2) | Responder → getriggerde zender | Bij een hello-ronde: thuiskanaal van de hello-verzender. Bij een thuis-zending van de trigger (ANN/FULL/ack1 op eigen kanaal): dat kanaal zelf — de trigger-zender is daar per definitie luisteraar |

Ack1 bevat de identiteit van het originele bericht en de afzender van ack1.

### 6.3 Hello

5 bytes, zie sectie 4.1.

### 6.4 Aankondiging (ANN)

| Veld | Grootte |
|---|---|
| Type + flags | 1 byte (incl. kanaal van FULL, 2 bits) |
| Thread-ID | 2 bytes |
| Message-ID | 2 bytes |
| **Totaal** | **5 bytes** |

### 6.5 Groot thread-bericht (FULL)

| Veld | Grootte |
|---|---|
| Type + flags | 1 byte |
| Thread-ID | 2 bytes |
| Message-ID | 2 bytes |
| Root-geo | 4 bytes (middelpunt van de thread; dezelfde 16-bits codering als sectie 5: ca. 305 m breedte en 376 m lengte in NL) |
| Straal | 1 byte (logaritmische schaal) |
| Voorganger message-ID | 2 bytes (0 bij de root) |
| Payload | n bytes |

**Status:** Voorstel, zie sectie 8.

---

## 7. Bericht- en ackproces (unicast)

### 7.1 Doorsturen met dubbele ack

Voorbeeld: A â B â C â D.

1. A wisselt naar het thuiskanaal van B, zendt, en ontvangt ack1 daar op hetzelfde kanaal (wisselregel, sectie 2). Daarna keert A terug naar zijn eigen thuiskanaal.
2. B probeert C, maximaal twee keer. Geen ack1 van C: B probeert D, eveneens maximaal twee keer.
3. Ack1 van D ontvangen: B stuurt direct ack2 naar A, op het thuiskanaal van A.
4. A weet dan dat het bericht twee hops is gekomen.
5. D stuurt een ack3 naar de bron, als die is gevraagd. Dat is een routed bericht.

**Status:** Besloten.

### 7.2 Betekenis van ack2

- Positief: de volgende hop heeft het bericht **ontvangen**, niet per se doorgestuurd. Dat is een bewuste keuze, zodat ack2 niet op de keten hoeft te wachten.
- Negatief: de volgende hop kon het bericht niet doorsturen. A kan dan direct een andere route kiezen.

Verantwoordelijkheid per hop (begrensde backtrack): de node die het bericht heeft ontvangen, bewaart géén bericht, alleen een **pending-record** `(message-ID, vorige hop, volgende hop, teller)` in een vaste pending-buffer (bijv. 16–32 slots, FIFO; vol = oudste eruit). Dat record leeft tot `T_ack2` van de eigen hop verstrijkt of tot de retry-reeks is afgerond. Daarna vervalt het record en blijft alleen de dedup-cache-entry over. Lukt doorsturen niet na zijn retries, dan probeert hij een andere buur (backtrack binnen het venster). Lukt dat ook niet, of valt de volgende hop uit ná het venster, dan stuurt hij een failed delivery report terug naar de bron, als routed bericht (met message-ID, eigen node-ID, oorzaak). Dat rapport genereert zelf geen nieuw rapport.

Bewuste prijs: na een positieve ack2 kan de bron niet meer op de vorige hop terugvallen als de fout later en dieper optreedt; het enige signaal is dan het failure report en het uitblijven van ack3. Er is dus één terugkanaal voor late fouten: het routed failure report. Hop-state bestaat op dat moment niet meer.

**Status:** Besloten: begrensde backtrack via pending-records (geen volledige per-pakket state). Late negatieve meldingen komen altijd via het routed failure report bij de bron terecht (zie 10.5); daarmee is het eerdere open punt opgelost.

### 7.3 Timeouts

| Timeout | Formule | Status |
|---|---|---|
| ack1 (kort) | `t_data + t_ack + verwerking + marge`, gemeten vanaf het eind van de eigen zending | Besloten (principe) |
| ack2 bij A | `T_ack2 = 6 Ã (t_data + T1) + 5 Ã B_max + S + t_ack + m` | Voorstel |

- `T_ack2` hangt niet af van de padlengte, want A kijkt alleen naar B. `T_ack2` is óók de levensduur van het pending-record bij B: één timer, twee functies (timeout bij A, venster voor backtrack bij B).
- `S` is de kanaalwissel van het kanaal van de volgende hop terug naar het thuiskanaal van A. Per hop maximaal twee wissels voor de verzender; de ontvanger blijft op zijn thuiskanaal (wisselregel, sectie 2).
- Valt B zelf uit, dan komt er geen ack1. A merkt dat op de ack1-timeout.
- Als B alle zes pogingen verliest, stuurt hij direct een negatieve ack2.

### 7.4 Retries

- Maximaal drie buren per hop.
- Per buur maximaal twee zendpogingen.
- Ack1 ontvangen maar ack2 niet: wissel naar de volgende buur.
- Negatieve ack2: direct naar de volgende buur, zonder wachten.
- Tussen pogingen naar dezelfde buur een willekeurige wachttijd, groter dan de airtime van het volgende pakket.

**Status:** Besloten.

---

## 8. Threads en regionale chat

### 8.1 Concept

- Een thread is een gesprek dat door Ã©Ã©n starter is begonnen.
- De starter bepaalt het gebied: het middelpunt is zijn eigen geo, en de straal wordt afgeleid van de dichtheid.
- Replies blijven binnen de straal van de starter. Een relay stuurt een reply alleen door als hij binnen die straal zit.
- Er zijn geen vaste cellen. Dat past bij gebieden met heel verschillende dichtheden.
- Een thread heeft een eigen identiteit: thread-ID = hash van starter-ID en een teller of tijdstempel. Twee threads verstoren elkaar daardoor niet.

**Status:** Besloten.

### 8.2 Straalbepaling

- De straal is de afstand tot de k-de dichtstbijzijnde node in de kaart van de starter, met `k â 15`.
- Dichte gebieden krijgen een kleine straal, dunne gebieden een grote.
- De straal wordt in 1 byte gecodeerd op logaritmische schaal, van ca. 500 m tot ca. 60 km. Waarden daarboven vallen af op de maximale code.
- De ondergrens is niet kritisch: een te kleine straal schaadt niemand. De bovengrens is wel kritisch. Die is een firmwareconstante met startwaarde 40 km, geen waarde in het pakket.
- Relays wijzen een straal boven de cap van hun firmwareversie af. Een verhoging gaat gefaseerd: eerst accepteren alle nodes de nieuwe cap, daarna gebruiken starters hem. De firmwareversies in de tabellen bepalen wanneer dat kan.
- De straal wordt Ã©Ã©n keer per thread berekend en daarna in de header meegestuurd.

**Status:** Besloten voor de methode (optie 1, kNN). Open: exacte k en de grenzen.

### 8.3 Verspreiding binnen de straal

**Dominant pruning.** Een ontvanger herzendt alleen als hij buren heeft die de zender niet bereikt. Hij gebruikt daarvoor de buurtabel van de zender, die hij al heeft via sectie 4.3. Er is dus geen extra header-veld voor de buurlijst.

**Wachttijd en onderdrukking.** Een kandidaat-herzender wacht een random tijd die korter is naarmate hij meer nieuwe buren kan bereiken. Hoort hij een herzending van iemand anders, dan herberekent hij zijn dekking met de buurlijst van die zender. Blijven er nog buren over die door geen van de gehoorde zenders gedekt zijn, dan blijft hij wachten en zendt hij voor die rest. Is er niets meer over, dan zwijgt hij. Onderdrukking geldt alleen voor de kanalen die de node zelf afluistert; duplicaten op andere kanalen worden door de dedup-cache opgevangen.

**Dedup.** Een cache op `(thread-ID, message-ID)` vangt herhalingen op.

**Status:** Voorstel.

### 8.4 Herstel en sequentienummers

- Threadberichten (FULL) bevatten het message-ID van hun voorganger, 2 bytes, met 0 voor de root.
- Een node die FULL M ontvangt met een voorganger P die hij niet heeft, vraagt P op via unicast bij de zender van M. Bij uitval vraagt hij P bij een andere buur die P heeft. Dit herhaalt zich tot de root.
- Staartverlies: als het laatste bericht van een thread is gemist en er volgt niets meer, merkt niemand het. De oorspronkelijke zender herhaalt zijn ANN daarom Ã©Ã©n keer na een random vertraging van enkele minuten. Nodes die FULL al hebben, onderdrukken dat. Dat kost Ã©Ã©n ANN van 5 bytes per bericht.
- Herstel na een gemiste ANN: (a) elke herzender kondigt zijn eigen forward opnieuw aan op alle drie de kanalen, dus een bericht passeert meerdere keren langs nodes op elk kanaal; (b) een FULL die op het thuiskanaal van een herzender wordt uitgezonden, wordt direct gehoord door nodes op dat kanaal, zonder ANN; (c) een later bericht waarvan de voorganger ontbreekt, triggert een unicast-opvraag bij de bron van dat bericht, met wisseling naar diens kanaal.
- Nieuwkomers vragen bij het eerste contact met een buur de recente threads van die buur op. Dat is gebeurtenisgestuurd, niet periodiek, dus het kost niets in de hello.
- Threadinformatie komt niet in de hello. Er is geen periodieke digest-vergelijking.

**Status:** Voorstel. Open: de vertraging van de herhaalde ANN, en de cachegrootte.

### 8.5 Aankondiging en grote bericht (ANN / FULL)

Om airtime te besparen, wordt het grote bericht maar Ã©Ã©n keer verstuurd:

1. De zender van een FULL stuurt eerst een ANN, op alle drie de kanalen in vaste volgorde, met een korte gap `g` ertussen.
2. Daarna stuurt hij FULL op zijn eigen thuiskanaal.
3. Een node die een ANN hoort en FULL nog niet heeft, wacht een korte random tijd `f`. Heeft hij FULL gehoord, dan stopt hij. Anders schakelt hij naar het kanaal van de aankondiger en luistert op FULL.
4. De timeout voor die fetch is `T_full = T_ANN + t_full + marge`.
5. Bij timeout probeert hij maximaal twee andere ANN-bronnen. Daarna is het sequentie-herstel aan zet.

De wachttijd voor zenders gebruikt dezelfde logica als in 8.3.

**Parameters om te simuleren:** `W_base`, `J`, `g`, `F`, `marge`, maximum fetch-pogingen (nu 2).

**Status:** Voorstel. Parameters: Open.

### 8.6 Nieuwkomers

- Elke node bewaart threads binnen zijn straal in een vaste hoeveelheid geheugen. Er is geen tijdslimiet. Is het geheugen vol, dan vervalt de thread met de oudste laatste activiteit (LRU).
- Laatste activiteit wordt bijgehouden met een lokale teller, niet met een klok. Zo is er geen tijdsynchronisatie nodig.
- Een nieuwe node vraagt na zijn eerste hello Ã©Ã©n buur om die cache: unicast, geen flood.
- Een reply zonder root: de node vraagt de root op bij een buur.

**Status:** Besloten (principe, vaste geheugengrootte met LRU). Cachegrootte: Open.

### 8.7 Belastingbegrenzing

- Per thread kan een maximum aan berichten per minuut gelden.
- Per zender een wachttijd tussen berichten.
- Chat is best-effort. Voor threadberichten is ack3 niet nodig.

**Status:** Voorstel.

---

## 9. Budget (vervangt TTL)

- EÃ©n byte in de basisheader, beginwaarde **32**.
- Elke doorsturing kost 1. Elke retry kost 1, afgetrokken door de verzender voor het opnieuw zenden.
- Bij nul stuurt de node niet meer door en meldt hij dat negatief.
- Lussen zijn uitgesloten, want elke doorsturing haalt minimaal 1 af.
- Het budget is een totaal per bericht, dus de pogingen vermenigvuldigen zich niet over het pad.

**Status:** Besloten. De waarde 32 moet in de simulatie worden getoetst.

### 9.1 Korte TTL voor threads

Threads kunnen met een kortere limiet werken, bijvoorbeeld drie bits. Dat geeft een paar bits extra ruimte in de flags. Winst: hooguit Ã©Ã©n byte.

**Status:** Voorstel. Een pad in de header is niet nodig, want het kost meer dan het oplevert.

---

## 10. Routering

### 10.1 Kaart

- Per node: de directe buren met volledige tabel, en de buren van buren als ID, geo en link.
- Dat is de 2-hop-kaart. Verder is een node niet bekend.
- Links worden alleen gebruikt als ze boven een kwaliteitsdrempel liggen, en in beide richtingen werken. Elke richting wordt bewezen door de ontvanger van die richting: een hello of (afgeluisterde) hello-ack bewijst alleen zender→ontvanger ("gezien"); de tegenrichting komt uitsluitend van de buur zelf, via een aan ons gerichte hello-ack, via zijn tabelpull (4.3) of via een geslaagde unicast met ack1 ("bevestigd"). Routering kiest de volgende hop alleen via bevestigde links; directe antwoorden in gebonden uitwisselingen zijn vrijgesteld van die eis (4.3).

**Status:** Besloten.

### 10.2 Keuze van de volgende hop

| Situatie | Regel | Status |
|---|---|---|
| Bestemming in de 2-hop-kaart | Kortste route op ETX-achtige kwaliteit | Besloten |
| Bestemming buiten de kaart, geo bekend | Greedy: de buur met de beste kwaliteit die dichter bij de bestemming ligt | Besloten |
| Geen buur dichter bij (void) | Terugval: waypoint of perimeter (zie 10.3) | Voorstel |
| Bestemming onbekend (alleen ID) | Niet flooden. Wacht tot de kaart de bestemming bevat, of geef een negatief rapport terug | Besloten |

**Waypoints.** De bron kan een tussenpunt meesturen, bijvoorbeeld de geo van een repeater op de Afsluitdijk. Stateless en goedkoop.

**Status:** Voorstel.

### 10.3 Perimeter-routing (later)

- Basis: GPSR-achtige perimeter-modus.
- De Gabriel-graaf wordt berekend op de bekende links. Een link tussen u en v blijft als er geen witness w binnen de cirkel met diameter uv ligt.
- Witnesses liggen vrijwel altijd binnen de 2-hop-kaart, want de cirkel valt binnen het radiobereik.
- Een pakket in perimeter-modus draagt tien bytes extra: geo van het startpunt van de perimeter en de laatste faceovergang.
- Beperking: links tussen twee-hop-nodes zijn niet bekend. De perimeter rond een gat kan dus onvolledig zijn. Dat moet in de simulatie worden getoetst, bijvoorbeeld rond het IJsselmeer.

**Status:** Later. Eerst waypoints en failed delivery reports, perimeter alleen als de simulatie dat vraagt.

### 10.4 Penalty en herstel

- Een mislukte zending verhoogt de penalty van de link. Opeenvolgende fouten vergroten de penalty. Succes laat hem langzaam dalen.
- De penalty is lokaal. Een gedeeld "verdacht"-veld kan later worden toegevoegd als informatie, niet als besluit.
- Probing: periodiek een verzoek naar een gepenaliseerde buur, zodat herstel zichtbaar wordt.
- Hysterese en een minimale wachttijd voorkomen oscillatie.

**Status:** Voorstel. Parameters: Open.

### 10.5 Failed delivery report

- Een node die na alle retries niet heeft doorgestuurd, stuurt een negatief rapport terug naar de bron.
- Het rapport is een routed bericht, geen single-hop. Het gebruikt de normale routering naar de bron, met de bron-geo als die beschikbaar is.
- Het rapport bevat: message-ID, node-ID van de melder, oorzaak, en optioneel de geo van de melder.
- Het rapport heeft een eigen klein budget, bijvoorbeeld de helft van het standaardbudget.
- Het verbetert de betrouwbaarheid niet. Het is informatie voor de client.
- Een node houdt een pending-record aan tot `T_ack2` van de eigen hop of tot afgeronde retry-reeks; daarna blijft alleen de dedup-cache over. Het bericht zelf wordt niet bewaard.
- Het rapport krijgt een vlag die voorkomt dat het zelf een rapport oproept.

**Status:** Besloten. Het type wordt nu gereserveerd, de implementatie kan later.

---

## 11. Clients en ack3

- Ack3 is een clientfunctie. Routers sturen het alleen door als routed data.
- Ack3 wordt alleen gestuurd als de flag is gezet.
- Broadcast en threadberichten vragen geen ack3.

**Status:** Besloten.

---

## 12. Open punten

| # | Vraag | Waarom het telt |
|---|---|---|
| 2 | Straal op basis van lokale dichtheid: doel ca. 100 nodes. k-NN werkt alleen op een vrijwel volledige 2-hop-kaart, dus zolang de kaart klein is geldt een bootstrap op linkafstanden. Bovengrens (startwaarde 40 km) als firmwareconstante, nog te toetsen in de simulatie. Ondergrens niet kritisch | Bepaalt de grootte van threads |
| 3 | Wachttijd W in rangen: W = slot Ã 3/n, met n het aantal nieuwe buren van de kandidaat (buren die de zender niet bereikt). Jitter J kleiner dan de helft van een slot, zodat de volgorde tussen rangen zuiver blijft. Verder nog te toetsen: slot-grootte, `g`, `F`, marge | Bepaalt dubbelwerk en botsingen |
| 4 | Budget 32: genoeg of te ruim? | Begrenst worstcase-verkeer |
| 5 | Companion-precisie: terugschalen van de gepubliceerde geo (sectie 5) vóór publiceren; de niveaus reizen niet meer mee. Te bepalen welk niveau standaard is | Privacy en nauwkeurigheid |
| 8 | RTS/CTS: experiment met en zonder | Alleen als hidden-node-botsingen veel verlies geven |
| 9 | SF8-nodes in de Nederlandse data: welke reikwijdte en airtime | Radio-model |
| 10 | Chunkgrootte van tabellen (circa 200 bytes) | Fragmentatie |
| 11 | Accucapaciteit: voor zonne-repeaters afgehandeld (continu luisteren is standaard, zie sectie 3). Vraag verplaatst naar companions: accucapaciteit en levensduur bij continu luisteren, en of companions een luisterprofiel kiezen op basis van laadbron/accu | Bepaalt het companiongedrag, niet het protocol |
| 12 | Werking van de companion uitwerken: BLE-protocol naar de telefoon, koppeling met de thuis-repeater, en het gedrag bij uit bereik zijn (geparkeerd, op de todo-lijst). Companions krijgen geen node-ID. Adressering via de thuis-repeater en een slotnummer, zodat aan- en afmelden geen wijzigingen in de buurtabellen veroorzaakt | Companion is nog grotendeels niet uitgewerkt |
| 13 | Node-ID als geo-tag plus random suffix: nog niet doorgevoerd, op de lijst. Nadeel: ID verandert bij verplaatsing, headers worden langer, en de locatie zit altijd in het pakket | Besparing zit vooral in de buurtabel |
| 14 | SF8-terugval voor afgelegen nodes. Een receiver luistert op Ã©Ã©n SF, dus de keuze geldt per link of per regio, niet per node. Winst naar schatting 25 Ã  30% bereik tegen ongeveer 2x airtime | Radio-ontwerp, mogelijk per regio |
| 15 | Lokale drukte-meting (kanaalbezetting via CAD, eigen duty-cycle gebruik, faalpercentage van acks) om onderhoud te plannen in rustige momenten: extra routeinformatie ophalen, gepenaliseerde links proberen, en de Gabriel-graaf herberekenen. De meting blijft lokaal en verandert de hello niet | Onderhoud zonder extra netwerkbelasting. Taken: (a) lokaal rekenwerk zonder airtime: Gabriel-graaf herberekenen, routes opnieuw uitrekenen, tombstones opruimen, dedup-cache comprimeren; (b) kleine unicast-taken: routeinformatie van 2-hop-buren ophalen, buurtabellen van buren verversen; (c) probes: gepenaliseerde links proberen, en de kwaliteit in beide richtingen meten door een kleine probe te sturen; (d) meetwerk voor de SF8-vraag. Niet toegestaan in rustige periodes: floods, brede pulls, en alles wat de hello verandert. Kennis die via buren binnenkomt, krijgt een leeftijd en een bron, en weegt zwaarder naarmate ze recenter is. Zo kan de kennis van de kaart over tijd verder reiken dan twee hops, zonder dat Ã©Ã©n node alles hoeft op te halen |
| 16 | Triggergeneralisatie (sectie 2): de hello-ack volgt op elke identificerende thuis-zending van een buur. Te meten in scenario 2: de stabiele ack-lucht mag niet boven het op-behoefte-schema uitkomen (0,12–0,17% per kanaal); verder ack-botsingen bij ANN/FULL-triggers en de rangspreiding daar | Bepaalt of vloer-hello en gegeneraliseerde ack samen het rustverkeer dragen |
| 17 | T_sym voor de herbevestiging van symmetrie (4.2, startwaarde 6 u), en de koppeling T_vloer : T_expire = 1 : 4 (startwaarden 1 u / 4 u, 4.4) | Route-stabiliteit tegenover airtime; te toetsen in scenario 5 |
| 18 | Ack-kanaal bij een vloer-hello op kanaal c: (a) de verzender blijft op c in een kort ontvangstvenster, responders zonder wissel (voorkeur, sectie 2); of (b) de huidige regel: acks wisselen naar het thuiskanaal van de verzender | (a) concentreert de thuis-doofheid bij één node per vloer-hello, (b) verspreidt wissels over alle responders |

---

## 13. Hardware-doel

**Doelplatforms**

| Platform | RAM | Flash | Radio | Opmerking |
|---|---|---|---|---|
| nRF52840 (bijv. RAK4631) | 256 KB | 1 MB | SX1262 | BLE gebruikt een deel van het RAM |
| ESP32-S3 | 512 KB | 4 tot 16 MB | SX1262 | BLE en eventueel WiFi |
| ESP32 (klassiek) | 520 KB | 4 MB en meer | SX1276 | Veel boards hebben geen PSRAM |

**Geheugenbudget (voorstel)**

| Onderdeel | nRF52840 | ESP32-S3 |
|---|---|---|
| Beschikbaar voor het protocol | 48 KB | 128 KB |
| Threadcache (LRU, sectie 8.6) | 24 KB | 64 KB |
| Dedup-cache (512 entries van 4 bytes) | 2 KB | 2 KB |
| Buurtabel en 2-hop-kaart (ca. 200 nodes van 12 bytes) | 3 KB | 3 KB |
| Linklijst (ca. 1000 links van 5 bytes) | 5 KB | 5 KB |
| Chunk-reassembly (Ã©Ã©n per buur) | 4 KB | 8 KB |
| Wachtrijen en zendbuffers | rest | rest |

**Ontwerpregels voor portabiliteit**

- Vaste buffers en geen dynamische allocatie. Heap-fragmentatie op 256 KB is een reÃ«el risico.
- Geo-berekeningen met gehele getallen waar dat kan, met de 16-bits waarden uit sectie 5. De oost-west-as schaalt met cos(breedtegraad): binnen een regio is dat een constante, dus volstaat een vast breukgetal (bijv. 154/250 ≈ cos 52° voor NL).
- Willekeurige getallen van de hardware-RNG voor backoff en jitter.
- EÃ©n radio-abstractie voor SX126x en SX127x. De modemparameters zijn gelijk.
- Tijd via een lokale teller en een hardware-timer. Geen klok nodig.
- Het aantal nodes in de 2-hop-kaart wordt begrensd, zodat de Gabriel-berekening en de kNN-straal passen binnen het rekenbudget.

**Status:** Budgetten besloten. Open: of een BLE-companion op dezelfde chip RAM concurreert met het protocol.

## 13b. Implementatie-uitgangspunten

- **Firmware in C of C++.** Python is alleen voor de simulatie. Op de chip is er te weinig RAM voor een interpreter, en de timing (ack-timeouts, CAD, kanaalwissels) vraagt om voorspelbare afhandeling via interrupts.
- **Gedeelde protocolkern.** De kern (hello, buurtabel, ack's, routering, pruning, caches) is Ã©Ã©n C/C++-bibliotheek. De simulator roept die kern aan via cffi of ctypes. Zo draait de simulatie niet op een eigen implementatie die uit de pas kan lopen met de firmware.
- **Hardwarelaag.** Radio, timers, RNG en flash zitten achter een dunne interface. Per platform (nRF52840, ESP32-S3) wordt alleen die laag vervangen.
- **Geen dynamische geheugenallocatie.** Alle buffers zijn vast en begrensd, conform de budgetten in dit document.

## 14. Simulatie

**Lagen**

1. Radio en kanaal: airtime, padverlies, capture effect, duty cycle
2. Hello en buurtabel
3. Data, ack1, ack2, retries en budget
4. Routering: kaart, greedy, failed reports
5. Voids en perimeter (IJsselmeer)
6. Threads: straal, pruning, ANN/FULL, herstel
7. Churn: uitval en terugkomst

**Metingen**

- Delivery ratio per bericht (ack3 waar gevraagd, anders ack2)
- Oorzaak van falen: botsing, geen buur, budget op, void
- Airtime per kanaal, en duty cycle
- Budgetverbruik per bericht
- Latentie (ter info)
- Dubbelwerk bij threads: verhouding FULL-zendingen en het aantal nodes
- Vloer-hello's per node per kanaal, versie-verversingsvertraging, en herbevestigings-uitwisselingen (T_sym, 4.2)

**Topologie:** echte repeaterlocaties als die beschikbaar zijn, anders synthetisch met dezelfde dichtheid per regio. Water wordt uitgesloten (bijvoorbeeld via OpenStreetMap).

**Kalibratie:** de SNR-grenzen van de Nederlandse kaart (slecht onder -2,5 dB, goed boven 3,5 dB) en de gemeten kanaalbezetting.

### 14.1 Concreet bouwplan (Voorstel)

**Architectuur.** Event-gestuurde discrete simulatie in Python. De gedeelde C-protocolkern (13b) wordt via ctypes/cffi aangeroepen: elke gesimuleerde node heeft een eigen instantie van de kern (geen gedeelde state). De simulator levert de klok, de radio en de RNG aan de hardware-interface van de kern. Radio-events: `tx_start`, `tx_end`, `rx_begin`, `collision`. Tijd in microseconden (geen floats over lange runs).

**Laag 1 — radio en kanaal (eerst bouwen)**

| Grootheid | Model | Waarde (SF7, 62,5 kHz, CR4/5) |
|---|---|---|
| t_symbol | 2^SF/BW | 2,048 ms |
| hello / ANN (5 B) | lora-airtime | 70 ms |
| ack1 / ack2 / hello-ack (7 B) | idem | 78 ms |
| data (100 B) | idem | 459 ms |
| FULL / chunk (200 B) | idem | 869 ms |
| hello-ronde (3 kanalen + gaps `g`) | 3 × 70 ms + 2g | ~210 ms + gaps |
| Ontvangstgeslaagd | SNR = signaal - ruis + padverlies(pad, antennes); drempels -2,5 / +3,5 dB | NL-kalibratie |
| Botsing | op zelfde kanaal + overlappende tijd; capture effect: sterkste wint bij marge ≥ ~6 dB (te kalibreren) | Voorstel |
| Kanaalwissel `S` | vast, bijv. 5–10 ms incl. frequentie-settling | Open |
| Duty cycle | per node per kanaal, 10%, venster 1 uur | Besloten |

**Radiomodel in de simulator (Voorstel, okt 2026; NL-kalibratie: Open).** Scenario's met meer dan enkele nodes gebruiken een knooppuntlijst met GPS-positie en hoogte (`id,lat,lon,hoogte_m,naam`). Het linkbudget per richting: ontvangst = TX (27 dBm ERP) − FSPL(869 MHz) − 2 × clutter(h), met clutter(h) = 30 − 14·log10(1+h) dB en een radiohorizon d ≤ 4,12·(√h1 + √h2) km (breking meegerekend); ontvangst lukt boven −110 dBm (fade-marge ingerekend). Per richting een deterministische fading-offset van ±5 dB, zodat links asymmetrisch zijn zoals in de praktijk. Grof gekalibreerd: masten van 20–40 m reiken ~12–14 km, hoge masten (300–375 m) dekken een provincie. Kalibratie tegen de Nederlandse kaart (SNR-drempels −2,5/+3,5 dB) volgt.

**Laag 2–3 — hello, tabel, data, acks.** De kern draait de echte state machines: adaptieve hello-interval, RSSI-gestuurde hello-ack-spreiding, wisselregel, ack1/ack2 met retries (3 buren × 2 pogingen), pending-records met `T_ack2`, budget 32. De simulator meet bovenop de kern: **thuis-doofheid** per node, mislukte unicast-zendingen gedetailleerd naar oorzaak (botsing / geen ack1 / buur weg).

**Eerste scenario's (oplopend)**

1. *Koppeltest:* 2 nodes, hello + tabelpull + één unicast. Verifieert dat de C-kern en de simulator het eens zijn over pakketgroottes, airtimes en ack-volgorde.
2. *Lokaal net:* 30–50 nodes op één dichtheid, uit een knooppuntlijst met GPS-positie en hoogte (radiomodel hierboven). Meet hello-airtime en ack-botsingen bij de hello-acks (RSSI-spreiding), voorts vloer-hello's per node per kanaal en de stabiele ack-lucht bij gegeneraliseerde triggers (ANN/FULL) ten opzichte van het op-behoefte-schema (0,12–0,17%).
3. *Dichtheidsgrens:* 200, 500, 1000, 3000 nodes, synthetisch NL-verspreiding. Airtime per kanaal, duty-cycle pieken, delivery ratio.
4. *Dun gebied + void:* IJsselmeer/Flevoland-patroon. Straal-k, waypoints, failed reports.
5. *Churn:* nodes vallen uit en komen terug (winters, storingen). Tombstone-gedrag, herstel.

**Meetrapport per run:** delivery ratio (ack3/ack2), faaloorzaken, airtime + duty cycle per kanaal, thuis-doofheid, budgetverbruik, latentie, thread-dubbelwerk.

**Uitgang:** scenario 1 en 2 kunnen zonder thread-logica; sectie 8 komt pas in beeld bij scenario 3 en daarna. Zo is er vroeg een werkende basis die de radio- en ack-keuzes (62,5 kHz, wisselregel, T_ack2) hard toetst.

---

## 15. Gewijzigde en vervallen ideeÃ«n

| Idee | Status | Reden |
|---|---|---|
| Gossip-routing als hoofdroute | Vervallen | Gekozen voor link-state met kaart |
| Distance-vector | Vervallen | Vervangen door de kaart |
| Vaste geografische cellen voor chat | Vervallen | Dichtheid verschilt te veel |
| Gedeelde chatkanalen (#public, #test) | Vervallen | Broadcast-storm |
| Kanaal per chat-regio | Vervallen | Vervangen door threads |
| Flood als terugval | Vervallen | Schaalt niet |
| Pad in de header | Vervallen | Kost meer dan het oplevert |
| Enkelvoudige hop-ack voor failure reports | Vervangen | Routed rapport is beter |
| Tabelpull van tweede-hopnodes | Vervallen (standaard) | Te veel verkeer, mogelijk later off-peak |
| Versleutelde geo | Vervallen | Relays hebben de geo nodig |
| Repeater-WiFi standaard aan | Vervallen | Stroomverbruik op zonne-energie. Standaard uit, op aanvraag of op vaste tijden |
| Repeater-opslag voor companions | Vervallen | Companions halen threads op uit de threadcache van hun repeater |
| "Hello op behoefte", regels 1 en 2 (stale-buur- en versietrigger) | Vervangen (okt 2026) | Snooping en de gegeneraliseerde hello-ack dekken liveness en versie; overblijvend is alleen de per-kanaalvloer (4.1) |
| Hello-ack alleen na een hello | Vervangen (okt 2026) | Gegeneraliseerd tot elke identificerende thuis-zending, met nood-gate, cooldown en herbevestigings-vlag (sectie 2, 4.2) |
| Verwijdering door stilte mét tombstone | Vervangen (okt 2026) | Stilte is dooddetectie (4 u, 4.4) zonder tombstone; de tombstone resteert voor kap-churn op hub-sites (10.4) |