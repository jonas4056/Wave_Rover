# Wave Rover – Stappenplan

Doel op lange termijn: een robot die je met spraak iets kan vragen, zoals *"zoek de
televisie in deze ruimte en vertel me wat je erop ziet"*. Hij moet dan zelf door het huis
navigeren, kijken, begrijpen en antwoorden. Dat werk gebeurt op de Jetson; daarvoor
hebben we hem.

Werkwijze: we bouwen fase per fase. Een fase is pas klaar als alle punten onder
**"Klaar wanneer"** werken. Pas dan beginnen we aan de volgende.

Iets kleins tussendoor? Zie [Ideeën voor tussendoor](#ideeën-voor-tussendoor-nice-to-have).

---

## Hardware

| Onderdeel | Details |
|---|---|
| Computer | Jetson Orin Nano 8 GB, JetPack 7.2 (L4T R39.2.1, Ubuntu 24.04), NVMe 1 TB |
| Chassis | [Waveshare WAVE ROVER](https://www.waveshare.com/wiki/WAVE_ROVER), 4WD, **N20-motoren zonder encoders**, max. 1,25 m/s |
| Motorsturing | ESP32 general driver board, JSON via serieel (115200 baud). `T:1` stuurt de wielsnelheid (−0,5…+0,5), `T:131` zet de feedbackstroom aan. Heeft een **9-assige IMU** en een INA219 voor de batterijspanning. |
| RC | TBS Mambo (of TX16S) → TBS Crossfire Nano RX → UART van de Jetson (`/dev/ttyTHS1`, CRSF, 420000 baud) |
| Lidar | Slamtec RPLidar C1 (USB, 460800 baud) |
| Camera | CSI-camera (Argus) |
| Spraak (tijdelijk) | Yahboom-spraakbord: herkent alleen vooraf geprogrammeerde woorden |
| Spraak (later) | USB-microfoon en luidspreker |
| Visualisatie | Linux-pc met ROS 2 Jazzy (native) en RViz2 |
| Dashboard | Webapp: iPhone 18 Pro, of een oude iPhone/iPad die op de robot of op tafel ligt |

---

## Belangrijkste keuzes

### 1. ROS 2 Jazzy in Docker op de Jetson
- NVIDIA's software voor beeldverwerking en AI (Isaac ROS: cuVSLAM, nvblox, TensorRT-modellen)
  en de meeste AI-modellen voor de Jetson worden als containers geleverd. Isaac ROS 4.6
  ondersteunt Jazzy en JetPack 7.2 sinds augustus 2026. Voor de Orin Nano is het nog
  dun gedocumenteerd, en daarom testen we het pas in fase 4.
- We starten met een eenvoudig `ros:jazzy`-image. In fase 4 kan het basisimage
  vervangen worden door Isaac ROS zonder dat de rest verandert.
- Eén `docker-compose.yml` beschrijft alles:
  - host-netwerk, zodat de pc de ROS-topics rechtstreeks ziet
  - de USB- en UART-apparaten
  - de GPU
  - automatisch herstarten
- Gaat er iets mis, dan gooi je de container weg en bouw je hem opnieuw. Het
  JetPack-systeem zelf blijft schoon.
- De pc blijft native Jazzy draaien. ROS praat over het netwerk, dus dat werkt samen.

### 2. Odometrie zonder encoders
De motoren hebben geen encoders, dus de robot weet niet hoe ver zijn wielen gedraaid
hebben. In de oude versie bleef `/odom` daardoor altijd op positie (0,0,0) staan.
Voor SLAM en Nav2 leek het zo alsof de robot stilstond.

De nieuwe aanpak:
- **Lidar-odometrie:** de beweging wordt afgeleid uit opeenvolgende lidarscans, met
  `rf2o_laser_odometry` of KISS-ICP.
- **Gecombineerd met de IMU** (gyroscoop) in een EKF (`robot_localization`). Dat geeft
  een stabiele `odom → base_link`.
- **Snelheden kalibreren:** we meten welke `T:1`-waarde welke snelheid in m/s geeft,
  zodat Nav2 realistische commando's stuurt.

### 3. RC-zender heeft altijd voorrang
`twist_mux` kiest welke bron mag rijden. Volgorde van voorrang:
1. **noodstop**
2. **TX16S** (handmatig)
3. **dashboard** (iPhone)
4. **Nav2** (autonoom)

Een schakelaar op de TX16S dient als noodstop of als overname van de besturing.

### 4. Alleen de basis start automatisch
- **Altijd aan bij het booten:** ESP32-bridge, RC-besturing, twist_mux en robotmodel.
  Die zijn licht en moeten altijd werken.
- **Op vraag (modules):** lidar, SLAM, Nav2, camera, beeldherkenning en AI. Ze worden
  gestart en gestopt via een **module-manager** met een ROS-service per module.
  Het dashboard roept die services aan. Zo blijven CPU, GPU, geheugen en stroom vrij als
  de robot stilstaat.

### 5. Dashboard als webapp
- Een webpagina die op de Jetson draait en via `rosbridge` met ROS praat.
- Op de iPhone of iPad open je die in Safari en zet je hem met "Zet op beginscherm" als icoon op het beginscherm.
- Je hebt geen Mac, Xcode of Apple-ontwikkelaarsaccount nodig, en ze werkt ook op oudere iOS-versies.
- Een native app is enkel zinvol als de webapp tekortschiet.

---

## Ideeën voor tussendoor (nice to have)
Losse klussen die niet nodig zijn voor de volgende fase, om af en toe eentje tussen te nemen.
Grootte: **S** = een uurtje, **M** = een middag, **L** = meerdere sessies. Meer uitleg staat
bij de fase tussen haakjes; is een idee klaar, vink het dan hier en daar af.

**Veiligheid en rijden**
- [ ] (S-M) Gaten zien met de camera: floor_obstacles markeert ook wat onder de horizon
      *verder* is dan de vloer (trap naar beneden). Eerst beelden aan de trap opnemen (fase 3)
- [ ] (M) Cliff-sensor: ToF-sensor vooraan die naar de vloer kijkt, bv. VL53L1X (fase 3).
      Tot dan enkel verkennen met de trap afgesloten (deur dicht of iets van ≥25 cm hoog)
- [ ] (M) Verboden zones op de kaart (Nav2-keepoutfilter), bv. trap en tapijt; tekenen in de app
- [ ] (S) Links/rechts ongelijk bij ter plaatse draaien corrigeren (fase 3)
- [ ] (M) Locaties met een naam ("zetel", "tv") en er vanuit de app naartoe rijden (fase 3)
- [ ] (S) Batterij-% corrigeren voor de belasting: de app gaf 47%, maar in rust waren de cellen
      3,97 V gemiddeld (= ±73%). Onder belasting zakt de spanning, dus `power_guard` (20%)
      sluit de Jetson te vroeg af. Corrigeren met de gemeten stroom (spanning + stroom x
      weerstand), eerst de weerstand meten

**Pupper-inspiratie** (fase 6)
- [ ] (M) Spraak uit: Piper TTS op de USB-luidspreker (batterijmeldingen, "ik ben er")
- [ ] (S-M) Expressie: geluidjes of een gezichtje in het dashboard
- [ ] (M) Volgmodus: de dichtstbijzijnde persoon volgen (YOLO `person` + lidar-afstand)
- [ ] (L) Taalmodel met tools: rijden, een foto beschrijven, naar een locatie rijden

**Beeld** (fase 4)
- [ ] (M) Beeldtaalmodel (VLM) dat beschrijft wat de camera ziet
- [ ] (M) Herkende objecten op de kaart aanduiden (richting uit het beeld + lidar-afstand)
- [ ] (M) Open woordenschat (YOLOE / YOLO-World): zoeken naar willekeurige woorden
- [ ] (S) Kleurzweem (paars beeld) uitzoeken
- [ ] (S per functie) Gezichten, gebaren, kleur, beweging, lijn volgen

**Sneller en zuiniger** (Optimalisaties)
- [ ] (S) Ongebruikte Nav2-servers niet starten (`docking_server`, `route_server`)
- [ ] (M) pose_saver naar C++ (6-13% van een kern)
- [ ] (M) Detectie en vloerobstakels in één proces (±250 MB RAM minder)
- [ ] (M) JPEG in hardware decoderen (nu 9 ms CPU per beeld)

---

## Fases

### Fase 0 – Fundament
- [x] Docker en NVIDIA container runtime op de Jetson controleren
- [x] Mappenstructuur in de repo: `docker/`, `src/`, `scripts/`, `docs/`
- [x] Dockerfile (`ros:jazzy`) en `docker-compose.yml`
- [ ] udev-regels voor vaste apparaatnamen (`/dev/esp32`, `/dev/lidar`) → bij fase 1/2, als de apparaten aangesloten zijn
- [x] UART op de 40-pin-header: `/dev/ttyTHS1` is vrij (de console zit op `ttyTCU0`)
- [x] Netwerk tussen de Jetson-container en de pc instellen (`ROS_DOMAIN_ID`, zelfde DDS)

**Klaar wanneer:** een testtopic dat op de Jetson in de container gepubliceerd wordt,
zichtbaar is op de pc met `ros2 topic echo`.

### Fase 1 – Rijden met de TX16S
- [x] `wave_rover_base`: nieuwe ESP32-bridge (`/cmd_vel` → `T:1`, met daaruit de IMU, batterijspanning en een watchdog die stopt als er geen commando's meer komen)
- [x] CRSF-ontvanger → `/joy` → `rc_teleop` → `/cmd_vel` (TBS Mambo, TAER: arm op kanaal 5, snelheid op kanaal 6)
- [x] `twist_mux` met RC-voorrang; robot stopt bij ontwapenen of verlies van verbinding
- [x] URDF van de robot (afmetingen en lidarpositie nagemeten; camera en IMU nog geschat)
- [x] Alles start met één `docker compose up`, en automatisch bij het booten

**Klaar wanneer:** je met de TX16S vlot rijdt, de robot stopt als de zender uitvalt, en
je het robotmodel in RViz op de pc ziet.

### Fase 2 – Lidar en kaart maken
- [x] Module-manager: modules starten/stoppen/status via ROS-services (eerste module: lidar)
- [x] RPLidar C1 → `/scan` (Slamtec `sllidar_ros2`; lidar 180° gedraaid gemonteerd, gecorrigeerd in de URDF)
- [x] Lidar-odometrie (rf2o) + IMU → EKF → `odom → base_footprint`
- [ ] Kalibratie van `T:1` naar m/s
- [x] `slam_toolbox`: handmatig de living rondrijden en een kaart opbouwen (op snelheid 1)
- [x] Kaart opslaan (`living_v2`, op de Jetson in `~/wave_rover/maps`, niet in git)
- [ ] Lidar steviger bevestigen (M2,5-standoffs) en kaart opnieuw maken

**Klaar wanneer:** er een nette kaart van de living in RViz staat, zonder "dubbele"
muren.

### Fase 3 – Autonoom navigeren
- [x] Nav2 instellen voor een robot zonder encoders (RPP, op de plaats draaien; lokalisatie
      sinds 29-10-2026 met AMCL, zie hieronder)
- [x] Navigeren naar een punt dat je in RViz aanklikt
- [x] Zelf verkennen (module `explore`, eigen verkenner `explorer.py`, knop in de app bij Kaarten).
      Eerste volledige rit 29-10-2026: living + twee gangen in kaart (`livingauto2`, 13 doelen),
      met floor_obstacles aan; de camera zag de trap naar beneden als obstakel
- [x] Lidarscans rechtzetten voor het draaien (`scan_deskew`, 29-10-2026): de driver stempelt
      het begin van de omwenteling (97 ms) en de scan was bij draaien tot 7° uitgesmeerd.
      Nu stempel op het midden en elk punt met de gyro teruggedraaid; rf2o 8x nauwkeuriger
      bij ter plaatse draaien
- [x] Lokalisatie op een vaste kaart met **AMCL** (`OmniMotionModel`; het differentiële model
      liep weg bij ter plaatse draaien met zijdelings slippen). Na ritten 3 cm / 2,4° naast
      "Zoek mezelf" (slam_toolbox-lokalisatie: 12-14°). `localization:=slam` kan nog
- [x] Gyroregeling van de draaisnelheid in esp32_bridge (PI op de gyro, `yaw_loop`): bochten
      volgen nu 93-94% van wat Nav2 vraagt (was onderstuurd)
- [x] Niet rijden met de laadkabel in (`/charging_lock` van battery_monitor, lock in twist_mux;
      ook de RC en de app-joystick)
- [ ] Oranje stippen (en de robot) springen in de app tijdens het rijden. Gemeten 29-10-2026
      op `livingauto5` (`scripts/nav_log.py`): AMCL verbetert bij elke update (±0,3 s tijdens het
      rijden) 2-14 cm en 1-3°, afwisselend links en rechts; de schatting schommelt ±3-4°. Bij
      stilstand werkt AMCL niet meer bij en blijft de laatste schatting staan (na de rit 5,5°
      naast de muren); de odometrie zelf drijft bij stilstand maar ±1° per minuut. Te proberen:
      `resample_interval` 2-3, `/request_nomotion_update` elke seconde bij stilstand (AMCL blijft
      dan bijwerken), eventueel minder deeltjes. Geparkeerd op 29-10 voor iets leukers
- [ ] Verkennen: RPP meldt in smalle doorgangen vaak "collision ahead" en schuift terug;
      inflatie/collision-detectie van RPP afstellen
- [ ] Verkennen: floor_obstacles standaard mee laten starten (lage obstakels)
- [ ] Verkennen 29-10-2026 met de deur dicht (`livingauto5`): living in 2 min volledig in kaart,
      "Alles verkend" klopte. Daarna lukte terugrijden niet: vast bij meubels (lidar 15 cm),
      "collision ahead", achteruit en draaien mislukten; de richting liep ±10° weg. Opslaan
      mislukte omdat `map_saver_cli` maar 2 s op de kaart wacht (Jetson druk): nu 10 s
- **Trap/afgrond** (geparkeerd 29-10-2026, zie Ideeën voor tussendoor): floor_obstacles ziet alleen wat *uit* de vloer steekt; een trap naar
      beneden is een gat en wordt niet gezien (29-10-2026: tijdens het verkennen van de trap
      gereden, opgevangen). De lidar kijkt over het trapgat heen, dus de verkenner wil er
      heen. Voorlopig: enkel verkennen met de trap afgesloten (iets van ≥25 cm hoog) of met
      iemand klaar om te vangen. Mogelijk: in floor_obstacles ook "verder weg dan de vloer"
      (onder de horizon) als rand markeren; eerst beelden aan de trap opnemen om valse gaten
      (donkere vloer, schaduw, weerkaatsing) uit te sluiten. Een naar beneden gerichte
      ToF-sensor vooraan blijft het betrouwbaarste vangnet
- [ ] Links/rechts ongelijk bij ter plaatse draaien (rechts ±20% sneller, draait niet rond
      zijn as): opmeten met de lidar en correctie per kant in esp32_bridge
- [ ] Locaties met een naam opslaan ("zetel", "tv", "deur")

**Klaar wanneer:** de robot zelf de living in kaart brengt en daarna naar "tv" kan
rijden.

### Waar we gebleven zijn (stand 29-10-2026, middag)
Nieuw vandaag (zie fase 3): zelf verkennen, scans rechtzetten voor het draaien
(`scan_deskew`), AMCL als lokalisatie, gyroregeling voor bochten, niet rijden met de
laadkabel in. Pauze om te laden (batterij 47%), Jetson afgesloten.

Kaarten: `livingauto4` is de nieuwste (rechtgezette scans, scherpere muren), maar niet af:
verkennen gestopt omdat hij van de trap reed. Het tapijt is weg (de wielen draaiden erop
door). `livingauto2` heeft ook de gangen, maar met uitgesmeerde scans gemaakt.

Opstarten: Jetson aanzetten, thuis komt hij vanzelf op de wifi; app openen; modules aanzetten
in de app. Gebruik je de app al lang open? Hij herlaadt zichzelf na een update.

Volgende stappen, in deze volgorde:
1. Navigeren op `livingauto4` en meten waarom de stippen tijdens het rijden springen
   (AMCL-correcties?); zo nodig AMCL rustiger afstellen.
2. Eventueel opnieuw verkennen (trap afgesloten) voor een volledige kaart met de gangen.
3. Rijtest vloerobstakels: navigeren naar een doel achter een bal/kabel.
4. Tussendoor: iets uit de [Ideeën voor tussendoor](#ideeën-voor-tussendoor-nice-to-have).

Te controleren:
- **Batterijcel:** de zwakke cel laadde 's middags weer met 1,5 A (was 150 mA) en werd niet
  warm. 29-10-2026, halfleeg na gebruik en 15 min rust: 3,98 / 3,99 / 3,95 V (verschil
  0,04 V = goed). Daarna apart volgeladen in de externe lader (= weer gelijk).
  29-10-2026 14u, uit de lader, voor het terugplaatsen: 4,2 / 4,2 / 4,2 V (pas een paar uur
  rust, dus nog geen echte zelfontladingstest). Nog doen: na gebruik en 15 min rust de
  cellen opnieuw meten; blijft de zwakke cel duidelijk achter (> 0,05 V) = vervangen.
- `scripts/status.sh` aan het begin van elke sessie (batterijen, modules, NVMe, boots).

### Openstaand (stand 21-10-2026)
- [ ] Lidar steviger bevestigen (M2,5-standoffs), daarna kaart `living_v3` maken met de
      gecorrigeerde odometrie en de navigatie daarop testen
- [x] Lidarmotor altijd stoppen bij het uitzetten van de module, een crash of het herstarten
      van de container (`on_stop` in de module-manager + `lidar_stop`)
- [x] Kaart `living_v3` (stevige lidar + gecorrigeerde odometrie); navigatie werkt erop
- [x] Kalibratie draaisnelheid (23-10-2026, scripts/calibrate_motion.py): op de plaats
      86-104%, bochten 80-90%, rechtdoor ~100% (spin_gain 1.6, curve_gain 0.9)
- [x] Gezondheidscontrole in de module-manager: geen /scan gedurende 3 s -> lidar herstarten
      (zonder odometrie/navigatie te stoppen). Aanleiding: bij het uittrekken van de adapter
      schakelt de UPS over naar batterij en valt de lidar-USB ~1 s weg (gezien 23-10-2026);
      de driver bleef dan hangen zonder te stoppen.
- [ ] Aandachtspunt: de USB-chip (CP2102N) van de lidar liep één keer vast na snel na elkaar
      herstarten van de container ('cp210x_open - Unable to enable UART'); alleen USB uit- en
      insteken hielp. Komt het terug: automatische USB-reset of schakelbare USB-poort
- [ ] Spraakmodule testen: "display power", "robot stop", "go ahead"
- [x] Laatst bekende positie onthouden (`pose_saver` -> `<kaart>.lastpose`); de navigatie start daar
- [x] Globaal lokaliseren: knop "Zoek mezelf op de kaart" in het dashboard (28-10-2026,
      `relocalize.py` in map_manager). Eén lidarscan tegen de .pgm: grof zoeken op alle vrije
      plekken, fijn bijstellen, daarna per straal controleren of hij niet "door een muur kijkt".
      Op living_v3: zekerheid 0,56-0,63 tegen hoogstens 0,33 voor de beste andere plek; ~1 s CPU
- [x] Gyroscoop-nulpunt (28-10-2026): het ESP32-board gaf bij stilstand x +17, y -9, z -8 °/s,
      waardoor de robot op de kaart ronddraaide (4°/s) en slam_toolbox elke startpositie
      kwijtraakte. esp32_bridge meet het nulpunt nu zelf bij stilstand en trekt het af
- [x] Obstakels live op de kaart in het dashboard: oranje = lidar, rood = camera (vloerobstakels)
- [x] Wifi-hotspot `WaveRover` als er geen bekend wifinetwerk is (bv. bij de ouders), zie
      Systeeminstellingen. Getest 28-10-2026 met de iPhone: app, navigatie en "Zoek mezelf" werken
- [ ] Snel een nieuwe ruimte in kaart brengen en daarna navigeren (workflow + dashboardknop)
- [ ] Zelf verkennen (exploration) zodat de robot zelf een kaart maakt
- [ ] Locaties met een naam, gekoppeld aan "go to the point A..D"

### Navigatie- en rijprestaties verbeteren (zodra de UPS-batterij terug werkt)
- [ ] Route strakker volgen en vloeiender bochten in auto-modus (RPP verder afstellen of
      opnieuw MPPI proberen nu de draaisnelheid gekalibreerd is)
- [x] Lokalisatie tijdens het draaien (scans die even naast de muren liggen): opgelost op
      29-10-2026 met `scan_deskew` en AMCL (zie fase 3)
- [ ] Testrondes met meerdere doelen, slaagpercentage bijhouden (demo: 6 van 11 aangekomen)

### Optimalisaties (CPU/RAM sparen voor de AI)
- [x] Nav2 met `use_composition` (één proces i.p.v. ±12)
- [x] RC: `/joy` aan 50 Hz i.p.v. elk CRSF-pakket (150 Hz). Daarbij gevonden: de UART werd
      per 256 bytes gelezen, waardoor pakketten per ~7 en tot 50 ms te laat kwamen; nu direct
- [ ] Zware Python-nodes (esp32_bridge, lidar_odom_fix) eventueel naar C++
- [ ] pose_saver naar C++: TF ontvangen (40 berichten/s) kost in Python alleen al ~6% van
      een kern; met de obstakels voor het dashboard samen ~13% (gemeten 28-10-2026)
- [x] Gemeten (28-10-2026, navigatie aan, stilstaand): CPU 1,78 -> 1,11 kernen, geheugen ROS 2049 -> 1584 MB,
      47 -> 36 processen

### Fase 4 – Camera en beeldherkenning op de GPU
- [x] CSI-camera: Argus draait op de host (GStreamer, hardware-JPEG), de container krijgt de
      beelden via TCP
- [x] Camera in ROS als module (`/camera/image/compressed`, 30 fps) en live in het dashboard;
      `/camera/image_low/compressed` (10 Hz) voor herkenning. Een topic zonder luisteraars
      wordt niet gepubliceerd; de health-controle kijkt naar `/camera/frames` (1 Hz-teller)
- [x] GPU in de container: TensorRT 10.16 + CUDA 13.2 in het image, driver via CDI
- [x] Module `detection`: YOLO26n (COCO, 80 klassen) met TensorRT FP16 -> `/detections`.
      Model in `~/wave_rover/models` (`scripts/export_yolo.sh`); engine wordt de eerste keer
      gebouwd (±6 min, daarna enkele seconden). Gemeten 28-10-2026 aan 10 Hz: 19% van één
      kern, ~680 MB RAM (waarvan ~320 MB gedeelde CUDA/TensorRT-bibliotheken), GPU 4 ms per
      beeld op volle klok (12 ms bij 10 Hz: de GPU klokt tussen de beelden terug)
- [x] Detecties in het dashboard (kaders over het camerabeeld)
- [ ] JPEG decoderen in hardware of ruwe beelden via de VIC van de host (nu 9 ms CPU per beeld)
- [ ] Objecten op de kaart aanduiden (richting uit het beeld + lidar-afstand)
- [ ] Open woordenschat (bv. YOLOE / YOLO-World): zoeken naar willekeurige woorden i.p.v.
      enkel de 80 COCO-klassen
- [ ] Beeldfuncties (zoals de Waveshare UGV Rover): objectherkenning, gezichtsdetectie,
      gebaren, kleurherkenning, bewegingsdetectie, lijn volgen
- [ ] Diepte: eerst monoculair (Depth Anything op de GPU); later eventueel een OAK-D (Lite)
      i.p.v. twee losse IMX219's als stereo (geen hardware-sync, stijve montage nodig)
- [ ] Kleurzweem (paars beeld): NoIR-module of ontbrekende ISP-kalibratie uitzoeken
- [x] **Lage obstakels** (speelgoed, kabels; de lidar scant op 20 cm): module `floor_obstacles`.
      Depth Anything V2 Small (TensorRT) + vaste cameragegevens: alles wat uit de vloer steekt
      wordt een punt op `/floor_scan` (virtuele lidar op vloerhoogte), in Nav2 een eigen laag
      `floor_layer` (de lidar mag ze niet wissen; blinde zone < 0,35 m wordt niet gewist).
      Gekalibreerd 28-10-2026 (`scripts/calibrate_floor.py`): lens 10,5 cm hoog, f 989,5 px,
      quasi waterpas. Getest stilstaand: bal (9 cm) op 60 cm correct in de costmap (±3 cm).
      Kost aan 5 Hz: 16% van één kern, ~590 MB RAM (waarvan ~300 MB gedeeld met CUDA)
- [ ] Rijtest: navigeren naar een doel achter een bal/kabel (rijdt hij er omheen?)
- [ ] Enkel vooruit zichtbaar (66 graden) en blinde zone vlak voor de robot; eventueel
      goedkope ToF-sensoren laag vooraan als vangnet
- [ ] Detectie en vloerobstakels in één proces (één CUDA-context en één keer decoderen,
      spaart ±250 MB RAM als ze samen aan staan)
- [ ] Isaac ROS 4.6 op de Orin Nano testen (cuVSLAM en eventueel nvblox voor 3D-obstakels)

**Klaar wanneer:** de robot detecties zoals "tv" en "zetel" in realtime op de kaart
zet.

### Fase 5 – Dashboard (iPhone/iPad)
Een eerste versie kan al na fase 1. Deze fase kan dus naar voren schuiven.
- [ ] `rosbridge` + webapp op de Jetson
- [ ] Batterij, status, noodstop, joystick
- [ ] Kaart met de positie van de robot; tik op de kaart om naartoe te rijden
- [ ] Knoppen voor functies (kaart maken, naar een locatie rijden, ...)

**Klaar wanneer:** je de robot zonder pc kunt bedienen vanaf de iPhone.

### Fase 6 – Spraak en AI-assistent
- [x] Tussenstap: commando's van het Yahboom-bord koppelen aan acties (stop, go ahead, batterijstatus)
- [ ] USB-microfoon en luidspreker
- [ ] Wekwoord → spraakherkenning (Whisper) → taalmodel met "tools" (naar een locatie rijden,
  rondkijken, een foto beschrijven) → spraak (Piper TTS)
- [ ] Een beeldtaalmodel (VLM) beschrijft wat de camera ziet en zegt het luidop (voorkeur van
      Jonas boven een open-woordenschat-detector; niet realtime is oké)
- [ ] Spraak uit: USB-luidspreker (eerst zonder microfoon) + Piper TTS; later een Jabra
      (microfoon + luidspreker) voor spraakbediening
- [ ] Doelscenario: *"zoek de televisie en vertel me wat je erop ziet"*

**Inspiratie: [Pupper v3](https://pupper-v3-documentation.readthedocs.io/en/latest/)**
(open-source robothond van Stanford). Hoe zijn AI werkt staat op
[Using Pupper AI](https://pupper-v3-documentation.readthedocs.io/en/latest/development/ai.html):
een spraakmodel praat en kiest zelf "tools", die de robotsoftware uitvoert in plaats van ze
voor te lezen. Ideeën om over te nemen:
- [ ] Tools voor het taalmodel: `move` (snelheidscommando naar Nav2/twist_mux),
      `analyze_camera_image` (foto naar het VLM, antwoord terug als context), "rondkijken",
      naar een locatie met een naam rijden, volume van de speaker (bij Pupper ±10 regels code)
- [ ] Volgmodus: de dichtstbijzijnde persoon volgen (YOLO `person` + lidar-afstand)
- [ ] Expressie: een gezicht op een schermpje of in het dashboard, geluidjes; Pupper heeft
      een scherm en beweegbare oren
- [ ] Veiligheid rond kinderen ([Safety](https://pupper-v3-documentation.readthedocs.io/en/latest/using_pupper/safety.html))
- Keuze voor later: Pupper draait spraak, taalmodel en beeld in de cloud (OpenAI Realtime,
  Cartesia TTS, Google Gemini): snel en slim, maar internet en kosten. Wij mikken eerst op
  lokaal (Whisper, Piper, een klein VLM op de GPU), eventueel met de cloud als optie.

**Aandachtspunt:** de 8 GB geheugen wordt gedeeld door ROS, Nav2 en de AI-modellen.
We kiezen kleine, gequantiseerde modellen en meten het geheugengebruik. Blijkt het te
krap, dan kan het zwaarste model als terugvaloptie op de pc draaien. Die heeft een
NVIDIA-GPU.

### Systeeminstellingen op de Jetson (buiten de repo)
- **Geen bureaublad:** `systemctl set-default multi-user.target` (28-10-2026). Bespaart ~230 MB
  RAM. Tijdelijk terug met een scherm: `sudo systemctl start gdm`; blijvend:
  `sudo systemctl set-default graphical.target`.
- **GPU voor Docker via CDI:** `/etc/cdi/nvidia.yaml` (`scripts/install_gpu.sh`). Opnieuw
  maken na een JetPack-update; `L4T` in de Dockerfile dan ook aanpassen.
- **Batterijcellen UPS:** de UPS laadt de 3 cellen in serie zonder balancing. Op 28-10-2026
  stond na een afsluiting één cel op 2,66 V (andere 4,2 V); de UPS-beveiliging koppelde de
  batterij los (geen stroom zonder adapter, lader meet 13,3 V/0 A). Af en toe de cellen in een
  losse lader balanceren; zwakke cel vervangen als ze na een dag rust duidelijk lager staat.
  Let op: zonder (aangesloten) batterij meet de UPS de adapter en schommelt de waarde
  (12,4-13,6 V); boven 12,8 V toont het dashboard "geen batterij", daaronder is de meting
  misleidend. Laadstroom 0 A met adapter betekent dus niet altijd "vol".
- Afsluiten bij lege batterij: `power_guard` (container) maakt `~/wave_rover/run/shutdown_request`
  aan als de UPS 30 s onder 20% zit (waarschuwing vanaf 30%) en de aftelling van 60 s afloopt; `wave-rover-shutdown.path`
  ziet dat en voert `systemctl poweroff` uit. Installeren: `scripts/install_power_guard.sh`.
  Na een automatische afsluiting de UPS-schakelaar uitzetten (de UPS levert nog een kleine stroom).
- Docker start pas na kloksynchronisatie (`docker.service.d/10-wait-time-sync.conf`, max 90 s via
  `systemd-time-wait-sync.service.d/10-timeout.conf`): zonder RTC-batterij start de klok in 1970.
- **NVMe: nooit DISCARD sturen.** Op 23-10-2026 vastgelegd: de SPCC-SSD (MAXIO MAP1202) loopt
  vast op een DISCARD-opdracht (`req_op:DISCARD ... timeout, reset controller`), waarna alle
  schrijfopdrachten blokkeren. Oplossing: udev-regel `/etc/udev/rules.d/60-nvme-no-discard.rules`
  (`discard_max_bytes=0`) en `fstrim.timer` uitgeschakeld.
- Openstaand: `kernel.hung_task_panic=1` stond aan maar de Jetson herstartte niet vanzelf toen
  de schijf vastzat; uitzoeken waarom (panic of hardware-watchdog).
- NVMe-energiebesparing (APST) uit: `nvme_core.default_ps_max_latency_us=0` in
  `/boot/extlinux/extlinux.conf`. De Silicon Power-SSD (MAXIO MAP1202) viel anders
  willekeurig weg. Een backup staat in `extlinux.conf.bak`.
- Energiebesparing van de wifi uit: `/etc/NetworkManager/conf.d/99-wifi-powersave-off.conf`
- Twee wifiprofielen: `TP-Link_9374` (prioriteit 20) en `De Clercq` (prioriteit 10, beneden).
  Valt het ene weg, dan verbindt de Jetson met het andere. Beide via dezelfde router (192.168.1.x).
  Getest op 23-10-2026: zelfde IP (192.168.1.28) op beide netwerken.
- Wifi-regiocode België: `/etc/modprobe.d/cfg80211-regdom.conf` (`ieee80211_regdom=BE`).
- Het ESP32-bord van de Wave Rover maakt zelf een wifi-hotspot `UGV`.
- **Hotspot als er geen bekend wifinetwerk is** (28-10-2026, `scripts/install_wifi_fallback.sh`,
  dienst `wave-rover-wifi`): na 60 s zonder wifi maakt de Jetson het netwerk `WaveRover`
  (2,4 GHz, WPA2, Jetson = 10.42.0.1). De app werkt dan gewoon op http://jetson.local:8080.
  Zit er 5 min niemand op de hotspot, dan zoekt hij opnieuw een bekend netwerk. Wachtwoord
  opvragen: `sudo nmcli -s -g 802-11-wireless-security.psk connection show WaveRover-hotspot`.
- **Interface `ros0` (10.99.99.1)**, dienst `wave-rover-ros0` vóór Docker: ROS-nodes geven bij
  het starten hun adressen door; zonder vast adres vonden nodes die na het omschakelen naar
  de hotspot starten de andere niet meer. Thuis verandert er niets (pc ziet de topics).
- Hardware-watchdog (`RuntimeWatchdogSec=30s`), herstart na kernelpanic (`kernel.panic=10`,
  `kernel.hung_task_panic=1`), NVMe Host Memory Buffer uit (`nvme.max_host_mem_size_mb=0`).
- Diensten `dnsmasq`, `isc-dhcp-server` en `isc-dhcp-server6` uitgezet (29-10-2026): ze
  faalden bij elke start en worden niet gebruikt (de hotspot gebruikt dnsmasq van NetworkManager).
- `jonas` zit in de groepen `docker` en `dialout`, en heeft sudo zonder wachtwoord.
- Pakket `docker-compose-v2` is geïnstalleerd.

### Doorlopend
- Automatisch opstarten bij het booten (Docker-herstartbeleid)
- Na elke fase de documentatie bijwerken in `docs/`
