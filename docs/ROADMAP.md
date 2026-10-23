# Wave Rover – Stappenplan

Doel op lange termijn: een robot die je met spraak iets kan vragen, zoals *"zoek de
televisie in deze ruimte en vertel me wat je erop ziet"*. Hij moet dan zelf door het huis
navigeren, kijken, begrijpen en antwoorden. Dat werk gebeurt op de Jetson; daarvoor
hebben we hem.

Werkwijze: we bouwen fase per fase. Een fase is pas klaar als alle punten onder
**"Klaar wanneer"** werken. Pas dan beginnen we aan de volgende.

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
- [x] Nav2 instellen voor een robot zonder encoders (RPP, op de plaats draaien, lokalisatie met slam_toolbox)
- [x] Navigeren naar een punt dat je in RViz aanklikt
- [ ] Autonoom verkennen (`explore_lite`): de robot brengt zelf de ruimte in kaart
- [ ] Locaties met een naam opslaan ("zetel", "tv", "deur")

**Klaar wanneer:** de robot zelf de living in kaart brengt en daarna naar "tv" kan
rijden.

### Openstaand (stand 21-10-2026)
- [ ] Lidar steviger bevestigen (M2,5-standoffs), daarna kaart `living_v3` maken met de
      gecorrigeerde odometrie en de navigatie daarop testen
- [x] Lidarmotor altijd stoppen bij het uitzetten van de module, een crash of het herstarten
      van de container (`on_stop` in de module-manager + `lidar_stop`)
- [x] Kaart `living_v3` (stevige lidar + gecorrigeerde odometrie); navigatie werkt erop
- [x] Kalibratie draaisnelheid (23-10-2026, scripts/calibrate_motion.py): op de plaats
      86-104%, bochten 80-90%, rechtdoor ~100% (spin_gain 1.6, curve_gain 0.9)
- [ ] Aandachtspunt: de USB-chip (CP2102N) van de lidar liep één keer vast na snel na elkaar
      herstarten van de container ('cp210x_open - Unable to enable UART'); alleen USB uit- en
      insteken hielp. Komt het terug: automatische USB-reset of schakelbare USB-poort
- [ ] Spraakmodule testen: "display power", "robot stop", "go ahead"
- [x] Laatst bekende positie onthouden (`pose_saver` -> `<kaart>.lastpose`); de navigatie start daar
- [ ] Globaal lokaliseren ("zoek jezelf") als de robot verplaatst werd: knop in het dashboard
      (bv. tijdelijk AMCL met global localization + rondje draaien)
- [ ] Snel een nieuwe ruimte in kaart brengen en daarna navigeren (workflow + dashboardknop)
- [ ] Zelf verkennen (exploration) zodat de robot zelf een kaart maakt
- [ ] Locaties met een naam, gekoppeld aan "go to the point A..D"

### Fase 4 – Camera en beeldherkenning op de GPU
- [ ] CSI-camera in de container (Argus op JetPack 7 testen)
- [ ] Objectdetectie met TensorRT (bv. YOLO) → objecten worden op de kaart aangeduid
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
- [ ] Een beeldtaalmodel (VLM) beantwoordt vragen over wat de camera ziet
- [ ] Doelscenario: *"zoek de televisie en vertel me wat je erop ziet"*

**Aandachtspunt:** de 8 GB geheugen wordt gedeeld door ROS, Nav2 en de AI-modellen.
We kiezen kleine, gequantiseerde modellen en meten het geheugengebruik. Blijkt het te
krap, dan kan het zwaarste model als terugvaloptie op de pc draaien. Die heeft een
NVIDIA-GPU.

### Systeeminstellingen op de Jetson (buiten de repo)
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
- Hardware-watchdog (`RuntimeWatchdogSec=30s`), herstart na kernelpanic (`kernel.panic=10`,
  `kernel.hung_task_panic=1`), NVMe Host Memory Buffer uit (`nvme.max_host_mem_size_mb=0`).
- `jonas` zit in de groepen `docker` en `dialout`, en heeft sudo zonder wachtwoord.
- Pakket `docker-compose-v2` is geïnstalleerd.

### Doorlopend
- Automatisch opstarten bij het booten (Docker-herstartbeleid)
- Na elke fase de documentatie bijwerken in `docs/`
