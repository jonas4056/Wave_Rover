# Wave Rover – Plan dashboard (fase 5)

Doel: de robot demonstreren en bedienen vanaf een iPhone/iPad, **zonder pc**.
Alles wat nu al werkt (rijden, modules starten, kaart maken, navigeren) moet je vanuit
Safari kunnen bedienen.

Werkwijze zoals in `ROADMAP.md`: stap per stap, een stap is pas klaar als alle punten
onder **"Klaar wanneer"** werken.

---

## Uitgangspunten

- **Draait op de Jetson, in de bestaande container.** Geen extra container, geen
  aparte server.
- **Hoort bij de basis, niet bij een module.** De basis start altijd bij het booten.
  Het dashboard moet bereikbaar zijn vóór er modules draaien, want je start de modules
  net vanuit het dashboard.
- **Licht.** `rosbridge` (WebSocket, poort 9090) plus een eenvoudige statische
  webserver (poort 8080). Geen Node.js-build, geen framework: HTML, CSS en JavaScript
  met `roslib.js`.
- **`roslib.js` lokaal in de repo**, niet van een CDN. Dan werkt het dashboard ook
  als de robot of de iPhone geen internet heeft.
- **Bestaande interfaces hergebruiken.** Het dashboard praat alleen met topics en
  services die al bestaan. Waar iets ontbreekt, maken we een kleine ROS-node in
  plaats van logica in de browser.
- **Veiligheid gaat voor.** De RC-zender blijft altijd voorrang hebben (`twist_mux`).
  Het dashboard rijdt via `cmd_vel_dashboard` (prioriteit 50).

## Bestaande interfaces die het dashboard gebruikt

| Wat | Interface | Type |
|---|---|---|
| Modules starten/stoppen | `/modules/<naam>/enable` | service `std_srvs/SetBool` |
| Status van de modules | `/modules/status` | topic `std_msgs/String` (JSON, transient local) |
| Batterij UPS (Jetson) | `/ups/battery` | `sensor_msgs/BatteryState` |
| Batterij chassis | `/chassis/battery` | `sensor_msgs/BatteryState` |
| RC-zender verbonden | `/rc/connected` | `std_msgs/Bool` |
| Noodstop | `/e_stop` | `std_msgs/Bool` (lock in `twist_mux`, prioriteit 255) |
| Rijden vanaf het dashboard | `/cmd_vel_dashboard` | `geometry_msgs/TwistStamped`, timeout 0,5 s |
| Kaart | `/map` | `nav_msgs/OccupancyGrid` |
| Naar een punt rijden | `/goal_pose` | `geometry_msgs/PoseStamped` (bt_navigator luistert hierop) |
| Navigatie annuleren | `/navigate_to_pose/_action/cancel_goal` | `action_msgs/CancelGoal` (zoals in `yahboom_voice.py`) |
| Startpositie zetten | `/initialpose_rviz` | `PoseWithCovarianceStamped` (via `initialpose_relay`) |

Nog te maken:
- **`/robot_pose`** (`geometry_msgs/PoseStamped`, 5 Hz): positie van `base_footprint`
  in `map`. Een kleine node in `wave_rover_base`, zodat de browser geen TF hoeft te
  verwerken.
- **Kaart opslaan met een naam**: een service die slam_toolbox laat opslaan in
  `~/wave_rover/maps/<naam>` (zelfde manier als nu handmatig).

---

## Stap A – Infrastructuur

- [ ] `ros-jazzy-rosbridge-server` toevoegen aan de Dockerfile
- [ ] Nieuw package `wave_rover_dashboard` met de map `web/` (index.html, app.js,
      style.css, `roslib.min.js`)
- [ ] `dashboard.launch.py`: `rosbridge_websocket` (9090) + statische webserver (8080)
      op de map `web/`
- [ ] Opnemen in `base.launch.py`, zodat het bij het booten mee start
- [ ] Pagina toont een indicator "verbonden / niet verbonden" met rosbridge, en
      verbindt automatisch opnieuw
- [ ] Bereikbaar via `http://<jetson>:8080`; iOS-icoon en
      `apple-mobile-web-app-capable` voor "Zet op beginscherm"

**Klaar wanneer:** de pagina op de iPhone opent, "verbonden" toont, en na een herstart
van de container vanzelf opnieuw verbindt.

## Stap B – Status en bediening (eerste demoversie)

- [ ] Batterij UPS en chassis (percentage en spanning, kleur bij laag)
- [ ] RC-zender verbonden ja/nee
- [ ] Lijst van modules met status (uit, start, aan, stopt, fout) en een schakelaar per
      module. De manager regelt `requires` en `conflicts` al zelf.
- [ ] Grote **noodstop**-knop: `e_stop = true` (latched); aparte knop "vrijgeven"
- [ ] Virtuele joystick → `cmd_vel_dashboard` op ~10 Hz
  - alleen publiceren zolang de vinger op de joystick ligt (dodemansknop)
  - stoppen bij loslaten, bij `visibilitychange` (app naar achtergrond) en bij
    verlies van verbinding
  - snelheidskeuze in drie standen, zoals op de RC-zender (`max_linear`, `max_angular`)

**Klaar wanneer:** je met enkel de iPhone de robot kunt laten rijden, een module kunt
starten en stoppen, en de noodstop de robot stilzet, ook als Nav2 rijdt.

## Stap C – Kaart en navigatie

- [ ] Node `/robot_pose` (zie hierboven)
- [ ] `/map` tekenen op een canvas, met de robot als pijltje erop
- [ ] Zoomen en verschuiven met twee vingers
- [ ] Tik op de kaart → richting kiezen door te slepen → `/goal_pose`
- [ ] Knop "navigatie annuleren"
- [ ] Startpositie zetten (zoals "2D Pose Estimate" in RViz) via `/initialpose_rviz`

**Klaar wanneer:** je zonder RViz de navigatiemodule start, de robot op de kaart ziet,
en hem door te tikken naar een plek in de living laat rijden.

## Stap D – Werkstromen

- [ ] **Kaart maken:** knop start de SLAM-module → rondrijden met joystick of RC →
      naam invullen → kaart opslaan
- [ ] **Kaart kiezen** voor de navigatie (nu vast `living_v3` in `modules.yaml`):
      de navigatiemodule moet een kaartnaam als argument kunnen krijgen
- [ ] **Locaties met een naam** ("zetel", "tv") opslaan en er met één knop naartoe
      rijden. Samen te ontwerpen met de roadmap-taak "locaties met een naam" en later de
      spraakcommando's.

**Klaar wanneer:** iemand die de robot niet kent, met de iPhone een nieuwe ruimte in
kaart brengt en de robot daarna naar een opgeslagen locatie stuurt.

---

## Openstaande vragen

- **Netwerk voor demo's buiten huis:** zonder thuiswifi is er geen verbinding. Optie:
  de Jetson als wifi-hotspot (NetworkManager), of de hotspot van de iPhone.
- **Beveiliging:** voorlopig alleen op het thuisnetwerk, zonder wachtwoord. Niet
  openzetten naar het internet.
- **Camerabeeld** (fase 4): later via `web_video_server` als MJPEG-stream toevoegen.
- **Dataverbruik van `/map`:** de kaart wordt alleen opnieuw verzonden als ze
  verandert. Tijdens SLAM eventueel beperken (`throttle_rate` in rosbridge).
