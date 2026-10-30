// Wave Rover-dashboard: status, modules, noodstop en rijden via rosbridge.
'use strict';

// Zelfde host als de pagina: werkt via jetson.local, een IP-adres en later de hotspot.
const ROSBRIDGE_URL = `ws://${window.location.hostname}:9090`;
const RECONNECT_DELAY_MS = 2000;

// Snelheidsstanden zoals op de RC-zender (rc_teleop in base.yaml): m/s en rad/s.
const MAX_LINEAR = [0.15, 0.3, 0.6];
const MAX_ANGULAR = [0.9, 1.8, 3.0];
const DRIVE_RATE_MS = 100;  // 10 Hz; twist_mux stopt na 0,5 s zonder commando

const $ = (id) => document.getElementById(id);

let ros = null;
let reconnectTimer = null;
let connected = false;
let cmdVel = null;
let estopService = null;

// --- verbinding ---------------------------------------------------------------------

function setConnection(state, text) {
  const badge = $('connection');
  badge.className = `badge ${state}`;
  badge.textContent = text;
}

function scheduleReconnect() {
  if (reconnectTimer === null) {
    reconnectTimer = setTimeout(() => {
      reconnectTimer = null;
      connect();
    }, RECONNECT_DELAY_MS);
  }
}

// Staat er een nieuwere versie van het dashboard op de Jetson (na een update)? Dan
// herladen: een webapp op het beginscherm blijft anders de oude scripts gebruiken.
function reloadIfUpdated() {
  const meta = document.querySelector('meta[name="dashboard-version"]');
  if (!meta) return;
  fetch('/version', { cache: 'no-store' })
    .then((r) => (r.ok ? r.text() : null))
    .then((version) => {
      if (version && version.trim() !== meta.content) window.location.reload();
    })
    .catch(() => {});
}

// Ook als de app open blijft: bij het terug openen en elke minuut kijken, maar niet herladen
// terwijl je met de joystick rijdt.
document.addEventListener('visibilitychange', () => {
  if (!document.hidden) reloadIfUpdated();
});
setInterval(() => {
  if (!document.hidden && driveTimer === null) reloadIfUpdated();
}, 60000);

function connect() {
  setConnection('connecting', 'verbinden…');
  ros = new ROSLIB.Ros({ url: ROSBRIDGE_URL });

  ros.on('connection', () => {
    connected = true;
    $('poweroff-banner').hidden = true;
    setConnection('online', 'verbonden');
    reloadIfUpdated();
    subscribeAll();
  });
  ros.on('error', () => setConnection('offline', 'niet verbonden'));
  ros.on('close', () => {
    connected = false;
    stopDriving();
    setConnection('offline', 'niet verbonden');
    scheduleReconnect();
  });
}

function topic(name, messageType, options = {}) {
  return new ROSLIB.Topic({ ros, name, messageType, ...options });
}

function subscribeAll() {
  // Rosbridge remt af (throttle_rate in ms) zodat de Jetson zo weinig mogelijk doorstuurt.
  topic('/ups/battery', 'sensor_msgs/msg/BatteryState', { throttle_rate: 2000 })
    .subscribe((m) => showBattery('ups', m));
  topic('/chassis/battery', 'sensor_msgs/msg/BatteryState', { throttle_rate: 2000 })
    .subscribe((m) => showBattery('chassis', m));
  topic('/rc/connected', 'std_msgs/msg/Bool', { throttle_rate: 1000 })
    .subscribe((m) => showRc(m.data));
  topic('/modules/status', 'std_msgs/msg/String')
    .subscribe((m) => showModules(JSON.parse(m.data)));
  topic('/e_stop', 'std_msgs/msg/Bool')
    .subscribe((m) => showEstop(m.data));
  topic('/charging_lock', 'std_msgs/msg/Bool')
    .subscribe((m) => showCharging(m.data));
  topic('/power/shutdown_in', 'std_msgs/msg/Int32')
    .subscribe((m) => showShutdown(m.data));

  cmdVel = topic('/cmd_vel_dashboard', 'geometry_msgs/msg/TwistStamped');
  estopService = new ROSLIB.Service({ ros, name: '/e_stop/set', serviceType: 'std_srvs/srv/SetBool' });
  if (cameraOn) setCamera(true);  // na herverbinding opnieuw abonneren
  if (openPanel === 'modules') watchSystem(true);  // na herverbinding opnieuw volgen
  MapView.start(ros);
  MapsCard.start(ros);
  TalkCard.start(ros);
}

// --- status -------------------------------------------------------------------------

const CHARGING = 1;  // sensor_msgs/BatteryState POWER_SUPPLY_STATUS_CHARGING

function showBattery(which, msg) {
  const value = $(`bat-${which}`);
  const sub = $(`bat-${which}-sub`);
  const pct = Math.round(msg.percentage * 100);
  if (!msg.present || Number.isNaN(pct)) {
    // UPS zonder batterij (of losgekoppeld door de beveiliging): hij meet de adapter.
    // battery_monitor houdt dat 30 s vast, zodat dit niet heen en weer springt.
    const adapter = which === 'ups' && msg.voltage > 5;
    value.textContent = adapter ? 'adapter' : '–';
    value.className = `stat-value ${adapter ? 'warn' : ''}`;
    sub.textContent = adapter ? 'geen batterij' : 'geen meting';
    return;
  }
  // Tijdens het laden leest het percentage te hoog: dan "laadt" tonen.
  if (msg.power_supply_status === CHARGING) {
    value.textContent = 'laadt';
    value.className = 'stat-value ok';
  } else {
    value.textContent = `${pct}%`;
    value.className = `stat-value ${pct < 20 ? 'bad' : pct < 40 ? 'warn' : 'ok'}`;
  }
  sub.textContent = `${msg.voltage.toFixed(2)} V`;
}

function showRc(isConnected) {
  const rc = $('rc');
  rc.textContent = isConnected ? 'aan' : 'uit';
  rc.className = `stat-value ${isConnected ? 'ok' : ''}`;
}

// --- afsluiten bij lege batterij ---------------------------------------------------

function showShutdown(seconds) {
  $('shutdown-banner').hidden = seconds < 0;
  $('shutdown-seconds').textContent = seconds;
}

// --- camera -------------------------------------------------------------------------

// Alleen beelden ophalen als iemand kijkt: spaart wifi en CPU op de Jetson.
// Beelden komen als doorlopende MJPEG-stroom (640x360) van camera_bridge, niet via rosbridge:
// die buffert onbeperkt, waardoor de vertraging tot seconden opliep. camera_bridge houdt de
// zendbuffer klein en slaat beelden over als de gsm niet volgt. De kaders van de module
// detection komen wel via rosbridge.
const CAMERA_STREAM = `http://${window.location.hostname}:8081/stream.mjpg?fps=15`;
let cameraOn = false;
let detectionsTopic = null;
let detectionsClear = null;
const CAMERA_SIZE = { w: 1280, h: 720 };  // kaders in /detections zijn in deze pixels

function showCameraImage() {
  if (!cameraOn) return;
  $('camera-image').hidden = false;
  $('camera-hint').hidden = true;
}
$('camera-image').addEventListener('load', showCameraImage);
$('camera-image').addEventListener('error', () => {
  // Camera start nog op (of kort weg): opnieuw proberen.
  if (cameraOn) setTimeout(() => { if (cameraOn) $('camera-image').src = `${CAMERA_STREAM}&t=${Date.now()}`; }, 1000);
});

function setCamera(on) {
  const img = $('camera-image');
  img.hidden = true;  // pas tonen bij het eerste nieuwe beeld, niet het oude van vorige keer
  cameraOn = false;  // eerst: het sluiten van de stroom mag geen herverbinding starten
  img.src = '';
  img.removeAttribute('src');
  $('camera-hint').textContent = 'Camera start…';
  $('camera-hint').hidden = false;
  cameraOn = on;
  if (detectionsTopic) {
    detectionsTopic.unsubscribe();
    detectionsTopic = null;
  }
  drawDetections([]);
  if (!on) return;
  img.src = `${CAMERA_STREAM}&t=${Date.now()}`;
  setTimeout(showCameraImage, 3000);  // niet elke browser meldt 'load' bij een stroom
  if (ros && connected) {
    // Kaders van de module detection (10 Hz); niets als die uit staat.
    detectionsTopic = topic('/detections', 'vision_msgs/msg/Detection2DArray',
      { throttle_rate: 100, queue_length: 1 });
    detectionsTopic.subscribe((m) => {
      drawDetections(m.detections);
      clearTimeout(detectionsClear);  // module gestopt: oude kaders weghalen
      detectionsClear = setTimeout(() => drawDetections([]), 1000);
    });
  }
}

function drawDetections(detections) {
  const canvas = $('camera-overlay');
  const ratio = window.devicePixelRatio || 1;
  canvas.width = canvas.clientWidth * ratio;
  canvas.height = canvas.clientHeight * ratio;
  const ctx = canvas.getContext('2d');
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  if ($('camera-image').hidden) return;
  const sx = canvas.width / CAMERA_SIZE.w;
  const sy = canvas.height / CAMERA_SIZE.h;
  ctx.lineWidth = 2 * ratio;
  ctx.font = `600 ${12 * ratio}px -apple-system, sans-serif`;
  ctx.textBaseline = 'bottom';
  for (const d of detections) {
    const hyp = d.results[0].hypothesis;
    const w = d.bbox.size_x * sx;
    const h = d.bbox.size_y * sy;
    const x = d.bbox.center.position.x * sx - w / 2;
    const y = d.bbox.center.position.y * sy - h / 2;
    const label = `${hyp.class_id} ${Math.round(hyp.score * 100)}%`;
    ctx.strokeStyle = '#3ccf7a';
    ctx.strokeRect(x, y, w, h);
    const tw = ctx.measureText(label).width + 8 * ratio;
    const th = 16 * ratio;
    const ty = y > th ? y : y + th;  // label binnen het beeld houden
    ctx.fillStyle = '#3ccf7a';
    ctx.fillRect(x - ratio, ty - th, tw, th);
    ctx.fillStyle = '#10161f';
    ctx.fillText(label, x + 4 * ratio, ty - 2 * ratio);
  }
}

// Beeld tonen zolang het camerapaneel open is. De module camera gaat mee aan (uitzetten doe
// je bij de modules); elke andere toestand dan aan/start (uit, fout of onbekend): aanzetten.
function startCamera() {
  const state = lastModules.camera && lastModules.camera.state;
  if (state !== 'aan' && state !== 'start') callEnable('camera', true);
  setCamera(true);
}

document.addEventListener('visibilitychange', () => {
  if (document.hidden && cameraOn) setCamera(false);
  else if (!document.hidden) updateCamera();
});

// Eén camerabeeld (één videostroom): in het camerapaneel, of onder de joystick zolang het
// rijpaneel open is (knop "Camera" daar; de keuze wordt onthouden). Anders uit.
let driveCamera = true;
try { driveCamera = localStorage.getItem('driveCamera') !== 'uit'; } catch (e) { /* privé */ }

function updateCamera() {
  const inDrive = openPanel === 'drive' && driveCamera;
  const wanted = openPanel === 'camera' || inDrive;
  $(inDrive ? 'drive-camera-slot' : 'camera-slot').append($('camera-wrap'));
  $('drive-camera').classList.toggle('active', driveCamera);
  if (wanted && !cameraOn) startCamera();
  if (!wanted && cameraOn) setCamera(false);
}

$('drive-camera').addEventListener('click', () => {
  driveCamera = !driveCamera;
  try { localStorage.setItem('driveCamera', driveCamera ? 'aan' : 'uit'); } catch (e) { /* privé */ }
  updateCamera();
});

// --- panelen ------------------------------------------------------------------------

// De kaart vult het scherm; elke knop onderaan schuift een paneel over de kaart. Nog eens
// tikken, het kruisje of omlaag vegen sluit het. Rijden en camerabeeld lopen alleen zolang
// hun paneel open is.
let openPanel = null;

function showPanel(name) {
  const previous = openPanel;
  openPanel = name === openPanel ? null : name;
  $('sheet').classList.toggle('open', openPanel !== null);
  document.querySelectorAll('.tabbar [data-tab]').forEach((b) => {
    b.classList.toggle('active', b.dataset.tab === openPanel);
  });
  if (openPanel) {
    // Bij sluiten blijft de inhoud staan tot het paneel weggeschoven is.
    document.querySelectorAll('.panel').forEach((p) => { p.hidden = p.dataset.panel !== openPanel; });
    $('sheet-title').textContent = document.querySelector(`.panel[data-panel="${openPanel}"]`).dataset.title;
  }
  if (previous === 'drive' && openPanel !== 'drive') stopDriving();
  if ((previous === 'modules') !== (openPanel === 'modules')) watchSystem(openPanel === 'modules');
  updateCamera();
}

document.querySelectorAll('.tabbar [data-tab]').forEach((b) => {
  b.addEventListener('click', () => showPanel(b.dataset.tab));
});
$('sheet-close').addEventListener('click', () => showPanel(null));

// Omlaag vegen aan de kop van het paneel sluit het.
let sheetSwipe = null;
$('sheet-head').addEventListener('pointerdown', (event) => { sheetSwipe = event.clientY; });
$('sheet-head').addEventListener('pointerup', (event) => {
  if (sheetSwipe !== null && event.clientY - sheetSwipe > 40) showPanel(null);
  sheetSwipe = null;
});

// --- noodstop -----------------------------------------------------------------------

let estopActive = false;
let chargingLock = false;

function showEstop(active) {
  estopActive = active;
  $('estop-state').hidden = !active;
  $('joystick').classList.toggle('disabled', estopActive || chargingLock);
}

// Laadkabel in (battery_monitor): twist_mux blokkeert dan alle rijcommando's.
function showCharging(locked) {
  chargingLock = locked;
  $('charging-state').hidden = !locked;
  $('joystick').classList.toggle('disabled', estopActive || chargingLock);
}

function setEstop(active) {
  if (!estopService) return;
  if (active) stopDriving();
  estopService.callService({ data: active }, () => {}, (err) => alertError(err));
}

// pointerdown i.p.v. click: iOS stuurt geen click als er al een vinger op de joystick ligt.
$('estop').addEventListener('pointerdown', (event) => {
  event.preventDefault();
  setEstop(true);
});
$('estop-release').addEventListener('click', () => setEstop(false));

// --- modules ------------------------------------------------------------------------

const STATE_LABELS = { uit: 'uit', start: 'start…', aan: 'aan', stopt: 'stopt…', fout: 'fout' };
const STATE_CLASS = { aan: 'ok', start: 'warn', stopt: 'warn', fout: 'bad' };

let lastModules = {};

function showModules(modules) {
  lastModules = modules;
  MapView.onModules(modules);
  const list = $('modules');
  list.replaceChildren();
  for (const [name, info] of Object.entries(modules)) {
    const li = document.createElement('li');
    const on = info.state === 'aan' || info.state === 'start';
    const busy = info.state === 'start' || info.state === 'stopt';

    const text = document.createElement('div');
    text.className = 'module-info';
    const title = document.createElement('div');
    title.className = 'module-name';
    title.textContent = name;
    const desc = document.createElement('div');
    desc.className = 'module-desc';
    desc.textContent = info.description;
    const use = document.createElement('div');
    use.className = 'module-use';
    use.dataset.use = name;
    use.textContent = moduleUse(name);
    text.append(title, desc, use);

    const state = document.createElement('span');
    state.className = `module-state ${STATE_CLASS[info.state] || ''}`;
    state.textContent = STATE_LABELS[info.state] || info.state;

    const toggle = document.createElement('button');
    toggle.type = 'button';
    toggle.className = `switch${on ? ' on' : ''}${busy ? ' busy' : ''}`;
    toggle.setAttribute('aria-label', `${name} ${on ? 'uitzetten' : 'aanzetten'}`);
    toggle.disabled = busy;
    toggle.addEventListener('click', () => enableModule(name, !on, toggle));

    li.append(text, state, toggle);
    list.append(li);
  }
}

// --- toestand van de Jetson ----------------------------------------------------------

// Alleen zolang het paneel Modules open is (module_manager meet niets als niemand kijkt).
let systemTopic = null;
let lastSystem = null;
const gb = (mb) => `${(mb / 1000).toFixed(1).replace('.', ',')} GB`;

function moduleUse(name) {
  const m = lastSystem && lastSystem.modules[name];
  if (!m) return '';
  return `${m.cpu === null ? '–' : String(m.cpu).replace('.', ',')}% CPU · ${gb(m.mb)}`;
}

function setBar(id, pct, warnAt, badAt) {
  const bar = $(`${id}-bar`);
  bar.style.width = `${Math.min(100, pct || 0)}%`;
  bar.className = pct >= badAt ? 'bad' : pct >= warnAt ? 'warn' : '';
}

function showSystem(data) {
  lastSystem = data;
  const ram = Math.round((100 * data.ram_used) / data.ram_total);
  $('sys-cpu').textContent = data.cpu === null ? '–' : `${data.cpu}%`;
  $('sys-ram').textContent = `${gb(data.ram_used)} / ${gb(data.ram_total)}`;
  $('sys-gpu').textContent = `${data.gpu}%`;
  setBar('sys-cpu', data.cpu, 70, 90);
  setBar('sys-ram', ram, 85, 95);
  setBar('sys-gpu', data.gpu, 70, 90);
  const meta = [];
  if (data.temp !== null) meta.push(`${data.temp} °C`);
  if (data.power !== null) meta.push(`${String(data.power).replace('.', ',')} W`);
  if (data.fan !== null) meta.push(`ventilator ${data.fan} tpm`);
  meta.push(`basis ${gb(data.basis.mb)}`, `overig ${gb(data.overig_mb)}`);
  $('sys-meta').textContent = meta.join(' · ');
  document.querySelectorAll('[data-use]').forEach((el) => { el.textContent = moduleUse(el.dataset.use); });
}

function watchSystem(on) {
  if (systemTopic) {
    systemTopic.unsubscribe();
    systemTopic = null;
  }
  if (on && ros && connected) {
    systemTopic = topic('/system/status', 'std_msgs/msg/String');
    systemTopic.subscribe((m) => showSystem(JSON.parse(m.data)));
  }
}

function enableModule(name, enable, toggle) {
  toggle.classList.add('busy');
  toggle.disabled = true;
  callEnable(name, enable);
}

function callEnable(name, enable) {
  const service = new ROSLIB.Service({
    ros, name: `/modules/${name}/enable`, serviceType: 'std_srvs/srv/SetBool',
  });
  // Starten kan even duren (bv. navigatie); de status komt via /modules/status.
  service.callService({ data: enable }, (res) => {
    if (!res.success) alertError(res.message);
  }, (err) => alertError(err));
}

function alertError(message) {
  setConnection('offline', `fout: ${message}`);
  setTimeout(() => setConnection(connected ? 'online' : 'offline',
    connected ? 'verbonden' : 'niet verbonden'), 4000);
}

// --- robot afsluiten ----------------------------------------------------------------

// power_guard stopt alle modules, zegt via de speaker welke schakelaars uit moeten en zet
// daarna de Jetson uit. Beter voor de SSD dan meteen de UPS-schakelaar uitzetten.
$('robot-off').addEventListener('click', () => {
  if (!ros || !connected) return;
  if (!window.confirm('Robot afsluiten? Alle modules gaan uit en de Jetson sluit af.')) return;
  stopDriving();
  new ROSLIB.Service({ ros, name: '/power/shutdown', serviceType: 'std_srvs/srv/Trigger' })
    .callService({}, (res) => {
      if (!res.success) {
        alertError(res.message);
        return;
      }
      $('poweroff-banner').hidden = false;
      showPanel(null);
    }, (err) => alertError(err));
});

// --- rijden -------------------------------------------------------------------------

let speedLevel = 0;
let stick = { x: 0, y: 0 };
let driveTimer = null;

document.querySelectorAll('.speed button').forEach((btn) => {
  btn.addEventListener('click', () => {
    speedLevel = Number(btn.dataset.speed);
    document.querySelectorAll('.speed button').forEach((b) => b.classList.toggle('active', b === btn));
  });
});

function publishDrive() {
  if (!cmdVel || !connected) return;
  const now = Date.now();
  cmdVel.publish({
    header: {
      stamp: { sec: Math.floor(now / 1000), nanosec: (now % 1000) * 1e6 },
      frame_id: 'base_link',
    },
    twist: {
      linear: { x: stick.y * MAX_LINEAR[speedLevel], y: 0, z: 0 },
      angular: { x: 0, y: 0, z: -stick.x * MAX_ANGULAR[speedLevel] },
    },
  });
}

function stopDriving() {
  if (driveTimer !== null) {
    clearInterval(driveTimer);
    driveTimer = null;
    stick = { x: 0, y: 0 };
    publishDrive();  // expliciet stoppen
  }
  $('knob').style.transform = '';
}

const joystick = $('joystick');

function updateStick(event) {
  const rect = joystick.getBoundingClientRect();
  const radius = rect.width / 2;
  let x = (event.clientX - rect.left - radius) / radius;
  let y = (rect.top + radius - event.clientY) / radius;
  const length = Math.hypot(x, y);
  if (length > 1) { x /= length; y /= length; }
  stick = { x, y };
  $('knob').style.transform = `translate(${x * radius * 0.66}px, ${-y * radius * 0.66}px)`;
}

joystick.addEventListener('pointerdown', (event) => {
  if (joystick.classList.contains('disabled')) return;
  joystick.setPointerCapture(event.pointerId);
  updateStick(event);
  publishDrive();
  driveTimer = setInterval(publishDrive, DRIVE_RATE_MS);
});
joystick.addEventListener('pointermove', (event) => {
  if (driveTimer !== null) updateStick(event);
});
['pointerup', 'pointercancel', 'lostpointercapture'].forEach((type) => {
  joystick.addEventListener(type, stopDriving);
});
// App naar de achtergrond of scherm uit: meteen stoppen (dodemansknop).
document.addEventListener('visibilitychange', () => {
  if (document.hidden) stopDriving();
});

connect();
