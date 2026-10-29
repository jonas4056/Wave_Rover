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

function connect() {
  setConnection('connecting', 'verbinden…');
  ros = new ROSLIB.Ros({ url: ROSBRIDGE_URL });

  ros.on('connection', () => {
    connected = true;
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
  MapView.start(ros);
  MapsCard.start(ros);
}

// --- status -------------------------------------------------------------------------

const CHARGING = 1;  // sensor_msgs/BatteryState POWER_SUPPLY_STATUS_CHARGING

function showBattery(which, msg) {
  const value = $(`bat-${which}`);
  const sub = $(`bat-${which}-sub`);
  const pct = Math.round(msg.percentage * 100);
  if (!msg.present || Number.isNaN(pct)) {
    // UPS: spanning boven een vol pakket = adapter gemeten, batterij afwezig of losgekoppeld.
    const disconnected = which === 'ups' && msg.voltage > 12.8;
    value.textContent = disconnected ? 'geen batterij' : '–';
    value.className = `stat-value ${disconnected ? 'warn' : ''}`;
    sub.textContent = disconnected ? 'adapter / beveiliging' : 'geen meting';
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
  if (seconds >= 0) window.scrollTo({ top: 0, behavior: 'smooth' });
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
const CAMERA_HINT = 'Tik op "Toon beeld"; de module camera gaat dan vanzelf aan.';
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
  const btn = $('camera-toggle');
  btn.classList.toggle('active', on);
  btn.textContent = on ? 'Verberg beeld' : 'Toon beeld';
  const img = $('camera-image');
  img.hidden = true;  // pas tonen bij het eerste nieuwe beeld, niet het oude van vorige keer
  cameraOn = false;  // eerst: het sluiten van de stroom mag geen herverbinding starten
  img.src = '';
  img.removeAttribute('src');
  $('camera-hint').textContent = on ? 'Camera start…' : CAMERA_HINT;
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

$('camera-toggle').addEventListener('click', () => {
  const on = !cameraOn;
  // De module camera gaat mee aan (uitzetten doe je bij de modules).
  // Elke andere toestand dan aan/start (uit, fout of nog onbekend): aanzetten.
  const state = lastModules.camera && lastModules.camera.state;
  if (on && state !== 'aan' && state !== 'start') {
    callEnable('camera', true);
  }
  setCamera(on);
});
document.addEventListener('visibilitychange', () => {
  if (document.hidden && cameraOn) setCamera(false);
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
    text.append(title, desc);

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
