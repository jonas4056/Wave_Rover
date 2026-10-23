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

function connect() {
  setConnection('connecting', 'verbinden…');
  ros = new ROSLIB.Ros({ url: ROSBRIDGE_URL });

  ros.on('connection', () => {
    connected = true;
    setConnection('online', 'verbonden');
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
  topic('/power/shutdown_in', 'std_msgs/msg/Int32')
    .subscribe((m) => showShutdown(m.data));

  cmdVel = topic('/cmd_vel_dashboard', 'geometry_msgs/msg/TwistStamped');
  estopService = new ROSLIB.Service({ ros, name: '/e_stop/set', serviceType: 'std_srvs/srv/SetBool' });
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
    value.textContent = '–';
    value.className = 'stat-value';
    sub.textContent = 'geen meting';
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

// --- noodstop -----------------------------------------------------------------------

function showEstop(active) {
  $('estop-state').hidden = !active;
  $('joystick').classList.toggle('disabled', active);
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

function showModules(modules) {
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
