// Kaart in het dashboard: /map, /robot_pose en /plan tekenen; doel en startpositie zetten.
// Wordt gestart vanuit app.js met MapView.start(ros) na elke (her)verbinding.
'use strict';

const MapView = (() => {
  const canvas = document.getElementById('map');
  const ctx = canvas.getContext('2d');
  const statusEl = document.getElementById('nav-status');
  const hintEl = document.getElementById('map-hint');

  // Kleuren (donker thema): onbekend, vrij, bezet.
  const COLOR_UNKNOWN = [43, 55, 73];
  const COLOR_FREE = [201, 211, 223];
  const COLOR_OCCUPIED = [11, 15, 21];

  // Goal-statussen van action_msgs/GoalStatus.
  const NAV_STATUS = { 1: 'onderweg', 2: 'onderweg', 3: 'annuleren…', 4: 'aangekomen', 5: 'geannuleerd', 6: 'opgegeven' };

  let ros = null;
  let mapInfo = null;      // resolution, width, height, origin
  let mapKey = '';         // afmetingen + oorsprong van de getoonde kaart
  let mapImage = null;     // offscreen canvas met de kaart (1 pixel per cel)
  let robot = null;        // {x, y, yaw}
  let plan = [];           // [{x, y}]
  // Obstakels rond de robot (pose_saver): {points: [x1, y1, ...] in cm, time}
  const obstacles = { lidar: { points: [], time: 0 }, floor: { points: [], time: 0 } };
  const OBSTACLE_TIMEOUT = 2000;  // ms: geen nieuws (module uit) = niet meer tonen
  // pixels per meter, wereldpunt in het midden, draaiing van de kaart op het scherm (radialen)
  let view = { scale: 40, cx: 0, cy: 0, rot: 0 };
  let mode = 'pan';
  let dirty = true;
  let fitted = false;

  // Publishers/services, opnieuw aangemaakt bij elke verbinding.
  let goalTopic = null;
  let initialTopic = null;
  let cancelService = null;
  let findTopic = null;
  let statusTimer = null;

  // --- coördinaten ------------------------------------------------------------------

  function size() {
    return { w: canvas.clientWidth, h: canvas.clientHeight };
  }

  function worldToScreen(x, y) {
    const { w, h } = size();
    const c = Math.cos(view.rot);
    const s = Math.sin(view.rot);
    const dx = x - view.cx;
    const dy = y - view.cy;
    return { x: w / 2 + (dx * c - dy * s) * view.scale, y: h / 2 - (dx * s + dy * c) * view.scale };
  }

  function screenToWorld(sx, sy) {
    const { w, h } = size();
    const c = Math.cos(view.rot);
    const s = Math.sin(view.rot);
    const rx = (sx - w / 2) / view.scale;
    const ry = -(sy - h / 2) / view.scale;
    return { x: view.cx + rx * c + ry * s, y: view.cy - rx * s + ry * c };
  }

  function yawFromQuaternion(q) {
    return Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
  }

  function quaternionFromYaw(yaw) {
    return { x: 0, y: 0, z: Math.sin(yaw / 2), w: Math.cos(yaw / 2) };
  }

  // --- data -------------------------------------------------------------------------

  function onMap(msg) {
    const { width, height, resolution, origin } = msg.info;
    // Andere kaart tijdens de navigatie (map_manager vond de robot in een andere vaste kaart):
    // opnieuw passend maken. Bij slam/explore groeit de kaart gewoon, dan niet.
    const key = `${width}x${height}@${origin.position.x.toFixed(2)},${origin.position.y.toFixed(2)}`;
    if (mapKey && key !== mapKey && lastMapModuleState.navigation === 'aan') fitted = false;
    mapKey = key;
    mapInfo = { width, height, resolution, ox: origin.position.x, oy: origin.position.y };
    const off = document.createElement('canvas');
    off.width = width;
    off.height = height;
    const img = off.getContext('2d').createImageData(width, height);
    for (let row = 0; row < height; row++) {
      for (let col = 0; col < width; col++) {
        const v = msg.data[row * width + col];
        const c = v < 0 ? COLOR_UNKNOWN : v >= 65 ? COLOR_OCCUPIED : COLOR_FREE;
        // Rij 0 van de kaart ligt onderaan (kleinste y): verticaal spiegelen.
        const i = ((height - 1 - row) * width + col) * 4;
        img.data[i] = c[0];
        img.data[i + 1] = c[1];
        img.data[i + 2] = c[2];
        img.data[i + 3] = 255;
      }
    }
    off.getContext('2d').putImageData(img, 0, 0);
    mapImage = off;
    if (!fitted) fit();
    if (statusEl.textContent === 'geen kaart') statusEl.textContent = 'klaar';
    dirty = true;
  }

  function onPose(msg) {
    robot = { x: msg.pose.position.x, y: msg.pose.position.y, yaw: yawFromQuaternion(msg.pose.orientation) };
    dirty = true;
  }

  function onPlan(msg) {
    plan = msg.poses.map((p) => ({ x: p.pose.position.x, y: p.pose.position.y }));
    dirty = true;
  }

  function onObstacles(kind, msg) {
    obstacles[kind] = { points: msg.data, time: performance.now() };
    dirty = true;
  }

  // Tijdelijke melding in de status rechtsboven (bv. resultaat van "Zoek mezelf").
  function showStatus(text, ms) {
    statusEl.textContent = text;
    clearTimeout(statusTimer);
    if (ms) statusTimer = setTimeout(() => { statusEl.textContent = ''; }, ms);
  }

  function onFindStatus(msg) {
    const s = JSON.parse(msg.data);
    document.getElementById('nav-find').disabled = s.busy;
    if (s.message) showStatus(s.message, s.busy ? 0 : 8000);
  }

  function onNavStatus(msg) {
    if (!msg.status_list.length) return;
    const last = msg.status_list[msg.status_list.length - 1];
    statusEl.textContent = NAV_STATUS[last.status] || '';
    if (last.status >= 4) {
      plan = [];
      dirty = true;
    }
  }

  // --- tekenen ----------------------------------------------------------------------

  function fit() {
    if (!mapInfo) return;
    const { w, h } = size();
    const mw = mapInfo.width * mapInfo.resolution;
    const mh = mapInfo.height * mapInfo.resolution;
    // Afmetingen van de gedraaide kaart op het scherm.
    const c = Math.abs(Math.cos(view.rot));
    const s = Math.abs(Math.sin(view.rot));
    view.scale = Math.min(w / (mw * c + mh * s), h / (mw * s + mh * c)) * 0.95;
    view.cx = mapInfo.ox + mw / 2;
    view.cy = mapInfo.oy + mh / 2;
    fitted = true;
    dirty = true;
  }

  function drawArrow(x, y, yaw, color, length) {
    const p = worldToScreen(x, y);
    ctx.save();
    ctx.translate(p.x, p.y);
    ctx.rotate(-(yaw + view.rot));
    ctx.fillStyle = color;
    ctx.beginPath();
    ctx.moveTo(length, 0);
    ctx.lineTo(-length * 0.6, length * 0.6);
    ctx.lineTo(-length * 0.3, 0);
    ctx.lineTo(-length * 0.6, -length * 0.6);
    ctx.closePath();
    ctx.fill();
    ctx.restore();
  }

  function draw() {
    const dpr = window.devicePixelRatio || 1;
    const { w, h } = size();
    if (canvas.width !== Math.round(w * dpr) || canvas.height !== Math.round(h * dpr)) {
      canvas.width = Math.round(w * dpr);
      canvas.height = Math.round(h * dpr);
    }
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.fillStyle = '#121821';
    ctx.fillRect(0, 0, w, h);

    if (!mapImage) {
      ctx.fillStyle = '#8a97a8';
      ctx.font = '600 15px -apple-system, sans-serif';
      ctx.textAlign = 'center';
      ctx.fillText(mapModule.starting ? `${mapModule.name} start…` : 'Geen kaart', w / 2, h / 2 - 10);
      ctx.font = '13px -apple-system, sans-serif';
      ctx.fillText(mapModule.starting ? 'Even geduld, dat duurt tot een halve minuut'
        : 'Zet navigation aan bij Modules', w / 2, h / 2 + 12);
      ctx.textAlign = 'start';
    } else {
      // Zelfde omrekening als worldToScreen, als canvastransformatie (met draaiing).
      ctx.save();
      ctx.translate(w / 2, h / 2);
      ctx.rotate(-view.rot);
      ctx.scale(view.scale, -view.scale);
      ctx.translate(-view.cx, -view.cy);
      // Bovenste rij van de afbeelding = grootste y.
      ctx.translate(mapInfo.ox, mapInfo.oy + mapInfo.height * mapInfo.resolution);
      ctx.scale(mapInfo.resolution, -mapInfo.resolution);
      ctx.imageSmoothingEnabled = false;
      ctx.drawImage(mapImage, 0, 0);
      ctx.restore();
    }

    if (plan.length > 1) {
      ctx.strokeStyle = '#3ccf7a';
      ctx.lineWidth = 3;
      ctx.lineJoin = 'round';
      ctx.beginPath();
      plan.forEach((p, i) => {
        const s = worldToScreen(p.x, p.y);
        if (i === 0) ctx.moveTo(s.x, s.y); else ctx.lineTo(s.x, s.y);
      });
      ctx.stroke();
    }

    // Oranje = lidar (live), rood = camera (lage obstakels, met geheugen).
    const now = performance.now();
    const dot = Math.max(2, Math.min(5, 0.04 * view.scale));
    [['lidar', '#f5a142'], ['floor', '#f25c5c']].forEach(([kind, color]) => {
      const o = obstacles[kind];
      if (now - o.time > OBSTACLE_TIMEOUT) return;
      ctx.fillStyle = color;
      for (let i = 0; i + 1 < o.points.length; i += 2) {
        const s = worldToScreen(o.points[i] / 100, o.points[i + 1] / 100);
        ctx.fillRect(s.x - dot / 2, s.y - dot / 2, dot, dot);
      }
    });

    if (robot) drawArrow(robot.x, robot.y, robot.yaw, '#409cff', 12);

    if (gesture.pick) {
      const color = mode === 'goal' ? '#f5b942' : '#c47dff';
      drawArrow(gesture.pick.x, gesture.pick.y, gesture.pick.yaw, color, 14);
    }
  }

  function loop() {
    // Verlopen obstakels wegwerken, ook zonder nieuwe berichten.
    const now = performance.now();
    if (Object.values(obstacles).some((o) => o.points.length && now - o.time > OBSTACLE_TIMEOUT)) {
      Object.values(obstacles).forEach((o) => { if (now - o.time > OBSTACLE_TIMEOUT) o.points = []; });
      dirty = true;
    }
    if (dirty) {
      dirty = false;
      draw();
    }
    requestAnimationFrame(loop);
  }

  // --- doel en startpositie ---------------------------------------------------------

  function publishPick(pick) {
    const pose = { position: { x: pick.x, y: pick.y, z: 0 }, orientation: quaternionFromYaw(pick.yaw) };
    const header = { frame_id: 'map', stamp: { sec: 0, nanosec: 0 } };
    if (mode === 'goal' && goalTopic) {
      goalTopic.publish({ header, pose });
      statusEl.textContent = 'doel verstuurd';
    } else if (mode === 'initial' && initialTopic) {
      const covariance = new Array(36).fill(0);
      covariance[0] = 0.25;
      covariance[7] = 0.25;
      covariance[35] = 0.07;
      initialTopic.publish({ header, pose: { pose, covariance } });
      statusEl.textContent = 'startpositie gezet';
    }
    setMode('pan');
  }

  const HINTS = {
    goal: 'Tik op de kaart waar hij heen moet; sleep voor de richting.',
    initial: 'Tik waar de robot staat; sleep in de richting waarin hij kijkt.',
  };

  function setMode(newMode) {
    mode = newMode;
    document.querySelectorAll('.map-tools [data-mode]').forEach((b) => {
      b.classList.toggle('active', b.dataset.mode === mode);
    });
    hintEl.textContent = HINTS[mode] || '';
    hintEl.hidden = !HINTS[mode];
  }

  document.querySelectorAll('.map-tools [data-mode]').forEach((b) => {
    b.addEventListener('click', () => setMode(b.dataset.mode));
  });

  document.getElementById('nav-cancel').addEventListener('click', () => {
    if (!cancelService) return;
    // Lege goal-id en tijdstempel 0: alle doelen annuleren.
    cancelService.callService(
      { goal_info: { goal_id: { uuid: new Array(16).fill(0) }, stamp: { sec: 0, nanosec: 0 } } },
      () => { statusEl.textContent = 'gestopt'; });
  });

  document.getElementById('map-fit').addEventListener('click', fit);

  // --- draaiing per kaart ------------------------------------------------------------
  // Hoe je de kaart wilt zien hangt af van waar je zit (bureau: aan de pc = 180 graden).
  // Per kaart onthouden op dit toestel; knop "Draai" = 90 graden verder.
  const DEFAULT_ROTATION = { bureau: 180 };
  let mapName = '';

  function storedRotations() {
    try {
      return JSON.parse(localStorage.getItem('mapRotation') || '{}');
    } catch (e) {
      return {};
    }
  }

  function setMapName(name) {
    if (!name || name === mapName) return;
    mapName = name;
    const deg = storedRotations()[name] ?? DEFAULT_ROTATION[name] ?? 0;
    view.rot = (deg * Math.PI) / 180;
    fitted = false;
    if (mapInfo) fit();
    dirty = true;
  }

  document.getElementById('map-rotate').addEventListener('click', () => {
    const deg = (Math.round((view.rot * 180) / Math.PI) + 90) % 360;
    view.rot = (deg * Math.PI) / 180;
    if (mapName) {
      const all = storedRotations();
      all[mapName] = deg;
      try {
        localStorage.setItem('mapRotation', JSON.stringify(all));
      } catch (e) { /* privé venster: alleen voor nu */ }
    }
    fit();
  });

  // --- gebaren ----------------------------------------------------------------------

  const pointers = new Map();
  const gesture = { last: null, pinch: null, pick: null };

  function canvasPoint(event) {
    const rect = canvas.getBoundingClientRect();
    return { x: event.clientX - rect.left, y: event.clientY - rect.top };
  }

  canvas.addEventListener('pointerdown', (event) => {
    canvas.setPointerCapture(event.pointerId);
    pointers.set(event.pointerId, canvasPoint(event));
    if (pointers.size === 1) {
      const p = canvasPoint(event);
      gesture.last = p;
      if (mode !== 'pan') {
        const wp = screenToWorld(p.x, p.y);
        gesture.pick = { x: wp.x, y: wp.y, yaw: robot ? robot.yaw : 0, sx: p.x, sy: p.y };
        dirty = true;
      }
    } else if (pointers.size === 2) {
      gesture.pick = null;
      const [a, b] = [...pointers.values()];
      gesture.pinch = { dist: Math.hypot(a.x - b.x, a.y - b.y), scale: view.scale };
    }
  });

  canvas.addEventListener('pointermove', (event) => {
    if (!pointers.has(event.pointerId)) return;
    const p = canvasPoint(event);
    pointers.set(event.pointerId, p);

    if (pointers.size === 2 && gesture.pinch) {
      const [a, b] = [...pointers.values()];
      const dist = Math.hypot(a.x - b.x, a.y - b.y);
      const center = { x: (a.x + b.x) / 2, y: (a.y + b.y) / 2 };
      const before = screenToWorld(center.x, center.y);
      view.scale = Math.min(400, Math.max(5, gesture.pinch.scale * dist / gesture.pinch.dist));
      const after = screenToWorld(center.x, center.y);
      view.cx += before.x - after.x;
      view.cy += before.y - after.y;
      dirty = true;
    } else if (pointers.size === 1 && gesture.pick) {
      const dx = p.x - gesture.pick.sx;
      const dy = p.y - gesture.pick.sy;
      if (Math.hypot(dx, dy) > 10) gesture.pick.yaw = Math.atan2(-dy, dx) - view.rot;
      dirty = true;
    } else if (pointers.size === 1 && gesture.last) {
      const before = screenToWorld(gesture.last.x, gesture.last.y);
      const after = screenToWorld(p.x, p.y);
      view.cx -= after.x - before.x;
      view.cy -= after.y - before.y;
      gesture.last = p;
      dirty = true;
    }
  });

  function endPointer(event) {
    pointers.delete(event.pointerId);
    if (pointers.size === 0) {
      if (gesture.pick && event.type === 'pointerup') publishPick(gesture.pick);
      gesture.pick = null;
      gesture.pinch = null;
      gesture.last = null;
      dirty = true;
    } else if (pointers.size === 1) {
      gesture.pinch = null;
      gesture.last = [...pointers.values()][0];
    }
  }

  document.getElementById('nav-find').addEventListener('click', () => {
    if (findTopic) findTopic.publish({});
  });

  ['pointerup', 'pointercancel'].forEach((type) => canvas.addEventListener(type, endPointer));
  // Ook als het paneel ernaast opengaat (iPad liggend) of de werkbalk van Safari verdwijnt.
  new ResizeObserver(() => { dirty = true; }).observe(canvas);

  // --- verbinding -------------------------------------------------------------------

  function start(rosConnection) {
    ros = rosConnection;
    const topic = (name, messageType, options = {}) =>
      new ROSLIB.Topic({ ros, name, messageType, ...options });

    // De kaart verandert zelden: hooguit om de 5 s ophalen (alleen de laatste).
    subscribeMap();
    topic('/robot_pose', 'geometry_msgs/msg/PoseStamped', { throttle_rate: 200 }).subscribe(onPose);
    topic('/plan', 'nav_msgs/msg/Path', { throttle_rate: 1000, queue_length: 1 }).subscribe(onPlan);
    topic('/dashboard/lidar_points', 'std_msgs/msg/Int16MultiArray', { throttle_rate: 500, queue_length: 1 })
      .subscribe((m) => onObstacles('lidar', m));
    topic('/dashboard/floor_points', 'std_msgs/msg/Int16MultiArray', { throttle_rate: 500, queue_length: 1 })
      .subscribe((m) => onObstacles('floor', m));
    topic('/navigate_to_pose/_action/status', 'action_msgs/msg/GoalStatusArray', { throttle_rate: 500 })
      .subscribe(onNavStatus);

    topic('/localization/status', 'std_msgs/msg/String').subscribe(onFindStatus);
    topic('/maps/list', 'std_msgs/msg/String').subscribe((m) => setMapName(JSON.parse(m.data).current));
    findTopic = topic('/localization/find', 'std_msgs/msg/Empty');

    goalTopic = topic('/goal_pose', 'geometry_msgs/msg/PoseStamped');
    initialTopic = topic('/initialpose_rviz', 'geometry_msgs/msg/PoseWithCovarianceStamped');
    cancelService = new ROSLIB.Service({
      ros, name: '/navigate_to_pose/_action/cancel_goal', serviceType: 'action_msgs/srv/CancelGoal',
    });
  }

  requestAnimationFrame(loop);
  // --- kaart opnieuw ophalen bij het wisselen van module ---------------------------
  // Rosbridge kiest de QoS van zijn abonnement op het moment van abonneren. Is er dan geen
  // publisher met "transient local" (bv. net tussen explore en navigation), dan luistert hij
  // gewoon en krijgt hij de kaart van map_server (één keer gepubliceerd) nooit: de kaart bleef
  // leeg tot de app herstartte (demo 29-10-2026). Daarom opnieuw abonneren zodra een
  // kaartmodule aan staat, en nog eens kort daarna (map_server activeert pas na de module).
  const MAP_MODULES = ['navigation', 'explore', 'slam'];
  let mapTopic = null;
  let mapModule = { name: '', starting: false };
  let lastMapModuleState = {};
  let resubscribeTimers = [];

  function subscribeMap() {
    if (mapTopic) mapTopic.unsubscribe();
    mapTopic = new ROSLIB.Topic({
      ros, name: '/map', messageType: 'nav_msgs/msg/OccupancyGrid', throttle_rate: 5000, queue_length: 1,
    });
    mapTopic.subscribe(onMap);
  }

  function onModules(modules) {
    const starting = MAP_MODULES.find((m) => modules[m] && modules[m].state === 'start');
    mapModule = { name: starting ? (starting === 'navigation' ? 'Navigatie' : 'Kaart maken') : '', starting: !!starting };
    dirty = true;
    for (const name of MAP_MODULES) {
      const state = modules[name] && modules[name].state;
      if (state === 'aan' && lastMapModuleState[name] !== 'aan' && ros) {
        // Andere kaart op komst: de oude niet meer tonen tot de nieuwe binnen is.
        mapImage = null;
        fitted = false;
        resubscribeTimers.forEach(clearTimeout);
        resubscribeTimers = [0, 3000, 10000].map((ms) => setTimeout(subscribeMap, ms));
      }
      lastMapModuleState[name] = state;
    }
  }

  return { start, onModules };
})();
