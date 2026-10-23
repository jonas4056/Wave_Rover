// Wave Rover-dashboard: verbinding met rosbridge op de Jetson.
'use strict';

// Zelfde host als de pagina: werkt via jetson.local, een IP-adres en later de hotspot.
const ROSBRIDGE_URL = `ws://${window.location.hostname}:9090`;
const RECONNECT_DELAY_MS = 2000;

const badge = document.getElementById('connection');
const detail = document.getElementById('connection-detail');

let ros = null;
let reconnectTimer = null;

function setStatus(state, text, detailText) {
  badge.className = `badge ${state}`;
  badge.textContent = text;
  detail.textContent = detailText;
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
  setStatus('connecting', 'verbinden…', `Verbinden met ${ROSBRIDGE_URL}…`);
  ros = new ROSLIB.Ros({ url: ROSBRIDGE_URL });

  ros.on('connection', () => {
    setStatus('online', 'verbonden', `Verbonden met ${ROSBRIDGE_URL}`);
  });
  ros.on('error', () => {
    setStatus('offline', 'niet verbonden', 'Robot niet bereikbaar, opnieuw proberen…');
  });
  ros.on('close', () => {
    setStatus('offline', 'niet verbonden', 'Verbinding verbroken, opnieuw proberen…');
    scheduleReconnect();
  });
}

connect();
