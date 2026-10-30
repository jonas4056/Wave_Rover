// "Praat met Wave" in het dashboard: chatten met de robot (module scene, zie assistant.py).
// Praten kan met de dicteerknop (microfoon) van het iPhone-toetsenbord; Wave antwoordt hier en
// via de speaker. Wordt gestart vanuit app.js met TalkCard.start(ros) na elke (her)verbinding.
'use strict';

const TalkCard = (() => {
  const logEl = document.getElementById('chat-log');
  const form = document.getElementById('chat-form');
  const input = document.getElementById('chat-input');
  const status = document.getElementById('chat-status');
  const STATUS_TEXT = {
    laden: 'Wave wordt wakker (taalmodel laden)…',
    denken: 'Wave denkt…',
    kijken: 'Wave kijkt…',
    spreken: 'Wave spreekt…',
    klaar: '',
  };
  const INTRO = logEl.innerHTML;
  let sayTopic = null;
  let resetTopic = null;
  let state = '';
  let pending = [];  // berichten die wachten tot het taalmodel geladen is
  let pendingTimer = null;

  // Staat de module scene niet aan (of laadt Wave nog), dan aanzetten en het bericht bewaren
  // tot Wave "klaar" meldt.
  function send(text) {
    if (!sayTopic || !text) return;
    const module = lastModules.scene && lastModules.scene.state;
    if (module === 'aan' && state && state !== 'laden') {
      sayTopic.publish({ data: text });
      return;
    }
    pending.push(text);
    status.textContent = STATUS_TEXT.laden;
    if (module !== 'aan' && module !== 'start') callEnable('scene', true);
    clearTimeout(pendingTimer);
    pendingTimer = setTimeout(() => {
      if (pending.length) status.textContent = 'Wave reageert niet; kijk bij Modules';
      pending = [];
    }, 120000);
  }

  form.addEventListener('submit', (event) => {
    event.preventDefault();
    send(input.value.trim());
    input.value = '';  // toetsenbord open laten voor het volgende
  });
  document.querySelectorAll('[data-say]').forEach((b) => {
    b.addEventListener('click', () => send(b.dataset.say));
  });
  document.getElementById('chat-reset').addEventListener('click', () => {
    if (resetTopic) resetTopic.publish({});
  });

  function showLog(entries) {
    if (!entries.length) {
      logEl.innerHTML = INTRO;
      return;
    }
    logEl.replaceChildren(...entries.map((e) => {
      const p = document.createElement('p');
      p.className = `msg ${e.wie === 'jij' ? 'me' : 'wave'}`;
      p.textContent = e.tekst;
      return p;
    }));
    logEl.scrollTop = logEl.scrollHeight;
  }

  function onStatus(msg) {
    const s = JSON.parse(msg.data);
    state = s.state;
    status.textContent = s.state in STATUS_TEXT ? STATUS_TEXT[s.state] : s.text;
    if (s.state === 'klaar' && pending.length) {
      pending.forEach((text) => sayTopic.publish({ data: text }));
      pending = [];
    }
  }

  function start(ros) {
    new ROSLIB.Topic({ ros, name: '/assistant/log', messageType: 'std_msgs/msg/String' })
      .subscribe((m) => showLog(JSON.parse(m.data)));
    new ROSLIB.Topic({ ros, name: '/assistant/status', messageType: 'std_msgs/msg/String' })
      .subscribe(onStatus);
    sayTopic = new ROSLIB.Topic({ ros, name: '/assistant/say', messageType: 'std_msgs/msg/String' });
    resetTopic = new ROSLIB.Topic({ ros, name: '/assistant/reset', messageType: 'std_msgs/msg/Empty' });
  }

  return { start };
})();
