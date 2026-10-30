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
  const chatterBtn = document.getElementById('chat-chatter');
  let sayTopic = null;
  let resetTopic = null;
  let chatterTopic = null;
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
    remember(input.value.trim());
    send(input.value.trim());
    input.value = '';  // toetsenbord open laten voor het volgende
  });

  // Versturen met de blauwe pijl van het iPhone-toetsenbord (enterkeyhint="send") of met
  // Stuur. Vanzelf versturen na 2 s stilte is weggehaald: bij een pauze midden in een zin
  // vertrok dan een half bericht (30-10-2026).
  // Na het versturen zet het dicteren van iOS de definitieve tekst soms nog eens in het lege
  // vak: die tekst wissen in plaats van ze nog eens te versturen.
  let lastSent = { text: '', at: 0 };
  input.addEventListener('input', () => {
    if (input.value.trim() === lastSent.text && Date.now() - lastSent.at < 10000) {
      input.value = '';
    }
  });

  // Onthouden wat net verstuurd werd (zie hierboven).
  function remember(text) {
    lastSent = { text, at: Date.now() };
  }
  document.querySelectorAll('[data-say]').forEach((b) => {
    b.addEventListener('click', () => send(b.dataset.say));
  });
  document.getElementById('chat-reset').addEventListener('click', () => {
    if (resetTopic) resetTopic.publish({});
  });
  chatterBtn.addEventListener('click', () => {
    if (chatterTopic) chatterTopic.publish({ data: !chatterBtn.classList.contains('on') });
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
    if (s.chatter !== undefined) chatterBtn.classList.toggle('on', s.chatter);
    // "klaar" met een waarschuwing (bv. klein taalmodel) toch tonen.
    const warn = s.state === 'klaar' && s.text.startsWith('Klein');
    status.textContent = !warn && s.state in STATUS_TEXT ? STATUS_TEXT[s.state] : s.text;
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
    chatterTopic = new ROSLIB.Topic({ ros, name: '/assistant/chatter', messageType: 'std_msgs/msg/Bool' });
  }

  return { start };
})();
