// "Wat zie je?" in het dashboard: de robot beschrijft het camerabeeld en zegt het via de
// speaker (module scene, zie scene_describer.py).
// Wordt gestart vanuit app.js met SceneCard.start(ros) na elke (her)verbinding.
'use strict';

const SceneCard = (() => {
  const ask = document.getElementById('scene-ask');
  const question = document.getElementById('scene-question');
  const answer = document.getElementById('scene-text');
  const status = document.getElementById('scene-status');
  const BUSY = ['kijken', 'denken', 'spreken'];
  let describeTopic = null;
  let state = '';
  let pending = null;  // vraag die wacht tot het taalmodel geladen is
  let pendingTimer = null;

  function send(text) {
    pending = null;
    describeTopic.publish({ data: text });
  }

  // Module scene aanzetten (ook de camera) als dat nog niet zo is; de vraag gaat pas weg als
  // het taalmodel geladen is (status "klaar").
  ask.addEventListener('click', () => {
    if (!describeTopic) return;
    const text = question.value.trim();
    question.blur();
    if (state === 'klaar' || state === 'fout') {
      send(text);
      return;
    }
    pending = text;
    status.textContent = 'Module scene starten…';
    callEnable('scene', true);
    clearTimeout(pendingTimer);
    pendingTimer = setTimeout(() => {
      if (pending !== null) status.textContent = 'Het taalmodel reageert niet; kijk bij Modules';
      pending = null;
    }, 120000);
  });

  function onStatus(msg) {
    const s = JSON.parse(msg.data);
    state = s.state;
    if (s.state === 'klaar' && pending !== null) send(pending);
    const busy = BUSY.includes(s.state) || pending !== null;
    ask.disabled = busy;
    if (BUSY.includes(s.state) && s.state !== 'spreken') {
      status.textContent = s.text;
    } else if (s.state === 'laden') {
      status.textContent = 'Taalmodel laden (±20 s)…';
    } else if (s.state === 'fout') {
      status.textContent = s.text;
    } else if (s.seconds !== undefined) {
      answer.textContent = s.text;
      answer.hidden = false;
      status.textContent = s.state === 'spreken' ? `Bedacht in ${s.seconds} s; spreekt…`
        : `Bedacht in ${s.seconds} s`;
    } else {
      status.textContent = s.text;
    }
  }

  function start(ros) {
    new ROSLIB.Topic({ ros, name: '/scene/status', messageType: 'std_msgs/msg/String' })
      .subscribe(onStatus);
    describeTopic = new ROSLIB.Topic({ ros, name: '/scene/describe', messageType: 'std_msgs/msg/String' });
  }

  return { start };
})();
