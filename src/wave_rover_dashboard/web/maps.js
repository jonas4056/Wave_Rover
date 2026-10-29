// Kaarten in het dashboard: oplijsten, kiezen voor de navigatie, de SLAM-kaart opslaan en
// zelf verkennen (module explore, zie explorer.py).
// Wordt gestart vanuit app.js met MapsCard.start(ros) na elke (her)verbinding.
'use strict';

const MapsCard = (() => {
  const list = document.getElementById('maps');
  const message = document.getElementById('maps-message');
  const form = document.getElementById('map-save');
  const nameInput = document.getElementById('map-name');
  let saveTopic = null;
  let selectTopic = null;

  const exploreForm = document.getElementById('explore');
  const exploreName = document.getElementById('explore-name');
  const exploreStatus = document.getElementById('explore-status');
  const exploreStop = document.getElementById('explore-stop');
  const EXPLORE_BUSY = ['verkennen', 'terug', 'stoppen', 'opslaan'];
  let exploreStartTopic = null;
  let exploreStopTopic = null;
  let pendingName = '';  // start verstuurd, nog niet door de verkenner bevestigd
  let pendingTimer = null;

  function show(data) {
    list.replaceChildren();
    for (const name of data.maps) {
      const li = document.createElement('li');
      const label = document.createElement('span');
      label.textContent = name;
      if (name === data.current) {
        label.className = 'current';
        label.textContent = `${name} (in gebruik)`;
      }
      li.append(label);
      if (name !== data.current) {
        const use = document.createElement('button');
        use.type = 'button';
        use.textContent = 'gebruik';
        use.addEventListener('click', () => selectTopic.publish({ data: name }));
        li.append(use);
      }
      list.append(li);
    }
    if (data.message) message.textContent = data.message;
    form.querySelector('button').disabled = data.busy;
  }

  form.addEventListener('submit', (event) => {
    event.preventDefault();
    if (!saveTopic || !nameInput.checkValidity()) return;
    saveTopic.publish({ data: nameInput.value.trim() });
    nameInput.blur();
  });

  // Starten: module explore aanzetten (stopt slam/navigation) en de naam sturen. De verkenner
  // bestaat pas als de module draait, dus de start herhalen tot hij hem bevestigt.
  exploreForm.addEventListener('submit', (event) => {
    event.preventDefault();
    if (!exploreStartTopic || !exploreName.checkValidity()) return;
    pendingName = exploreName.value.trim();
    exploreName.blur();
    exploreStatus.textContent = 'Module explore starten…';
    callEnable('explore', true);
    clearInterval(pendingTimer);
    const giveUp = Date.now() + 120000;
    pendingTimer = setInterval(() => {
      if (!pendingName || Date.now() > giveUp) {
        clearInterval(pendingTimer);
        if (pendingName) exploreStatus.textContent = 'De verkenner reageert niet; kijk bij Modules';
        pendingName = '';
        return;
      }
      exploreStartTopic.publish({ data: pendingName });
    }, 2000);
  });

  exploreStop.addEventListener('click', () => {
    pendingName = '';
    if (exploreStopTopic) exploreStopTopic.publish({});
  });

  function onExploreStatus(msg) {
    const s = JSON.parse(msg.data);
    if (s.state !== 'wacht') pendingName = '';  // bevestigd (of een fout): niet meer herhalen
    exploreStatus.textContent = s.message;
    const busy = EXPLORE_BUSY.includes(s.state) || (s.state === 'wacht' && pendingName !== '');
    exploreStop.hidden = !busy;
    exploreForm.querySelector('button').disabled = busy;
  }

  function start(ros) {
    new ROSLIB.Topic({ ros, name: '/maps/list', messageType: 'std_msgs/msg/String' })
      .subscribe((m) => show(JSON.parse(m.data)));
    saveTopic = new ROSLIB.Topic({ ros, name: '/maps/save', messageType: 'std_msgs/msg/String' });
    selectTopic = new ROSLIB.Topic({ ros, name: '/maps/select', messageType: 'std_msgs/msg/String' });
    new ROSLIB.Topic({ ros, name: '/explore/status', messageType: 'std_msgs/msg/String' })
      .subscribe(onExploreStatus);
    exploreStartTopic = new ROSLIB.Topic({ ros, name: '/explore/start', messageType: 'std_msgs/msg/String' });
    exploreStopTopic = new ROSLIB.Topic({ ros, name: '/explore/stop', messageType: 'std_msgs/msg/Empty' });
  }

  return { start };
})();
