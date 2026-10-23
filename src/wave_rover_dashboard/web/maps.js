// Kaarten in het dashboard: oplijsten, kiezen voor de navigatie en de SLAM-kaart opslaan.
// Wordt gestart vanuit app.js met MapsCard.start(ros) na elke (her)verbinding.
'use strict';

const MapsCard = (() => {
  const list = document.getElementById('maps');
  const message = document.getElementById('maps-message');
  const form = document.getElementById('map-save');
  const nameInput = document.getElementById('map-name');
  let saveTopic = null;
  let selectTopic = null;

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

  function start(ros) {
    new ROSLIB.Topic({ ros, name: '/maps/list', messageType: 'std_msgs/msg/String' })
      .subscribe((m) => show(JSON.parse(m.data)));
    saveTopic = new ROSLIB.Topic({ ros, name: '/maps/save', messageType: 'std_msgs/msg/String' });
    selectTopic = new ROSLIB.Topic({ ros, name: '/maps/select', messageType: 'std_msgs/msg/String' });
  }

  return { start };
})();
