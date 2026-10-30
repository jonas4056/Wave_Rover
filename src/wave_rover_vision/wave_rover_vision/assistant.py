"""Praten met de robot (module scene): Wave antwoordt op wat je zegt of typt in de app.

Het brein is het beeldtaalmodel van llama-server (scene.launch.py, lokaal op de GPU). Het kiest
zelf een tool als het iets moet zien of weten (getest 30-10-2026: Qwen3-VL-4B koos in het
Nederlands telkens de juiste tool, 1-2 s); deze node voert de tools uit. Het antwoord gaat als
tekst naar de app en via Piper (Vlaamse stem) naar de speaker.

Topics (gewone types, zodat rosbridge geen eigen interfaces nodig heeft):
- /assistant/say (std_msgs/String): wat jij zegt (getypt, of gedicteerd met de iPhone).
- /assistant/reset (std_msgs/Empty): nieuw gesprek.
- /assistant/log (std_msgs/String, JSON, latched): [{"wie": "jij|wave", "tekst": "..."}, ...]
- /assistant/status (std_msgs/String, JSON, latched):
  {"state": "laden|klaar|denken|kijken|spreken|fout", "text": "..."}

Tools (nog niet rijden): kijk (één camerabeeld, beschreven door hetzelfde model met jouw vraag
erbij), batterij, uur, en het spel "ik zie, ik zie wat jij niet ziet" (Wave kiest stiekem een
voorwerp op zijn camerabeeld). Het camerabeeld wordt alleen opgehaald als er gekeken wordt;
zonder vragen kost de node niets.
"""
import base64
import io
import json
import queue
import re
import threading
import time
import urllib.error
import urllib.request

import rclpy
from PIL import Image
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import BatteryState, CompressedImage
from std_msgs.msg import Empty, String
from wave_rover_base import speech

PERSONA = (
    'Je bent Wave, een vrolijke kleine robot op wielen die bij Jonas thuis woont (een bureau '
    'boven en een benedenverdieping). Je praat Vlaams-Nederlands, kort en gezellig: één of '
    'twee zinnen, zoals je hardop praat. Geen opsommingen, geen opmaak, geen emoji. Gebruik een '
    'tool als je iets moet zien of over jezelf moet weten, en verzin niets. Rijden via een '
    'gesprek kan nog niet; zeg dat eerlijk als Jonas het vraagt.'
)
# Getest op camerabeelden (29-10-2026): zonder voorbeeld en met lage temperature viel
# Qwen3-VL-2B in herhaallussen ("een kast, een wastafel, een kast, ...") of gaf het alleen
# losse woorden; met een voorbeeldzin en de sampling die Qwen aanraadt twee gewone zinnen.
DESCRIBE_PROMPT = (
    'Je bent de ogen van een kleine robot op wielen die door een huis rijdt. Je vertelt in '
    'het Nederlands wat je ziet, in twee korte, gewone zinnen zoals je het hardop zou zeggen. '
    'Zeg waar de belangrijkste dingen staan (links, rechts, voor mij). Geen opsommingen, geen '
    'opmaak. Verzin niets: twijfel je, zeg dat dan.\n'
    'Voorbeeld: "Ik zie een woonkamer met een grijze zetel aan de rechterkant. Vlak voor mij '
    'ligt een rode bal op de vloer."'
)
# presence_penalty voorkomt herhalingen (aanbevolen voor Qwen3-VL Instruct).
QWEN_SAMPLING = {'temperature': 0.7, 'top_p': 0.8, 'top_k': 20, 'presence_penalty': 1.5}
DEFAULT_QUESTION = 'Wat zie je? Begin met "Ik zie".'

TOOLS = [
    {'type': 'function', 'function': {
        'name': 'kijk',
        'description': 'Neem een foto met je camera en kijk wat er voor je staat. Gebruik dit '
                       'voor elke vraag over wat je ziet.',
        'parameters': {'type': 'object', 'properties': {}, 'required': []}}},
    {'type': 'function', 'function': {
        'name': 'batterij',
        'description': 'Hoe vol je batterijen zijn (Jetson en chassis).',
        'parameters': {'type': 'object', 'properties': {}, 'required': []}}},
    {'type': 'function', 'function': {
        'name': 'uur',
        'description': 'Het huidige uur en de datum.',
        'parameters': {'type': 'object', 'properties': {}, 'required': []}}},
    {'type': 'function', 'function': {
        'name': 'start_ik_zie_spel',
        'description': 'Start het spel "ik zie, ik zie wat jij niet ziet": je kiest stiekem een '
                       'voorwerp dat je met je camera ziet en Jonas moet het raden.',
        'parameters': {'type': 'object', 'properties': {}, 'required': []}}},
    {'type': 'function', 'function': {
        'name': 'stop_ik_zie_spel',
        'description': 'Het spel is voorbij: Jonas heeft het geraden of geeft op.',
        'parameters': {'type': 'object', 'properties': {}, 'required': []}}},
]
DAYS = ['maandag', 'dinsdag', 'woensdag', 'donderdag', 'vrijdag', 'zaterdag', 'zondag']
MONTHS = ['januari', 'februari', 'maart', 'april', 'mei', 'juni', 'juli', 'augustus',
          'september', 'oktober', 'november', 'december']
MAX_HISTORY = 8      # berichten (jij + Wave) die meegaan; de context is 2048 tokens
MAX_TOOL_ROUNDS = 3
LOG_LENGTH = 30      # berichten in /assistant/log voor de app
# Alles wat Piper niet kan uitspreken (emoji, opmaak) weg.
UNSPEAKABLE = re.compile(r'[*_#`~>|\[\]]|[\U0001F000-\U0001FAFF☀-➿️]')


class Assistant(Node):

    def __init__(self):
        super().__init__('assistant')
        self.declare_parameter('server', 'http://127.0.0.1:8090')
        # Kleiner beeld = minder beeldtokens = sneller; 640 px is genoeg om te beschrijven.
        self.declare_parameter('image_width', 640)
        self.declare_parameter('piper', speech.PIPER)
        self.declare_parameter('voice', speech.VOICE)
        self.declare_parameter('speaker', speech.SPEAKER)
        self.p = {n: self.get_parameter(n).value for n in (
            'server', 'image_width', 'piper', 'voice', 'speaker')}

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(String, 'assistant/status', latched)
        self.log_pub = self.create_publisher(String, 'assistant/log', latched)
        self.create_subscription(String, 'assistant/say', self.on_say, 10)
        self.create_subscription(Empty, 'assistant/reset', self.on_reset, 10)
        self.battery = {}
        self.create_subscription(BatteryState, 'ups/battery',
                                 lambda m: self.battery.__setitem__('Jetson', m), 10)
        self.create_subscription(BatteryState, 'chassis/battery',
                                 lambda m: self.battery.__setitem__('chassis', m), 10)

        self.history = []    # [{'role': 'user'|'assistant', 'content': tekst}]
        self.log = []        # [{'wie': ..., 'tekst': ...}] voor de app
        self.game = None     # {'voorwerp': ..., 'kleur': ...} tijdens "ik zie, ik zie"
        self.inbox = queue.Queue(maxsize=3)
        # Camerabeeld op vraag: abonneren moet in de ROS-thread, de vraag komt uit de werker.
        self.frame = None
        self.frame_ready = threading.Event()
        self.image_sub = None
        self.wake = self.create_guard_condition(self.subscribe_image)

        self.state = 'laden'
        self.publish_status('Taalmodel laden…')
        self.publish_log()
        self.ready_timer = self.create_timer(1.0, self.check_ready)
        threading.Thread(target=self.work, daemon=True).start()

    # --- taalmodel klaar? -------------------------------------------------------------

    def check_ready(self):
        """llama-server laadt het model (±10-30 s); /health geeft 200 zodra het klaar is."""
        try:
            with urllib.request.urlopen(self.p['server'] + '/health', timeout=0.5) as r:
                ok = r.status == 200
        except (urllib.error.URLError, OSError):
            ok = False
        if ok:
            self.ready_timer.cancel()
            self.set_state('klaar', 'Klaar: zeg of typ iets')

    # --- ontvangen --------------------------------------------------------------------

    def on_say(self, msg):
        text = msg.data.strip()
        if not text:
            return
        try:
            self.inbox.put_nowait(text)
        except queue.Full:
            self.get_logger().warn(f'Te veel tegelijk, genegeerd: {text}')

    def on_reset(self, _msg):
        self.history, self.log, self.game = [], [], None
        self.publish_log()
        self.publish_status('Nieuw gesprek')

    # --- gesprek (werker-thread) ------------------------------------------------------

    def work(self):
        while rclpy.ok():
            text = self.inbox.get()
            while self.state == 'laden':
                time.sleep(0.5)
            self.add_log('jij', text)
            try:
                answer = self.chat(text)
            except Exception as e:  # noqa: BLE001 - elke fout melden, de node blijft draaien
                self.get_logger().error(f'Antwoorden mislukt: {e}')
                self.set_state('fout', f'Antwoorden mislukt: {e}')
                self.state = 'klaar'  # een volgende vraag mag gewoon opnieuw
                continue
            self.add_log('wave', answer)
            self.set_state('spreken', 'Spreekt…')
            try:
                speech.say(UNSPEAKABLE.sub('', answer), self.p['voice'], self.p['speaker'],
                           self.p['piper'])
            except (OSError, RuntimeError) as e:
                self.get_logger().warn(f'Speaker: {e}')
            self.set_state('klaar', 'Klaar')

    def chat(self, text):
        start = time.monotonic()
        self.set_state('denken', 'Denkt…')
        self.history.append({'role': 'user', 'content': text})
        system = PERSONA
        if self.game:
            system += (f'\nJullie spelen "ik zie, ik zie wat jij niet ziet". Het geheime '
                       f'voorwerp is: {self.game["voorwerp"]} (kleur: {self.game["kleur"]}). '
                       f'Verklap het niet. Zeg of een gok juist is en geef een kleine hint als '
                       f'Jonas erom vraagt. Is het geraden of geeft hij op, gebruik dan '
                       f'stop_ik_zie_spel.')
        messages = [{'role': 'system', 'content': system}] + self.history[-MAX_HISTORY:]
        answer = ''
        for _ in range(MAX_TOOL_ROUNDS):
            reply = self.ask(messages, tools=TOOLS, max_tokens=150)
            calls = reply.get('tool_calls') or []
            if not calls:
                answer = (reply.get('content') or '').strip()
                break
            messages.append({'role': 'assistant', 'content': reply.get('content') or '',
                             'tool_calls': calls})
            for call in calls:
                name = call['function']['name']
                result = self.run_tool(name, text)
                self.get_logger().info(f'Tool {name}: {result}')
                messages.append({'role': 'tool', 'tool_call_id': call.get('id', name),
                                 'content': result})
        answer = answer or 'Sorry, daar weet ik even geen antwoord op.'
        self.history.append({'role': 'assistant', 'content': answer})
        self.get_logger().info(f'"{text}" -> "{answer}" ({time.monotonic() - start:.1f} s)')
        return answer

    def run_tool(self, name, question):
        if name == 'kijk':
            return self.look(question)
        if name == 'batterij':
            return self.battery_text()
        if name == 'uur':
            t = time.localtime()
            return (f'Het is {t.tm_hour}u{t.tm_min:02d} op {DAYS[t.tm_wday]} {t.tm_mday} '
                    f'{MONTHS[t.tm_mon - 1]} {t.tm_year}.')
        if name == 'start_ik_zie_spel':
            return self.start_game()
        if name == 'stop_ik_zie_spel':
            self.game = None
            return 'Het spel is gestopt.'
        return f'Onbekende tool {name}.'

    # --- tools ------------------------------------------------------------------------

    def battery_text(self):
        parts = []
        for name in ('Jetson', 'chassis'):
            m = self.battery.get(name)
            if m is None:
                parts.append(f'{name}: geen meting')
            elif not m.present:
                parts.append(f'{name}: geen batterij (op de adapter)')
            else:
                charging = ' en laadt' if m.power_supply_status == BatteryState.POWER_SUPPLY_STATUS_CHARGING else ''
                parts.append(f'{name}: {round(m.percentage * 100)}%{charging}')
        return '; '.join(parts) + '.'

    def look(self, question):
        jpeg = self.grab_frame()
        if jpeg is None:
            return 'Geen camerabeeld: de camera staat uit.'
        self.set_state('kijken', 'Kijkt…')
        reply = self.ask([
            {'role': 'system', 'content': DESCRIBE_PROMPT},
            {'role': 'user', 'content': [
                self.image_part(jpeg),
                {'type': 'text', 'text': f'{DEFAULT_QUESTION} Jonas vroeg: "{question}"'}]}],
            max_tokens=100)
        self.set_state('denken', 'Denkt…')
        return (reply.get('content') or '').strip() or 'Ik zie niets duidelijks.'

    def start_game(self):
        jpeg = self.grab_frame()
        if jpeg is None:
            return 'Geen camerabeeld: de camera staat uit, het spel kan niet.'
        self.set_state('kijken', 'Kiest iets…')
        reply = self.ask([{'role': 'user', 'content': [
            self.image_part(jpeg),
            {'type': 'text', 'text': 'We spelen "ik zie, ik zie wat jij niet ziet". Kies één '
                                     'duidelijk zichtbaar voorwerp op deze foto. Antwoord '
                                     'alleen met JSON: {"voorwerp": "...", "kleur": "..."}'}]}],
            max_tokens=60, temperature=0.7)
        self.set_state('denken', 'Denkt…')
        match = re.search(r'\{.*\}', reply.get('content') or '', re.S)
        try:
            game = json.loads(match.group(0)) if match else {}
        except json.JSONDecodeError:
            game = {}
        if not game.get('voorwerp'):
            return 'Ik kon niets kiezen op mijn camerabeeld.'
        self.game = {'voorwerp': game['voorwerp'], 'kleur': game.get('kleur') or 'onbekend'}
        self.get_logger().info(f'Ik zie, ik zie: {self.game}')
        return (f'Spel gestart. Geheim voorwerp: {self.game["voorwerp"]} (kleur: '
                f'{self.game["kleur"]}). Zeg nu: "Ik zie, ik zie wat jij niet ziet, en het is '
                f'{self.game["kleur"]}." Verklap het voorwerp niet.')

    # --- camerabeeld op vraag ---------------------------------------------------------

    def grab_frame(self, timeout=5.0):
        self.frame = None
        self.frame_ready.clear()
        self.wake.trigger()  # subscribe_image in de ROS-thread
        if not self.frame_ready.wait(timeout):
            return None
        return self.small_jpeg(self.frame)

    def subscribe_image(self):
        if self.image_sub is None:
            self.image_sub = self.create_subscription(
                CompressedImage, 'camera/image_low/compressed', self.on_image,
                qos_profile_sensor_data)

    def on_image(self, msg):
        if self.image_sub is None:
            return
        self.destroy_subscription(self.image_sub)
        self.image_sub = None
        self.frame = bytes(msg.data)
        self.frame_ready.set()

    def small_jpeg(self, jpeg):
        image = Image.open(io.BytesIO(jpeg))
        width = self.p['image_width']
        if image.width > width:
            image = image.resize((width, round(image.height * width / image.width)))
        out = io.BytesIO()
        image.convert('RGB').save(out, 'JPEG', quality=85)
        return out.getvalue()

    @staticmethod
    def image_part(jpeg):
        return {'type': 'image_url', 'image_url': {
            'url': 'data:image/jpeg;base64,' + base64.b64encode(jpeg).decode()}}

    # --- taalmodel --------------------------------------------------------------------

    def ask(self, messages, tools=None, max_tokens=100, **sampling):
        body = {'messages': messages, 'max_tokens': max_tokens, **QWEN_SAMPLING, **sampling}
        if tools:
            body['tools'] = tools
        request = urllib.request.Request(
            self.p['server'] + '/v1/chat/completions', data=json.dumps(body).encode(),
            headers={'Content-Type': 'application/json'})
        with urllib.request.urlopen(request, timeout=120) as r:
            choice = json.load(r)['choices'][0]
        message = choice['message']
        if choice.get('finish_reason') == 'length' and message.get('content'):
            # Afgebroken door max_tokens: alleen de volledige zinnen.
            text = message['content'].strip()
            end = max(text.rfind(c) for c in '.!?')
            message['content'] = text[:end + 1] if end > 0 else text
        return message

    # --- publiceren -------------------------------------------------------------------

    def add_log(self, who, text):
        self.log = (self.log + [{'wie': who, 'tekst': text}])[-LOG_LENGTH:]
        self.publish_log()

    def publish_log(self):
        self.log_pub.publish(String(data=json.dumps(self.log)))

    def set_state(self, state, text):
        self.state = state
        self.publish_status(text)

    def publish_status(self, text):
        self.status_pub.publish(String(data=json.dumps({'state': self.state, 'text': text})))


def main(args=None):
    rclpy.init(args=args)
    node = Assistant()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
