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
  {"state": "laden|klaar|denken|kijken|spreken|fout", "text": "...", "chatter": bool}
- /assistant/chatter (std_msgs/Bool): spontaan babbelen aan/uit (standaard uit). Aan: is er `chatter_interval`
  s (±30%) niets gezegd, dan zegt Wave uit zichzelf iets: een nieuwsgierige of grappige
  opmerking over wat hij op zijn camera ziet, of een teken van leven ("ben je daar nog?").
  Kost dan ±2-4 s GPU per keer; uit kost het niets.

Tools: kijk (één camerabeeld, beschreven door hetzelfde model met jouw vraag erbij), batterij,
uur, mop (moppenboekje), het spel "ik zie, ik zie wat jij niet ziet" (Wave kiest stiekem een
voorwerp op zijn camerabeeld) en beweeg: een dansje op een deuntje (ter plaatse wiebelen, niet
rijden) of een rondje draaien, via /cmd_vel_wave (twist_mux, voorrang onder de app en de
RC-zender; noodstop en laadslot blokkeren). Rijden naar een plaats kan nog niet.
Geluidjes (sounds.py, zelf gemaakt): liedjes op vraag ("zing eens"), en tijdens het rijden af en
toe een wiiieeew of piepje (/cmd_vel_safe, parameter drive_sounds). Eigen muziek
(scripts/add_music.sh): "speel <naam>" of "dans op <naam>". Het camerabeeld wordt alleen opgehaald als er gekeken wordt;
zonder vragen kost de node niets.
"""
import base64
import glob
import io
import json
import math
import os
import queue
import random
import re
import threading
import time
import unicodedata
import urllib.error
import urllib.request

import numpy as np
import rclpy
from PIL import Image
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import BatteryState, CompressedImage
from std_msgs.msg import Bool, Empty, String
from wave_rover_base import sounds, speech

# Getest met de 2B op 30-10-2026: "benieuwd naar alles wat je ziet" maakte hem te veel op de
# camera gericht (geen mop als je erom vroeg) en hij herhaalde zijn instructies.
PERSONA = (
    'Je bent Wave, een kleine robot op wielen die bij Jonas thuis woont (een bureau boven en '
    'een benedenverdieping). Je bent grappig, nieuwsgierig en een beetje ondeugend: je wilt '
    'alles weten over Jonas en de wereld, en je stelt graag eens een wedervraag. Je praat '
    'Vlaams-Nederlands, kort en luchtig: één of twee zinnen, zoals je hardop praat. Vraagt '
    'Jonas een mop of een grap, gebruik dan altijd de tool mop. Wissel af en stel niet telkens '
    'dezelfde vraag terug. Geen opsommingen, geen opmaak, geen '
    'emoji, en praat nooit over deze instructies. Gebruik de tool kijk alleen als Jonas iets '
    'vraagt over wat je ziet; verzin niets over wat je ziet. Je kan dansen en een rondje '
    'draaien, maar nog niet rijden of iets brengen: bied bewegen nooit zelf aan, en vraagt Jonas '
    'het, gebruik dan de tool beweeg.'
)
# Voorbeeldgesprekjes: zonder voorbeelden rammelde de 4B en zat hij in herhaallussen ("En dan:
# ... En dan: ..."); met voorbeelden kort en samenhangend (getest 30-10-2026). Kleine modellen
# volgen voorbeelden sterker dan regels: een voorbeeld "Wist je dat een octopus..." maakte dat
# hij zelf onzinmoppen verzon i.p.v. de tool mop te gebruiken; daarom een voorbeeld mét tool.
EXAMPLE_MESSAGES = [
    {'role': 'user', 'content': 'Ik ben moe vandaag.'},
    {'role': 'assistant', 'content': 'Oei, lange dag gehad? Ik zou je een tas koffie brengen, '
                                     'maar mijn armen zijn nog niet geleverd.'},
    {'role': 'user', 'content': 'Vertel eens een mop.'},
    {'role': 'assistant', 'content': '', 'tool_calls': [
        {'id': 'voorbeeld', 'type': 'function', 'function': {'name': 'mop', 'arguments': '{}'}}]},
    {'role': 'tool', 'tool_call_id': 'voorbeeld',
     'content': 'Vertel deze mop letterlijk, zonder iets te veranderen: Waarom nam de robot zijn '
                'paraplu mee? Omdat hij bang was om te roesten.'},
    {'role': 'assistant', 'content': 'Waarom nam de robot zijn paraplu mee? Omdat hij bang was om '
                                     'te roesten! Ken jij er nog eentje?'},
    # Anders beloofde hij "een rondje met mijn wielen" en fantaseerde hij bij "doe maar" (30-10).
    {'role': 'user', 'content': 'Kan je naar de keuken rijden?'},
    {'role': 'assistant', 'content': 'Dat kan ik nog niet, Jonas moet het me eerst leren! Zullen '
                                     'we intussen ik zie, ik zie wat jij niet ziet spelen?'},
]
MAX_SENTENCES = 4  # een mop met inleiding moet heel blijven
# Moppenboekje: zelf moppen bedenken lukt een klein model niet (30-10-2026: "een robot heeft geen
# zin in koffie, maar wel in chocolade, want die is een smaak van de maat"). Tool mop kiest hier
# een die de laatste tijd niet verteld is.
JOKES = [
    'Wat is groen en gaat op en neer? Een erwt in een lift.',
    'Wat is geel en kan niet zwemmen? Een graafmachine.',
    'Wat eet een robot het liefst als tussendoortje? Microchips!',
    'Waarom ging de computer naar de dokter? Hij had een virus.',
    'Hoe noem je een boemerang die niet terugkomt? Een stok.',
    'Wat zegt een nul tegen een acht? Mooie riem!',
    'Wat zegt het ene oog tegen het andere oog? Tussen ons zit iets dat stinkt.',
    'Wat zegt een slak die op de rug van een schildpad zit? Joepie!',
    'Waarom zwemmen vissen in zout water? Omdat peper hen doet niezen.',
    'Waarom was de robot nooit zenuwachtig? Hij had stalen zenuwen.',
    'Waarom is een robot nooit bang in het donker? Hij heeft altijd wel een lampje aan.',
    'Wat doet een robot als hij verdrietig is? Hij laat een traantje olie.',
    'Waarom stak de kip de straat over? Om aan de overkant te komen.',
    'Wat is rood en gaat op en neer? Een tomaat in een lift.',
    'Wat zegt de ene muur tegen de andere muur? We zien elkaar wel op de hoek.',
    'Hoe noem je een robot die de hele dag rondjes rijdt? Een draaikont.',
]
# Spontaan babbelen: eerst een gewone beschrijving (betrouwbaar), daarna een grappige zin erover.
LOOK_AROUND = ('Je keek net even rond en zag dit: "{seen}" Zeg er in één korte, grappige of '
               'nieuwsgierige zin iets over tegen Jonas.')
CHECK_IN = ('Het is al even stil en je hebt Jonas niet gehoord. Zeg in één korte, grappige zin '
            'iets om te checken of hij er nog is, of stel hem een nieuwsgierige vraag.')
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
        'name': 'mop',
        'description': 'Haal een mop uit je moppenboekje. Gebruik dit altijd als Jonas een mop '
                       'of grap vraagt.',
        'parameters': {'type': 'object', 'properties': {}, 'required': []}}},
    {'type': 'function', 'function': {
        'name': 'beweeg',
        'description': 'Bewegen: dansen, een rondje draaien, rijden of iets brengen. Gebruik '
                       'dit als Jonas vraagt dat je beweegt.',
        'parameters': {'type': 'object', 'properties': {
            'wat': {'type': 'string', 'description': 'wat Jonas vraagt, bv. "rondje draaien"'}},
            'required': []}}},
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
SPONTANEOUS = object()  # in de wachtrij: Wave zegt uit zichzelf iets
LOG_LENGTH = 30      # berichten in /assistant/log voor de app
# Vraagt Jonas een mop, dan altijd uit het boekje, zonder het taalmodel: ook met een voorbeeld
# verzon de 4B bij "nog eentje" weer zelf onzinmoppen (30-10-2026).
JOKE_REQUEST = re.compile(r'\b(mop|moppen|mopje|grap|grapje|grappig|grappigs)\b', re.I)
MORE_REQUEST = re.compile(r'\bnog (eentje|een|één|ene)\b|\bnog\s*!?$', re.I)
AFTER_JOKE = ['Ken jij er nog eentje?', 'Nog eentje?', 'Haha, die is goed hé!', 'Snap je hem?']
# Bewegen: alleen draaien ter plaatse, geen vooruit/achteruit (geen obstakelcontrole hier).
# Stappen (vooruit m/s, draaien rad/s, seconden); samen hoogstens ±6 s.
MOVES = {
    'dans': [(0.0, 1.2, 0.35), (0.0, -1.2, 0.35)] * 4 + [(0.0, 2.0, 0.6), (0.0, -2.0, 0.6)],
    'rondje': [(0.0, 1.2, 2 * math.pi / 1.2)],
}
DANCE_REQUEST = re.compile(r'\b(dans|dansje|dansen|danst)\b', re.I)
SING_REQUEST = re.compile(r'\b(zing|zingen|liedje|liedjes|muziekje|deuntje)\b', re.I)
# Eigen muziek (scripts/add_music.sh): "speel <naam>" (begin van het nummer) of "dans op <naam>".
MUSIC_DIR = '/home/ubuntu/wave_rover/models/muziek'
PLAY_REQUEST = re.compile(r'\b(speel|muziek)\b', re.I)
MUSIC_SECONDS = 30
DANCE_ROUNDS = 3      # dansje zo vaak herhalen op muziek (3 x 4 s = 12 s)
SONG_NAMES = {'jacob': 'Vader Jacob', 'kortjakje': 'Altijd is Kortjakje ziek',
              'verjaardag': 'Happy Birthday', 'jarig': 'Happy Birthday', 'birthday': 'Happy Birthday'}
# Geluidjes tijdens het rijden: bij vertrekken na zoveel s stilstand, met deze kans, hoogstens
# om de zoveel s.
DRIVE_SOUND_STILL = 5.0
DRIVE_SOUND_CHANCE = 0.4
DRIVE_SOUND_GAP = 30.0
SPIN_REQUEST = re.compile(r'\b(rondje|ronddraaien|draai|draaien)\b', re.I)
MOVE_LINES = {
    'dans': ['Kijk eens naar mijn moves!', 'Muziek in mijn wielen!', 'Opzij, hier komt de '
             'dansrobot!'],
    'rondje': ['Wieeee, een rondje!', 'Daar ga ik, helemaal rond!'],
}
# Alles wat Piper niet kan uitspreken (emoji, opmaak) weg.
UNSPEAKABLE = re.compile(r'[*_#`~>|\[\]]|[\U0001F000-\U0001FAFF☀-➿️]')


def neutral_colors(image):
    """Paarse zweem van de camera weg, alleen voor het model (de app blijft zoals ze is).

    Gemeten 30-10-2026: een witte muur gaf R 192, G 180, B 193 (7% te weinig groen), waardoor
    het model een oranje bal "roze" noemde. Bijna-grijze, niet te donkere pixels (muren,
    plafond, tafelpoten) worden neutraal gezet; "grijze wereld" over het hele beeld maakte alles
    groenig. ±5 ms per beeld.
    """
    a = np.asarray(image, np.float32)
    lum = a.mean(axis=2)
    gray = (lum > 60) & ((a.max(axis=2) - a.min(axis=2)) / np.maximum(lum, 1) < 0.15)
    if gray.sum() < 500:
        return image  # weinig grijs in beeld: niets aan doen
    pixels = a[gray]
    gains = np.clip(pixels.mean() / pixels.mean(axis=0), 0.85, 1.2)
    return Image.fromarray(np.clip(a * gains, 0, 255).astype(np.uint8))


class Assistant(Node):

    def __init__(self):
        super().__init__('assistant')
        self.declare_parameter('server', 'http://127.0.0.1:8090')
        # Kleiner beeld = minder beeldtokens = sneller; 640 px is genoeg om te beschrijven.
        self.declare_parameter('image_width', 640)
        self.declare_parameter('piper', speech.PIPER)
        self.declare_parameter('voice', speech.VOICE)
        self.declare_parameter('speaker', speech.SPEAKER)
        # Stem: altijd Vlaams (nl_BE); afstelling zoals speech.STYLE.
        for name, value in speech.STYLE.items():
            self.declare_parameter(name, value)
        self.declare_parameter('chatter', False)
        self.declare_parameter('chatter_interval', 60.0)
        self.p = {n: self.get_parameter(n).value for n in (
            'server', 'image_width', 'piper', 'voice', 'speaker', 'chatter_interval')}
        self.chatter = self.get_parameter('chatter').value
        self.style = {n: self.get_parameter(n).value for n in speech.STYLE}

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(String, 'assistant/status', latched)
        self.log_pub = self.create_publisher(String, 'assistant/log', latched)
        self.create_subscription(String, 'assistant/say', self.on_say, 10)
        self.create_subscription(Empty, 'assistant/reset', self.on_reset, 10)
        self.create_subscription(Bool, 'assistant/chatter', self.on_chatter, 10)
        self.battery = {}
        self.create_subscription(BatteryState, 'ups/battery',
                                 lambda m: self.battery.__setitem__('Jetson', m), 10)
        self.create_subscription(BatteryState, 'chassis/battery',
                                 lambda m: self.battery.__setitem__('chassis', m), 10)
        self.cmd_pub = self.create_publisher(TwistStamped, 'cmd_vel_wave', 10)
        self.blocked = {'e_stop': False, 'charging_lock': False}
        for name in self.blocked:
            self.create_subscription(Bool, name, lambda m, n=name: self.blocked.__setitem__(n, m.data),
                                     latched)
        self.moving = threading.Lock()
        # Eén geluid tegelijk op de speaker (spreken, deuntje, geluidje tijdens het rijden).
        self.audio = threading.Lock()
        self.after = None     # ('move', naam) of ('song', titel): na het spreken uitvoeren
        self.declare_parameter('drive_sounds', True)
        self.drive_sounds = self.get_parameter('drive_sounds').value
        self.still_since = time.monotonic()
        self.driving = False
        self.last_drive_sound = 0.0
        self.create_subscription(TwistStamped, 'cmd_vel_safe', self.on_drive, 10)

        self.history = []    # [{'role': 'user'|'assistant', 'content': tekst}]
        self.log = []        # [{'wie': ..., 'tekst': ...}] voor de app
        self.game = None     # {'voorwerp': ..., 'kleur': ...} tijdens "ik zie, ik zie"
        self.recent = []     # wat Wave laatst spontaan zei (niet herhalen)
        self.told = []       # moppen die al verteld zijn
        self.last_move = None  # laatste beweging: "en nu op <nummer>" danst opnieuw
        self.last_was_joke = False
        self.quiet_since = time.monotonic()
        self.next_chatter = self.chatter_delay()
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
        self.create_timer(5.0, self.check_chatter)
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
            # De 2B kiest scene.launch.py als het geheugen krap is; praten lukt daarmee slecht
            # (geen mop, herhaalt zijn instructies, verzint wat hij ziet; getest 30-10-2026).
            try:
                with urllib.request.urlopen(self.p['server'] + '/v1/models', timeout=2) as r:
                    model = json.load(r)['data'][0]['id']
            except (urllib.error.URLError, OSError, KeyError, IndexError, ValueError):
                model = ''
            self.get_logger().info(f'Taalmodel: {model}')
            if '2B' in model:
                self.set_state('klaar', 'Klein taalmodel (2B) geladen: praten lukt slecht. Zet '
                                        'detection uit en scene opnieuw aan voor de 4B.')
            else:
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
        self.history, self.log, self.game, self.recent = [], [], None, []
        self.publish_log()
        self.publish_status('Nieuw gesprek')

    def on_chatter(self, msg):
        self.chatter = msg.data
        self.quiet_since = time.monotonic()
        self.publish_status('Spontaan babbelen ' + ('aan' if self.chatter else 'uit'))

    # --- spontaan babbelen ------------------------------------------------------------

    def chatter_delay(self):
        return self.p['chatter_interval'] * random.uniform(0.7, 1.3)

    def check_chatter(self):
        if (self.chatter and self.state == 'klaar' and self.inbox.empty()
                and time.monotonic() - self.quiet_since > self.next_chatter):
            self.next_chatter = self.chatter_delay()
            self.quiet_since = time.monotonic()
            try:
                self.inbox.put_nowait(SPONTANEOUS)
            except queue.Full:
                pass

    def spontaneous(self):
        """Iets uit zichzelf zeggen: meestal rondkijken, anders een teken van leven."""
        self.set_state('denken', 'Denkt…')
        avoid = ''
        if self.recent:
            avoid = ' Zeg niet opnieuw iets zoals: ' + ' / '.join(self.recent[-3:])
        seen = self.look('') if random.random() < 0.5 else ''
        if seen and not seen.startswith('Geen camerabeeld'):
            content = LOOK_AROUND.format(seen=seen) + avoid
        else:
            content = CHECK_IN + avoid
        reply = self.ask([{'role': 'system', 'content': PERSONA},
                          {'role': 'user', 'content': content}], max_tokens=60)
        line = (reply.get('content') or '').strip().strip('"')
        if line:
            self.recent = (self.recent + [line])[-5:]
            self.history.append({'role': 'assistant', 'content': line})
        return line

    # --- gesprek (werker-thread) ------------------------------------------------------

    def work(self):
        while rclpy.ok():
            text = self.inbox.get()
            while self.state == 'laden':
                time.sleep(0.5)
            if text is not SPONTANEOUS:
                self.add_log('jij', text)
            try:
                answer = self.spontaneous() if text is SPONTANEOUS else self.chat(text)
                if not answer:
                    self.set_state('klaar', 'Klaar')
                    continue
            except Exception as e:  # noqa: BLE001 - elke fout melden, de node blijft draaien
                self.get_logger().error(f'Antwoorden mislukt: {e}')
                self.set_state('fout', f'Antwoorden mislukt: {e}')
                self.state = 'klaar'  # een volgende vraag mag gewoon opnieuw
                continue
            self.add_log('wave', answer)
            self.set_state('spreken', 'Spreekt…')
            with self.audio:
                try:
                    speech.say(UNSPEAKABLE.sub('', answer), self.p['voice'], self.p['speaker'],
                               self.p['piper'], self.style)
                except (OSError, RuntimeError) as e:
                    self.get_logger().warn(f'Speaker: {e}')
                self.do_after()
            self.quiet_since = time.monotonic()
            self.set_state('klaar', 'Klaar')

    def chat(self, text):
        start = time.monotonic()
        self.set_state('denken', 'Denkt…')
        self.history.append({'role': 'user', 'content': text})
        move = 'dans' if DANCE_REQUEST.search(text) else 'rondje' if SPIN_REQUEST.search(text) else None
        song = self.find_music(text)
        if song and not move and not PLAY_REQUEST.search(text) and self.last_move == 'dans':
            move = 'dans'  # "en nu op <nummer>" vlak na een dansje
        elif song and not move:
            text = f'speel {text}'  # een nummer noemen = afspelen
        self.last_move = move
        if move == 'dans' and song:
            title = os.path.basename(song)[:-4].replace('-', ' ')
            if self.blocked['e_stop'] or self.blocked['charging_lock']:
                # Niet bewegen mag (noodstop of laadkabel): dan alleen de muziek.
                self.after = ('muziek', song)
                reason = 'de noodstop staat aan' if self.blocked['e_stop'] else 'ik zit aan de lader vast'
                answer = f'Dansen gaat niet, want {reason}. Ik speel {title} wel even!'
            else:
                self.after = ('dans_muziek', song)
                answer = f'{random.choice(MOVE_LINES["dans"])} Op {title}!'
            self.history.append({'role': 'assistant', 'content': answer})
            return answer
        if not move and PLAY_REQUEST.search(text):
            names = sorted(self.music())
            if song:
                self.after = ('muziek', song)
                answer = f'Daar gaan we: {os.path.basename(song)[:-4].replace("-", " ")}!'
            elif names:
                answer = f'Dat nummer ken ik niet. Ik heb: {", ".join(names)}.'
            else:
                answer = None  # geen eigen muziek: dan een piepliedje (hieronder)
            if answer:
                self.history.append({'role': 'assistant', 'content': answer})
                return answer
        if not move and (SING_REQUEST.search(text) or PLAY_REQUEST.search(text)):
            lower = text.lower()
            title = next((t for k, t in SONG_NAMES.items() if k in lower),
                         random.choice(list(sounds.SONGS)))
            self.after = ('song', title)
            answer = f'Zingen kan ik niet, maar piepen wel! Hier komt {title}.'
            self.history.append({'role': 'assistant', 'content': answer})
            return answer
        if move:
            # Zeker herkennen in plaats van het model te laten kiezen (zoals bij moppen).
            answer = self.start_move(move)
            self.history.append({'role': 'assistant', 'content': answer})
            self.get_logger().info(f'"{text}" -> {move}: "{answer}"')
            return answer
        wants_joke = bool(JOKE_REQUEST.search(text)) or (
            self.last_was_joke and bool(MORE_REQUEST.search(text)))
        self.last_was_joke = wants_joke
        if wants_joke:
            answer = f'{self.pick_joke()} {random.choice(AFTER_JOKE)}'
            self.history.append({'role': 'assistant', 'content': answer})
            self.get_logger().info(f'"{text}" -> mop: "{answer}"')
            return answer
        system = PERSONA
        if self.game:
            system += (f'\nJullie spelen "ik zie, ik zie wat jij niet ziet". Het geheime '
                       f'voorwerp is: {self.game["voorwerp"]} (kleur: {self.game["kleur"]}). '
                       f'Verklap het niet. Zeg of een gok juist is en geef een kleine hint als '
                       f'Jonas erom vraagt. Is het geraden of geeft hij op, gebruik dan '
                       f'stop_ik_zie_spel.')
        messages = ([{'role': 'system', 'content': system}] + EXAMPLE_MESSAGES
                    + self.history[-MAX_HISTORY:])
        answer = self.drop_repeated(self.answer(messages, text))
        if self.repeats(answer):
            # Zelfde antwoord als eerder: één keer opnieuw vragen.
            answer = self.answer(messages + [{'role': 'system', 'content':
                                              'Zeg iets nieuws; herhaal jezelf niet.'}], text)
        answer = answer or 'Sorry, daar weet ik even geen antwoord op.'
        self.history.append({'role': 'assistant', 'content': answer})
        self.get_logger().info(f'"{text}" -> "{answer}" ({time.monotonic() - start:.1f} s)')
        return answer

    def answer(self, messages, text):
        """Vragen tot er een antwoord is (met tussendoor tools), opgeschoond."""
        messages = list(messages)
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
        return self.tidy(answer)

    @staticmethod
    def tidy(text):
        """Herhaalde zinnen eruit (kleine modellen lopen soms in een lus) en hoogstens
        MAX_SENTENCES zinnen."""
        seen, kept = set(), []
        for sentence in re.findall(r'[^.!?]+[.!?]*', text):
            key = sentence.strip().lower()
            if key and key not in seen:
                seen.add(key)
                kept.append(sentence.strip())
        return ' '.join(kept[:MAX_SENTENCES])

    # --- bewegen ---------------------------------------------------------------------

    def start_move(self, name):
        """Beweging starten (tegelijk met het spreken) en de zin teruggeven die Wave zegt."""
        if self.blocked['e_stop']:
            return 'Ik zou graag, maar de noodstop staat aan!'
        if self.blocked['charging_lock']:
            return 'Ik zit aan de lader vast, dus dansen gaat even niet.'
        if self.moving.locked():
            return 'Wacht even, ik ben al aan het bewegen!'
        self.after = ('move', name)  # na het spreken, op een deuntje
        return random.choice(MOVE_LINES[name])

    def do_after(self):
        """Na het spreken (met de speaker nog in handen): dansen op muziek of een liedje."""
        action, self.after = self.after, None
        if action is None:
            return
        kind, name = action
        try:
            if kind == 'muziek':
                sounds.play_file(name, MUSIC_SECONDS, self.p['speaker'])
            elif kind == 'dans_muziek' and self.moving.acquire(blocking=False):
                threading.Thread(target=self.run_move, args=('dans', DANCE_ROUNDS),
                                 daemon=True).start()
                seconds = DANCE_ROUNDS * sum(t for _, _, t in MOVES['dans'])
                sounds.play_file(name, seconds, self.p['speaker'])
            elif kind == 'move' and self.moving.acquire(blocking=False):
                threading.Thread(target=self.run_move, args=(name,), daemon=True).start()
                sounds.play(sounds.melody(sounds.DANCE_TUNE, beat=0.5) if name == 'dans'
                            else sounds.effect('wiew'), self.p['speaker'])
            elif kind == 'song':
                sounds.play(sounds.melody(sounds.SONGS[name]), self.p['speaker'])
        except (OSError, RuntimeError) as e:
            self.get_logger().warn(f'Speaker: {e}')

    # --- geluidjes tijdens het rijden -------------------------------------------------

    def on_drive(self, msg):
        moving = abs(msg.twist.linear.x) > 0.02 or abs(msg.twist.angular.z) > 0.1
        now = time.monotonic()
        if moving and not self.driving:
            if (self.drive_sounds and now - self.still_since > DRIVE_SOUND_STILL
                    and now - self.last_drive_sound > DRIVE_SOUND_GAP
                    and random.random() < DRIVE_SOUND_CHANCE and not self.moving.locked()):
                self.last_drive_sound = now
                threading.Thread(target=self.drive_sound, daemon=True).start()
        if not moving and self.driving:
            self.still_since = now
        self.driving = moving

    def drive_sound(self):
        if not self.audio.acquire(blocking=False):
            return  # Wave praat net: dan geen geluidje
        try:
            sounds.play(sounds.effect(random.choice(['wiew', 'wiew', 'piep', 'boing'])),
                        self.p['speaker'])
        except (OSError, RuntimeError) as e:
            self.get_logger().warn(f'Speaker: {e}')
        finally:
            self.audio.release()

    def music(self):
        """Eigen nummers: {naam met spaties: pad}."""
        return {os.path.basename(p)[:-4].replace('-', ' '): p
                for p in glob.glob(os.path.join(MUSIC_DIR, '*.wav'))}

    @staticmethod
    def plain_words(text):
        plain = unicodedata.normalize('NFKD', text.lower()).encode('ascii', 'ignore').decode()
        return re.findall(r'[a-z0-9]+', plain)

    def find_music(self, text):
        """Het eigen nummer waarvan de naam of een bijnaam (<naam>.namen, bv. "lamine yamal")
        helemaal in de vraag staat; de langste naam wint."""
        words = set(self.plain_words(text))
        hits = []
        for name, path in self.music().items():
            names = [name]
            try:
                with open(path[:-4] + '.namen') as f:
                    names += [line.strip() for line in f if line.strip()]
            except OSError:
                pass
            hits += [(len(n), path) for n in names
                     if self.plain_words(n) and set(self.plain_words(n)) <= words]
        return max(hits)[1] if hits else None

    def run_move(self, name, rounds=1):
        try:
            for vx, wz, seconds in MOVES[name] * rounds:
                end = time.monotonic() + seconds
                while time.monotonic() < end:
                    if self.blocked['e_stop'] or self.blocked['charging_lock']:
                        return
                    self.send_cmd(vx, wz)
                    time.sleep(0.05)
        finally:
            for _ in range(3):
                self.send_cmd(0.0, 0.0)  # expliciet stoppen
                time.sleep(0.05)
            self.moving.release()

    def send_cmd(self, vx, wz):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = vx
        msg.twist.angular.z = wz
        self.cmd_pub.publish(msg)

    def pick_joke(self):
        """Een mop die de laatste tijd niet verteld is."""
        joke = random.choice([j for j in JOKES if j not in self.told] or JOKES)
        self.told = (self.told + [joke])[-len(JOKES) // 2:]
        return joke

    def drop_repeated(self, answer):
        """Zinnen weglaten die hij in de laatste antwoorden al zei (de 4B plakte anders aan elk
        antwoord "En ja, dat is een beetje grappig, maar ook echt!")."""
        said = set()
        for m in [m for m in self.history if m['role'] == 'assistant'][-4:]:
            said.update(x.strip().lower() for x in re.findall(r'[^.!?]+[.!?]*', m['content']))
        kept = [x.strip() for x in re.findall(r'[^.!?]+[.!?]*', answer)
                if x.strip() and x.strip().lower() not in said]
        return ' '.join(kept) or answer

    def repeats(self, answer):
        previous = {m['content'].strip().lower() for m in self.history
                    if m['role'] == 'assistant'}
        return bool(answer) and answer.strip().lower() in previous

    def run_tool(self, name, question):
        if name == 'kijk':
            return self.look(question)
        if name == 'batterij':
            return self.battery_text()
        if name == 'uur':
            t = time.localtime()
            return (f'Het is {t.tm_hour}u{t.tm_min:02d} op {DAYS[t.tm_wday]} {t.tm_mday} '
                    f'{MONTHS[t.tm_mon - 1]} {t.tm_year}.')
        if name == 'mop':
            return f'Vertel deze mop letterlijk, zonder iets te veranderen: {self.pick_joke()}'
        if name == 'beweeg':
            wish = question.lower()
            move = 'dans' if 'dans' in wish else 'rondje' if ('rond' in wish or 'draai' in wish) else None
            if move:
                return f'Gedaan. Zeg dit: {self.start_move(move)}'
            return ('Dat kan nog niet: ik kan alleen dansen of een rondje draaien, rijden naar een '
                    'plek komt later. Zeg dat eerlijk en grappig.')
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
                parts.append(f'{name}: draait gewoon op de adapter, er zitten geen batterijen in')
            else:
                charging = ', laadt' if m.power_supply_status == BatteryState.POWER_SUPPLY_STATUS_CHARGING else ''
                volts = f'{m.voltage:.2f}'.replace('.', ',')
                parts.append(f'{name}: {round(m.percentage * 100)}% ({volts} volt{charging})')
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
        neutral_colors(image.convert('RGB')).save(out, 'JPEG', quality=85)
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
        data = {'state': self.state, 'text': text, 'chatter': self.chatter}
        self.status_pub.publish(String(data=json.dumps(data)))


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
