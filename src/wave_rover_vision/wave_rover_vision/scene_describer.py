"""Vertelt op vraag wat de camera ziet, in het Nederlands, via de USB-speaker (module scene).

1. /scene/describe (std_msgs/String) vanuit het dashboard: een vraag over het beeld, of leeg
   voor "wat zie je?".
2. Neemt één beeld van /camera/image_low/compressed (alleen op dat moment geabonneerd, dus
   niets kost CPU zolang niemand iets vraagt), verkleint het naar `image_width` px breed en
   stuurt het naar llama-server: een beeldtaalmodel op de GPU, gestart door scene.launch.py.
3. Spreekt het antwoord uit met Piper (Vlaamse stem) en aplay op de USB-speaker.
Status en antwoord op /scene/status (std_msgs/String, JSON, latched):
  {"state": "laden|klaar|kijken|denken|spreken|fout", "text": "...", "seconds": 4.2}
"""
import base64
import io
import json
import subprocess
import threading
import time
import urllib.error
import urllib.request

import rclpy
from PIL import Image
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

# Getest op een camerabeeld (29-10-2026): zonder voorbeeld en met lage temperature viel
# Qwen3-VL-2B in herhaallussen ("een kast, een wastafel, een kast, ...") of gaf het alleen
# losse woorden; met een voorbeeldzin en de sampling die Qwen aanraadt (QWEN_SAMPLING) twee
# gewone zinnen in 1-2 s.
SYSTEM_PROMPT = (
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


class SceneDescriber(Node):

    def __init__(self):
        super().__init__('scene_describer')
        self.declare_parameter('server', 'http://127.0.0.1:8090')
        # Kleiner beeld = minder beeldtokens = sneller; 640 px is genoeg voor een beschrijving.
        self.declare_parameter('image_width', 640)
        self.declare_parameter('max_tokens', 100)
        self.declare_parameter('piper', '/opt/piper/piper')
        self.declare_parameter('voice', '/home/ubuntu/wave_rover/models/piper/nl_BE-nathalie-medium.onnx')
        # ALSA-naam van de USB-speaker ("USB2.0 Device" -> kaart Device).
        self.declare_parameter('speaker', 'plughw:CARD=Device,DEV=0')
        self.p = {n: self.get_parameter(n).value for n in (
            'server', 'image_width', 'max_tokens', 'piper', 'voice', 'speaker')}

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(String, 'scene/status', latched)
        self.create_subscription(String, 'scene/describe', self.on_describe, 10)

        self.state = 'laden'
        self.question = ''
        self.image_sub = None
        self.image_timer = None
        self.publish_status('Taalmodel laden…')
        self.ready_timer = self.create_timer(1.0, self.check_ready)

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
            self.state = 'klaar'
            self.publish_status('Klaar: vraag wat ik zie')

    # --- vraag ------------------------------------------------------------------------

    def on_describe(self, msg):
        if self.state != 'klaar':
            return  # nog aan het laden of al bezig; het dashboard toont de status
        self.question = msg.data.strip() or DEFAULT_QUESTION
        self.state = 'kijken'
        self.publish_status('Kijken…')
        self.image_sub = self.create_subscription(
            CompressedImage, 'camera/image_low/compressed', self.on_image, qos_profile_sensor_data)
        self.image_timer = self.create_timer(5.0, self.no_image)

    def no_image(self):
        self.stop_image()
        self.done('fout', 'Geen camerabeeld: staat de module camera aan?')

    def stop_image(self):
        if self.image_sub is not None:
            self.destroy_subscription(self.image_sub)
            self.image_sub = None
        if self.image_timer is not None:
            self.image_timer.cancel()
            self.destroy_timer(self.image_timer)
            self.image_timer = None

    def on_image(self, msg):
        if self.image_sub is None:
            return  # nog een beeld dat al onderweg was
        self.stop_image()
        # Het taalmodel denkt seconden lang: niet in de ROS-thread.
        threading.Thread(target=self.describe, args=(bytes(msg.data),), daemon=True).start()

    def describe(self, jpeg):
        start = time.monotonic()
        try:
            self.state = 'denken'
            self.publish_status('Denken…')
            text = self.ask_model(self.small_jpeg(jpeg))
            seconds = time.monotonic() - start
            self.get_logger().info(f'Beschrijving ({seconds:.1f} s): {text}')
            self.state = 'spreken'
            self.publish_status(text, seconds)
            self.speak(text)
            self.done('klaar', text, seconds)
        except Exception as e:  # noqa: BLE001 - elke fout melden, de module blijft draaien
            self.get_logger().error(f'Beschrijven mislukt: {e}')
            self.done('fout', f'Beschrijven mislukt: {e}')

    def small_jpeg(self, jpeg):
        image = Image.open(io.BytesIO(jpeg))
        width = self.p['image_width']
        if image.width > width:
            image = image.resize((width, round(image.height * width / image.width)))
        out = io.BytesIO()
        image.convert('RGB').save(out, 'JPEG', quality=85)
        return out.getvalue()

    def ask_model(self, jpeg):
        image_url = 'data:image/jpeg;base64,' + base64.b64encode(jpeg).decode()
        body = {
            'messages': [
                {'role': 'system', 'content': SYSTEM_PROMPT},
                {'role': 'user', 'content': [
                    {'type': 'image_url', 'image_url': {'url': image_url}},
                    {'type': 'text', 'text': self.question},
                ]},
            ],
            'max_tokens': self.p['max_tokens'],
            **QWEN_SAMPLING,
        }
        request = urllib.request.Request(
            self.p['server'] + '/v1/chat/completions', data=json.dumps(body).encode(),
            headers={'Content-Type': 'application/json'})
        with urllib.request.urlopen(request, timeout=120) as r:
            choice = json.load(r)['choices'][0]
        text = choice['message']['content'].strip()
        if choice.get('finish_reason') == 'length':
            # Afgebroken door max_tokens: alleen de volledige zinnen uitspreken.
            end = max(text.rfind(c) for c in '.!?')
            text = text[:end + 1] if end > 0 else ' '.join(text.split()[:30]) + '.'
        return text

    def speak(self, text):
        """Piper maakt ruwe audio (16 bit mono), aplay speelt ze meteen af."""
        with open(self.p['voice'] + '.json') as f:
            rate = json.load(f)['audio']['sample_rate']
        piper = subprocess.Popen(
            [self.p['piper'], '--model', self.p['voice'], '--output-raw'],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
        aplay = subprocess.Popen(
            ['aplay', '-q', '-D', self.p['speaker'], '-r', str(rate), '-f', 'S16_LE', '-c', '1',
             '-t', 'raw'], stdin=piper.stdout, stderr=subprocess.PIPE)
        piper.stdout.close()  # aplay is nu de enige lezer
        piper.stdin.write(text.replace('\n', ' ').encode() + b'\n')
        piper.stdin.close()
        _, err = aplay.communicate(timeout=120)
        piper.wait(timeout=10)
        if aplay.returncode != 0:
            raise RuntimeError(f'speaker: {err.decode().strip() or aplay.returncode}')

    def done(self, state, text, seconds=None):
        self.state = 'klaar' if state == 'klaar' else 'fout'
        self.publish_status(text, seconds)
        if self.state == 'fout':
            self.state = 'klaar'  # een volgende vraag mag gewoon opnieuw

    def publish_status(self, text, seconds=None):
        data = {'state': self.state, 'text': text}
        if seconds is not None:
            data['seconds'] = round(seconds, 1)
        self.status_pub.publish(String(data=json.dumps(data)))


def main(args=None):
    rclpy.init(args=args)
    node = SceneDescriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
