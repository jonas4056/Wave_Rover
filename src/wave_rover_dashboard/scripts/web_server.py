"""Statische webserver voor het dashboard.

Zoals `python3 -m http.server`, met twee aanpassingen zodat de iPhone altijd de nieuwste
versie gebruikt (een webapp op het beginscherm hield anders uren een oude `map.js` vast):
- `index.html` wordt telkens opnieuw opgebouwd: achter elk script en stylesheet komt
  `?v=<versie>` (de laatste wijziging van de bestanden), en de versie staat in een
  `<meta name="dashboard-version">`. Een gewijzigd bestand heeft dus een nieuwe URL.
- `/version` geeft de huidige versie; app.js herlaadt de pagina als die veranderd is.
De rest krijgt `Cache-Control: no-cache` (meestal een kort "304 ongewijzigd").
"""
import functools
import os
import re
import sys
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer

ASSET = re.compile(r'(src|href)="([\w.-]+\.(?:js|css))"')


class DashboardHandler(SimpleHTTPRequestHandler):

    cache_control = 'no-cache'

    def do_GET(self):
        path = self.path.split('?', 1)[0]
        if path in ('/', '/index.html'):
            self.send_generated(self.index_html(), 'text/html; charset=utf-8')
        elif path == '/version':
            self.send_generated(self.version(), 'text/plain')
        else:
            super().do_GET()

    def version(self):
        return str(max(int(e.stat().st_mtime) for e in os.scandir(self.directory)
                       if e.is_file()))

    def index_html(self):
        version = self.version()
        with open(os.path.join(self.directory, 'index.html'), encoding='utf-8') as f:
            html = f.read()
        html = ASSET.sub(lambda m: f'{m[1]}="{m[2]}?v={version}"', html)
        return html.replace(
            '</head>', f'  <meta name="dashboard-version" content="{version}">\n</head>', 1)

    def send_generated(self, text, content_type):
        body = text.encode('utf-8')
        self.cache_control = 'no-store'
        self.send_response(200)
        self.send_header('Content-Type', content_type)
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def end_headers(self):
        self.send_header('Cache-Control', self.cache_control)
        super().end_headers()

    def log_message(self, *args):
        pass  # elke aanvraag loggen vult enkel de log


def main():
    port, directory = int(sys.argv[1]), sys.argv[2]
    handler = functools.partial(DashboardHandler, directory=directory)
    ThreadingHTTPServer(('', port), handler).serve_forever()


if __name__ == '__main__':
    main()
