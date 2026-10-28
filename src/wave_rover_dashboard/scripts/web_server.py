"""Statische webserver voor het dashboard.

Zoals `python3 -m http.server`, maar met `Cache-Control: no-cache`: Safari vraagt dan bij
elk bezoek na of een bestand gewijzigd is (meestal een kort "304 ongewijzigd"), zodat een
nieuwe versie van het dashboard meteen gebruikt wordt.
"""
import functools
import sys
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer


class NoCacheHandler(SimpleHTTPRequestHandler):

    def end_headers(self):
        self.send_header('Cache-Control', 'no-cache')
        super().end_headers()

    def log_message(self, *args):
        pass  # elke aanvraag loggen vult enkel de log


def main():
    port, directory = int(sys.argv[1]), sys.argv[2]
    handler = functools.partial(NoCacheHandler, directory=directory)
    ThreadingHTTPServer(('', port), handler).serve_forever()


if __name__ == '__main__':
    main()
