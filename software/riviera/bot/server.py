#!/usr/bin/env python3
"""
License: MIT License
Copyright (c) 2023 Miel Donkers
Very simple HTTP server in python for logging requests
Usage::
    ./server.py [<port>]
"""

from http.server import BaseHTTPRequestHandler, HTTPServer
import logging
import json
import numpy

class CoccinellesServer(BaseHTTPRequestHandler):
    def _set_response(self):
        self.send_response(200)
        self.send_header('Content-type', 'text/html')
        self.end_headers()

    def do_GET(self):
        logging.info("GET request,\nPath: %s\nHeaders:\n%s\n", str(self.path), str(self.headers))
        self._set_response()
        if self.path.startswith("/get_plant"):
            self.wfile.write(json.dumps(numpy.random.rand(36,2).tolist()).encode('utf-8'))
        if self.path.startswith("/get_time"):
            self.wfile.write(json.dumps(numpy.random.rand(1).tolist()).encode('utf-8'))

def run(server_class=HTTPServer, handler_class=CoccinellesServer, port=8080):
    logging.basicConfig(level=logging.INFO)
    server_address = ('', port)
    httpd = server_class(server_address, handler_class)
    logging.info('Starting httpd...\n')
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass
    httpd.server_close()
    logging.info('Stopping httpd...\n')
