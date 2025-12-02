#!/usr/bin/env python3
"""Simple HTTP server for serving the web interface"""
import http.server
import socketserver
import os

PORT = 8082
DIRECTORY = "/app/web"

class MyHTTPRequestHandler(http.server.SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=DIRECTORY, **kwargs)

    def end_headers(self):
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')
        super().end_headers()

if __name__ == '__main__':
    with socketserver.TCPServer(("", PORT), MyHTTPRequestHandler) as httpd:
        print(f"Serving HTTP on port {PORT} from {DIRECTORY}")
        print(f"Access at: http://0.0.0.0:{PORT}/")
        httpd.serve_forever()
