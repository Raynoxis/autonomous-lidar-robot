#!/usr/bin/env python3
"""Simple HTTP server for serving the Vite build"""
import http.server
import socketserver
import os
from pathlib import Path

PORT = 8082
DIRECTORY = "/app/web/dist"

class SPAHTTPRequestHandler(http.server.SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=DIRECTORY, **kwargs)

    def end_headers(self):
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')
        super().end_headers()

    def do_GET(self):
        # Serve index.html for all routes (SPA routing)
        path = self.translate_path(self.path)

        # If the file doesn't exist and it's not a static asset, serve index.html
        if not os.path.exists(path) and not self.path.startswith('/assets/'):
            self.path = '/index.html'

        return super().do_GET()

if __name__ == '__main__':
    # Check if dist directory exists
    if not os.path.exists(DIRECTORY):
        print(f"ERROR: Directory {DIRECTORY} does not exist!")
        print("Please run 'npm run build' first to create the dist directory.")
        exit(1)

    with socketserver.TCPServer(("", PORT), SPAHTTPRequestHandler) as httpd:
        print(f"Serving Vite build on port {PORT} from {DIRECTORY}")
        print(f"Access at: http://0.0.0.0:{PORT}/")
        httpd.serve_forever()
