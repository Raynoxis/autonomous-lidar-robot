#!/usr/bin/env python3
"""Simple HTTP server for serving the Vite build + proxy /api -> ros_api (8083)."""
import http.server
import socketserver
import os
import http.client
import json
from pathlib import Path

PORT = 8082
DIRECTORY = "/app/web/dist"
API_BACKEND_PORT = 8083
API_PREFIX = "/api"

class SPAHTTPRequestHandler(http.server.SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=DIRECTORY, **kwargs)

    def end_headers(self):
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')
        super().end_headers()

    def _proxy_to_api(self):
        """Proxy API calls to the backend ros_api server (8083)."""
        try:
            content_length = int(self.headers.get('Content-Length', 0))
            body = self.rfile.read(content_length) if content_length > 0 else None

            # Strip API prefix to target backend path
            target_path = self.path[len(API_PREFIX):] or '/'

            conn = http.client.HTTPConnection('127.0.0.1', API_BACKEND_PORT, timeout=5)
            conn.request(self.command, target_path, body=body, headers=dict(self.headers))
            resp = conn.getresponse()
            data = resp.read()

            self.send_response(resp.status)
            for header, value in resp.getheaders():
                if header.lower() in (
                    'connection',
                    'keep-alive',
                    'proxy-authenticate',
                    'proxy-authorization',
                    'te',
                    'trailers',
                    'transfer-encoding',
                    'upgrade',
                ):
                    continue
                self.send_header(header, value)
            self.end_headers()
            self.wfile.write(data)
        except Exception as e:
            self.send_response(502)
            self.end_headers()
            self.wfile.write(json.dumps({"success": False, "message": f"Proxy error: {str(e)}"}).encode())

    def do_OPTIONS(self):
        if self.path.startswith(API_PREFIX):
            self._proxy_to_api()
            return
        return super().do_OPTIONS()

    def do_POST(self):
        if self.path.startswith(API_PREFIX):
            self._proxy_to_api()
            return
        return super().do_POST()

    def do_GET(self):
        if self.path.startswith(API_PREFIX):
            self._proxy_to_api()
            return

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
