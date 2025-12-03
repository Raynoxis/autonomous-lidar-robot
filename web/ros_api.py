#!/usr/bin/env python3
"""
ROS2 Control API Server
Allows web interface to start/stop ROS2 nodes and launch files
"""
import http.server
import socketserver
import json
import subprocess
import threading
import os
import signal
from urllib.parse import urlparse, parse_qs

PORT = 8083

# Store running processes
running_processes = {}

class ROS2APIHandler(http.server.BaseHTTPRequestHandler):
    def _set_headers(self, status=200):
        self.send_response(status)
        self.send_header('Content-type', 'application/json')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')
        self.end_headers()

    def do_OPTIONS(self):
        self._set_headers()

    def do_POST(self):
        content_length = int(self.headers['Content-Length'])
        post_data = self.rfile.read(content_length)
        data = json.loads(post_data.decode('utf-8'))

        action = data.get('action')
        response = {'success': False, 'message': 'Unknown action'}

        try:
            if action == 'start_explore':
                response = self.start_explore()
            elif action == 'stop_explore':
                response = self.stop_explore()
            elif action == 'check_process':
                process_name = data.get('process_name')
                response = self.check_process(process_name)
            elif action == 'list_nodes':
                response = self.list_nodes()
            else:
                response = {'success': False, 'message': f'Unknown action: {action}'}
        except Exception as e:
            response = {'success': False, 'message': str(e)}

        self._set_headers()
        self.wfile.write(json.dumps(response).encode())

    def do_GET(self):
        parsed_path = urlparse(self.path)
        query = parse_qs(parsed_path.query)

        response = {'success': False, 'message': 'Unknown endpoint'}

        try:
            if parsed_path.path == '/status':
                response = self.get_status()
            elif parsed_path.path == '/processes':
                response = self.get_processes()
            else:
                response = {'success': False, 'message': f'Unknown path: {parsed_path.path}'}
        except Exception as e:
            response = {'success': False, 'message': str(e)}

        self._set_headers()
        self.wfile.write(json.dumps(response).encode())

    def start_explore(self):
        """Start explore_lite launch file"""
        if 'explore' in running_processes and running_processes['explore'].poll() is None:
            return {'success': False, 'message': 'Exploration already running'}

        try:
            # Source ROS2 and launch explore_lite
            cmd = [
                'bash', '-c',
                'source /opt/ros/iron/setup.bash && '
                'source /app/install/setup.bash && '
                'ros2 launch explore_lite explore.launch.py'
            ]

            process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid
            )

            running_processes['explore'] = process

            return {
                'success': True,
                'message': 'Exploration started',
                'pid': process.pid
            }
        except Exception as e:
            return {'success': False, 'message': f'Failed to start exploration: {str(e)}'}

    def stop_explore(self):
        """Stop explore_lite process"""
        if 'explore' not in running_processes:
            return {'success': False, 'message': 'Exploration not running'}

        try:
            process = running_processes['explore']
            if process.poll() is None:
                # Kill process group to kill all child processes
                os.killpg(os.getpgid(process.pid), signal.SIGTERM)
                process.wait(timeout=5)

            del running_processes['explore']

            return {'success': True, 'message': 'Exploration stopped'}
        except Exception as e:
            return {'success': False, 'message': f'Failed to stop exploration: {str(e)}'}

    def check_process(self, process_name):
        """Check if a process is running"""
        if process_name in running_processes:
            process = running_processes[process_name]
            is_running = process.poll() is None
            return {
                'success': True,
                'running': is_running,
                'pid': process.pid if is_running else None
            }
        return {'success': True, 'running': False}

    def list_nodes(self):
        """List all ROS2 nodes"""
        try:
            result = subprocess.run(
                ['bash', '-c', 'source /opt/ros/iron/setup.bash && ros2 node list'],
                capture_output=True,
                text=True,
                timeout=5
            )
            nodes = result.stdout.strip().split('\n')
            return {'success': True, 'nodes': nodes}
        except Exception as e:
            return {'success': False, 'message': str(e)}

    def get_status(self):
        """Get overall status"""
        return {
            'success': True,
            'running_processes': {
                name: proc.poll() is None
                for name, proc in running_processes.items()
            }
        }

    def get_processes(self):
        """Get list of managed processes"""
        return {
            'success': True,
            'processes': list(running_processes.keys())
        }

    def log_message(self, format, *args):
        """Custom log format"""
        print(f"[ROS2 API] {format % args}")


def run_server():
    with socketserver.TCPServer(("", PORT), ROS2APIHandler) as httpd:
        print(f"ROS2 Control API Server running on port {PORT}")
        print(f"Endpoints:")
        print(f"  POST /  - Execute actions (start_explore, stop_explore, etc.)")
        print(f"  GET /status - Get status of running processes")
        print(f"  GET /processes - List managed processes")
        httpd.serve_forever()


if __name__ == '__main__':
    run_server()
