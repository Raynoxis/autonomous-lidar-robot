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
EXPLORE_PID_FILE = "/tmp/explore_node.pid"
EXPLORE_LOG = "/app/logs/explore.log"

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
        # Handle requests with or without body
        content_length = int(self.headers.get('Content-Length', 0))

        if content_length > 0:
            post_data = self.rfile.read(content_length)
            try:
                data = json.loads(post_data.decode('utf-8'))
            except json.JSONDecodeError:
                self._set_headers(400)
                self.wfile.write(json.dumps({'success': False, 'message': 'Invalid JSON'}).encode())
                return
        else:
            data = {}

        # Support both action-based and REST endpoints
        parsed_path = urlparse(self.path)
        action = data.get('action')
        response = {'success': False, 'message': 'Unknown action'}

        try:
            # REST-style endpoints
            if parsed_path.path == '/api/explore/start':
                response = self.start_explore()
            elif parsed_path.path == '/api/explore/stop':
                response = self.stop_explore()
            elif parsed_path.path == '/api/nav/goal':
                x = float(data.get('x', 0.0))
                y = float(data.get('y', 0.0))
                theta = float(data.get('theta', 0.0))
                response = self.nav_goal(x, y, theta)
            elif parsed_path.path == '/api/nav/cancel':
                response = self.nav_cancel()
            elif parsed_path.path == '/api/map/save':
                map_name = data.get('map_name') or 'map'
                response = self.save_map(map_name)
            elif parsed_path.path == '/api/map/load':
                map_name = data.get('map_name') or 'map'
                response = self.load_map(map_name)
            elif parsed_path.path == '/api/map/clear':
                response = self.clear_map()
            # Action-based (legacy support)
            elif action == 'start_explore':
                response = self.start_explore()
            elif action == 'stop_explore':
                response = self.stop_explore()
            elif action == 'nav_goal':
                response = self.nav_goal(float(data.get('x', 0.0)), float(data.get('y', 0.0)), float(data.get('theta', 0.0)))
            elif action == 'nav_cancel':
                response = self.nav_cancel()
            elif action == 'save_map':
                map_name = data.get('map_name') or 'map'
                response = self.save_map(map_name)
            elif action == 'load_map':
                map_name = data.get('map_name') or 'map'
                response = self.load_map(map_name)
            elif action == 'clear_map':
                response = self.clear_map()
            elif action == 'check_process':
                process_name = data.get('process_name')
                response = self.check_process(process_name)
            elif action == 'list_nodes':
                response = self.list_nodes()
            else:
                response = {'success': False, 'message': f'Unknown action or endpoint: {action or parsed_path.path}'}
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
            elif parsed_path.path == '/explore/status':
                response = self.explore_status()
            else:
                response = {'success': False, 'message': f'Unknown path: {parsed_path.path}'}
        except Exception as e:
            response = {'success': False, 'message': str(e)}

        self._set_headers()
        self.wfile.write(json.dumps(response).encode())

    def start_explore(self):
        """Start explore_lite avec paramètres officiels KaiAI"""
        if 'explore' in running_processes and running_processes['explore'].poll() is None:
            return {'success': False, 'message': 'Exploration already running'}

        try:
            # Source ROS2 and launch explore_lite avec paramètres officiels
            cmd = [
                'bash', '-c',
                'source /opt/ros/iron/setup.bash && '
                'source /app/ros_ws/install/setup.bash && '
                'ros2 run explore_lite explore --ros-args '
                '-p robot_base_frame:=base_link '
                '-p costmap_topic:=map '
                '-p costmap_updates_topic:=map_updates '
                '-p visualize:=true '
                '-p planner_frequency:=0.15 '
                '-p progress_timeout:=30.0 '
                '-p min_frontier_size:=0.75 '
                '> /app/logs/explore.log 2>&1'
            ]

            process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid
            )

            running_processes['explore'] = process
            # Persist PID pour supervision côté front/API
            try:
                with open(EXPLORE_PID_FILE, 'w') as f:
                    f.write(str(process.pid))
            except Exception:
                pass

            return {
                'success': True,
                'message': 'Exploration started',
                'data': {'pid': process.pid}
            }
        except Exception as e:
            return {'success': False, 'message': f'Failed to start exploration: {str(e)}'}

    def stop_explore(self):
        """Stop explore_lite process"""
        if 'explore' not in running_processes:
            # Déjà arrêté côté process, on renvoie succès pour débloquer l'IHM
            return {'success': True, 'message': 'Exploration already stopped'}

        try:
            process = running_processes['explore']
            if process.poll() is None:
                # Kill process group to kill all child processes
                os.killpg(os.getpgid(process.pid), signal.SIGTERM)
                process.wait(timeout=5)

            del running_processes['explore']
            if os.path.exists(EXPLORE_PID_FILE):
                try:
                    os.remove(EXPLORE_PID_FILE)
                except Exception:
                    pass

            return {'success': True, 'message': 'Exploration stopped'}
        except Exception as e:
            return {'success': False, 'message': f'Failed to stop exploration: {str(e)}'}

    def explore_status(self):
        """Retourne l'état du process explore (PID encore actif) + statut de fin"""
        running = False
        pid = None
        finished = False

        # 1) vérifier le process enregistré
        if 'explore' in running_processes:
            proc = running_processes['explore']
            running = proc.poll() is None
            pid = proc.pid if running else None

        # 2) fallback: vérifier le fichier PID
        if not running and os.path.exists(EXPLORE_PID_FILE):
            try:
                with open(EXPLORE_PID_FILE, 'r') as f:
                    pid_candidate = int(f.read().strip())
                # Vérifier si le PID existe
                os.kill(pid_candidate, 0)
                running = True
                pid = pid_candidate
            except Exception:
                running = False
                pid = None

        # 3) vérifier le log pour détecter la fin même si le process reste vivant
        if os.path.exists(EXPLORE_LOG):
            try:
                with open(EXPLORE_LOG, 'r') as f:
                    tail = f.read()[-2000:]  # dernier bloc
                if 'Exploration stopped' in tail or 'All frontiers traversed' in tail or 'No frontiers found' in tail:
                    finished = True
            except Exception:
                finished = False

        return {'success': True, 'running': running, 'pid': pid, 'finished': finished}

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

    def nav_goal(self, x: float, y: float, theta: float = 0.0):
        """Send a NavigateToPose goal via CLI (ROS2 action)"""
        try:
            # Orientation from yaw
            import math
            qz = math.sin(theta / 2.0)
            qw = math.cos(theta / 2.0)
            goal_yaml = (
                f"{{pose: {{ header: {{ frame_id: map }}, pose: "
                f"{{ position: {{ x: {x}, y: {y}, z: 0.0 }}, "
                f"orientation: {{ x: 0.0, y: 0.0, z: {qz}, w: {qw} }} }} }} }}"
            )
            cmd = [
                'bash', '-c',
                'source /opt/ros/iron/setup.bash && '
                'source /app/ros_ws/install/setup.bash && '
                f"ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \"{goal_yaml}\" --feedback "
                "> /app/logs/nav_goal.log 2>&1"
            ]
            proc = subprocess.Popen(cmd, preexec_fn=os.setsid)
            running_processes['nav_goal'] = proc
            return {'success': True, 'message': 'Goal sent', 'data': {'pid': proc.pid}}
        except Exception as e:
            return {'success': False, 'message': f'Failed to send nav goal: {e}'}

    def nav_cancel(self):
        """Cancel navigate_to_pose action"""
        try:
            cmd = [
                'bash', '-c',
                'source /opt/ros/iron/setup.bash && '
                'source /app/ros_ws/install/setup.bash && '
                'ros2 action cancel /navigate_to_pose > /app/logs/nav_cancel.log 2>&1'
            ]
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=5)
            # Clean up running nav_goal process if any
            if 'nav_goal' in running_processes:
                proc = running_processes['nav_goal']
                if proc.poll() is None:
                    os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
                del running_processes['nav_goal']
            return {'success': True, 'message': result.stdout.strip() or 'Cancel sent'}
        except Exception as e:
            return {'success': False, 'message': f'Failed to cancel goal: {e}'}

    def save_map(self, map_name: str):
        """Save map using nav2 map_saver_cli"""
        try:
            # Ensure extension
            filename = map_name if map_name.endswith('.yaml') else f"{map_name}.yaml"
            cmd = [
                'bash', '-c',
                'source /opt/ros/iron/setup.bash && '
                'source /app/ros_ws/install/setup.bash && '
                f"ros2 run nav2_map_server map_saver_cli -f /app/maps/{filename} "
                "> /app/logs/map_save.log 2>&1"
            ]
            proc = subprocess.run(cmd, capture_output=True, text=True, timeout=30)
            if proc.returncode == 0:
                return {'success': True, 'message': f'Map saved to /app/maps/{filename}'}
            return {'success': False, 'message': proc.stderr or proc.stdout}
        except Exception as e:
            return {'success': False, 'message': f'Failed to save map: {e}'}

    def load_map(self, map_name: str):
        """Load map using nav2 map_server load_map service"""
        try:
            filename = map_name if map_name.endswith('.yaml') else f"{map_name}.yaml"
            full_path = f"/app/maps/{filename}"
            cmd = [
                'bash', '-c',
                'source /opt/ros/iron/setup.bash && '
                'source /app/ros_ws/install/setup.bash && '
                f"ros2 service call /map_server/load_map nav2_msgs/srv/LoadMap '{{map_url: \"{full_path}\"}}' "
                "> /app/logs/map_load.log 2>&1"
            ]
            proc = subprocess.run(cmd, capture_output=True, text=True, timeout=15)
            if proc.returncode == 0:
                return {'success': True, 'message': f'Map loaded from {full_path}'}
            return {'success': False, 'message': proc.stderr or proc.stdout}
        except Exception as e:
            return {'success': False, 'message': f'Failed to load map: {e}'}

    def clear_map(self):
        """Clear current map changes in SLAM Toolbox"""
        try:
            cmd = [
                'bash', '-c',
                'source /opt/ros/iron/setup.bash && '
                'source /app/ros_ws/install/setup.bash && '
                "ros2 service call /slam_toolbox/clear_changes slam_toolbox/srv/Clear '{}' "
                "> /app/logs/map_clear.log 2>&1"
            ]
            proc = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
            if proc.returncode == 0:
                return {'success': True, 'message': 'Map cleared'}
            return {'success': False, 'message': proc.stderr or proc.stdout}
        except Exception as e:
            return {'success': False, 'message': f'Failed to clear map: {e}'}

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
