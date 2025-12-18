"""Swift visualization backend for pib_ik package."""

import http.server
import json
import threading
import time
from pathlib import Path
from typing import Callable, Dict, List, Optional

import numpy as np

from .base import RobotBackend

# All joint names (36 total) matching URDF structure
ALL_JOINTS = [
    # Head (2)
    'head_horizontal', 'head_vertical',
    # Left Arm (6)
    'shoulder_vertical_left', 'shoulder_horizontal_left',
    'upper_arm_left', 'elbow_left', 'forearm_left', 'wrist_left',
    # Left Hand (10)
    'thumb_left_opposition', 'thumb_left_proximal', 'thumb_left_distal',
    'index_left_proximal', 'index_left_distal',
    'middle_left_proximal', 'middle_left_distal',
    'ring_left_proximal', 'ring_left_distal',
    'pinky_left_proximal', 'pinky_left_distal',
    # Right Arm (6)
    'shoulder_vertical_right', 'shoulder_horizontal_right',
    'upper_arm_right', 'elbow_right', 'forearm_right', 'wrist_right',
    # Right Hand (10)
    'thumb_right_opposition', 'thumb_right_proximal', 'thumb_right_distal',
    'index_right_proximal', 'index_right_distal',
    'middle_right_proximal', 'middle_right_distal',
    'ring_right_proximal', 'ring_right_distal',
    'pinky_right_proximal', 'pinky_right_distal',
]

# Mapping from slider joint names to URDF link names
SLIDER_TO_URDF_LINK = {
    'head_horizontal': 'urdf_head_base',
    'head_vertical': 'urdf_head',
    'shoulder_vertical_left': 'urdf_shoulder_vertical',
    'shoulder_horizontal_left': 'urdf_shoulder_horizontal',
    'upper_arm_left': 'urdf_elbow-upper',
    'elbow_left': 'urdf_elbow-lower',
    'forearm_left': 'urdf_forearm',
    'wrist_left': 'urdf_palm_left',
    'thumb_left_opposition': 'urdf_thumb_rotator_left',
    'thumb_left_proximal': 'urdf_finger_proximal',
    'thumb_left_distal': 'urdf_finger_distal',
    'index_left_proximal': 'urdf_finger_proximal_2',
    'index_left_distal': 'urdf_finger_distal_2',
    'middle_left_proximal': 'urdf_finger_proximal_3',
    'middle_left_distal': 'urdf_finger_distal_3',
    'ring_left_proximal': 'urdf_finger_proximal_4',
    'ring_left_distal': 'urdf_finger_distal_4',
    'pinky_left_proximal': 'urdf_finger_proximal_5',
    'pinky_left_distal': 'urdf_finger_distal_5',
    'shoulder_vertical_right': 'urdf_shoulder_vertical_2',
    'shoulder_horizontal_right': 'urdf_shoulder_horizontal_2',
    'upper_arm_right': 'urdf_elbow-upper_2',
    'elbow_right': 'urdf_elbow-lower_2',
    'forearm_right': 'urdf_forearm_2',
    'wrist_right': 'urdf_palm_right',
    'thumb_right_opposition': 'urdf_thumb_rotator_right',
    'thumb_right_proximal': 'urdf_finger_proximal_6',
    'thumb_right_distal': 'urdf_finger_distal_6',
    'index_right_proximal': 'urdf_finger_proximal_7',
    'index_right_distal': 'urdf_finger_distal_7',
    'middle_right_proximal': 'urdf_finger_proximal_8',
    'middle_right_distal': 'urdf_finger_distal_8',
    'ring_right_proximal': 'urdf_finger_proximal_9',
    'ring_right_distal': 'urdf_finger_distal_9',
    'pinky_right_proximal': 'urdf_finger_proximal_10',
    'pinky_right_distal': 'urdf_finger_distal_10',
}

# Preset poses
PRESETS = {
    'zero': {name: 0.0 for name in ALL_JOINTS},
    'tpose': {
        **{name: 0.0 for name in ALL_JOINTS},
        'shoulder_vertical_left': -0.57,
        'elbow_left': 0.785,
        'shoulder_vertical_right': 0.57,
        'elbow_right': 0.785,
    },
}


def _generate_slider_html() -> str:
    """Generate the HTML page for joint control sliders."""

    def make_section(title, joints, strip_suffix=""):
        controls = []
        for j in joints:
            label = j.replace(strip_suffix, '').replace('_', ' ').title()
            controls.append(f'''<div class="control">
  <label>{label}</label>
  <input type="range" min="-3.14" max="3.14" step="0.05" value="0"
         data-joint="{j}" oninput="update('{j}', this.value)">
  <span class="value" id="{j}_val">0.00</span>
</div>''')
        return f'<div class="section"><h2>{title}</h2>\n' + '\n'.join(controls) + '</div>'

    sections = [
        make_section("Head", ALL_JOINTS[:2]),
        make_section("Left Arm", ALL_JOINTS[2:8], "_left"),
        make_section("Left Hand", ALL_JOINTS[8:19], "_left"),
        make_section("Right Arm", ALL_JOINTS[19:25], "_right"),
        make_section("Right Hand", ALL_JOINTS[25:], "_right"),
    ]

    return f"""<!DOCTYPE html>
<html>
<head>
<title>PIB Joint Control</title>
<style>
  body {{
    font-family: Arial, sans-serif;
    padding: 20px;
    max-width: 1400px;
    margin: 0 auto;
    background: #1a1a2e;
    color: #eee;
  }}
  h1 {{ color: #16c79a; }}
  h2 {{
    color: #16c79a;
    margin-top: 25px;
    border-bottom: 1px solid #333;
    padding-bottom: 5px;
  }}
  .controls-grid {{
    display: grid;
    grid-template-columns: repeat(auto-fill, minmax(400px, 1fr));
    gap: 15px;
  }}
  .section {{
    background: #222;
    padding: 15px;
    border-radius: 8px;
    margin-bottom: 15px;
  }}
  .control {{
    display: flex;
    align-items: center;
    margin-bottom: 8px;
  }}
  label {{
    min-width: 180px;
    font-size: 13px;
  }}
  input[type="range"] {{
    flex: 1;
    margin: 0 10px;
  }}
  .value {{
    min-width: 50px;
    font-weight: bold;
    font-family: monospace;
    text-align: right;
  }}
  .preset-buttons {{
    margin: 20px 0;
  }}
  button {{
    background: #16c79a;
    border: none;
    padding: 10px 20px;
    margin-right: 10px;
    border-radius: 5px;
    cursor: pointer;
    font-weight: bold;
    color: #1a1a2e;
  }}
  button:hover {{
    background: #12a87f;
  }}
  #status {{
    position: fixed;
    top: 10px;
    right: 10px;
    padding: 10px;
    background: #333;
    border-radius: 5px;
  }}
</style>
<script>
async function update(name, value) {{
    document.getElementById(name+'_val').innerText = parseFloat(value).toFixed(2);
    await fetch('/update', {{
        method: 'POST',
        headers: {{'Content-Type': 'application/json'}},
        body: JSON.stringify({{name: name, value: parseFloat(value)}})
    }});
}}

async function setPreset(presetName) {{
    const response = await fetch('/preset/' + presetName);
    const data = await response.json();
    for (const [name, value] of Object.entries(data)) {{
        const slider = document.querySelector(`input[data-joint="${{name}}"]`);
        if (slider) {{
            slider.value = value;
            document.getElementById(name + '_val').innerText = parseFloat(value).toFixed(2);
        }}
    }}
}}

async function resetAll() {{
    setPreset('zero');
}}
</script>
</head>
<body>
<h1>PIB Robot Joint Control</h1>
<p>Adjust sliders to control individual joints. Values are in radians.</p>

<div class="preset-buttons">
  <button onclick="setPreset('zero')">Zero Pose</button>
  <button onclick="setPreset('tpose')">T-Pose</button>
  <button onclick="resetAll()">Reset All</button>
</div>

<div id="status">Swift: Check localhost:52000</div>

<div class="controls-grid">
{chr(10).join(sections)}
</div>
</body>
</html>
"""


class SwiftBackend(RobotBackend):
    """
    Visualize trajectories in Swift browser viewer.

    Uses roboticstoolbox's Swift backend for 3D visualization
    in a web browser.

    Example:
        >>> from pib_ik.backends import SwiftBackend
        >>> with SwiftBackend() as viz:
        ...     viz.run_trajectory("trajectory.json")

        # Interactive mode with sliders:
        >>> viz = SwiftBackend()
        >>> viz.launch_interactive()  # Opens slider UI at localhost:8001
    """

    SWIFT_OFFSET = -1.0  # radians offset for Swift motors

    def __init__(self, realtime: bool = True):
        """
        Initialize Swift visualization backend.

        Args:
            realtime: If True, playback at real-time speed.
                     If False, render as fast as possible.
        """
        self.realtime = realtime
        self._env = None
        self._robot = None
        self._joint_name_to_idx: Dict[str, int] = {}
        self._link_name_to_idx: Dict[str, int] = {}
        self._joint_state: Dict[str, float] = {name: 0.0 for name in ALL_JOINTS}
        self._server = None
        self._server_thread = None

    def _to_backend_format(self, radians: np.ndarray) -> np.ndarray:
        """Swift uses URDF radians directly."""
        return radians + self.SWIFT_OFFSET

    def _from_backend_format(self, values: np.ndarray) -> np.ndarray:
        """Swift uses URDF radians directly."""
        return values + self.SWIFT_OFFSET

    def _get_urdf_path(self) -> Path:
        """Get path to bundled URDF file."""
        return Path(__file__).parent.parent / "resources" / "pib_model.urdf"

    def connect(self) -> None:
        """Launch Swift visualization with robot model."""
        # Patch numpy for compatibility
        if not hasattr(np, 'disp'):
            np.disp = lambda x: print(x)

        try:
            import roboticstoolbox as rtb
            import swift
        except ImportError as e:
            raise ImportError(
                f"roboticstoolbox and swift are required for visualization. "
                f"Install with: pip install roboticstoolbox-python swift. "
                f"Error: {e}"
            )

        # Patch asyncio for compatibility
        try:
            import asyncio
            original_get_running_loop = asyncio.get_running_loop

            def patched_get_running_loop():
                try:
                    return original_get_running_loop()
                except RuntimeError:
                    try:
                        return asyncio.get_event_loop()
                    except RuntimeError:
                        loop = asyncio.new_event_loop()
                        asyncio.set_event_loop(loop)
                        return loop

            asyncio.get_running_loop = patched_get_running_loop
        except Exception:
            pass

        # Load robot
        urdf_path = self._get_urdf_path()
        if not urdf_path.exists():
            raise FileNotFoundError(f"URDF file not found: {urdf_path}")

        self._robot = rtb.ERobot.URDF(str(urdf_path))

        # Build link name to index mapping
        self._link_name_to_idx = {}
        for link in self._robot.links:
            if link.isjoint and link.jindex is not None:
                self._link_name_to_idx[link.name] = link.jindex

        # Build joint name to index mapping (from trajectory motor names)
        from ..trajectory import URDF_TO_MOTOR_NAME
        self._joint_name_to_idx = {}
        for idx, motor_name in URDF_TO_MOTOR_NAME.items():
            self._joint_name_to_idx[motor_name] = idx

        # Launch Swift
        self._env = swift.Swift()
        self._env.launch(realtime=self.realtime)
        self._env.add(self._robot)

        # Initial step to show robot
        self._robot.q = np.zeros(self._robot.n)
        self._env.step(0.1)

    def disconnect(self) -> None:
        """Close Swift visualization and stop slider server."""
        self._stop_slider_server()
        if self._env is not None:
            try:
                self._env.close()
            except Exception:
                pass
            self._env = None
            self._robot = None

    @property
    def is_connected(self) -> bool:
        """Check if visualization is active."""
        return self._env is not None and self._robot is not None

    def _get_joint_radians(self, motor_name: str) -> Optional[float]:
        """
        Get current position of a single joint in radians.

        Args:
            motor_name: Name of motor (e.g., "elbow_left").

        Returns:
            Current position in radians, or None if unavailable.
        """
        if not self.is_connected:
            return None

        if motor_name in self._joint_name_to_idx:
            idx = self._joint_name_to_idx[motor_name]
            if idx < len(self._robot.q):
                return float(self._robot.q[idx])

        return None

    def _get_joints_radians(
        self,
        motor_names: Optional[List[str]] = None,
        timeout: Optional[float] = None,
    ) -> Dict[str, float]:
        """
        Get current positions of multiple joints in radians.

        Args:
            motor_names: List of motor names to query. If None, returns all
                        available joints.
            timeout: Ignored (Swift has synchronous access).

        Returns:
            Dict mapping motor names to positions in radians.
        """
        if not self.is_connected:
            return {}

        result = {}

        if motor_names is None:
            # Return all joints we know about
            for name, idx in self._joint_name_to_idx.items():
                if idx < len(self._robot.q):
                    result[name] = float(self._robot.q[idx])
        else:
            for name in motor_names:
                if name in self._joint_name_to_idx:
                    idx = self._joint_name_to_idx[name]
                    if idx < len(self._robot.q):
                        result[name] = float(self._robot.q[idx]) + self.SWIFT_OFFSET

        return result

    def _set_joints_impl(self, positions_radians: Dict[str, float]) -> bool:
        """Set joint positions and update visualization."""
        if not self.is_connected:
            return False

        q = self._robot.q.copy()

        for joint_name, position in positions_radians.items():
            if joint_name in self._joint_name_to_idx:
                idx = self._joint_name_to_idx[joint_name]
                if idx < len(q):
                    q[idx] = position + self.SWIFT_OFFSET

        self._robot.q = q
        self._env.step(0.05)
        return True

    def _execute_waypoints(
        self,
        joint_names: List[str],
        waypoints: np.ndarray,
        rate_hz: float,
        progress_callback: Optional[Callable[[int, int], None]],
    ) -> bool:
        """Visualize waypoints in Swift."""
        if not self.is_connected:
            return False

        period = 1.0 / rate_hz
        total = len(waypoints)

        for i, point in enumerate(waypoints):
            self._robot.q = point
            self._env.step(period)

            if self.realtime:
                time.sleep(period)

            if progress_callback:
                progress_callback(i + 1, total)

        return True

    def add_paper(
        self,
        center_x: float,
        center_y: float,
        height_z: float,
        size: float = 0.12,
    ) -> bool:
        """
        Add paper visualization to the scene.

        Args:
            center_x: Paper center X position.
            center_y: Paper center Y position.
            height_z: Paper height (Z position).
            size: Paper size (width/height) in meters.

        Returns:
            True if successful.
        """
        if not self.is_connected:
            return False

        try:
            import spatialmath as sm
            import spatialgeometry as sg

            paper_thickness = 0.005
            paper_pose = sm.SE3(center_x, center_y, height_z - paper_thickness / 2)
            paper = sg.Cuboid(
                scale=[size, size, paper_thickness],
                pose=paper_pose,
                color=[0.95, 0.95, 0.9],
            )
            self._env.add(paper)
            self._env.step(0.1)
            return True
        except Exception:
            return False

    # --- Interactive Slider Mode ---

    def _create_handler(self):
        """Create HTTP request handler with access to backend state."""
        backend = self

        class SliderHandler(http.server.BaseHTTPRequestHandler):
            def do_GET(self):
                if self.path == '/':
                    self.send_response(200)
                    self.send_header('Content-type', 'text/html')
                    self.end_headers()
                    self.wfile.write(_generate_slider_html().encode('utf-8'))
                elif self.path.startswith('/preset/'):
                    preset_name = self.path.split('/')[-1]
                    if preset_name in PRESETS:
                        preset = PRESETS[preset_name]
                        for name, val in preset.items():
                            backend._joint_state[name] = val
                        self.send_response(200)
                        self.send_header('Content-type', 'application/json')
                        self.end_headers()
                        self.wfile.write(json.dumps(preset).encode('utf-8'))
                    else:
                        self.send_response(404)
                        self.end_headers()
                else:
                    self.send_response(404)
                    self.end_headers()

            def do_POST(self):
                if self.path == '/update':
                    content_length = int(self.headers['Content-Length'])
                    post_data = self.rfile.read(content_length)
                    data = json.loads(post_data.decode('utf-8'))

                    name = data.get('name')
                    val = data.get('value')
                    if name in backend._joint_state:
                        backend._joint_state[name] = val

                    self.send_response(200)
                    self.end_headers()
                    self.wfile.write(b'OK')

            def log_message(self, format, *args):
                pass  # Suppress logs

        return SliderHandler

    def _start_slider_server(self, port: int = 8001) -> None:
        """Start the web slider server."""
        handler = self._create_handler()
        self._server = http.server.HTTPServer(('0.0.0.0', port), handler)

        def serve():
            self._server.serve_forever()

        self._server_thread = threading.Thread(target=serve, daemon=True)
        self._server_thread.start()
        print(f"Slider UI available at http://localhost:{port}")

    def _stop_slider_server(self) -> None:
        """Stop the web slider server."""
        if self._server is not None:
            self._server.shutdown()
            self._server = None
            self._server_thread = None

    def _update_from_sliders(self) -> None:
        """Update robot pose from current slider values."""
        if not self.is_connected:
            return

        q = np.zeros(self._robot.n)

        for joint_name, value in self._joint_state.items():
            if joint_name in SLIDER_TO_URDF_LINK:
                link_name = SLIDER_TO_URDF_LINK[joint_name]
                if link_name in self._link_name_to_idx:
                    idx = self._link_name_to_idx[link_name]
                    q[idx] = value

        self._robot.q = q
        try:
            self._env.step(0.05)
        except Exception:
            pass

    def launch_interactive(self, port: int = 8001) -> None:
        """
        Launch interactive mode with web-based slider controls.

        Opens a web interface at localhost:{port} with sliders for
        all robot joints. Slider changes update the Swift visualization
        in real-time.

        Args:
            port: HTTP server port for slider UI (default: 8001).

        Example:
            >>> from pib_ik import Swift
            >>> viz = Swift()
            >>> viz.launch_interactive()
            # Open http://localhost:8001 for sliders
            # Open http://localhost:52000 for 3D view
            # Press Ctrl+C to exit
        """
        if not self.is_connected:
            self.connect()

        self._start_slider_server(port)

        print("\n" + "=" * 60)
        print(f"  Slider controls: http://localhost:{port}")
        print("  Swift 3D view:   http://localhost:52000")
        print("  Press Ctrl+C to exit")
        print("=" * 60 + "\n")

        try:
            while True:
                self._update_from_sliders()
                time.sleep(0.05)
        except KeyboardInterrupt:
            print("\nStopping interactive mode...")
            self.disconnect()
