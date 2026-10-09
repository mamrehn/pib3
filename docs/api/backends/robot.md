# Real Robot Backend

Control the physical PIB robot via rosbridge websocket connection.

## Overview

`RealRobotBackend` connects to a PIB robot's ROS system via rosbridge and provides joint control and trajectory execution.

## RealRobotBackend Class

::: pib3.backends.robot.RealRobotBackend
    options:
      show_root_heading: true
      show_source: false
      members: false

---

## Quick Start

```python
from pib3 import Robot, Joint

with Robot(host="172.26.34.149") as robot:
    robot.set_joint(Joint.ELBOW_LEFT, 50.0)  # IDE tab completion
    pos = robot.get_joint(Joint.ELBOW_LEFT)
    robot.run_trajectory("trajectory.json")
```

---

## Constructor

```python
Robot(
    host: str = "172.26.34.149",
    port: int = 9090,
    timeout: float = 5.0,
    motor_mode: str = "direct",
    estop_keys=True,
    stop_button="auto",
)
```

**Parameters:**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `host` | `str` | `"172.26.34.149"` | IP address of the robot. |
| `port` | `int` | `9090` | Rosbridge websocket port. |
| `timeout` | `float` | `5.0` | Connection timeout in seconds. |
| `motor_mode` | `str` | `"direct"` | Motor control mode: `"direct"` (Tinkerforge) or `"ros"` (via rosbridge). Anything else raises `ValueError`. |
| `estop_keys` | `bool`, `str` or list | `True` | Emergency-stop keys, armed by the program's first motion command: `True` = Space, Esc, Numpad-0, Pause; a name or list for others; `False` for none. Ctrl+C stops too. A stop latches every pib3 program on this robot. See [Safety](../../getting-started/safety.md). |
| `stop_button` | `bool` or `"auto"` | `True` | On-screen STOP window while the stop is armed (the visible "armed" sign, lists the working triggers); `"auto"` only where the keys cannot work; `False` never. |

**Example:**

```python
from pib3 import Robot

# Default connection (direct Tinkerforge control, auto-discovered)
robot = Robot()

# Custom host
robot = Robot(host="192.168.1.100")

# All parameters
robot = Robot(
    host="192.168.1.100",
    port=9090,
    timeout=10.0,
)

# Use ROS for motor control instead of Tinkerforge
robot = Robot(host="192.168.1.100", motor_mode="ros")
```

!!! note "ROS motor mode"
    The robot's `motor_control` node expects **one trajectory point per
    joint**. pib3 0.2 sends all joints of a command in one such request
    (earlier versions moved only the first joint of a trajectory waypoint).
    The node ignores the velocity, so `speed=` has no effect in ROS mode;
    joints move with the speed set in Cerebra.

---

## Subsystem Properties

The robot provides high-level subsystem APIs for simplified access to AI and camera features.

### `robot.ai`

Access the [AI subsystem](../ai-camera-subsystems.md#aisubsystem-robotai) for AI inference with auto-managed subscriptions.

```python
with Robot(host="172.26.34.149") as robot:
    robot.ai.set_model(AIModel.HAND)
    for hand in robot.ai.get_hand_landmarks():
        print(f"{hand.handedness}: index={hand.finger_angles.index:.0f}°")
```

### `robot.camera`

Access the [camera subsystem](../ai-camera-subsystems.md#camerasubsystem-robotcamera) for camera streaming with auto-managed subscriptions.

```python
with Robot(host="172.26.34.149") as robot:
    frame = robot.camera.get_frame()
    if frame:
        img = frame.to_numpy()  # Requires OpenCV
```

!!! tip "Subsystems vs Raw Subscriptions"
    For most use cases, use `robot.ai` and `robot.camera` instead of the low-level 
    `subscribe_ai_detections()` and `subscribe_camera_image()` methods. The subsystems 
    automatically manage subscription lifecycle and provide typed return values.

---

## Connection

### connect()

Establish websocket connection to the robot's rosbridge server.

```python
def connect(self) -> None
```

**Raises:**

- `ConnectionError`: If unable to connect within timeout

**Example:**

```python
from pib3 import Robot

robot = Robot(host="172.26.34.149")

try:
    robot.connect()
    print(f"Connected: {robot.is_connected}")
    # ... use robot ...
finally:
    robot.disconnect()
```

### Using Context Manager

```python
from pib3 import Robot, Joint

with Robot(host="172.26.34.149") as robot:
    robot.set_joint(Joint.ELBOW_LEFT, 50.0)
# Automatically disconnected
```

---

## Joint Control

The robot backend inherits all methods from [`RobotBackend`](base.md). Key methods:

### set_joint()

Set a single joint position.

```python
def set_joint(
    self,
    motor_name: Union[str, Joint],
    position: float,
    unit: Literal["percent", "rad", "deg"] = "percent",
    async_: bool = False,
    timeout: float = 2.0,
    tolerance: Optional[float] = None,
    speed: Optional[float] = None,
) -> bool
```

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `motor_name` | `str` or `Joint` | *required* | Motor name or `Joint` enum (e.g., `Joint.ELBOW_LEFT`). |
| `position` | `float` | *required* | Target position in specified unit. |
| `unit` | `"percent"`, `"rad"`, `"deg"` | `"percent"` | Position unit. |
| `async_` | `bool` | `False` | If `False` (default), poll until joint reaches target. |
| `timeout` | `float` | `2.0` | Max wait time when `async_=False`. |
| `tolerance` | `float` | `None` | Acceptable error (default: 2%, 3°, or 0.05 rad). |
| `speed` | `float` or `None` | `None` | Movement speed in deg/s. `None` uses the servo channel's current motion config (default ≈ 150°/s on the direct path — see [`DEFAULT_MOTION_VELOCITY`](#default-motion-config)). |

**Example:**

```python
from pib3 import Robot, Joint

with Robot(host="172.26.34.149") as robot:
    robot.set_joint(Joint.TURN_HEAD, 50.0)               # Percentage
    robot.set_joint(Joint.ELBOW_LEFT, 1.25, unit="rad")  # Radians
    robot.set_joint(Joint.ELBOW_LEFT, -30.0, unit="deg") # Degrees

    # Fire and forget (default waits for completion)
    robot.set_joint(Joint.ELBOW_LEFT, 50.0, async_=True)
```

### set_joints()

Set multiple joint positions simultaneously.

```python
def set_joints(
    self,
    positions: Dict[Union[str, Joint], float],
    unit: Literal["percent", "rad", "deg"] = "percent",
    async_: bool = False,
    timeout: float = 2.0,
    tolerance: Optional[float] = None,
    speed: Optional[float] = None,
) -> bool
```

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `positions` | `Dict[str\|Joint, float]` | *required* | Target positions. A `HandPose` or sequence raises `TypeError` — use `set_joints_pose()` / `set_joints_sequence()` instead. |
| `unit` | `"percent"`, `"rad"`, `"deg"` | `"percent"` | Position unit. |
| `async_` | `bool` | `False` | If `False` (default), poll until joints reach targets. |
| `timeout` | `float` | `2.0` | Max wait time when `async_=False`. |
| `tolerance` | `float` | `None` | Acceptable error. |
| `speed` | `float` or `None` | `None` | Movement speed in deg/s. On the direct path this rewrites the servo's shared motion config — see [base class docs](base.md#set_joints). |

**Example:**

```python
from pib3 import Robot, Joint

with Robot(host="172.26.34.149") as robot:
    robot.set_joints({
        Joint.SHOULDER_VERTICAL_LEFT: 30.0,
        Joint.ELBOW_LEFT: 60.0,
    })
```

### get_joint()

Read a single joint position.

```python
def get_joint(
    self,
    motor_name: Union[str, Joint],
    unit: Literal["percent", "rad", "deg"] = "percent",
    timeout: Optional[float] = None,
) -> Optional[float]
```

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `motor_name` | `str` or `Joint` | *required* | Motor name or `Joint` enum. |
| `unit` | `"percent"`, `"rad"`, `"deg"` | `"percent"` | Return unit. |
| `timeout` | `float` | `5.0` | Max wait time for ROS messages (seconds). |

**Returns:** `float` or `None` - Current position.

```python
from pib3 import Robot, Joint

with Robot(host="172.26.34.149") as robot:
    pos = robot.get_joint(Joint.ELBOW_LEFT)
    pos_rad = robot.get_joint(Joint.ELBOW_LEFT, unit="rad")
```

### get_joints()

Read multiple joint positions.

```python
def get_joints(
    self,
    motor_names: Optional[List[Union[str, Joint]]] = None,
    unit: Literal["percent", "rad", "deg"] = "percent",
    timeout: Optional[float] = None,
) -> Dict[str, float]
```

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `motor_names` | `List[str\|Joint]` or `None` | `None` | Motors to query. `None` returns all. |
| `unit` | `"percent"`, `"rad"`, `"deg"` | `"percent"` | Return unit. |
| `timeout` | `float` | `5.0` | Max wait time for ROS messages (seconds). |

**Returns:** `Dict[str, float]` - Motor names mapped to positions.

```python
from pib3 import Robot, Joint

with Robot(host="172.26.34.149") as robot:
    arm = robot.get_joints([Joint.ELBOW_LEFT, Joint.WRIST_LEFT])
    all_joints = robot.get_joints()
```

---

## Trajectory Execution

### run_trajectory()

```python
def run_trajectory(
    self,
    trajectory: Union[str, Path, Trajectory],
    rate_hz: float = 20.0,
    progress_callback: Optional[Callable[[int, int], None]] = None,
) -> bool
```

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `trajectory` | `str`, `Path`, `Trajectory` | *required* | Trajectory file or object. |
| `rate_hz` | `float` | `20.0` | Waypoints per second. |
| `progress_callback` | `Callable[[int, int], None]` | `None` | Progress callback `(current, total)`. |

```python
from pib3 import Robot, Trajectory

with Robot(host="172.26.34.149") as robot:
    robot.run_trajectory("trajectory.json")

    # With progress
    robot.run_trajectory(
        trajectory,
        rate_hz=20.0,
        progress_callback=lambda c, t: print(f"\r{c}/{t}", end=""),
    )
```

---

## Save and Restore Poses

```python
import json
from pib3 import Robot

with Robot(host="172.26.34.149") as robot:
    pose = robot.get_joints()                    # Save
    with open("pose.json", "w") as f:
        json.dump(pose, f)

    with open("pose.json") as f:
        robot.set_joints(json.load(f))           # Restore
```

---

## Camera Streaming

### subscribe_camera_image()

Subscribe to camera images (raw JPEG bytes). The camera node publishes them as base64 text on `/camera_topic` and only encodes frames while someone is subscribed. The frame is 1280×720 by default.

```python
def subscribe_camera_image(callback: Callable[[bytes], None]) -> roslibpy.Topic
```

Call `.unsubscribe()` on the returned topic to stop streaming.

```python
import cv2, numpy as np

def on_frame(jpeg_bytes):
    frame = cv2.imdecode(np.frombuffer(jpeg_bytes, np.uint8), cv2.IMREAD_COLOR)
    cv2.imshow("Camera", frame)
    cv2.waitKey(1)

with Robot(host="192.168.178.71") as robot:
    sub = robot.subscribe_camera_image(on_frame)
    time.sleep(10)
    sub.unsubscribe()
```

### set_camera_config()

```python
def set_camera_config(
    fps: Optional[int] = None,
    quality: Optional[int] = None,      # JPEG quality 1-100
    resolution: Optional[tuple] = None,  # (width, height)
) -> None
```

Each setting goes to its own topic of the camera node (`quality_factor_topic`, `timer_period_topic`, `size_topic`); there are matching `set_camera_quality()`, `set_camera_timer_period()` and `set_camera_preview_size()`.

!!! note
    Changing the resolution restarts the camera pipeline. Keep it 16:9: the AI models read the same frame, and the camera refuses to feed them one of another aspect.

---

## AI Detection

The camera runs the models of the robot's model store. A client asks for a model with `/start_model` and names itself as an *owner*; the model runs while any owner holds it. The robot publishes each model's results as `datatypes/DetectionArray` on `detections/<model>`. `robot.ai` wraps all of this; the methods below are the layer underneath.

### ai_owner

The name this client uses for `/start_model` and `/stop_model`: `pib3-<user>@<computer>` unless set with `Robot(ai_owner="group-3")`. Clients with the same name share, and stop, each other's models. The default is stable on purpose: a script that crashed leaves its model held under that name, and the next run releases it.

### get_available_ai_models()

```python
models = robot.get_available_ai_models()   # dict: model id -> info
# {"yolo26n_coco_512x288": {"task": "object_detection", "licence": "...",
#                           "shaves": 4, "size_bytes": 5472216,
#                           "available": True, "active": False}, ...}
```

Calls `/list_models`. Only models with `available: True` can be started. Empty if the service does not answer.

### start_ai_model() / stop_ai_model()

```python
def start_ai_model(model, shaves: int = 0, timeout: float = 30.0) -> Tuple[bool, str]
def stop_ai_model(model, timeout: float = 30.0) -> Tuple[bool, str]
```

Call `/start_model` and `/stop_model` as `ai_owner` and return `(success, message)`. The message says why a start failed ("Unknown model", "Model is unavailable", ...). `shaves=0` takes the number the model was compiled for.

```python
ok, message = robot.start_ai_model(AIModel.HAND)
if not ok:
    print(message)
robot.stop_ai_model(AIModel.HAND)
```

!!! note
    A start rebuilds the camera pipeline and can take longer than rosbridge waits for a service answer. When the call gets no answer, `start_ai_model` asks `/list_models` whether the model runs and counts it as started if it does. A stop that gets no answer is reported as failed although the robot may still stop the model; `/models_status` shows the truth.

### set_ai_model()

Same as `robot.ai.set_model(model)`: run this model and none of the others this client started. Each start or stop rebuilds the camera pipeline.

```python
if robot.set_ai_model(AIModel.YOLO26N):
    print("Model ready!")
```

### subscribe_ai_detections()

```python
def subscribe_ai_detections(model, callback: Callable[[dict], None]) -> roslibpy.Topic
```

Subscribes to the model's topic. The model must be running; the topic is silent until then. The callback receives one `DetectionArray` per frame:

```python
{"header": {"stamp": {"sec": 1790000000, "nanosec": 123000000}},
 "model_id": "yolo26n_coco_512x288",
 "frame_width": 1280, "frame_height": 720,
 "detections": [{"label": "person", "score": 0.92,
                 "x_min": 353, "y_min": 257, "x_max": 547, "y_max": 570,   # pixels
                 "keypoint_names": [], "keypoint_x": [], "keypoint_y": [], "keypoint_z": [],
                 "scalar_names": [], "scalar_values": []}]}
```

Pixels refer to `frame_width` × `frame_height`. `keypoint_z` is millimetres, 0 meaning none. The hand models publish their hand-relative depth unitless instead (see `z_source` in `scalar_names`).

```python
def on_detection(message):
    for det in message['detections']:
        print(f"{det['label']} @ {det['score']:.0%}")

sub = robot.subscribe_ai_detections(AIModel.YOLO26N, on_detection)
time.sleep(10)
sub.unsubscribe()
```

For typed results use `AIDetectionReceiver` or `parse_detection_message()` from `pib3.backends`.

### get_ai_detections()

```python
message = robot.get_ai_detections(AIModel.YOLO26N)   # one DetectionArray, or None
```

Calls `/get_detections` once. `frame_width` is 0 while the camera has no frame yet.

### subscribe_ai_status()

```python
sub = robot.subscribe_ai_status(callback)   # /models_status, about 1 Hz
```

The callback receives `{"models": [{"model_id", "state", "message", "active", "fps", "shaves"}, ...]}`; `state` is `idle`, `starting`, `running` or `failed`, and `message` says why a model failed. `fps` is averaged over at least ten seconds.

---

## IMU Sensor

The BMI270 IMU of the OAK-D Lite streams at a fixed 100 Hz on `/imu` (`sensor_msgs/Imu`) whenever the camera node runs. Axes follow ROS (REP-103): x forward, y left, z up, so a robot at rest reads about +9.8 m/s² on z. The BMI270 cannot fuse an orientation: `orientation` stays the identity and `orientation_covariance[0]` is -1.

### subscribe_imu()

```python
def subscribe_imu(
    callback: Callable[[dict], None],
    data_type: str = "full",  # "full", "accelerometer", or "gyroscope"
) -> roslibpy.Topic
```

| `data_type` | Callback receives |
|-------------|-------------------|
| `"full"` | the whole message: `header`, `linear_acceleration`, `angular_velocity`, `orientation` (identity), covariances |
| `"accelerometer"` | `{"header", "vector": {x,y,z}}` |
| `"gyroscope"` | `{"header", "vector": {x,y,z}}` |

```python
def on_imu(data):
    accel = data['linear_acceleration']
    print(f"Accel: {accel['x']:.2f}, {accel['y']:.2f}, {accel['z']:.2f}")

sub = robot.subscribe_imu(on_imu)
time.sleep(5)
sub.unsubscribe()
```

### subscribe_imu_raw()

```python
sub = robot.subscribe_imu_raw(callback)   # the whole sensor_msgs/Imu message as a dict
```

### set_imu_frequency()

Raises `NotImplementedError`: the camera node publishes at a fixed 100 Hz. Skip samples in your callback for a lower rate.

---

## Helper Functions

### rle_decode()

Decode RLE-encoded segmentation masks (`det.mask_rle`). The robot has no segmentation model; the simulation produces them for `"segmentation"`.

```python
from pib3.backends import rle_decode

mask = rle_decode(det.mask_rle)  # Returns np.ndarray (height, width)
```

---

## Text-to-Speech

`RealRobotBackend.speak()` overrides the base implementation and prefers the robot's on-board TTS service `/play_audio_from_speech` — speech is synthesized directly on the robot, avoiding Piper inference on the client and streaming audio over rosbridge. Falls back to base-class Piper synthesis for `AudioOutput.LOCAL` or when the robot service is unavailable.

```python
def speak(
    text: str,
    output: Optional[AudioOutput] = None,
    voice: Optional[str] = None,          # Piper voice (local fallback only)
    block: bool = True,
    use_robot_tts: Optional[bool] = None, # None = auto (prefer robot when output includes ROBOT)
    language: str = "de",                 # Robot TTS language code
) -> bool
```

See [`play_audio_from_speech()`](../audio.md) for the underlying ROS service (supports `wait=False` for fire-and-forget).

---

## Low-Latency Mode

Bypass ROS for direct Tinkerforge motor control with ~5-20ms latency (vs ~100-200ms via ROS). When enabled, both `get_joint()`/`get_joints()` and `set_joint()`/`set_joints()` use direct Tinkerforge communication.

### Quick Setup

Direct Tinkerforge control is the default mode. On connect pib3 reads the
robot's own motor table from pib-api (`http://<host>:5000/motor`, the table
Cerebra edits). It gives the exact bricklet UID and pin of every motor, plus
each motor's `invert` flag and rotation range, so direct control moves every
joint exactly like the ROS path. Without pib-api, pib3 falls back to
enumerating the servo bricklets (`LowLatencyConfig.use_robot_motor_config`).

!!! note "What `get_joint()` reads"
    pib's hobby servos report nothing back. In direct mode `get_joint()`
    returns the position of the bricklet's motion ramp, i.e. where the servo
    is being *told* to be right now. A blocked joint therefore still reads
    as "arrived".

```python
from pib3 import Robot

with Robot(host="172.26.34.149") as robot:
    robot.set_joint("elbow_left", 0.5, unit="rad")  # Direct write
    pos = robot.get_joint("elbow_left", unit="rad")  # Direct read

# To use ROS for motor control instead:
with Robot(host="172.26.34.149", motor_mode="ros") as robot:
    robot.set_joint("elbow_left", 0.5, unit="rad")  # Via ROS
```

### Properties

| Property | Type | Description |
|----------|------|-------------|
| `low_latency_available` | `bool` | True if connected and configured |
| `low_latency_enabled` | `bool` | Get/set enabled state. The setter **raises `RuntimeError`** when the backend is not connected or when Tinkerforge discovery fails (the flag is rolled back). |
| `low_latency_sync_to_ros` | `bool` | Get/set cache sync setting |
| `discovered_servo_uids` | `List[str]` | UIDs from last discovery |

### Default motion config

Every motion command sets its velocity: `speed=...`, else `robot.default_speed`
(150 deg/s). Acceleration and deceleration come from these constants; change
them with `configure_all_servo_channels()`.

| Constant | Default | Unit | Notes |
|----------|---------|------|-------|
| `RealRobotBackend.DEFAULT_MOTION_VELOCITY` | `15000` | 0.01 °/s | ≈ 150°/s |
| `RealRobotBackend.DEFAULT_MOTION_ACCELERATION` | `15000` | 0.01 °/s² | |
| `RealRobotBackend.DEFAULT_MOTION_DECELERATION` | `15000` | 0.01 °/s² | |

### Methods

#### discover_servo_bricklets()

```python
def discover_servo_bricklets(timeout: float = 1.0) -> List[str]
```

Returns list of Servo Bricklet V2 UIDs.

#### configure_motor_mapping()

```python
def configure_motor_mapping(
    mapping: Dict[str, Tuple[str, int]],
    reinitialize: bool = True,
) -> None
```

Configure motor-to-bricklet mapping at runtime. Auto-configures servo channels.

#### configure_servo_channel()

```python
def configure_servo_channel(
    motor_name: str,
    pulse_width_min: int = 700,
    pulse_width_max: int = 2500,
    velocity: int = 9000,
    acceleration: int = 9000,
    deceleration: int = 9000,
) -> bool
```

Configure individual servo channel settings.

### Helper Functions

```python
from pib3 import build_motor_mapping, PIB_SERVO_CHANNELS

# Build complete mapping from 3 UIDs
mapping = build_motor_mapping("UID1", "UID2", "UID3")

# Reference standard channel assignments: (bricklet_number, channel)
print(PIB_SERVO_CHANNELS["elbow_left"])  # (3, 8)
```

See the [Low-Latency Tutorial](../../tutorials/low-latency-mode.md) for complete examples.

---

## ROS Integration

| Topic/Service | Purpose |
|---------------|---------|
| `/motor_current` | Joint position feedback |
| `/apply_joint_trajectory` | Motor commands |

Commands use ROS2 JointTrajectory format with positions in centidegrees.

---

## Troubleshooting

| Problem | Cause | Solution |
|---------|-------|----------|
| Connection refused | Rosbridge not running | `ros2 launch rosbridge_server rosbridge_websocket_launch.xml` |
| Connection timeout | Wrong IP or network issue | `ping <ip>`, increase `timeout` parameter |
| Joint not moving | Limits not calibrated | [Calibrate](../../getting-started/calibration.md) or use `unit="rad"` |
| `get_joint` returns None | ROS message timeout | Increase `timeout`, verify `/motor_current` topic |
