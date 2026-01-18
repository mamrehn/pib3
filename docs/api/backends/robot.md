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
)
```

**Parameters:**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `host` | `str` | `"172.26.34.149"` | IP address of the robot. |
| `port` | `int` | `9090` | Rosbridge websocket port. |
| `timeout` | `float` | `5.0` | Connection timeout in seconds. |

**Example:**

```python
from pib3 import Robot

# Default connection
robot = Robot()

# Custom host
robot = Robot(host="192.168.1.100")

# All parameters
robot = Robot(
    host="192.168.1.100",
    port=9090,
    timeout=10.0,
)
```

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
    async_: bool = True,
    timeout: float = 1.0,
    tolerance: Optional[float] = None,
) -> bool
```

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `motor_name` | `str` or `Joint` | *required* | Motor name or `Joint` enum (e.g., `Joint.ELBOW_LEFT`). |
| `position` | `float` | *required* | Target position in specified unit. |
| `unit` | `"percent"`, `"rad"`, `"deg"` | `"percent"` | Position unit. |
| `async_` | `bool` | `True` | If `False`, wait for joint to reach target. |
| `timeout` | `float` | `1.0` | Max wait time when `async_=False`. |
| `tolerance` | `float` | `None` | Acceptable error (default: 2%, 3°, or 0.05 rad). |

**Example:**

```python
from pib3 import Robot, Joint

with Robot(host="172.26.34.149") as robot:
    robot.set_joint(Joint.TURN_HEAD, 50.0)           # Percentage
    robot.set_joint(Joint.ELBOW_LEFT, 1.25, unit="rad")  # Radians
    robot.set_joint(Joint.ELBOW_LEFT, -30.0, unit="deg") # Degrees

    # Wait for completion
    success = robot.set_joint(Joint.ELBOW_LEFT, 50.0, async_=False)
```

### set_joints()

Set multiple joint positions simultaneously.

```python
def set_joints(
    self,
    positions: Union[Dict[Union[str, Joint], float], Sequence[float]],
    unit: Literal["percent", "rad", "deg"] = "percent",
    async_: bool = True,
    timeout: float = 1.0,
    tolerance: Optional[float] = None,
) -> bool
```

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `positions` | `Dict[str\|Joint, float]` or `Sequence[float]` | *required* | Target positions. |
| `unit` | `"percent"`, `"rad"`, `"deg"` | `"percent"` | Position unit. |
| `async_` | `bool` | `True` | If `False`, wait for joints to reach targets. |
| `timeout` | `float` | `1.0` | Max wait time when `async_=False`. |
| `tolerance` | `float` | `None` | Acceptable error. |

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

Subscribe to camera images (raw JPEG bytes).

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

!!! note
    Changing settings causes ~100-200ms stream interruption.

---

## AI Detection

AI inference runs only while subscribers are active. Unsubscribe to stop and save resources.

### subscribe_ai_detections()

```python
def subscribe_ai_detections(callback: Callable[[dict], None]) -> roslibpy.Topic
```

Callback receives:
```python
{"model": "mobilenet-ssd", "type": "detection", "frame_id": 42,
 "result": {"detections": [{"label": 15, "confidence": 0.92, "bbox": {...}}]}}
```

```python
def on_detection(data):
    for det in data['result']['detections']:
        print(f"Class {det['label']} @ {det['confidence']:.0%}")

sub = robot.subscribe_ai_detections(on_detection)
time.sleep(10)
sub.unsubscribe()
```

### set_ai_model()

Switch AI model (synchronous, waits for confirmation).

```python
def set_ai_model(model_name: str, timeout: float = 5.0) -> bool
```

```python
if robot.set_ai_model("yolov8n"):
    print("Model ready!")
```

!!! note
    Model switching causes ~200-500ms interruption.

### set_ai_config()

```python
def set_ai_config(
    model: Optional[str] = None,
    confidence: Optional[float] = None,           # 0.0-1.0
    segmentation_mode: Optional[str] = None,      # "bbox" or "mask"
    segmentation_target_class: Optional[int] = None,
) -> None
```

### get_available_ai_models()

```python
models = robot.get_available_ai_models()  # Returns dict of model info
```

---

## IMU Sensor

Access BMI270 IMU data from OAK-D Lite.

### subscribe_imu()

```python
def subscribe_imu(
    callback: Callable[[dict], None],
    data_type: str = "full",  # "full", "accelerometer", or "gyroscope"
) -> roslibpy.Topic
```

| `data_type` | Callback receives |
|-------------|-------------------|
| `"full"` | `{"linear_acceleration": {x,y,z}, "angular_velocity": {x,y,z}}` |
| `"accelerometer"` | `{"vector": {x,y,z}}` |
| `"gyroscope"` | `{"vector": {x,y,z}}` |

```python
def on_imu(data):
    accel = data['linear_acceleration']
    print(f"Accel: {accel['x']:.2f}, {accel['y']:.2f}, {accel['z']:.2f}")

sub = robot.subscribe_imu(on_imu)
time.sleep(5)
sub.unsubscribe()
```

### set_imu_frequency()

```python
def set_imu_frequency(frequency: int) -> None  # 25, 50, 100, 200, or 250 Hz
```

---

## Helper Functions

### rle_decode()

Decode RLE-encoded segmentation masks.

```python
from pib3.backends import rle_decode

mask = rle_decode(result['mask_rle'])  # Returns np.ndarray (height, width)
```

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
