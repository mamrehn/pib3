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
from pib3 import Robot

with Robot(host="172.26.34.149") as robot:
    # Control joints
    robot.set_joint("elbow_left", 50.0)

    # Read positions
    pos = robot.get_joint("elbow_left")
    print(f"Elbow at {pos:.1f}%")

    # Execute trajectory
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

The recommended way to use the robot:

```python
from pib3 import Robot

with Robot(host="172.26.34.149") as robot:
    # Automatically connected
    robot.set_joint("elbow_left", 50.0)
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
    motor_name: str,
    position: float,
    unit: Literal["percent", "rad"] = "percent",
    async_: bool = True,
    timeout: float = 1.0,
    tolerance: Optional[float] = None,
) -> bool
```

**Parameters:**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `motor_name` | `str` | *required* | Motor name (e.g., `"elbow_left"`). |
| `position` | `float` | *required* | Target position (0-100 for percent, radians for rad). |
| `unit` | `"percent"` or `"rad"` | `"percent"` | Position unit. |
| `async_` | `bool` | `True` | If `True`, return immediately. If `False`, wait for joint to reach target. |
| `timeout` | `float` | `1.0` | Max wait time (only used when `async_=False`). |
| `tolerance` | `float` or `None` | `None` | Acceptable error (2.0% or 0.05 rad default). |

**Returns:** `bool` - `True` if successful.

**Example:**

```python
with Robot(host="172.26.34.149") as robot:
    # Percentage (default)
    robot.set_joint("turn_head_motor", 50.0)  # Center head

    # Radians
    robot.set_joint("elbow_left", 1.25, unit="rad")

    # Wait for completion
    success = robot.set_joint(
        "elbow_left",
        50.0,
        async_=False,
        timeout=2.0,
        tolerance=2.0,
    )
    if success:
        print("Joint reached target!")
    else:
        print("Timeout - joint didn't reach target")
```

### set_joints()

Set multiple joint positions simultaneously.

```python
def set_joints(
    self,
    positions: Union[Dict[str, float], Sequence[float]],
    unit: Literal["percent", "rad"] = "percent",
    async_: bool = True,
    timeout: float = 1.0,
    tolerance: Optional[float] = None,
) -> bool
```

**Parameters:**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `positions` | `Dict[str, float]` or `Sequence[float]` | *required* | Target positions as dict or sequence. |
| `unit` | `"percent"` or `"rad"` | `"percent"` | Position unit. |
| `async_` | `bool` | `True` | If `True`, return immediately. If `False`, wait for joints to reach targets. |
| `timeout` | `float` | `1.0` | Max wait time (only used when `async_=False`). |
| `tolerance` | `float` or `None` | `None` | Acceptable error. |

**Example:**

```python
with Robot(host="172.26.34.149") as robot:
    robot.set_joints({
        "shoulder_vertical_left": 30.0,
        "elbow_left": 60.0,
    })
```

### get_joint()

Read a single joint position.

```python
def get_joint(
    self,
    motor_name: str,
    unit: Literal["percent", "rad"] = "percent",
    timeout: Optional[float] = None,
) -> Optional[float]
```

**Parameters:**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `motor_name` | `str` | *required* | Motor name to query. |
| `unit` | `"percent"` or `"rad"` | `"percent"` | Return unit. |
| `timeout` | `float` or `None` | `5.0` | Max time to wait for ROS messages to arrive (seconds). |

**Returns:** `float` or `None` - Current position, or `None` if unavailable.

**Example:**

```python
with Robot(host="172.26.34.149") as robot:
    # Uses default 5s timeout
    pos = robot.get_joint("elbow_left")
    print(f"Elbow at {pos:.1f}%")

    # Custom timeout
    pos_rad = robot.get_joint("elbow_left", unit="rad", timeout=2.0)
    print(f"Elbow at {pos_rad:.3f} rad")
```

### get_joints()

Read multiple joint positions.

```python
def get_joints(
    self,
    motor_names: Optional[List[str]] = None,
    unit: Literal["percent", "rad"] = "percent",
    timeout: Optional[float] = None,
) -> Dict[str, float]
```

**Parameters:**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `motor_names` | `List[str]` or `None` | `None` | Motors to query. `None` returns all. |
| `unit` | `"percent"` or `"rad"` | `"percent"` | Return unit. |
| `timeout` | `float` or `None` | `5.0` | Max time to wait for ROS messages to arrive (seconds). |

**Returns:** `Dict[str, float]` - Motor names mapped to positions.

**Example:**

```python
with Robot(host="172.26.34.149") as robot:
    # Get specific joints (uses default 5s timeout)
    arm = robot.get_joints([
        "shoulder_vertical_left",
        "elbow_left",
        "wrist_left",
    ])

    # Get all joints with custom timeout
    all_joints = robot.get_joints(timeout=2.0)
```

---

## Trajectory Execution

### run_trajectory()

Execute a trajectory on the robot.

```python
def run_trajectory(
    self,
    trajectory: Union[str, Path, Trajectory],
    rate_hz: float = 20.0,
    progress_callback: Optional[Callable[[int, int], None]] = None,
) -> bool
```

**Parameters:**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `trajectory` | `str`, `Path`, or `Trajectory` | *required* | Trajectory file path or object. |
| `rate_hz` | `float` | `20.0` | Playback rate (waypoints per second). |
| `progress_callback` | `Callable[[int, int], None]` or `None` | `None` | Progress callback `(current, total)`. |

**Returns:** `bool` - `True` if completed successfully.

**Example:**

```python
from pib3 import Robot, Trajectory

with Robot(host="172.26.34.149") as robot:
    # From file
    robot.run_trajectory("trajectory.json")

    # From Trajectory object
    trajectory = Trajectory.from_json("trajectory.json")
    robot.run_trajectory(trajectory, rate_hz=20.0)

    # With progress
    def progress(current, total):
        print(f"\r{current}/{total} waypoints", end="")

    robot.run_trajectory(
        trajectory,
        rate_hz=20.0,
        progress_callback=progress,
    )
```

---

## Save and Restore Poses

```python
import json
from pib3 import Robot

with Robot(host="172.26.34.149") as robot:
    # Save current pose
    pose = robot.get_joints()

    # Save to file
    with open("saved_pose.json", "w") as f:
        json.dump(pose, f, indent=2)

    # Later: restore from file
    with open("saved_pose.json") as f:
        saved = json.load(f)
    robot.set_joints(saved)
```

---

## Camera Streaming

Access the OAK-D Lite camera via streaming.

### subscribe_camera_image()

Subscribe to camera images. The callback receives raw JPEG bytes.

```python
def subscribe_camera_image(
    self,
    callback: Callable[[bytes], None],
) -> roslibpy.Topic
```

**Parameters:**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `callback` | `Callable[[bytes], None]` | *required* | Called with raw JPEG bytes for each frame. |

**Returns:** `roslibpy.Topic` - Call `.unsubscribe()` when done to stop streaming.

!!! note "Data Transport"
    Data is transmitted as base64-encoded JSON over the WebSocket connection.
    The pib3 library automatically decodes this to raw JPEG bytes before
    calling your callback.

**Example:**

```python
import cv2
import numpy as np

def on_frame(jpeg_bytes):
    img = np.frombuffer(jpeg_bytes, dtype=np.uint8)
    frame = cv2.imdecode(img, cv2.IMREAD_COLOR)
    cv2.imshow("Camera", frame)
    cv2.waitKey(1)

with Robot(host="192.168.178.71") as robot:
    sub = robot.subscribe_camera_image(on_frame)
    time.sleep(10)
    sub.unsubscribe()
```

### set_camera_config()

Configure camera settings.

```python
def set_camera_config(
    self,
    fps: Optional[int] = None,
    quality: Optional[int] = None,
    resolution: Optional[tuple] = None,
) -> None
```

**Parameters:**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `fps` | `int` or `None` | `None` | Frames per second (e.g., 30). |
| `quality` | `int` or `None` | `None` | JPEG quality 1-100. |
| `resolution` | `tuple` or `None` | `None` | (width, height) tuple. |

!!! note "Pipeline Rebuild"
    Changing camera settings causes a brief stream interruption (~100-200ms).

---

## AI Detection

Access AI inference from the OAK-D Lite's neural compute engine.

!!! tip "On-Demand Pattern"
    AI inference only runs while you have active subscribers. Unsubscribe to stop inference and save resources.

### subscribe_ai_detections()

Subscribe to AI detection results.

```python
def subscribe_ai_detections(
    self,
    callback: Callable[[dict], None],
) -> roslibpy.Topic
```

**Callback data format:**

```python
{
    "model": "mobilenet-ssd",
    "type": "detection",  # or "classification", "segmentation", "pose"
    "frame_id": 42,
    "timestamp_ns": 1234567890123456789,
    "latency_ms": 12.5,
    "result": {
        "detections": [
            {"label": 15, "confidence": 0.92, "bbox": {...}}
        ]
    }
}
```

**Example:**

```python
def on_detection(data):
    if data['type'] == 'detection':
        for det in data['result']['detections']:
            print(f"Class {det['label']} at {det['bbox']}")

with Robot(host="172.26.34.149") as robot:
    sub = robot.subscribe_ai_detections(on_detection)
    time.sleep(10)
    sub.unsubscribe()  # Inference stops
```

### get_available_ai_models()

Get list of available AI models.

```python
def get_available_ai_models(
    self,
    timeout: float = 5.0,
) -> dict
```

**Returns:** Dict mapping model names to their info.

**Example:**

```python
models = robot.get_available_ai_models()
for name, info in models.items():
    print(f"{name}: {info['type']}")
```

### set_ai_model()

Switch the active AI model (synchronous).

```python
def set_ai_model(
    self,
    model_name: str,
    timeout: float = 5.0,
) -> bool
```

**Parameters:**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `model_name` | `str` | *required* | Name of the model to switch to. |
| `timeout` | `float` | `5.0` | Maximum time to wait for confirmation (seconds). |

**Returns:** `bool` - `True` if model switch confirmed, `False` if timeout.

This method is **synchronous** - it waits for the robot to confirm that the model
has been loaded before returning. This ensures the model is ready to use immediately
after the call returns.

**Example:**

```python
with Robot(host="192.168.178.71") as robot:
    # Switch model and check success
    if robot.set_ai_model("yolov8n"):
        print("Model ready!")
    else:
        print("Model switch timed out")

    # With custom timeout
    if robot.set_ai_model("human-pose-estimation", timeout=10.0):
        print("Pose model ready!")
```

!!! note "Pipeline Rebuild"
    Model switching causes ~200-500ms interruption as the neural network
    pipeline rebuilds on the OAK-D Lite.

### set_ai_config()

Configure AI inference settings.

```python
def set_ai_config(
    self,
    model: Optional[str] = None,
    confidence: Optional[float] = None,
    segmentation_mode: Optional[str] = None,
    segmentation_target_class: Optional[int] = None,
) -> None
```

**Parameters:**

| Parameter | Type | Description |
|-----------|------|-------------|
| `model` | `str` | Model name to switch to. |
| `confidence` | `float` | Detection threshold (0.0-1.0). |
| `segmentation_mode` | `str` | `"bbox"` (lightweight) or `"mask"` (detailed RLE). |
| `segmentation_target_class` | `int` | Class ID for mask mode. |

### subscribe_current_ai_model()

Subscribe to model change notifications.

```python
def subscribe_current_ai_model(
    self,
    callback: Callable[[dict], None],
) -> roslibpy.Topic
```

---

## IMU Sensor

Access IMU data from the OAK-D Lite's BMI270 sensor.

### subscribe_imu()

Subscribe to IMU data.

```python
def subscribe_imu(
    self,
    callback: Callable[[dict], None],
    data_type: str = "full",
) -> roslibpy.Topic
```

**Parameters:**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `callback` | `Callable[[dict], None]` | *required* | Called with IMU data. |
| `data_type` | `str` | `"full"` | `"full"`, `"accelerometer"`, or `"gyroscope"`. |

!!! info "Data Source"
    All data types subscribe to the same `/imu/data` ROS topic. The `accelerometer`
    and `gyroscope` options filter the data client-side for convenience, providing
    a simplified data format.

**Callback data formats:**

| `data_type` | Callback receives |
|-------------|-------------------|
| `"full"` | `{"linear_acceleration": {x, y, z}, "angular_velocity": {x, y, z}, "header": {...}}` |
| `"accelerometer"` | `{"vector": {x, y, z}, "header": {...}}` |
| `"gyroscope"` | `{"vector": {x, y, z}, "header": {...}}` |

**Example:**

```python
# Full IMU data
def on_imu(data):
    accel = data['linear_acceleration']
    gyro = data['angular_velocity']
    print(f"Accel: {accel['x']:.2f}, {accel['y']:.2f}, {accel['z']:.2f}")

with Robot(host="192.168.178.71") as robot:
    sub = robot.subscribe_imu(on_imu, data_type="full")
    time.sleep(5)
    sub.unsubscribe()

# Accelerometer only
def on_accel(data):
    vec = data['vector']
    print(f"Accel: {vec['x']:.2f}, {vec['y']:.2f}, {vec['z']:.2f} m/s²")

with Robot(host="192.168.178.71") as robot:
    sub = robot.subscribe_imu(on_accel, data_type="accelerometer")
    time.sleep(3)
    sub.unsubscribe()
```

### set_imu_frequency()

Set IMU sampling frequency.

```python
def set_imu_frequency(self, frequency: int) -> None
```

Valid frequencies: 25, 50, 100, 200, 250 Hz. BMI270 rounds down to nearest valid frequency.

---

## Helper Functions

### rle_decode()

Decode RLE-encoded segmentation masks.

```python
from pib3.backends import rle_decode

def rle_decode(rle: dict) -> np.ndarray
```

**Parameters:**

| Parameter | Type | Description |
|-----------|------|-------------|
| `rle` | `dict` | Dict with `'size'` [height, width] and `'counts'` list. |

**Returns:** Binary mask as numpy array of shape (height, width).

**Example:**

```python
from pib3.backends import rle_decode

def on_segmentation(data):
    if data['type'] == 'segmentation':
        result = data['result']
        if result.get('mode') == 'mask':
            mask = rle_decode(result['mask_rle'])
            print(f"Mask shape: {mask.shape}")
```

---

## ROS Integration

### Topics and Services

| Name | Type | Purpose |
|------|------|---------|
| `/motor_current` | `DiagnosticStatus` | Joint position feedback |
| `/apply_joint_trajectory` | `ApplyJointTrajectory` | Send motor commands |

### Message Format

Commands are sent in ROS2 JointTrajectory format with positions in centidegrees:

```python
# Internal conversion (handled automatically)
centidegrees = round(degrees(radians) * 100)
```

---

## Troubleshooting

!!! warning "Connection Refused"
    **Cause:** Rosbridge is not running on the robot.

    **Solution:** Start rosbridge on the robot:

    ```bash
    ros2 launch rosbridge_server rosbridge_websocket_launch.xml
    ```

!!! warning "Connection Timeout"
    **Cause:** Network issue or incorrect IP address.

    **Solution:**

    1. Verify IP address: `ping 172.26.34.149`
    2. Check rosbridge port: `nc -zv 172.26.34.149 9090`
    3. Increase timeout:

    ```python
    robot = Robot(host="172.26.34.149", timeout=10.0)
    ```

!!! warning "Joint Not Moving"
    **Cause:** Motor not connected or joint limits not calibrated.

    **Solution:**

    1. Verify motor responds in Cerebra
    2. Calibrate joint limits (see [Calibration Guide](../../getting-started/calibration.md))
    3. Try using radians directly:

    ```python
    robot.set_joint("elbow_left", 1.0, unit="rad")
    ```

!!! warning "Position Reading is None or Empty Dict"
    **Cause:** Motor feedback not received within timeout, or motor not publishing.

    **Solution:** The `get_joint()` and `get_joints()` methods wait up to 5 seconds by default for ROS messages to arrive. If you're still getting `None`, try:

    1. Increase the timeout:

    ```python
    pos = robot.get_joint("elbow_left", timeout=10.0)
    ```

    2. Verify the motor is publishing feedback in ROS:

    ```bash
    ros2 topic echo /motor_current
    ```
