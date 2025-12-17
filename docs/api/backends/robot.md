# Real Robot Backend

Control the physical PIB robot via rosbridge websocket connection.

## Overview

`RealRobotBackend` connects to a PIB robot's ROS system via rosbridge and provides joint control and trajectory execution.

## RealRobotBackend Class

::: pib_ik.backends.robot.RealRobotBackend
    options:
      show_root_heading: true
      show_source: false
      members: false

---

## Quick Start

```python
from pib_ik import Robot

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
from pib_ik import Robot

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
from pib_ik import Robot

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
from pib_ik import Robot

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
    verify: bool = False,
    verify_timeout: float = 1.0,
    verify_tolerance: Optional[float] = None,
) -> bool
```

**Parameters:**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `motor_name` | `str` | *required* | Motor name (e.g., `"elbow_left"`). |
| `position` | `float` | *required* | Target position (0-100 for percent, radians for rad). |
| `unit` | `"percent"` or `"rad"` | `"percent"` | Position unit. |
| `verify` | `bool` | `False` | Wait for joint to reach target. |
| `verify_timeout` | `float` | `1.0` | Max wait time for verification (seconds). |
| `verify_tolerance` | `float` or `None` | `None` | Acceptable error (2.0% or 0.05 rad default). |

**Returns:** `bool` - `True` if successful.

**Example:**

```python
with Robot(host="172.26.34.149") as robot:
    # Percentage (default)
    robot.set_joint("turn_head_motor", 50.0)  # Center head

    # Radians
    robot.set_joint("elbow_left", 1.25, unit="rad")

    # With verification
    success = robot.set_joint(
        "elbow_left",
        50.0,
        verify=True,
        verify_timeout=2.0,
        verify_tolerance=2.0,
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
    verify: bool = False,
    verify_timeout: float = 1.0,
    verify_tolerance: Optional[float] = None,
) -> bool
```

**Parameters:**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `positions` | `Dict[str, float]` or `Sequence[float]` | *required* | Target positions as dict or sequence. |
| `unit` | `"percent"` or `"rad"` | `"percent"` | Position unit. |
| `verify` | `bool` | `False` | Wait for joints to reach targets. |
| `verify_timeout` | `float` | `1.0` | Max wait time (seconds). |
| `verify_tolerance` | `float` or `None` | `None` | Acceptable error. |

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
) -> Optional[float]
```

**Parameters:**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `motor_name` | `str` | *required* | Motor name to query. |
| `unit` | `"percent"` or `"rad"` | `"percent"` | Return unit. |

**Returns:** `float` or `None` - Current position, or `None` if unavailable.

**Example:**

```python
with Robot(host="172.26.34.149") as robot:
    pos = robot.get_joint("elbow_left")
    print(f"Elbow at {pos:.1f}%")

    pos_rad = robot.get_joint("elbow_left", unit="rad")
    print(f"Elbow at {pos_rad:.3f} rad")
```

### get_joints()

Read multiple joint positions.

```python
def get_joints(
    self,
    motor_names: Optional[List[str]] = None,
    unit: Literal["percent", "rad"] = "percent",
) -> Dict[str, float]
```

**Parameters:**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `motor_names` | `List[str]` or `None` | `None` | Motors to query. `None` returns all. |
| `unit` | `"percent"` or `"rad"` | `"percent"` | Return unit. |

**Returns:** `Dict[str, float]` - Motor names mapped to positions.

**Example:**

```python
with Robot(host="172.26.34.149") as robot:
    # Get specific joints
    arm = robot.get_joints([
        "shoulder_vertical_left",
        "elbow_left",
        "wrist_left",
    ])

    # Get all joints (for saving pose)
    all_joints = robot.get_joints()
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
from pib_ik import Robot, Trajectory

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
from pib_ik import Robot

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

!!! warning "Position Reading is None"
    **Cause:** Motor feedback not received yet.

    **Solution:** Wait briefly after connecting:

    ```python
    import time

    robot = Robot(host="172.26.34.149")
    robot.connect()
    time.sleep(1.0)  # Wait for feedback
    pos = robot.get_joint("elbow_left")
    ```
