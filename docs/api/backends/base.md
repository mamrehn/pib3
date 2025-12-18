# Base Backend

Abstract base class defining the common interface for all robot control backends.

## Overview

All backends (Robot, Swift, Webots) inherit from `RobotBackend` and share a common API for joint control and trajectory execution.

```python
from pib_ik import Robot, Swift

# All backends support the same methods
with Swift() as backend:
    backend.set_joint("elbow_left", 50.0)
    pos = backend.get_joint("elbow_left")
    backend.run_trajectory("trajectory.json")
```

---

## RobotBackend Class

::: pib_ik.backends.base.RobotBackend
    options:
      show_root_heading: true
      show_source: false
      members: false

---

## Connection Methods

### connect()

Establish connection to the backend.

```python
def connect(self) -> None
```

Must be called before using any other methods. Alternatively, use the context manager.

**Example:**

```python
from pib_ik import Robot

# Manual connection
robot = Robot(host="172.26.34.149")
robot.connect()
try:
    robot.set_joint("elbow_left", 50.0)
finally:
    robot.disconnect()

# Context manager (recommended)
with Robot(host="172.26.34.149") as robot:
    robot.set_joint("elbow_left", 50.0)
```

---

### disconnect()

Close connection to the backend.

```python
def disconnect(self) -> None
```

Called automatically when using context manager.

---

### is_connected

Property indicating connection status.

```python
@property
def is_connected(self) -> bool
```

**Returns:** `True` if connected, `False` otherwise.

**Example:**

```python
robot = Robot(host="172.26.34.149")
print(robot.is_connected)  # False

robot.connect()
print(robot.is_connected)  # True

robot.disconnect()
print(robot.is_connected)  # False
```

---

## Reading Joint Positions

### get_joint()

Get the current position of a single joint.

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
| `motor_name` | `str` | *required* | Name of the motor to query. Must be one of the names in `MOTOR_NAMES`. |
| `unit` | `"percent"` or `"rad"` | `"percent"` | Unit for the returned value. `"percent"` returns 0-100% of the joint's calibrated range. `"rad"` returns raw radians. |
| `timeout` | `float` or `None` | `None` | Max time to wait for joint data (seconds). See note below. |

**Timeout Behavior by Backend:**

| Backend | Default | Behavior |
|---------|---------|----------|
| `RealRobotBackend` | 5.0s | Waits for ROS messages to arrive |
| `WebotsBackend` | 5.0s | Waits for motor readings to stabilize |
| `SwiftBackend` | ignored | Returns immediately (synchronous access) |

**Returns:** `float` or `None`

- Current position in the specified unit
- `None` if the position is unavailable (e.g., sensor not ready, timeout expired)

**Example:**

```python
from pib_ik import Robot

with Robot(host="172.26.34.149") as robot:
    # Get position (waits up to 5s by default)
    pos_percent = robot.get_joint("elbow_left")
    print(f"Elbow at {pos_percent:.1f}%")

    # Get position with custom timeout
    pos_rad = robot.get_joint("elbow_left", unit="rad", timeout=2.0)
    print(f"Elbow at {pos_rad:.3f} rad")
```

!!! note "Percentage Requires Calibration"
    The percentage unit requires calibrated joint limits. See [Calibration](../../getting-started/calibration.md).

---

### get_joints()

Get current positions of multiple joints.

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
| `motor_names` | `List[str]` or `None` | `None` | List of motor names to query. If `None`, returns all available joints. |
| `unit` | `"percent"` or `"rad"` | `"percent"` | Unit for the returned values. |
| `timeout` | `float` or `None` | `None` | Max time to wait for joint data (seconds). See `get_joint()` for backend-specific behavior. |

**Timeout Behavior by Backend:**

| Backend | Default | Behavior |
|---------|---------|----------|
| `RealRobotBackend` | 5.0s | Waits for ROS messages to arrive |
| `WebotsBackend` | 5.0s | Waits for motor readings to stabilize |
| `SwiftBackend` | ignored | Returns immediately (synchronous access) |

**Returns:** `Dict[str, float]`

- Dictionary mapping motor names to their current positions

**Example:**

```python
from pib_ik import Robot

with Robot(host="172.26.34.149") as robot:
    # Get all joints (waits up to 5s by default)
    all_positions = robot.get_joints()
    print(f"Got {len(all_positions)} joint positions")

    # Get specific joints with custom timeout
    arm_positions = robot.get_joints(
        ["shoulder_vertical_left", "elbow_left", "wrist_left"],
        timeout=2.0,
    )
    for name, pos in arm_positions.items():
        print(f"{name}: {pos:.1f}%")

    # Get in radians
    arm_rad = robot.get_joints(
        ["elbow_left", "wrist_left"],
        unit="rad"
    )
```

---

## Setting Joint Positions

### set_joint()

Set the position of a single joint.

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
| `motor_name` | `str` | *required* | Name of the motor to control. |
| `position` | `float` | *required* | Target position. For `"percent"`: 0.0 to 100.0. For `"rad"`: angle in radians. |
| `unit` | `"percent"` or `"rad"` | `"percent"` | Unit for the position value. |
| `verify` | `bool` | `False` | If `True`, wait and confirm the joint reached the target. |
| `verify_timeout` | `float` | `1.0` | Maximum seconds to wait for verification. |
| `verify_tolerance` | `float` or `None` | `None` | Acceptable error for verification. Default: 2.0% or 0.05 rad. |

**Returns:** `bool`

- `True` if the command was sent successfully (and position verified if `verify=True`)
- `False` if failed

**Example:**

```python
from pib_ik import Swift
import math

with Swift() as viz:
    # Set using percentage (recommended)
    viz.set_joint("elbow_left", 50.0)  # Move to 50%
    viz.set_joint("elbow_left", 0.0)   # Move to minimum
    viz.set_joint("elbow_left", 100.0) # Move to maximum

    # Set using radians
    viz.set_joint("elbow_left", math.pi / 4, unit="rad")  # 45 degrees

    # Set with verification (waits until joint reaches position)
    success = viz.set_joint(
        "elbow_left",
        75.0,
        verify=True,
        verify_timeout=2.0,
        verify_tolerance=3.0,  # Accept within 3%
    )
    if success:
        print("Joint reached target position")
    else:
        print("Joint did not reach target in time")
```

---

### set_joints()

Set positions of multiple joints simultaneously.

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
| `positions` | `Dict[str, float]` or `Sequence[float]` | *required* | Target positions. Either a dict mapping motor names to positions, or a sequence of 26 values in `MOTOR_NAMES` order. |
| `unit` | `"percent"` or `"rad"` | `"percent"` | Unit for position values. |
| `verify` | `bool` | `False` | If `True`, wait and confirm all joints reached their targets. |
| `verify_timeout` | `float` | `1.0` | Maximum seconds to wait for verification. |
| `verify_tolerance` | `float` or `None` | `None` | Acceptable error for verification. |

**Returns:** `bool`

- `True` if successful
- `False` if failed

**Example:**

```python
from pib_ik import Robot

with Robot(host="172.26.34.149") as robot:
    # Set multiple joints with a dictionary
    robot.set_joints({
        "shoulder_vertical_left": 30.0,
        "shoulder_horizontal_left": 40.0,
        "elbow_left": 60.0,
        "wrist_left": 50.0,
    })

    # Save and restore poses
    saved_pose = robot.get_joints()  # Save current pose

    # ... move robot around ...

    robot.set_joints(saved_pose)  # Restore saved pose

    # With verification
    success = robot.set_joints(
        {"elbow_left": 50.0, "wrist_left": 50.0},
        verify=True,
        verify_timeout=2.0,
    )
```

---

## Trajectory Execution

### run_trajectory()

Execute a trajectory on this backend.

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
| `trajectory` | `str`, `Path`, or `Trajectory` | *required* | The trajectory to execute. Can be a file path to a JSON file, or a `Trajectory` object. |
| `rate_hz` | `float` | `20.0` | Playback rate in Hz (waypoints per second). Higher values = faster playback. |
| `progress_callback` | `Callable[[int, int], None]` or `None` | `None` | Optional callback function called after each waypoint. Receives `(current_index, total_waypoints)`. |

**Returns:** `bool`

- `True` if trajectory completed successfully
- `False` if failed or interrupted

**Example:**

```python
from pib_ik import Swift, Trajectory

with Swift() as viz:
    # From file path
    viz.run_trajectory("my_trajectory.json")

    # From Trajectory object
    trajectory = Trajectory.from_json("my_trajectory.json")
    viz.run_trajectory(trajectory)

    # With custom playback rate
    viz.run_trajectory(trajectory, rate_hz=30.0)  # Faster

    # With progress callback
    def on_progress(current, total):
        percent = (current / total) * 100
        print(f"\rProgress: {percent:.1f}%", end="", flush=True)

    viz.run_trajectory(trajectory, progress_callback=on_progress)
    print()  # Newline after progress
```

---

## Available Motor Names

All 26 motors available on the PIB robot:

```python
from pib_ik.backends.base import RobotBackend

print(RobotBackend.MOTOR_NAMES)
```

### Head (2 motors)

| Motor Name | Description |
|------------|-------------|
| `turn_head_motor` | Head rotation (left/right) |
| `tilt_forward_motor` | Head tilt (up/down) |

### Left Arm (6 motors)

| Motor Name | Description |
|------------|-------------|
| `shoulder_vertical_left` | Shoulder raise/lower |
| `shoulder_horizontal_left` | Shoulder forward/back |
| `upper_arm_left_rotation` | Upper arm rotation |
| `elbow_left` | Elbow bend |
| `lower_arm_left_rotation` | Forearm rotation |
| `wrist_left` | Wrist bend |

### Left Hand (6 motors)

| Motor Name | Description |
|------------|-------------|
| `thumb_left_opposition` | Thumb rotation |
| `thumb_left_stretch` | Thumb bend |
| `index_left_stretch` | Index finger bend |
| `middle_left_stretch` | Middle finger bend |
| `ring_left_stretch` | Ring finger bend |
| `pinky_left_stretch` | Pinky finger bend |

### Right Arm (6 motors)

| Motor Name | Description |
|------------|-------------|
| `shoulder_vertical_right` | Shoulder raise/lower |
| `shoulder_horizontal_right` | Shoulder forward/back |
| `upper_arm_right_rotation` | Upper arm rotation |
| `elbow_right` | Elbow bend |
| `lower_arm_right_rotation` | Forearm rotation |
| `wrist_right` | Wrist bend |

### Right Hand (6 motors)

| Motor Name | Description |
|------------|-------------|
| `thumb_right_opposition` | Thumb rotation |
| `thumb_right_stretch` | Thumb bend |
| `index_right_stretch` | Index finger bend |
| `middle_right_stretch` | Middle finger bend |
| `ring_right_stretch` | Ring finger bend |
| `pinky_right_stretch` | Pinky finger bend |

---

## Unit System

All backends support two units for position values:

| Unit | Value Range | Description |
|------|-------------|-------------|
| `"percent"` | 0.0 to 100.0 | Percentage of calibrated joint range (default) |
| `"rad"` | varies | Raw angle in radians |

**Percentage** is recommended for most use cases:

- `0%` = joint at minimum position
- `50%` = joint at midpoint
- `100%` = joint at maximum position

```python
# These are equivalent ways to move to the middle
backend.set_joint("elbow_left", 50.0)               # 50%
backend.set_joint("elbow_left", 50.0, unit="percent")  # Explicit

# Use radians for precise angular control
import math
backend.set_joint("elbow_left", math.pi / 4, unit="rad")  # 45 degrees
```

---

## Available Backends

| Backend | Class | Import | Use Case |
|---------|-------|--------|----------|
| [Real Robot](robot.md) | `RealRobotBackend` | `from pib_ik import Robot` | Control physical PIB robot |
| [Swift](swift.md) | `SwiftBackend` | `from pib_ik import Swift` | Browser-based 3D visualization |
| [Webots](webots.md) | `WebotsBackend` | `from pib_ik.backends import WebotsBackend` | Webots physics simulation |
