# Webots Simulator Backend

Run trajectories in the Webots robotics simulator.

## Overview

`WebotsBackend` integrates with the Webots simulator, allowing you to test trajectories in a physics-based simulation environment.

## WebotsBackend Class

::: pib_ik.backends.webots.WebotsBackend
    options:
      show_root_heading: true
      show_source: false
      members: false

---

## Quick Start

```python
# In your Webots controller file:
from pib_ik.backends import WebotsBackend

with WebotsBackend() as backend:
    backend.run_trajectory("trajectory.json")
```

!!! warning "Webots Environment Required"
    This backend must be instantiated from within a Webots controller script. It cannot be used standalone.

---

## Constructor

```python
WebotsBackend(step_ms: int = 50)
```

**Parameters:**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `step_ms` | `int` | `50` | Simulation time step per waypoint in milliseconds. Smaller values give smoother motion but slower playback. |

**Example:**

```python
from pib_ik.backends import WebotsBackend

# Default: 50ms per step
backend = WebotsBackend()

# Custom step timing (slower, smoother)
backend = WebotsBackend(step_ms=100)

# Faster playback
backend = WebotsBackend(step_ms=20)
```

---

## Connection

### connect()

Initialize Webots robot and motor devices.

```python
def connect(self) -> None
```

**Raises:**

- `ImportError`: If not running from within a Webots controller

**Example:**

```python
from pib_ik.backends import WebotsBackend

backend = WebotsBackend()
backend.connect()  # Initializes robot and motors
# ... use backend ...
backend.disconnect()
```

### Using Context Manager

The recommended way to use the Webots backend:

```python
from pib_ik.backends import WebotsBackend

with WebotsBackend() as backend:
    # Robot automatically initialized
    backend.set_joint("elbow_left", 50.0)
# Cleanup handled automatically
```

---

## Joint Control

The Webots backend inherits all methods from [`RobotBackend`](base.md). Key methods:

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
with WebotsBackend() as robot:
    # Set individual joints
    robot.set_joint("elbow_left", 50.0)  # 50%

    # Using radians
    robot.set_joint("elbow_left", 1.25, unit="rad")
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
with WebotsBackend() as robot:
    robot.set_joints({
        "shoulder_vertical_left": 30.0,
        "shoulder_horizontal_left": 40.0,
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
with WebotsBackend() as robot:
    pos = robot.get_joint("elbow_left")
    print(f"Elbow at {pos:.1f}%")
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

---

## Trajectory Execution

### run_trajectory()

Execute a trajectory in simulation.

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
from pib_ik.backends import WebotsBackend
from pib_ik import Trajectory

trajectory = Trajectory.from_json("trajectory.json")

def on_progress(current, total):
    print(f"\rProgress: {current}/{total}", end="")

with WebotsBackend() as robot:
    robot.run_trajectory(
        trajectory,
        rate_hz=20.0,
        progress_callback=on_progress,
    )
    print("\nDone!")
```

---

## Motor Mapping

The backend maps trajectory joint names to Webots motor device names:

| Trajectory Name | Webots Motor |
|-----------------|--------------|
| `turn_head_motor` | `head_horizontal` |
| `tilt_forward_motor` | `head_vertical` |
| `shoulder_vertical_left` | `shoulder_vertical_left` |
| `shoulder_horizontal_left` | `shoulder_horizontal_left` |
| `upper_arm_left_rotation` | `upper_arm_left` |
| `elbow_left` | `elbow_left` |
| `lower_arm_left_rotation` | `forearm_left` |
| `wrist_left` | `wrist_left` |
| `thumb_left_opposition` | `thumb_left_opposition` |
| `thumb_left_stretch` | `thumb_left_distal` |

Similar mappings exist for right arm and remaining fingers.

---

## Coordinate System

The canonical format uses **Webots motor radians** directly. No offset is needed:

```
Webots_position = Canonical_radians  (no offset)
```

Webots motors use sensible radian ranges (e.g., -π/2 to +π/2 for the head motor).

---

## Webots Project Setup

### Directory Structure

```
my_webots_project/
├── worlds/
│   └── pib_world.wbt
├── controllers/
│   └── pib_controller/
│       └── pib_controller.py
└── protos/
    └── pib.proto
```

### Controller Script Example

Create a controller file `pib_controller.py`:

```python
"""PIB robot controller for Webots."""
from pib_ik.backends import WebotsBackend

def main():
    with WebotsBackend() as robot:
        # Run a pre-generated trajectory
        robot.run_trajectory("trajectory.json")

if __name__ == "__main__":
    main()
```

### World File Configuration

In your `.wbt` world file, configure the robot to use your controller:

```
Robot {
  controller "pib_controller"
  ...
}
```

---

## Examples

### Wave Animation

```python
from pib_ik.backends import WebotsBackend
import time

with WebotsBackend(step_ms=50) as robot:
    # Raise arm
    robot.set_joint("shoulder_vertical_left", 30.0)

    # Wave back and forth
    for _ in range(5):
        robot.set_joint("wrist_left", 20.0)
        time.sleep(0.3)
        robot.set_joint("wrist_left", 80.0)
        time.sleep(0.3)

    # Return to neutral
    robot.set_joint("wrist_left", 50.0)
    robot.set_joint("shoulder_vertical_left", 50.0)
```

### Complete Drawing Session

```python
from pib_ik.backends import WebotsBackend
import pib_ik

# Generate trajectory from image (outside Webots)
trajectory = pib_ik.generate_trajectory("drawing.png")
trajectory.to_json("drawing_trajectory.json")

# Execute in Webots (in controller)
with WebotsBackend() as robot:
    robot.run_trajectory("drawing_trajectory.json", rate_hz=30.0)
```

---

## Troubleshooting

!!! warning "ImportError: No module named 'controller'"
    **Cause:** Not running from within Webots.

    **Solution:** This backend must be used in a Webots controller script. It cannot be run standalone from the command line.

!!! warning "Motor Not Found"
    **Cause:** Motor name mismatch between trajectory and Webots model.

    **Solution:** Check that the motor names in your Webots proto file match the expected names in `JOINT_TO_WEBOTS_MOTOR`.

!!! warning "Simulation Runs Too Fast/Slow"
    **Cause:** Step timing mismatch.

    **Solution:** Adjust the `step_ms` parameter or `rate_hz`:

    ```python
    # Slower, more accurate simulation
    backend = WebotsBackend(step_ms=100)

    # Faster simulation
    backend = WebotsBackend(step_ms=20)

    # Match rate_hz to timestep (for 50ms, use 20 Hz)
    robot.run_trajectory(trajectory, rate_hz=20.0)
    ```

!!! warning "Robot Doesn't Move Smoothly"
    **Cause:** Step timing too large.

    **Solution:** Use smaller `step_ms` values for smoother motion:

    ```python
    backend = WebotsBackend(step_ms=20)  # Smoother
    ```
