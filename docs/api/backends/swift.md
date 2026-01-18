# Swift Visualization Backend

Browser-based 3D visualization for testing without hardware.

## Overview

`SwiftBackend` renders the PIB robot in a web browser using the Swift visualization library from the Robotics Toolbox.

## SwiftBackend Class

::: pib3.backends.swift.SwiftBackend
    options:
      show_root_heading: true
      show_source: false
      members: false

---

## Quick Start

```python
from pib3 import Swift

# Run a trajectory
with Swift() as viz:
    viz.run_trajectory("trajectory.json")

# Interactive mode with sliders
viz = Swift()
viz.connect()
viz.launch_interactive()  # Opens browser
```

---

## Constructor

```python
Swift(realtime: bool = True)
```

**Parameters:**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `realtime` | `bool` | `True` | If `True`, animations play at real-time speed. If `False`, visualization updates as fast as possible. |

**Example:**

```python
from pib3 import Swift

# Real-time playback (default)
viz = Swift(realtime=True)

# Fast playback (for testing)
viz = Swift(realtime=False)
```

---

## Connection

### connect()

Initialize the Swift visualization environment and open the browser.

```python
def connect(self) -> None
```

Opens a browser window with the 3D robot visualization at `http://localhost:52000`.

**Example:**

```python
from pib3 import Swift

viz = Swift()
viz.connect()  # Opens browser
# ... use visualization ...
viz.disconnect()
```

### Using Context Manager

The recommended way to use Swift:

```python
from pib3 import Swift, Joint

with Swift() as viz:
    # Browser opens automatically
    viz.set_joint(Joint.ELBOW_LEFT, 50.0)
# Browser closes on exit
```

---

## Joint Control

Swift inherits all methods from [`RobotBackend`](base.md). Key methods:

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
from pib3 import Swift, Joint
import time

with Swift() as viz:
    # Set positions
    viz.set_joint(Joint.TURN_HEAD, 0.0)    # Look left
    time.sleep(0.5)
    viz.set_joint(Joint.TURN_HEAD, 100.0)  # Look right
    time.sleep(0.5)
    viz.set_joint(Joint.TURN_HEAD, 50.0)   # Center
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
with Swift() as viz:
    viz.set_joints({
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
from pib3 import Joint

with Swift() as viz:
    pos = viz.get_joint(Joint.TURN_HEAD)
    print(f"Head at {pos:.1f}%")
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

Execute a trajectory with visualization.

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
from pib3 import Swift, Trajectory

trajectory = Trajectory.from_json("trajectory.json")

with Swift() as viz:
    # Basic execution
    viz.run_trajectory(trajectory)

    # With custom rate
    viz.run_trajectory(trajectory, rate_hz=30.0)

    # With progress callback
    def progress(current, total):
        print(f"\r{current}/{total}", end="")

    viz.run_trajectory(trajectory, progress_callback=progress)
```

---

## Interactive Mode

### launch_interactive()

Launch a web interface with slider controls for all joints.

```python
def launch_interactive(self, port: int = 8001) -> None
```

**Parameters:**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `port` | `int` | `8001` | Port for the slider control web server. |

Opens two browser windows:

- `http://localhost:52000` - 3D robot visualization
- `http://localhost:8001` - Slider controls

**Example:**

```python
from pib3 import Swift

viz = Swift()
viz.connect()
viz.launch_interactive(port=8001)

# Opens:
#   3D View: http://localhost:52000
#   Sliders: http://localhost:8001
#
# Press Ctrl+C to exit
```

### Slider Features

- All 26 joints controllable
- Preset poses (Zero, T-Pose)
- Real-time visualization updates
- Dark theme UI

---

## Adding Visual Elements

### add_paper()

Add a paper surface for drawing visualization.

```python
def add_paper(
    self,
    center_x: float = 0.15,
    center_y: float = 0.15,
    height_z: float = 0.74,
    size: float = 0.12,
) -> None
```

**Parameters:**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `center_x` | `float` | `0.15` | Paper center X position (meters, forward from robot). |
| `center_y` | `float` | `0.15` | Paper center Y position (meters, left of robot). |
| `height_z` | `float` | `0.74` | Paper surface Z position (meters, table height). |
| `size` | `float` | `0.12` | Paper size (meters, square). |

**Example:**

```python
from pib3 import Swift

with Swift() as viz:
    # Add paper at specific position
    viz.add_paper(
        center_x=0.15,    # Forward from robot
        center_y=0.15,    # Left of robot
        height_z=0.74,    # Table height
        size=0.12,        # 12cm square
    )

    # Run trajectory on paper
    viz.run_trajectory("trajectory.json")
```

---

## Visualization During IK

Watch inverse kinematics solving in real-time:

```python
import pib3

sketch = pib3.image_to_sketch("drawing.png")
trajectory = pib3.sketch_to_trajectory(
    sketch,
    visualize=True  # Opens Swift during IK solving
)
```

---

## Browser Access

| URL | Content |
|-----|---------|
| `http://localhost:52000` | 3D robot visualization |
| `http://localhost:8001` | Slider controls (interactive mode) |

---

## Examples

### Animation Sequence

```python
from pib3 import Swift, Joint
import time

with Swift() as viz:
    # Wave animation
    for i in range(3):
        viz.set_joint(Joint.ELBOW_LEFT, 20.0)
        time.sleep(0.3)
        viz.set_joint(Joint.ELBOW_LEFT, 80.0)
        time.sleep(0.3)

    # Return to neutral
    viz.set_joint(Joint.ELBOW_LEFT, 50.0)
```

### Pose Comparison

```python
import json
from pib3 import Swift

with Swift() as viz:
    # Load saved poses
    with open("pose1.json") as f:
        pose1 = json.load(f)
    with open("pose2.json") as f:
        pose2 = json.load(f)

    # Show pose 1
    viz.set_joints(pose1)
    input("Press Enter to show pose 2...")

    # Show pose 2
    viz.set_joints(pose2)
    input("Press Enter to exit...")
```

---

## Troubleshooting

!!! warning "Browser Shows Blank Page"
    **Cause:** Swift server not ready or port in use.

    **Solution:**

    1. Wait a few seconds and refresh the browser
    2. Check no other Swift instance is running
    3. Try a different browser

!!! warning "Module 'swift' Not Found"
    **Cause:** Visualization dependencies not installed.

    **Solution:** Install with the viz extra:

    ```bash
    pip install "pib3[viz] @ git+https://github.com/mamrehn/pib3.git"
    ```

!!! warning "Robot Model Not Visible"
    **Cause:** URDF file or mesh files missing.

    **Solution:**

    1. Verify URDF file exists in package
    2. Check browser console for errors
    3. Reinstall the package:

    ```bash
    pip uninstall pib3
    pip install "pib3[viz] @ git+https://github.com/mamrehn/pib3.git"
    ```

!!! warning "Sliders Not Updating Visualization"
    **Cause:** Mismatched Swift instances.

    **Solution:**

    1. Ensure both URLs are from same Swift instance
    2. Refresh the 3D view page
    3. Check terminal for error messages
