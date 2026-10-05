# Base Backend

Abstract base class defining the common interface for all robot control backends.

## Overview

All backends (Robot, Webots) inherit from `RobotBackend` and share a common API for joint control and trajectory execution.

```python
from pib3 import Robot, Joint

# All backends support the same methods
with Robot() as backend:
    backend.set_joint(Joint.ELBOW_LEFT, 50.0)
    pos = backend.get_joint(Joint.ELBOW_LEFT)
    backend.run_trajectory("trajectory.json")
```

---

## RobotBackend Class

::: pib3.backends.base.RobotBackend
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
from pib3 import Robot, Joint

# Manual connection
robot = Robot(host="172.26.34.149")
robot.connect()
try:
    robot.set_joint(Joint.ELBOW_LEFT, 50.0)
finally:
    robot.disconnect()

# Context manager (recommended)
with Robot(host="172.26.34.149") as robot:
    robot.set_joint(Joint.ELBOW_LEFT, 50.0)
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
    motor_name: Union[str, Joint],
    unit: Literal["percent", "rad", "deg"] = "percent",
    timeout: Optional[float] = None,
) -> Optional[float]
```

**Parameters:**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `motor_name` | `str` or `Joint` | *required* | Motor name or `Joint` enum. Must be one of the names in `MOTOR_NAMES`. |
| `unit` | `"percent"`, `"rad"`, or `"deg"` | `"percent"` | Unit for the returned value. `"percent"` returns 0-100% of the joint's calibrated range, `"rad"` returns raw radians, `"deg"` returns degrees. |
| `timeout` | `float` or `None` | `None` | Max time to wait for joint data (seconds). See note below. |

**Timeout Behavior by Backend:**

| Backend | Default | Behavior |
|---------|---------|----------|
| `RealRobotBackend` | 5.0s | Waits for ROS messages to arrive |
| `WebotsBackend` | 5.0s | Waits for motor readings to stabilize |


**Returns:** `float` or `None`

- Current position in the specified unit
- `None` if the position is unavailable (e.g., sensor not ready, timeout expired)

**Example:**

```python
from pib3 import Robot, Joint

with Robot(host="172.26.34.149") as robot:
    # Get position (waits up to 5s by default)
    pos_percent = robot.get_joint(Joint.ELBOW_LEFT)
    print(f"Elbow at {pos_percent:.1f}%")

    # Get position with custom timeout
    pos_rad = robot.get_joint(Joint.ELBOW_LEFT, unit="rad", timeout=2.0)
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
    motor_names: Optional[List[Union[str, Joint]]] = None,
    unit: Literal["percent", "rad", "deg"] = "percent",
    timeout: Optional[float] = None,
) -> Dict[str, float]
```

**Parameters:**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `motor_names` | `List[str\|Joint]` or `None` | `None` | List of motor names (strings or `Joint` enums) to query. If `None`, returns all available joints. |
| `unit` | `"percent"`, `"rad"`, or `"deg"` | `"percent"` | Unit for the returned values. |
| `timeout` | `float` or `None` | `None` | Max time to wait for joint data (seconds). See `get_joint()` for backend-specific behavior. |

**Timeout Behavior by Backend:**

| Backend | Default | Behavior |
|---------|---------|----------|
| `RealRobotBackend` | 5.0s | Waits for ROS messages to arrive |
| `WebotsBackend` | 5.0s | Waits for motor readings to stabilize |


**Returns:** `Dict[str, float]`

- Dictionary mapping motor names to their current positions

**Example:**

```python
from pib3 import Robot

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
    motor_name: Union[str, Joint],
    position: float,
    unit: Literal["percent", "rad", "deg"] = "percent",
    async_: bool = False,
    timeout: Optional[float] = None,
    tolerance: Optional[float] = None,
    speed: Optional[float] = None,
) -> bool
```

**Parameters:**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `motor_name` | `str` or `Joint` | *required* | Motor name or `Joint` enum. |
| `position` | `float` | *required* | Target position. For `"percent"`: 0.0 to 100.0. For `"rad"`: angle in radians. For `"deg"`: angle in degrees. |
| `unit` | `"percent"`, `"rad"`, or `"deg"` | `"percent"` | Unit for the position value. |
| `async_` | `bool` | `False` | If `True`, return immediately. If `False` (default), poll the actual position until it matches the target. |
| `timeout` | `float` | `2.0` | Maximum seconds to wait (only used when `async_=False`). |
| `tolerance` | `float` or `None` | `None` | Acceptable error for completion. Default: `DEFAULT_VERIFY_TOLERANCE_PERCENT` (2.0%), `DEFAULT_VERIFY_TOLERANCE_DEG` (3.0°), or `DEFAULT_VERIFY_TOLERANCE` (0.05 rad). |
| `speed` | `float` or `None` | `None` | Movement speed in degrees/second (e.g. `90.0` = 90°/s), must be > 0. `None` uses `robot.default_speed` (150 deg/s on the robot and in Webots). |

**Returns:** `bool`

- `True` if the command was sent successfully (and position reached if `async_=False`)
- `False` if failed

**Example:**

```python
from pib3 import Robot, Joint
import math

with Robot(host="172.26.34.149") as robot:
    # Set using percentage (recommended)
    robot.set_joint(Joint.ELBOW_LEFT, 50.0)  # Move to 50%
    robot.set_joint(Joint.ELBOW_LEFT, 0.0)   # Move to minimum
    robot.set_joint(Joint.ELBOW_LEFT, 100.0) # Move to maximum

    # Set using radians
    robot.set_joint(Joint.ELBOW_LEFT, math.pi / 4, unit="rad")  # 45 degrees

    # Wait for completion
    success = robot.set_joint(
        Joint.ELBOW_LEFT,
        75.0,
        async_=False,
        timeout=2.0,
        tolerance=3.0,  # Accept within 3%
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
    positions: Dict[Union[str, Joint], float],
    unit: Literal["percent", "rad", "deg"] = "percent",
    async_: bool = False,
    timeout: Optional[float] = None,
    tolerance: Optional[float] = None,
    speed: Optional[float] = None,
) -> bool
```

**Parameters:**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `positions` | `Dict[str\|Joint, float]` | *required* | Dict mapping motor names (string or `Joint` enum) to target positions. A plain dict is **required** — passing a `HandPose` raises `TypeError` (use [`set_joints_pose()`](#set_joints_pose) instead); passing a `Sequence[float]` raises `TypeError` (use [`set_joints_sequence()`](#set_joints_sequence) instead). |
| `unit` | `"percent"`, `"rad"`, or `"deg"` | `"percent"` | Unit for position values. |
| `async_` | `bool` | `False` | If `True`, return immediately. If `False` (default), poll actual positions until they match targets. |
| `timeout` | `float` | `2.0` | Maximum seconds to wait (only used when `async_=False`). |
| `tolerance` | `float` or `None` | `None` | Acceptable error for completion. |
| `speed` | `float` or `None` | `None` | Movement speed in degrees/second. `None` uses the backend's default. |

!!! note "Every command sets its own speed"
    `speed=None` means `robot.default_speed`, not "whatever the last call
    left behind". Earlier versions kept the last speed on the Tinkerforge
    channel, so after the slow `go_home()` every later move crawled at
    10 deg/s. Slow down a whole program with `robot.default_speed = 45`.

!!! info "Timeouts, limits and the return value"
    `timeout=None` derives the wait from the speed and the joint's range, so
    slow moves no longer "fail" after a fixed 2 s. Targets outside a joint's
    range are clamped to the limit (with a one-time hint) and a blocking call
    then returns `False`. Unknown joint names and units raise `ValueError`
    with a suggestion; `unit` accepts `"deg"`, `"degree"`, `"degrees"`,
    `"rad"`, `"%"`, ...

??? note "Before pib3 0.2: `speed` was shared state on the Tinkerforge direct path"
    On the real-robot direct path, `speed` rewrote the servo channel's
    motion configuration, which is **shared state**. A later call (even
    an `async_=True` one) that passed a different `speed` changed
    the channel's velocity; subsequent calls that omitted `speed` kept
    using the last value.

**Returns:** `bool`

- `True` if successful
- `False` if failed

**Example:**

```python
from pib3 import Robot, Joint

with Robot(host="172.26.34.149") as robot:
    # Set multiple joints with a dictionary
    robot.set_joints({
        Joint.SHOULDER_VERTICAL_LEFT: 30.0,
        Joint.SHOULDER_HORIZONTAL_LEFT: 40.0,
        Joint.ELBOW_LEFT: 60.0,
        Joint.WRIST_LEFT: 50.0,
    })

    # Save and restore poses
    saved_pose = robot.get_joints()  # Save current pose

    # ... move robot around ...

    robot.set_joints(saved_pose)  # Restore saved pose

    # Fire-and-forget
    robot.set_joints(
        {Joint.ELBOW_LEFT: 50.0, Joint.WRIST_LEFT: 50.0},
        async_=True,
    )

    # Slow motion at 45 deg/s
    robot.set_joints({Joint.ELBOW_LEFT: 50.0}, speed=45.0)
```

---

### set_joints_pose()

Apply a [`HandPose`](../hand-poses.md) preset to the robot.

```python
def set_joints_pose(
    self,
    pose: HandPose,
    async_: bool = False,
    timeout: Optional[float] = None,
    tolerance: Optional[float] = None,
    speed: Optional[float] = None,
) -> bool
```

**Parameters:**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `pose` | `HandPose` | *required* | A `HandPose` enum member (e.g. `HandPose.LEFT_OPEN`). Raises `TypeError` for any other type. |
| `async_` | `bool` | `False` | If `True`, fire-and-forget. If `False` (default), wait for completion. |
| `timeout` | `float` | `2.0` | Max wait time when `async_=False`. |
| `tolerance` | `float` or `None` | `None` | Position tolerance (in percent — pose values are always percent). |
| `speed` | `float` or `None` | `None` | Movement speed in degrees/second. |

**Example:**

```python
from pib3 import Robot, HandPose

with Robot(host="172.26.34.149") as robot:
    robot.set_joints_pose(HandPose.LEFT_CLOSED)
    robot.set_joints_pose(HandPose.RIGHT_OPEN, speed=45.0)
```

---

### set_joints_sequence()

Play a sequence of joint-position waypoints at a fixed rate. Lightweight sibling to `run_trajectory()` — no IK, no interpolation, no file format.

```python
def set_joints_sequence(
    self,
    sequence: Sequence[Dict[Union[str, Joint], float]],
    unit: Literal["percent", "rad", "deg"] = "percent",
    rate_hz: float = 20.0,
    progress_callback: Optional[Callable[[int, int], None]] = None,
    speed: Optional[float] = None,
) -> bool
```

**Parameters:**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `sequence` | `Sequence[Dict[str\|Joint, float]]` | *required* | Iterable of waypoint dicts. Joints omitted in a given waypoint stay at their last commanded value. |
| `unit` | `"percent"`, `"rad"`, `"deg"` | `"percent"` | Unit for all positions. |
| `rate_hz` | `float` | `20.0` | Waypoint dispatch rate. |
| `progress_callback` | `Callable[[int, int], None]` or `None` | `None` | Optional `callback(current_index, total)`. |
| `speed` | `float` or `None` | `None` | Per-waypoint movement speed (deg/s); `None` = `default_speed`. |

Waypoints go out on a fixed schedule (command time does not add up). Stops early and returns `False` on an [emergency stop](#emergency-stop).

**Example:**

```python
from pib3 import Joint

backend.set_joints_sequence([
    {Joint.ELBOW_LEFT: 0.0},
    {Joint.ELBOW_LEFT: 50.0},
    {Joint.ELBOW_LEFT: 0.0},
], rate_hz=4.0)
```

---

## Home Position

### home_percent

Property returning the home position (0 radians per joint) expressed as percentages.

```python
@property
def home_percent(self) -> Dict[str, float]
```

For symmetric joints the value is ~50%, but asymmetric joints (e.g. elbow: −45° to +90°) will differ.

### go_home()

Move all joints to their home position (0 radians — Webots proto zero / real-robot servo midpoint).

```python
def go_home(self, async_: bool = False, timeout: Optional[float] = None, speed: Optional[float] = None) -> bool
```

Equivalent to `set_joints({...: 0.0}, unit="rad")` for every motor in `MOTOR_NAMES`.

!!! warning "Homing starts from an unknown pose"
    The real robot powers on with its servos **un-driven** — the arms hang loose — so homing may swing every joint through its full range at once. `RealRobotBackend` therefore homes *very* slowly: `DEFAULT_HOME_SPEED` is 10 deg/s, against ~150 deg/s for normal motion, so the worst case (90° to zero) takes about 9 seconds. Clear the arm radius before calling it.

`speed=None` (default) uses the backend's `DEFAULT_HOME_SPEED`; `WebotsBackend` leaves this at `None` (simulation starts already at zero, so there is nothing to swing).

```python
with backend as robot:
    robot.go_home()             # all joints to 0 rad, at the safe homing speed
    robot.go_home(speed=90.0)   # deliberately faster
```

---

## Emergency Stop

See the [Safety page](../../getting-started/safety.md) for the classroom view.
In short: the stop **latches**, freezes the motors where they are, and every
later motion command raises `EmergencyStopError` until `resume()`.

### stop() / resume() / stopped / stop_reason

```python
def stop(self, reason: Optional[str] = None) -> None  # freeze + latch, any thread
def resume(self) -> None                              # deliberate release
@property
def stopped(self) -> bool
@property
def stop_reason(self) -> Optional[str]
```

`stop()` freezes every servo at its current ramp position (deceleration
briefly 0, so the bricklet does not brake through a 75° overshoot), ends any
blocking wait, `run_trajectory()` or `set_joints_sequence()` at once (they
return `False`), and makes later motion commands raise
`pib3.EmergencyStopError`. Pressing a stop key again does **not** resume.

A program that ends with an exception or Ctrl+C inside `with Robot(...)`
freezes the motors as well; a normal end lets the last moves finish.

### enable_estop_key() / disable_estop_key() / estop_keys

```python
def enable_estop_key(self, keys: Union[None, str, Sequence[str]] = None) -> bool
def disable_estop_key(self) -> None
@property
def estop_keys(self) -> Tuple[str, ...]
```

Default keys: **Space, Esc, Numpad-0, Pause** (`pib3.DEFAULT_STOP_KEYS`);
pass a name or list for others (`"f12"`, `"enter"`, single characters).
The real robot arms them on connect (`Robot(estop_keys=...)`).

Returns `False` and says why when the global keyboard hook cannot work:
macOS without *Input Monitoring* permission, Linux under Wayland, no
display. One process-wide `pynput` listener serves all robot objects.

!!! bug "Fixed in 0.2: Numpad-0 never worked"
    The old default `KeyCode.from_vk(96)` never compared equal to a real key
    press on Windows or Linux (pynput also compares the scan code / X11
    symbol), and on macOS it matched **F5** instead of Numpad-0.

### show_stop_button() / hide_stop_button()

```python
def show_stop_button(self, title: Optional[str] = None) -> bool
def hide_stop_button(self) -> None
```

A big red STOP window (separate process, always on top). Click it with a
touchpad, or press Space/Esc/Enter while it has focus. The real robot opens
it by itself when the keys cannot work (`Robot(stop_button="auto")`).

### Ctrl+C

While a robot is connected, Ctrl+C first freezes the motors and then raises
the usual `KeyboardInterrupt`. That works on every system without any
permission (`RobotBackend.STOP_ON_CTRL_C`).

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
    approach_speed: Optional[float] = None,
) -> bool
```

Before streaming, the arm moves to the first waypoint at `approach_speed`
(default `DEFAULT_APPROACH_SPEED` = 30 deg/s; `0` skips it) and waits until it
is there, so the start of a drawing is not smeared. Waypoints outside the
joint limits are clamped with one warning.

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
from pib3 import Robot, Trajectory

with Robot(host="172.26.34.149") as robot:
    # From file path
    robot.run_trajectory("my_trajectory.json")

    # From Trajectory object
    trajectory = Trajectory.from_json("my_trajectory.json")
    robot.run_trajectory(trajectory)

    # With custom playback rate
    robot.run_trajectory(trajectory, rate_hz=30.0)  # Faster

    # With progress callback
    def on_progress(current, total):
        percent = (current / total) * 100
        print(f"\rProgress: {percent:.1f}%", end="", flush=True)

    robot.run_trajectory(trajectory, progress_callback=on_progress)
    print()  # Newline after progress
```

---

## Available Motor Names

All 26 motors available on the PIB robot:

```python
from pib3.backends.base import RobotBackend

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

All backends support three units for position values:

| Unit | Value Range | Description |
|------|-------------|-------------|
| `"percent"` | 0.0 to 100.0 | Percentage of calibrated joint range (default) |
| `"rad"` | varies | Raw angle in radians |
| `"deg"` | varies | Raw angle in degrees |

**Percentage** is recommended for most use cases:

- `0%` = joint at minimum position
- `50%` = joint at midpoint
- `100%` = joint at maximum position

```python
from pib3 import Joint
import math

# These are equivalent ways to move to the middle
backend.set_joint(Joint.ELBOW_LEFT, 50.0)                  # 50%
backend.set_joint(Joint.ELBOW_LEFT, 50.0, unit="percent")  # Explicit

# Use radians or degrees for precise angular control
backend.set_joint(Joint.ELBOW_LEFT, math.pi / 4, unit="rad")  # 45 degrees
backend.set_joint(Joint.ELBOW_LEFT, -30.0, unit="deg")        # -30 degrees
```

---

## Available Backends

| Backend | Class | Import | Use Case |
|---------|-------|--------|----------|
| [Real Robot](robot.md) | `RealRobotBackend` | `from pib3 import Robot` | Control physical PIB robot |

| [Webots](webots.md) | `WebotsBackend` | `from pib3.backends import WebotsBackend` | Webots physics simulation |
