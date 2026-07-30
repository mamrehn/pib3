# Webots Simulator Backend

Run trajectories in the Webots robotics simulator.

## Overview

`WebotsBackend` integrates with the Webots simulator, allowing you to test trajectories in a physics-based simulation environment.

## WebotsBackend Class

::: pib3.backends.webots.WebotsBackend
    options:
      show_root_heading: true
      show_source: false
      members: false

---

## Quick Start

```python
# In your Webots controller file:
from pib3.backends import WebotsBackend

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
from pib3.backends import WebotsBackend

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
from pib3.backends import WebotsBackend

backend = WebotsBackend()
backend.connect()  # Initializes robot and motors
# ... use backend ...
backend.disconnect()
```

### Using Context Manager

The recommended way to use the Webots backend:

```python
from pib3 import Joint
from pib3.backends import WebotsBackend

with WebotsBackend() as backend:
    # Robot automatically initialized
    backend.set_joint(Joint.ELBOW_LEFT, 50.0)
# Cleanup handled automatically
```

---

## Joint Control

The Webots backend inherits all methods from [`RobotBackend`](base.md). Key methods:

### set_joint()

Set a single joint position. Inherits from [`RobotBackend`](base.md#set_joint) — see the base class for the full parameter list (including `speed` and the `"deg"` unit).

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

**Parameters:**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `motor_name` | `str` or `Joint` | *required* | Motor name or `Joint` enum (e.g., `Joint.ELBOW_LEFT`). |
| `position` | `float` | *required* | Target position (0-100 for percent, radians for rad, degrees for deg). |
| `unit` | `"percent"`, `"rad"`, `"deg"` | `"percent"` | Position unit. |
| `async_` | `bool` | `False` | If `True`, return immediately. If `False` (default), step simulation until motor reaches target. |
| `timeout` | `float` | `2.0` | Max wait time (only used when `async_=False`). |
| `tolerance` | `float` or `None` | `None` | Acceptable error (2.0%, 3.0°, or 0.05 rad default). |
| `speed` | `float` or `None` | `None` | Movement speed in deg/s. |

**Returns:** `bool` - `True` if successful.

**Example:**

```python
from pib3 import Joint

with WebotsBackend() as robot:
    # Set individual joints
    robot.set_joint(Joint.ELBOW_LEFT, 50.0)  # 50%

    # Using radians
    robot.set_joint(Joint.ELBOW_LEFT, 1.25, unit="rad")
```

### set_joints()

Set multiple joint positions simultaneously. Takes a plain dict — for hand-pose presets use [`set_joints_pose()`](base.md#set_joints_pose); for a sequence of waypoints use [`set_joints_sequence()`](base.md#set_joints_sequence).

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

**Parameters:**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `positions` | `Dict[str\|Joint, float]` | *required* | Target positions. Passing a `HandPose` or a plain sequence raises `TypeError`. |
| `unit` | `"percent"`, `"rad"`, `"deg"` | `"percent"` | Position unit. |
| `async_` | `bool` | `False` | If `True`, return immediately. If `False` (default), step simulation until motors reach targets. |
| `timeout` | `float` | `2.0` | Max wait time (only used when `async_=False`). |
| `tolerance` | `float` or `None` | `None` | Acceptable error. |
| `speed` | `float` or `None` | `None` | Movement speed in deg/s. |

**Example:**

```python
from pib3 import Joint

with WebotsBackend() as robot:
    robot.set_joints({
        Joint.SHOULDER_VERTICAL_LEFT: 30.0,
        Joint.SHOULDER_HORIZONTAL_LEFT: 40.0,
        Joint.ELBOW_LEFT: 60.0,
    })
```

### get_joint()

Read a single joint position. Waits for motor readings to stabilize (same value twice) before returning.

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
| `motor_name` | `str` or `Joint` | *required* | Motor name or `Joint` enum to query. |
| `unit` | `"percent"`, `"rad"`, `"deg"` | `"percent"` | Return unit. |
| `timeout` | `float` or `None` | `5.0` | Max time to wait for motor reading to stabilize (seconds). |

**Returns:** `float` or `None` - Current position, or `None` if unavailable or motor still moving.

**Example:**

```python
from pib3 import Joint

with WebotsBackend() as robot:
    # Uses default 5s timeout for stabilization
    pos = robot.get_joint(Joint.ELBOW_LEFT)
    print(f"Elbow at {pos:.1f}%")

    # Shorter timeout
    pos = robot.get_joint(Joint.ELBOW_LEFT, timeout=1.0)
```

### get_joints()

Read multiple joint positions. Waits for each motor reading to stabilize before returning.

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
| `motor_names` | `List[str\|Joint]` or `None` | `None` | Motors to query. `None` returns all. |
| `unit` | `"percent"`, `"rad"`, `"deg"` | `"percent"` | Return unit. |
| `timeout` | `float` or `None` | `5.0` | Max time to wait for each motor reading to stabilize (seconds). |

**Returns:** `Dict[str, float]` - Motor names mapped to positions.

---

## Simulated Perception

The pib proto carries a `Camera` mounted on `urdf_camera_link`, a child of the head. Its view therefore follows `turn_head_motor` and `tilt_forward_motor` — which is what makes visual servoing in simulation a genuinely **closed** loop, rather than a controller driving against a picture that never changes.

`WebotsBackend` exposes the same `camera` and `ai` contract as `RealRobotBackend`, so perception code ports between the two unchanged.

### step()

```python
def step(self, duration_ms: Optional[int] = None) -> bool
```

Advance the simulation by one time step. Returns `False` when Webots asks the controller to terminate, so it reads naturally as a loop condition.

!!! warning "Perception loops must step"
    Motion calls step the simulator themselves, but a loop that only *reads* does not. The camera renders a new image only when simulated time moves forward — without `step()` you get the same frame forever and the loop spins on stale data.

### camera

```python
frame = sim.camera.get_frame()      # CameraFrame, or None before first render
img   = frame.to_numpy()            # HxWx3 BGR, ready for OpenCV
```

Webots delivers BGRA, so the backend drops alpha and hands back BGR with no encode/decode round-trip — `CameraFrame.from_numpy()` carries the array directly and `jpeg_bytes` stays empty. Call `frame.to_jpeg()` if you need encoded bytes.

There is no buffer: Webots renders synchronously, so `get_frame()` always describes the current step. `timeout` is accepted for API compatibility and ignored, and `configure()` is a no-op — resolution and field of view belong to the `Camera` node in the proto.

### ai

Webots gives only RGB pixels, so `sim.ai` runs equivalent (or newer) models on the host and emits **the same payload dicts** the robot publishes on `/camera/ai/detections`. Those payloads go through the same `AIDetectionReceiver` and parser as real ones, so the typed results are identical — `Detection`, `HandLandmarks`, `PoseKeypoints`, buffering, `fps`, `avg_latency_ms`, `latest_only`.

The topologies match by construction, which is what makes this a substitution rather than an approximation:

| pib3 type | Convention | Simulated with |
|---|---|---|
| `PoseKeypoints` | 17 COCO keypoints | ultralytics `*-pose` |
| `HandLandmarks` | 21 MediaPipe hand landmarks | `mediapipe` Hands |
| `Detection` | normalized xyxy + class id | ultralytics detect / seg |

Install the optional backends with `pip install "pib3[sim]"`.

```python
sim.ai.set_model("pose")                       # -> yolo11n-pose.pt
for p in sim.ai.get_poses(latest_only=True):
    print(p.left_shoulder, p.nose)             # same code as on the robot

sim.ai.set_model("hand")                       # -> MediaPipe Hands
for h in sim.ai.get_hand_landmarks(latest_only=True):
    print(h.handedness, h.finger_angles.index)

sim.ai.set_model("yolov8n-seg")                # masks, RLE-encoded like the robot
```

`AIModel` names are mapped onto available weights by `SIM_MODEL_ALIASES` — where the OAK-D blob has no host equivalent, the closest current model is substituted (`mobilenet-ssd` and `yolov6n` → `yolo11n.pt`). `gaze` and `lines` have no simulated equivalent and raise a clear error.

`"recognition"` (the default) is the fourth source and needs no model at all: Webots ground truth via the `Recognition` node — exact boxes, `confidence` always `1.0`. Ideal for teaching downstream logic (debouncing, state machines, control) without perception noise in the way. Objects in the **world** must opt in:

- set `recognitionColors` on a Solid, otherwise it is never reported;
- the Solid's `model` field becomes `det.label`.

Inference runs at most once per rendered frame, so polling several getters within one `step()` costs one inference.

!!! warning "`set_model()` reports nothing until the next step"
    Enabling a Webots sensor never yields data in the same step, and
    `recognitionEnable()` is no exception. A `get_detections()` placed straight
    after `set_model()` therefore returns an empty list — and since results are
    cached per frame, that emptiness holds for the current frame too. Inside a
    `while sim.step():` loop this resolves itself; in a straight-line script,
    step first:

    ```python
    sim.ai.set_model("recognition")
    sim.step()                       # <- without this the first read is empty
    detections = sim.ai.get_detections()
    ```

!!! note "What synthetic imagery does and does not support"
    A COCO-trained detector sees very little in an untextured Webots world — that is the world, not a bug. Likewise, pose and hand models need something human-shaped in the scene to find; they are most useful for **developing and debugging the pipeline** without hardware, not for judging perception quality. Two things that help: texture your objects, or project a real photo/video onto a plane in front of the camera.

    Conversely, classical CV (HSV masks, contours) works *better* in simulation than in reality, because a saturated primary-colour object is a perfect blob.

!!! note "Latency is honest, and that is the point"
    `avg_latency_ms` reports real host-hardware timing, which is usually **slower** than the OAK-D's dedicated accelerator. Showing that gap is the whole argument for edge AI — do not read it as a defect of the simulation.

### Fixing the camera orientation

Webots cameras look along their own **+x** axis (+y left, +z up). The proto aims that axis out of the robot's face, but the composed frame of `urdf_camera_link` makes this easy to get wrong. If your first frame shows the inside of the head, the ceiling, or the floor, change **one line** — the `rotation` of the `Camera` node in `pib3/resources/pib.proto`:

```
Camera {
  name "camera"
  translation 0.000000 0.108232 0.095400
  rotation 0.577350 0.577350 0.577350 2.094395   # <- this line
  ...
}
```

Quickest way to check: run a controller that saves one frame, and confirm the view turns *with* the head.

```python
with pib3.Webots() as sim:
    for _ in range(10):
        sim.step()
    frame = sim.camera.get_frame()
    cv2.imwrite("view.png", frame.to_numpy())
    sim.set_joint(Joint.TURN_HEAD, 80.0)
    frame = sim.camera.get_frame()
    cv2.imwrite("view_turned.png", frame.to_numpy())
```

If the two images are identical, the camera is not on the head. If the scene looks wrong, adjust the rotation above.

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
from pib3.backends import WebotsBackend
from pib3 import Trajectory

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
from pib3.backends import WebotsBackend

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
from pib3 import Joint
from pib3.backends import WebotsBackend

with WebotsBackend() as robot:
    # Raise arm (default async_=False waits for motor to reach position)
    robot.set_joint(Joint.SHOULDER_VERTICAL_LEFT, 30.0)

    # Wave back and forth
    for _ in range(5):
        robot.set_joint(Joint.WRIST_LEFT, 20.0)
        robot.set_joint(Joint.WRIST_LEFT, 80.0)

    # Return to neutral
    robot.set_joint(Joint.WRIST_LEFT, 50.0)
    robot.set_joint(Joint.SHOULDER_VERTICAL_LEFT, 50.0)
```

!!! tip "The default `async_=False` replaces `time.sleep()`"
    In Webots, simulation time only advances when `robot.step()` is called internally.
    Using `time.sleep()` wastes wall-clock time without advancing the simulation.
    The default `async_=False` automatically steps the simulation until motors reach their targets.
    Pass `async_=True` to override and fire-and-forget.

### Complete Drawing Session

```python
from pib3.backends import WebotsBackend
import pib3

# Generate trajectory from image (outside Webots)
trajectory = pib3.generate_trajectory("drawing.png")
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

!!! warning "Motors Don't Reach Target Position"
    **Cause:** Explicitly passing `async_=True` only steps the simulation once.

    **Solution:** Drop the `async_=True` override — the default `async_=False` waits for motors to reach their targets:

    ```python
    # Wrong - returns immediately, motor barely moves
    robot.set_joint(Joint.ELBOW_LEFT, 50.0, async_=True)

    # Correct - default behavior waits for motor to reach position
    robot.set_joint(Joint.ELBOW_LEFT, 50.0)
    ```

    This also ensures code compatibility with the real robot, which behaves the same way.
