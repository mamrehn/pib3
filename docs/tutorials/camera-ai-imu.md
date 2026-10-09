# Camera, AI Detection, and IMU

Access the OAK-D Lite camera, AI inference, and IMU sensors through the pib3 API.

## Objectives

By the end of this tutorial, you will:

- Stream camera images from the OAK-D Lite
- Run AI models on the camera: object detection, body pose, hand landmarks
- Switch models, and run several at once
- Access IMU accelerometer and gyroscope data
- Understand how models are shared between clients

## Prerequisites

- pib3 installed: `pip install "pib3 @ git+https://github.com/mamrehn/pib3.git"`
- A PIB robot with OAK-D Lite camera connected
- Rosbridge running on the robot (port 9090)
- The models in the robot's model store; for the YOLO26 models of this tutorial see the note in [AI & Camera Subsystems](../api/ai-camera-subsystems.md#aimodel-enum)

---

## Key Concept: Models Run on Request, and Are Shared

The camera node of the robot runs AI models in its OAK-D Lite on request:

- **Image and IMU streams** only run while someone is subscribed.
- **A model** runs from `/start_model` until the last client that asked for it calls `/stop_model`. Each client names itself (the *owner*), so two scripts, cerebra and the web programs can use the same model without switching it away from each other.
- **Every start or stop rebuilds the camera pipeline**: video and IMU pause for a few seconds.
- **Depth** is the camera's resting state. It runs while no model runs and is gone while one does, because both need the camera's cores.

`robot.ai` does the bookkeeping: `robot.ai.set_model(...)` starts a model and releases the others this client started, `robot.ai.stop()` releases everything (it also runs when the `with` block ends).

```python
# Image stream: subscribe -> frames arrive, unsubscribe -> they stop
sub = robot.subscribe_camera_image(callback)
sub.unsubscribe()

# A model: start -> results arrive, stop -> the robot may free the cores
robot.ai.set_model(AIModel.YOLO26N)
robot.ai.stop()
```

---

## Quick Start: Subsystem API (Recommended)

For most use cases, use the simplified subsystem APIs instead of raw subscriptions. The subsystems automatically manage subscriptions and provide typed results.

### AI Subsystem (`robot.ai`)

```python
from pib3 import Robot, AIModel

with Robot(host="192.168.178.71") as robot:
    # Set AI model (waits for confirmation)
    robot.ai.set_model(AIModel.YOLO26N)
    
    # Get detections (waits automatically for results)
    for det in robot.ai.get_detections(latest_only=True):
        print(f"{det.label}: {det.confidence:.0%} at {det.bbox.center}")
    
    # Check performance
    print(f"FPS: {robot.ai.fps:.1f}")
```

### Hand Tracking with Servo Control

```python
from pib3 import Robot, AIModel

with Robot(host="192.168.178.71") as robot:
    robot.ai.set_model(AIModel.HAND)
    
    for hand in robot.ai.get_hand_landmarks(latest_only=True):
        print(f"{hand.handedness}: index={hand.finger_angles.index:.0f}°")
        
        # Convert finger angles to robot servo percentages
        servos = hand.finger_angles.to_servo_values()
        
        # Mirror to robot hand
        if hand.handedness.value == "left":
            robot.set_joints({
                "index_left_stretch": servos["index"],
                "middle_left_stretch": servos["middle"],
            })
```

### Camera Subsystem (`robot.camera`)

```python
from pib3 import Robot

with Robot(host="192.168.178.71") as robot:
    # Get latest frame
    frame = robot.camera.get_frame()
    if frame:
        print(f"Frame {frame.frame_id}: {len(frame.jpeg_bytes)} bytes")
        
        # Decode to numpy (requires OpenCV)
        img = frame.to_numpy()
        
        # Configure camera
        robot.camera.configure(fps=10, quality=80)
```

!!! tip "API Reference"
    For complete details on subsystem methods and types, see the 
    [AI & Camera Subsystems](../api/ai-camera-subsystems.md) API reference.

---

## The Same Code in Simulation

The simulated pib has a camera **in its head**, so `sim.camera` and `sim.ai`
offer the same contract as above. Because the head carries the camera, turning
the head changes what the robot sees — visual servoing in simulation is a real
closed loop, not a controller pushing against a static picture.

```python
import pib3
from pib3 import Joint

with pib3.Webots() as sim:               # inside a Webots controller
    sim.ai.set_model("recognition")      # the default source

    while sim.step():                    # step() renders the next frame
        img = sim.camera.get_frame().to_numpy()      # BGR, same as the robot
        for det in sim.ai.get_detections():
            x, _ = det.bbox.center
            sim.set_joint(Joint.TURN_HEAD, 50 + 60 * (x - 0.5), async_=True)
```

Four differences from the real robot:

| | Real robot | Simulation |
|---|---|---|
| Frame source | MJPEG over rosbridge, buffered | rendered per step, one frame cached |
| Loop driver | frames arrive on their own | **you must call `sim.step()`** |
| Inference | on the OAK-D's accelerator | host CPU/GPU, or ground truth |
| Hand / pose | on-device models | needs `pib3[sim]`; see below |

!!! warning "A perception loop must call `sim.step()`"
    Motion calls step the simulator internally, but a loop that only reads does
    not. Without a step the camera keeps handing back the same frame and the
    loop spins on stale data. `sim.step()` returns `False` on shutdown, so it
    reads naturally as the loop condition.

    The same rule bites once more at startup: **`set_model()` only takes effect
    on the following step.** Enabling a Webots sensor never yields data in the
    same step, so a `get_detections()` placed immediately after `set_model()`
    returns an empty list — and since results are cached per frame, that
    emptiness persists for the current frame. Step first, then read:

    ```python
    sim.ai.set_model("recognition")
    sim.step()                        # <- without this the first read is empty
    detections = sim.ai.get_detections()
    ```

    In the `while sim.step():` loop above this is automatic. It only shows up
    in straight-line scripts.

### Choosing a perception source

`sim.ai.set_model()` picks between simulator ground truth and a real network:

- **`"recognition"`** (default) — Webots reports objects directly: exact boxes,
  `confidence` always `1.0`, no model and no inference cost. Ideal for teaching
  the *downstream* logic (debouncing, state machines, control) without
  perception noise in the way.
- **`AIModel.YOLO26N`, `AIModel.POSE_YOLO`, `AIModel.HAND`** — runs ultralytics or
  mediapipe on the simulated frames and emits the same message the robot
  publishes, so results come back as the same typed `Detection` /
  `PoseKeypoints` / `HandLandmarks`. Models run together, as on the robot.
  Install with `pip install "pib3[sim] @ git+https://github.com/mamrehn/pib3.git"`.

!!! note "Objects must opt into recognition"
    A Solid is reported only if it sets `recognitionColors`, and its `model`
    field becomes `det.label`:

    ```
    Solid {
      translation 0 -0.6 1.0
      children [ Shape {
        appearance PBRAppearance { baseColor 1 0 0 roughness 1 metalness 0 }
        geometry Sphere { radius 0.06 }
      } ]
      name "testball"
      model "ball"
      recognitionColors [ 1 0 0 ]
    }
    ```

    A COCO-trained detector, by contrast, sees very little in an untextured
    synthetic scene — that is the world, not a bug.

Runnable example: [`examples/webots_camera_view.py`](https://github.com/mamrehn/pib3/blob/main/examples/webots_camera_view.py).
If the camera misbehaves, [`examples/webots_camera_check.py`](https://github.com/mamrehn/pib3/blob/main/examples/webots_camera_check.py)
diagnoses the device, its mounting and the recognition setup step by step.

---

## Low-Level API: Raw Subscriptions

The following sections cover the raw subscription API for advanced use cases 
(custom buffering, multiple callbacks, etc.).

---

## Camera Streaming

### Basic Camera Streaming

The camera publishes JPEG frames (base64 text on `/camera_topic`). The callback receives raw JPEG bytes:

```python
from pib3 import Robot
import cv2
import numpy as np

def on_frame(jpeg_bytes):
    """Callback receives raw JPEG bytes."""
    # Decode JPEG to numpy array
    img_array = np.frombuffer(jpeg_bytes, dtype=np.uint8)
    frame = cv2.imdecode(img_array, cv2.IMREAD_COLOR)

    # Process frame...
    print(f"Frame shape: {frame.shape}")

    # Display (optional)
    cv2.imshow("Camera", frame)
    cv2.waitKey(1)

with Robot(host="192.168.178.71") as robot:
    # Subscribe to camera (streaming starts)
    sub = robot.subscribe_camera_image(on_frame)

    # Keep running for 10 seconds
    import time
    time.sleep(10)

    # Unsubscribe (streaming stops)
    sub.unsubscribe()
    cv2.destroyAllWindows()
```

### Using PIL for Image Processing

```python
from PIL import Image
import io

def on_frame(jpeg_bytes):
    """Process frames with PIL."""
    img = Image.open(io.BytesIO(jpeg_bytes))
    print(f"Image size: {img.size[0]}x{img.size[1]} pixels")

    # Save frame
    img.save("captured_frame.jpg")

with Robot(host="192.168.178.71") as robot:
    sub = robot.subscribe_camera_image(on_frame)
    time.sleep(5)
    sub.unsubscribe()
```

### Configuring Camera Settings

Adjust FPS, quality, and resolution:

```python
with Robot(host="192.168.178.71") as robot:
    # Set individual parameters
    robot.set_camera_config(fps=10)
    robot.set_camera_config(quality=80)
    robot.set_camera_config(resolution=(1280, 720))

    # Or set multiple at once
    robot.set_camera_config(
        fps=10,
        quality=80,
        resolution=(1280, 720)
    )
```

!!! warning "Keep the frame 16:9"
    The camera publishes 1280×720 by default. AI models read the same frame,
    and the camera refuses to feed them one of another aspect, so use sizes
    such as 1280×720 or 640×360. Changing the resolution restarts the camera
    pipeline.

---

## AI Models

### Available Models

`robot.ai.available_models()` lists the models in the robot's model store:

```python
with Robot(host="192.168.178.71") as robot:
    for info in robot.ai.available_models():
        state = "running" if info.active else ("ready" if info.available else "not installed")
        print(f"{info.name:36s} {info.task:22s} {info.shaves} cores  {state}")
```

The `AIModel` enum names the common ones (`YOLO26N`, `POSE_YOLO`, `HAND`, faces, QR codes); see [AI & Camera Subsystems](../api/ai-camera-subsystems.md#aimodel-enum) for the table. Any listed id also works as a string. The camera has 16 processing cores, and a model uses a fixed number of them, so two or three models fit together.

### Running a Model

```python
from pib3 import AIModel, Robot

with Robot(host="192.168.178.71") as robot:
    if not robot.ai.set_model(AIModel.YOLO26N):
        print("The robot did not start the model")   # the log says why
    else:
        for det in robot.ai.get_detections(latest_only=True):
            print(f"{det.label}: {det.confidence:.0%}")
```

`set_model` stops the models this client started before, then starts the new one, and returns when the robot reports it running. That is two pipeline rebuilds, a few seconds each. A model that is already running costs nothing.

If the robot does not have the model, `set_model` returns `False` and the log lists the models it does offer.

!!! warning "A failed start can stop the other models"
    If a model fails to start, the camera falls back to colour only, which also stops the models that ran before. `robot.ai.available_models()` shows what runs; start the others again.

### Reading Results

```python
dets  = robot.ai.get_detections(latest_only=True)       # boxes (objects, persons, faces, ...)
hands = robot.ai.get_hand_landmarks(latest_only=True)   # HandLandmarks, with finger angles
poses = robot.ai.get_poses(latest_only=True)            # PoseKeypoints, 17 COCO keypoints
```

`latest_only=True` returns the newest frame only. Without it you get every buffered frame (up to 100), so in a loop the same object shows up once per frame. Use it for control loops and counting; leave it off only when you want every frame since the last call.

Results of the face family carry more than a box:

```python
robot.ai.set_model(AIModel.HEAD_POSE)
for det in robot.ai.get_detections(latest_only=True):
    print(det.scalars)      # {"yaw": ..., "pitch": ..., "roll": ...} in degrees
```

### Several Models at Once

```python
robot.ai.start_model(AIModel.YOLO26N)
robot.ai.start_model(AIModel.POSE_YOLO)       # YOLO26N keeps running

objects = robot.ai.get_detections(latest_only=True, model=AIModel.YOLO26N)
people  = robot.ai.get_poses(latest_only=True)   # the current model: POSE_YOLO

robot.ai.stop_model(AIModel.POSE_YOLO)
```

### Another Client Uses the Camera

`robot.ai.stop_model()` only releases *this* client's hold; a model another client holds keeps running. A model another client starts does not affect you either: your results come on your model's own topic. What other clients do change is the camera's timing, since they rebuild the pipeline.

### Raw Subscriptions

Under `robot.ai`, each model publishes `datatypes/DetectionArray` messages on `detections/<model>`:

```python
from pib3 import AIModel, parse_detection_message   # typed objects from one message

def on_message(message):
    for obj in parse_detection_message(message):
        print(type(obj).__name__, getattr(obj, "label", ""))

with Robot(host="192.168.178.71") as robot:
    robot.start_ai_model(AIModel.YOLO26N)             # the model must be running
    sub = robot.subscribe_ai_detections(AIModel.YOLO26N, on_message)
    time.sleep(10)
    sub.unsubscribe()
    robot.stop_ai_model(AIModel.YOLO26N)
```

The message holds pixels with the frame size, `keypoint_names`/`keypoint_x`/`keypoint_y` and `scalar_names`/`scalar_values`; see [the robot backend reference](../api/backends/robot.md#ai-detection).

### Watching Model Status

```python
sub = robot.subscribe_ai_status(lambda status: print(status["models"]))   # about 1 Hz
```

Each model reports `state` (`idle`, `starting`, `running`, `failed`), a `message` that says why it failed, `fps` and `active`. [`examples/monitor_ai_model.py`](https://github.com/mamrehn/pib3/blob/main/examples/monitor_ai_model.py) prints state changes.

---

## IMU Sensor Data

Access accelerometer and gyroscope from the OAK-D Lite's BMI270 IMU.

!!! info "Data Source"
    All IMU data types (`full`, `accelerometer`, `gyroscope`) subscribe to the
    same `/imu` ROS topic (`sensor_msgs/Imu`). The `accelerometer` and
    `gyroscope` options pick one half client-side for convenience. Axes follow
    ROS (x forward, y left, z up): a robot at rest reads about +9.8 on z.

### Full IMU Data

Get both accelerometer and gyroscope:

```python
def on_imu(data):
    """Callback receives full IMU data."""
    # Accelerometer (m/s²)
    accel = data['linear_acceleration']
    print(f"Accel: x={accel['x']:.2f}, y={accel['y']:.2f}, z={accel['z']:.2f}")

    # Gyroscope (rad/s)
    gyro = data['angular_velocity']
    print(f"Gyro: x={gyro['x']:.4f}, y={gyro['y']:.4f}, z={gyro['z']:.4f}")

with Robot(host="192.168.178.71") as robot:
    sub = robot.subscribe_imu(on_imu, data_type="full")

    import time
    time.sleep(5)

    sub.unsubscribe()
```

### Accelerometer Only

Get just acceleration data in a simplified format:

```python
def on_accel(data):
    """Callback receives accelerometer data."""
    # Data is in Vector3Stamped-like format
    vec = data['vector']
    header = data['header']
    print(f"Accel: {vec['x']:.2f}, {vec['y']:.2f}, {vec['z']:.2f} m/s²")

with Robot(host="192.168.178.71") as robot:
    sub = robot.subscribe_imu(on_accel, data_type="accelerometer")
    time.sleep(3)
    sub.unsubscribe()
```

### Gyroscope Only

```python
def on_gyro(data):
    """Callback receives gyroscope data."""
    vec = data['vector']
    print(f"Gyro: {vec['x']:.4f}, {vec['y']:.4f}, {vec['z']:.4f} rad/s")

with Robot(host="192.168.178.71") as robot:
    sub = robot.subscribe_imu(on_gyro, data_type="gyroscope")
    time.sleep(3)
    sub.unsubscribe()
```

### IMU Rate

The camera node publishes the IMU at a fixed 100 Hz on `/imu`; there is nothing to set (`set_imu_frequency()` raises `NotImplementedError`). Skip samples in your callback for a lower rate.

### IMU Data Format

**Full IMU (`data_type="full"`):**

```python
{
    "header": {
        "stamp": {"sec": 1234567890, "nanosec": 123456789},
        "frame_id": "oak_imu_frame"
    },
    "linear_acceleration": {"x": 0.05, "y": -0.02, "z": 9.81},
    "angular_velocity": {"x": 0.001, "y": -0.002, "z": 0.0005},
    "orientation": {"x": 0, "y": 0, "z": 0, "w": 1},   # identity: the BMI270 cannot fuse one
    "linear_acceleration_covariance": [...],
    "angular_velocity_covariance": [...],
    "orientation_covariance": [-1.0, 0.0, ...]          # -1 in [0] marks orientation as unavailable
}
```

**Accelerometer only (`data_type="accelerometer"`):**

```python
{
    "header": {...},
    "vector": {"x": 0.05, "y": -0.02, "z": 9.81}
}
```

**Gyroscope only (`data_type="gyroscope"`):**

```python
{
    "header": {...},
    "vector": {"x": 0.001, "y": -0.002, "z": 0.0005}
}
```

---

## Complete Example: Multi-Model AI Demo

Detection, then body pose, then both at once:

```python
import time
from pib3 import AIModel, Robot


def watch(robot, seconds, read, describe):
    """Print what ``read`` finds in the newest frame, twice a second."""
    end = time.time() + seconds
    while time.time() < end:
        for item in read(timeout=1.0, latest_only=True):
            print("  ", describe(item))
        time.sleep(0.5)


with Robot(host="192.168.178.71") as robot:
    print("OBJECT DETECTION")
    if robot.ai.set_model(AIModel.YOLO26N):
        watch(robot, 5, robot.ai.get_detections,
              lambda d: f"{d.label} ({d.confidence:.2f})")

    print("POSE ESTIMATION")
    if robot.ai.set_model(AIModel.POSE_YOLO):          # stops YOLO26N first
        watch(robot, 5, robot.ai.get_poses,
              lambda p: f"nose at ({p.nose.x:.2f}, {p.nose.y:.2f})")

    print("BOTH AT ONCE")
    if robot.ai.start_model(AIModel.YOLO26N):          # POSE_YOLO keeps running
        time.sleep(3)
        print("  objects:", len(robot.ai.get_detections(latest_only=True, model=AIModel.YOLO26N)))
        print("  people: ", len(robot.ai.get_poses(latest_only=True, model=AIModel.POSE_YOLO)))
# leaving the with-block releases every model this script started
```

---

## Complete Example: Vision-Based Control

Combine camera, AI, and robot control. The loop reads the newest frame each time and moves the head toward the most confident person:

```python
import time
from pib3 import AIModel, Joint, Robot


def track_person(robot, duration=30):
    """Turn the head toward a person for ``duration`` seconds."""
    if not robot.ai.set_model(AIModel.YOLO26N):
        print("The robot did not start the model")
        return

    end = time.time() + duration
    while time.time() < end:
        people = [d for d in robot.ai.get_detections(timeout=1.0, latest_only=True)
                  if d.label == "person" and d.confidence > 0.7]
        if people:
            center_x = max(people, key=lambda d: d.confidence).bbox.center[0]
            if center_x < 0.4:
                robot.set_joint(Joint.TURN_HEAD, 60)    # person on the left: turn left
            elif center_x > 0.6:
                robot.set_joint(Joint.TURN_HEAD, 40)    # person on the right: turn right
            else:
                robot.set_joint(Joint.TURN_HEAD, 50)    # person centered
        time.sleep(0.1)


with Robot(host="192.168.178.71") as robot:
    track_person(robot)
```

---

## API Reference Summary

### Camera Methods

| Method | Description |
|--------|-------------|
| `subscribe_camera_image(callback)` | Stream camera images (JPEG bytes) |
| `set_camera_config(fps, quality, resolution)` | Configure camera settings |

### AI Methods

| Method | Description |
|--------|-------------|
| `robot.ai.set_model(model)` | Run this model, release the others this client started |
| `robot.ai.start_model(model)` / `stop_model(model)` | Run several models; release one |
| `robot.ai.get_detections()` / `get_hand_landmarks()` / `get_poses()` | Typed results, `latest_only=True`, `model=` |
| `robot.ai.available_models()` | The models the robot offers, with state |
| `robot.ai.stop()` | Release every model this client started |
| `get_available_ai_models(timeout)` | The same list as a dict |
| `start_ai_model(model)` / `stop_ai_model(model)` | The robot services, `(success, message)` |
| `subscribe_ai_detections(model, callback)` | Raw `DetectionArray` messages of one model |
| `get_ai_detections(model)` | The latest `DetectionArray` once |
| `subscribe_ai_status(callback)` | State of the models (`/models_status`) |

### IMU Methods

| Method | Description |
|--------|-------------|
| `subscribe_imu(callback, data_type)` | Subscribe to IMU data (`full`, `accelerometer`, `gyroscope`), 100 Hz |
| `subscribe_imu_raw(callback)` | The whole `sensor_msgs/Imu` message |

### Helper Functions

| Function | Description |
|----------|-------------|
| `parse_detection_message(message)` | One `DetectionArray` as `Detection` / `HandLandmarks` / `PoseKeypoints` |
| `rle_decode(rle)` | Decode RLE segmentation mask (simulation only) |

---

## Troubleshooting

### No Camera Data

1. Check OAK-D Lite is connected to the robot
2. Verify the ROS camera node is running: `ros2 topic list | grep camera_topic`
3. Check network connectivity to the robot
4. Verify rosbridge is running on port 9090

### set_model Returns False

1. The log line names the reason the robot gave ("Unknown model", "Model is unavailable") and lists the models it offers
2. A model the robot does not list is not in its model store; `robot.ai.available_models()` shows `available`
3. A model that failed to start shows `state: failed` with a message in `robot.subscribe_ai_status()`; the camera then runs colour only until a model starts
4. Allow time: a start rebuilds the camera pipeline, a few seconds

### Results Are Empty

1. Check that the model runs: `robot.ai.model` and `robot.ai.available_models()`
2. A hand or pose model returns nothing while no hand or person is in view
3. The YOLO models drop detections below 0.5 confidence; this is set in the model's archive and pib3 cannot change it
4. Depth and a running model exclude each other; this does not affect detections

### AI Inference Slow

1. The models run on the camera, but the robot's Raspberry Pi receives and parses the results; pib-backend measured about half the rate of a laptop for the same model (11 versus 21.5 results/s for YOLOv6n), most likely because of that host side
2. Several models at once share the camera's 16 cores and its USB link; `robot.ai.fps` shows the current model
3. Reduce camera resolution

### IMU Data Issues

1. Use `data_type="full"` to verify data is coming through
2. Check the `/imu` topic: `ros2 topic echo /imu`
3. The BMI270 gives no orientation; integrate the gyroscope or fuse it yourself

### Connection Issues

1. Verify robot IP: `ping 192.168.178.71`
2. Check rosbridge port: `nc -zv 192.168.178.71 9090`
3. Increase connection timeout: `Robot(host="...", timeout=10.0)`

---

## Next Steps

- [Controlling the Robot](controlling-robot.md) - Joint control basics
- [Image to Trajectory](image-to-trajectory.md) - Drawing with IK
- [Custom Configurations](custom-configurations.md) - Fine-tune parameters
