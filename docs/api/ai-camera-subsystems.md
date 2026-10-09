# AI and Camera Subsystems

Simplified, high-level APIs for accessing AI inference and camera streaming on the PIB robot.

## Overview

The subsystem APIs (`robot.ai` and `robot.camera`) provide a simpler alternative to manual subscription management. They automatically handle subscription lifecycle and provide typed results.

| Approach | When to Use |
|----------|-------------|
| **Subsystem API** (`robot.ai`, `robot.camera`) | Most use cases—simple, auto-managed |
| **Raw subscriptions** (`subscribe_ai_detections(model, callback)`) | Custom buffering, multiple callbacks, advanced scenarios |

```python
from pib3 import Robot, AIModel

with Robot(host="172.26.34.149") as robot:
    # Subsystem API - simple and direct
    robot.ai.set_model(AIModel.HAND)
    for hand in robot.ai.get_hand_landmarks():
        print(f"{hand.handedness}: index={hand.finger_angles.index:.0f}°")
```

---

## AISubsystem (`robot.ai`)

Run AI models on the OAK-D Lite and read their results as typed objects.

The camera runs the models of the robot's **model store** (pib-backend). A client asks for a model with `/start_model` and names itself as an *owner*; the model runs as long as any owner holds it. That is why a script, cerebra and a neighbouring group do not switch each other's models away. `robot.ai` starts and stops models under this client's owner name (`pib3-<user>@<computer>`, or `Robot(ai_owner="group-3")`) and keeps one receiver per model.

!!! note "Starting or stopping a model restarts the camera pipeline"
    Video and the IMU pause for a few seconds, unless another owner already runs the model. Depth is gone while any model runs (see [Depth](#depth)).

### Quick Start

```python
from pib3 import Robot, AIModel

with Robot(host="172.26.34.149") as robot:
    # Run a model (returns when the robot reports it running)
    robot.ai.set_model(AIModel.YOLO26S)

    # Get detections (waits automatically for results)
    for det in robot.ai.get_detections(latest_only=True):
        print(f"{det.label}: {det.confidence:.0%} at {det.bbox}")

    # Check performance
    print(f"FPS: {robot.ai.fps:.1f}")
```

### Methods

#### `set_model()`

Run one model and none of the others this client started.

```python
def set_model(
    model: Union[AIModel, str],
    timeout: float = 30.0
) -> bool
```

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `model` | `AIModel` or `str` | *required* | Model to run (enum, or any model id the robot lists) |
| `timeout` | `float` | `30.0` | Max seconds for each answer of the robot |

**Returns:** `True` if the robot reports the model running; `False` if it refused. The reason is logged together with the models the robot offers.

`set_model` stops the models this client started earlier, then starts the new one. Each start and stop rebuilds the camera pipeline and takes a few seconds, except when another owner already runs that model. A model that is already running costs nothing.

```python
from pib3 import AIModel

robot.ai.set_model(AIModel.HAND)
robot.ai.set_model(AIModel.YOLO26S)       # stops the hand model first
robot.ai.set_model("yolov6n_coco_640x640")  # any id from get_available_ai_models()
```

#### `start_model()` / `stop_model()`

Run several models at once. The camera has 16 processing cores; each model uses a fixed number (`AIModelInfo.shaves`, 4 for the YOLO models, 8 for the hand chain), so two or three models fit together.

```python
robot.ai.start_model(AIModel.YOLO26S)
robot.ai.start_model(AIModel.HAND)      # both run; HAND is now the current model

dets = robot.ai.get_detections(latest_only=True, model=AIModel.YOLO26S)
hands = robot.ai.get_hand_landmarks(latest_only=True)   # the current model

robot.ai.stop_model(AIModel.HAND)
```

`stop_model()` only releases *this client's* hold; the model keeps running while another owner holds it.

---

#### `get_detections()`

Get every box a model reports (objects, but also the persons of a pose model, hands, faces, QR codes).

```python
def get_detections(timeout: float = 5.0, latest_only: bool = False, model=None) -> List[Detection]
```

Waits automatically for results if buffer is empty. Raises `RuntimeError` if no model has been started.

```python
robot.ai.set_model(AIModel.YOLO26S)
for det in robot.ai.get_detections(latest_only=True):
    print(f"Found {det.label} ({det.confidence:.0%})")
    print(f"  BBox: {det.bbox.center}")
```

Models of the face family attach more to `det.keypoints` and `det.scalars`:

```python
robot.ai.set_model(AIModel.HEAD_POSE)
for det in robot.ai.get_detections(latest_only=True):
    print(det.scalars)        # {"yaw": -12.0, "pitch": 3.5, "roll": 0.8}
```

---

#### `get_hand_landmarks()`

Get hand tracking results with finger angles.

```python
def get_hand_landmarks(timeout: float = 5.0, latest_only: bool = False, model=None) -> List[HandLandmarks]
```

```python
robot.ai.set_model(AIModel.HAND)
for hand in robot.ai.get_hand_landmarks(latest_only=True):
    print(f"{hand.handedness}: index={hand.finger_angles.index:.0f}°")

    # Convert to servo values for robot hand control
    servos = hand.finger_angles.to_servo_values()
    robot.set_joints({
        "index_left_stretch": servos["index"],
        "middle_left_stretch": servos["middle"],
    })
```

The list is empty while no hand is in view. Finger angles are measured in pixel space, so the 16:9 frame does not skew them.

---

#### `get_poses()`

Get body pose estimation results.

```python
def get_poses(timeout: float = 5.0, latest_only: bool = False, model=None) -> List[PoseKeypoints]
```

```python
robot.ai.set_model(AIModel.POSE_YOLO)
for pose in robot.ai.get_poses(latest_only=True):
    if pose.nose:
        print(f"Nose at: ({pose.nose.x:.2f}, {pose.nose.y:.2f})")
```

---

### Properties

| Property | Type | Description |
|----------|------|-------------|
| `model` | `Optional[str]` | Id of the current model: the one the getters read by default |
| `models` | `Tuple[str, ...]` | Ids of all models this client started |
| `fps` | `float` | Results per second of the current model |
| `avg_latency_ms` | `float` | Average age of a result on arrival, in milliseconds |

`avg_latency_ms` compares the robot's timestamp with this computer's clock, so it includes the network and is only meaningful when both clocks are synchronised (NTP). Implausible values are ignored. In the simulation it is the inference time.

### Other Methods

| Method | Description |
|--------|-------------|
| `available_models()` | The models the robot offers, as `AIModelInfo` (`available`, `active`, `shaves`, `task`, ...) |
| `clear()` | Clear buffered results |
| `stop()` | Release every model this client started and unsubscribe (also runs on disconnect) |

---

## CameraSubsystem (`robot.camera`)

Access raw camera frames from the OAK-D Lite with automatic subscription management.

### Quick Start

```python
from pib3 import Robot

with Robot(host="172.26.34.149") as robot:
    # Get latest frame
    frame = robot.camera.get_frame()
    if frame:
        print(f"Frame {frame.frame_id}, {len(frame.jpeg_bytes)} bytes")
        
        # Decode to numpy (requires OpenCV)
        img = frame.to_numpy()
```

### Methods

#### `get_frame()`

Get the latest camera frame.

```python
def get_frame(timeout: float = 5.0) -> Optional[CameraFrame]
```

```python
frame = robot.camera.get_frame()
if frame:
    # Save JPEG directly
    with open("capture.jpg", "wb") as f:
        f.write(frame.jpeg_bytes)
    
    # Or decode to numpy for processing
    img = frame.to_numpy()  # Requires OpenCV
```

---

#### `get_frames()`

Get all buffered frames and clear the buffer.

```python
def get_frames() -> List[CameraFrame]
```

---

#### `configure()`

Configure camera settings.

```python
def configure(
    fps: Optional[int] = None,
    quality: Optional[int] = None,
    resolution: Optional[tuple] = None,
) -> None
```

| Parameter | Type | Description |
|-----------|------|-------------|
| `fps` | `int` | Frames per second (e.g., 30) |
| `quality` | `int` | JPEG quality 1-100 (e.g., 80) |
| `resolution` | `tuple` | (width, height) e.g., (1280, 720) |

```python
robot.camera.configure(fps=10, quality=80, resolution=(1280, 720))
```

!!! warning "Keep the frame 16:9"
    The camera publishes 1280×720 by default. AI models read the same frame, and the camera refuses to feed them one of another aspect, so use 16:9 sizes such as 1280×720 or 640×360. Changing the resolution restarts the camera pipeline; quality and frame rate do not.

---

### Properties

| Property | Type | Description |
|----------|------|-------------|
| `frame_count` | `int` | Total number of frames received |

### Other Methods

| Method | Description |
|--------|-------------|
| `stop()` | Stop camera streaming (unsubscribe) |

### Depth

`robot.camera.get_depth_frame()` and `get_distance_at_px()` read the camera's stereo depth. Depth is the camera's resting state: it runs while **no AI model runs** and is gone while one does, because the stereo pair and a model share the camera's cores. After `robot.ai.stop()` it returns within a few seconds. So a script cannot look up the distance of a detected object while the detector runs; measure first, or stop the model.

The OAK-D Lite has no time-of-flight sensor; depth comes from comparing the two black-and-white cameras (about 0.8 m to 12 m).

---

## AIModel Enum

Type-safe names for the models of the robot's model store (pib-backend, `models/manifest.yaml`). The enum value is the model id that `/list_models` reports and that cerebra shows.

```python
from pib3 import AIModel
```

| Enum value | Model id | Task | Cores |
|------------|----------|------|-------|
| `AIModel.YOLO26S` | `yolo26s_coco_512x288` | Object detection, 80 COCO classes (default) | 4 |
| `AIModel.YOLO26N` | `yolo26n_coco_512x288` | Object detection, faster and less accurate | 4 |
| `AIModel.POSE_YOLO` (also `POSE_YOLO26S`) | `yolo26s_pose_coco_512x288` | Body pose, 17 COCO keypoints (default) | 4 |
| `AIModel.POSE_YOLO26N` | `yolo26n_pose_coco_512x288` | Body pose, faster and less accurate | 4 |
| `AIModel.YOLO26M`, `AIModel.POSE_YOLO26M` | `yolo26m_coco_512x288`, `yolo26m_pose_coco_512x288` | More accurate, **laptop (simulation) only** | - |
| `AIModel.HAND` | `hand_tracking_mp` | Hand landmarks, 21 points per hand | 8 |
| `AIModel.FACE` | `face_detection_yunet_160x120` | Face boxes | 4 |
| `AIModel.EMOTION` | `emotion_recognition_crop` | Emotion (one probability per emotion in `det.scalars`) | 8 |
| `AIModel.HEAD_POSE` | `head_pose_estimation_crop` | Head `yaw`, `pitch`, `roll` in `det.scalars` | 8 |
| `AIModel.QR_CODE` | `qr_code_detection_384x384` | QR code boxes | 4 |

`AIModel.YOLO26M` and `POSE_YOLO26M` are in `LAPTOP_ONLY_MODELS`: on the OAK-D YOLO26m reached 4 results/s, so the robot's store does not have it, and `robot.ai` refuses them without asking the robot.

pib-backend withdrew the face mesh (`facemesh_crop`) and the 68 facial landmarks (`facial_landmarks_68_crop`) from its camera model list (PR-1957), so `AIModel` no longer names them.

!!! warning "Hand tracking speed is unknown"
    `AIModel.HAND` is `hand_tracking_mp`, the newer of the backend's two hand chains. pib-backend measured its older chain, `hand_tracking`, at **1.0 result/s** on a robot. Nobody has published a measurement of `hand_tracking_mp`. A hand-mirroring program needs several results per second: check `robot.ai.fps` on your robot before relying on it. `AIModel` has no member for the older chain; `"hand_tracking"` works as a string.

Any other model id works as a plain string; `robot.ai.available_models()` lists what the robot has. The model ids and core counts come from the backend's manifest and were not all measured by pib3; `available_models()` reports the numbers the robot itself uses.

!!! warning "The YOLO26 models come with pib-backend's b3 fork"
    The four YOLO26 models are Ultralytics builds (AGPL-3.0) at 512×288, the camera's 16:9. pib-backend's b3 fork (`mamrehn/pib-backend`, branch `b3-develop`) lists them and provisions them from its model release (`setup/setup-pib.sh --models`). On a robot without them, `set_model` returns `False` and logs the models the robot does offer, for example `yolov6n_coco_640x640` (pib-rocks' detector, 640×640, same COCO classes).

!!! note "Speed and accuracy on the OAK-D Lite"
    Measured with the camera node's own chain on an OAK-D Lite (laptop, USB 3); COCO accuracy (mAP50-95) is Ultralytics' figure at 640×640.

    | Model | Accuracy | Results/s | Age of a result |
    |-------|----------|-----------|-----------------|
    | `YOLO26S` | 48.6 | 12.6 | 0.22 s |
    | `YOLO26N` | 40.9 | 25.2 | 0.15 s |
    | `POSE_YOLO` | 63.0 | 11.6 | 0.25 s |
    | `POSE_YOLO26N` | 57.2 | 21.9 | 0.16 s |

    The camera itself parses the results, so the robot's Raspberry Pi only forwards them; these rates are the ceiling there. They were not measured on a robot.

!!! tip "Hidden keypoints"
    A pose model also places keypoints it cannot see, such as a wrist behind the back, and gives them a low `keypoint.confidence`. Check it before using a point: `if wrist.confidence > 0.5: ...`. Robots on an older backend send no confidences; pib3 then reports 1.0.

### Retired names

These were exposed by earlier versions of this SDK. `set_model()` remaps them with a `DeprecationWarning`, on the robot and in the simulation:

| Old name | Now uses |
|----------|----------|
| `"yolo26n"`, `"yolov6n"`, `"yolov10n"`, `"yolov8n"`, `"yolo11n"` | `AIModel.YOLO26N` |
| `"yolo26s"`, `"yolo11s"`, `"mobilenet-ssd"` | `AIModel.YOLO26S` |
| `"pose_yolo"`, `"pose_hrnet"`, `"pose"` | `AIModel.POSE_YOLO` |
| `"pose_yolov8"` | `AIModel.POSE_YOLO26N` |
| `"hand"` | `AIModel.HAND` |
| `"face"` | `AIModel.FACE` |
| `"yolov8n-seg"`, `"deeplabv3"`, `"fastsam"` | `"segmentation"` (simulation only) |

The robot never had segmentation, gaze, line or person-only models in this store; they were part of an earlier backend branch.

!!! note "Simulation"
    The Webots backend (`sim.ai`) has the same methods and runs ultralytics or MediaPipe on your laptop instead of the OAK-D: `YOLO26S` → `yolo26s.pt`, `YOLO26N` → `yolo26n.pt`, `YOLO26M` → `yolo26m.pt`, `POSE_YOLO` → `yolo26s-pose.pt`, `POSE_YOLO26N` / `POSE_YOLO26M` → `yolo26n-pose.pt` / `yolo26m-pose.pt`, `HAND` → MediaPipe, `"segmentation"` → `yolo26n-seg.pt` (simulation only, with `det.mask_rle`). Use s by default; n on a laptop that is too slow for it, m when you want more accuracy. On a laptop CPU one frame took 38 / 86 / 212 ms (n / s / m); `pib3.backends.sim_ai.inference_ms(model)` measures it on yours. Weights are loaded from `$PIB3_WEIGHTS_DIR`, the running script's folder or the working directory before ultralytics downloads them. It also takes a weights file name directly (`"yolo26l.pt"`) and `"recognition"` for Webots ground truth. Faces, emotion, head pose and QR codes have no simulated equivalent. Models run together, each inferring on every rendered frame.

---

## Type Reference

### Detection

One box a model reports.

```python
@dataclass
class Detection:
    label_id: int                     # COCO class id, or -1 for labels outside COCO
    confidence: float                 # 0.0 to 1.0
    bbox: BoundingBox                 # normalized coordinates
    label: str                        # class name, e.g. "person", "hand"
    mask_rle: Optional[Dict]          # RLE mask (simulation only)
    keypoints: List[Keypoint]         # named landmarks, in normalized coordinates
    scalars: Dict[str, float]         # named values: yaw, pitch, handedness, ...
```

The robot sends pixel coordinates together with the frame size (`DetectionArray`); pib3 normalizes them, so `bbox` and `keypoints` are in [0, 1] whatever the camera's resolution.

**Properties:**

```python
det.label          # "person"
det.confidence     # 0.92
det.bbox.center    # (0.5, 0.3) - center point
det.bbox.width     # 0.2
det.bbox.height    # 0.4
det.bbox.to_pixels(640, 480)  # (x1, y1, x2, y2) in pixels
det.keypoints[0].name         # "nose" (pose and face models)
det.scalars["yaw"]            # head pose, in degrees
```

---

### HandLandmarks

Hand tracking result with 21 landmarks and calculated finger angles.

```python
@dataclass
class HandLandmarks:
    landmarks: np.ndarray          # Shape (21, 2) normalized coordinates
    keypoints: List[Keypoint]      # 21 named Keypoint objects
    handedness: Handedness         # LEFT, RIGHT, or UNKNOWN
    confidence: float              # Landmark score of the hand
    finger_angles: FingerAngles    # Calculated finger bend angles
    aspect_ratio: float            # Frame width / height the landmarks come from
```

`handedness` is the model's probability of "right" (above 0.5) as MediaPipe labels it, from the camera's point of view.

**Properties:**

```python
hand.handedness           # Handedness.LEFT
hand.wrist_position       # (x, y) tuple
hand.finger_angles.index  # Index finger bend angle in degrees
```

---

### FingerAngles

Finger bend angles calculated from hand landmarks.

```python
@dataclass
class FingerAngles:
    thumb_bend: float       # Thumb flexion angle
    thumb_rotation: float   # Thumb rotation relative to palm
    index: float            # Index finger bend angle (0° = straight, 90° = bent)
    middle: float           # Middle finger bend angle
    ring: float             # Ring finger bend angle
    pinky: float            # Pinky finger bend angle
```

**Methods:**

```python
# Convert to list
angles = finger_angles.to_list()  # [thumb_bend, thumb_rot, index, middle, ring, pinky]

# Convert to servo percentages (for robot hand control)
servos = finger_angles.to_servo_values()  # {"index": 75.0, "middle": 80.0, ...}

# Use directly with robot
robot.set_joints({
    "index_left_stretch": servos["index"],
    "middle_left_stretch": servos["middle"],
})
```

---

### PoseKeypoints

Body pose estimation with 17 COCO keypoints.

```python
@dataclass
class PoseKeypoints:
    keypoints: List[Keypoint]       # 17 keypoints, in COCO order
    confidence: float               # Confidence of the person detection
    bbox: Optional[BoundingBox]     # Bounding box around person
```

**Convenience properties:**

```python
pose.nose             # Keypoint or None
pose.left_shoulder    # Keypoint or None
pose.right_shoulder   # Keypoint or None
pose.get_keypoint(5)  # Get by COCO index (0-16)
```

---

### CameraFrame

Single camera frame with metadata.

```python
@dataclass
class CameraFrame:
    jpeg_bytes: bytes       # Raw JPEG image data
    frame_id: int           # Sequential frame number
    timestamp_ns: int       # Timestamp in nanoseconds
```

**Properties and Methods:**

```python
frame.timestamp        # Timestamp as float seconds
frame.to_numpy()       # Decode to BGR numpy array (requires OpenCV)
```

---

## Complete Example

Hand tracking with robot hand mirroring:

```python
from pib3 import Robot, AIModel
import time

with Robot(host="172.26.34.149") as robot:
    # Enable hand tracking
    if not robot.ai.set_model(AIModel.HAND):
        print("Failed to set model")
        exit(1)
    
    print("Tracking hands... (Ctrl+C to stop)")
    
    try:
        while True:
            # Newest frame only (waits automatically)
            hands = robot.ai.get_hand_landmarks(timeout=1.0, latest_only=True)
            
            for hand in hands:
                print(f"\n{hand.handedness}:")
                print(f"  Index: {hand.finger_angles.index:.0f}°")
                print(f"  Middle: {hand.finger_angles.middle:.0f}°")
                
                # Mirror to robot hand
                servos = hand.finger_angles.to_servo_values()
                if hand.handedness.value == "left":
                    robot.set_joints({
                        "index_left_stretch": servos["index"],
                        "middle_left_stretch": servos["middle"],
                        "ring_left_stretch": servos["ring"],
                        "pinky_left_stretch": servos["pinky"],
                    })
            
            # Show stats
            print(f"FPS: {robot.ai.fps:.1f}")
            
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        robot.ai.stop()   # release the model; depth comes back
```

---

## See Also

- [Camera, AI, and IMU Tutorial](../tutorials/camera-ai-imu.md) - Step-by-step guide
- [RealRobotBackend](backends/robot.md) - Low-level subscription methods
- [Types Reference](types.md) - All pib3 types
