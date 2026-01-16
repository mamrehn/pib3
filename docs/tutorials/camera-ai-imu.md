# Camera, AI Detection, and IMU

Access the OAK-D Lite camera, AI inference, and IMU sensors through the pib3 API.

## Objectives

By the end of this tutorial, you will:

- Stream camera images with efficient CBOR compression
- Use on-demand AI object detection and segmentation
- Access IMU accelerometer and gyroscope data
- Understand the on-demand activation pattern

## Prerequisites

- pib3 installed with robot support: `pip install "pib3[robot] @ git+https://github.com/mamrehn/pib3.git"`
- A PIB robot with OAK-D Lite camera connected
- Rosbridge running on the robot

---

## Key Concept: On-Demand Activation

All camera, AI, and IMU features follow an **on-demand pattern**:

- **Streaming/inference only runs while you're subscribed**
- When you call `subscribe_*()`, the robot starts the pipeline
- When you call `.unsubscribe()`, the robot stops the pipeline
- This saves computational resources and power

```python
# Subscribe → Pipeline starts automatically
sub = robot.subscribe_camera_image(callback)

# ... do your processing ...

# Unsubscribe → Pipeline stops automatically
sub.unsubscribe()
```

---

## Camera Streaming

### Efficient Binary Streaming (Recommended)

Use CBOR compression for efficient binary transfer without base64 overhead:

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

with Robot(host="172.26.34.149") as robot:
    # Subscribe to camera (streaming starts)
    sub = robot.subscribe_camera_image(on_frame, compression="cbor")

    # Keep running for 10 seconds
    import time
    time.sleep(10)

    # Unsubscribe (streaming stops)
    sub.unsubscribe()
    cv2.destroyAllWindows()
```

### Legacy Base64 Streaming

For backward compatibility or when CBOR is not available:

```python
import base64

def on_base64_frame(base64_str):
    """Callback receives base64-encoded JPEG string."""
    jpeg_bytes = base64.b64decode(base64_str)
    # ... process as above

with Robot(host="172.26.34.149") as robot:
    sub = robot.subscribe_camera_legacy(on_base64_frame)
    # ...
    sub.unsubscribe()
```

### Configuring Camera Settings

Adjust FPS, quality, and resolution:

```python
with Robot(host="172.26.34.149") as robot:
    # Set individual parameters
    robot.set_camera_config(fps=30)
    robot.set_camera_config(quality=80)
    robot.set_camera_config(resolution=(1280, 720))

    # Or set multiple at once
    robot.set_camera_config(
        fps=30,
        quality=80,
        resolution=(1280, 720)
    )
```

!!! warning "Brief Interruption"
    Changing camera settings causes a brief stream interruption (~100-200ms)
    as the pipeline rebuilds.

---

## AI Object Detection

### Subscribing to Detections

AI inference only runs while you're subscribed:

```python
from pib3 import Robot

def on_detection(data):
    """Callback receives detection results."""
    model = data['model']
    det_type = data['type']
    latency = data['latency_ms']

    print(f"Model: {model}, Type: {det_type}, Latency: {latency:.1f}ms")

    if det_type == 'detection':
        for det in data['result']['detections']:
            label = det['label']
            conf = det['confidence']
            bbox = det['bbox']
            print(f"  Found class {label} ({conf:.2f}) at {bbox}")

with Robot(host="172.26.34.149") as robot:
    # Subscribe to AI detections (inference starts)
    sub = robot.subscribe_ai_detections(on_detection)

    import time
    time.sleep(10)

    # Unsubscribe (inference stops)
    sub.unsubscribe()
```

### Detection Result Format

All detection messages include common metadata:

```python
{
    "model": "mobilenet-ssd",
    "type": "detection",  # or "classification", "segmentation", "pose"
    "frame_id": 42,
    "timestamp_ns": 1234567890123456789,
    "latency_ms": 12.5,
    "result": { ... }  # Model-specific results
}
```

#### Detection Models

```python
{
    "result": {
        "detections": [
            {
                "label": 15,        # Class ID
                "confidence": 0.92,
                "bbox": {
                    "xmin": 0.1, "ymin": 0.05,
                    "xmax": 0.3, "ymax": 0.4
                }
            }
        ],
        "count": 1
    }
}
```

#### Classification Models

```python
{
    "result": {
        "classifications": [
            {"class_id": 281, "confidence": 0.85},
            {"class_id": 282, "confidence": 0.10}
        ]
    }
}
```

### Available AI Models

Query what models are available on the robot:

```python
with Robot(host="172.26.34.149") as robot:
    models = robot.get_available_ai_models()

    for name, info in models.items():
        print(f"{name}:")
        print(f"  Type: {info['type']}")
        print(f"  Input size: {info['input_size']}")
        print(f"  FPS: {info['fps']}")
```

### Switching AI Models

Change the active AI model at runtime:

```python
with Robot(host="172.26.34.149") as robot:
    # List available models
    models = robot.get_available_ai_models()
    print(f"Available: {list(models.keys())}")

    # Switch to YOLO
    robot.set_ai_model("yolov8n")

    # Or use full config
    robot.set_ai_config(
        model="yolov8n",
        confidence=0.5,  # Detection threshold
    )
```

!!! warning "Model Switch Delay"
    Switching models causes a brief interruption (~200-500ms)
    as the neural network pipeline rebuilds.

### Track Current Model

Subscribe to model change notifications:

```python
def on_model_change(info):
    print(f"Now using: {info['name']} ({info['type']})")

with Robot(host="172.26.34.149") as robot:
    sub = robot.subscribe_current_ai_model(on_model_change)
    # ... model changes will trigger callback
    sub.unsubscribe()
```

---

## Segmentation Models

Segmentation models support two output modes:

### BBox Mode (Default, Lightweight)

Returns bounding boxes around detected segments:

```python
robot.set_ai_config(
    model="deeplabv3",
    segmentation_mode="bbox"
)
```

Result:

```python
{
    "result": {
        "mode": "bbox",
        "image_size": [256, 256],
        "classes_detected": [0, 15],
        "bboxes": [
            {"class_id": 15, "bbox": {"xmin": 0.2, ...}}
        ]
    }
}
```

### Mask Mode (Detailed)

Returns full segmentation mask as RLE (Run-Length Encoded):

```python
from pib3.backends import rle_decode

robot.set_ai_config(
    model="deeplabv3",
    segmentation_mode="mask",
    segmentation_target_class=15  # Person class
)

def on_segmentation(data):
    if data['type'] == 'segmentation':
        result = data['result']
        if result.get('mode') == 'mask':
            # Decode RLE to numpy mask
            mask = rle_decode(result['mask_rle'])
            print(f"Mask shape: {mask.shape}")
            # mask is binary numpy array (0 or 1)

sub = robot.subscribe_ai_detections(on_segmentation)
```

---

## IMU Sensor Data

Access accelerometer and gyroscope from the OAK-D Lite's BMI270 IMU.

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

with Robot(host="172.26.34.149") as robot:
    sub = robot.subscribe_imu(on_imu, data_type="full")

    import time
    time.sleep(5)

    sub.unsubscribe()
```

### Accelerometer Only

Lighter weight if you only need acceleration:

```python
def on_accel(data):
    vec = data['vector']
    print(f"Accel: {vec['x']:.2f}, {vec['y']:.2f}, {vec['z']:.2f} m/s²")

with Robot(host="172.26.34.149") as robot:
    sub = robot.subscribe_imu(on_accel, data_type="accelerometer")
    # ...
    sub.unsubscribe()
```

### Gyroscope Only

```python
def on_gyro(data):
    vec = data['vector']
    print(f"Gyro: {vec['x']:.4f}, {vec['y']:.4f}, {vec['z']:.4f} rad/s")

with Robot(host="172.26.34.149") as robot:
    sub = robot.subscribe_imu(on_gyro, data_type="gyroscope")
    # ...
    sub.unsubscribe()
```

### Setting IMU Frequency

The BMI270 supports specific frequencies:

```python
with Robot(host="172.26.34.149") as robot:
    # Valid frequencies: 25, 50, 100, 200, 250 Hz
    robot.set_imu_frequency(100)

    # Note: BMI270 rounds DOWN to nearest valid frequency
    # Request 99Hz → Get 50Hz
    # Request 150Hz → Get 100Hz
```

---

## Complete Example: Vision-Based Control

Combine camera, AI, and robot control:

```python
from pib3 import Robot
import time

class VisionController:
    def __init__(self, robot):
        self.robot = robot
        self.person_detected = False
        self.last_bbox = None

    def on_detection(self, data):
        """React to AI detections."""
        if data['type'] != 'detection':
            return

        for det in data['result']['detections']:
            if det['label'] == 15 and det['confidence'] > 0.7:  # Person
                self.person_detected = True
                self.last_bbox = det['bbox']
                print(f"Person at: {det['bbox']}")

    def run(self, duration=30):
        """Run vision-based control loop."""
        # Subscribe to detections
        sub = self.robot.subscribe_ai_detections(self.on_detection)

        try:
            start = time.time()
            while time.time() - start < duration:
                if self.person_detected and self.last_bbox:
                    # Track person by moving head
                    center_x = (self.last_bbox['xmin'] + self.last_bbox['xmax']) / 2

                    if center_x < 0.4:
                        # Person on left, turn head left
                        self.robot.set_joint("turn_head_motor", 60)
                    elif center_x > 0.6:
                        # Person on right, turn head right
                        self.robot.set_joint("turn_head_motor", 40)
                    else:
                        # Person centered
                        self.robot.set_joint("turn_head_motor", 50)

                    self.person_detected = False

                time.sleep(0.1)
        finally:
            sub.unsubscribe()

# Usage
with Robot(host="172.26.34.149") as robot:
    controller = VisionController(robot)
    controller.run(duration=30)
```

---

## API Reference Summary

### Camera Methods

| Method | Description |
|--------|-------------|
| `subscribe_camera_image(callback, compression="cbor")` | Stream camera with CBOR binary |
| `subscribe_camera_legacy(callback)` | Stream camera with base64 (legacy) |
| `set_camera_config(fps, quality, resolution)` | Configure camera settings |

### AI Detection Methods

| Method | Description |
|--------|-------------|
| `subscribe_ai_detections(callback)` | Subscribe to AI results |
| `get_available_ai_models(timeout)` | List available models |
| `set_ai_model(model_name)` | Switch AI model |
| `set_ai_config(model, confidence, ...)` | Full AI configuration |
| `subscribe_current_ai_model(callback)` | Track model changes |

### IMU Methods

| Method | Description |
|--------|-------------|
| `subscribe_imu(callback, data_type)` | Subscribe to IMU data |
| `set_imu_frequency(frequency)` | Set sampling rate (25-250 Hz) |

### Helper Functions

| Function | Description |
|----------|-------------|
| `rle_decode(rle)` | Decode RLE segmentation mask to numpy array |

---

## Troubleshooting

### No Camera Data

1. Check OAK-D Lite is connected to the robot
2. Verify the ROS camera node is running
3. Check network connectivity

### AI Inference Slow

1. Some models are more demanding - try `mobilenet-ssd` for speed
2. Check robot CPU/NPU usage
3. Reduce camera resolution

### IMU Data Noisy

1. IMU needs calibration at startup
2. Keep robot stationary for first few seconds
3. Use lower frequency for more stable readings

---

## Next Steps

- [Controlling the Robot](controlling-robot.md) - Joint control basics
- [Image to Trajectory](image-to-trajectory.md) - Drawing with IK
- [Custom Configurations](custom-configurations.md) - Fine-tune parameters
