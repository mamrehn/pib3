<h1 align="center">
<a href="https://ghloc.vercel.app/mamrehn/pib3?branch=main"><img src="examples/pib3_logo.png" width="440"></a>
</h1><br>

Repository statistics: 🐍 ![Python LOC](https://img.shields.io/endpoint?url=https://ghloc.vercel.app/api/mamrehn/pib3/badge?filter=.py$&format=human), 📜 ![Documentation LOC](https://img.shields.io/endpoint?url=https://ghloc.vercel.app/api/mamrehn/pib3/badge?filter=.md$&format=human), 🤖 ![Robot Config LOC](https://img.shields.io/endpoint?url=https://ghloc.vercel.app/api/mamrehn/pib3/badge?filter=.proto$,.urdf$,.yaml$,.stl$&format=human)


**piB3** provides motor control, inverse kinematics, and same code for digital twin and the [pib](https://pib.rocks/) printable humanoid robot.

**Key Features:**
 - 🎮 **Joint Control** - Read/write motors with IDE autocomplete via `Joint` enum
 - 📐 **Multiple Units** - Work in percent (0-100%), degrees, or radians
 - ✍️ **Trajectory Generation** - Convert 2-D images to 3-D robot arm drawing trajectories
 - 🖐️ **Hand Poses** - Pre-calibrated hand gestures and interpolated grips
 - 📷 **Camera Streaming** - Real-time camera image subscription
 - 🤖 **AI Detection** - On-device object detection with model switching
 - 📊 **IMU Sensors** - Accelerometer and gyroscope data streaming
 - 🔄 **Same Code Everywhere** - Test in Webots or Swift simulation, then run on real robot

## Installation

```bash
python -m venv venv
venv\Scripts\activate  # OR on Linux: source ./venv/bin/activate 
pip install -U "pib3[all] @ git+https://github.com/mamrehn/pib3.git"
```

## Quick Start

### Joint Control

```python
from pib3 import Robot, Joint

with Robot(host="172.26.34.149") as robot:
    # IDE autocomplete with Joint enum
    robot.set_joint(Joint.TURN_HEAD, 50.0)      # 50% of range
    robot.set_joint(Joint.ELBOW_LEFT, 75.0)

    # Read current position
    pos = robot.get_joint(Joint.ELBOW_LEFT)
    print(f"Left elbow is at {pos:.1f}%")

    # Use degrees or radians
    robot.set_joint(Joint.TURN_HEAD, -30.0, unit="deg")
    angle_rad = robot.get_joint(Joint.ELBOW_LEFT, unit="rad")

    # Wait for movement to complete
    robot.set_joint(Joint.ELBOW_LEFT, 50.0, async_=False, timeout=2.0)

    # Save and restore poses
    saved_pose = robot.get_joints()
    # ... do something else ...
    robot.set_joints(saved_pose)
```

### Hand Control

```python
from pib3 import Robot, left_hand_pose, LEFT_HAND_PENCIL_GRIP

with Robot(host="172.26.34.149") as robot:
    # Interpolated grip (0.0 = open, 1.0 = closed)
    robot.set_joints(left_hand_pose(0.5))  # Half grip
    
    # Pre-calibrated poses
    robot.set_joints(LEFT_HAND_PENCIL_GRIP)
```

### Image to Drawing Trajectory

```python
import pib3

# Convert image to trajectory
trajectory = pib3.generate_trajectory("drawing.png")
trajectory.to_json("output.json")

# Visualize in browser
with pib3.Swift() as viz:
    viz.run_trajectory("output.json")
```

### Camera & AI Detection

```python
from pib3 import Robot

with Robot(host="172.26.34.149") as robot:
    # Camera streaming
    def on_frame(jpeg_bytes):
        print(f"Got frame: {len(jpeg_bytes)} bytes")
    
    sub = robot.subscribe_camera_image(on_frame)
    # ... camera streams in background ...
    sub.unsubscribe()

    # AI object detection
    def on_detection(data):
        for det in data.get('result', {}).get('detections', []):
            print(f"Detected: {det['label']} ({det['confidence']:.2f})")
    
    sub = robot.subscribe_ai_detections(on_detection)
```

### IMU Sensor Data

```python
from pib3 import Robot

with Robot(host="172.26.34.149") as robot:
    def on_imu(data):
        accel = data['linear_acceleration']
        gyro = data['angular_velocity']
        print(f"Accel: x={accel['x']:.2f} m/s²")
    
    robot.set_imu_frequency(50)  # 50 Hz
    sub = robot.subscribe_imu(on_imu, data_type="full")
```

### Digital Twin

```python
# Same code works on simulation and real robot
with pib3.Swift() as viz:          # Browser visualization
    viz.run_trajectory("output.json")

with pib3.Webots() as sim:         # Webots simulation
    sim.run_trajectory("output.json")

with pib3.Robot(host="...") as robot:  # Real robot
    robot.run_trajectory("output.json")
```

## Documentation

Full documentation: **[mamrehn.github.io/pib3](https://mamrehn.github.io/pib3/)**

- [Installation Guide](https://mamrehn.github.io/pib3/getting-started/installation/) - Detailed setup for Linux/Windows
- [Quick Start](https://mamrehn.github.io/pib3/getting-started/quickstart/) - Basic usage examples
- [Calibration Guide](https://mamrehn.github.io/pib3/getting-started/calibration/) - Configure joint limits
- [API Reference](https://mamrehn.github.io/pib3/api/) - Complete API documentation
- [Tutorials](https://mamrehn.github.io/pib3/tutorials/) - Step-by-step guides


## Acknowledgments

- [PIB Project](https://pib.rocks/) - Open source humanoid robot
- [Robotics Toolbox for Python](https://github.com/petercorke/robotics-toolbox-python)
