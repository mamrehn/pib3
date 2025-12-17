# pib-ik

Inverse kinematics and trajectory generation for the PIB (Printable, Intelligent Bot) humanoid robot. Convert images to robot arm trajectories for drawing tasks.

![PIB Robot](https://pib.rocks/wp-content/uploads/2023/09/pib_1-1.png)

## Installation

```bash
# Basic installation
pip install git+https://github.com/mamrehn/pib_ik.git

# With image processing support (recommended)
pip install "pib-ik[image] @ git+https://github.com/mamrehn/pib_ik.git"

# With Swift visualization
pip install "pib-ik[viz] @ git+https://github.com/mamrehn/pib_ik.git"

# With real robot support
pip install "pib-ik[robot] @ git+https://github.com/mamrehn/pib_ik.git"

# All features
pip install "pib-ik[all] @ git+https://github.com/mamrehn/pib_ik.git"
```

### Development Installation

```bash
# Clone the repository
git clone https://github.com/mamrehn/pib_ik.git
cd pib_ik

# Install in development mode
pip install -e .

# Or with all optional dependencies
pip install -e ".[all]"
```

## Quick Start

```python
import pib_ik

# One-shot: image to trajectory
trajectory = pib_ik.generate_trajectory("drawing.png")
trajectory.to_json("output.json")

# Or step-by-step for more control:
sketch = pib_ik.image_to_sketch("drawing.png")
trajectory = pib_ik.sketch_to_trajectory(sketch)
```

## Backends

### Swift Visualization (Browser-based 3D viewer)

```python
import pib_ik

trajectory = pib_ik.generate_trajectory("drawing.png")

# Run trajectory visualization
with pib_ik.Swift() as viz:
    viz.run_trajectory(trajectory)

# Or launch interactive mode with slider controls
with pib_ik.Swift() as viz:
    viz.launch_interactive()  # Opens browser with joint sliders
```

### Real Robot (via rosbridge)

```python
import pib_ik

trajectory = pib_ik.generate_trajectory("drawing.png")

# Execute on real robot
with pib_ik.Robot(host="172.26.34.149") as robot:
    robot.run_trajectory(trajectory)

# Manual joint control
with pib_ik.Robot(host="172.26.34.149") as robot:
    # Set a single joint
    robot.set_joint("elbow_left", 0.5)  # radians

    # Set with verification (waits until joint reaches position)
    success = robot.set_joint("elbow_left", 0.5, verify=True)

    # Read current joint position
    angle = robot.get_joint("elbow_left")
    print(f"Elbow at {angle:.2f} rad")

    # Save current pose
    saved_pose = robot.get_joints()

    # ... do something ...

    # Restore saved pose
    robot.set_joints(saved_pose)

    # Set multiple joints with verification
    robot.set_joints({
        "shoulder_vertical_left": 0.3,
        "elbow_left": 0.8,
    }, verify=True, verify_timeout=2.0)
```

### Webots Simulation

```python
import pib_ik

# In a Webots controller script
with pib_ik.Webots(robot_instance) as webots:
    trajectory = pib_ik.Trajectory.from_json("trajectory.json")
    webots.run_trajectory(trajectory)
```

## Configuration

All configuration is done via dataclasses:

```python
import pib_ik
from pib_ik import TrajectoryConfig, PaperConfig, IKConfig, ImageConfig

# Custom paper configuration
config = TrajectoryConfig(
    paper=PaperConfig(
        size=0.15,           # 15cm x 15cm drawing area
        offset_x=0.0,        # Paper center X offset
        offset_y=0.15,       # Paper center Y offset (left of robot)
        offset_z=0.74,       # Paper height
        drawing_scale=0.85,  # Use 85% of paper area
    ),
    ik=IKConfig(
        max_iterations=100,
        tolerance=0.001,
        damping=0.1,
    ),
    image=ImageConfig(
        simplify_tolerance=2.0,
        min_contour_length=5,
    ),
)

trajectory = pib_ik.generate_trajectory("drawing.png", config=config)
```

## API Reference

### Core Types

- `Stroke` - A single 2D line (list of (x, y) points normalized to [0, 1])
- `Sketch` - A collection of strokes representing a drawing
- `Trajectory` - Robot joint positions over time

### Functions

- `image_to_sketch(image, config)` - Convert image to 2D strokes
- `sketch_to_trajectory(sketch, config)` - Convert strokes to robot trajectory
- `generate_trajectory(image, config)` - One-shot image to trajectory

### Backends

- `Swift` / `SwiftBackend` - Browser-based 3D visualization
- `Robot` / `RealRobotBackend` - Real robot via rosbridge websocket
- `Webots` / `WebotsBackend` - Webots simulator integration

All backends provide these methods:

| Method | Description |
|--------|-------------|
| `get_joint(name)` | Get current position of a joint (radians) |
| `get_joints(names=None)` | Get positions of multiple joints (dict) |
| `set_joint(name, pos, verify=False)` | Set a single joint position |
| `set_joints(positions, verify=False)` | Set multiple joints (dict or saved pose) |
| `run_trajectory(trajectory)` | Execute a full trajectory |

The `verify` parameter waits until the joint(s) reach the target position within tolerance.

## Tools

### Proto to URDF Converter

Convert Webots `.proto` files to URDF format:

```python
from pib_ik.tools import convert_proto_to_urdf

# Convert proto file to URDF
urdf_content = convert_proto_to_urdf("pibsim_webots/protos/pib.proto")
with open("pib_model.urdf", "w") as f:
    f.write(urdf_content)
```

## Trajectory JSON Format

Trajectories are stored in a standardized JSON format:

```json
{
  "format_version": "1.0",
  "unit": "radians",
  "coordinate_frame": "urdf",
  "joint_names": ["turn_head_motor", "tilt_forward_motor", ...],
  "waypoints": [
    [0.1, 0.2, 0.3, ...],
    [0.15, 0.25, 0.35, ...]
  ],
  "metadata": {
    "created_at": "2024-01-01T12:00:00Z",
    "offsets": {
      "webots": 1.0,
      "description": "Add to convert radians -> Webots motor positions"
    }
  }
}
```

Each waypoint is an array of joint positions (in radians) matching the order of `joint_names`.

All joint positions are stored in radians. Backend-specific conversions:
- **Webots**: Adds 1.0 radian offset
- **Real Robot**: Converts to centidegrees (degrees × 100)

## Unit Conversions

| Context | Unit | Conversion |
|---------|------|------------|
| Trajectory JSON | radians | canonical format |
| Webots motors | radians + 1.0 | `webots_pos = radians + 1.0` |
| Real robot (ROS) | centidegrees | `centideg = degrees(radians) * 100` |

## Robot Specifications

- **Robot**: PIB humanoid (36 DOF)
- **Drawing Arm**: Left arm (6 DOF) + index finger
- **End Effector**: Left index finger tip
- **Default Workspace**: ~12cm x 12cm at ~74cm height

## Requirements

Core dependencies:
- numpy >= 1.20.0
- Pillow >= 8.0.0
- roboticstoolbox-python >= 1.0.0
- spatialmath-python >= 1.0.0

Optional:
- scikit-image >= 0.18.0 (image processing)
- swift-sim >= 1.0.0 (visualization)
- roslibpy >= 1.0.0 (real robot)

## License

MIT License

## Acknowledgments

- [PIB Project](https://pib.rocks/) - Open source humanoid robot
- [Robotics Toolbox for Python](https://github.com/petercorke/robotics-toolbox-python)
- [Webots](https://cyberbotics.com/) - Robot simulator
