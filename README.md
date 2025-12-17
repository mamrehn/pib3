# pib-ik

Inverse kinematics and trajectory generation for the [PIB](https://pib.rocks/) (Printable, Intelligent Bot) humanoid robot. Convert images to robot arm trajectories for drawing tasks.

![PIB Robot](https://pib.rocks/wp-content/uploads/2023/09/pib_1-1.png)

## Features

- Convert images to robot drawing trajectories
- Percentage-based joint control (0-100%) that works across all backends
- Support for real robot, Webots simulation, and Swift 3D visualization
- Interactive joint calibration tool
- Save and restore robot poses

## Quick Install

```bash
pip install "pib-ik[all] @ git+https://github.com/mamrehn/pib_ik.git"
```

For detailed installation instructions including virtual environment setup, see [INSTALLATION.md](INSTALLATION.md).

## Quick Start

### Generate a Drawing Trajectory

```python
import pib_ik

# Convert image to robot trajectory
trajectory = pib_ik.generate_trajectory("drawing.png")
trajectory.to_json("output.json")
```

### Control the Real Robot

```python
from pib_ik import Robot

with Robot(host="172.26.34.149") as robot:
    # Control joints using percentage (0-100%)
    robot.set_joint("turn_head_motor", 50.0)  # Center head
    robot.set_joint("elbow_left", 75.0)       # 75% of range

    # Read current position
    pos = robot.get_joint("elbow_left")
    print(f"Elbow at {pos:.1f}%")

    # Save and restore poses
    saved_pose = robot.get_joints()
    # ... do something ...
    robot.set_joints(saved_pose)

    # Execute a trajectory
    robot.run_trajectory("output.json")
```

### Visualize in Browser (Swift)

```python
from pib_ik import Swift

with Swift() as viz:
    # Run a trajectory
    viz.run_trajectory("output.json")

    # Or launch interactive mode with sliders
    # viz.launch_interactive()
```

## Documentation

| Document | Description |
|----------|-------------|
| [INSTALLATION.md](INSTALLATION.md) | Detailed installation guide for Linux and Windows |
| [CALIBRATION.md](CALIBRATION.md) | How to calibrate joint limits for your robot |

## Joint Control API

All backends (Robot, Swift, Webots) share the same API:

```python
# Percentage (default) - 0% = min position, 100% = max position
robot.set_joint("elbow_left", 50.0)           # Set to 50%
robot.set_joints({"elbow_left": 50.0, "wrist_left": 25.0})

# Radians (optional)
robot.set_joint("elbow_left", 1.25, unit="rad")

# Reading positions
pos = robot.get_joint("elbow_left")           # Returns percentage
pos_rad = robot.get_joint("elbow_left", unit="rad")

# Save/restore poses
saved = robot.get_joints()
robot.set_joints(saved)

# Verification (wait until joint reaches target)
robot.set_joint("elbow_left", 50.0, verify=True)
```

**Note**: The percentage system requires calibrated joint limits. See [CALIBRATION.md](CALIBRATION.md).

## Backends

| Backend | Description | Installation |
|---------|-------------|--------------|
| `Robot` | Real PIB robot via rosbridge | `pip install pib-ik[robot]` |
| `Swift` | Browser-based 3D visualization | `pip install pib-ik[viz]` |
| `Webots` | Webots simulator integration | (included in base) |

## Configuration

```python
from pib_ik import TrajectoryConfig, PaperConfig

config = TrajectoryConfig(
    paper=PaperConfig(
        size=0.15,           # 15cm x 15cm paper
        offset_y=0.15,       # Position left of robot
        offset_z=0.74,       # Paper height
        drawing_scale=0.85,  # Use 85% of paper
    ),
)

trajectory = pib_ik.generate_trajectory("drawing.png", config=config)
```

## Tools

### Joint Calibration

```bash
# Calibrate left hand joints
python -m pib_ik.tools.calibrate_joints --host 172.26.34.149 --group left_hand

# List available joints
python -m pib_ik.tools.calibrate_joints --list
```

See [CALIBRATION.md](CALIBRATION.md) for the complete guide.

## Requirements

- Python 3.8+
- numpy, Pillow, PyYAML
- roboticstoolbox-python, spatialmath-python

Optional:
- scikit-image (image processing)
- swift-sim (visualization)
- roslibpy (real robot)

## License

MIT License

## Acknowledgments

- [PIB Project](https://pib.rocks/) - Open source humanoid robot
- [Robotics Toolbox for Python](https://github.com/petercorke/robotics-toolbox-python)
