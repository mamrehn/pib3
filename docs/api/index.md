# API Reference

Complete API documentation for pib-ik.

## Module Overview

```
pib_ik
├── Core Functions
│   ├── generate_trajectory()    # One-shot image to trajectory
│   ├── image_to_sketch()        # Image to 2D strokes
│   └── sketch_to_trajectory()   # 2D strokes to 3D trajectory
│
├── Core Types
│   ├── Stroke                   # Single continuous line
│   ├── Sketch                   # Collection of strokes
│   └── Trajectory               # Robot joint positions
│
├── Configuration
│   ├── TrajectoryConfig         # Main configuration
│   ├── PaperConfig              # Drawing surface settings
│   ├── IKConfig                 # IK solver settings
│   ├── ImageConfig              # Image processing settings
│   └── RobotConfig              # Robot connection settings
│
├── Backends
│   ├── Robot / RealRobotBackend # Real robot via rosbridge
│   ├── Swift / SwiftBackend     # Browser visualization
│   └── Webots / WebotsBackend   # Webots simulator
│
├── Tools
│   └── calibrate_joints         # Joint calibration tool
│
└── Hand Poses
    ├── left_hand_pose()         # Get left hand joint positions
    └── right_hand_pose()        # Get right hand joint positions
```

## Quick Reference

### Main Functions

| Function | Description |
|----------|-------------|
| [`generate_trajectory()`](trajectory.md#pib_ik.generate_trajectory) | Convert image directly to trajectory |
| [`image_to_sketch()`](image.md#pib_ik.image.image_to_sketch) | Convert image to 2D strokes |
| [`sketch_to_trajectory()`](trajectory.md#pib_ik.trajectory.sketch_to_trajectory) | Convert strokes to 3D trajectory |

### Core Types

| Type | Description |
|------|-------------|
| [`Stroke`](types.md#pib_ik.types.Stroke) | Single continuous drawing line |
| [`Sketch`](types.md#pib_ik.types.Sketch) | Collection of strokes |
| [`Trajectory`](trajectory.md#pib_ik.trajectory.Trajectory) | Robot joint positions over time |

### Backends

| Backend | Description |
|---------|-------------|
| [`RealRobotBackend`](backends/robot.md) | Control real PIB robot |
| [`SwiftBackend`](backends/swift.md) | Browser-based visualization |
| [`WebotsBackend`](backends/webots.md) | Webots simulator |

## Import Examples

```python
# Main functions
import pib_ik
trajectory = pib_ik.generate_trajectory("image.png")

# Types
from pib_ik import Stroke, Sketch, Trajectory

# Configuration
from pib_ik import TrajectoryConfig, PaperConfig, IKConfig, ImageConfig

# Backends (short names)
from pib_ik import Robot, Swift, Webots

# Backends (full names)
from pib_ik.backends import RealRobotBackend, SwiftBackend, WebotsBackend

# Hand poses
from pib_ik import left_hand_pose, right_hand_pose
```

## Documentation Sections

<div class="grid cards" markdown>

-   :material-vector-polyline: **[Core Types](types.md)**

    ---

    Stroke, Sketch - Data structures for 2D drawings

-   :material-cog: **[Configuration](config.md)**

    ---

    All configuration dataclasses and options

-   :material-image: **[Image Processing](image.md)**

    ---

    Image to sketch conversion functions

-   :material-robot-industrial: **[Trajectory Generation](trajectory.md)**

    ---

    Trajectory class and IK functions

-   :material-chip: **[Backends](backends/base.md)**

    ---

    Robot, Swift, and Webots backends

-   :material-tools: **[Tools](tools.md)**

    ---

    Calibration and utility tools

-   :material-hand-wave: **[Hand Poses](hand-poses.md)**

    ---

    Hand position presets and functions

</div>
