# Configuration

All configuration dataclasses for pib-ik.

## Overview

Configuration is organized into specialized dataclasses:

| Class | Purpose |
|-------|---------|
| `TrajectoryConfig` | Main config combining all others |
| `PaperConfig` | Drawing surface settings |
| `IKConfig` | Inverse kinematics solver |
| `ImageConfig` | Image processing |
| `RobotConfig` | Robot connection |

## TrajectoryConfig

Main configuration class that combines all settings.

::: pib_ik.config.TrajectoryConfig
    options:
      show_root_heading: true
      show_source: true

### Usage

```python
from pib_ik import TrajectoryConfig, PaperConfig, IKConfig, ImageConfig

# Default configuration
config = TrajectoryConfig()

# Custom configuration
config = TrajectoryConfig(
    paper=PaperConfig(size=0.15, drawing_scale=0.9),
    ik=IKConfig(max_iterations=200),
    image=ImageConfig(threshold=100),
    point_density=0.005,
)

# Use with trajectory generation
import pib_ik
trajectory = pib_ik.generate_trajectory("image.png", config=config)
```

---

## PaperConfig

Configuration for the drawing surface.

::: pib_ik.config.PaperConfig
    options:
      show_root_heading: true
      show_source: true

### Usage

```python
from pib_ik import PaperConfig

# Default paper
paper = PaperConfig()

# Large paper, higher table
paper = PaperConfig(
    size=0.20,       # 20cm x 20cm
    height_z=0.80,   # 80cm table height
    drawing_scale=0.85,
)

# Specific position
paper = PaperConfig(
    start_x=0.08,
    center_y=0.18,
    lift_height=0.02,
)
```

### Parameter Guide

| Parameter | Default | Range | Notes |
|-----------|---------|-------|-------|
| `start_x` | 0.10 | 0.05-0.20 | Closer = easier reach |
| `size` | 0.12 | 0.08-0.25 | Limited by arm reach |
| `height_z` | 0.74 | 0.60-0.85 | Table height |
| `drawing_scale` | 0.8 | 0.5-1.0 | Margin within paper |
| `lift_height` | 0.03 | 0.01-0.05 | Pen lift distance |

---

## IKConfig

Configuration for the inverse kinematics solver.

::: pib_ik.config.IKConfig
    options:
      show_root_heading: true
      show_source: true

### Usage

```python
from pib_ik import IKConfig

# Default settings
ik = IKConfig()

# High accuracy
ik = IKConfig(
    max_iterations=300,
    tolerance=0.001,  # 1mm
    step_size=0.2,
)

# Fast (less accurate)
ik = IKConfig(
    max_iterations=50,
    tolerance=0.005,  # 5mm
)

# Use right arm
ik = IKConfig(arm="right")
```

### Parameter Guide

| Parameter | Default | Range | Notes |
|-----------|---------|-------|-------|
| `max_iterations` | 150 | 50-500 | More = slower but better |
| `tolerance` | 0.002 | 0.001-0.01 | Position accuracy (meters) |
| `step_size` | 0.4 | 0.1-0.8 | Gradient descent step |
| `damping` | 0.01 | 0.001-0.1 | Numerical stability |
| `arm` | "left" | "left"/"right" | Which arm to use |

---

## ImageConfig

Configuration for image processing and contour extraction.

::: pib_ik.config.ImageConfig
    options:
      show_root_heading: true
      show_source: true

### Usage

```python
from pib_ik import ImageConfig

# Default settings
image = ImageConfig()

# For light pencil sketches
image = ImageConfig(
    threshold=200,
    simplify_tolerance=3.0,
)

# For detailed images
image = ImageConfig(
    simplify_tolerance=0.5,
    min_contour_length=5,
)

# Noisy images
image = ImageConfig(
    simplify_tolerance=4.0,
    min_contour_length=20,
    min_contour_points=5,
)
```

### Parameter Guide

| Parameter | Default | Range | Notes |
|-----------|---------|-------|-------|
| `threshold` | 128 | 0-255 | Lower = more sensitive |
| `auto_foreground` | True | - | Auto-detect dark/light |
| `simplify_tolerance` | 2.0 | 0.5-5.0 | Higher = simpler |
| `min_contour_length` | 10 | 5-50 | Filter small contours |
| `min_contour_points` | 3 | 3-10 | Min vertices per stroke |
| `margin` | 0.05 | 0-0.2 | Border padding |
| `optimize_path_order` | True | - | Minimize pen travel |

---

## RobotConfig

Configuration for real robot connection.

::: pib_ik.config.RobotConfig
    options:
      show_root_heading: true
      show_source: true

### Usage

```python
from pib_ik import RobotConfig
from pib_ik.backends import RealRobotBackend

# Default settings
config = RobotConfig()

# Custom connection
config = RobotConfig(
    host="192.168.1.100",
    port=9090,
    timeout=10.0,
)

# Create backend from config
robot = RealRobotBackend.from_config(config)
```
