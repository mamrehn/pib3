# Quick Start

Get up and running with pib-ik in 5 minutes.

## Prerequisites

Make sure you have [installed pib-ik](installation.md) with the features you need:

```bash
# Install with all features
pip install "pib-ik[all] @ git+https://github.com/mamrehn/pib3.git"
```

## Your First Trajectory

### Step 1: Create a Drawing

Create or find a simple black-and-white image. For best results:

- Use high contrast (black lines on white background)
- Simple shapes work best
- PNG or JPG format

### Step 2: Generate the Trajectory

```python
import pib3

# Convert image to robot trajectory
trajectory = pib3.generate_trajectory("my_drawing.png")

# Save to JSON file
trajectory.to_json("my_trajectory.json")

print(f"Generated {len(trajectory)} waypoints")
```

That's it! You now have a trajectory file ready for execution.

### Step 3: Visualize (Optional)

Before running on the real robot, visualize the trajectory:

```python
import pib3

# Load the trajectory
trajectory = pib3.Trajectory.from_json("my_trajectory.json")

# Visualize in Swift (opens browser)
with pib3.Swift() as viz:
    viz.run_trajectory(trajectory)
```

### Step 4: Execute on Robot

```python
import pib3

# Load trajectory
trajectory = pib3.Trajectory.from_json("my_trajectory.json")

# Execute on real robot
with pib3.Robot(host="172.26.34.149") as robot:
    robot.run_trajectory(trajectory)
```

---

## Controlling Individual Joints

pib-ik uses a percentage-based system for joint control:

- **0%** = minimum position
- **100%** = maximum position
- **50%** = middle of the range

### Basic Joint Control

```python
from pib3 import Robot

with Robot(host="172.26.34.149") as robot:
    # Move head to center
    robot.set_joint("turn_head_motor", 50.0)

    # Bend elbow to 75%
    robot.set_joint("elbow_left", 75.0)

    # Read current position
    pos = robot.get_joint("elbow_left")
    print(f"Elbow is at {pos:.1f}%")
```

### Save and Restore Poses

```python
from pib3 import Robot

with Robot(host="172.26.34.149") as robot:
    # Save current pose
    saved_pose = robot.get_joints()

    # Move robot around
    robot.set_joint("elbow_left", 25.0)
    robot.set_joint("wrist_left", 75.0)

    # Restore original pose
    robot.set_joints(saved_pose)
```

### Using Radians

If you prefer radians:

```python
from pib3 import Robot

with Robot(host="172.26.34.149") as robot:
    # Set position in radians
    robot.set_joint("elbow_left", 1.25, unit="rad")

    # Read position in radians
    pos_rad = robot.get_joint("elbow_left", unit="rad")
    print(f"Elbow is at {pos_rad:.3f} radians")
```

---

## Swift Visualization

### Running a Trajectory

```python
from pib3 import Swift, Trajectory

# Load trajectory
trajectory = Trajectory.from_json("my_trajectory.json")

# Visualize
with Swift() as viz:
    viz.run_trajectory(trajectory)
```

### Interactive Mode

Launch an interactive session with slider controls:

```python
from pib3 import Swift

viz = Swift()
viz.connect()
viz.launch_interactive()  # Opens browser with sliders

# Open http://localhost:8001 for sliders
# Open http://localhost:52000 for 3D view
# Press Ctrl+C to exit
```

---

## Step-by-Step Processing

For more control, use the step-by-step approach:

```python
import pib3

# Step 1: Convert image to sketch (2D strokes)
sketch = pib3.image_to_sketch("drawing.png")
print(f"Extracted {len(sketch)} strokes")

# Step 2: Convert sketch to trajectory (3D robot motion)
trajectory = pib3.sketch_to_trajectory(sketch)
print(f"Generated {len(trajectory)} waypoints")

# Step 3: Save or execute
trajectory.to_json("output.json")
```

---

## Common Patterns

### Progress Callback

Track IK solving progress:

```python
import pib3

def on_progress(current, total, success):
    status = "OK" if success else "FAILED"
    print(f"Point {current}/{total}: {status}")

sketch = pib3.image_to_sketch("drawing.png")
trajectory = pib3.sketch_to_trajectory(
    sketch,
    progress_callback=on_progress
)
```

### Custom Configuration

Adjust parameters for your setup:

```python
import pib3
from pib3 import TrajectoryConfig, PaperConfig, ImageConfig

config = TrajectoryConfig(
    paper=PaperConfig(
        size=0.15,           # 15cm paper
        height_z=0.74,       # Table height
        drawing_scale=0.9,   # Use 90% of paper
    ),
    image=ImageConfig(
        threshold=100,       # Darker threshold
        simplify_tolerance=3.0,  # More simplification
    ),
)

trajectory = pib3.generate_trajectory("drawing.png", config=config)
```

---

## Available Joint Names

Here are the joints you can control:

### Head
- `turn_head_motor` - Horizontal rotation
- `tilt_forward_motor` - Vertical tilt

### Left Arm
- `shoulder_vertical_left`
- `shoulder_horizontal_left`
- `upper_arm_left_rotation`
- `elbow_left`
- `lower_arm_left_rotation`
- `wrist_left`

### Left Hand
- `thumb_left_opposition`, `thumb_left_stretch`
- `index_left_stretch`
- `middle_left_stretch`
- `ring_left_stretch`
- `pinky_left_stretch`

### Right Arm/Hand
Same as left, replace `_left` with `_right`.

---

## Troubleshooting

!!! warning "No successful IK solutions found"
    The target drawing positions may be outside the robot's reach. Try:

    - Moving the paper closer to the robot
    - Using a smaller drawing scale
    - Adjusting paper height

!!! warning "Robot not responding"
    1. Check robot is powered on
    2. Verify rosbridge is running
    3. Confirm IP address is correct
    4. Test with: `ping 172.26.34.149`

!!! warning "Joints not reaching target positions"
    Joint limits may not be calibrated. See the [Calibration Guide](calibration.md).

---

## Next Steps

- [Calibration Guide](calibration.md) - Calibrate joints for accurate control
- [Image to Trajectory Tutorial](../tutorials/image-to-trajectory.md) - Deep dive into image processing
- [Controlling the Robot Tutorial](../tutorials/controlling-robot.md) - Master joint control
- [API Reference](../api/index.md) - Full API documentation
