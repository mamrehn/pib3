# Swift Visualization

Use browser-based 3D visualization to test trajectories without hardware.

## Objectives

By the end of this tutorial, you will:

- Launch Swift visualization in your browser
- Visualize robot trajectories
- Use interactive slider controls
- Add visual elements like paper
- Control the robot model programmatically

## Prerequisites

- pib-ik installed with visualization support: `pip install "pib-ik[viz] @ git+https://github.com/mamrehn/pib3.git"`
- A modern web browser

---

## What is Swift?

Swift is a browser-based 3D robot visualization tool from the Robotics Toolbox. It renders the robot model in real-time and responds to joint position changes.

When you launch Swift:

- **3D View**: Opens at `http://localhost:52000`
- **Slider Controls** (interactive mode): Opens at `http://localhost:8001`

---

## Basic Usage

### Running a Trajectory

```python
from pib3 import Swift, Trajectory

# Load a trajectory
trajectory = Trajectory.from_json("my_trajectory.json")

# Visualize it
with Swift() as viz:
    viz.run_trajectory(trajectory)
```

This opens your browser and plays back the trajectory animation.

### Real-time vs Fast Mode

```python
from pib3 import Swift

# Real-time playback (default)
with Swift(realtime=True) as viz:
    viz.run_trajectory(trajectory)

# Fast playback (as fast as possible)
with Swift(realtime=False) as viz:
    viz.run_trajectory(trajectory)
```

---

## Interactive Mode

Launch an interactive session with slider controls:

```python
from pib3 import Swift

# Create and connect
viz = Swift()
viz.connect()

# Launch interactive mode
viz.launch_interactive()

# This blocks until you press Ctrl+C
```

When running:

1. Open `http://localhost:52000` for the 3D view
2. Open `http://localhost:8001` for slider controls
3. Move sliders to control joints in real-time
4. Press ++ctrl+c++ in the terminal to exit

### Using a Different Port

```python
viz.launch_interactive(port=8080)  # Sliders at http://localhost:8080
```

---

## Controlling Joints Programmatically

Swift uses the same API as the real robot:

```python
from pib3 import Swift
import time

with Swift() as viz:
    # Set joint positions (percentage)
    viz.set_joint("turn_head_motor", 0.0)    # Look left
    time.sleep(1)
    viz.set_joint("turn_head_motor", 100.0)  # Look right
    time.sleep(1)
    viz.set_joint("turn_head_motor", 50.0)   # Center

    # Read positions
    pos = viz.get_joint("turn_head_motor")
    print(f"Head at {pos:.1f}%")

    # Set multiple joints
    viz.set_joints({
        "shoulder_vertical_left": 30.0,
        "elbow_left": 60.0,
        "wrist_left": 50.0,
    })
```

---

## Adding Visual Elements

### Adding Paper

Visualize the drawing surface:

```python
from pib3 import Swift

with Swift() as viz:
    # Add paper at specific position
    viz.add_paper(
        center_x=0.15,    # X position (forward from robot)
        center_y=0.15,    # Y position (left of robot)
        height_z=0.74,    # Height (table level)
        size=0.12,        # Paper size in meters
    )

    # Now run your trajectory
    trajectory = Trajectory.from_json("my_trajectory.json")
    viz.run_trajectory(trajectory)
```

---

## Visualizing During Trajectory Generation

Watch the IK solving process:

```python
import pib3

sketch = pib3.image_to_sketch("drawing.png")

# Enable visualization during IK
trajectory = pib3.sketch_to_trajectory(
    sketch,
    visualize=True  # Opens Swift and shows IK solving
)
```

---

## Complete Example: Interactive Pose Editor

```python
"""
Interactive pose editor using Swift visualization.
"""
from pib3 import Swift
import time

def main():
    viz = Swift()
    viz.connect()

    print("Swift Pose Editor")
    print("=" * 40)
    print("3D View: http://localhost:52000")
    print("Sliders: http://localhost:8001")
    print()
    print("Commands:")
    print("  s - Save current pose")
    print("  r - Restore saved pose")
    print("  z - Zero all joints")
    print("  q - Quit")
    print()

    saved_pose = None

    # Start slider server
    viz._start_slider_server(port=8001)

    try:
        while True:
            # Update from sliders
            viz._update_from_sliders()
            time.sleep(0.05)

            # Check for keyboard input (non-blocking would be better)
            # This is a simplified example
    except KeyboardInterrupt:
        pass
    finally:
        viz.disconnect()
        print("\nExiting...")

if __name__ == "__main__":
    main()
```

---

## Animation with Progress Tracking

```python
from pib3 import Swift, Trajectory

trajectory = Trajectory.from_json("my_trajectory.json")

def on_progress(current, total):
    percent = current / total * 100
    bar_length = 40
    filled = int(bar_length * current / total)
    bar = "█" * filled + "░" * (bar_length - filled)
    print(f"\r[{bar}] {percent:.1f}%", end="")

with Swift() as viz:
    viz.run_trajectory(
        trajectory,
        rate_hz=20.0,
        progress_callback=on_progress,
    )
    print("\nDone!")
```

---

## Preset Poses

Swift includes some preset poses for quick testing:

```python
from pib3 import Swift
import time

with Swift() as viz:
    # The slider UI has preset buttons:
    # - Zero Pose: All joints at 0
    # - T-Pose: Arms extended to sides

    # Programmatically set T-pose-like position
    viz.set_joints({
        "shoulder_vertical_left": -33.0,   # ~-0.57 rad
        "elbow_left": 45.0,                # ~0.785 rad
        "shoulder_vertical_right": 33.0,
        "elbow_right": 45.0,
    })

    time.sleep(2)
```

---

## Tips for Effective Visualization

### Performance Tips

```python
from pib3 import Swift

# Disable real-time for faster batch processing
with Swift(realtime=False) as viz:
    for trajectory in trajectories:
        viz.run_trajectory(trajectory)
```

### Browser Tips

- Use Chrome or Firefox for best performance
- If 3D view doesn't load, refresh the page
- Multiple browser tabs can view the same robot

### Debugging Tips

```python
from pib3 import Swift

with Swift() as viz:
    # Check current joint configuration
    joints = viz.get_joints()
    for name, pos in sorted(joints.items()):
        print(f"{name}: {pos:.1f}%")
```

---

## Troubleshooting

!!! warning "Browser shows blank page"
    **Cause:** WebSocket connection issue.

    **Solution:**

    1. Wait a few seconds and refresh
    2. Check that no other Swift instance is running
    3. Try a different browser

!!! warning "Sliders not responding"
    **Cause:** HTTP server port conflict.

    **Solution:**

    ```python
    viz.launch_interactive(port=8002)  # Try different port
    ```

!!! warning "'swift' module not found"
    **Cause:** Visualization dependencies not installed.

    **Solution:**

    ```bash
    pip install "pib-ik[viz] @ git+https://github.com/mamrehn/pib3.git"
    ```

!!! warning "Robot model not loading"
    **Cause:** URDF or mesh files issue.

    **Solution:** The package includes bundled resources. If issues persist, reinstall:

    ```bash
    pip uninstall pib-ik
    pip install "pib-ik[viz] @ git+https://github.com/mamrehn/pib3.git"
    ```

!!! warning "Animation is jerky"
    **Cause:** Rate too high for system.

    **Solution:**

    ```python
    viz.run_trajectory(trajectory, rate_hz=10.0)  # Lower rate
    ```

---

## Next Steps

- [Controlling the Robot](controlling-robot.md) - Apply to real hardware
- [Image to Trajectory](image-to-trajectory.md) - Generate trajectories
- [API Reference: SwiftBackend](../api/backends/swift.md) - Full API documentation
