# Controlling the Robot

Master the joint control API for precise robot manipulation.

## Objectives

By the end of this tutorial, you will:

- Connect to a real PIB robot
- Control joints using percentage and radian units
- Save and restore robot poses
- Use verification to ensure movements complete
- Control hand positions

## Prerequisites

- pib3 installed with robot support: `pip install "pib3[robot] @ git+https://github.com/mamrehn/pib3.git"`
- A PIB robot connected to your network
- Rosbridge running on the robot
- [Calibrated joint limits](../getting-started/calibration.md) (recommended)

---

## Connecting to the Robot

### Basic Connection

```python
from pib3 import Robot

# Connect using context manager (recommended)
with Robot(host="172.26.34.149") as robot:
    # Robot is connected here
    print(f"Connected: {robot.is_connected}")
    # ... control the robot ...
# Automatically disconnected when exiting the block
```

### Manual Connection

```python
from pib3 import Robot

robot = Robot(host="172.26.34.149", port=9090)
robot.connect()

try:
    # Control the robot
    print(f"Connected: {robot.is_connected}")
finally:
    robot.disconnect()
```

### Connection Options

```python
from pib3 import Robot

robot = Robot(
    host="172.26.34.149",  # Robot IP address
    port=9090,             # Rosbridge port (default: 9090)
    timeout=5.0,           # Connection timeout in seconds
)
```

---

## Percentage-Based Control

The percentage system maps joint ranges to 0-100%:

- **0%** = minimum position (joint limit)
- **100%** = maximum position (joint limit)
- **50%** = middle of range

### Setting Joint Positions

```python
from pib3 import Robot

with Robot(host="172.26.34.149") as robot:
    # Move head to center (50%)
    robot.set_joint("turn_head_motor", 50.0)

    # Move elbow to 75% of its range
    robot.set_joint("elbow_left", 75.0)

    # Move shoulder down (0%)
    robot.set_joint("shoulder_vertical_left", 0.0)
```

### Reading Joint Positions

Joint positions are received asynchronously from the robot. By default, `get_joint()` and `get_joints()` wait up to 5 seconds for data to arrive.

```python
from pib3 import Robot

with Robot(host="172.26.34.149") as robot:
    # Read single joint (waits up to 5s by default)
    elbow_pos = robot.get_joint("elbow_left")
    print(f"Elbow is at {elbow_pos:.1f}%")

    # Read with custom timeout
    elbow_pos = robot.get_joint("elbow_left", timeout=2.0)

    # Read multiple joints
    arm_joints = robot.get_joints([
        "shoulder_vertical_left",
        "elbow_left",
        "wrist_left"
    ])
    for name, pos in arm_joints.items():
        print(f"  {name}: {pos:.1f}%")

    # Read all available joints
    all_joints = robot.get_joints()
    print(f"Total joints read: {len(all_joints)}")
```

---

## Using Radians

If you prefer working with radians:

```python
from pib3 import Robot

with Robot(host="172.26.34.149") as robot:
    # Set position in radians
    robot.set_joint("elbow_left", 1.25, unit="rad")

    # Read position in radians
    pos_rad = robot.get_joint("elbow_left", unit="rad")
    print(f"Elbow is at {pos_rad:.3f} radians")

    # Set multiple joints in radians
    robot.set_joints({
        "shoulder_vertical_left": 0.5,
        "elbow_left": 1.2,
    }, unit="rad")
```

---

## Setting Multiple Joints

### Using a Dictionary

```python
from pib3 import Robot

with Robot(host="172.26.34.149") as robot:
    # Move multiple joints at once
    robot.set_joints({
        "shoulder_vertical_left": 30.0,
        "shoulder_horizontal_left": 50.0,
        "elbow_left": 60.0,
        "wrist_left": 50.0,
    })
```

### Setting All Joints

```python
from pib3 import Robot

with Robot(host="172.26.34.149") as robot:
    # Get current pose as dictionary
    current = robot.get_joints()

    # Modify specific values
    current["elbow_left"] = 75.0
    current["turn_head_motor"] = 50.0

    # Apply all joints
    robot.set_joints(current)
```

---

## Saving and Restoring Poses

A common pattern is to save a pose, do something, then restore:

```python
from pib3 import Robot

with Robot(host="172.26.34.149") as robot:
    # Save current pose
    saved_pose = robot.get_joints()
    print(f"Saved {len(saved_pose)} joint positions")

    # Move robot around
    robot.set_joint("turn_head_motor", 0.0)    # Look left
    robot.set_joint("turn_head_motor", 100.0)  # Look right

    # Restore original pose
    robot.set_joints(saved_pose)
    print("Restored original pose")
```

### Saving to File

```python
import json
from pib3 import Robot

with Robot(host="172.26.34.149") as robot:
    # Save pose to file
    pose = robot.get_joints()
    with open("my_pose.json", "w") as f:
        json.dump(pose, f, indent=2)

# Later: Load and restore
with Robot(host="172.26.34.149") as robot:
    with open("my_pose.json") as f:
        saved_pose = json.load(f)
    robot.set_joints(saved_pose)
```

---

## Position Verification

Use verification to ensure joints reach their target positions:

```python
from pib3 import Robot

with Robot(host="172.26.34.149") as robot:
    # Wait until joint reaches position
    success = robot.set_joint(
        "elbow_left",
        50.0,
        async_=False,        # Wait for completion
        timeout=2.0,         # Max wait time (seconds)
        tolerance=2.0,       # Acceptable error (percentage)
    )

    if success:
        print("Joint reached target position!")
    else:
        print("Joint did not reach target in time")
```

### Waiting for Multiple Joints

```python
from pib3 import Robot

with Robot(host="172.26.34.149") as robot:
    success = robot.set_joints(
        {
            "shoulder_vertical_left": 30.0,
            "elbow_left": 60.0,
        },
        async_=False,
        timeout=3.0,
    )

    if success:
        print("All joints reached targets!")
```

---

## Controlling the Hands

### Left Hand Joints

```python
from pib3 import Robot

with Robot(host="172.26.34.149") as robot:
    # Open all fingers (0%)
    robot.set_joints({
        "thumb_left_opposition": 0.0,
        "thumb_left_stretch": 0.0,
        "index_left_stretch": 0.0,
        "middle_left_stretch": 0.0,
        "ring_left_stretch": 0.0,
        "pinky_left_stretch": 0.0,
    })

    # Close all fingers (100%)
    robot.set_joints({
        "thumb_left_opposition": 100.0,
        "thumb_left_stretch": 100.0,
        "index_left_stretch": 100.0,
        "middle_left_stretch": 100.0,
        "ring_left_stretch": 100.0,
        "pinky_left_stretch": 100.0,
    })
```

### Using Hand Pose Presets

```python
from pib3 import Robot, left_hand_pose

with Robot(host="172.26.34.149") as robot:
    # Open hand (grip=0.0)
    robot.set_joints(left_hand_pose(0.0), unit="rad")

    # Half closed (grip=0.5)
    robot.set_joints(left_hand_pose(0.5), unit="rad")

    # Fully closed (grip=1.0)
    robot.set_joints(left_hand_pose(1.0), unit="rad")
```

---

## Running Trajectories

Execute pre-generated trajectories:

```python
from pib3 import Robot, Trajectory

with Robot(host="172.26.34.149") as robot:
    # Load trajectory from file
    trajectory = Trajectory.from_json("my_trajectory.json")

    # Execute with progress callback
    def on_progress(current, total):
        print(f"\rProgress: {current}/{total}", end="")

    robot.run_trajectory(
        trajectory,
        rate_hz=20.0,  # Playback rate
        progress_callback=on_progress,
    )
    print("\nDone!")
```

---

## Complete Example: Drawing Session

```python
"""
Complete robot drawing session example.
"""
import pib3
from pib3 import Robot, Trajectory

def main():
    # Generate trajectory from image
    print("Generating trajectory...")
    trajectory = pib3.generate_trajectory("drawing.png")
    trajectory.to_json("session_trajectory.json")
    print(f"Generated {len(trajectory)} waypoints")

    # Connect and execute
    with Robot(host="172.26.34.149") as robot:
        # Save initial pose
        initial_pose = robot.get_joints()
        print("Saved initial pose")

        # Move to starting position
        print("Moving to start position...")
        robot.set_joints({
            "shoulder_vertical_left": 50.0,
            "elbow_left": 50.0,
        }, async_=False)

        # Execute drawing
        print("Drawing...")
        def progress(current, total):
            if current % 100 == 0:
                print(f"  {current}/{total} waypoints")

        robot.run_trajectory(
            trajectory,
            rate_hz=20.0,
            progress_callback=progress,
        )

        print("Drawing complete!")

        # Ask user before restoring
        input("Press Enter to restore initial pose...")
        robot.set_joints(initial_pose)
        print("Restored initial pose")

if __name__ == "__main__":
    main()
```

---

## Available Joints Reference

### Head

| Joint Name | Description |
|------------|-------------|
| `turn_head_motor` | Horizontal rotation (left/right) |
| `tilt_forward_motor` | Vertical tilt (up/down) |

### Left Arm

| Joint Name | Description |
|------------|-------------|
| `shoulder_vertical_left` | Shoulder up/down |
| `shoulder_horizontal_left` | Shoulder forward/back |
| `upper_arm_left_rotation` | Upper arm rotation |
| `elbow_left` | Elbow bend |
| `lower_arm_left_rotation` | Forearm rotation |
| `wrist_left` | Wrist bend |

### Left Hand

| Joint Name | Description |
|------------|-------------|
| `thumb_left_opposition` | Thumb rotation |
| `thumb_left_stretch` | Thumb curl |
| `index_left_stretch` | Index finger curl |
| `middle_left_stretch` | Middle finger curl |
| `ring_left_stretch` | Ring finger curl |
| `pinky_left_stretch` | Pinky curl |

### Right Arm/Hand

Same as left, replace `_left` with `_right`.

---

## Troubleshooting

!!! warning "Connection refused"
    **Cause:** Rosbridge not running or wrong IP.

    **Solution:**

    ```bash
    # On robot, start rosbridge
    ros2 launch rosbridge_server rosbridge_websocket_launch.xml

    # Verify connectivity
    ping 172.26.34.149
    ```

!!! warning "Joints not moving"
    **Cause:** Joint limits not calibrated, or values out of range.

    **Solution:**

    1. [Calibrate joint limits](../getting-started/calibration.md)
    2. Try using radians directly: `robot.set_joint("elbow_left", 1.0, unit="rad")`

!!! warning "Waiting for completion always fails"
    **Cause:** Timeout too short or tolerance too tight.

    **Solution:**

    ```python
    robot.set_joint(
        "elbow_left",
        50.0,
        async_=False,
        timeout=5.0,      # Increase timeout
        tolerance=5.0,    # Increase tolerance
    )
    ```

!!! warning "Robot moves slowly"
    **Cause:** Trajectory rate too low.

    **Solution:**

    ```python
    robot.run_trajectory(trajectory, rate_hz=30.0)  # Increase rate
    ```

---

## Next Steps

- [Swift Visualization](swift-visualization.md) - Test without hardware
- [Calibration Guide](../getting-started/calibration.md) - Calibrate joint limits
- [API Reference: RealRobotBackend](../api/backends/robot.md) - Full API documentation
