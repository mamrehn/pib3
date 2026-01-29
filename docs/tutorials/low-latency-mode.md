# Low-Latency Motor Control

Bypass ROS for direct Tinkerforge motor control with significantly reduced latency.

## Overview

The standard motor control path goes through:

```
Python → roslibpy → rosbridge → ROS node → Tinkerforge
```

This adds ~100-200ms latency per command. Low-latency mode bypasses ROS:

```
Python → Tinkerforge (direct)
```

Reducing latency to ~5-20ms per command.

## When to Use Low-Latency Mode

- **Real-time control**: Hand tracking, gesture following, teleoperation
- **High-frequency updates**: Smooth motion requiring >10 Hz update rates
- **Latency-sensitive applications**: Interactive demos, gaming

## Prerequisites

Install the Tinkerforge Python bindings:

```bash
pip install tinkerforge
```

Ensure the robot's Tinkerforge Brick Daemon is accessible (port 4223).

---

## Quick Start

### Option 1: Provide Mapping at Construction

If you know your servo bricklet UIDs:

```python
import pib3

# Build motor mapping from your robot's servo bricklet UIDs
mapping = pib3.build_motor_mapping(
    servo1_uid="ABC1",  # Right arm bricklet
    servo2_uid="ABC2",  # Shoulder verticals bricklet
    servo3_uid="ABC3",  # Left arm + hand bricklet
)

# Create robot with low-latency enabled
config = pib3.RobotConfig(
    host="172.26.34.149",
    low_latency=pib3.LowLatencyConfig(
        enabled=True,
        motor_mapping=mapping,
    ),
)

with pib3.Robot(config=config) as robot:
    # Motor commands now use direct Tinkerforge control
    robot.set_joint("elbow_left", 0.5, unit="rad")
```

### Option 2: Discover UIDs at Runtime

If you don't know your UIDs:

```python
import pib3

with pib3.Robot(host="172.26.34.149") as robot:
    # Step 1: Discover servo bricklets
    uids = robot.discover_servo_bricklets()
    print(f"Found servo bricklets: {uids}")
    # Output: Found servo bricklets: ['29Fy', '29F5', '29F3']

    # Step 2: Build mapping (order depends on your wiring)
    # You may need to test to determine which UID controls which arm
    mapping = pib3.build_motor_mapping(
        servo1_uid=uids[0],  # Right arm
        servo2_uid=uids[1],  # Shoulders
        servo3_uid=uids[2],  # Left arm
    )

    # Step 3: Configure and enable
    robot.configure_motor_mapping(mapping)
    robot.low_latency_enabled = True

    # Now using low-latency mode
    robot.set_joint("elbow_left", 0.5, unit="rad")
```

---

## Configuration Reference

### LowLatencyConfig

```python
from pib3 import LowLatencyConfig

config = LowLatencyConfig(
    enabled=True,                    # Enable low-latency mode
    tinkerforge_host=None,           # Defaults to robot host IP
    tinkerforge_port=4223,           # Standard Tinkerforge port
    motor_mapping=None,              # Motor-to-bricklet mapping
    sync_to_ros=True,                # Update local cache for get_joint()
    command_timeout=0.5,             # Command timeout in seconds
)
```

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `enabled` | `bool` | `False` | Enable low-latency mode |
| `tinkerforge_host` | `str` | `None` | Tinkerforge daemon host (defaults to robot IP) |
| `tinkerforge_port` | `int` | `4223` | Tinkerforge daemon port |
| `motor_mapping` | `dict` | `None` | Maps motor names to `(bricklet_uid, channel)` |
| `sync_to_ros` | `bool` | `True` | Update local position cache after commands |
| `command_timeout` | `float` | `0.5` | Timeout for direct commands |

### sync_to_ros Explained

When `sync_to_ros=True` (default), the local position cache is updated after each low-latency command. This ensures `get_joint()` returns the correct value.

!!! warning "No ROS Topic Updates"
    Low-latency commands do **not** publish to ROS topics. The position will not be visible in ROS tools like `rostopic echo`. This is intentional - publishing to `/joint_trajectory` would trigger a second motor command.

---

## Helper Functions

### build_motor_mapping()

Creates a complete motor mapping from three servo bricklet UIDs:

```python
from pib3 import build_motor_mapping

mapping = build_motor_mapping(
    servo1_uid="ABC1",  # Controls right arm motors
    servo2_uid="ABC2",  # Controls shoulder vertical motors (both arms)
    servo3_uid="ABC3",  # Controls left arm + left hand motors
)
```

This uses the standard PIB wiring where:

- **Servo 1**: Right arm (shoulder horizontal, elbow, rotation, hand)
- **Servo 2**: Both shoulder verticals
- **Servo 3**: Left arm + left hand

### PIB_SERVO_CHANNELS

Reference for standard PIB channel assignments:

```python
from pib3 import PIB_SERVO_CHANNELS

# See which channel each motor uses
print(PIB_SERVO_CHANNELS["elbow_left"])  # 8
print(PIB_SERVO_CHANNELS["wrist_left"])  # 6
```

---

## Runtime Control

### Enable/Disable at Runtime

```python
with pib3.Robot(host="172.26.34.149") as robot:
    # Configure mapping first
    mapping = pib3.build_motor_mapping("UID1", "UID2", "UID3")
    robot.configure_motor_mapping(mapping)

    # Enable low-latency mode
    robot.low_latency_enabled = True

    # Check status
    print(f"Low-latency available: {robot.low_latency_available}")
    print(f"Low-latency enabled: {robot.low_latency_enabled}")

    # Disable for specific operations
    robot.low_latency_enabled = False
    robot.run_trajectory("trajectory.json")  # Uses ROS

    # Re-enable
    robot.low_latency_enabled = True
```

### Per-Call Override

Force ROS or low-latency mode for specific calls:

```python
# Force ROS mode for this call (even if low-latency is enabled)
robot.set_joint("elbow_left", 0.5, unit="rad", low_latency=False)

# Force low-latency mode (if available)
robot.set_joint("elbow_left", 0.5, unit="rad", low_latency=True)
```

---

## Custom Servo Configuration

Servo channels are automatically configured with default settings when you call `configure_motor_mapping()`. To customize:

### Configure Individual Motor

```python
robot.configure_servo_channel(
    "elbow_left",
    pulse_width_min=700,    # Minimum PWM pulse (microseconds)
    pulse_width_max=2500,   # Maximum PWM pulse (microseconds)
    velocity=9000,          # Max velocity (0.01°/s) = 90°/s
    acceleration=9000,      # Acceleration (0.01°/s²)
    deceleration=9000,      # Deceleration (0.01°/s²)
)
```

### Configure All Motors

```python
robot.configure_all_servo_channels(
    pulse_width_min=700,
    pulse_width_max=2500,
    velocity=12000,         # Faster: 120°/s
    acceleration=15000,
    deceleration=15000,
)
```

---

## Fallback Behavior

Motors not in the Tinkerforge mapping automatically fall back to ROS control:

```python
# If only left arm is mapped, right arm uses ROS
robot.set_joints({
    "elbow_left": 0.5,   # Low-latency (in mapping)
    "elbow_right": 0.5,  # Falls back to ROS (not in mapping)
}, unit="rad")
```

---

## Determining Bricklet Order

If you're unsure which UID controls which motors:

```python
import pib3
import time

with pib3.Robot(host="172.26.34.149") as robot:
    uids = robot.discover_servo_bricklets()
    print(f"Found UIDs: {uids}")

    # Test each UID to see which arm it controls
    for i, uid in enumerate(uids):
        print(f"\nTesting UID {uid} (index {i})...")

        # Create a test mapping with just this bricklet
        test_mapping = {
            "elbow_left": (uid, 8),  # Standard left elbow channel
        }
        robot.configure_motor_mapping(test_mapping)
        robot.low_latency_enabled = True

        # Try to move - watch which arm responds
        robot.set_joint("elbow_left", 0.3, unit="rad")
        time.sleep(1)
        robot.set_joint("elbow_left", 0.0, unit="rad")
        time.sleep(1)

        input(f"Did the LEFT elbow move? (Press Enter to continue)")
```

---

## Complete Example

```python
import pib3
import time
import math

def main():
    # Configuration with low-latency enabled
    mapping = pib3.build_motor_mapping("29Fy", "29F5", "29F3")

    config = pib3.RobotConfig(
        host="172.26.34.149",
        low_latency=pib3.LowLatencyConfig(
            enabled=True,
            motor_mapping=mapping,
        ),
    )

    with pib3.Robot(config=config) as robot:
        print(f"Low-latency available: {robot.low_latency_available}")

        # Smooth sinusoidal motion at 20 Hz
        start_time = time.time()
        duration = 5.0  # seconds
        frequency = 20  # Hz

        while time.time() - start_time < duration:
            t = time.time() - start_time
            # Sinusoidal motion between 0.2 and 0.8 radians
            position = 0.5 + 0.3 * math.sin(2 * math.pi * 0.5 * t)

            robot.set_joint("elbow_left", position, unit="rad")
            time.sleep(1.0 / frequency)

        print("Done!")

if __name__ == "__main__":
    main()
```

---

## Troubleshooting

| Problem | Cause | Solution |
|---------|-------|----------|
| `low_latency_available` is `False` | Tinkerforge not installed | `pip install tinkerforge` |
| | Connection failed | Check robot IP and port 4223 is accessible |
| | No motor mapping | Call `configure_motor_mapping()` first |
| Motors don't move | Wrong bricklet UID | Use `discover_servo_bricklets()` to find correct UIDs |
| | Wrong channel | Check `PIB_SERVO_CHANNELS` for standard assignments |
| `get_joint()` returns old value | `sync_to_ros=False` | Set `sync_to_ros=True` in config |
| Jerky motion | Velocity too low | Increase velocity in `configure_servo_channel()` |

---

## Performance Comparison

| Mode | Typical Latency | Best For |
|------|-----------------|----------|
| ROS (standard) | 100-200ms | Trajectory playback, non-time-critical |
| Low-latency | 5-20ms | Real-time control, high-frequency updates |

!!! tip "When to Use Each"
    Use **ROS mode** for trajectory execution (`run_trajectory()`) where timing is managed by waypoints. Use **low-latency mode** for interactive applications where you need immediate response to input.
