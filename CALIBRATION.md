# Joint Calibration Guide

This guide explains how to calibrate joint limits for your PIB robot. Calibration is required to use the percentage-based joint control API, which allows you to control joints using values from 0% (minimum position) to 100% (maximum position).

## Why Calibrate?

The `pib-ik` library uses a percentage-based unit system by default:

```python
# Move elbow to middle of its range
robot.set_joint("elbow_left", 50.0)  # 50%

# Move head to center (looking forward)
robot.set_joint("turn_head_motor", 50.0)  # 50%
```

For this to work correctly, the library needs to know the actual minimum and maximum positions (in radians) for each joint on your specific robot. These limits may vary between robots due to hardware differences.

## Prerequisites

1. **Robot connected and powered on**
2. **Rosbridge running** on the robot (default port: 9090)
3. **Cerebra** or another control interface to manually move joints
4. **pib-ik installed** with robot support:
   ```bash
   pip install "pib-ik[robot] @ git+https://github.com/mamrehn/pib_ik.git"
   ```

## Quick Start

```bash
# Calibrate left hand joints
python -m pib_ik.tools.calibrate_joints --host 172.26.34.149 --group left_hand

# Calibrate left arm
python -m pib_ik.tools.calibrate_joints --host 172.26.34.149 --group left_arm

# Calibrate everything
python -m pib_ik.tools.calibrate_joints --host 172.26.34.149 --all
```

## Calibration Process

### Step 1: List Available Joints

```bash
python -m pib_ik.tools.calibrate_joints --list
```

Output:
```
Available joint groups:

  head:
    - turn_head_motor
    - tilt_forward_motor

  left_arm:
    - shoulder_vertical_left
    - shoulder_horizontal_left
    - upper_arm_left_rotation
    - elbow_left
    - lower_arm_left_rotation
    - wrist_left

  left_hand:
    - thumb_left_opposition
    - thumb_left_stretch
    - index_left_stretch
    - middle_left_stretch
    - ring_left_stretch
    - pinky_left_stretch

  right_arm:
    - shoulder_vertical_right
    - shoulder_horizontal_right
    - upper_arm_right_rotation
    - elbow_right
    - lower_arm_right_rotation
    - wrist_right

  right_hand:
    - thumb_right_opposition
    - thumb_right_stretch
    - index_right_stretch
    - middle_right_stretch
    - ring_right_stretch
    - pinky_right_stretch

Total joints: 26
```

### Step 2: Run Calibration

Choose which joints to calibrate:

```bash
# By group
python -m pib_ik.tools.calibrate_joints --host YOUR_ROBOT_IP --group left_arm

# Specific joints only
python -m pib_ik.tools.calibrate_joints --host YOUR_ROBOT_IP --joints elbow_left wrist_left

# All joints (takes longer)
python -m pib_ik.tools.calibrate_joints --host YOUR_ROBOT_IP --all
```

### Step 3: Follow the Prompts

For each joint, the tool will:

1. **Ask you to move the joint to its MINIMUM position** using Cerebra
   - Press Enter when ready
   - The tool reads and records the position

2. **Ask you to move the joint to its MAXIMUM position** using Cerebra
   - Press Enter when ready
   - The tool reads and records the position

You can press `s` to skip any joint you don't want to calibrate.

### Step 4: Review and Save

After calibrating all selected joints, the tool shows a summary:

```
CALIBRATION SUMMARY
============================================================
  elbow_left: min=0.1234, max=2.3456 (range: 127.3 deg)
  wrist_left: min=-1.5700, max=1.5700 (range: 180.0 deg)

Save calibrated values to config file?
  [y] Yes, save
  [n] No, discard
```

Enter `y` to save the values to `pib_ik/resources/joint_limits.yaml`.

## Command Reference

```
usage: calibrate_joints.py [-h] [--host HOST] [--port PORT]
                           [--joints JOINTS [JOINTS ...]]
                           [--group {head,left_arm,left_hand,right_arm,right_hand}]
                           [--all] [--list]

Options:
  --host HOST           Robot IP address (default: 172.26.34.149)
  --port PORT           Rosbridge port (default: 9090)
  --joints JOINTS       Specific joints to calibrate
  --group GROUP         Calibrate a joint group
  --all                 Calibrate all joints
  --list                List available joints and groups
```

## Configuration File

Calibration values are stored in `pib_ik/resources/joint_limits.yaml`:

```yaml
joints:
  # Head joints
  turn_head_motor:
    min: -1.5700
    max: 1.5700
  tilt_forward_motor:
    min: -0.5000
    max: 0.5000

  # Left arm joints
  elbow_left:
    min: 0.0000
    max: 2.5000
  # ... etc
```

### Manual Editing

You can also edit this file manually if you know the exact values:

```yaml
joints:
  elbow_left:
    min: 0.0      # Fully extended
    max: 2.5      # Fully bent (~143 degrees)
```

## Tips for Accurate Calibration

1. **Move joints slowly** to their physical limits to avoid damage
2. **Be consistent** - always use the same definition of "minimum" and "maximum"
3. **Don't force joints** past their mechanical stops
4. **Recalibrate** if you notice position drift or inaccuracy
5. **Test after calibration** with simple percentage commands:
   ```python
   robot.set_joint("elbow_left", 0)    # Should go to min
   robot.set_joint("elbow_left", 100)  # Should go to max
   robot.set_joint("elbow_left", 50)   # Should be in middle
   ```

## Using Calibrated Joints

After calibration, use the percentage-based API:

```python
from pib_ik import Robot

with Robot(host="172.26.34.149") as robot:
    # Percentage (default) - works across all backends
    robot.set_joint("turn_head_motor", 50.0)   # Center
    robot.set_joint("turn_head_motor", 0.0)    # Full left
    robot.set_joint("turn_head_motor", 100.0)  # Full right

    # Read position in percentage
    pos = robot.get_joint("turn_head_motor")
    print(f"Head at {pos:.1f}%")

    # Still supports radians if needed
    robot.set_joint("elbow_left", 1.25, unit="rad")
    pos_rad = robot.get_joint("elbow_left", unit="rad")
```

## Troubleshooting

### "Could not read position for joint X"

- Check that the motor is connected and powered
- Verify the joint name is correct (use `--list`)
- Ensure rosbridge is running on the robot

### Values seem swapped

The tool automatically swaps min/max if you enter them in the wrong order. If positions still seem wrong, recalibrate that joint.

### Robot not reachable

```
Error connecting to robot: Failed to connect to ROS
```

- Check robot IP address
- Verify rosbridge is running: `ros2 launch rosbridge_server rosbridge_websocket_launch.xml`
- Check network connectivity: `ping YOUR_ROBOT_IP`

## See Also

- [Installation Guide](INSTALLATION.md) - How to install pib-ik
- [README](README.md) - Quick start and overview
