# Hand Poses

Control individual finger joints for hand gestures and poses.

## Overview

The PIB robot has 12 finger joints (6 per hand). All values are in percent where **0% = open** and **100% = closed**.

## HandPose Enum

The `HandPose` enum provides preset poses for quick hand control:

```python
from pib3 import HandPose, LEFT_HAND_JOINTS, RIGHT_HAND_JOINTS

with Robot(host="172.26.34.149") as robot:
    # Preset poses
    robot.set_joints(HandPose.LEFT_OPEN)      # Open left hand
    robot.set_joints(HandPose.LEFT_CLOSED)    # Close left hand
    robot.set_joints(HandPose.RIGHT_OPEN)     # Open right hand
    robot.set_joints(HandPose.RIGHT_CLOSED)   # Close right hand

    # Partial grip using joint lists
    robot.set_joints({j: 50.0 for j in LEFT_HAND_JOINTS})   # 50% grip
    robot.set_joints({j: 75.0 for j in RIGHT_HAND_JOINTS})  # 75% grip
```

## Available Poses

| Pose | Description |
|------|-------------|
| `HandPose.LEFT_OPEN` | Left hand fully open (all joints at 0%) |
| `HandPose.LEFT_CLOSED` | Left hand fully closed (all joints at 100%) |
| `HandPose.RIGHT_OPEN` | Right hand fully open (all joints at 0%) |
| `HandPose.RIGHT_CLOSED` | Right hand fully closed (all joints at 100%) |

## Joint Lists

For custom poses, use the joint lists with the `Joint` enum:

| List | Contents |
|------|----------|
| `LEFT_HAND_JOINTS` | All 6 left hand joints as `Joint` enums |
| `RIGHT_HAND_JOINTS` | All 6 right hand joints as `Joint` enums |

## Available Finger Joints

### Left Hand

| Joint Enum | String Value | Description |
|------------|--------------|-------------|
| `Joint.THUMB_LEFT_OPPOSITION` | `thumb_left_opposition` | Thumb rotation |
| `Joint.THUMB_LEFT_STRETCH` | `thumb_left_stretch` | Thumb bend |
| `Joint.INDEX_LEFT` | `index_left_stretch` | Index finger bend |
| `Joint.MIDDLE_LEFT` | `middle_left_stretch` | Middle finger bend |
| `Joint.RING_LEFT` | `ring_left_stretch` | Ring finger bend |
| `Joint.PINKY_LEFT` | `pinky_left_stretch` | Pinky finger bend |

### Right Hand

| Joint Enum | String Value | Description |
|------------|--------------|-------------|
| `Joint.THUMB_RIGHT_OPPOSITION` | `thumb_right_opposition` | Thumb rotation |
| `Joint.THUMB_RIGHT_STRETCH` | `thumb_right_stretch` | Thumb bend |
| `Joint.INDEX_RIGHT` | `index_right_stretch` | Index finger bend |
| `Joint.MIDDLE_RIGHT` | `middle_right_stretch` | Middle finger bend |
| `Joint.RING_RIGHT` | `ring_right_stretch` | Ring finger bend |
| `Joint.PINKY_RIGHT` | `pinky_right_stretch` | Pinky finger bend |

## Basic Control

```python
from pib3 import Robot, Joint

with Robot(host="172.26.34.149") as robot:
    # Individual finger control
    robot.set_joint(Joint.INDEX_LEFT, 100.0)  # Fully closed
    robot.set_joint(Joint.INDEX_LEFT, 0.0)    # Fully open

    # Multiple fingers at once
    robot.set_joints({
        Joint.INDEX_LEFT: 50.0,
        Joint.MIDDLE_LEFT: 50.0,
        Joint.RING_LEFT: 50.0,
        Joint.PINKY_LEFT: 50.0,
    })
```

## Custom Gestures

### Pointing Gesture

```python
from pib3 import Robot, Joint

with Robot(host="172.26.34.149") as robot:
    robot.set_joints({
        Joint.THUMB_LEFT_OPPOSITION: 50.0,
        Joint.THUMB_LEFT_STRETCH: 50.0,
        Joint.INDEX_LEFT: 0.0,       # Open (pointing)
        Joint.MIDDLE_LEFT: 100.0,    # Closed
        Joint.RING_LEFT: 100.0,      # Closed
        Joint.PINKY_LEFT: 100.0,     # Closed
    })
```

### Peace Sign

```python
from pib3 import Robot, Joint

with Robot(host="172.26.34.149") as robot:
    robot.set_joints({
        Joint.THUMB_LEFT_OPPOSITION: 100.0,
        Joint.THUMB_LEFT_STRETCH: 100.0,
        Joint.INDEX_LEFT: 0.0,       # Open
        Joint.MIDDLE_LEFT: 0.0,      # Open
        Joint.RING_LEFT: 100.0,      # Closed
        Joint.PINKY_LEFT: 100.0,     # Closed
    })
```

### Thumbs Up

```python
from pib3 import Robot, Joint

with Robot(host="172.26.34.149") as robot:
    robot.set_joints({
        Joint.THUMB_LEFT_OPPOSITION: 0.0,
        Joint.THUMB_LEFT_STRETCH: 0.0,     # Open (up)
        Joint.INDEX_LEFT: 100.0,           # Closed
        Joint.MIDDLE_LEFT: 100.0,          # Closed
        Joint.RING_LEFT: 100.0,            # Closed
        Joint.PINKY_LEFT: 100.0,           # Closed
    })
```

## Animation Example

```python
from pib3 import Robot, Joint, HandPose, LEFT_HAND_JOINTS

with Robot(host="172.26.34.149") as robot:
    # Open hand
    robot.set_joints(HandPose.LEFT_OPEN, async_=False)

    # Wave by moving wrist
    for _ in range(3):
        robot.set_joint(Joint.WRIST_LEFT, 30.0, async_=False)
        robot.set_joint(Joint.WRIST_LEFT, 70.0, async_=False)

    # Return to neutral
    robot.set_joint(Joint.WRIST_LEFT, 50.0, async_=False)
```

## Mirrored Gestures

Apply the same grip to both hands:

```python
from pib3 import Robot, LEFT_HAND_JOINTS, RIGHT_HAND_JOINTS

with Robot(host="172.26.34.149") as robot:
    grip = 50.0  # 50% closed

    # Apply to both hands
    both_hands = {j: grip for j in LEFT_HAND_JOINTS + RIGHT_HAND_JOINTS}
    robot.set_joints(both_hands)
```
