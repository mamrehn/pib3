# Hand Poses

Control individual finger joints for hand gestures and poses.

## Overview

The PIB robot has 12 finger joints (6 per hand), allowing for various hand poses and gestures.

## Available Finger Joints

### Left Hand

| Joint Name | Description | Range |
|------------|-------------|-------|
| `thumb_left_opposition` | Thumb rotation (opposition) | 0-100% |
| `thumb_left_stretch` | Thumb bend | 0-100% |
| `index_left_stretch` | Index finger bend | 0-100% |
| `middle_left_stretch` | Middle finger bend | 0-100% |
| `ring_left_stretch` | Ring finger bend | 0-100% |
| `pinky_left_stretch` | Pinky finger bend | 0-100% |

### Right Hand

| Joint Name | Description | Range |
|------------|-------------|-------|
| `thumb_right_opposition` | Thumb rotation (opposition) | 0-100% |
| `thumb_right_stretch` | Thumb bend | 0-100% |
| `index_right_stretch` | Index finger bend | 0-100% |
| `middle_right_stretch` | Middle finger bend | 0-100% |
| `ring_right_stretch` | Ring finger bend | 0-100% |
| `pinky_right_stretch` | Pinky finger bend | 0-100% |

## Basic Control

```python
from pib3 import Robot, Swift, Joint

# Works with any backend
with Swift() as viz:
    # Individual finger control
    viz.set_joint(Joint.INDEX_LEFT_STRETCH, 100.0)  # Fully extended
    viz.set_joint(Joint.INDEX_LEFT_STRETCH, 0.0)    # Fully bent

    # Multiple fingers at once
    viz.set_joints({
        Joint.INDEX_LEFT_STRETCH: 50.0,
        Joint.MIDDLE_LEFT_STRETCH: 50.0,
        Joint.RING_LEFT_STRETCH: 50.0,
        Joint.PINKY_LEFT_STRETCH: 50.0,
    })
```

## Preset Poses

### Open Hand (Flat)

```python
from pib3 import Joint

def open_hand_left(backend):
    """Fully open left hand."""
    backend.set_joints({
        Joint.THUMB_LEFT_OPPOSITION: 0.0,
        Joint.THUMB_LEFT_STRETCH: 100.0,
        Joint.INDEX_LEFT_STRETCH: 100.0,
        Joint.MIDDLE_LEFT_STRETCH: 100.0,
        Joint.RING_LEFT_STRETCH: 100.0,
        Joint.PINKY_LEFT_STRETCH: 100.0,
    })
```

### Closed Fist

```python
def closed_fist_left(backend):
    """Close left hand into a fist."""
    backend.set_joints({
        Joint.THUMB_LEFT_OPPOSITION: 100.0,
        Joint.THUMB_LEFT_STRETCH: 0.0,
        Joint.INDEX_LEFT_STRETCH: 0.0,
        Joint.MIDDLE_LEFT_STRETCH: 0.0,
        Joint.RING_LEFT_STRETCH: 0.0,
        Joint.PINKY_LEFT_STRETCH: 0.0,
    })
```

### Pointing Gesture

```python
def point_left(backend):
    """Point with left index finger."""
    backend.set_joints({
        Joint.THUMB_LEFT_OPPOSITION: 50.0,
        Joint.THUMB_LEFT_STRETCH: 50.0,
        Joint.INDEX_LEFT_STRETCH: 100.0,   # Extended
        Joint.MIDDLE_LEFT_STRETCH: 0.0,    # Bent
        Joint.RING_LEFT_STRETCH: 0.0,      # Bent
        Joint.PINKY_LEFT_STRETCH: 0.0,     # Bent
    })
```

### Thumbs Up

```python
def thumbs_up_left(backend):
    """Thumbs up gesture."""
    backend.set_joints({
        Joint.THUMB_LEFT_OPPOSITION: 0.0,
        Joint.THUMB_LEFT_STRETCH: 100.0,   # Extended
        Joint.INDEX_LEFT_STRETCH: 0.0,     # Bent
        Joint.MIDDLE_LEFT_STRETCH: 0.0,    # Bent
        Joint.RING_LEFT_STRETCH: 0.0,      # Bent
        Joint.PINKY_LEFT_STRETCH: 0.0,     # Bent
    })
```

### Peace Sign

```python
def peace_sign_left(backend):
    """Peace/victory sign."""
    backend.set_joints({
        Joint.THUMB_LEFT_OPPOSITION: 100.0,
        Joint.THUMB_LEFT_STRETCH: 0.0,
        Joint.INDEX_LEFT_STRETCH: 100.0,   # Extended
        Joint.MIDDLE_LEFT_STRETCH: 100.0,  # Extended
        Joint.RING_LEFT_STRETCH: 0.0,      # Bent
        Joint.PINKY_LEFT_STRETCH: 0.0,     # Bent
    })
```

### OK Gesture

```python
def ok_gesture_left(backend):
    """OK gesture (thumb and index touching)."""
    backend.set_joints({
        Joint.THUMB_LEFT_OPPOSITION: 100.0,
        Joint.THUMB_LEFT_STRETCH: 50.0,    # Partially bent
        Joint.INDEX_LEFT_STRETCH: 50.0,    # Partially bent
        Joint.MIDDLE_LEFT_STRETCH: 100.0,  # Extended
        Joint.RING_LEFT_STRETCH: 100.0,    # Extended
        Joint.PINKY_LEFT_STRETCH: 100.0,   # Extended
    })
```

### Pinch Grip (for Drawing)

```python
def pinch_grip_left(backend):
    """Pinch grip for holding a pen."""
    backend.set_joints({
        Joint.THUMB_LEFT_OPPOSITION: 80.0,
        Joint.THUMB_LEFT_STRETCH: 60.0,
        Joint.INDEX_LEFT_STRETCH: 60.0,
        Joint.MIDDLE_LEFT_STRETCH: 40.0,
        Joint.RING_LEFT_STRETCH: 30.0,
        Joint.PINKY_LEFT_STRETCH: 30.0,
    })
```

## Animation Example

### Wave Animation

```python
from pib3 import Swift, Joint
import time

def wave_animation(backend):
    """Simple wave animation."""
    # Open hand
    open_hand_left(backend)
    time.sleep(0.5)

    # Wave back and forth
    for _ in range(3):
        backend.set_joint(Joint.WRIST_LEFT, 30.0)
        time.sleep(0.3)
        backend.set_joint(Joint.WRIST_LEFT, 70.0)
        time.sleep(0.3)

    # Return to neutral
    backend.set_joint(Joint.WRIST_LEFT, 50.0)

with Swift() as viz:
    wave_animation(viz)
```

### Finger Counting

```python
from pib3 import Swift
import time

def count_to_five(backend):
    """Count from 1 to 5 using fingers."""
    fingers = [
        "index_left_stretch",
        "middle_left_stretch",
        "ring_left_stretch",
        "pinky_left_stretch",
        "thumb_left_stretch",
    ]

    # Start with closed fist
    closed_fist_left(backend)
    time.sleep(0.5)

    # Extend one finger at a time
    for i, finger in enumerate(fingers):
        backend.set_joint(finger, 100.0)
        print(f"Count: {i + 1}")
        time.sleep(0.5)

with Swift() as viz:
    count_to_five(viz)
```

## Hand Pose Class

For more complex applications, create a reusable pose class:

```python
from dataclasses import dataclass
from typing import Dict

@dataclass
class HandPose:
    """Represents a hand pose configuration."""
    thumb_opposition: float = 50.0
    thumb_stretch: float = 50.0
    index_stretch: float = 50.0
    middle_stretch: float = 50.0
    ring_stretch: float = 50.0
    pinky_stretch: float = 50.0

    def to_left_joints(self) -> Dict[str, float]:
        """Convert to left hand joint dictionary."""
        return {
            "thumb_left_opposition": self.thumb_opposition,
            "thumb_left_stretch": self.thumb_stretch,
            "index_left_stretch": self.index_stretch,
            "middle_left_stretch": self.middle_stretch,
            "ring_left_stretch": self.ring_stretch,
            "pinky_left_stretch": self.pinky_stretch,
        }

    def to_right_joints(self) -> Dict[str, float]:
        """Convert to right hand joint dictionary."""
        return {
            "thumb_right_opposition": self.thumb_opposition,
            "thumb_right_stretch": self.thumb_stretch,
            "index_right_stretch": self.index_stretch,
            "middle_right_stretch": self.middle_stretch,
            "ring_right_stretch": self.ring_stretch,
            "pinky_right_stretch": self.pinky_stretch,
        }

# Predefined poses
POSES = {
    "open": HandPose(0, 100, 100, 100, 100, 100),
    "closed": HandPose(100, 0, 0, 0, 0, 0),
    "point": HandPose(50, 50, 100, 0, 0, 0),
    "peace": HandPose(100, 0, 100, 100, 0, 0),
    "pinch": HandPose(80, 60, 60, 40, 30, 30),
}

# Usage
with Swift() as viz:
    viz.set_joints(POSES["peace"].to_left_joints())
```

## Tips

### Smooth Transitions

For smooth pose transitions, interpolate between poses:

```python
import numpy as np
import time

def interpolate_poses(backend, start_pose: dict, end_pose: dict, duration: float = 1.0, steps: int = 20):
    """Smoothly transition between two poses."""
    for i in range(steps + 1):
        t = i / steps
        current = {}
        for joint in start_pose:
            current[joint] = start_pose[joint] + t * (end_pose[joint] - start_pose[joint])
        backend.set_joints(current)
        time.sleep(duration / steps)

with Swift() as viz:
    interpolate_poses(viz, POSES["open"].to_left_joints(), POSES["closed"].to_left_joints())
```

### Mirrored Gestures

Apply the same pose to both hands:

```python
def mirror_pose(backend, pose: HandPose):
    """Apply pose to both hands."""
    backend.set_joints(pose.to_left_joints())
    backend.set_joints(pose.to_right_joints())

with Swift() as viz:
    mirror_pose(viz, POSES["open"])
```
