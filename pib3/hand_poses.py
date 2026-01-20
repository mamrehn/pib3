"""Hand pose presets for PIB robot.

These values are in percent (0% = bent/closed, 100% = stretched/open) for consistency
with the robot API. Use with robot.set_joints(pose).

Example:
    >>> robot.set_joints(LEFT_HAND_OPEN)           # Open left hand
    >>> robot.set_joints(LEFT_HAND_CLOSED)         # Close left hand
    >>> robot.set_joints({j: 50.0 for j in LEFT_HAND_JOINTS})  # Half grip
"""

from typing import Dict, List

# Left Hand (percent: 0% = bent/closed, 100% = stretched/open)
LEFT_HAND_OPEN: Dict[str, float] = {
    "thumb_left_opposition": 100.0,
    "thumb_left_stretch": 100.0,
    "index_left_stretch": 100.0,
    "middle_left_stretch": 100.0,
    "ring_left_stretch": 100.0,
    "pinky_left_stretch": 100.0,
}

LEFT_HAND_CLOSED: Dict[str, float] = {
    "thumb_left_opposition": 0.0,
    "thumb_left_stretch": 0.0,
    "index_left_stretch": 0.0,
    "middle_left_stretch": 0.0,
    "ring_left_stretch": 0.0,
    "pinky_left_stretch": 0.0,
}

# Right Hand (percent: 0% = bent/closed, 100% = stretched/open)
RIGHT_HAND_OPEN: Dict[str, float] = {
    "thumb_right_opposition": 100.0,
    "thumb_right_stretch": 100.0,
    "index_right_stretch": 100.0,
    "middle_right_stretch": 100.0,
    "ring_right_stretch": 100.0,
    "pinky_right_stretch": 100.0,
}

RIGHT_HAND_CLOSED: Dict[str, float] = {
    "thumb_right_opposition": 0.0,
    "thumb_right_stretch": 0.0,
    "index_right_stretch": 0.0,
    "middle_right_stretch": 0.0,
    "ring_right_stretch": 0.0,
    "pinky_right_stretch": 0.0,
}

# Joint names for convenience
LEFT_HAND_JOINTS: List[str] = list(LEFT_HAND_OPEN.keys())
RIGHT_HAND_JOINTS: List[str] = list(RIGHT_HAND_OPEN.keys())
