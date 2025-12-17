"""Hand pose presets and limits for PIB robot.

These values are calibrated for specific robot hardware.
"""

from pathlib import Path
from typing import Dict, Optional

# Left Hand Limits (radians)
LEFT_HAND_OPEN: Dict[str, float] = {
    "thumb_left_opposition": 0.0,
    "thumb_left_stretch": 0.0,
    "index_left_stretch": 0.0,
    "middle_left_stretch": 0.0,
    "ring_left_stretch": 0.0,
    "pinky_left_stretch": 0.0,
}

LEFT_HAND_CLOSED: Dict[str, float] = {
    "thumb_left_opposition": 0.0276,
    "thumb_left_stretch": 0.0555,
    "index_left_stretch": 0.0538,
    "middle_left_stretch": 0.0456,
    "ring_left_stretch": 0.0442,
    "pinky_left_stretch": 0.0267,
}

# Right Hand Limits (radians) - TODO: calibrate
RIGHT_HAND_OPEN: Dict[str, float] = {
    "thumb_right_opposition": 0.0,
    "thumb_right_stretch": 0.0,
    "index_right_stretch": 0.0,
    "middle_right_stretch": 0.0,
    "ring_right_stretch": 0.0,
    "pinky_right_stretch": 0.0,
}

RIGHT_HAND_CLOSED: Optional[Dict[str, float]] = None  # Not yet calibrated

# Joint names for convenience
LEFT_HAND_JOINTS = list(LEFT_HAND_OPEN.keys())
RIGHT_HAND_JOINTS = list(RIGHT_HAND_OPEN.keys())


def interpolate_hand_pose(
    open_pose: Dict[str, float],
    closed_pose: Dict[str, float],
    t: float,
) -> Dict[str, float]:
    """
    Interpolate between open and closed hand poses.

    Args:
        open_pose: Joint positions for open hand.
        closed_pose: Joint positions for closed hand.
        t: Interpolation factor (0.0 = open, 1.0 = closed).

    Returns:
        Interpolated joint positions.
    """
    t = max(0.0, min(1.0, t))  # Clamp to [0, 1]
    return {
        joint: open_pose[joint] + t * (closed_pose[joint] - open_pose[joint])
        for joint in open_pose
    }


def left_hand_pose(grip: float = 0.0) -> Dict[str, float]:
    """
    Get left hand pose for a given grip level.

    Args:
        grip: Grip level from 0.0 (fully open) to 1.0 (fully closed).

    Returns:
        Joint positions dict.

    Example:
        >>> robot.set_joints(left_hand_pose(0.0))   # Open hand
        >>> robot.set_joints(left_hand_pose(1.0))   # Close hand
        >>> robot.set_joints(left_hand_pose(0.5))   # Half grip
    """
    return interpolate_hand_pose(LEFT_HAND_OPEN, LEFT_HAND_CLOSED, grip)


def right_hand_pose(grip: float = 0.0) -> Optional[Dict[str, float]]:
    """
    Get right hand pose for a given grip level.

    Args:
        grip: Grip level from 0.0 (fully open) to 1.0 (fully closed).

    Returns:
        Joint positions dict, or None if not calibrated.
    """
    if RIGHT_HAND_CLOSED is None:
        return None
    return interpolate_hand_pose(RIGHT_HAND_OPEN, RIGHT_HAND_CLOSED, grip)
