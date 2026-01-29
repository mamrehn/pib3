"""DH robot models for PIB arm kinematics.

Expert-calibrated Denavit-Hartenberg parameters for the PIB humanoid robot arms.
Based on pib_DH.py from the PIB SDK.

Note on base transforms:
    The expert's pib_DH.py includes base transforms that position the shoulder
    in the robot's world frame. We omit these here because our IK solver uses
    the URDF model to get the shoulder position directly, then passes
    shoulder-relative targets to the DH model. This avoids potential
    coordinate frame mismatches between DH and URDF representations.
"""

from typing import Dict, Literal, Optional, Tuple

import numpy as np
from roboticstoolbox import DHRobot, RevoluteDH
from spatialmath import SE3


# Camera transform from camera link to robot base (in mm)
# From expert pib_DH.py - useful for vision-based applications
CAMERA_TRANSFORM = SE3.Tx(-91) * SE3.Tz(643.6) * SE3.Rz(-np.pi / 2) * SE3.Rx(np.pi / 2)

# Default tool transform from expert pib_DH.py (in mm)
# This represents the transform from wrist to tool tip with standard gripper
DEFAULT_TOOL_TRANSFORM = SE3.Rz(-np.pi / 2) * SE3.Tz(-42.3) * SE3.Rx(np.pi / 2)


class PibLeft(DHRobot):
    """DH model for PIB's left arm with calibrated parameters."""

    def __init__(self, tool: Optional[SE3] = None):
        pi = np.pi

        # DH parameters (in mm) from expert calibration
        d = [-59, 0, 216.3, 0, -242, -2.7]
        a = [0, 0, -21, 21, 15, -45.2]
        alpha = [pi / 2, pi / 2, pi / 2, pi / 2, pi / 2, -pi / 2]
        offset = [0, 0, pi / 2, pi / 4, 3 * pi / 2, 0]
        qlim_lower = [-pi / 2, -pi / 2, -pi / 2, -pi / 4, -pi / 2, -pi / 4]
        qlim_upper = [pi / 2, pi / 2, pi / 2, pi / 2, pi / 2, pi / 4]

        links = []
        for i in range(6):
            link = RevoluteDH(
                d=d[i],
                a=a[i],
                alpha=alpha[i],
                offset=offset[i],
                qlim=[qlim_lower[i], qlim_upper[i]],
            )
            links.append(link)

        super().__init__(
            links,
            name="pib_left",
            manufacturer="isento GmbH",
            tool=tool,
        )

        # Named configurations
        self.qz = np.zeros(6)
        self.addconfiguration("qz", self.qz)
        self.q_observe = np.radians([-40, 90, 30, 40, 90, 0])
        self.addconfiguration("q_observe", self.q_observe)
        self.q_rest = np.radians([-90, 90, 0, -40, 0, 0])
        self.addconfiguration("q_rest", self.q_rest)


class PibRight(DHRobot):
    """DH model for PIB's right arm with calibrated parameters."""

    def __init__(self, tool: Optional[SE3] = None):
        pi = np.pi

        # DH parameters (in mm) from expert calibration
        d = [-59, 0, 216.3, 0, -242, 2.7]  # Note: d[5] differs from left
        a = [0, 0, -21, 21, 15, -45.2]
        alpha = [pi / 2, pi / 2, pi / 2, pi / 2, pi / 2, -pi / 2]
        offset = [0, 0, pi / 2, pi / 4, pi / 2, 0]  # Note: offset[4] differs from left
        qlim_lower = [-pi / 2, -pi / 2, -pi / 2, -pi / 4, -pi / 2, -pi / 4]
        qlim_upper = [pi / 2, pi / 2, pi / 2, pi / 2, pi / 2, pi / 4]

        links = []
        for i in range(6):
            link = RevoluteDH(
                d=d[i],
                a=a[i],
                alpha=alpha[i],
                offset=offset[i],
                qlim=[qlim_lower[i], qlim_upper[i]],
            )
            links.append(link)

        super().__init__(
            links,
            name="pib_right",
            manufacturer="isento GmbH",
            tool=tool,
        )

        # Named configurations
        self.qz = np.zeros(6)
        self.addconfiguration("qz", self.qz)
        self.q_observe = np.radians([40, -90, -30, 40, -90, 0])
        self.addconfiguration("q_observe", self.q_observe)
        self.q_rest = np.radians([90, -90, 0, -40, 0, 0])
        self.addconfiguration("q_rest", self.q_rest)


# Module-level caches
_DH_ROBOT_CACHE: Dict[Tuple, DHRobot] = {}
_SHOULDER_CACHE: Dict[str, np.ndarray] = {}

# URDF link names for shoulders
SHOULDER_LINKS = {
    "left": "urdf_shoulder_vertical",
    "right": "urdf_shoulder_vertical_2",
}


def get_dh_robot(arm: Literal["left", "right"], tool_offset: SE3) -> DHRobot:
    """
    Get or create a cached DH robot for the given arm and tool offset.

    Args:
        arm: Which arm ("left" or "right").
        tool_offset: SE3 transform from wrist to tool tip (in meters).

    Returns:
        DHRobot configured with the tool offset (converted to mm).
    """
    tool_key = tuple(np.array(tool_offset.t).flatten().round(6))
    cache_key = (arm, tool_key)

    if cache_key not in _DH_ROBOT_CACHE:
        # Convert tool offset from meters to mm
        tool_t_mm = np.array(tool_offset.t).flatten() * 1000
        tool_transform = SE3(tool_t_mm)

        if arm == "left":
            _DH_ROBOT_CACHE[cache_key] = PibLeft(tool=tool_transform)
        else:
            _DH_ROBOT_CACHE[cache_key] = PibRight(tool=tool_transform)

    return _DH_ROBOT_CACHE[cache_key]


def get_shoulder_position(urdf_robot, arm: Literal["left", "right"]) -> np.ndarray:
    """
    Get the shoulder position in world frame from the URDF model.

    This is needed to transform targets from URDF world coordinates to
    DH shoulder-relative coordinates.

    Args:
        urdf_robot: URDF robot model (ERobot).
        arm: Which arm ("left" or "right").

    Returns:
        Shoulder position [x, y, z] in meters.
    """
    if arm not in _SHOULDER_CACHE:
        shoulder_link = SHOULDER_LINKS[arm]
        q_zero = np.zeros(urdf_robot.n)
        T_shoulder = urdf_robot.fkine(q_zero, end=shoulder_link)
        if isinstance(T_shoulder, np.ndarray):
            T_shoulder = SE3(T_shoulder)
        _SHOULDER_CACHE[arm] = np.array(T_shoulder.t).flatten()

    return _SHOULDER_CACHE[arm]


def clear_caches():
    """Clear all module-level caches (useful for testing)."""
    _DH_ROBOT_CACHE.clear()
    _SHOULDER_CACHE.clear()
