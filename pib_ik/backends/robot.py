"""Real robot backend via rosbridge for pib_ik package."""

import math
import time
from typing import Callable, Dict, List, Optional

import numpy as np

from .base import RobotBackend
from ..config import RobotConfig


# Mapping from trajectory joint names to real robot motor names (for apply_joint_trajectory)
JOINT_TO_ROBOT_MOTOR = {
    "turn_head_motor": "turn_head_motor",
    "tilt_forward_motor": "tilt_forward_motor",
    "shoulder_vertical_left": "shoulder_vertical_left",
    "shoulder_horizontal_left": "shoulder_horizontal_left",
    "upper_arm_left_rotation": "upper_arm_left_rotation",
    "elbow_left": "elbow_left",
    "lower_arm_left_rotation": "lower_arm_left_rotation",
    "wrist_left": "wrist_left",
    "thumb_left_opposition": "thumb_left_opposition",
    "thumb_left_stretch": "thumb_left_stretch",
    "index_left_stretch": "index_left_stretch",
    "middle_left_stretch": "middle_left_stretch",
    "ring_left_stretch": "ring_left_stretch",
    "pinky_left_stretch": "pinky_left_stretch",
    "shoulder_vertical_right": "shoulder_vertical_right",
    "shoulder_horizontal_right": "shoulder_horizontal_right",
    "upper_arm_right_rotation": "upper_arm_right_rotation",
    "elbow_right": "elbow_right",
    "lower_arm_right_rotation": "lower_arm_right_rotation",
    "wrist_right": "wrist_right",
    "thumb_right_opposition": "thumb_right_opposition",
    "thumb_right_stretch": "thumb_right_stretch",
    "index_right_stretch": "index_right_stretch",
    "middle_right_stretch": "middle_right_stretch",
    "ring_right_stretch": "ring_right_stretch",
    "pinky_right_stretch": "pinky_right_stretch",
}


class RealRobotBackend(RobotBackend):
    """
    Execute trajectories on real PIB robot via rosbridge.

    Connects to the robot's ROS system via websocket and sends
    joint commands using the apply_joint_trajectory service.

    Position values are converted from radians to centidegrees.

    Example:
        >>> from pib_ik.backends import RealRobotBackend
        >>> with RealRobotBackend(host="172.26.34.149") as robot:
        ...     robot.run_trajectory("trajectory.json")
        ...
        ...     # Or control individual joints
        ...     robot.set_joint("elbow_left", 0.5)  # radians
    """

    def __init__(
        self,
        host: str = "172.26.34.149",
        port: int = 9090,
        timeout: float = 5.0,
    ):
        """
        Initialize real robot backend.

        Args:
            host: Robot IP address.
            port: Rosbridge websocket port.
            timeout: Connection timeout in seconds.
        """
        self.host = host
        self.port = port
        self.timeout = timeout
        self._client = None
        self._service = None

    @classmethod
    def from_config(cls, config: RobotConfig) -> "RealRobotBackend":
        """Create backend from RobotConfig."""
        return cls(host=config.host, port=config.port, timeout=config.timeout)

    def _to_backend_format(self, radians: np.ndarray) -> np.ndarray:
        """Convert radians to centidegrees."""
        return np.round(np.degrees(radians) * 100).astype(int)

    def _from_backend_format(self, centidegrees: np.ndarray) -> np.ndarray:
        """Convert centidegrees to radians."""
        return np.radians(np.asarray(centidegrees) / 100.0)

    def _radians_to_centidegrees(self, radians: float) -> int:
        """Convert single value from radians to centidegrees."""
        return round(math.degrees(radians) * 100)

    def connect(self) -> None:
        """Establish connection to robot via rosbridge websocket."""
        try:
            import roslibpy
        except ImportError:
            raise ImportError(
                "roslibpy is required for real robot connection. "
                "Install with: pip install roslibpy"
            )

        self._client = roslibpy.Ros(host=self.host, port=self.port)
        self._client.run()

        # Wait for connection
        start = time.time()
        while not self._client.is_connected and (time.time() - start) < self.timeout:
            time.sleep(0.1)

        if not self._client.is_connected:
            self._client = None
            raise ConnectionError(
                f"Failed to connect to robot at {self.host}:{self.port}. "
                f"Check that rosbridge_server is running."
            )

        # Initialize service client
        self._service = roslibpy.Service(
            self._client,
            '/apply_joint_trajectory',
            'datatypes/ApplyJointTrajectory'
        )

    def disconnect(self) -> None:
        """Close connection to robot."""
        if self._client is not None:
            try:
                self._client.terminate()
            except Exception:
                pass
            self._client = None
            self._service = None

    @property
    def is_connected(self) -> bool:
        """Check if connected to robot."""
        return self._client is not None and self._client.is_connected

    def set_joints(self, positions_radians: Dict[str, float]) -> bool:
        """
        Set joint positions via apply_joint_trajectory service.

        Args:
            positions_radians: Dict mapping motor names to positions in radians.

        Returns:
            True if successful.
        """
        if not self.is_connected:
            return False

        # Build JointTrajectory message
        joint_names = []
        points = []

        for joint_name, position in positions_radians.items():
            motor_name = JOINT_TO_ROBOT_MOTOR.get(joint_name, joint_name)
            centidegrees = self._radians_to_centidegrees(position)
            joint_names.append(motor_name)
            points.append({
                'positions': [float(centidegrees)],
                'velocities': [],
                'accelerations': [],
                'effort': [],
                'time_from_start': {'secs': 0, 'nsecs': 0},
            })

        message = {
            'joint_trajectory': {
                'header': {
                    'seq': 0,
                    'stamp': {'secs': 0, 'nsecs': 0},
                    'frame_id': '',
                },
                'joint_names': joint_names,
                'points': points,
            }
        }

        try:
            import roslibpy
            request = roslibpy.ServiceRequest(message)
            result = self._service.call(request, timeout=self.timeout)
            return result.get('successful', False)
        except Exception:
            return False

    def _execute_waypoints(
        self,
        joint_names: List[str],
        waypoints: np.ndarray,
        rate_hz: float,
        progress_callback: Optional[Callable[[int, int], None]],
    ) -> bool:
        """Execute waypoints on real robot."""
        if not self.is_connected:
            return False

        period = 1.0 / rate_hz
        total = len(waypoints)

        # Map joint names to robot motor names
        motor_names = [JOINT_TO_ROBOT_MOTOR.get(n, n) for n in joint_names]

        for i, point in enumerate(waypoints):
            # Build positions dict (already in centidegrees from _to_backend_format)
            positions = {}
            for j, motor_name in enumerate(motor_names):
                positions[motor_name] = float(point[j])

            # Send via service (one point at a time for real-time control)
            # Build single-point trajectory
            points_msg = []
            for motor_name in motor_names:
                points_msg.append({
                    'positions': [positions.get(motor_name, 0)],
                    'velocities': [],
                    'accelerations': [],
                    'effort': [],
                    'time_from_start': {'secs': 0, 'nsecs': 0},
                })

            message = {
                'joint_trajectory': {
                    'header': {
                        'seq': i,
                        'stamp': {'secs': 0, 'nsecs': 0},
                        'frame_id': '',
                    },
                    'joint_names': motor_names,
                    'points': points_msg,
                }
            }

            try:
                import roslibpy
                request = roslibpy.ServiceRequest(message)
                self._service.call(request, timeout=self.timeout)
            except Exception:
                pass  # Continue even if one point fails

            if progress_callback:
                progress_callback(i + 1, total)

            # Wait for next cycle
            time.sleep(period)

        return True
