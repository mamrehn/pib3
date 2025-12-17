"""Real robot backend via rosbridge for pib_ik package."""

import math
import threading
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
        ...     # Control individual joints
        ...     robot.set_joint("elbow_left", 0.5)  # radians
        ...
        ...     # Read joint positions
        ...     angle = robot.get_joint("elbow_left")
        ...
        ...     # Save and restore pose
        ...     saved_pose = robot.get_joints()
        ...     robot.set_joints(saved_pose)
        ...
        ...     # Set with verification
        ...     success = robot.set_joint("elbow_left", 0.5, verify=True)
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
        self._joint_states_subscriber = None
        self._joint_states: Dict[str, float] = {}
        self._joint_states_lock = threading.Lock()

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

    def _centidegrees_to_radians(self, centidegrees: float) -> float:
        """Convert single value from centidegrees to radians."""
        return math.radians(centidegrees / 100.0)

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

        # Subscribe to motor_current for reading positions
        # PIB robot publishes motor positions as DiagnosticStatus messages
        self._joint_states_subscriber = roslibpy.Topic(
            self._client,
            '/motor_current',
            'diagnostic_msgs/msg/DiagnosticStatus'
        )
        self._joint_states_subscriber.subscribe(self._on_motor_current)

    def _on_motor_current(self, message: dict) -> None:
        """Callback for motor current/position updates from ROS.

        PIB robot publishes motor positions via /motor_current topic as
        DiagnosticStatus messages with position in centidegrees.
        """
        name = message.get('name', '')
        values = message.get('values', [])

        if name and values:
            try:
                # Position is stored as string in the first value
                centidegrees = float(values[0].get('value', 0))
                with self._joint_states_lock:
                    self._joint_states[name] = self._centidegrees_to_radians(centidegrees)
            except (ValueError, IndexError, TypeError):
                pass

    def disconnect(self) -> None:
        """Close connection to robot."""
        if self._joint_states_subscriber is not None:
            try:
                self._joint_states_subscriber.unsubscribe()
            except Exception:
                pass
            self._joint_states_subscriber = None

        if self._client is not None:
            try:
                self._client.terminate()
            except Exception:
                pass
            self._client = None
            self._service = None

        with self._joint_states_lock:
            self._joint_states.clear()

    @property
    def is_connected(self) -> bool:
        """Check if connected to robot."""
        return self._client is not None and self._client.is_connected

    def get_joint(self, motor_name: str) -> Optional[float]:
        """
        Get current position of a single joint.

        Args:
            motor_name: Name of motor (e.g., "elbow_left").

        Returns:
            Current position in radians, or None if unavailable.
        """
        with self._joint_states_lock:
            return self._joint_states.get(motor_name)

    def get_joints(
        self,
        motor_names: Optional[List[str]] = None,
    ) -> Dict[str, float]:
        """
        Get current positions of multiple joints.

        Args:
            motor_names: List of motor names to query. If None, returns all
                        available joints.

        Returns:
            Dict mapping motor names to positions in radians.
        """
        with self._joint_states_lock:
            if motor_names is None:
                return dict(self._joint_states)
            return {
                name: self._joint_states[name]
                for name in motor_names
                if name in self._joint_states
            }

    def _set_joints_impl(self, positions_radians: Dict[str, float]) -> bool:
        """
        Set joint positions via apply_joint_trajectory service.

        Args:
            positions_radians: Dict mapping motor names to positions in radians.

        Returns:
            True if successful.
        """
        if not self.is_connected:
            return False

        # Build JointTrajectory message (ROS2 format)
        joint_names = []
        positions_centideg = []

        for joint_name, position in positions_radians.items():
            motor_name = JOINT_TO_ROBOT_MOTOR.get(joint_name, joint_name)
            centidegrees = self._radians_to_centidegrees(position)
            joint_names.append(motor_name)
            positions_centideg.append(float(centidegrees))

        message = {
            'joint_trajectory': {
                'header': {
                    'stamp': {'sec': 0, 'nanosec': 0},
                    'frame_id': '',
                },
                'joint_names': joint_names,
                'points': [{
                    'positions': positions_centideg,
                    'velocities': [],
                    'accelerations': [],
                    'effort': [],
                    'time_from_start': {'sec': 0, 'nanosec': 0},
                }],
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
            # Build positions list (already in centidegrees from _to_backend_format)
            positions_centideg = [float(point[j]) for j in range(len(motor_names))]

            # Send via service (ROS2 format)
            message = {
                'joint_trajectory': {
                    'header': {
                        'stamp': {'sec': 0, 'nanosec': 0},
                        'frame_id': '',
                    },
                    'joint_names': motor_names,
                    'points': [{
                        'positions': positions_centideg,
                        'velocities': [],
                        'accelerations': [],
                        'effort': [],
                        'time_from_start': {'sec': 0, 'nanosec': 0},
                    }],
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
