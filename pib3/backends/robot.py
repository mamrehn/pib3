"""Real robot backend via rosbridge for pib3 package."""

import logging
import math
import threading
import time
from typing import Callable, Dict, List, Optional

import numpy as np

logger = logging.getLogger(__name__)

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

    Position feedback is received via subscription to /joint_trajectory topic.
    The robot publishes current positions when motors move.

    Note:
        Uses joint_limits_robot.yaml for percentage <-> radians conversion.
        Calibrate with: python -m pib3.tools.calibrate_joints

    Example:
        >>> from pib3.backends import RealRobotBackend
        >>> with RealRobotBackend(host="172.26.34.149") as robot:
        ...     robot.run_trajectory("trajectory.json")
        ...
        ...     # Control individual joints
        ...     robot.set_joint("elbow_left", 50.0)  # 50% of calibrated range
        ...     robot.set_joint("elbow_left", 45.0, unit="deg")  # 45 degrees
        ...     robot.set_joint("elbow_left", 0.5, unit="rad")  # 0.5 radians
        ...
        ...     # Read current position
        ...     angle = robot.get_joint("elbow_left")  # Returns percentage
        ...     angle_deg = robot.get_joint("elbow_left", unit="deg")
        ...
        ...     # Save and restore pose
        ...     saved_pose = robot.get_joints()
        ...     robot.set_joints(saved_pose)
    """

    # Use robot-specific joint limits (requires calibration for percentage mode)
    JOINT_LIMITS_FILE = "joint_limits_robot.yaml"

    # Default timeout for waiting for joint data from ROS (seconds)
    DEFAULT_GET_JOINTS_TIMEOUT = 5.0

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
        self._position_subscriber = None
        # Joint positions received from robot via /joint_trajectory topic
        self._joint_positions: Dict[str, float] = {}
        self._joint_positions_lock = threading.Lock()

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

        try:
            self._client.run(timeout=self.timeout)
        except Exception as e:
            # Handle Twisted reactor issues (ReactorNotRestartable)
            # This commonly happens in Jupyter notebooks
            if "ReactorNotRestartable" in str(type(e).__name__) or "ReactorNotRestartable" in str(e):
                raise ConnectionError(
                    "Cannot reconnect: Twisted reactor cannot be restarted. "
                    "In Jupyter notebooks, you must restart the kernel to reconnect. "
                    "Alternatively, keep a single Robot connection open for the session."
                ) from e
            raise

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

        # Subscribe to /joint_trajectory for position feedback
        # Robot publishes current positions when motors move
        self._position_subscriber = roslibpy.Topic(
            self._client,
            '/joint_trajectory',
            'trajectory_msgs/msg/JointTrajectory'
        )
        self._position_subscriber.subscribe(self._on_joint_trajectory)

    def _on_joint_trajectory(self, message: dict) -> None:
        """Callback for position updates from /joint_trajectory topic.

        The robot publishes current positions when motors move.
        Message format: trajectory_msgs/msg/JointTrajectory
        """
        joint_names = message.get('joint_names', [])
        points = message.get('points', [])

        if joint_names and points:
            try:
                positions = points[0].get('positions', [])
                with self._joint_positions_lock:
                    for i, name in enumerate(joint_names):
                        if i < len(positions):
                            # Convert centidegrees to radians
                            centidegrees = float(positions[i])
                            self._joint_positions[name] = self._centidegrees_to_radians(centidegrees)
            except (ValueError, IndexError, TypeError):
                pass

    def disconnect(self) -> None:
        """Close connection to robot."""
        if self._position_subscriber is not None:
            try:
                self._position_subscriber.unsubscribe()
            except Exception:
                pass
            self._position_subscriber = None

        if self._client is not None:
            try:
                self._client.terminate()
            except Exception:
                pass
            self._client = None
            self._service = None

        with self._joint_positions_lock:
            self._joint_positions.clear()

    @property
    def is_connected(self) -> bool:
        """Check if connected to robot."""
        return self._client is not None and self._client.is_connected

    def _get_joint_radians(
        self,
        motor_name: str,
        timeout: Optional[float] = None,
    ) -> Optional[float]:
        """
        Get current position of a single joint in radians.

        Position is received via subscription to /joint_trajectory topic.

        Args:
            motor_name: Name of motor (e.g., "elbow_left").
            timeout: Max time to wait for position data (seconds).
                    If None, uses DEFAULT_GET_JOINTS_TIMEOUT (5.0s).

        Returns:
            Current position in radians, or None if unavailable.
        """
        if timeout is None:
            timeout = self.DEFAULT_GET_JOINTS_TIMEOUT

        start = time.time()
        while (time.time() - start) < timeout:
            with self._joint_positions_lock:
                if motor_name in self._joint_positions:
                    return self._joint_positions[motor_name]
            time.sleep(0.05)  # Poll every 50ms

        with self._joint_positions_lock:
            return self._joint_positions.get(motor_name)

    def _get_joints_radians(
        self,
        motor_names: Optional[List[str]] = None,
        timeout: Optional[float] = None,
    ) -> Dict[str, float]:
        """
        Get current positions of multiple joints in radians.

        Positions are received via subscription to /joint_trajectory topic.

        Args:
            motor_names: List of motor names to query. If None, returns all
                        available positions.
            timeout: Max time to wait for position data (seconds).
                    If None, uses DEFAULT_GET_JOINTS_TIMEOUT (5.0s).

        Returns:
            Dict mapping motor names to positions in radians.
            May contain fewer joints than requested if timeout expires.
        """
        if timeout is None:
            timeout = self.DEFAULT_GET_JOINTS_TIMEOUT

        # Determine which joints we're waiting for
        expected_joints = set(motor_names) if motor_names else None

        # Wait for requested joints to have data
        start = time.time()
        while (time.time() - start) < timeout:
            with self._joint_positions_lock:
                if expected_joints is None:
                    # Return all available if we have any
                    if self._joint_positions:
                        return dict(self._joint_positions)
                else:
                    available_joints = set(self._joint_positions.keys())
                    if expected_joints <= available_joints:
                        # All expected joints are available
                        return {
                            name: self._joint_positions[name]
                            for name in expected_joints
                        }
            time.sleep(0.05)  # Poll every 50ms

        # Timeout expired - return whatever we have
        with self._joint_positions_lock:
            if motor_names is None:
                return dict(self._joint_positions)
            return {
                name: self._joint_positions[name]
                for name in motor_names
                if name in self._joint_positions
            }

    # Default velocity for motor movements (centidegrees per second)
    DEFAULT_VELOCITY_CENTIDEG = 10000  # 100 degrees/sec

    def _set_joints_impl(
        self,
        positions_radians: Dict[str, float],
        velocity_centideg: Optional[float] = None,
    ) -> bool:
        """
        Set joint positions via apply_joint_trajectory service.

        Sends each joint as a separate service call because the PIB robot's
        ROS implementation processes one joint per call.

        Args:
            positions_radians: Dict mapping motor names to positions in radians.
            velocity_centideg: Velocity in centidegrees/sec. If None, uses default.

        Returns:
            True if all joints were set successfully.
        """
        if not self.is_connected:
            return False

        if velocity_centideg is None:
            velocity_centideg = self.DEFAULT_VELOCITY_CENTIDEG

        import roslibpy

        all_successful = True

        # Send each joint as a separate service call
        for joint_name, position in positions_radians.items():
            motor_name = JOINT_TO_ROBOT_MOTOR.get(joint_name, joint_name)
            centidegrees = self._radians_to_centidegrees(position)

            message = {
                'joint_trajectory': {
                    'header': {
                        'stamp': {'sec': 0, 'nanosec': 0},
                        'frame_id': '',
                    },
                    'joint_names': [motor_name],
                    'points': [{
                        'positions': [float(centidegrees)],
                        'velocities': [float(velocity_centideg)],
                        'accelerations': [],
                        'effort': [],
                        'time_from_start': {'sec': 0, 'nanosec': 0},
                    }],
                }
            }

            logger.debug(
                f"Sending to {motor_name}: position={centidegrees} centideg "
                f"({position:.4f} rad), velocity={velocity_centideg}"
            )

            try:
                request = roslibpy.ServiceRequest(message)
                result = self._service.call(request, timeout=self.timeout)
                success = result.get('successful', False)
                logger.debug(f"  Result: {result}")
                if not success:
                    all_successful = False
            except Exception as e:
                logger.debug(f"  Exception: {e}")
                all_successful = False

        return all_successful

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
