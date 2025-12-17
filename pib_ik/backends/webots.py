"""Webots simulator backend for pib_ik package."""

from typing import Callable, Dict, List, Optional

import numpy as np

from .base import RobotBackend


# Mapping from trajectory joint names to Webots motor device names
JOINT_TO_WEBOTS_MOTOR = {
    "turn_head_motor": "head_horizontal",
    "tilt_forward_motor": "head_vertical",
    "shoulder_vertical_left": "shoulder_vertical_left",
    "shoulder_horizontal_left": "shoulder_horizontal_left",
    "upper_arm_left_rotation": "upper_arm_left",
    "elbow_left": "elbow_left",
    "lower_arm_left_rotation": "forearm_left",
    "wrist_left": "wrist_left",
    "thumb_left_opposition": "thumb_left_opposition",
    "thumb_left_stretch": "thumb_left_distal",
    "index_left_stretch": "index_left_distal",
    "middle_left_stretch": "middle_left_distal",
    "ring_left_stretch": "ring_left_distal",
    "pinky_left_stretch": "pinky_left_distal",
    "shoulder_vertical_right": "shoulder_vertical_right",
    "shoulder_horizontal_right": "shoulder_horizontal_right",
    "upper_arm_right_rotation": "upper_arm_right",
    "elbow_right": "elbow_right",
    "lower_arm_right_rotation": "forearm_right",
    "wrist_right": "wrist_right",
    "thumb_right_opposition": "thumb_right_opposition",
    "thumb_right_stretch": "thumb_right_distal",
    "index_right_stretch": "index_right_distal",
    "middle_right_stretch": "middle_right_distal",
    "ring_right_stretch": "ring_right_distal",
    "pinky_right_stretch": "pinky_right_distal",
}


class WebotsBackend(RobotBackend):
    """
    Execute trajectories in Webots simulator.

    Must be instantiated from within a Webots controller script.

    Example:
        # In your Webots controller file:
        from pib_ik.backends import WebotsBackend

        with WebotsBackend() as backend:
            backend.run_trajectory("trajectory.json")
    """

    WEBOTS_OFFSET = 0.0  # radians offset for Webots motors

    def __init__(self, step_ms: int = 50):
        """
        Initialize Webots backend.

        Args:
            step_ms: Time step per waypoint in milliseconds.
        """
        self.step_ms = step_ms
        self._robot = None
        self._timestep = None
        self._motors: Dict[str, any] = {}

    def _to_backend_format(self, radians: np.ndarray) -> np.ndarray:
        """Convert radians to Webots format (add offset)."""
        return radians + self.WEBOTS_OFFSET

    def _from_backend_format(self, values: np.ndarray) -> np.ndarray:
        """Convert Webots format to radians (subtract offset)."""
        return values - self.WEBOTS_OFFSET

    def connect(self) -> None:
        """Initialize Webots robot and motors."""
        try:
            from controller import Robot
        except ImportError:
            raise ImportError(
                "Webots controller module not found. "
                "This backend must be used from within a Webots controller script."
            )

        self._robot = Robot()
        self._timestep = int(self._robot.getBasicTimeStep())

        # Initialize all motors
        if len (self._motors) == 0:
            self._motors = {}
            for joint_name, motor_name in JOINT_TO_WEBOTS_MOTOR.items():
                motor = self._robot.getDevice(motor_name)
                if motor is not None:
                    self._motors[joint_name] = motor

                    # Enable once, forever. Readout will be available at each timestep.
                    sensor = motor.getPositionSensor()
                    if sensor is not None:
                        sensor.enable(self._timestep)

    def disconnect(self) -> None:
        """Cleanup (no-op for Webots, robot lifecycle managed by simulator)."""
        # self._motors = {}
        pass

    @property
    def is_connected(self) -> bool:
        """Check if robot is initialized."""
        return self._robot is not None

    def _get_joint_radians(self, motor_name: str) -> Optional[float]:
        """
        Get current position of a single joint in radians.

        Args:
            motor_name: Name of motor (e.g., "elbow_left").

        Returns:
            Current position in radians, or None if unavailable.
        """
        if not self.is_connected:
            return None

        if motor_name in self._motors:
            motor = self._motors[motor_name]
            # Get position sensor if available
            sensor = motor.getPositionSensor()
            if sensor is not None:
                webots_pos = sensor.getValue()
                return webots_pos - self.WEBOTS_OFFSET

        return None

    def _get_joints_radians(
        self,
        motor_names: Optional[List[str]] = None,
    ) -> Dict[str, float]:
        """
        Get current positions of multiple joints in radians.

        Args:
            motor_names: List of motor names to query. If None, returns all
                        available joints.

        Returns:
            Dict mapping motor names to positions in radians.
        """
        if not self.is_connected:
            return {}

        result = {}
        names_to_query = motor_names if motor_names is not None else list(self._motors.keys())

        for name in names_to_query:
            pos = self._get_joint_radians(name)
            if pos is not None:
                result[name] = pos

        return result

    def _set_joints_impl(self, positions_radians: Dict[str, float]) -> bool:
        """Set joint positions (converts to Webots format internally)."""
        if not self.is_connected:
            return False

        for joint_name, position in positions_radians.items():
            if joint_name in self._motors:
                webots_pos = position + self.WEBOTS_OFFSET
                self._motors[joint_name].setPosition(webots_pos)

        self._robot.step(self._timestep)
        return True

    def _execute_waypoints(
        self,
        joint_names: List[str],
        waypoints: np.ndarray,
        rate_hz: float,
        progress_callback: Optional[Callable[[int, int], None]],
    ) -> bool:
        """Execute waypoints in Webots."""
        if not self.is_connected:
            return False

        # Build index mapping
        joint_indices = {}
        for i, name in enumerate(joint_names):
            if name in self._motors:
                joint_indices[name] = i

        step_ms = max(int(1000 / rate_hz), self._timestep)
        total = len(waypoints)

        for i, point in enumerate(waypoints):
            # Set all motor positions
            for name, idx in joint_indices.items():
                self._motors[name].setPosition(point[idx])

            # Step simulation
            self._robot.step(step_ms)

            if progress_callback:
                progress_callback(i + 1, total)

        return True
