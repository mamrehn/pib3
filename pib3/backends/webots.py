"""Webots simulator backend for pib3 package."""

import logging
import time
from typing import Callable, Dict, List, Optional

import numpy as np

logger = logging.getLogger(__name__)

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

# Finger joints that have both proximal and distal motors in Webots.
# The real robot has a single motor per finger that controls both joints coupled.
# To match this behavior in Webots, we set both proximal and distal to the same position.
# Webots finger joints: 0° = open, 90° = closed (range 0 to π/2 radians)
FINGER_PROXIMAL_MOTORS = {
    "thumb_left_stretch": "thumb_left_proximal",
    "index_left_stretch": "index_left_proximal",
    "middle_left_stretch": "middle_left_proximal",
    "ring_left_stretch": "ring_left_proximal",
    "pinky_left_stretch": "pinky_left_proximal",
    "thumb_right_stretch": "thumb_right_proximal",
    "index_right_stretch": "index_right_proximal",
    "middle_right_stretch": "middle_right_proximal",
    "ring_right_stretch": "ring_right_proximal",
    "pinky_right_stretch": "pinky_right_proximal",
}


class WebotsBackend(RobotBackend):
    """
    Execute trajectories in Webots simulator.

    Must be instantiated from within a Webots controller script.

    When reading joint positions, the backend waits for motor readings to
    stabilize (same value twice) to ensure accurate readings when motors
    are in motion. Use the `timeout` parameter in get_joints() to control
    how long to wait (default: 5.0 seconds).

    Example:
        # In your Webots controller file:
        from pib3.backends import WebotsBackend

        with WebotsBackend() as backend:
            backend.run_trajectory("trajectory.json")

            # Read joint positions (waits up to 5s for stabilization)
            joints = backend.get_joints()

            # Read with custom timeout
            joints = backend.get_joints(timeout=2.0)
    """

    WEBOTS_OFFSET = 0.0  # radians offset for Webots motors

    def __init__(
        self,
        host: str = "localhost",
        port: int = 9090,
        step_ms: int = 50,
    ):
        """
        Initialize Webots backend.

        Args:
            host: Webots host (unused).
            port: Webots port (unused).
            step_ms: Time step per waypoint in milliseconds.
        """
        super().__init__(host=host, port=port)
        self.step_ms = step_ms
        self._robot = None
        self._timestep = None
        self._motors: Dict[str, any] = {}
        self._proximal_motors: Dict[str, any] = {}

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

        # Initialize all motors (only once)
        if len(self._motors) == 0:
            self._motors = {}
            
            for joint_name, motor_name in JOINT_TO_WEBOTS_MOTOR.items():
                motor = self._robot.getDevice(motor_name)
                if motor is not None:
                    self._motors[joint_name] = motor

                    # Enable once, forever. Readout will be available at each timestep.
                    sensor = motor.getPositionSensor()
                    if sensor is not None:
                        sensor.enable(self._timestep)
        
        # Initialize proximal finger motors (only once)
        if len(self._proximal_motors) == 0:
            self._proximal_motors = {}
            for joint_name, proximal_motor_name in FINGER_PROXIMAL_MOTORS.items():
                motor = self._robot.getDevice(proximal_motor_name)
                if motor is not None:
                    self._proximal_motors[joint_name] = motor
                    logger.debug(f"Initialized proximal motor: {joint_name} -> {proximal_motor_name}")
                    sensor = motor.getPositionSensor()
                    if sensor is not None:
                        sensor.enable(self._timestep)
                else:
                    logger.warning(f"Proximal motor not found: {proximal_motor_name}")

        # Reset all motors to true zero to clear saved scene offsets.
        # Webots uses absolute motor positions (relative to proto definition),
        # so setPosition(0.0) moves to the true mechanical zero regardless
        # of saved .wbt joint positions.
        self._reset_to_zero()

    def _reset_to_zero(self) -> None:
        """Reset all motors to true mechanical zero and wait for convergence.

        When a Webots world is saved with joints at non-zero positions,
        motors start at those positions. Since Webots setPosition() uses
        absolute positioning (relative to the proto/URDF joint definition),
        commanding 0.0 moves each joint to its true zero.
        """
        # Step once so sensors produce valid readings
        self._robot.step(self._timestep)

        # Log initial positions for diagnostics
        for name, motor in self._motors.items():
            sensor = motor.getPositionSensor()
            if sensor is not None:
                pos = sensor.getValue()
                if abs(pos) > 0.01:
                    logger.warning(
                        f"Joint {name} starts at {pos:.4f} rad "
                        f"({pos * 180 / 3.14159:.1f} deg) — resetting to zero"
                    )

        # Command all motors to absolute zero
        for motor in self._motors.values():
            motor.setPosition(0.0)
        for motor in self._proximal_motors.values():
            motor.setPosition(0.0)

        # Step simulation until all motors converge (or timeout)
        tolerance = 0.01  # radians
        max_steps = int(5000 / self._timestep)  # 5 seconds max

        for _ in range(max_steps):
            self._robot.step(self._timestep)

            all_at_zero = True
            for motor in self._motors.values():
                sensor = motor.getPositionSensor()
                if sensor is not None and abs(sensor.getValue()) > tolerance:
                    all_at_zero = False
                    break

            if all_at_zero:
                logger.info("All motors reached zero position.")
                return

        logger.warning("Timeout waiting for motors to reach zero position.")

    def disconnect(self) -> None:
        """Cleanup (no-op for Webots, robot lifecycle managed by simulator)."""
        # self._motors = {}
        pass

    @property
    def is_connected(self) -> bool:
        """Check if robot is initialized."""
        return self._robot is not None

    # Default timeout for waiting for motor stabilization (seconds)
    DEFAULT_GET_JOINTS_TIMEOUT = 5.0

    def _get_joint_radians(
        self,
        motor_name: str,
        timeout: Optional[float] = None,
    ) -> Optional[float]:
        """
        Get current position of a single joint in radians.

        Args:
            motor_name: Name of motor (e.g., "elbow_left").
            timeout: Max time to wait for motor to stabilize (seconds).
                    If None, uses DEFAULT_GET_JOINTS_TIMEOUT (5.0s).

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
                if timeout is None:
                    timeout = self.DEFAULT_GET_JOINTS_TIMEOUT
                start = time.time()
                webots_pos_old = sensor.getValue()
                while (time.time() - start) < timeout:
                    self._robot.step(self._timestep)  # Advance simulation
                    webots_pos = sensor.getValue()
                    # Check if motor has stabilized (same reading twice)
                    if abs(webots_pos - webots_pos_old) < 0.0001:
                        return webots_pos - self.WEBOTS_OFFSET
                    else:
                        webots_pos_old = webots_pos
            return None

        return None

    def _get_joints_radians(
        self,
        motor_names: Optional[List[str]] = None,
        timeout: Optional[float] = None,
    ) -> Dict[str, float]:
        """
        Get current positions of multiple joints in radians.

        Args:
            motor_names: List of motor names to query. If None, returns all
                        available joints.
            timeout: Max time to wait for each motor to stabilize (seconds).
                    If None, uses DEFAULT_GET_JOINTS_TIMEOUT (5.0s).

        Returns:
            Dict mapping motor names to positions in radians.
        """
        if not self.is_connected:
            return {}

        result = {}
        names_to_query = motor_names if motor_names is not None else list(self._motors.keys())

        for name in names_to_query:
            pos = self._get_joint_radians(name, timeout=timeout)
            if pos is not None:
                result[name] = pos

        return result

    def _set_joints_impl(self, positions_radians: Dict[str, float]) -> bool:
        """Set joint positions (converts to Webots format internally).
        
        For finger joints, this sets both the proximal and distal motors
        to the same position for realistic coupled finger movement.
        """
        if not self.is_connected:
            return False

        for joint_name, position in positions_radians.items():
            if joint_name in self._motors:
                webots_pos = position + self.WEBOTS_OFFSET
                logger.debug(f"Setting {joint_name} to {webots_pos:.4f} rad ({webots_pos * 180 / 3.14159:.1f} deg)")
                self._motors[joint_name].setPosition(webots_pos)
                
                # Also set proximal motor for finger joints (same position)
                if hasattr(self, '_proximal_motors') and joint_name in self._proximal_motors:
                    logger.debug(f"Setting proximal for {joint_name} to {webots_pos:.4f} rad")
                    self._proximal_motors[joint_name].setPosition(webots_pos)

        # Step simulation once to initiate movement
        self._robot.step(self._timestep)
        return True

    def _verify_positions(
        self,
        target_positions: Dict[str, float],
        unit: str,
        timeout: float,
        tolerance: float,
    ) -> bool:
        """
        Verify joints reached target positions by stepping the simulation.

        Overrides base class to continuously step Webots simulation until
        motors reach their targets, providing blocking behavior that matches
        the real robot API.

        Args:
            target_positions: Dict of target positions (in specified unit).
            unit: Unit of the target positions ("percent", "deg", or "rad").
            timeout: Max time to wait (seconds).
            tolerance: Acceptable error (in same unit).

        Returns:
            True if all joints are within tolerance.
        """
        import math

        # Convert targets to radians for comparison
        if unit == "percent":
            targets_rad = {
                name: self._percent_to_radians(name, pos)
                for name, pos in target_positions.items()
            }
            # Convert tolerance from percent to radians (approximate)
            tolerance_rad = tolerance * 0.01 * math.pi  # ~1.8° per 1%
        elif unit == "deg":
            targets_rad = {name: math.radians(pos) for name, pos in target_positions.items()}
            tolerance_rad = math.radians(tolerance)
        else:  # rad
            targets_rad = dict(target_positions)
            tolerance_rad = tolerance

        start_time = time.time()
        max_steps = int(timeout * 1000 / self._timestep)  # Convert timeout to max simulation steps

        for _ in range(max_steps):
            # Check if all joints are within tolerance
            all_within_tolerance = True
            for joint_name, target_rad in targets_rad.items():
                if joint_name not in self._motors:
                    continue

                motor = self._motors[joint_name]
                sensor = motor.getPositionSensor()
                if sensor is None:
                    continue

                current_pos = sensor.getValue() - self.WEBOTS_OFFSET
                error = abs(current_pos - target_rad)

                if error > tolerance_rad:
                    all_within_tolerance = False
                    break

            if all_within_tolerance:
                return True

            # Step simulation to let motors move
            if self._robot.step(self._timestep) == -1:
                # Simulation ended
                return False

            # Also check wall-clock timeout
            if (time.time() - start_time) >= timeout:
                return False

        return False

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
                position = point[idx]
                self._motors[name].setPosition(position)

                # Also set proximal motor for finger joints (coupled motion)
                if name in self._proximal_motors:
                    self._proximal_motors[name].setPosition(position)

            # Step simulation
            self._robot.step(step_ms)

            if progress_callback:
                progress_callback(i + 1, total)

        return True

    # ==================== UNIFIED AUDIO OVERRIDES ====================

    def _is_webots(self) -> bool:
        """
        Webots backend returns True.

        This causes ROBOT and LOCAL_AND_ROBOT to resolve to LOCAL only,
        avoiding duplicate playback in simulation.
        """
        return True
