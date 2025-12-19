"""Abstract base class for robot control backends."""

import logging
import math
from abc import ABC, abstractmethod
from pathlib import Path
from typing import Callable, Dict, List, Literal, Optional, Sequence, Union

import numpy as np
import yaml

logger = logging.getLogger(__name__)

# Type alias for unit parameter
UnitType = Literal["percent", "rad"]

# Cache for loaded joint limits files (filename -> limits dict)
_JOINT_LIMITS_CACHE: Dict[str, Dict[str, Dict[str, float]]] = {}


def load_joint_limits(filename: str) -> Dict[str, Dict[str, float]]:
    """
    Load joint limits from a YAML config file.

    Args:
        filename: Name of the limits file (e.g., "joint_limits_robot.yaml").

    Returns:
        Dict mapping joint names to their min/max limits.
    """
    if filename in _JOINT_LIMITS_CACHE:
        return _JOINT_LIMITS_CACHE[filename]

    config_path = Path(__file__).parent.parent / "resources" / filename
    if not config_path.exists():
        logger.warning(f"Joint limits config not found: {config_path}")
        return {}

    with open(config_path, "r") as f:
        data = yaml.safe_load(f)

    limits = data.get("joints", {})
    _JOINT_LIMITS_CACHE[filename] = limits
    return limits


def clear_joint_limits_cache() -> None:
    """Clear the joint limits cache. Useful after calibration."""
    _JOINT_LIMITS_CACHE.clear()


def get_joint_limits() -> Dict[str, Dict[str, float]]:
    """
    Get default joint limits (for backward compatibility).

    Deprecated: Use load_joint_limits() with specific filename instead.
    """
    return load_joint_limits("joint_limits_webots.yaml")


class RobotBackend(ABC):
    """
    Abstract base class for robot control backends.

    Provides a unified interface for controlling the PIB robot through
    different backends (Webots simulation, real robot, Swift visualization).

    Position values use percentage (0-100%) by default, which maps to the
    joint's configured min/max range. Use unit="rad" for raw radians.

    Implementations:
        - WebotsBackend: Webots simulator
        - RealRobotBackend: Real robot via rosbridge
        - SwiftBackend: Swift browser visualization

    Example:
        >>> with backend as robot:
        ...     # Set a single joint (percentage by default)
        ...     robot.set_joint("elbow_left", 50.0)  # 50% of range
        ...
        ...     # Get current position in percentage
        ...     angle = robot.get_joint("elbow_left")  # Returns 0-100
        ...
        ...     # Use radians if needed
        ...     robot.set_joint("elbow_left", 0.5, unit="rad")
        ...     angle_rad = robot.get_joint("elbow_left", unit="rad")
        ...
        ...     # Save and restore pose (in percentage)
        ...     saved_pose = robot.get_joints()
        ...     robot.set_joints(saved_pose)
        ...
        ...     # Set with verification
        ...     success = robot.set_joint("elbow_left", 50.0, verify=True)
    """

    # Motor names available on PIB robot
    MOTOR_NAMES: List[str] = [
        "turn_head_motor", "tilt_forward_motor",
        "shoulder_vertical_left", "shoulder_horizontal_left",
        "upper_arm_left_rotation", "elbow_left",
        "lower_arm_left_rotation", "wrist_left",
        "thumb_left_opposition", "thumb_left_stretch",
        "index_left_stretch", "middle_left_stretch",
        "ring_left_stretch", "pinky_left_stretch",
        "shoulder_vertical_right", "shoulder_horizontal_right",
        "upper_arm_right_rotation", "elbow_right",
        "lower_arm_right_rotation", "wrist_right",
        "thumb_right_opposition", "thumb_right_stretch",
        "index_right_stretch", "middle_right_stretch",
        "ring_right_stretch", "pinky_right_stretch",
    ]

    # Default tolerance for verification (radians)
    DEFAULT_VERIFY_TOLERANCE = 0.05  # ~2.9 degrees
    # Default tolerance for verification (percentage)
    DEFAULT_VERIFY_TOLERANCE_PERCENT = 2.0  # 2%

    # Joint limits file for this backend (override in subclasses)
    # - "joint_limits_webots.yaml" for simulation (Webots, Swift)
    # - "joint_limits_robot.yaml" for real robot
    JOINT_LIMITS_FILE: str = "joint_limits_webots.yaml"

    def _get_joint_limits(self) -> Dict[str, Dict[str, float]]:
        """Get joint limits for this backend."""
        return load_joint_limits(self.JOINT_LIMITS_FILE)

    # --- Unit Conversion Methods ---

    def _percent_to_radians(self, motor_name: str, percent: float) -> float:
        """
        Convert percentage (0-100) to radians based on joint limits.

        Args:
            motor_name: Name of the motor.
            percent: Position as percentage (0 = min, 100 = max).

        Returns:
            Position in radians.

        Raises:
            ValueError: If joint limits are not calibrated (min/max is None).
        """
        limits = self._get_joint_limits().get(motor_name)
        if limits is None:
            raise ValueError(
                f"No limits configured for joint '{motor_name}' in {self.JOINT_LIMITS_FILE}. "
                f"Run calibration or use unit='rad'."
            )

        min_rad = limits.get("min")
        max_rad = limits.get("max")

        if min_rad is None or max_rad is None:
            raise ValueError(
                f"Joint '{motor_name}' is not calibrated (min={min_rad}, max={max_rad}). "
                f"Run: python -m pib3.tools.calibrate_joints --joints {motor_name}"
            )

        # Warn if out of range
        if percent < 0.0 or percent > 100.0:
            logger.warning(
                f"Joint '{motor_name}' position {percent:.1f}% is outside "
                f"0-100% range (will be clamped by hardware/simulation)"
            )

        # Linear interpolation: 0% -> min, 100% -> max
        radians = min_rad + (percent / 100.0) * (max_rad - min_rad)
        return radians

    def _radians_to_percent(self, motor_name: str, radians: float) -> float:
        """
        Convert radians to percentage (0-100) based on joint limits.

        Args:
            motor_name: Name of the motor.
            radians: Position in radians.

        Returns:
            Position as percentage (0 = min, 100 = max).

        Raises:
            ValueError: If joint limits are not calibrated (min/max is None).
        """
        limits = self._get_joint_limits().get(motor_name)
        if limits is None:
            raise ValueError(
                f"No limits configured for joint '{motor_name}' in {self.JOINT_LIMITS_FILE}. "
                f"Run calibration or use unit='rad'."
            )

        min_rad = limits.get("min")
        max_rad = limits.get("max")

        if min_rad is None or max_rad is None:
            raise ValueError(
                f"Joint '{motor_name}' is not calibrated (min={min_rad}, max={max_rad}). "
                f"Run: python -m pib3.tools.calibrate_joints --joints {motor_name}"
            )

        # Avoid division by zero
        range_rad = max_rad - min_rad
        if abs(range_rad) < 1e-9:
            return 0.0

        # Linear interpolation: min -> 0%, max -> 100%
        percent = ((radians - min_rad) / range_rad) * 100.0

        # Warn if out of range
        if percent < 0.0 or percent > 100.0:
            logger.warning(
                f"Joint '{motor_name}' position {percent:.1f}% is outside "
                f"0-100% range"
            )

        return percent

    @abstractmethod
    def _to_backend_format(self, radians: np.ndarray) -> np.ndarray:
        """Convert canonical radians to backend-specific format."""
        ...

    @abstractmethod
    def _from_backend_format(self, values: np.ndarray) -> np.ndarray:
        """Convert backend-specific format to canonical radians."""
        ...

    @abstractmethod
    def connect(self) -> None:
        """Establish connection to the backend."""
        ...

    @abstractmethod
    def disconnect(self) -> None:
        """Close connection to the backend."""
        ...

    @property
    @abstractmethod
    def is_connected(self) -> bool:
        """Check if connected to the backend."""
        ...

    def __enter__(self) -> "RobotBackend":
        """Context manager entry - connect to backend."""
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit - disconnect from backend."""
        self.disconnect()

    # --- Get Methods ---

    def get_joint(
        self,
        motor_name: str,
        unit: UnitType = "percent",
        timeout: Optional[float] = None,
    ) -> Optional[float]:
        """
        Get current position of a single joint.

        Args:
            motor_name: Name of motor (e.g., "elbow_left").
            unit: Unit for the returned value ("percent" or "rad").
                  Default is "percent" (0-100% of joint range).
            timeout: Max time to wait for joint data (seconds). Behavior varies
                    by backend:
                    - RealRobotBackend: Waits for ROS messages to arrive.
                      Default: 5.0 seconds.
                    - WebotsBackend: Waits for motor reading to stabilize
                      (same value twice). Default: 5.0 seconds.
                    - SwiftBackend: Ignored (synchronous access).

        Returns:
            Current position in specified unit, or None if unavailable.

        Example:
            >>> angle = backend.get_joint("elbow_left")  # Returns percentage
            >>> print(f"Elbow is at {angle:.1f}%")
            >>>
            >>> angle_rad = backend.get_joint("elbow_left", unit="rad")
            >>> print(f"Elbow is at {angle_rad:.2f} radians")
        """
        radians = self._get_joint_radians(motor_name, timeout=timeout)
        if radians is None:
            return None

        if unit == "rad":
            return radians
        else:  # percent
            return self._radians_to_percent(motor_name, radians)

    @abstractmethod
    def _get_joint_radians(
        self,
        motor_name: str,
        timeout: Optional[float] = None,
    ) -> Optional[float]:
        """
        Get current position of a single joint in radians (internal).

        Args:
            motor_name: Name of motor.
            timeout: Max time to wait for joint data (seconds).
                    May be ignored by backends with synchronous access.

        Returns:
            Current position in radians, or None if unavailable.
        """
        ...

    def get_joints(
        self,
        motor_names: Optional[List[str]] = None,
        unit: UnitType = "percent",
        timeout: Optional[float] = None,
    ) -> Dict[str, float]:
        """
        Get current positions of multiple joints.

        Args:
            motor_names: List of motor names to query. If None, returns all
                        available joints.
            unit: Unit for the returned values ("percent" or "rad").
                  Default is "percent" (0-100% of joint range).
            timeout: Max time to wait for joint data (seconds). Behavior varies
                    by backend:
                    - RealRobotBackend: Waits for ROS messages to arrive.
                      Default: 5.0 seconds.
                    - WebotsBackend: Waits for motor readings to stabilize
                      (same value twice). Default: 5.0 seconds.
                    - SwiftBackend: Ignored (synchronous access).

        Returns:
            Dict mapping motor names to positions in specified unit.

        Example:
            >>> # Get all joints in percentage (can be used to save pose)
            >>> saved_pose = backend.get_joints()
            >>>
            >>> # Get specific joints in radians
            >>> arm = backend.get_joints(["elbow_left", "wrist_left"], unit="rad")
            >>>
            >>> # Custom timeout
            >>> joints = robot.get_joints(timeout=2.0)
        """
        radians_dict = self._get_joints_radians(motor_names, timeout=timeout)

        if unit == "rad":
            return radians_dict
        else:  # percent
            return {
                name: self._radians_to_percent(name, rad)
                for name, rad in radians_dict.items()
            }

    @abstractmethod
    def _get_joints_radians(
        self,
        motor_names: Optional[List[str]] = None,
        timeout: Optional[float] = None,
    ) -> Dict[str, float]:
        """
        Get current positions of multiple joints in radians (internal).

        Args:
            motor_names: List of motor names to query. If None, returns all
                        available joints.
            timeout: Max time to wait for joint data if none available (seconds).
                    May be ignored by backends with synchronous access.

        Returns:
            Dict mapping motor names to positions in radians.
        """
        ...

    # --- Set Methods ---

    def set_joint(
        self,
        motor_name: str,
        position: float,
        unit: UnitType = "percent",
        verify: bool = False,
        verify_timeout: float = 1.0,
        verify_tolerance: Optional[float] = None,
    ) -> bool:
        """
        Set position of a single joint.

        Args:
            motor_name: Name of motor (e.g., "elbow_left").
            position: Target position in specified unit.
            unit: Unit for position ("percent" or "rad").
                  Default is "percent" (0 = min, 100 = max).
            verify: If True, verify the joint reached the target position.
            verify_timeout: Max time to wait for verification (seconds).
            verify_tolerance: Position tolerance for verification.
                            In same unit as position. Defaults to 2% or 0.05 rad.

        Returns:
            True if successful (and verified if verify=True).

        Example:
            >>> backend.set_joint("elbow_left", 50.0)  # 50% of range
            >>> backend.set_joint("elbow_left", 0.5, unit="rad")  # radians
            >>> backend.set_joint("elbow_left", 50.0, verify=True)
        """
        return self.set_joints(
            {motor_name: position},
            unit=unit,
            verify=verify,
            verify_timeout=verify_timeout,
            verify_tolerance=verify_tolerance,
        )

    def set_joints(
        self,
        positions: Union[Dict[str, float], Sequence[float]],
        unit: UnitType = "percent",
        verify: bool = False,
        verify_timeout: float = 1.0,
        verify_tolerance: Optional[float] = None,
    ) -> bool:
        """
        Set positions of multiple joints simultaneously.

        Args:
            positions: Either a dict mapping motor names to positions,
                      or a sequence of positions for all motors in MOTOR_NAMES order.
            unit: Unit for positions ("percent" or "rad").
                  Default is "percent" (0 = min, 100 = max).
            verify: If True, verify joints reached target positions.
            verify_timeout: Max time to wait for verification (seconds).
            verify_tolerance: Position tolerance for verification.
                            In same unit as positions. Defaults to 2% or 0.05 rad.

        Returns:
            True if successful (and verified if verify=True).

        Example:
            >>> # Using dict with percentage (default)
            >>> backend.set_joints({
            ...     "shoulder_vertical_left": 30.0,  # 30%
            ...     "elbow_left": 80.0,              # 80%
            ... })
            >>>
            >>> # Using radians
            >>> backend.set_joints({"elbow_left": 0.5}, unit="rad")
            >>>
            >>> # Save and restore pose
            >>> saved_pose = backend.get_joints()  # Returns percentages
            >>> # ... do something ...
            >>> backend.set_joints(saved_pose)  # Restore
            >>>
            >>> # With verification
            >>> success = backend.set_joints({"elbow_left": 50.0}, verify=True)
        """
        # Convert sequence to dict if needed
        if not isinstance(positions, dict):
            positions_list = list(positions)
            if len(positions_list) != len(self.MOTOR_NAMES):
                raise ValueError(
                    f"Expected {len(self.MOTOR_NAMES)} positions, "
                    f"got {len(positions_list)}"
                )
            positions = dict(zip(self.MOTOR_NAMES, positions_list))

        # Convert to radians if needed
        if unit == "percent":
            positions_radians = {
                name: self._percent_to_radians(name, pos)
                for name, pos in positions.items()
            }
        else:  # rad
            positions_radians = dict(positions)

        # Send command
        success = self._set_joints_impl(positions_radians)

        if not success:
            return False

        # Optionally verify
        if verify:
            # Use appropriate default tolerance based on unit
            if verify_tolerance is None:
                if unit == "percent":
                    verify_tolerance = self.DEFAULT_VERIFY_TOLERANCE_PERCENT
                else:
                    verify_tolerance = self.DEFAULT_VERIFY_TOLERANCE

            return self._verify_positions(
                positions,  # Original positions in user's unit
                unit=unit,
                timeout=verify_timeout,
                tolerance=verify_tolerance,
            )

        return True

    @abstractmethod
    def _set_joints_impl(self, positions_radians: Dict[str, float]) -> bool:
        """
        Backend-specific implementation to set joint positions.

        Args:
            positions_radians: Dict mapping motor names to positions in radians.

        Returns:
            True if command was sent successfully.
        """
        ...

    def _verify_positions(
        self,
        target_positions: Dict[str, float],
        unit: UnitType,
        timeout: float,
        tolerance: float,
    ) -> bool:
        """
        Verify joints reached target positions within tolerance.

        Args:
            target_positions: Dict of target positions (in specified unit).
            unit: Unit of the target positions ("percent" or "rad").
            timeout: Max time to wait (seconds).
            tolerance: Acceptable error (in same unit).

        Returns:
            True if all joints are within tolerance.
        """
        import time

        start_time = time.time()
        check_interval = 0.05  # 50ms between checks

        while (time.time() - start_time) < timeout:
            current = self.get_joints(list(target_positions.keys()), unit=unit)

            all_within_tolerance = True
            for name, target in target_positions.items():
                if name not in current:
                    all_within_tolerance = False
                    break
                error = abs(current[name] - target)
                if error > tolerance:
                    all_within_tolerance = False
                    break

            if all_within_tolerance:
                return True

            time.sleep(check_interval)

        return False

    def run_trajectory(
        self,
        trajectory: Union[str, Path, "Trajectory"],
        rate_hz: float = 20.0,
        progress_callback: Optional[Callable[[int, int], None]] = None,
    ) -> bool:
        """
        Execute trajectory on this backend.

        Args:
            trajectory: Trajectory object or path to trajectory JSON file.
            rate_hz: Playback rate in Hz.
            progress_callback: Optional callback(current_point, total_points).

        Returns:
            True if completed successfully.
        """
        from ..trajectory import Trajectory

        if isinstance(trajectory, (str, Path)):
            trajectory = Trajectory.from_json(trajectory)

        # Convert to backend format
        waypoints = self._to_backend_format(trajectory.waypoints)

        return self._execute_waypoints(
            trajectory.joint_names,
            waypoints,
            rate_hz,
            progress_callback,
        )

    @abstractmethod
    def _execute_waypoints(
        self,
        joint_names: List[str],
        waypoints: np.ndarray,
        rate_hz: float,
        progress_callback: Optional[Callable[[int, int], None]],
    ) -> bool:
        """Backend-specific waypoint execution."""
        ...
