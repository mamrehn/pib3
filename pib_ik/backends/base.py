"""Abstract base class for robot control backends."""

import math
from abc import ABC, abstractmethod
from pathlib import Path
from typing import Callable, Dict, List, Optional, Sequence, Union

import numpy as np


class RobotBackend(ABC):
    """
    Abstract base class for robot control backends.

    Provides a unified interface for controlling the PIB robot through
    different backends (Webots simulation, real robot, Swift visualization).

    All position values in the public API use radians (URDF convention).
    Each backend handles conversion to its native format internally.

    Implementations:
        - WebotsBackend: Webots simulator
        - RealRobotBackend: Real robot via rosbridge
        - SwiftBackend: Swift browser visualization

    Example:
        >>> with backend as robot:
        ...     # Set a single joint
        ...     robot.set_joint("elbow_left", 0.5)
        ...
        ...     # Get current position
        ...     angle = robot.get_joint("elbow_left")
        ...
        ...     # Save current pose
        ...     saved_pose = robot.get_joints()
        ...
        ...     # Restore pose later
        ...     robot.set_joints(saved_pose)
        ...
        ...     # Set with verification
        ...     success = robot.set_joint("elbow_left", 0.5, verify=True)
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

    @abstractmethod
    def get_joint(self, motor_name: str) -> Optional[float]:
        """
        Get current position of a single joint.

        Args:
            motor_name: Name of motor (e.g., "elbow_left").

        Returns:
            Current position in radians, or None if unavailable.

        Example:
            >>> angle = backend.get_joint("elbow_left")
            >>> print(f"Elbow is at {angle:.2f} radians")
        """
        ...

    @abstractmethod
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

        Example:
            >>> # Get all joints (can be used to save pose)
            >>> saved_pose = backend.get_joints()
            >>>
            >>> # Get specific joints
            >>> arm = backend.get_joints(["elbow_left", "shoulder_vertical_left"])
        """
        ...

    # --- Set Methods ---

    def set_joint(
        self,
        motor_name: str,
        position_radians: float,
        verify: bool = False,
        verify_timeout: float = 1.0,
        verify_tolerance: Optional[float] = None,
    ) -> bool:
        """
        Set position of a single joint.

        Args:
            motor_name: Name of motor (e.g., "elbow_left").
            position_radians: Target position in radians.
            verify: If True, verify the joint reached the target position.
            verify_timeout: Max time to wait for verification (seconds).
            verify_tolerance: Position tolerance for verification (radians).
                            Defaults to DEFAULT_VERIFY_TOLERANCE.

        Returns:
            True if successful (and verified if verify=True).

        Example:
            >>> backend.set_joint("elbow_left", 0.5)
            >>> backend.set_joint("elbow_left", 0.5, verify=True)
        """
        return self.set_joints(
            {motor_name: position_radians},
            verify=verify,
            verify_timeout=verify_timeout,
            verify_tolerance=verify_tolerance,
        )

    def set_joints(
        self,
        positions: Union[Dict[str, float], Sequence[float]],
        verify: bool = False,
        verify_timeout: float = 1.0,
        verify_tolerance: Optional[float] = None,
    ) -> bool:
        """
        Set positions of multiple joints simultaneously.

        Args:
            positions: Either a dict mapping motor names to positions (radians),
                      or a sequence of positions for all motors in MOTOR_NAMES order.
            verify: If True, verify joints reached target positions.
            verify_timeout: Max time to wait for verification (seconds).
            verify_tolerance: Position tolerance for verification (radians).
                            Defaults to DEFAULT_VERIFY_TOLERANCE.

        Returns:
            True if successful (and verified if verify=True).

        Example:
            >>> # Using dict
            >>> backend.set_joints({
            ...     "shoulder_vertical_left": 0.3,
            ...     "elbow_left": 0.8,
            ... })
            >>>
            >>> # Restore a saved pose
            >>> saved_pose = backend.get_joints()
            >>> # ... do something ...
            >>> backend.set_joints(saved_pose)  # Restore
            >>>
            >>> # With verification
            >>> success = backend.set_joints({"elbow_left": 0.5}, verify=True)
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

        # Send command
        success = self._set_joints_impl(positions)

        if not success:
            return False

        # Optionally verify
        if verify:
            return self._verify_positions(
                positions,
                timeout=verify_timeout,
                tolerance=verify_tolerance or self.DEFAULT_VERIFY_TOLERANCE,
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
        timeout: float,
        tolerance: float,
    ) -> bool:
        """
        Verify joints reached target positions within tolerance.

        Args:
            target_positions: Dict of target positions (radians).
            timeout: Max time to wait (seconds).
            tolerance: Acceptable error (radians).

        Returns:
            True if all joints are within tolerance.
        """
        import time

        start_time = time.time()
        check_interval = 0.05  # 50ms between checks

        while (time.time() - start_time) < timeout:
            current = self.get_joints(list(target_positions.keys()))

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
