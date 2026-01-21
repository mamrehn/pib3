"""Abstract base class for robot control backends."""

import logging
import math
from abc import ABC, abstractmethod
from pathlib import Path
from typing import Callable, Dict, List, Literal, Optional, Sequence, Union

import numpy as np
import yaml

from ..types import Joint, HandPose

logger = logging.getLogger(__name__)

# Type alias for unit parameter
UnitType = Literal["percent", "rad", "deg"]

# Type alias for motor name (accepts both str and Joint enum)
MotorNameType = Union[str, Joint]

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
    different backends (Webots simulation, real robot).

    Position values use percentage (0-100%) by default, which maps to the
    joint's calibrated range:
    - 0% = calibrated minimum angle
    - 100% = calibrated maximum angle

    Use unit="deg" for degrees or unit="rad" for radians.

    Implementations:
        - WebotsBackend: Webots simulator
        - RealRobotBackend: Real robot via rosbridge

    Example:
        >>> from pib3 import Joint
        >>> with backend as robot:
        ...     # Set a single joint (percentage: 0%=min, 100%=max of calibrated range)
        ...     robot.set_joint(Joint.ELBOW_LEFT, 0.0)    # 0% = min angle
        ...     robot.set_joint(Joint.ELBOW_LEFT, 50.0)   # 50% = middle of range
        ...     robot.set_joint(Joint.ELBOW_LEFT, 100.0)  # 100% = max angle
        ...
        ...     # Use degrees directly
        ...     robot.set_joint(Joint.ELBOW_LEFT, -30.0, unit="deg")  # -30 degrees
        ...     angle_deg = robot.get_joint(Joint.ELBOW_LEFT, unit="deg")
        ...
        ...     # Use radians if needed
        ...     robot.set_joint(Joint.ELBOW_LEFT, 0.5, unit="rad")
        ...     angle_rad = robot.get_joint(Joint.ELBOW_LEFT, unit="rad")
        ...
        ...     # Save and restore pose (in percentage)
        ...     saved_pose = robot.get_joints()
        ...     robot.set_joints(saved_pose)
        ...
        ...     # Set and wait for completion
        ...     success = robot.set_joint(Joint.ELBOW_LEFT, 50.0, async_=False)
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
    # Default tolerance for verification (degrees)
    DEFAULT_VERIFY_TOLERANCE_DEG = 3.0  # 3 degrees

    # Joint limits file for this backend (override in subclasses)
    # - "joint_limits_webots.yaml" for simulation (Webots)
    # - "joint_limits_robot.yaml" for real robot
    # - "joint_limits_webots.yaml" for simulation (Webots)
    # - "joint_limits_robot.yaml" for real robot
    JOINT_LIMITS_FILE: str = "joint_limits_webots.yaml"

    def __init__(self, host: str = "localhost", port: int = 9090):
        self.host = host
        self.port = port
        self._is_connected = False

        # Audio device management
        from pib3.backends.audio import (
            NoOpAudioBackend,
            NoOpAudioInputBackend,
            AudioBackend,
            AudioInputBackend,
            AudioDevice,
            AudioDeviceManager,
        )
        self.audio: AudioBackend = NoOpAudioBackend()
        self.audio_input: AudioInputBackend = NoOpAudioInputBackend()
        self._audio_device_manager: Optional[AudioDeviceManager] = None
        self._selected_input_device: Optional[AudioDevice] = None
        self._selected_output_device: Optional[AudioDevice] = None

    def _get_joint_limits(self) -> Dict[str, Dict[str, float]]:
        """Get joint limits for this backend."""
        return load_joint_limits(self.JOINT_LIMITS_FILE)

    # --- Unit Conversion Methods ---

    def _percent_to_radians(self, motor_name: str, percent: float) -> float:
        """
        Convert percentage (0-100) to radians using calibrated joint limits.

        The percentage maps linearly to the joint's calibrated range:
        - 0% = calibrated min angle
        - 100% = calibrated max angle

        Args:
            motor_name: Name of the motor.
            percent: Position as percentage (0 = min, 100 = max).

        Returns:
            Position in radians.

        Raises:
            ValueError: If joint limits are not calibrated.
        """
        limits = self._get_joint_limits().get(motor_name)
        if limits is None:
            raise ValueError(
                f"No limits configured for joint '{motor_name}' in {self.JOINT_LIMITS_FILE}. "
                f"Run calibration or use unit='rad' or unit='deg'."
            )

        min_rad = limits.get("min")
        max_rad = limits.get("max")

        if min_rad is None or max_rad is None:
            raise ValueError(
                f"Joint '{motor_name}' is not calibrated (min={min_rad}, max={max_rad}). "
                f"Run: python -m pib3.tools.calibrate_joints --joints {motor_name}"
            )

        # Warn if out of 0-100% range
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
        Convert radians to percentage (0-100) using calibrated joint limits.

        The percentage maps linearly from the joint's calibrated range:
        - calibrated min = 0%
        - calibrated max = 100%

        Args:
            motor_name: Name of the motor.
            radians: Position in radians.

        Returns:
            Position as percentage (0 = min, 100 = max).

        Raises:
            ValueError: If joint limits are not calibrated.
        """
        limits = self._get_joint_limits().get(motor_name)
        if limits is None:
            raise ValueError(
                f"No limits configured for joint '{motor_name}' in {self.JOINT_LIMITS_FILE}. "
                f"Run calibration or use unit='rad' or unit='deg'."
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

    # --- Audio Device Methods ---

    def list_audio_input_devices(self) -> List["AudioDevice"]:
        """
        List available audio input devices (microphones).

        Returns devices from both local system and robot (if applicable).

        Returns:
            List of AudioDevice objects that can record audio.

        Example:
            >>> for mic in robot.list_audio_input_devices():
            ...     print(f"  {mic.id}: {mic.name}")
        """
        from pib3.backends.audio import AudioDeviceManager
        if self._audio_device_manager is None:
            self._audio_device_manager = AudioDeviceManager(
                include_robot=self._should_include_robot_audio()
            )
        return self._audio_device_manager.list_input_devices()

    def list_audio_output_devices(self) -> List["AudioDevice"]:
        """
        List available audio output devices (speakers).

        Returns devices from both local system and robot (if applicable).

        Returns:
            List of AudioDevice objects that can play audio.

        Example:
            >>> for speaker in robot.list_audio_output_devices():
            ...     print(f"  {speaker.id}: {speaker.name}")
        """
        from pib3.backends.audio import AudioDeviceManager
        if self._audio_device_manager is None:
            self._audio_device_manager = AudioDeviceManager(
                include_robot=self._should_include_robot_audio()
            )
        return self._audio_device_manager.list_output_devices()

    def set_audio_input_device(self, device: Union[str, "AudioDevice", None]) -> None:
        """
        Set the active audio input device (microphone).

        Args:
            device: Device to use. Can be:
                - AudioDevice object
                - Device ID string (e.g., "local_0", "robot_mic")
                - None to reset to default

        Raises:
            ValueError: If device ID is not found.

        Example:
            >>> mics = robot.list_audio_input_devices()
            >>> robot.set_audio_input_device(mics[0])
            >>> # Or by ID
            >>> robot.set_audio_input_device("robot_mic")
        """
        from pib3.backends.audio import AudioDevice, AudioDeviceType

        if device is None:
            self._selected_input_device = None
            self._setup_audio_input_backend()
            return

        if isinstance(device, str):
            # Look up by ID
            found = None
            for d in self.list_audio_input_devices():
                if d.id == device:
                    found = d
                    break
            if found is None:
                raise ValueError(f"Audio input device '{device}' not found")
            device = found

        if not device.is_input:
            raise ValueError(f"Device '{device.name}' is not an input device")

        self._selected_input_device = device
        self._setup_audio_input_backend()

    def set_audio_output_device(self, device: Union[str, "AudioDevice", None]) -> None:
        """
        Set the active audio output device (speaker).

        Args:
            device: Device to use. Can be:
                - AudioDevice object
                - Device ID string (e.g., "local_0", "robot_speaker")
                - None to reset to default

        Raises:
            ValueError: If device ID is not found.

        Example:
            >>> speakers = robot.list_audio_output_devices()
            >>> robot.set_audio_output_device(speakers[0])
            >>> # Or by ID
            >>> robot.set_audio_output_device("robot_speaker")
        """
        from pib3.backends.audio import AudioDevice, AudioDeviceType

        if device is None:
            self._selected_output_device = None
            self._setup_audio_output_backend()
            return

        if isinstance(device, str):
            # Look up by ID
            found = None
            for d in self.list_audio_output_devices():
                if d.id == device:
                    found = d
                    break
            if found is None:
                raise ValueError(f"Audio output device '{device}' not found")
            device = found

        if not device.is_output:
            raise ValueError(f"Device '{device.name}' is not an output device")

        self._selected_output_device = device
        self._setup_audio_output_backend()

    @property
    def audio_input_device(self) -> Optional["AudioDevice"]:
        """Get the currently selected audio input device."""
        return self._selected_input_device

    @property
    def audio_output_device(self) -> Optional["AudioDevice"]:
        """Get the currently selected audio output device."""
        return self._selected_output_device

    def _should_include_robot_audio(self) -> bool:
        """
        Whether to include robot audio devices in listings.

        Override in subclasses. Returns True for RealRobotBackend,
        False for WebotsBackend.
        """
        return False

    def _setup_audio_input_backend(self) -> None:
        """
        Set up audio input backend based on selected device.

        Override in subclasses to provide appropriate backends.
        """
        from pib3.backends.audio import (
            NoOpAudioInputBackend,
            SystemAudioInputBackend,
            AudioDeviceType,
        )

        device = self._selected_input_device

        if device is None:
            # Use smart default
            device = self._get_default_input_device()

        if device is None:
            self.audio_input = NoOpAudioInputBackend()
            return

        if device.device_type == AudioDeviceType.LOCAL:
            try:
                self.audio_input = SystemAudioInputBackend(device=device)
            except ImportError:
                self.audio_input = NoOpAudioInputBackend()
        else:
            # Robot device - subclass should override
            self.audio_input = NoOpAudioInputBackend()

    def _setup_audio_output_backend(self) -> None:
        """
        Set up audio output backend based on selected device.

        Override in subclasses to provide appropriate backends.
        """
        from pib3.backends.audio import (
            NoOpAudioBackend,
            SystemAudioBackend,
            AudioDeviceType,
        )

        device = self._selected_output_device

        if device is None:
            # Use smart default
            device = self._get_default_output_device()

        if device is None:
            self.audio = NoOpAudioBackend()
            return

        if device.device_type == AudioDeviceType.LOCAL:
            try:
                self.audio = SystemAudioBackend(device=device)
            except ImportError:
                self.audio = NoOpAudioBackend()
        else:
            # Robot device - subclass should override
            self.audio = NoOpAudioBackend()

    def _get_default_input_device(self) -> Optional["AudioDevice"]:
        """
        Get the default input device for this backend.

        Override in subclasses to provide smart defaults.
        """
        from pib3.backends.audio import AudioDeviceManager
        if self._audio_device_manager is None:
            self._audio_device_manager = AudioDeviceManager(
                include_robot=self._should_include_robot_audio()
            )
        return self._audio_device_manager.get_default_input_device()

    def _get_default_output_device(self) -> Optional["AudioDevice"]:
        """
        Get the default output device for this backend.

        Override in subclasses to provide smart defaults.
        """
        from pib3.backends.audio import AudioDeviceManager
        if self._audio_device_manager is None:
            self._audio_device_manager = AudioDeviceManager(
                include_robot=self._should_include_robot_audio()
            )
        return self._audio_device_manager.get_default_output_device()

    # --- Get Methods ---

    def get_joint(
        self,
        motor_name: MotorNameType,
        unit: UnitType = "percent",
        timeout: Optional[float] = None,
    ) -> Optional[float]:
        """
        Get current position of a single joint.

        Args:
            motor_name: Motor name as string or Joint enum (e.g., Joint.ELBOW_LEFT).
            unit: Unit for the returned value ("percent", "deg", or "rad").
                  Default is "percent" (0%=min, 100%=max of calibrated range).
            timeout: Max time to wait for joint data (seconds). Behavior varies
                    by backend:
                    - RealRobotBackend: Waits for ROS messages to arrive.
                      Default: 5.0 seconds.
                    - WebotsBackend: Waits for motor reading to stabilize
                      (same value twice). Default: 5.0 seconds.

        Returns:
            Current position in specified unit, or None if unavailable.

        Example:
            >>> from pib3 import Joint
            >>> angle = backend.get_joint(Joint.ELBOW_LEFT)  # Returns percentage
            >>> print(f"Elbow is at {angle:.1f}%")
            >>>
            >>> angle_deg = backend.get_joint(Joint.ELBOW_LEFT, unit="deg")
            >>> print(f"Elbow is at {angle_deg:.1f} degrees")
        """
        # Normalize motor_name to string (Joint enum values are already strings)
        motor_str = str(motor_name.value if isinstance(motor_name, Joint) else motor_name)
        radians = self._get_joint_radians(motor_str, timeout=timeout)
        if radians is None:
            return None

        if unit == "rad":
            return radians
        elif unit == "deg":
            return math.degrees(radians)
        else:  # percent
            return self._radians_to_percent(motor_str, radians)

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
        motor_names: Optional[List[MotorNameType]] = None,
        unit: UnitType = "percent",
        timeout: Optional[float] = None,
    ) -> Dict[str, float]:
        """
        Get current positions of multiple joints.

        Args:
            motor_names: List of motor names (str or Joint enum). If None, returns all.
            unit: Unit for values ("percent", "deg", or "rad"). Default: "percent".
            timeout: Max wait time for joint data (seconds). Backend-specific.

        Returns:
            Dict mapping motor names (str) to positions in specified unit.

        Example:
            >>> from pib3 import Joint
            >>> saved_pose = backend.get_joints()  # All joints
            >>> arm = backend.get_joints([Joint.ELBOW_LEFT, Joint.WRIST_LEFT], unit="rad")
        """
        # Normalize motor names to strings
        if motor_names is not None:
            motor_names_str = [
                str(m.value if isinstance(m, Joint) else m) for m in motor_names
            ]
        else:
            motor_names_str = None

        radians_dict = self._get_joints_radians(motor_names_str, timeout=timeout)

        if unit == "rad":
            return radians_dict
        elif unit == "deg":
            return {
                name: math.degrees(rad)
                for name, rad in radians_dict.items()
            }
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
        motor_name: MotorNameType,
        position: float,
        unit: UnitType = "percent",
        async_: bool = True,
        timeout: float = 1.0,
        tolerance: Optional[float] = None,
    ) -> bool:
        """
        Set position of a single joint.

        Args:
            motor_name: Motor name as string or Joint enum (e.g., Joint.ELBOW_LEFT).
            position: Target position in specified unit.
            unit: Unit for position ("percent", "deg", or "rad"). Default: "percent".
            async_: If True, return immediately. If False, wait for completion.
            timeout: Max wait time when async_=False (seconds).
            tolerance: Position tolerance. Defaults to 2%, 3°, or 0.05 rad.

        Returns:
            True if successful (and position reached if async_=False).

        Example:
            >>> from pib3 import Joint
            >>> backend.set_joint(Joint.ELBOW_LEFT, 50.0)  # 50% of range
            >>> backend.set_joint(Joint.ELBOW_LEFT, -30.0, unit="deg")
            >>> backend.set_joint(Joint.ELBOW_LEFT, 50.0, async_=False)  # wait
        """
        motor_str = str(motor_name.value if isinstance(motor_name, Joint) else motor_name)
        return self.set_joints(
            {motor_str: position},
            unit=unit,
            async_=async_,
            timeout=timeout,
            tolerance=tolerance,
        )

    def set_joints(
        self,
        positions: Union[Dict[MotorNameType, float], Sequence[float], HandPose],
        unit: UnitType = "percent",
        async_: bool = True,
        timeout: float = 1.0,
        tolerance: Optional[float] = None,
    ) -> bool:
        """
        Set positions of multiple joints simultaneously.

        Args:
            positions: Dict mapping motor names (str or Joint) to positions,
                      sequence of positions for all MOTOR_NAMES in order,
                      or a HandPose enum member.
            unit: Unit for positions ("percent", "deg", or "rad"). Default: "percent".
            async_: If True, return immediately. If False, wait for completion.
            timeout: Max wait time when async_=False (seconds).
            tolerance: Position tolerance. Defaults to 2%, 3°, or 0.05 rad.

        Returns:
            True if successful (and positions reached if async_=False).

        Example:
            >>> from pib3 import Joint, HandPose
            >>> backend.set_joints({
            ...     Joint.SHOULDER_VERTICAL_LEFT: 50.0,
            ...     Joint.ELBOW_LEFT: 0.0,
            ... })
            >>> backend.set_joints({Joint.ELBOW_LEFT: -30.0}, unit="deg")
            >>> backend.set_joints(HandPose.LEFT_CLOSED)  # Hand pose preset
        """
        # Handle HandPose enum
        if isinstance(positions, HandPose):
            positions = positions.value

        # Convert sequence to dict if needed
        if not isinstance(positions, dict):
            positions_list = list(positions)
            if len(positions_list) != len(self.MOTOR_NAMES):
                raise ValueError(
                    f"Expected {len(self.MOTOR_NAMES)} positions, "
                    f"got {len(positions_list)}"
                )
            positions = dict(zip(self.MOTOR_NAMES, positions_list))

        # Normalize Joint enum keys to strings
        positions_str: Dict[str, float] = {
            str(k.value if isinstance(k, Joint) else k): v
            for k, v in positions.items()
        }

        # Convert to radians if needed
        if unit == "percent":
            positions_radians = {
                name: self._percent_to_radians(name, pos)
                for name, pos in positions_str.items()
            }
        elif unit == "deg":
            positions_radians = {
                name: math.radians(pos)
                for name, pos in positions_str.items()
            }
        else:  # rad
            positions_radians = dict(positions_str)

        # Send command
        success = self._set_joints_impl(positions_radians)

        if not success:
            return False

        # Wait for completion if not async
        if not async_:
            # Use appropriate default tolerance based on unit
            if tolerance is None:
                if unit == "percent":
                    tolerance = self.DEFAULT_VERIFY_TOLERANCE_PERCENT
                elif unit == "deg":
                    tolerance = self.DEFAULT_VERIFY_TOLERANCE_DEG
                else:  # rad
                    tolerance = self.DEFAULT_VERIFY_TOLERANCE

            return self._verify_positions(
                positions_str,  # Original positions in user's unit
                unit=unit,
                timeout=timeout,
                tolerance=tolerance,
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
