"""Real robot backend via rosbridge for pib3 package."""

import json
import logging
import math
import threading
import time
import time
from typing import Callable, Dict, List, Optional, Tuple, Union

import numpy as np

logger = logging.getLogger(__name__)

from .base import RobotBackend
from .audio import AudioOutput, AudioInput, RobotAudioPlayer, RobotAudioRecorder, DEFAULT_SAMPLE_RATE
from ..config import RobotConfig, LowLatencyConfig
from ..types import ImuType, AiTaskType

# Type alias for Tinkerforge motor mapping: motor_name -> (bricklet_uid, channel)
TinkerforgeMotorMapping = Dict[str, Tuple[str, int]]


# Standard PIB servo channel assignments (same wiring for all robots)
# Maps motor names to their channel index on the servo bricklet.
# The bricklet UIDs differ per robot, but channel assignments are consistent.
PIB_SERVO_CHANNELS = {
    # Servo Bricklet 1 (right arm) - channels
    "shoulder_horizontal_right": 9,
    "upper_arm_right_rotation": 9,  # Same as shoulder_horizontal_right
    "elbow_right": 8,
    "lower_arm_right_rotation": 7,
    # Right hand (if on same bricklet)
    "thumb_right_opposition": 0,
    "thumb_right_stretch": 1,
    "index_right_stretch": 2,
    "middle_right_stretch": 3,
    "ring_right_stretch": 4,
    "pinky_right_stretch": 5,

    # Servo Bricklet 2 (shoulders vertical) - channels
    "shoulder_vertical_right": 1,
    "shoulder_vertical_left": 9,

    # Servo Bricklet 3 (left arm + hand) - channels
    "shoulder_horizontal_left": 9,
    "upper_arm_left_rotation": 9,  # Same as shoulder_horizontal_left
    "elbow_left": 8,
    "lower_arm_left_rotation": 7,
    "wrist_left": 6,
    "thumb_left_opposition": 0,
    "thumb_left_stretch": 1,
    "index_left_stretch": 2,
    "middle_left_stretch": 3,
    "ring_left_stretch": 4,
    "pinky_left_stretch": 5,
}


def build_motor_mapping(
    servo1_uid: str,
    servo2_uid: str,
    servo3_uid: str,
) -> TinkerforgeMotorMapping:
    """
    Build motor mapping from servo bricklet UIDs.

    Uses the standard PIB wiring where:
    - servo1: Right arm
    - servo2: Shoulder verticals (both arms)
    - servo3: Left arm + left hand

    Args:
        servo1_uid: UID of servo bricklet for right arm.
        servo2_uid: UID of servo bricklet for shoulder verticals.
        servo3_uid: UID of servo bricklet for left arm + hand.

    Returns:
        Complete motor mapping dict for LowLatencyConfig.

    Example:
        >>> uids = robot.discover_servo_bricklets()
        >>> # Determine which UID is which by testing
        >>> mapping = build_motor_mapping(
        ...     servo1_uid=uids[0],  # Right arm
        ...     servo2_uid=uids[1],  # Shoulders
        ...     servo3_uid=uids[2],  # Left arm
        ... )
        >>> robot.configure_motor_mapping(mapping)
    """
    return {
        # Servo 1 - Right arm
        "shoulder_horizontal_right": (servo1_uid, 9),
        "upper_arm_right_rotation": (servo1_uid, 9),
        "elbow_right": (servo1_uid, 8),
        "lower_arm_right_rotation": (servo1_uid, 7),
        "thumb_right_opposition": (servo1_uid, 0),
        "thumb_right_stretch": (servo1_uid, 1),
        "index_right_stretch": (servo1_uid, 2),
        "middle_right_stretch": (servo1_uid, 3),
        "ring_right_stretch": (servo1_uid, 4),
        "pinky_right_stretch": (servo1_uid, 5),

        # Servo 2 - Shoulder verticals
        "shoulder_vertical_right": (servo2_uid, 1),
        "shoulder_vertical_left": (servo2_uid, 9),

        # Servo 3 - Left arm + hand
        "shoulder_horizontal_left": (servo3_uid, 9),
        "upper_arm_left_rotation": (servo3_uid, 9),
        "elbow_left": (servo3_uid, 8),
        "lower_arm_left_rotation": (servo3_uid, 7),
        "wrist_left": (servo3_uid, 6),
        "thumb_left_opposition": (servo3_uid, 0),
        "thumb_left_stretch": (servo3_uid, 1),
        "index_left_stretch": (servo3_uid, 2),
        "middle_left_stretch": (servo3_uid, 3),
        "ring_left_stretch": (servo3_uid, 4),
        "pinky_left_stretch": (servo3_uid, 5),
    }


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
        >>> from pib3 import Joint
        >>> from pib3.backends import RealRobotBackend
        >>> with RealRobotBackend(host="172.26.34.149") as robot:
        ...     robot.run_trajectory("trajectory.json")
        ...
        ...     # Control individual joints
        ...     robot.set_joint(Joint.ELBOW_LEFT, 50.0)  # 50% of calibrated range
        ...     robot.set_joint(Joint.ELBOW_LEFT, 45.0, unit="deg")  # 45 degrees
        ...     robot.set_joint(Joint.ELBOW_LEFT, 0.5, unit="rad")  # 0.5 radians
        ...
        ...     # Read current position
        ...     angle = robot.get_joint(Joint.ELBOW_LEFT)  # Returns percentage
        ...     angle_deg = robot.get_joint(Joint.ELBOW_LEFT, unit="deg")
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
        low_latency: Optional[LowLatencyConfig] = None,
    ):
        """
        Initialize real robot backend.

        Args:
            host: Robot IP address.
            port: Rosbridge websocket port.
            timeout: Connection timeout in seconds.
            low_latency: Configuration for direct Tinkerforge motor control.
                If enabled, motor commands bypass ROS for lower latency.
        """
        super().__init__(host=host, port=port)
        self.timeout = timeout
        self._client = None
        self._service = None
        self._motor_settings_service = None
        self._position_subscriber = None
        self._motor_settings_subscriber = None
        # Joint positions received from robot via /joint_trajectory topic
        self._joint_positions: Dict[str, float] = {}
        self._joint_positions_lock = threading.Lock()
        # Motor settings received from robot via /motor_settings topic
        self._motor_settings: Dict[str, Dict] = {}
        self._motor_settings_lock = threading.Lock()

        # Unified audio system
        self._robot_audio_player: Optional[RobotAudioPlayer] = None
        self._robot_audio_recorder: Optional[RobotAudioRecorder] = None

        # Low-latency Tinkerforge direct control
        self._low_latency_config = low_latency or LowLatencyConfig()
        self._tinkerforge_conn = None
        self._tinkerforge_servos: Dict[str, "BrickletServoV2"] = {}
        self._tinkerforge_motor_map: TinkerforgeMotorMapping = {}

    @classmethod
    def from_config(cls, config: RobotConfig) -> "RealRobotBackend":
        """Create backend from RobotConfig."""
        return cls(
            host=config.host,
            port=config.port,
            timeout=config.timeout,
            low_latency=config.low_latency,
        )

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

        # Initialize service clients
        self._service = roslibpy.Service(
            self._client,
            '/apply_joint_trajectory',
            'datatypes/ApplyJointTrajectory'
        )
        self._motor_settings_service = roslibpy.Service(
            self._client,
            '/apply_motor_settings',
            'datatypes/ApplyMotorSettings'
        )

        # Subscribe to /joint_trajectory for position feedback
        # Robot publishes current positions when motors move
        self._position_subscriber = roslibpy.Topic(
            self._client,
            '/joint_trajectory',
            'trajectory_msgs/msg/JointTrajectory'
        )
        self._position_subscriber.subscribe(self._on_joint_trajectory)

        # Subscribe to /motor_settings for settings feedback
        self._motor_settings_subscriber = roslibpy.Topic(
            self._client,
            '/motor_settings',
            'datatypes/MotorSettings'
        )
        self._motor_settings_subscriber.subscribe(self._on_motor_settings)

        # Connect Tinkerforge if low-latency mode is enabled
        if self._low_latency_config.enabled:
            self._connect_tinkerforge()

    def _on_motor_settings(self, message: dict) -> None:
        """Callback for motor settings updates from /motor_settings topic."""
        motor_name = message.get('motor_name')
        if motor_name:
            with self._motor_settings_lock:
                self._motor_settings[motor_name] = message

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

    # ==================== LOW-LATENCY TINKERFORGE METHODS ====================

    def _connect_tinkerforge(self) -> None:
        """Connect to Tinkerforge brick daemon for direct motor control.

        Establishes connection and discovers servo bricklets.
        """
        try:
            from tinkerforge.ip_connection import IPConnection
            from tinkerforge.bricklet_servo_v2 import BrickletServoV2
        except ImportError:
            logger.warning(
                "tinkerforge package not installed. Low-latency mode disabled. "
                "Install with: pip install tinkerforge"
            )
            self._low_latency_config.enabled = False
            return

        tf_host = self._low_latency_config.tinkerforge_host or self.host
        tf_port = self._low_latency_config.tinkerforge_port

        try:
            self._tinkerforge_conn = IPConnection()
            self._tinkerforge_conn.connect(tf_host, tf_port)
            logger.info(f"Connected to Tinkerforge daemon at {tf_host}:{tf_port}")

            # Use provided mapping or default
            motor_mapping = (
                self._low_latency_config.motor_mapping
                or DEFAULT_TINKERFORGE_MOTOR_MAPPING
            )

            if not motor_mapping:
                logger.warning(
                    "No Tinkerforge motor mapping configured. "
                    "Use LowLatencyConfig.motor_mapping to specify motor-to-bricklet mappings. "
                    "Format: {'motor_name': ('BRICKLET_UID', channel)}"
                )
                # Try auto-discovery of servo bricklets
                self._auto_discover_servos()
            else:
                self._tinkerforge_motor_map = motor_mapping
                # Initialize servo bricklet objects
                self._init_servo_bricklets()

        except Exception as e:
            logger.error(f"Failed to connect to Tinkerforge: {e}")
            self._low_latency_config.enabled = False
            self._tinkerforge_conn = None

    def _auto_discover_servos(self) -> None:
        """Auto-discover connected Tinkerforge servo bricklets.

        Note: This discovers bricklets but cannot automatically map them to motor names.
        The discovered UIDs should be used to configure motor_mapping.
        """
        if self._tinkerforge_conn is None:
            return

        self._discovered_servo_uids: List[str] = []

        def enumerate_callback(uid, connected_uid, position, hardware_version,
                               firmware_version, device_identifier, enumeration_type):
            # Servo Bricklet V2 device identifier is 2157
            if device_identifier == 2157:
                self._discovered_servo_uids.append(uid)
                logger.info(f"Discovered Servo Bricklet V2: UID={uid}, position={position}")

        self._tinkerforge_conn.register_callback(
            self._tinkerforge_conn.CALLBACK_ENUMERATE,
            enumerate_callback
        )
        self._tinkerforge_conn.enumerate()

        # Wait briefly for enumeration responses
        time.sleep(0.5)

        if self._discovered_servo_uids:
            logger.info(
                f"Found {len(self._discovered_servo_uids)} Servo Bricklet(s): {self._discovered_servo_uids}. "
                f"Configure LowLatencyConfig.motor_mapping with these UIDs."
            )
        else:
            logger.warning(
                "No Servo Bricklets discovered. Ensure bricklets are connected and "
                "Tinkerforge Brick Daemon is running."
            )

    def discover_servo_bricklets(self, timeout: float = 1.0) -> List[str]:
        """
        Discover connected Tinkerforge servo bricklets and return their UIDs.

        This is a convenience method to help configure motor_mapping.
        Connect to the robot first, then call this to find available bricklet UIDs.

        Args:
            timeout: Time to wait for enumeration responses (seconds).

        Returns:
            List of discovered Servo Bricklet V2 UIDs.

        Example:
            >>> with pib3.Robot(host="172.26.34.149") as robot:
            ...     # First, connect to Tinkerforge
            ...     robot._connect_tinkerforge()
            ...     # Discover available bricklets
            ...     uids = robot.discover_servo_bricklets()
            ...     print(f"Found servo bricklets: {uids}")
            ...     # Then configure mapping based on your robot's wiring
            ...     robot.configure_motor_mapping({
            ...         "elbow_left": (uids[0], 8),
            ...         # ... etc
            ...     })
        """
        if self._tinkerforge_conn is None:
            # Try to connect if not already connected
            if not self._low_latency_config.enabled:
                self._low_latency_config.enabled = True
            self._connect_tinkerforge()

        if self._tinkerforge_conn is None:
            logger.warning("Cannot discover bricklets: Tinkerforge not connected")
            return []

        discovered = []

        def enumerate_callback(uid, connected_uid, position, hardware_version,
                               firmware_version, device_identifier, enumeration_type):
            # Servo Bricklet V2 device identifier is 2157
            if device_identifier == 2157 and uid not in discovered:
                discovered.append(uid)
                logger.info(f"Discovered Servo Bricklet V2: UID={uid}, position={position}")

        self._tinkerforge_conn.register_callback(
            self._tinkerforge_conn.CALLBACK_ENUMERATE,
            enumerate_callback
        )
        self._tinkerforge_conn.enumerate()

        # Wait for enumeration responses
        time.sleep(timeout)

        return discovered

    @property
    def discovered_servo_uids(self) -> List[str]:
        """Get list of discovered Servo Bricklet UIDs from last auto-discovery."""
        return getattr(self, '_discovered_servo_uids', [])

    def _init_servo_bricklets(self, auto_configure: bool = True) -> None:
        """Initialize servo bricklet objects from the motor mapping.

        Args:
            auto_configure: If True, automatically configure all servo channels
                with default PWM and motion settings after initialization.
        """
        if self._tinkerforge_conn is None:
            return

        try:
            from tinkerforge.bricklet_servo_v2 import BrickletServoV2
        except ImportError:
            return

        # Get unique bricklet UIDs from the mapping
        unique_uids = set()
        for motor_name, (uid, channel) in self._tinkerforge_motor_map.items():
            unique_uids.add(uid)

        # Create servo bricklet objects
        for uid in unique_uids:
            try:
                servo = BrickletServoV2(uid, self._tinkerforge_conn)
                self._tinkerforge_servos[uid] = servo
                logger.debug(f"Initialized Servo Bricklet V2: {uid}")
            except Exception as e:
                logger.error(f"Failed to initialize Servo Bricklet {uid}: {e}")

        # Auto-configure all channels with default settings
        if auto_configure and self._tinkerforge_servos:
            self.configure_all_servo_channels()
            logger.info("Auto-configured all servo channels with default settings")

    def _disconnect_tinkerforge(self) -> None:
        """Disconnect from Tinkerforge brick daemon."""
        if self._tinkerforge_conn is not None:
            try:
                self._tinkerforge_conn.disconnect()
            except Exception:
                pass
            self._tinkerforge_conn = None
            self._tinkerforge_servos.clear()
            self._tinkerforge_motor_map.clear()

    @property
    def low_latency_available(self) -> bool:
        """Check if low-latency mode is available and connected."""
        return (
            self._low_latency_config.enabled
            and self._tinkerforge_conn is not None
            and bool(self._tinkerforge_motor_map)
            and bool(self._tinkerforge_servos)
        )

    @property
    def low_latency_enabled(self) -> bool:
        """Get whether low-latency mode is currently enabled."""
        return self._low_latency_config.enabled

    @low_latency_enabled.setter
    def low_latency_enabled(self, value: bool) -> None:
        """Enable or disable low-latency mode at runtime.

        If enabling and not yet connected to Tinkerforge, attempts connection.
        """
        if value and not self._low_latency_config.enabled:
            self._low_latency_config.enabled = True
            if self._tinkerforge_conn is None and self.is_connected:
                self._connect_tinkerforge()
        elif not value:
            self._low_latency_config.enabled = False

    @property
    def low_latency_sync_to_ros(self) -> bool:
        """Get whether low-latency mode syncs positions back to ROS topics."""
        return self._low_latency_config.sync_to_ros

    @low_latency_sync_to_ros.setter
    def low_latency_sync_to_ros(self, value: bool) -> None:
        """Set whether to update local position cache after direct control.

        When True, the local position cache is updated after low-latency motor
        commands, ensuring get_joint() returns correct values. This does NOT
        publish to ROS topics (that would cause double motor commands).
        """
        self._low_latency_config.sync_to_ros = value

    def configure_motor_mapping(
        self,
        mapping: TinkerforgeMotorMapping,
        reinitialize: bool = True,
    ) -> None:
        """
        Configure Tinkerforge motor-to-bricklet mapping at runtime.

        This allows setting up motor mappings after connection, useful when
        the mapping isn't known at construction time.

        Args:
            mapping: Dict mapping motor names to (bricklet_uid, channel) tuples.
                Example: {"elbow_left": ("ABC", 0), "wrist_left": ("ABC", 1)}
            reinitialize: If True, reinitialize servo bricklet connections.

        Example:
            >>> robot.configure_motor_mapping({
            ...     "shoulder_vertical_left": ("XYZ", 0),
            ...     "shoulder_horizontal_left": ("XYZ", 1),
            ...     "upper_arm_left_rotation": ("XYZ", 2),
            ...     "elbow_left": ("XYZ", 3),
            ...     "lower_arm_left_rotation": ("XYZ", 4),
            ...     "wrist_left": ("XYZ", 5),
            ... })
            >>> robot.low_latency_enabled = True
        """
        self._tinkerforge_motor_map.update(mapping)

        if reinitialize and self._tinkerforge_conn is not None:
            self._init_servo_bricklets()

    def configure_servo_channel(
        self,
        motor_name: str,
        pulse_width_min: int = 700,
        pulse_width_max: int = 2500,
        velocity: int = 9000,
        acceleration: int = 9000,
        deceleration: int = 9000,
    ) -> bool:
        """
        Configure Tinkerforge servo channel settings for a motor.

        This configures the PWM pulse width range and motion parameters
        for a specific motor. Should be called once after connecting,
        before using low-latency mode.

        Args:
            motor_name: Name of the motor to configure.
            pulse_width_min: Minimum PWM pulse width in microseconds (default: 700).
            pulse_width_max: Maximum PWM pulse width in microseconds (default: 2500).
            velocity: Maximum velocity in 0.01°/s (default: 9000 = 90°/s).
            acceleration: Acceleration in 0.01°/s² (default: 9000).
            deceleration: Deceleration in 0.01°/s² (default: 9000).

        Returns:
            True if configuration was successful.

        Example:
            >>> robot.configure_servo_channel("elbow_left",
            ...     pulse_width_min=700,
            ...     pulse_width_max=2500,
            ...     velocity=9000,
            ...     acceleration=9000,
            ...     deceleration=9000
            ... )
        """
        if motor_name not in self._tinkerforge_motor_map:
            logger.warning(f"Motor {motor_name} not in Tinkerforge mapping")
            return False

        uid, channel = self._tinkerforge_motor_map[motor_name]
        servo = self._tinkerforge_servos.get(uid)

        if servo is None:
            logger.warning(f"Servo bricklet {uid} not initialized")
            return False

        try:
            servo.set_pulse_width(channel, pulse_width_min, pulse_width_max)
            servo.set_motion_configuration(channel, velocity, acceleration, deceleration)
            logger.debug(
                f"Configured servo {motor_name}: pulse_width=[{pulse_width_min}, {pulse_width_max}], "
                f"motion=[{velocity}, {acceleration}, {deceleration}]"
            )
            return True
        except Exception as e:
            logger.error(f"Failed to configure servo {motor_name}: {e}")
            return False

    def configure_all_servo_channels(
        self,
        pulse_width_min: int = 700,
        pulse_width_max: int = 2500,
        velocity: int = 9000,
        acceleration: int = 9000,
        deceleration: int = 9000,
    ) -> bool:
        """
        Configure all mapped servo channels with the same settings.

        Convenience method to initialize all servos with default settings.
        Call this after configure_motor_mapping() and before using low-latency mode.

        Args:
            pulse_width_min: Minimum PWM pulse width in microseconds (default: 700).
            pulse_width_max: Maximum PWM pulse width in microseconds (default: 2500).
            velocity: Maximum velocity in 0.01°/s (default: 9000 = 90°/s).
            acceleration: Acceleration in 0.01°/s² (default: 9000).
            deceleration: Deceleration in 0.01°/s² (default: 9000).

        Returns:
            True if all channels were configured successfully.
        """
        all_success = True
        for motor_name in self._tinkerforge_motor_map:
            success = self.configure_servo_channel(
                motor_name,
                pulse_width_min=pulse_width_min,
                pulse_width_max=pulse_width_max,
                velocity=velocity,
                acceleration=acceleration,
                deceleration=deceleration,
            )
            all_success = all_success and success
        return all_success

    def _set_motor_direct(
        self,
        motor_name: str,
        position_centidegrees: int,
        velocity_centideg: Optional[int] = None,
    ) -> bool:
        """
        Set motor position directly via Tinkerforge (low-latency mode).

        Args:
            motor_name: Name of the motor.
            position_centidegrees: Target position in centidegrees (1/100 degree).
            velocity_centideg: Optional velocity in centidegrees/second.

        Returns:
            True if command was sent successfully.
        """
        if motor_name not in self._tinkerforge_motor_map:
            logger.debug(
                f"Motor {motor_name} not in Tinkerforge mapping, falling back to ROS"
            )
            return False

        uid, channel = self._tinkerforge_motor_map[motor_name]
        servo = self._tinkerforge_servos.get(uid)

        if servo is None:
            logger.warning(f"Servo bricklet {uid} not initialized")
            return False

        try:
            # Tinkerforge Servo V2 position is in units of 0.01° (same as centidegrees)
            # Velocity is in 0.01°/s

            # Enable the servo channel if not already enabled
            servo.set_enable(channel, True)

            # Set velocity if provided (overrides motion_configuration velocity)
            if velocity_centideg is not None:
                servo.set_velocity(channel, abs(velocity_centideg))

            # Set position
            servo.set_position(channel, position_centidegrees)

            logger.debug(
                f"Direct motor set: {motor_name} -> {position_centidegrees} centideg "
                f"(bricklet={uid}, channel={channel})"
            )
            return True

        except Exception as e:
            logger.error(f"Failed to set motor {motor_name} directly: {e}")
            return False

    def disconnect(self) -> None:
        """Close connection to robot."""
        # Disconnect Tinkerforge first
        self._disconnect_tinkerforge()

        if self._position_subscriber is not None:
            try:
                self._position_subscriber.unsubscribe()
            except Exception:
                pass
            self._position_subscriber = None

        if self._motor_settings_subscriber is not None:
            try:
                self._motor_settings_subscriber.unsubscribe()
            except Exception:
                pass
            self._motor_settings_subscriber = None

        if self._client is not None:
            try:
                self._client.terminate()
            except Exception:
                pass
            self._client = None
            self._service = None
            self._motor_settings_service = None

        with self._joint_positions_lock:
            self._joint_positions.clear()

        with self._motor_settings_lock:
            self._motor_settings.clear()

    @property
    def is_connected(self) -> bool:
        """Check if connected to robot."""
        return self._client is not None and self._client.is_connected

    def set_motor_settings(
        self,
        motors: Union[str, List[str], "Joint"],
        use_defaults: bool = False,
        timeout: float = 5.0,
        **settings,
    ) -> bool:
        """
        Apply motor settings (velocity, acceleration, etc.) to one or more motors.

        Uses the /apply_motor_settings ROS service.

        Args:
            motors: Motor name(s) to configure. Can be:
                - Single motor name: "elbow_left"
                - Joint enum: Joint.ELBOW_LEFT
                - List of names: ["elbow_left", "wrist_left"]
                - Group name: "left_arm", "right_hand", "head" (from MOTOR_GROUPS)
            use_defaults: If True, merge with DEFAULT_MOTOR_SETTINGS first.
            timeout: Service call timeout in seconds.
            **settings: Motor settings to apply. Available options:
                - turned_on (bool): Enable/disable motor
                - visible (bool): Motor visibility
                - invert (bool): Invert motor direction
                - velocity (int): Motor velocity (default: 16000)
                - acceleration (int): Acceleration (default: 10000)
                - deceleration (int): Deceleration (default: 5000)
                - pulse_width_min (int): Min PWM pulse width (default: 700)
                - pulse_width_max (int): Max PWM pulse width (default: 2500)
                - period (int): PWM period (default: 19500)
                - rotation_range_min (int): Min angle in centidegrees (default: -9000)
                - rotation_range_max (int): Max angle in centidegrees (default: 9000)

        Returns:
            True if settings were applied successfully to all motors.

        Example:
            >>> # Set velocity for single motor
            >>> robot.set_motor_settings(Joint.ELBOW_LEFT, velocity=8000)

            >>> # Set velocity for entire arm
            >>> robot.set_motor_settings("left_arm", velocity=8000, acceleration=5000)

            >>> # Apply default settings to all hand motors
            >>> robot.set_motor_settings("left_hand", use_defaults=True)

            >>> # Multiple motors
            >>> robot.set_motor_settings(["elbow_left", "wrist_left"], velocity=10000)
        """
        from ..dh_model import MOTOR_GROUPS, DEFAULT_MOTOR_SETTINGS
        from ..types import Joint

        if not self.is_connected:
            raise ConnectionError("Not connected to robot")

        if self._motor_settings_service is None:
            raise RuntimeError("Motor settings service not initialized")

        # Resolve motor names
        motor_names: List[str] = []

        if isinstance(motors, Joint):
            motor_names = [motors.value]
        elif isinstance(motors, str):
            # Check if it's a group name
            if motors in MOTOR_GROUPS:
                motor_names = MOTOR_GROUPS[motors]
            else:
                motor_names = [motors]
        elif isinstance(motors, list):
            for m in motors:
                if isinstance(m, Joint):
                    motor_names.append(m.value)
                elif isinstance(m, str):
                    if m in MOTOR_GROUPS:
                        motor_names.extend(MOTOR_GROUPS[m])
                    else:
                        motor_names.append(m)
                else:
                    raise TypeError(f"Unsupported motor type: {type(m)}")
        else:
            raise TypeError(f"Unsupported motors argument type: {type(motors)}")

        # Build settings dict
        if use_defaults:
            final_settings = dict(DEFAULT_MOTOR_SETTINGS)
            final_settings.update(settings)
        else:
            final_settings = settings

        if not final_settings:
            logger.warning("No settings provided to set_motor_settings()")
            return True

        # Apply settings to each motor
        import roslibpy

        all_success = True
        for motor_name in motor_names:
            motor_settings = {"motor_name": motor_name}
            motor_settings.update(final_settings)

            request = roslibpy.ServiceRequest({"motor_settings": motor_settings})

            try:
                response = self._motor_settings_service.call(request, timeout=timeout)
                applied = bool(response.get("settings_applied", False))
                persisted = bool(response.get("settings_persisted", False))
                success = applied or persisted

                if not success:
                    logger.warning(
                        f"Motor settings not applied for {motor_name}: {response}"
                    )
                    all_success = False
                else:
                    logger.debug(f"Motor settings applied for {motor_name}")

            except Exception as e:
                logger.error(f"Failed to apply motor settings for {motor_name}: {e}")
                all_success = False

        return all_success

    def get_motor_settings(
        self,
        motors: Optional[Union[str, List[str], "Joint"]] = None,
        use_defaults: bool = False,
    ) -> Dict[str, Dict]:
        """
        Get motor settings for one or more motors.

        Returns cached settings received from the /motor_settings topic.
        If no settings have been received yet, returns defaults if requested.

        Args:
            motors: Motor name(s) to query. Can be:
                - None: Return all cached settings
                - Single motor name: "elbow_left"
                - Joint enum: Joint.ELBOW_LEFT
                - List of names: ["elbow_left", "wrist_left"]
                - Group name: "left_arm", "right_hand", "head" (from MOTOR_GROUPS)
            use_defaults: If True, return DEFAULT_MOTOR_SETTINGS for motors
                         that have no cached settings.

        Returns:
            Dict mapping motor names to their settings dicts.
            Settings may include: turned_on, visible, invert, velocity,
            acceleration, deceleration, pulse_width_min, pulse_width_max,
            period, rotation_range_min, rotation_range_max.

        Example:
            >>> # Get all cached settings
            >>> settings = robot.get_motor_settings()

            >>> # Get settings for specific motor
            >>> settings = robot.get_motor_settings(Joint.ELBOW_LEFT)

            >>> # Get settings for arm group, with defaults for missing motors
            >>> settings = robot.get_motor_settings("left_arm", use_defaults=True)
        """
        from ..dh_model import MOTOR_GROUPS, DEFAULT_MOTOR_SETTINGS
        from ..types import Joint

        # Resolve motor names
        motor_names: Optional[List[str]] = None

        if motors is not None:
            motor_names = []
            if isinstance(motors, Joint):
                motor_names = [motors.value]
            elif isinstance(motors, str):
                if motors in MOTOR_GROUPS:
                    motor_names = MOTOR_GROUPS[motors]
                else:
                    motor_names = [motors]
            elif isinstance(motors, list):
                for m in motors:
                    if isinstance(m, Joint):
                        motor_names.append(m.value)
                    elif isinstance(m, str):
                        if m in MOTOR_GROUPS:
                            motor_names.extend(MOTOR_GROUPS[m])
                        else:
                            motor_names.append(m)
                    else:
                        raise TypeError(f"Unsupported motor type: {type(m)}")
            else:
                raise TypeError(f"Unsupported motors argument type: {type(motors)}")

        # Get cached settings
        result: Dict[str, Dict] = {}

        with self._motor_settings_lock:
            if motor_names is None:
                # Return all cached settings
                result = {k: dict(v) for k, v in self._motor_settings.items()}
            else:
                # Return settings for specified motors
                for name in motor_names:
                    if name in self._motor_settings:
                        result[name] = dict(self._motor_settings[name])
                    elif use_defaults:
                        result[name] = {"motor_name": name, **DEFAULT_MOTOR_SETTINGS}

        return result

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
        low_latency: Optional[bool] = None,
    ) -> bool:
        """
        Set joint positions via apply_joint_trajectory service or direct Tinkerforge.

        When low_latency mode is enabled and available, sends commands directly
        to Tinkerforge servo bricklets, bypassing the ROS/rosbridge stack for
        reduced latency (~5-20ms vs ~100-200ms).

        Args:
            positions_radians: Dict mapping motor names to positions in radians.
            velocity_centideg: Velocity in centidegrees/sec. If None, uses default.
            low_latency: Override low_latency setting for this call.
                - None: Use configured default (LowLatencyConfig.enabled)
                - True: Force low-latency mode (if available)
                - False: Force ROS mode

        Returns:
            True if all joints were set successfully.

        Note:
            When using low_latency mode, the new position will NOT be visible
            in ROS topics unless sync_to_ros is enabled in LowLatencyConfig.
        """
        if not self.is_connected:
            return False

        if velocity_centideg is None:
            velocity_centideg = self.DEFAULT_VELOCITY_CENTIDEG

        # Determine whether to use low-latency mode
        use_low_latency = (
            low_latency if low_latency is not None
            else self._low_latency_config.enabled
        )
        use_low_latency = use_low_latency and self.low_latency_available

        all_successful = True
        motors_via_ros = {}  # Motors that couldn't be set directly

        if use_low_latency:
            # Try direct Tinkerforge control first
            for joint_name, position in positions_radians.items():
                motor_name = JOINT_TO_ROBOT_MOTOR.get(joint_name, joint_name)
                centidegrees = self._radians_to_centidegrees(position)

                success = self._set_motor_direct(
                    motor_name,
                    centidegrees,
                    velocity_centideg=int(velocity_centideg),
                )

                if not success:
                    # Motor not in Tinkerforge mapping, queue for ROS
                    motors_via_ros[joint_name] = position
                elif self._low_latency_config.sync_to_ros:
                    # Optionally sync to ROS topic for visibility
                    self._sync_position_to_ros(motor_name, centidegrees, velocity_centideg)

            # Fall back to ROS for motors not in Tinkerforge mapping
            if motors_via_ros:
                ros_success = self._set_joints_via_ros(motors_via_ros, velocity_centideg)
                all_successful = all_successful and ros_success

            return all_successful

        # Standard ROS path
        return self._set_joints_via_ros(positions_radians, velocity_centideg)

    def _set_joints_via_ros(
        self,
        positions_radians: Dict[str, float],
        velocity_centideg: float,
    ) -> bool:
        """
        Set joint positions via ROS apply_joint_trajectory service.

        Sends each joint as a separate service call because the PIB robot's
        ROS implementation processes one joint per call.

        Args:
            positions_radians: Dict mapping motor names to positions in radians.
            velocity_centideg: Velocity in centidegrees/sec.

        Returns:
            True if all joints were set successfully.
        """
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

    def _sync_position_to_ros(
        self,
        motor_name: str,
        centidegrees: int,
        velocity_centideg: float,
    ) -> None:
        """
        Update internal position cache after direct motor set.

        NOTE: We do NOT publish to /joint_trajectory topic because that topic
        is bidirectional - publishing would trigger a second motor command
        through the ROS node, causing the motor to move twice.

        Instead, we just update our local position cache so get_joint() returns
        the correct value after a low-latency set.
        """
        # Update local position cache (convert centidegrees back to radians)
        radians = self._centidegrees_to_radians(centidegrees)
        with self._joint_positions_lock:
            self._joint_positions[motor_name] = radians

        logger.debug(f"Updated local cache for {motor_name}: {centidegrees} centideg")

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

    # ==================== CAMERA METHODS ====================

    def subscribe_camera_image(
        self,
        callback: Callable[[bytes], None],
    ) -> "roslibpy.Topic":
        """
        Subscribe to camera image stream from OAK-D Lite.

        The camera publishes hardware-encoded MJPEG frames. The callback
        receives raw JPEG bytes after base64 decoding.

        Streaming only runs while subscribed (on-demand activation).

        Note:
            Data is transmitted as base64-encoded JSON. Binary CBOR transfer
            is not currently supported by roslibpy. For high-performance
            applications, consider using rosbridge directly with CBOR encoding.

        Args:
            callback: Called with raw JPEG bytes for each frame.

        Returns:
            Topic object (call .unsubscribe() when done to stop streaming).

        Example:
            >>> def on_frame(jpeg_bytes):
            ...     with open("frame.jpg", "wb") as f:
            ...         f.write(jpeg_bytes)
            >>> sub = robot.subscribe_camera_image(on_frame)
            >>> # ... later ...
            >>> sub.unsubscribe()
        """
        if not self.is_connected:
            raise ConnectionError("Not connected to robot")

        import base64
        import roslibpy

        topic = roslibpy.Topic(
            self._client,
            '/camera/image/compressed',
            'sensor_msgs/msg/CompressedImage',
        )

        def parse_and_forward(msg):
            # Data is base64-encoded JPEG in sensor_msgs/CompressedImage
            data = msg.get('data', '')
            if isinstance(data, str) and data:
                jpeg_bytes = base64.b64decode(data)
                callback(jpeg_bytes)

        topic.subscribe(parse_and_forward)
        return topic

    def subscribe_camera_legacy(
        self,
        callback: Callable[[str], None],
    ) -> "roslibpy.Topic":
        """
        Subscribe to legacy base64-encoded camera stream.

        This is the backward-compatible endpoint. For new code,
        use subscribe_camera_image() with CBOR for better performance.

        Args:
            callback: Called with base64-encoded JPEG string.

        Returns:
            Topic object (call .unsubscribe() when done).
        """
        if not self.is_connected:
            raise ConnectionError("Not connected to robot")

        import roslibpy

        topic = roslibpy.Topic(
            self._client,
            '/camera_topic',
            'std_msgs/msg/String'
        )

        def parse_and_forward(msg):
            callback(msg.get('data', ''))

        topic.subscribe(parse_and_forward)
        return topic

    def set_camera_config(
        self,
        fps: Optional[int] = None,
        quality: Optional[int] = None,
        resolution: Optional[tuple] = None,
    ) -> None:
        """
        Configure camera settings.

        Note: Changes may cause brief stream interruption (~100-200ms)
        as the pipeline rebuilds.

        Args:
            fps: Frames per second (e.g., 30).
            quality: JPEG quality 1-100 (e.g., 80).
            resolution: (width, height) tuple (e.g., (1280, 720)).
        """
        if not self.is_connected:
            raise ConnectionError("Not connected to robot")

        import roslibpy

        config = {}
        if fps is not None:
            config['fps'] = fps
        if quality is not None:
            config['quality'] = quality
        if resolution is not None:
            config['resolution'] = list(resolution)

        if not config:
            return

        topic = roslibpy.Topic(
            self._client,
            '/camera/config',
            'std_msgs/msg/String'
        )
        topic.publish({'data': json.dumps(config)})

    # ==================== AI DETECTION METHODS ====================

    def subscribe_ai_detections(
        self,
        callback: Callable[[dict], None],
    ) -> "roslibpy.Topic":
        """
        Subscribe to AI detection results from OAK-D Lite.

        Inference only runs while subscribed (on-demand activation).
        Results format depends on the currently loaded model type
        (detection, classification, segmentation, or pose).

        Args:
            callback: Called with detection dict containing:
                - model: str - Model name (e.g., "mobilenet-ssd")
                - type: str - "detection", "classification", "segmentation", "pose"
                - frame_id: int - Frame sequence number
                - timestamp_ns: int - Timestamp in nanoseconds
                - latency_ms: float - Inference latency
                - result: dict - Model-specific results

        Returns:
            Topic object (call .unsubscribe() when done to stop inference).

        Example:
            >>> def on_detection(data):
            ...     if data['type'] == 'detection':
            ...         for det in data['result']['detections']:
            ...             print(f"Found class {det['label']} at {det['bbox']}")
            >>> sub = robot.subscribe_ai_detections(on_detection)
        """
        if not self.is_connected:
            raise ConnectionError("Not connected to robot")

        import roslibpy

        topic = roslibpy.Topic(
            self._client,
            '/camera/ai/detections',
            'std_msgs/msg/String'
        )

        def parse_and_forward(msg):
            data = json.loads(msg.get('data', '{}'))
            callback(data)

        topic.subscribe(parse_and_forward)
        return topic

    def get_available_ai_models(self, timeout: float = 5.0) -> dict:
        """
        Get list of available AI models on the robot.

        Args:
            timeout: Max time to wait for response in seconds.

        Returns:
            Dict mapping model names to their info:
            {
                "mobilenet-ssd": {
                    "type": "detection",
                    "input_size": [300, 300],
                    "fps": 30,
                    "description": "Fast object detection"
                },
                ...
            }
        """
        if not self.is_connected:
            raise ConnectionError("Not connected to robot")

        import roslibpy

        result = {}
        event = threading.Event()

        def on_models(msg):
            nonlocal result
            result = json.loads(msg.get('data', '{}'))
            event.set()

        topic = roslibpy.Topic(
            self._client,
            '/camera/ai/available_models',
            'std_msgs/msg/String'
        )
        topic.subscribe(on_models)
        event.wait(timeout=timeout)
        topic.unsubscribe()
        return result

    def set_ai_model(self, model_name: str, timeout: float = 5.0) -> bool:
        """
        Switch AI model on the OAK-D Lite camera (synchronous).

        This method waits for confirmation that the model has been
        successfully loaded before returning.

        Note: Model switching causes brief interruption (~200-500ms)
        as the pipeline rebuilds with the new neural network.

        Args:
            model_name: One of the available model names
                (e.g., "mobilenet-ssd", "yolov8n", "deeplabv3").
            timeout: Maximum time to wait for model switch confirmation
                (default: 5.0 seconds).

        Returns:
            True if model switch confirmed, False if timeout occurred.

        Example:
            >>> models = robot.get_available_ai_models()
            >>> print(list(models.keys()))
            ['mobilenet-ssd', 'yolov8n', 'yolo11s', ...]
            >>> robot.set_ai_model("yolov8n")
            True
        """
        if not self.is_connected:
            raise ConnectionError("Not connected to robot")

        import threading
        import roslibpy

        model_ready = threading.Event()

        # Subscribe to current model updates first
        model_topic = roslibpy.Topic(
            self._client,
            '/camera/ai/current_model',
            'std_msgs/msg/String'
        )

        def on_model_update(msg):
            data = json.loads(msg.get('data', '{}'))
            current_model = data.get('name', data.get('model', ''))
            # Check if the target model is now active
            if model_name in current_model or current_model in model_name:
                model_ready.set()

        model_topic.subscribe(on_model_update)

        # Publish the model change request
        config_topic = roslibpy.Topic(
            self._client,
            '/ai/config',
            'std_msgs/msg/String'
        )
        config_topic.publish({'data': json.dumps({'model': model_name})})

        # Wait for confirmation
        success = model_ready.wait(timeout=timeout)

        # Clean up subscription
        model_topic.unsubscribe()

        return success

    def set_ai_config(
        self,
        model: Optional[str] = None,
        confidence: Optional[float] = None,
        segmentation_mode: Optional[str] = None,
        segmentation_target_class: Optional[int] = None,
    ) -> None:
        """
        Configure AI inference settings.

        Args:
            model: Model name to switch to.
            confidence: Detection confidence threshold (0.0-1.0).
            segmentation_mode: "bbox" (lightweight) or "mask" (detailed RLE).
            segmentation_target_class: Class ID for mask mode segmentation.

        Note: Model/confidence changes cause pipeline rebuild (~200-500ms).
              Segmentation mode changes are instant (output format only).
        """
        if not self.is_connected:
            raise ConnectionError("Not connected to robot")

        import roslibpy

        config = {}
        if model is not None:
            config['model'] = model
        if confidence is not None:
            config['confidence'] = confidence
        if segmentation_mode is not None:
            config['segmentation_mode'] = segmentation_mode
        if segmentation_target_class is not None:
            config['segmentation_target_class'] = segmentation_target_class

        if not config:
            return

        topic = roslibpy.Topic(
            self._client,
            '/ai/config',
            'std_msgs/msg/String'
        )
        topic.publish({'data': json.dumps(config)})

    def subscribe_current_ai_model(
        self,
        callback: Callable[[dict], None],
    ) -> "roslibpy.Topic":
        """
        Subscribe to current AI model info updates.

        Args:
            callback: Called with model info dict when model changes:
                {
                    "name": "mobilenet-ssd",
                    "type": "detection",
                    "input_size": [300, 300],
                    "fps": 30,
                    "description": "..."
                }

        Returns:
            Topic object (call .unsubscribe() when done).
        """
        if not self.is_connected:
            raise ConnectionError("Not connected to robot")

        import roslibpy

        topic = roslibpy.Topic(
            self._client,
            '/camera/ai/current_model',
            'std_msgs/msg/String'
        )

        def parse_and_forward(msg):
            data = json.loads(msg.get('data', '{}'))
            callback(data)

        topic.subscribe(parse_and_forward)
        return topic

    # ==================== AUDIO METHODS ====================

    def subscribe_audio_stream(
        self,
        callback: Callable[[List[int]], None],
    ) -> "roslibpy.Topic":
        """
        Subscribe to audio stream from robot's microphone array.

        Receives raw PCM audio data captured from the microphone.
        The callback receives int16 samples that can be played back
        or processed.

        Audio Format:
            - Sample rate: 16000 Hz (16kHz)
            - Channels: 1 (Mono)
            - Bit depth: 16-bit signed integers
            - Chunk size: ~1024 samples per message

        Args:
            callback: Called with list of int16 audio samples for each chunk.

        Returns:
            Topic object (call .unsubscribe() when done to stop streaming).

        Example:
            >>> import numpy as np
            >>> audio_buffer = []
            >>> def on_audio(samples):
            ...     audio_buffer.extend(samples)
            ...     print(f"Received {len(samples)} samples")
            >>> sub = robot.subscribe_audio_stream(on_audio)
            >>> # ... record for some time ...
            >>> sub.unsubscribe()
            >>> # Convert to numpy array for processing
            >>> audio_data = np.array(audio_buffer, dtype=np.int16)
        """
        if not self.is_connected:
            raise ConnectionError("Not connected to robot")

        import roslibpy

        topic = roslibpy.Topic(
            self._client,
            '/audio_stream',
            'std_msgs/msg/Int16MultiArray'
        )

        def parse_and_forward(msg):
            # Int16MultiArray message format:
            # { "layout": {...}, "data": [int16, int16, ...] }
            data = msg.get('data', [])
            if data:
                callback(data)

        topic.subscribe(parse_and_forward)
        return topic

    def play_audio_from_speech(
        self,
        text: str,
        language: str = "en",
        wait: bool = True,
        timeout: float = 30.0,
    ) -> bool:
        """
        Play text-to-speech audio on the robot.

        Uses the robot's TTS service to synthesize and play speech.
        This is useful for making the robot speak without needing
        to send audio data.

        Args:
            text: Text to speak.
            language: Language code (e.g., "en" for English, "de" for German).
            wait: If True, wait for speech to complete. If False, return immediately.
            timeout: Max time to wait for service response (seconds).

        Returns:
            True if speech was initiated successfully.

        Example:
            >>> robot.play_audio_from_speech("Hello, I am PIB!")
            >>> robot.play_audio_from_speech("Hallo!", language="de")
            >>> # Fire and forget
            >>> robot.play_audio_from_speech("Working...", wait=False)
        """
        if not self.is_connected:
            raise ConnectionError("Not connected to robot")

        import roslibpy

        service = roslibpy.Service(
            self._client,
            '/play_audio_from_speech',
            'datatypes/srv/PlayAudioFromSpeech'
        )

        request = roslibpy.ServiceRequest({
            'speech': text,
            'language': language,
        })

        try:
            result = service.call(request, timeout=timeout)
            return result.get('success', True)
        except Exception as e:
            logger.warning(f"TTS service call failed: {e}")
            return False

    def play_audio_from_file(
        self,
        filepath: str,
        wait: bool = True,
        timeout: float = 30.0,
    ) -> bool:
        """
        Play a WAV audio file that exists on the robot's filesystem.

        Note: This requires the audio file to already exist on the robot.
        There is currently no way to upload audio files via ROS topics.
        Use this for pre-installed sounds or files transferred separately.

        Args:
            filepath: Absolute path to the WAV file on the robot's filesystem.
            wait: If True (join=True), wait for playback to finish.
                 If False (join=False), return immediately while audio plays.
            timeout: Max time to wait for service response (seconds).

        Returns:
            True if playback was initiated successfully.

        Example:
            >>> # Play a pre-installed sound effect
            >>> robot.play_audio_from_file("/home/pib/sounds/startup.wav")
            >>> # Play without waiting
            >>> robot.play_audio_from_file("/home/pib/sounds/notification.wav", wait=False)
        """
        if not self.is_connected:
            raise ConnectionError("Not connected to robot")

        import roslibpy

        service = roslibpy.Service(
            self._client,
            '/play_audio_from_file',
            'datatypes/srv/PlayAudioFromFile'
        )

        request = roslibpy.ServiceRequest({
            'filepath': filepath,
            'join': wait,
        })

        try:
            result = service.call(request, timeout=timeout)
            return result.get('success', True)
        except Exception as e:
            logger.warning(f"Play audio file service call failed: {e}")
            return False

    # ==================== IMU METHODS ====================

    def subscribe_imu(
        self,
        callback: Callable[[dict], None],
        data_type: Union[str, ImuType] = "full",
    ) -> "roslibpy.Topic":
        """
        Subscribe to IMU data from OAK-D Lite BMI270.

        Streaming only runs while subscribed (on-demand activation).

        Data types use individual topics:
        - "full": Accelerometer data with IMU-like format (linear_acceleration)
        - "accelerometer": Vector3Stamped from /camera/imu/accelerometer
        - "gyroscope": Vector3Stamped from /camera/imu/gyroscope

        Args:
            callback: Called with IMU data dict.
            data_type: One of "full", "accelerometer", "gyroscope".
                Also accepts ImuType enum members.

        Returns:
            Topic object (call .unsubscribe() when done to stop streaming).

        Example:
            >>> def on_imu(data):
            ...     accel = data['linear_acceleration']
            ...     print(f"Accel: x={accel['x']:.2f} m/s²")
            >>> sub = robot.subscribe_imu(on_imu, data_type=ImuType.FULL)
            >>>
            >>> # Or for accelerometer only:
            >>> def on_accel(data):
            ...     print(f"Accel: x={data['vector']['x']:.2f} m/s²")
            >>> sub = robot.subscribe_imu(on_accel, data_type="accelerometer")
        """
        if not self.is_connected:
            raise ConnectionError("Not connected to robot")

        import roslibpy

        # Handle Enum or string
        dtype_str = data_type.value if isinstance(data_type, ImuType) else data_type

        valid_types = [ImuType.FULL.value, ImuType.ACCELEROMETER.value, ImuType.GYROSCOPE.value]
        if dtype_str not in valid_types:
            raise ValueError(f"data_type must be one of: {valid_types}")

        # IMU data comes from individual topics:
        # - /camera/imu/accelerometer (geometry_msgs/msg/Vector3Stamped)
        # - /camera/imu/gyroscope (geometry_msgs/msg/Vector3Stamped)
        # The combined /camera/imu topic may not always publish data.

        if dtype_str == ImuType.FULL.value:
            # For full IMU, we need to combine accel + gyro from separate topics
            # Subscribe to accelerometer and pass through with both fields
            topic = roslibpy.Topic(
                self._client,
                '/camera/imu/accelerometer',
                'geometry_msgs/msg/Vector3Stamped'
            )

            def convert_to_imu_format(msg):
                # Convert Vector3Stamped to IMU-like format
                callback({
                    'header': msg.get('header', {}),
                    'linear_acceleration': msg.get('vector', {}),
                    'angular_velocity': {},  # Not available in this message
                })

            topic.subscribe(convert_to_imu_format)
        elif dtype_str == ImuType.ACCELEROMETER.value:
            # Subscribe to accelerometer topic directly
            topic = roslibpy.Topic(
                self._client,
                '/camera/imu/accelerometer',
                'geometry_msgs/msg/Vector3Stamped'
            )
            topic.subscribe(callback)
        elif dtype_str == ImuType.GYROSCOPE.value:
            # Subscribe to gyroscope topic directly
            topic = roslibpy.Topic(
                self._client,
                '/camera/imu/gyroscope',
                'geometry_msgs/msg/Vector3Stamped'
            )
            topic.subscribe(callback)

        return topic

    def set_imu_frequency(self, frequency: int) -> None:
        """
        Set IMU sampling frequency.

        Note: BMI270 rounds down to nearest valid frequency.
        Valid frequencies: 25, 50, 100, 200, 250 Hz

        Examples:
            - Request 99Hz → Get 50Hz
            - Request 150Hz → Get 100Hz
            - Request 400Hz → Get 250Hz (max)

        Args:
            frequency: Desired frequency in Hz.
        """
        valid_frequencies = [25, 50, 100, 200, 250]
        if frequency not in valid_frequencies:
            raise ValueError(
                f"Invalid frequency {frequency} Hz. "
                f"Supported frequencies are: {', '.join(map(str, valid_frequencies))}"
            )

        if not self.is_connected:
            raise ConnectionError("Not connected to robot")

        import roslibpy

        topic = roslibpy.Topic(
            self._client,
            '/imu/config',
            'std_msgs/msg/String'
        )
        topic.publish({'data': json.dumps({'frequency': frequency})})

    # ==================== UNIFIED AUDIO OVERRIDES ====================

    @property
    def default_audio_output(self) -> AudioOutput:
        """Default audio output for real robot: ROBOT."""
        return AudioOutput.ROBOT

    def _get_robot_audio_player(self) -> Optional[RobotAudioPlayer]:
        """Get or create robot audio player."""
        if self._robot_audio_player is None and self.is_connected:
            try:
                self._robot_audio_player = RobotAudioPlayer(self._client)
            except Exception as e:
                logger.warning(f"Failed to create robot audio player: {e}")
        return self._robot_audio_player

    def _play_on_robot(
        self,
        data: np.ndarray,
        sample_rate: int = DEFAULT_SAMPLE_RATE,
        block: bool = True,
    ) -> bool:
        """
        Play audio on robot speakers via /audio_playback topic.

        Args:
            data: Audio data as int16 numpy array.
            sample_rate: Sample rate in Hz (default: 16000).
            block: If True, wait for estimated playback duration.

        Returns:
            True if audio was sent successfully.
        """
        if not self.is_connected:
            logger.warning("Cannot play on robot: not connected")
            return False

        player = self._get_robot_audio_player()
        if player is None:
            logger.warning("Robot audio player not available")
            return False

        return player.play(data, sample_rate, block=block)

    @property
    def default_audio_input(self) -> AudioInput:
        """Default audio input for real robot: ROBOT."""
        return AudioInput.ROBOT

    def _get_robot_audio_recorder(self) -> Optional[RobotAudioRecorder]:
        """Get or create robot audio recorder."""
        if self._robot_audio_recorder is None and self.is_connected:
            try:
                self._robot_audio_recorder = RobotAudioRecorder(self._client)
            except Exception as e:
                logger.warning(f"Failed to create robot audio recorder: {e}")
        return self._robot_audio_recorder

    def _record_from_robot(
        self,
        duration: float,
        sample_rate: int = DEFAULT_SAMPLE_RATE,
    ) -> Optional[np.ndarray]:
        """
        Record audio from robot's microphone via /audio_input topic.

        Args:
            duration: Recording duration in seconds.
            sample_rate: Sample rate in Hz (default: 16000).

        Returns:
            Audio data as numpy array of int16 samples, or None if failed.
        """
        if not self.is_connected:
            logger.warning("Cannot record from robot: not connected")
            return None

        recorder = self._get_robot_audio_recorder()
        if recorder is None:
            logger.warning("Robot audio recorder not available")
            return None

        try:
            return recorder.record(duration, sample_rate)
        except Exception as e:
            logger.warning(f"Robot audio recording failed: {e}")
            return None


# ==================== RLE DECODER HELPER ====================

def rle_decode(rle: dict) -> np.ndarray:
    """
    Decode RLE-encoded segmentation mask to numpy array.

    The mask is encoded in COCO RLE format (column-major order).

    Args:
        rle: Dict with 'size' [height, width] and 'counts' list.

    Returns:
        Binary mask as numpy array of shape (height, width).

    Example:
        >>> def on_detection(data):
        ...     if data['type'] == 'segmentation':
        ...         result = data['result']
        ...         if result.get('mode') == 'mask':
        ...             mask = rle_decode(result['mask_rle'])
        ...             print(f"Mask shape: {mask.shape}")
    """
    h, w = rle["size"]
    counts = rle["counts"]

    pixels = []
    val = 0
    for count in counts:
        pixels.extend([val] * count)
        val = 1 - val

    # Column-major reshape (Fortran order) for COCO compatibility
    return np.array(pixels, dtype=np.uint8).reshape((h, w), order='F')
