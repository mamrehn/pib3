"""Real robot backend via rosbridge for pib3 package."""

import json
import logging
import math
import threading
import time
import time
from typing import Callable, Dict, List, Optional, Union

import numpy as np

logger = logging.getLogger(__name__)

from .base import RobotBackend
from ..config import RobotConfig
from ..types import ImuType, AiTaskType


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
            '/camera/image',
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
            '/ai/detections',
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
            '/ai/available_models',
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
            '/ai/current_model',
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
            '/ai/current_model',
            'std_msgs/msg/String'
        )

        def parse_and_forward(msg):
            data = json.loads(msg.get('data', '{}'))
            callback(data)

        topic.subscribe(parse_and_forward)
        return topic

    # ==================== IMU METHODS ====================

    def subscribe_imu(
        self,
        callback: Callable[[dict], None],
        data_type: Union[str, ImuType] = "full",
    ) -> "roslibpy.Topic":
        """
        Subscribe to IMU data from OAK-D Lite BMI270.

        Streaming only runs while subscribed (on-demand activation).

        All data types subscribe to the same /imu/data topic but filter
        the data before passing it to your callback:
        - "full": Complete IMU message (linear_acceleration + angular_velocity)
        - "accelerometer": Only linear_acceleration in Vector3Stamped format
        - "gyroscope": Only angular_velocity in Vector3Stamped format

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

        # All IMU data comes from /imu/data - individual topics are not published
        topic = roslibpy.Topic(self._client, '/imu/data', 'sensor_msgs/msg/Imu')

        if dtype_str == ImuType.FULL.value:
            # Pass through the full IMU message
            topic.subscribe(callback)
        elif dtype_str == ImuType.ACCELEROMETER.value:
            # Extract only accelerometer data in Vector3Stamped-like format
            def extract_accel(msg):
                callback({
                    'header': msg.get('header', {}),
                    'vector': msg.get('linear_acceleration', {}),
                })
            topic.subscribe(extract_accel)
        elif dtype_str == ImuType.GYROSCOPE.value:
            # Extract only gyroscope data in Vector3Stamped-like format
            def extract_gyro(msg):
                callback({
                    'header': msg.get('header', {}),
                    'vector': msg.get('angular_velocity', {}),
                })
            topic.subscribe(extract_gyro)

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
