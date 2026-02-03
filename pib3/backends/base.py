"""Abstract base class for robot control backends."""

import logging
import math
from abc import ABC, abstractmethod
from pathlib import Path
from typing import Callable, Dict, List, Literal, Optional, Sequence, Union

import numpy as np
import yaml

from ..types import Joint, HandPose
from .audio import (
    AudioOutput,
    AudioInput,
    AudioDevice,
    LocalAudioPlayer,
    LocalAudioRecorder,
    PiperTTS,
    load_audio_file,
    save_audio_file,
    resample_audio,
    list_audio_input_devices,
    list_audio_output_devices,
    get_default_audio_input_device,
    get_default_audio_output_device,
    DEFAULT_SAMPLE_RATE,
    HAS_SOUNDDEVICE,
)

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
        ...     # Fire-and-forget (don't wait for completion)
        ...     robot.set_joint(Joint.ELBOW_LEFT, 50.0, async_=True)
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

        # Unified audio system
        self._audio_output: AudioOutput = AudioOutput.LOCAL
        self._audio_input: AudioInput = AudioInput.LOCAL
        self._local_player: Optional[LocalAudioPlayer] = None
        self._local_recorder: Optional[LocalAudioRecorder] = None
        self._tts: Optional[PiperTTS] = None

        # Device selection (None = system default)
        self._output_device: Optional[Union[int, str, AudioDevice]] = None
        self._input_device: Optional[Union[int, str, AudioDevice]] = None

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

    # --- Unified Audio Playback Methods ---

    @property
    def default_audio_output(self) -> AudioOutput:
        """
        Get the default audio output destination for this backend.

        Override in subclasses:
        - RealRobotBackend: ROBOT
        - WebotsBackend: LOCAL
        """
        return AudioOutput.LOCAL

    def set_audio_output(self, output: AudioOutput) -> None:
        """
        Set the default audio output destination.

        Args:
            output: AudioOutput.LOCAL, AudioOutput.ROBOT, or AudioOutput.LOCAL_AND_ROBOT.

        Example:
            >>> robot.set_audio_output(AudioOutput.ROBOT)
            >>> robot.speak("Hello")  # Plays on robot
        """
        self._audio_output = output

    def _get_local_player(self) -> Optional[LocalAudioPlayer]:
        """Get or create local audio player."""
        if self._local_player is None and HAS_SOUNDDEVICE:
            try:
                self._local_player = LocalAudioPlayer(device=self._output_device)
            except Exception as e:
                logger.warning(f"Failed to create local audio player: {e}")
        return self._local_player

    def _get_local_recorder(self) -> Optional[LocalAudioRecorder]:
        """Get or create local audio recorder."""
        if self._local_recorder is None and HAS_SOUNDDEVICE:
            try:
                self._local_recorder = LocalAudioRecorder(device=self._input_device)
            except Exception as e:
                logger.warning(f"Failed to create local audio recorder: {e}")
        return self._local_recorder

    def _get_tts(self, voice: Optional[str] = None) -> PiperTTS:
        """Get or create TTS engine."""
        if self._tts is None or (voice and self._tts.voice != voice):
            from .audio import DEFAULT_PIPER_VOICE
            self._tts = PiperTTS(voice=voice or DEFAULT_PIPER_VOICE)
        return self._tts

    def _play_on_local(
        self,
        data: np.ndarray,
        sample_rate: int = DEFAULT_SAMPLE_RATE,
        block: bool = True,
    ) -> bool:
        """Play audio on local speakers."""
        player = self._get_local_player()
        if player is None:
            logger.warning("Local audio playback not available (simpleaudio not installed)")
            return False
        return player.play(data, sample_rate, block=block)

    def _play_on_robot(
        self,
        data: np.ndarray,
        sample_rate: int = DEFAULT_SAMPLE_RATE,
        block: bool = True,
    ) -> bool:
        """
        Play audio on robot speakers.

        Override in RealRobotBackend to send via /audio_playback topic.
        Base implementation returns False (not supported).
        """
        logger.warning("Robot audio playback not supported in this backend")
        return False

    def _is_webots(self) -> bool:
        """
        Check if this is a Webots backend.

        Used to determine if ROBOT should be treated as LOCAL.
        """
        return False

    def play_audio(
        self,
        data: Union[bytes, np.ndarray, List[int]],
        sample_rate: int = DEFAULT_SAMPLE_RATE,
        output: Optional[AudioOutput] = None,
        block: bool = True,
    ) -> bool:
        """
        Play audio data.

        Args:
            data: Audio data as bytes, numpy array (int16), or list of int16.
            sample_rate: Sample rate in Hz (default: 16000).
            output: Playback destination (LOCAL, ROBOT, LOCAL_AND_ROBOT).
                    If None, uses default for this backend.
            block: If True, wait for playback to complete.

        Returns:
            True if playback succeeded (on at least one output).

        Example:
            >>> robot.play_audio(audio_data, output=AudioOutput.LOCAL)
            >>> robot.play_audio(audio_data, output=AudioOutput.ROBOT)
            >>> robot.play_audio(audio_data, output=AudioOutput.LOCAL_AND_ROBOT)
        """
        # Convert to numpy int16
        if isinstance(data, bytes):
            audio = np.frombuffer(data, dtype=np.int16)
        elif isinstance(data, list):
            audio = np.array(data, dtype=np.int16)
        elif isinstance(data, np.ndarray):
            audio = data.astype(np.int16)
        else:
            raise ValueError(f"Unsupported audio data type: {type(data)}")

        # Use default output if not specified
        if output is None:
            output = self._audio_output or self.default_audio_output

        # For Webots: ROBOT and LOCAL_AND_ROBOT both resolve to LOCAL only
        if self._is_webots():
            if output in (AudioOutput.ROBOT, AudioOutput.LOCAL_AND_ROBOT):
                output = AudioOutput.LOCAL

        # Dispatch based on output
        success = False

        if output == AudioOutput.LOCAL:
            success = self._play_on_local(audio, sample_rate, block=block)

        elif output == AudioOutput.ROBOT:
            success = self._play_on_robot(audio, sample_rate, block=block)

        elif output == AudioOutput.LOCAL_AND_ROBOT:
            # Play on both (best effort sync)
            # Start robot playback first (non-blocking), then local
            robot_success = self._play_on_robot(audio, sample_rate, block=False)
            local_success = self._play_on_local(audio, sample_rate, block=block)

            # If blocking, also wait for estimated robot playback duration
            if block and robot_success:
                import time
                # Robot playback already waited via estimated duration
                pass

            success = local_success or robot_success

        return success

    def play_file(
        self,
        filepath: Union[str, Path],
        output: Optional[AudioOutput] = None,
        block: bool = True,
    ) -> bool:
        """
        Play audio from a file.

        Supports WAV files. Audio is automatically converted to 16kHz mono.

        Args:
            filepath: Path to audio file.
            output: Playback destination (LOCAL, ROBOT, LOCAL_AND_ROBOT).
                    If None, uses default for this backend.
            block: If True, wait for playback to complete.

        Returns:
            True if playback succeeded.

        Example:
            >>> robot.play_file("sound.wav", output=AudioOutput.ROBOT)
        """
        try:
            audio, file_sample_rate = load_audio_file(filepath)

            # Resample to standard rate if needed
            if file_sample_rate != DEFAULT_SAMPLE_RATE:
                audio = resample_audio(audio, file_sample_rate, DEFAULT_SAMPLE_RATE)

            return self.play_audio(audio, DEFAULT_SAMPLE_RATE, output=output, block=block)

        except Exception as e:
            logger.error(f"Failed to play audio file {filepath}: {e}")
            return False

    def speak(
        self,
        text: str,
        output: Optional[AudioOutput] = None,
        voice: Optional[str] = None,
        block: bool = True,
    ) -> bool:
        """
        Synthesize and play text-to-speech.

        Uses Piper TTS with Thorsten German voice by default.

        Args:
            text: Text to speak.
            output: Playback destination (LOCAL, ROBOT, LOCAL_AND_ROBOT).
                    If None, uses default for this backend.
            voice: Piper voice model name (default: de_DE-thorsten-high).
            block: If True, wait for playback to complete.

        Returns:
            True if speech was played successfully.

        Example:
            >>> robot.speak("Hallo, ich bin pib!")
            >>> robot.speak("Hello world", voice="en_US-lessac-medium")
        """
        try:
            tts = self._get_tts(voice)
            audio = tts.synthesize(text, sample_rate=DEFAULT_SAMPLE_RATE)
            return self.play_audio(audio, DEFAULT_SAMPLE_RATE, output=output, block=block)

        except ImportError:
            logger.error(
                "TTS not available. Install with: pip install piper-tts"
            )
            return False
        except Exception as e:
            logger.error(f"TTS failed: {e}")
            return False

    # --- Audio Device Selection Methods ---

    def get_audio_output_devices(self) -> List[AudioDevice]:
        """
        List available audio output (speaker) devices.

        Returns:
            List of AudioDevice objects that support output.

        Example:
            >>> for device in robot.get_audio_output_devices():
            ...     print(device)
        """
        return list_audio_output_devices()

    def get_audio_input_devices(self) -> List[AudioDevice]:
        """
        List available audio input (microphone) devices.

        Returns:
            List of AudioDevice objects that support input.

        Example:
            >>> for device in robot.get_audio_input_devices():
            ...     print(device)
        """
        return list_audio_input_devices()

    def set_audio_output_device(self, device: Optional[Union[int, str, AudioDevice]]) -> None:
        """
        Set the audio output device for local playback.

        Args:
            device: Device index (int), name substring (str), AudioDevice object,
                    or None to use system default.

        Example:
            >>> devices = robot.get_audio_output_devices()
            >>> robot.set_audio_output_device(devices[0])  # Use first device
            >>> robot.set_audio_output_device("Speakers")  # Search by name
            >>> robot.set_audio_output_device(0)           # Use by index
            >>> robot.set_audio_output_device(None)        # Use system default
        """
        self._output_device = device
        # Update existing player if it exists
        if self._local_player is not None:
            self._local_player.set_device(device)

    def set_audio_input_device(self, device: Optional[Union[int, str, AudioDevice]]) -> None:
        """
        Set the audio input device for local recording.

        Args:
            device: Device index (int), name substring (str), AudioDevice object,
                    or None to use system default.

        Example:
            >>> devices = robot.get_audio_input_devices()
            >>> robot.set_audio_input_device(devices[0])  # Use first device
            >>> robot.set_audio_input_device("Microphone")  # Search by name
            >>> robot.set_audio_input_device(0)            # Use by index
            >>> robot.set_audio_input_device(None)         # Use system default
        """
        self._input_device = device
        # Update existing recorder if it exists
        if self._local_recorder is not None:
            self._local_recorder.set_device(device)

    def get_current_audio_output_device(self) -> Optional[AudioDevice]:
        """Get the currently selected audio output device (None = system default)."""
        if self._output_device is None:
            return get_default_audio_output_device()
        if isinstance(self._output_device, AudioDevice):
            return self._output_device
        # Resolve by index or name
        for d in list_audio_output_devices():
            if isinstance(self._output_device, int) and d.index == self._output_device:
                return d
            if isinstance(self._output_device, str) and self._output_device.lower() in d.name.lower():
                return d
        return None

    def get_current_audio_input_device(self) -> Optional[AudioDevice]:
        """Get the currently selected audio input device (None = system default)."""
        if self._input_device is None:
            return get_default_audio_input_device()
        if isinstance(self._input_device, AudioDevice):
            return self._input_device
        # Resolve by index or name
        for d in list_audio_input_devices():
            if isinstance(self._input_device, int) and d.index == self._input_device:
                return d
            if isinstance(self._input_device, str) and self._input_device.lower() in d.name.lower():
                return d
        return None

    # --- Unified Audio Recording Methods ---

    @property
    def default_audio_input(self) -> AudioInput:
        """
        Get the default audio input source for this backend.

        Override in subclasses:
        - RealRobotBackend: ROBOT
        - WebotsBackend: LOCAL
        """
        return AudioInput.LOCAL

    def set_audio_input(self, input_source: AudioInput) -> None:
        """
        Set the default audio input source.

        Args:
            input_source: AudioInput.LOCAL or AudioInput.ROBOT.

        Example:
            >>> robot.set_audio_input(AudioInput.ROBOT)
            >>> audio = robot.record_audio(duration=5.0)  # Records from robot mic
        """
        self._audio_input = input_source

    def _record_from_local(
        self,
        duration: float,
        sample_rate: int = DEFAULT_SAMPLE_RATE,
    ) -> Optional[np.ndarray]:
        """Record audio from local microphone."""
        recorder = self._get_local_recorder()
        if recorder is None:
            logger.warning("Local audio recording not available (sounddevice not installed)")
            return None
        try:
            return recorder.record(duration, sample_rate)
        except Exception as e:
            logger.warning(f"Local audio recording failed: {e}")
            return None

    def _record_from_robot(
        self,
        duration: float,
        sample_rate: int = DEFAULT_SAMPLE_RATE,
    ) -> Optional[np.ndarray]:
        """
        Record audio from robot microphone.

        Override in RealRobotBackend to receive via /audio_input topic.
        Base implementation returns None (not supported).
        """
        logger.warning("Robot audio recording not supported in this backend")
        return None

    def record_audio(
        self,
        duration: float,
        sample_rate: int = DEFAULT_SAMPLE_RATE,
        input_source: Optional[AudioInput] = None,
    ) -> Optional[np.ndarray]:
        """
        Record audio for a specified duration.

        Args:
            duration: Recording duration in seconds.
            sample_rate: Sample rate in Hz (default: 16000).
            input_source: Recording source (LOCAL or ROBOT).
                         If None, uses default for this backend.

        Returns:
            Audio data as numpy array of int16 samples, or None if failed.
            The returned array is compatible with play_audio() and save_audio_file().

        Example:
            >>> # Record from local microphone
            >>> audio = robot.record_audio(duration=5.0, input_source=AudioInput.LOCAL)
            >>>
            >>> # Record from robot microphone
            >>> audio = robot.record_audio(duration=5.0, input_source=AudioInput.ROBOT)
            >>>
            >>> # Play back the recording
            >>> robot.play_audio(audio)
            >>>
            >>> # Save to file
            >>> from pib3.backends.audio import save_audio_file
            >>> save_audio_file("recording.wav", audio)
        """
        # Use default input if not specified
        if input_source is None:
            input_source = self._audio_input or self.default_audio_input

        # Check for common mistake: passing AudioOutput instead of AudioInput
        if isinstance(input_source, AudioOutput):
            raise TypeError(
                f"record_audio() requires AudioInput, not AudioOutput. "
                f"Use AudioInput.LOCAL or AudioInput.ROBOT instead of {input_source}."
            )

        # Validate input_source is a valid AudioInput
        if not isinstance(input_source, AudioInput):
            raise TypeError(
                f"input_source must be AudioInput.LOCAL or AudioInput.ROBOT, "
                f"got {type(input_source).__name__}: {input_source}"
            )

        # For Webots: ROBOT resolves to LOCAL
        if self._is_webots() and input_source == AudioInput.ROBOT:
            input_source = AudioInput.LOCAL

        # Dispatch based on input source
        if input_source == AudioInput.LOCAL:
            return self._record_from_local(duration, sample_rate)
        elif input_source == AudioInput.ROBOT:
            return self._record_from_robot(duration, sample_rate)
        else:
            logger.error(f"Unknown audio input source: {input_source}")
            return None

    def record_to_file(
        self,
        filepath: Union[str, Path],
        duration: float,
        sample_rate: int = DEFAULT_SAMPLE_RATE,
        input_source: Optional[AudioInput] = None,
    ) -> bool:
        """
        Record audio and save directly to a file.

        Args:
            filepath: Output file path (WAV format).
            duration: Recording duration in seconds.
            sample_rate: Sample rate in Hz (default: 16000).
            input_source: Recording source (LOCAL or ROBOT).
                         If None, uses default for this backend.

        Returns:
            True if recording was saved successfully.

        Example:
            >>> robot.record_to_file("recording.wav", duration=5.0)
            >>> robot.record_to_file("robot_audio.wav", duration=10.0, input_source=AudioInput.ROBOT)
        """
        audio = self.record_audio(duration, sample_rate, input_source)
        if audio is None:
            return False

        try:
            save_audio_file(filepath, audio, sample_rate)
            return True
        except Exception as e:
            logger.error(f"Failed to save audio file {filepath}: {e}")
            return False

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
        async_: bool = False,
        timeout: float = 2.0,
        tolerance: Optional[float] = None,
    ) -> bool:
        """
        Set position of a single joint.

        Args:
            motor_name: Motor name as string or Joint enum (e.g., Joint.ELBOW_LEFT).
            position: Target position in specified unit.
            unit: Unit for position ("percent", "deg", or "rad"). Default: "percent".
            async_: If True, return immediately. If False (default), wait for
                completion by polling the actual position until it matches.
            timeout: Max wait time when async_=False (seconds).
            tolerance: Position tolerance. Defaults to 2%, 3°, or 0.05 rad.

        Returns:
            True if successful (and position reached if async_=False).

        Example:
            >>> from pib3 import Joint
            >>> backend.set_joint(Joint.ELBOW_LEFT, 50.0)  # waits for completion
            >>> backend.set_joint(Joint.ELBOW_LEFT, -30.0, unit="deg")
            >>> backend.set_joint(Joint.ELBOW_LEFT, 50.0, async_=True)  # fire-and-forget
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
        async_: bool = False,
        timeout: float = 2.0,
        tolerance: Optional[float] = None,
    ) -> bool:
        """
        Set positions of multiple joints simultaneously.

        Args:
            positions: Dict mapping motor names (str or Joint) to positions,
                      sequence of positions for all MOTOR_NAMES in order,
                      or a HandPose enum member.
            unit: Unit for positions ("percent", "deg", or "rad"). Default: "percent".
            async_: If True, return immediately. If False (default), wait for
                completion by polling actual positions until they match.
            timeout: Max wait time when async_=False (seconds).
            tolerance: Position tolerance. Defaults to 2%, 3°, or 0.05 rad.

        Returns:
            True if successful (and positions reached if async_=False).

        Example:
            >>> from pib3 import Joint, HandPose
            >>> backend.set_joints({
            ...     Joint.SHOULDER_VERTICAL_LEFT: 50.0,
            ...     Joint.ELBOW_LEFT: 0.0,
            ... })  # waits for completion
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
