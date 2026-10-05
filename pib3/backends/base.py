"""Abstract base class for robot control backends."""

import difflib
import time
import logging
import math
import threading
from abc import ABC, abstractmethod
from collections.abc import Mapping
from functools import lru_cache
from pathlib import Path
from typing import Callable, Dict, List, Literal, Optional, Sequence, Tuple, Union

import numpy as np
import yaml

from ..types import Joint, HandPose
from ..safety import (
    EmergencyStopError,
    KeyboardHook,
    SigintGuard,
    StopButton,
    coerce_keys,
    describe_keys,
    keyboard_hook_problem,
)
from .hints import hint
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

# Spellings people actually type. Anything else raises instead of silently
# falling back to percent, which used to turn unit="degree" into a percent
# command without any message.
_UNIT_ALIASES = {
    "percent": "percent", "%": "percent", "pct": "percent",
    "deg": "deg", "degree": "deg", "degrees": "deg", "°": "deg",
    "rad": "rad", "radian": "rad", "radians": "rad",
}


def normalize_unit(unit: str) -> str:
    """Return ``"percent"``, ``"deg"`` or ``"rad"`` for any accepted spelling.

    Raises:
        ValueError: for an unknown unit, naming the accepted ones.
    """
    key = unit.strip().lower() if isinstance(unit, str) else unit
    try:
        return _UNIT_ALIASES[key]
    except (KeyError, TypeError):
        raise ValueError(
            f"Unknown unit {unit!r}. Use unit=\"percent\" (0-100), "
            f"unit=\"deg\" (degrees) or unit=\"rad\" (radians)."
        ) from None


def resolve_joint_name(name: "MotorNameType", known: Sequence[str]) -> str:
    """Map a joint argument to its motor name, or raise with a suggestion.

    Accepts the ``Joint`` enum, the motor name (``"elbow_left"``) and the enum
    member name in any case (``"ELBOW_LEFT"``, ``"turn_head"``,
    ``"index_left"``). A typo used to be reported as "joint is not
    calibrated", or it failed without any message on the ROS path.
    """
    if isinstance(name, Joint):
        return name.value
    if not isinstance(name, str):
        raise TypeError(
            f"Joint must be a Joint (e.g. Joint.ELBOW_LEFT) or a motor name "
            f"string, got {type(name).__name__}: {name!r}"
        )
    text = name.strip()
    if text in known:
        return text
    member = Joint.__members__.get(text.upper())
    if member is not None:
        return member.value
    lower = text.lower()
    if lower in known:
        return lower
    candidates = list(known) + [m.lower() for m in Joint.__members__]
    close = difflib.get_close_matches(lower, candidates, n=1, cutoff=0.6)
    tip = f" Did you mean {close[0]!r}?" if close else ""
    raise ValueError(
        f"Unknown joint {name!r}.{tip} Use the Joint enum for autocomplete, "
        f"e.g. Joint.ELBOW_LEFT."
    )


def _check_number(what: str, value) -> float:
    """A finite real number, or a TypeError/ValueError that names ``what``."""
    if isinstance(value, bool) or not isinstance(value, (int, float, np.integer, np.floating)):
        raise TypeError(f"{what}: expected a number, got {type(value).__name__}: {value!r}")
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(f"{what}: {value} is not a finite number")
    return value


@lru_cache(maxsize=None)
def load_joint_limits(filename: str) -> Dict[str, Dict[str, float]]:
    """
    Load joint limits from a YAML config file (memoized per filename).

    Args:
        filename: Name of the limits file (e.g., "joint_limits_robot.yaml").

    Returns:
        Dict mapping joint names to their min/max limits.
    """
    config_path = Path(__file__).parent.parent / "resources" / filename
    if not config_path.exists():
        logger.warning(f"Joint limits config not found: {config_path}")
        return {}

    with open(config_path, "r") as f:
        data = yaml.safe_load(f)

    return data.get("joints", {})


def clear_joint_limits_cache() -> None:
    """Clear the joint limits cache. Useful after calibration."""
    load_joint_limits.cache_clear()


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
    # Default tolerance for verification (percentage of joint range)
    DEFAULT_VERIFY_TOLERANCE_PERCENT = 2.0  # 2%
    # Default tolerance for verification (degrees)
    DEFAULT_VERIFY_TOLERANCE_DEG = 3.0  # 3 degrees

    # Speed (deg/s) used by go_home() when the caller passes none. None means
    # "backend default". Homing is the one motion that starts from a *fully
    # unknown* pose, so backends whose joints can hang loose at power-on
    # override this with a conservative value (see RealRobotBackend).
    DEFAULT_HOME_SPEED: Optional[float] = None

    # Speed (deg/s) for every motion command that passes no ``speed``. None
    # means "backend native". A fixed default keeps a call's speed independent
    # of what ran before it (an earlier go_home() used to leave every later
    # move at the slow homing speed). Change it per object via
    # ``robot.default_speed``.
    DEFAULT_SPEED: Optional[float] = None

    # Speed (deg/s) at which run_trajectory() moves to the first waypoint
    # before it streams the rest. Without this approach the arm would still be
    # on its way to the start while the drawing is already being streamed.
    DEFAULT_APPROACH_SPEED: float = 30.0

    # Emergency-stop defaults applied by connect() (see _activate_safety).
    # The simulator needs no emergency stop, so it opts out; the real robot
    # opts in.
    ESTOP_KEYS_DEFAULT: Union[bool, Sequence[str]] = False
    ESTOP_BUTTON_DEFAULT: Union[bool, str] = False
    STOP_ON_CTRL_C: bool = True

    # Stability / race-avoidance knobs for position verification and sensor reads
    STABILITY_CONSECUTIVE_READS = 2
    STABILITY_THRESHOLD_RAD = 1e-4
    VERIFY_CONSECUTIVE_READS = 2
    # Minimum delay before accepting the first "within tolerance" poll —
    # avoids returning success before the motor has begun moving.
    VERIFY_MIN_SETTLE_SECONDS = 0.05

    # Joint limits file for this backend (override in subclasses)
    # - "joint_limits_webots.yaml" for simulation (Webots)
    # - "joint_limits_robot.yaml" for real robot
    JOINT_LIMITS_FILE: str = "joint_limits_webots.yaml"

    # Coordinate frame a Trajectory is stored in (canonical = Webots motor
    # radians, matching Trajectory.COORDINATE_FRAME). Used as the fallback
    # source when a trajectory's coordinate_frame is unknown — see
    # _trajectory_to_backend_radians.
    TRAJECTORY_SOURCE_LIMITS_FILE: str = "joint_limits_webots.yaml"

    # Maps a Trajectory.coordinate_frame name to the joint-limits file that
    # describes that frame's joint conventions, so playback can remap a
    # trajectory from whatever frame it was authored in into this backend's.
    TRAJECTORY_FRAME_LIMITS: Dict[str, str] = {
        "webots": "joint_limits_webots.yaml",
        "robot": "joint_limits_robot.yaml",
    }

    def __init__(self):
        # Networking (host/port) is a concern of networked backends only —
        # see RealRobotBackend. Simulation backends (WebotsBackend) have no
        # remote endpoint and must not pretend to.

        # Emergency stop: a latch, an Event so waits wake up at once, and a
        # lock that serialises "check latch + send command" against the
        # freeze, so no command slips out after stop() has frozen the motors.
        self._stopped = False
        self._stop_reason: Optional[str] = None
        self._stop_event = threading.Event()
        self._command_lock = threading.RLock()
        self._estop_token = id(self)
        self._estop_keys: Tuple[str, ...] = ()
        self._stop_button: Optional[StopButton] = None
        self._estop_keys_setting: Union[bool, Sequence[str]] = self.ESTOP_KEYS_DEFAULT
        self._estop_button_setting: Union[bool, str] = self.ESTOP_BUTTON_DEFAULT
        self._default_speed: Optional[float] = self.DEFAULT_SPEED

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

        # Values outside 0-100% are clamped (with a hint) by set_joints.
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
        """Context manager exit: freeze the motors if the program crashed, then disconnect.

        The servo bricklets run every move on their own. Without the freeze,
        a crash or Ctrl+C in the middle of a slow move would leave the arm
        travelling to its old target after the program had already ended.
        A normal exit (or ``sys.exit()``) lets the last commands finish.
        """
        try:
            if (exc_type is not None and not issubclass(exc_type, SystemExit)
                    and self.is_connected):
                try:
                    self._halt_motion()
                except Exception as exc:  # never mask the user's exception
                    logger.error("Could not freeze the motors: %s", exc)
                if not self._stopped:
                    logger.warning(
                        "Your program ended with %s while the robot was "
                        "connected; pib3 froze the motors where they are.",
                        exc_type.__name__,
                    )
        finally:
            self._deactivate_safety()
            self.disconnect()

    # --- Argument helpers ---

    def _resolve_joint_name(self, name: MotorNameType) -> str:
        """Canonical motor name for ``name``; raises with a suggestion on typos."""
        return resolve_joint_name(name, self.MOTOR_NAMES)

    def _to_radians(self, name: str, value: float, unit: str) -> float:
        if unit == "percent":
            return self._percent_to_radians(name, value)
        if unit == "deg":
            return math.radians(value)
        return float(value)

    def _from_radians(self, name: str, radians: float, unit: str) -> float:
        if unit == "percent":
            return self._radians_to_percent(name, radians)
        if unit == "deg":
            return math.degrees(radians)
        return float(radians)

    @staticmethod
    def _validate_speed(speed) -> float:
        value = _check_number("speed", speed)
        if value <= 0:
            # 0 would mean "no limit" (full speed) to the servo bricklets.
            # Nobody who types speed=0 wants that.
            raise ValueError(
                f"speed must be a positive number of degrees per second "
                f"(e.g. speed=60), got {speed!r}."
            )
        return value

    @property
    def default_speed(self) -> Optional[float]:
        """Speed in deg/s for motion commands that pass no ``speed``.

        Set it once to slow down a whole program, e.g. for a first run on the
        real robot::

            robot.default_speed = 45
        """
        return self._default_speed

    @default_speed.setter
    def default_speed(self, value: Optional[float]) -> None:
        self._default_speed = None if value is None else self._validate_speed(value)

    def _resolve_speed(self, speed: Optional[float]) -> Optional[float]:
        if speed is None:
            return self._default_speed
        return self._validate_speed(speed)

    def _limits_for(self, name: str) -> Optional[Tuple[float, float]]:
        lim = self._get_joint_limits().get(name) or {}
        lo, hi = lim.get("min"), lim.get("max")
        if lo is None or hi is None:
            return None
        return (min(lo, hi), max(lo, hi))

    def _clamp_to_limits(
        self,
        positions_radians: Dict[str, float],
        requested: Optional[Dict[str, float]] = None,
        unit: str = "rad",
    ) -> Tuple[Dict[str, float], List[str]]:
        """Clamp targets into the joint limits; hint once per joint.

        A servo driven against its mechanical stop stalls, draws current and
        heats up. The Tinkerforge bricklet itself allows +-90 deg on every
        channel, so the elbow (limit -45 deg) is not protected by hardware.
        """
        out: Dict[str, float] = {}
        clamped: List[str] = []
        for name, rad in positions_radians.items():
            bounds = self._limits_for(name)
            if bounds is not None:
                lo, hi = bounds
                if rad < lo - 1e-9 or rad > hi + 1e-9:
                    clamped.append(name)
                    shown = requested.get(name, rad) if requested else rad
                    lo_u = self._from_radians(name, lo, unit)
                    hi_u = self._from_radians(name, hi, unit)
                    lo_u, hi_u = min(lo_u, hi_u), max(lo_u, hi_u)
                    sym = {"percent": " %", "deg": " deg", "rad": " rad"}[unit]
                    hint(
                        f"clamp-{name}",
                        f"{name}: {shown:g}{sym} is outside the allowed range "
                        f"{lo_u:g} .. {hi_u:g}{sym}. pib3 moved the joint to the "
                        f"limit instead, and a blocking call returns False.",
                    )
                    rad = min(max(rad, lo), hi)
            out[name] = rad
        return out, clamped

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
            logger.warning("Local audio playback not available (sounddevice not installed)")
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

        Override in RealRobotBackend to receive via /audio_stream topic.
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
        motor_str = self._resolve_joint_name(motor_name)
        unit = normalize_unit(unit)
        radians = self._get_joint_radians(motor_str, timeout=timeout)
        if radians is None:
            return None
        return self._from_radians(motor_str, radians, unit)

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
        unit = normalize_unit(unit)
        if isinstance(motor_names, (str, Joint)):
            # get_joints(Joint.ELBOW_LEFT) would otherwise iterate the
            # characters of the string.
            motor_names = [motor_names]
        if motor_names is not None:
            motor_names_str = [self._resolve_joint_name(m) for m in motor_names]
        else:
            motor_names_str = None

        radians_dict = self._get_joints_radians(motor_names_str, timeout=timeout)
        out: Dict[str, float] = {}
        for name, rad in radians_dict.items():
            try:
                out[name] = self._from_radians(name, rad, unit)
            except ValueError:
                # A name the robot reports but pib3 has no limits for (only
                # possible in percent); skip it rather than fail the read.
                continue
        return out

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

    # --- Home Position Methods ---

    @property
    def home_percent(self) -> Dict[str, float]:
        """
        Home position (0 radians) expressed as percentage for each joint.

        This is the Webots starting position / real robot servo zero.
        For symmetric joints (-90° to +90°), this is 50%.
        For asymmetric joints (e.g. elbow: -45° to +90°), this varies.

        Note:
            0 radians is a *physical* convention, not a fixed percentage:
            finger joints differ between backends (Webots fingers map 0 rad to
            100% = open; the real robot maps 0 rad to 50%), so the same joint
            can report a different home percentage per backend. See
            ``go_home`` for the startup-state caveat.

        Example:
            >>> hp = backend.home_percent
            >>> print(hp["shoulder_vertical_left"])  # ~50.0
            >>> print(hp["elbow_left"])               # ~33.3
        """
        result = {}
        for name in self.MOTOR_NAMES:
            try:
                result[name] = self._radians_to_percent(name, 0.0)
            except ValueError:
                pass
        return result

    def go_home(
        self,
        async_: bool = False,
        timeout: Optional[float] = None,
        speed: Optional[float] = None,
    ) -> bool:
        """
        Move all joints to the neutral starting position (0 radians).

        This is the canonical zero pose: Webots proto zero / real robot servo
        zero.  Equivalent to ``set_joints({...: 0.0}, unit="rad")`` for all
        joints.

        Note:
            Startup state differs between backends. In Webots every motor is
            already holding 0 rad at simulation start (read from the proto
            file). The real robot powers on with its servos **un-driven** — the
            arms and fingers hang loose and report arbitrary positions until a
            command is sent. Calling ``go_home()`` is therefore the way to put
            the real robot into the same defined pose the simulator starts in
            (in direct mode this also enables/holds the servos).

        Warning:
            Homing is the only motion that starts from a **fully unknown**
            pose: on the real robot the arms hang loose, so every joint may
            travel its whole range at once. ``RealRobotBackend`` therefore
            homes *very* slowly by default (``DEFAULT_HOME_SPEED`` = 10 deg/s,
            vs ~150 deg/s for normal motion), so the worst case — 90 deg to
            zero — takes about 9 seconds. Keep people clear of the arm radius
            before calling this, and only raise the speed deliberately.

        Args:
            async_: If True, return immediately without waiting.
            timeout: Max wait time when async_=False (seconds). None
                (default) derives it from the speed, so a full-range home at
                ``DEFAULT_HOME_SPEED`` is covered.
            speed: Movement speed in degrees/second. ``None`` (default) uses
                the backend's ``DEFAULT_HOME_SPEED``, falling back to the
                backend's normal motion speed when that is ``None``.

                The homing speed applies to this call only; later moves use
                ``robot.default_speed`` again.

        Returns:
            True if all joints reached home (or command sent if async_).

        Example:
            >>> with backend as robot:
            ...     robot.go_home()             # all joints to 0 rad, safe speed
            ...     robot.go_home(speed=90.0)   # deliberately faster
        """
        home = {name: 0.0 for name in self.MOTOR_NAMES}
        effective_speed = speed if speed is not None else self.DEFAULT_HOME_SPEED
        return self.set_joints(
            home,
            unit="rad",
            async_=async_,
            timeout=timeout,
            speed=effective_speed,
        )

    # --- Emergency Stop ---

    @property
    def stopped(self) -> bool:
        """Whether the emergency stop is latched."""
        return self._stopped

    @property
    def stop_reason(self) -> Optional[str]:
        """What triggered the active emergency stop, or None."""
        return self._stop_reason if self._stopped else None

    def stop(self, reason: Optional[str] = None) -> None:
        """
        Emergency stop: freeze every motor where it is, and latch.

        The motors hold their current position. Every later motion command
        raises :class:`~pib3.EmergencyStopError` until :meth:`resume` is
        called, and running ``run_trajectory()`` / ``set_joints_sequence()`` /
        blocking ``set_joint()`` calls return False at once. Safe to call from
        any thread and more than once.

        The keys (Space, Esc, ...), Ctrl+C, the on-screen STOP button and the
        teacher's remote stop all end up here.

        Args:
            reason: Shown in the log and on the STOP button.

        Example:
            >>> robot.stop()       # freeze now
            >>> robot.resume()     # continue on purpose
        """
        first = not self._stopped
        self._stop_reason = reason or "robot.stop()"
        # Latch BEFORE freezing: a command racing with us either finished
        # sending already (the freeze below overrides it) or sees the latch.
        self._stopped = True
        self._stop_event.set()
        try:
            self._halt_motion()
        except Exception as exc:
            logger.error("Emergency stop could not freeze the motors: %s", exc)
        if first:
            logger.warning(
                "\n%s\nEMERGENCY STOP (%s): the motors hold their current position.\n"
                "Motion commands now raise EmergencyStopError.\n"
                "Continue on purpose with robot.resume(), or restart the program.\n%s",
                "=" * 70, self._stop_reason, "=" * 70,
            )
        button = self._stop_button
        if button is not None:
            button.notify_stopped(self._stop_reason)

    def resume(self) -> None:
        """
        Release the emergency stop so motion commands work again.

        This is a deliberate act, so it is a method call, never a key: a
        second press of the stop key must not start the robot again.

        Example:
            >>> robot.stop()
            >>> # ... check that nobody is in reach ...
            >>> robot.resume()
            >>> robot.set_joint(Joint.ELBOW_LEFT, 50.0)  # works again
        """
        was_stopped = self._stopped
        self._stopped = False
        self._stop_reason = None
        self._stop_event.clear()
        if was_stopped:
            logger.warning("Emergency stop released with resume(); motion commands work again.")
        button = self._stop_button
        if button is not None:
            button.notify_resumed()

    def _halt_motion(self) -> None:
        """Freeze every motor at its current position (backend-specific).

        Called by :meth:`stop` and when a program dies mid-motion. Must be
        safe to call from any thread and must not raise for a disconnected
        backend.
        """

    def _ensure_not_stopped(self, action: str) -> None:
        if self._stopped:
            raise EmergencyStopError(
                f"{action} was not sent: the emergency stop is active "
                f"({self._stop_reason}). The motors hold their position. "
                f"Call robot.resume() to continue on purpose, or restart "
                f"your program."
            )

    def enable_estop_key(
        self,
        keys: Union[None, str, Sequence[str]] = None,
    ) -> bool:
        """
        Make keys trigger :meth:`stop`, wherever the keyboard focus is.

        The default keys are Space, Esc, Numpad-0 and Pause. Every laptop has
        Space and Esc. The stop latches: pressing the key again does not
        resume.

        The keys need a global keyboard hook (``pynput``), which some systems
        block: macOS until the terminal or editor is allowed under *Input
        Monitoring*, and Linux desktops running Wayland. In those cases this
        method says so and returns False. Ctrl+C in the terminal and
        :meth:`show_stop_button` still work.

        Process-wide: all robot objects share one hook. Calling it again
        replaces this robot's key set.

        Args:
            keys: Key name or list of names, e.g. ``"space"``, ``"esc"``,
                ``"kp_0"``, ``"pause"``, ``"F12"`` or one character. ``None``
                means the defaults.

        Returns:
            True if the keys are expected to work.

        Example:
            >>> robot.enable_estop_key()                 # Space, Esc, Numpad-0, Pause
            >>> robot.enable_estop_key(["space", "f12"])
        """
        names = coerce_keys(keys)
        if not names:
            self.disable_estop_key()
            return False

        def on_key(name: str) -> None:
            if self._stopped:
                return
            # Leave the hook's thread at once; the freeze talks to hardware.
            threading.Thread(
                target=self.stop,
                kwargs={"reason": f"{describe_keys([name])} key"},
                name="pib3-estop-key",
                daemon=True,
            ).start()

        problem = keyboard_hook_problem()
        hook_problem = KeyboardHook.subscribe(self._estop_token, on_key, names)
        self._estop_keys = names if KeyboardHook.is_subscribed(self._estop_token) else ()
        problem = problem or hook_problem
        if problem:
            logger.warning(
                "The emergency-stop keys (%s) may NOT work: %s.\n"
                "  Ctrl+C in this terminal always stops the robot, and "
                "robot.show_stop_button() opens a STOP button to click.",
                describe_keys(names), problem,
            )
            return False
        logger.info("Emergency stop keys: %s", describe_keys(names))
        return True

    def disable_estop_key(self) -> None:
        """Stop listening for the emergency-stop keys for this robot object."""
        KeyboardHook.unsubscribe(self._estop_token)
        self._estop_keys = ()

    @property
    def estop_keys(self) -> Tuple[str, ...]:
        """Keys that currently trigger :meth:`stop` (empty if none)."""
        return self._estop_keys

    def show_stop_button(self, title: Optional[str] = None) -> bool:
        """
        Open a big red on-screen STOP button for this robot.

        Works on every laptop: click it with the touchpad, or press Space,
        Esc or Enter while it has focus. It stays on top of other windows and
        closes when the program ends. pib3 opens it by itself on the real
        robot when the stop keys cannot work.

        Needs tkinter, which the python.org and uv Python builds include. On
        Debian/Ubuntu system Python: ``sudo apt install python3-tk``.

        Returns:
            True once the button is on screen.
        """
        if self._stop_button is not None and self._stop_button.running:
            return True
        button = StopButton(
            on_stop=lambda: self.stop(reason="STOP button"),
            title=title or self._stop_button_title(),
        )
        if not button.start():
            logger.warning(
                "Could not open the on-screen STOP button: %s. "
                "Use Ctrl+C in the terminal to stop the robot.", button.failure,
            )
            return False
        self._stop_button = button
        if self._stopped:
            button.notify_stopped(self._stop_reason or "")
        return True

    def hide_stop_button(self) -> None:
        """Close the on-screen STOP button, if it is open."""
        button, self._stop_button = self._stop_button, None
        if button is not None:
            button.close()

    def _stop_button_title(self) -> str:
        return "pib3"

    def _activate_safety(self) -> None:
        """Arm the emergency stop; backends call this at the end of connect().

        Ctrl+C always freezes the robot (``STOP_ON_CTRL_C``). Keys and the
        on-screen button follow the constructor arguments ``estop_keys`` and
        ``stop_button``. With ``stop_button="auto"`` the button opens only
        when the keys cannot work on this system.
        """
        if self.STOP_ON_CTRL_C:
            SigintGuard.add(self._estop_token, lambda: self.stop(reason="Ctrl+C"))
        keys = self._estop_keys_setting
        keys_ok = True
        if keys:
            keys_ok = self.enable_estop_key(None if keys is True else keys)
        button = self._estop_button_setting
        if button is True or (button == "auto" and keys and not keys_ok):
            self.show_stop_button()
        if keys or button:
            ways = []
            if self._estop_keys and keys_ok:
                ways.append(describe_keys(self._estop_keys))
            if self.STOP_ON_CTRL_C:
                ways.append("Ctrl+C in this terminal")
            if self._stop_button is not None:
                ways.append("the STOP button")
            logger.warning("Emergency stop: %s.", " / ".join(ways))

    def _deactivate_safety(self) -> None:
        """Undo :meth:`_activate_safety`; backends call this in disconnect()."""
        SigintGuard.remove(self._estop_token)
        self.disable_estop_key()
        self.hide_stop_button()

    # --- Set Methods ---

    #: Acceleration (deg/s^2) the backend ramps with; used to size automatic
    #: timeouts. None = effectively instant.
    DEFAULT_ACCELERATION: Optional[float] = None

    #: Timeout (s) for blocking moves when no speed is known.
    FALLBACK_TIMEOUT = 2.0

    def _auto_timeout(self, names: Sequence[str], speed: Optional[float]) -> float:
        """Worst-case travel time for a blocking move, plus a margin.

        A fixed 2 s used to make slow moves "fail": at 30 deg/s a 90 deg move
        takes 3 s, so ``set_joint`` returned False although the joint arrived.
        Waiting is cheap, because the call returns as soon as the joint is
        there. Only a joint that never arrives waits the whole time.
        """
        if not speed:
            return self.FALLBACK_TIMEOUT
        span = 0.0
        for name in names:
            bounds = self._limits_for(name)
            span = max(span, math.degrees(bounds[1] - bounds[0]) if bounds else 180.0)
        ramp = speed / self.DEFAULT_ACCELERATION if self.DEFAULT_ACCELERATION else 0.0
        return max(self.FALLBACK_TIMEOUT, span / speed + ramp + 1.0)

    def set_joint(
        self,
        motor_name: MotorNameType,
        position: float,
        unit: UnitType = "percent",
        async_: bool = False,
        timeout: Optional[float] = None,
        tolerance: Optional[float] = None,
        speed: Optional[float] = None,
    ) -> bool:
        """
        Set position of a single joint.

        Args:
            motor_name: Joint enum (e.g. ``Joint.ELBOW_LEFT``) or motor name.
            position: Target position in specified unit. Values outside the
                joint's range are clamped to the limit (with a one-time hint).
            unit: ``"percent"`` (default, 0-100), ``"deg"`` or ``"rad"``.
            async_: If True, return right after sending. If False (default),
                wait until the joint arrives.
            timeout: Max wait when async_=False (seconds). None (default)
                derives it from the speed, so slow moves do not "fail".
            tolerance: Position tolerance. Defaults to 2%, 3°, or 0.05 rad.
            speed: Movement speed in degrees/second (e.g. 90.0 = 90°/s).
                None uses ``robot.default_speed``.

        Returns:
            True if the command was sent (async_) / the joint arrived. False
            if it did not arrive in time, was clamped to a limit, or the
            emergency stop interrupted the wait.

        Raises:
            EmergencyStopError: if the emergency stop is active.
            ValueError: for an unknown joint or unit, or speed <= 0.

        Example:
            >>> from pib3 import Joint
            >>> backend.set_joint(Joint.ELBOW_LEFT, 50.0)  # waits for completion
            >>> backend.set_joint(Joint.ELBOW_LEFT, -30.0, unit="deg")
            >>> backend.set_joint(Joint.ELBOW_LEFT, 50.0, async_=True)  # fire-and-forget
            >>> backend.set_joint(Joint.ELBOW_LEFT, 50.0, speed=45.0)  # slow movement
        """
        return self.set_joints(
            {self._resolve_joint_name(motor_name): position},
            unit=unit,
            async_=async_,
            timeout=timeout,
            tolerance=tolerance,
            speed=speed,
        )

    def set_joints(
        self,
        positions: Dict[MotorNameType, float],
        unit: UnitType = "percent",
        async_: bool = False,
        timeout: Optional[float] = None,
        tolerance: Optional[float] = None,
        speed: Optional[float] = None,
    ) -> bool:
        """
        Set positions of multiple joints simultaneously.

        For a hand-pose preset, use ``set_joints_pose(HandPose.X)``. For a
        sequence of waypoints, use ``set_joints_sequence([...])``.

        Args:
            positions: Dict mapping joints (Joint enum or motor name) to
                positions. Out-of-range values are clamped to the limit.
            unit: ``"percent"`` (default), ``"deg"`` or ``"rad"``.
            async_: If True, return right after sending. If False (default),
                wait until all joints arrive.
            timeout: Max wait when async_=False (seconds); None = derived from
                the speed.
            tolerance: Position tolerance. Defaults to 2%, 3°, or 0.05 rad.
            speed: Movement speed in degrees/second (e.g. 90.0 = 90°/s).
                None uses ``robot.default_speed``. Every command sets its
                speed, so a slow ``go_home()`` no longer slows down the moves
                after it.

        Returns:
            True if sent (async_) / all joints arrived; False otherwise (see
            :meth:`set_joint`).

        Raises:
            EmergencyStopError: if the emergency stop is active.

        Example:
            >>> from pib3 import Joint
            >>> backend.set_joints({
            ...     Joint.SHOULDER_VERTICAL_LEFT: 50.0,
            ...     Joint.ELBOW_LEFT: 0.0,
            ... })  # waits for completion
            >>> backend.set_joints({Joint.ELBOW_LEFT: -30.0}, unit="deg")
            >>> backend.set_joints({Joint.ELBOW_LEFT: 50.0}, speed=45.0)  # slow
        """
        if isinstance(positions, HandPose):
            raise TypeError(
                "Use set_joints_pose(HandPose.X) for a pose preset; "
                "set_joints takes only a mapping of motor->position."
            )
        # Accept any Mapping (plain dict, MappingProxyType from the deprecated
        # hand-pose constants, etc.) but reject non-mapping sequences.
        if not isinstance(positions, Mapping):
            raise TypeError(
                f"set_joints expects a mapping of motor->position (e.g. dict), "
                f"got {type(positions).__name__}. "
                f"For a sequence of waypoints, use set_joints_sequence()."
            )

        unit = normalize_unit(unit)
        requested: Dict[str, float] = {}
        for key, value in positions.items():
            name = self._resolve_joint_name(key)
            requested[name] = _check_number(f"position of {name}", value)
        speed = self._resolve_speed(speed)

        radians = {name: self._to_radians(name, pos, unit) for name, pos in requested.items()}
        targets, clamped = self._clamp_to_limits(radians, requested, unit)

        velocity_centideg = round(speed * 100) if speed is not None else None
        with self._command_lock:
            self._ensure_not_stopped("set_joints()")
            success = self._set_joints_impl(targets, velocity_centideg=velocity_centideg)

        if not success:
            return False
        if async_:
            return True

        if tolerance is None:
            tolerance = {
                "percent": self.DEFAULT_VERIFY_TOLERANCE_PERCENT,
                "deg": self.DEFAULT_VERIFY_TOLERANCE_DEG,
                "rad": self.DEFAULT_VERIFY_TOLERANCE,
            }[unit]
        if timeout is None:
            timeout = self._auto_timeout(list(targets), speed)

        # Verify against what was actually commanded (the clamped value), so
        # a clamped call returns as soon as the joint is at its limit.
        verify_targets = {name: self._from_radians(name, rad, unit) for name, rad in targets.items()}
        reached = self._verify_positions(
            verify_targets, unit=unit, timeout=timeout, tolerance=tolerance,
        )
        return reached and not clamped

    def set_joints_pose(
        self,
        pose: HandPose,
        async_: bool = False,
        timeout: Optional[float] = None,
        tolerance: Optional[float] = None,
        speed: Optional[float] = None,
    ) -> bool:
        """
        Apply a hand-pose preset (HandPose enum) to the robot.

        Args:
            pose: HandPose enum member (e.g. ``HandPose.LEFT_OPEN``).
            async_: If True, fire-and-forget. If False (default), wait for completion.
            timeout: Max wait time when async_=False (seconds); None = automatic.
            tolerance: Position tolerance (in percent; pose values are always percent).
            speed: Movement speed in degrees/second. None = ``default_speed``.

        Returns:
            True on success.

        Example:
            >>> from pib3 import HandPose
            >>> backend.set_joints_pose(HandPose.LEFT_CLOSED)
            >>> backend.set_joints_pose(HandPose.RIGHT_OPEN, speed=45.0)
        """
        if not isinstance(pose, HandPose):
            raise TypeError(
                f"set_joints_pose expects a HandPose member, got {type(pose).__name__}"
            )
        # HandPose values are MappingProxyType — copy into a plain dict for set_joints.
        return self.set_joints(
            dict(pose.value),
            unit="percent",
            async_=async_,
            timeout=timeout,
            tolerance=tolerance,
            speed=speed,
        )

    def set_joints_sequence(
        self,
        sequence: Sequence[Dict[MotorNameType, float]],
        unit: UnitType = "percent",
        rate_hz: float = 20.0,
        progress_callback: Optional[Callable[[int, int], None]] = None,
        speed: Optional[float] = None,
    ) -> bool:
        """
        Play a sequence of joint-position waypoints at a fixed rate.

        Lightweight sibling to ``run_trajectory()`` — no IK, no interpolation,
        no file format. Each dict in ``sequence`` is dispatched via
        ``set_joints(..., async_=True)`` every ``1/rate_hz`` seconds, on a
        fixed schedule (the time a command takes does not add up). Good for
        scripted demos, keyframe animations, or quick motion tests.

        Stops early (and returns False) on an emergency stop.

        Args:
            sequence: Iterable of dicts mapping motor names to positions.
                Each dict is one waypoint. Waypoints need not cover the
                same set of joints — joints omitted in a given waypoint
                stay at their previously commanded value.
            unit: Unit for all positions ("percent", "deg", "rad").
            rate_hz: Waypoint dispatch rate.
            progress_callback: Optional ``callback(current_index, total)``.
            speed: Movement speed in degrees/second. None = ``default_speed``.

        Returns:
            True if all waypoints were dispatched successfully.

        Example:
            >>> backend.set_joints_sequence([
            ...     {Joint.ELBOW_LEFT: 0.0},
            ...     {Joint.ELBOW_LEFT: 50.0},
            ...     {Joint.ELBOW_LEFT: 0.0},
            ... ], rate_hz=4.0)
        """
        rate_hz = _check_number("rate_hz", rate_hz)
        if rate_hz <= 0:
            raise ValueError(f"rate_hz must be positive, got {rate_hz}")
        seq = list(sequence)
        total = len(seq)
        if total == 0:
            return True
        self._ensure_not_stopped("set_joints_sequence()")

        period = 1.0 / rate_hz
        next_tick = time.monotonic()
        for i, waypoint in enumerate(seq):
            if self._stopped:
                logger.info("Sequence stopped at waypoint %d/%d", i, total)
                return False
            try:
                ok = self.set_joints(waypoint, unit=unit, async_=True, speed=speed)
            except EmergencyStopError:
                return False
            if not ok:
                logger.error("Sequence waypoint %d/%d failed to dispatch", i + 1, total)
                return False
            if progress_callback:
                progress_callback(i + 1, total)
            if i < total - 1:
                next_tick += period
                delay = next_tick - time.monotonic()
                if delay > 0:
                    if self._stop_event.wait(delay):
                        return False
                else:
                    # Fell behind: carry on from now instead of bursting.
                    next_tick = time.monotonic()

        return True

    @abstractmethod
    def _set_joints_impl(
        self,
        positions_radians: Dict[str, float],
        velocity_centideg: Optional[int] = None,
    ) -> bool:
        """
        Backend-specific implementation to set joint positions.

        Args:
            positions_radians: Dict mapping motor names to positions in radians.
            velocity_centideg: Movement speed in centidegrees/second.
                None means use the backend's default speed.

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

        Returns False at once when the emergency stop triggers.

        Args:
            target_positions: Dict of target positions (in specified unit).
            unit: Unit of the target positions ("percent", "deg" or "rad").
            timeout: Max time to wait (seconds).
            tolerance: Acceptable error (in same unit).

        Returns:
            True if all joints are within tolerance.
        """

        start_time = time.monotonic()
        check_interval = 0.05  # 50ms between checks
        required_stable = self.VERIFY_CONSECUTIVE_READS
        min_settle = self.VERIFY_MIN_SETTLE_SECONDS
        stable_count = 0

        # Give the motor a chance to start moving before the first sensor poll.
        if min_settle > 0 and self._stop_event.wait(min_settle):
            return False

        while (time.monotonic() - start_time) < timeout:
            if self._stopped:
                return False
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
                stable_count += 1
                if stable_count >= required_stable:
                    return True
            else:
                stable_count = 0

            if self._stop_event.wait(check_interval):
                return False

        return False

    def run_trajectory(
        self,
        trajectory: Union[str, Path, "Trajectory"],
        rate_hz: float = 20.0,
        progress_callback: Optional[Callable[[int, int], None]] = None,
        approach_speed: Optional[float] = None,
    ) -> bool:
        """
        Execute trajectory on this backend.

        Before streaming, the arm moves to the first waypoint at
        ``approach_speed`` and waits until it is there; otherwise the start of
        the drawing would be smeared along the way from wherever the arm was.
        Waypoints outside the joint limits are clamped (with a warning).

        Args:
            trajectory: Trajectory object or path to trajectory JSON file.
            rate_hz: Playback rate in Hz.
            progress_callback: Optional callback(current_point, total_points).
            approach_speed: Speed (deg/s) for the move to the start pose.
                None uses ``DEFAULT_APPROACH_SPEED`` (30 deg/s); 0 or a
                negative value skips the approach.

        Returns:
            True if completed successfully, False if a waypoint failed or the
            emergency stop interrupted it.

        Raises:
            EmergencyStopError: if the emergency stop is already active.
        """
        from ..trajectory import Trajectory

        rate_hz = _check_number("rate_hz", rate_hz)
        if rate_hz <= 0:
            raise ValueError(f"rate_hz must be positive, got {rate_hz}")
        if isinstance(trajectory, (str, Path)):
            trajectory = Trajectory.from_json(trajectory)
        if not isinstance(trajectory, Trajectory):
            raise TypeError(
                f"run_trajectory expects a Trajectory or a path to a trajectory "
                f"JSON file, got {type(trajectory).__name__}"
            )
        self._ensure_not_stopped("run_trajectory()")

        # Remap the trajectory's stored frame into this backend's own joint
        # convention, then into the backend's wire format. The remap is a no-op
        # on Webots and for arm/head joints; it corrects finger joints on the
        # real robot (different open/closed sign and range).
        backend_radians = self._trajectory_to_backend_radians(
            trajectory.joint_names,
            trajectory.waypoints,
            source_frame=trajectory.coordinate_frame,
        )
        backend_radians, n_clamped = self._clamp_waypoints(
            trajectory.joint_names, backend_radians
        )
        if n_clamped:
            logger.warning(
                "%d trajectory values were outside the joint limits and were "
                "clamped. trajectory.validate(robot) lists them.", n_clamped,
            )
        if backend_radians.ndim != 2 or len(backend_radians) == 0:
            return True

        if approach_speed is None:
            approach_speed = self.DEFAULT_APPROACH_SPEED
        if approach_speed and approach_speed > 0:
            self._approach_start(trajectory.joint_names, backend_radians[0], approach_speed)
            if self._stopped:
                return False

        waypoints = self._to_backend_format(backend_radians)

        return self._execute_waypoints(
            trajectory.joint_names,
            waypoints,
            rate_hz,
            progress_callback,
        )

    def _approach_start(self, joint_names: Sequence[str], first: np.ndarray,
                        speed: float) -> bool:
        """Move to a trajectory's first waypoint and wait until it is reached."""
        start = {
            name: float(first[j])
            for j, name in enumerate(joint_names)
            if name in self.MOTOR_NAMES and j < len(first)
        }
        if not start:
            return True
        try:
            ok = self.set_joints(start, unit="rad", async_=False, speed=speed)
        except EmergencyStopError:
            return False
        if not ok and not self._stopped:
            logger.warning(
                "The arm did not confirm the trajectory's start pose; "
                "playing the trajectory anyway."
            )
        return ok

    def _clamp_waypoints(self, joint_names: Sequence[str],
                         waypoints: np.ndarray) -> Tuple[np.ndarray, int]:
        """Clamp every column into its joint's limits. Returns (array, count)."""
        out = np.array(waypoints, dtype=np.float64, copy=True)
        if out.ndim != 2:
            return out, 0
        count = 0
        for col, name in enumerate(joint_names):
            if col >= out.shape[1]:
                break
            bounds = self._limits_for(name)
            if bounds is None:
                continue
            lo, hi = bounds
            column = out[:, col]
            outside = (column < lo - 1e-6) | (column > hi + 1e-6)
            count += int(outside.sum())
            out[:, col] = np.clip(column, lo, hi)
        return out, count

    def _trajectory_to_backend_radians(
        self,
        joint_names: List[str],
        waypoints: np.ndarray,
        source_frame: Optional[str] = None,
    ) -> np.ndarray:
        """Remap trajectory angles from their stored frame into this backend's.

        Trajectories are stored in a named coordinate frame (canonical is
        ``"webots"`` = Webots motor radians). Arm and head joints share identical
        limits across backends, so they pass through unchanged. Finger joints
        differ — Webots fingers run ``0 → +π/2`` (open → closed) while the real
        robot runs ``-π/2 → +π/2`` (closed → open) — so for those the *physical*
        open/closed fraction is preserved by mapping through percent:
        ``source_radians → percent (source limits) → backend_radians``.

        This is a no-op on the Webots backend (source limits == backend limits)
        and for any joint whose limits already match the source frame.

        Args:
            joint_names: Joint name for each waypoint column.
            waypoints: Array of shape (N, len(joint_names)) in source-frame radians.
            source_frame: Name of the trajectory's coordinate frame (e.g.
                ``"webots"`` or ``"robot"``). Unknown/None falls back to the
                canonical ``TRAJECTORY_SOURCE_LIMITS_FILE`` (Webots).

        Returns:
            Array of the same shape in this backend's joint convention.
        """
        source_file = self.TRAJECTORY_FRAME_LIMITS.get(
            (source_frame or "").lower(), self.TRAJECTORY_SOURCE_LIMITS_FILE
        )
        source = load_joint_limits(source_file)
        target = self._get_joint_limits()

        out = np.array(waypoints, dtype=np.float64, copy=True)
        if out.ndim != 2:
            return out

        for col, name in enumerate(joint_names):
            if col >= out.shape[1]:
                break
            s = source.get(name)
            t = target.get(name)
            if not s or not t:
                continue
            s_min, s_max = s.get("min"), s.get("max")
            t_min, t_max = t.get("min"), t.get("max")
            if s_min is None or s_max is None or t_min is None or t_max is None:
                continue
            # Identical limits (arm/head joints, or the Webots backend) → no-op.
            if abs(s_min - t_min) < 1e-9 and abs(s_max - t_max) < 1e-9:
                continue
            s_range = s_max - s_min
            if abs(s_range) < 1e-12:
                continue
            frac = (out[:, col] - s_min) / s_range
            out[:, col] = t_min + frac * (t_max - t_min)

        return out

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
