"""
Audio backends for PIB robot.

This module provides:
- AudioDevice: Dataclass representing an audio input/output device
- AudioDeviceManager: Lists and manages available audio devices
- AudioBackend: Abstract base class for playing audio
- AudioInputBackend: Abstract base class for recording audio
- NoOpAudioBackend: Fallback that does nothing
- SystemAudioBackend: Play audio on local system speakers
- SystemAudioInputBackend: Record from local system microphone
- ROSAudioBackend: Stream audio to robot via ROS topic
- ROSAudioInputBackend: Receive audio from robot's microphone
- AudioStreamReceiver: Convenience class to receive and buffer audio

Audio Format (for streaming):
- Sample rate: 16000 Hz (default)
- Channels: 1 (Mono)
- Bit depth: 16-bit signed integers (int16)

Device Types:
- "local": Physical devices connected to the machine running this code
- "robot": Robot's microphone/speaker (via ROS, only for RealRobotBackend)
"""

import abc
import threading
import warnings
from dataclasses import dataclass, field
from enum import Enum
from typing import Union, List, Optional, Callable, Dict, Any
import numpy as np

# Optional imports for audio libraries
try:
    import simpleaudio as sa
except ImportError:
    sa = None

try:
    import sounddevice as sd
except ImportError:
    sd = None

try:
    import roslibpy
except ImportError:
    roslibpy = None


# Default audio parameters
DEFAULT_SAMPLE_RATE = 16000
DEFAULT_CHANNELS = 1
DEFAULT_SAMPLE_WIDTH = 2  # 16-bit = 2 bytes
DEFAULT_CHUNK_SIZE = 1024


class AudioDeviceType(Enum):
    """Type of audio device location."""
    LOCAL = "local"    # Physical device on local machine
    ROBOT = "robot"    # Robot's audio device via ROS


@dataclass
class AudioDevice:
    """
    Represents an audio input or output device.

    Attributes:
        id: Unique identifier for the device.
        name: Human-readable device name.
        device_type: Whether device is local or on robot.
        is_input: True if device can record audio (microphone).
        is_output: True if device can play audio (speaker).
        sample_rate: Default/supported sample rate in Hz.
        channels: Number of audio channels.
        index: Backend-specific device index (for local devices).
    """
    id: str
    name: str
    device_type: AudioDeviceType
    is_input: bool = False
    is_output: bool = False
    sample_rate: int = DEFAULT_SAMPLE_RATE
    channels: int = DEFAULT_CHANNELS
    index: Optional[int] = None  # For sounddevice device index
    extra: Dict[str, Any] = field(default_factory=dict)

    def __str__(self) -> str:
        direction = []
        if self.is_input:
            direction.append("input")
        if self.is_output:
            direction.append("output")
        return f"{self.name} [{self.device_type.value}] ({'/'.join(direction)})"


# Pre-defined robot devices (logical, not physical)
ROBOT_MICROPHONE = AudioDevice(
    id="robot_mic",
    name="Robot Microphone",
    device_type=AudioDeviceType.ROBOT,
    is_input=True,
    is_output=False,
    sample_rate=DEFAULT_SAMPLE_RATE,
    channels=1,
)

ROBOT_SPEAKER = AudioDevice(
    id="robot_speaker",
    name="Robot Speaker",
    device_type=AudioDeviceType.ROBOT,
    is_input=False,
    is_output=True,
    sample_rate=DEFAULT_SAMPLE_RATE,
    channels=1,
)


class AudioDeviceManager:
    """
    Manages audio device discovery and listing.

    Provides methods to list available input (microphone) and output (speaker)
    devices from both local system and robot (when connected).

    Example:
        >>> manager = AudioDeviceManager()
        >>> # List all microphones
        >>> for mic in manager.list_input_devices():
        ...     print(f"  {mic}")
        >>> # List all speakers
        >>> for speaker in manager.list_output_devices():
        ...     print(f"  {speaker}")
    """

    def __init__(self, include_robot: bool = False):
        """
        Initialize the device manager.

        Args:
            include_robot: If True, include robot devices in listings.
                          Should be True when connected to real robot.
        """
        self.include_robot = include_robot
        self._cached_devices: Optional[List[AudioDevice]] = None

    def _get_local_devices(self) -> List[AudioDevice]:
        """Query local audio devices using sounddevice."""
        devices = []

        if sd is None:
            return devices

        try:
            device_list = sd.query_devices()
            for i, dev in enumerate(device_list):
                # Skip devices with no channels
                max_input = dev.get('max_input_channels', 0)
                max_output = dev.get('max_output_channels', 0)

                if max_input == 0 and max_output == 0:
                    continue

                device = AudioDevice(
                    id=f"local_{i}",
                    name=dev.get('name', f'Device {i}'),
                    device_type=AudioDeviceType.LOCAL,
                    is_input=max_input > 0,
                    is_output=max_output > 0,
                    sample_rate=int(dev.get('default_samplerate', DEFAULT_SAMPLE_RATE)),
                    channels=max(max_input, max_output),
                    index=i,
                    extra={
                        'hostapi': dev.get('hostapi'),
                        'max_input_channels': max_input,
                        'max_output_channels': max_output,
                    }
                )
                devices.append(device)
        except Exception as e:
            warnings.warn(f"Failed to query local audio devices: {e}")

        return devices

    def refresh(self) -> None:
        """Refresh the cached device list."""
        self._cached_devices = None

    def list_all_devices(self) -> List[AudioDevice]:
        """
        List all available audio devices.

        Returns:
            List of AudioDevice objects for all input and output devices.
        """
        if self._cached_devices is not None:
            return self._cached_devices

        devices = self._get_local_devices()

        # Add robot devices if enabled
        if self.include_robot:
            devices.append(ROBOT_MICROPHONE)
            devices.append(ROBOT_SPEAKER)

        self._cached_devices = devices
        return devices

    def list_input_devices(self) -> List[AudioDevice]:
        """
        List available input devices (microphones).

        Returns:
            List of AudioDevice objects that can record audio.
        """
        return [d for d in self.list_all_devices() if d.is_input]

    def list_output_devices(self) -> List[AudioDevice]:
        """
        List available output devices (speakers).

        Returns:
            List of AudioDevice objects that can play audio.
        """
        return [d for d in self.list_all_devices() if d.is_output]

    def get_device_by_id(self, device_id: str) -> Optional[AudioDevice]:
        """
        Get a device by its ID.

        Args:
            device_id: The device ID (e.g., "local_0", "robot_mic").

        Returns:
            AudioDevice if found, None otherwise.
        """
        for device in self.list_all_devices():
            if device.id == device_id:
                return device
        return None

    def get_default_input_device(self) -> Optional[AudioDevice]:
        """Get the default input device (microphone)."""
        if sd is not None:
            try:
                default_idx = sd.default.device[0]
                if default_idx is not None:
                    for device in self.list_input_devices():
                        if device.index == default_idx:
                            return device
            except Exception:
                pass

        # Fallback to first available input
        inputs = self.list_input_devices()
        return inputs[0] if inputs else None

    def get_default_output_device(self) -> Optional[AudioDevice]:
        """Get the default output device (speaker)."""
        if sd is not None:
            try:
                default_idx = sd.default.device[1]
                if default_idx is not None:
                    for device in self.list_output_devices():
                        if device.index == default_idx:
                            return device
            except Exception:
                pass

        # Fallback to first available output
        outputs = self.list_output_devices()
        return outputs[0] if outputs else None


# ==================== OUTPUT BACKENDS (Playback) ====================


class AudioBackend(abc.ABC):
    """Abstract base class for audio output (playback) backends."""

    @abc.abstractmethod
    def play(self, data: Union[bytes, np.ndarray, List[int]], sample_rate: int = 16000) -> bool:
        """
        Play audio data.

        Args:
            data: Audio data. Can be:
                - bytes: Raw 16-bit PCM bytes.
                - np.ndarray: NumPy array of int16 values.
                - List[int]: List of int16 values.
            sample_rate: Sample rate in Hz. Default: 16000.

        Returns:
            True if playback started successfully, False otherwise.
        """
        pass

    @abc.abstractmethod
    def stop(self) -> None:
        """Stop current playback."""
        pass


class NoOpAudioBackend(AudioBackend):
    """Fallback backend that does nothing."""

    def play(self, data: Union[bytes, np.ndarray, List[int]], sample_rate: int = 16000) -> bool:
        return False

    def stop(self) -> None:
        pass


class SystemAudioBackend(AudioBackend):
    """
    Plays audio locally using sounddevice or simpleaudio.

    Supports device selection for output to specific speakers.
    """

    def __init__(self, device: Optional[AudioDevice] = None):
        """
        Initialize system audio backend.

        Args:
            device: Optional AudioDevice to use for output.
                   If None, uses system default.
        """
        self._device = device
        self._device_index = device.index if device else None
        self.play_obj = None
        self._stream = None

        # Prefer sounddevice for device selection, fall back to simpleaudio
        if sd is not None:
            self._use_sounddevice = True
        elif sa is not None:
            self._use_sounddevice = False
            if device is not None:
                warnings.warn(
                    "Device selection requires sounddevice. "
                    "Using default output. Install with: pip install sounddevice"
                )
        else:
            raise ImportError(
                "Audio playback requires sounddevice or simpleaudio. "
                "Install with: pip install sounddevice"
            )

    @property
    def device(self) -> Optional[AudioDevice]:
        """Get the current output device."""
        return self._device

    def play(self, data: Union[bytes, np.ndarray, List[int]], sample_rate: int = 16000) -> bool:
        # Convert to numpy array
        if isinstance(data, bytes):
            audio_data = np.frombuffer(data, dtype=np.int16)
        elif isinstance(data, list):
            audio_data = np.array(data, dtype=np.int16)
        elif isinstance(data, np.ndarray):
            audio_data = data.astype(np.int16)
        else:
            raise ValueError(f"Unsupported audio data type: {type(data)}")

        self.stop()

        try:
            if self._use_sounddevice:
                # Use sounddevice for playback (supports device selection)
                audio_float = audio_data.astype(np.float32) / 32768.0
                sd.play(audio_float, sample_rate, device=self._device_index)
            else:
                # Fall back to simpleaudio
                self.play_obj = sa.play_buffer(audio_data.tobytes(), 1, 2, sample_rate)
            return True
        except Exception as e:
            warnings.warn(f"Error playing audio: {e}")
            return False

    def stop(self) -> None:
        if self._use_sounddevice:
            try:
                sd.stop()
            except Exception:
                pass
        elif self.play_obj and self.play_obj.is_playing():
            self.play_obj.stop()
            self.play_obj = None


class ROSAudioBackend(AudioBackend):
    """
    Streams audio to robot's speaker via ROS topic.

    Note: This publishes to /audio_stream topic. The robot must have
    an audio player node subscribed to play the audio.
    """

    def __init__(self, client):
        """
        Args:
            client: roslibpy.Ros instance.
        """
        if roslibpy is None:
            raise ImportError("roslibpy is required for ROSAudioBackend.")

        self.client = client
        self.topic = roslibpy.Topic(client, '/audio_stream', 'std_msgs/msg/Int16MultiArray')

    def play(self, data: Union[bytes, np.ndarray, List[int]], sample_rate: int = 16000) -> bool:
        if not self.client.is_connected:
            return False

        # Convert to List[int] for Int16MultiArray
        if isinstance(data, bytes):
            dt = np.dtype(np.int16).newbyteorder('<')
            int_data = np.frombuffer(data, dtype=dt).tolist()
        elif isinstance(data, np.ndarray):
            int_data = data.astype(np.int16).tolist()
        elif isinstance(data, list):
            int_data = [int(x) for x in data]
        else:
            raise ValueError(f"Unsupported audio data type: {type(data)}")

        msg = {
            "layout": {"dim": [], "data_offset": 0},
            "data": int_data
        }

        try:
            self.topic.publish(roslibpy.Message(msg))
            return True
        except Exception as e:
            warnings.warn(f"Error streaming audio to ROS: {e}")
            return False

    def stop(self) -> None:
        pass


# ==================== INPUT BACKENDS (Recording) ====================


class AudioInputBackend(abc.ABC):
    """Abstract base class for audio input (recording) backends."""

    @abc.abstractmethod
    def start_recording(self, callback: Callable[[np.ndarray], None]) -> bool:
        """
        Start recording audio.

        Args:
            callback: Called with numpy array of int16 samples for each chunk.

        Returns:
            True if recording started successfully.
        """
        pass

    @abc.abstractmethod
    def stop_recording(self) -> None:
        """Stop recording audio."""
        pass

    @property
    @abc.abstractmethod
    def is_recording(self) -> bool:
        """Check if currently recording."""
        pass


class NoOpAudioInputBackend(AudioInputBackend):
    """Fallback input backend that does nothing."""

    def start_recording(self, callback: Callable[[np.ndarray], None]) -> bool:
        return False

    def stop_recording(self) -> None:
        pass

    @property
    def is_recording(self) -> bool:
        return False


class SystemAudioInputBackend(AudioInputBackend):
    """
    Records audio from local microphone using sounddevice.

    Supports device selection for recording from specific microphones.
    """

    def __init__(
        self,
        device: Optional[AudioDevice] = None,
        sample_rate: int = DEFAULT_SAMPLE_RATE,
        chunk_size: int = DEFAULT_CHUNK_SIZE,
        channels: int = DEFAULT_CHANNELS,
    ):
        """
        Initialize system audio input backend.

        Args:
            device: Optional AudioDevice to use for input.
                   If None, uses system default microphone.
            sample_rate: Sample rate in Hz (default: 16000).
            chunk_size: Number of samples per callback (default: 1024).
            channels: Number of channels (default: 1 for mono).
        """
        if sd is None:
            raise ImportError(
                "sounddevice is required for microphone recording. "
                "Install with: pip install sounddevice"
            )

        self._device = device
        self._device_index = device.index if device else None
        self._sample_rate = sample_rate
        self._chunk_size = chunk_size
        self._channels = channels
        self._stream: Optional[sd.InputStream] = None
        self._callback: Optional[Callable[[np.ndarray], None]] = None
        self._recording = False

    @property
    def device(self) -> Optional[AudioDevice]:
        """Get the current input device."""
        return self._device

    def _audio_callback(self, indata, frames, time_info, status):
        """Internal callback for sounddevice stream."""
        if status:
            warnings.warn(f"Audio input status: {status}")
        if self._callback is not None:
            # Convert float32 to int16
            int_data = (indata[:, 0] * 32767).astype(np.int16)
            self._callback(int_data)

    def start_recording(self, callback: Callable[[np.ndarray], None]) -> bool:
        if self._recording:
            return False

        self._callback = callback

        try:
            self._stream = sd.InputStream(
                device=self._device_index,
                channels=self._channels,
                samplerate=self._sample_rate,
                blocksize=self._chunk_size,
                dtype=np.float32,
                callback=self._audio_callback,
            )
            self._stream.start()
            self._recording = True
            return True
        except Exception as e:
            warnings.warn(f"Failed to start recording: {e}")
            return False

    def stop_recording(self) -> None:
        if self._stream is not None:
            try:
                self._stream.stop()
                self._stream.close()
            except Exception:
                pass
            self._stream = None
        self._recording = False
        self._callback = None

    @property
    def is_recording(self) -> bool:
        return self._recording


class ROSAudioInputBackend(AudioInputBackend):
    """
    Receives audio from robot's microphone via ROS topic subscription.

    Subscribes to /audio_stream topic to receive audio from robot.
    """

    def __init__(self, client):
        """
        Args:
            client: roslibpy.Ros instance.
        """
        if roslibpy is None:
            raise ImportError("roslibpy is required for ROSAudioInputBackend.")

        self.client = client
        self._topic: Optional[roslibpy.Topic] = None
        self._callback: Optional[Callable[[np.ndarray], None]] = None
        self._recording = False

    def _on_message(self, msg):
        """Handle incoming audio message."""
        if self._callback is not None:
            data = msg.get('data', [])
            if data:
                self._callback(np.array(data, dtype=np.int16))

    def start_recording(self, callback: Callable[[np.ndarray], None]) -> bool:
        if not self.client.is_connected:
            return False

        if self._recording:
            return False

        self._callback = callback
        self._topic = roslibpy.Topic(
            self.client,
            '/audio_stream',
            'std_msgs/msg/Int16MultiArray'
        )
        self._topic.subscribe(self._on_message)
        self._recording = True
        return True

    def stop_recording(self) -> None:
        if self._topic is not None:
            try:
                self._topic.unsubscribe()
            except Exception:
                pass
            self._topic = None
        self._recording = False
        self._callback = None

    @property
    def is_recording(self) -> bool:
        return self._recording


# ==================== CONVENIENCE CLASSES ====================


class AudioStreamReceiver:
    """
    Receives and manages audio streams from any source.

    This class provides a convenient way to receive audio, buffer it,
    and optionally play it back through local speakers.

    Can be used with:
    - robot.subscribe_audio_stream() for robot microphone
    - SystemAudioInputBackend for local microphone

    Audio Format:
        - Sample rate: 16000 Hz
        - Channels: 1 (Mono)
        - Bit depth: 16-bit signed integers

    Example:
        >>> from pib3 import Robot, AudioStreamReceiver
        >>> with Robot(host="...") as robot:
        ...     # Create receiver with local playback
        ...     receiver = AudioStreamReceiver(playback=True)
        ...     sub = robot.subscribe_audio_stream(receiver.on_audio)
        ...     time.sleep(10)  # Record for 10 seconds
        ...     sub.unsubscribe()
        ...     # Get recorded audio
        ...     audio_data = receiver.get_audio()
        ...     receiver.save_wav("recording.wav")
    """

    def __init__(
        self,
        sample_rate: int = DEFAULT_SAMPLE_RATE,
        playback: bool = False,
        playback_device: Optional[AudioDevice] = None,
        max_buffer_seconds: float = 300.0,
    ):
        """
        Initialize audio stream receiver.

        Args:
            sample_rate: Expected sample rate in Hz (default: 16000).
            playback: If True, play audio through speakers as it arrives.
            playback_device: Optional AudioDevice for playback output.
                            If None, uses system default.
            max_buffer_seconds: Maximum audio to buffer (default: 300s = 5 minutes).
        """
        self.sample_rate = sample_rate
        self.playback = playback
        self.max_samples = int(max_buffer_seconds * sample_rate)

        self._buffer: List[int] = []
        self._buffer_lock = threading.Lock()
        self._play_backend: Optional[SystemAudioBackend] = None

        if playback:
            try:
                self._play_backend = SystemAudioBackend(device=playback_device)
            except ImportError:
                warnings.warn(
                    "Audio playback library not available, playback disabled. "
                    "Install with: pip install sounddevice"
                )
                self.playback = False

    def on_audio(self, samples: Union[List[int], np.ndarray]) -> None:
        """
        Callback for audio data.

        This method is designed to be passed directly to
        robot.subscribe_audio_stream() or used with AudioInputBackend.

        Args:
            samples: List or array of int16 audio samples.
        """
        if isinstance(samples, np.ndarray):
            samples_list = samples.tolist()
        else:
            samples_list = samples

        with self._buffer_lock:
            self._buffer.extend(samples_list)
            # Trim if exceeds max buffer
            if len(self._buffer) > self.max_samples:
                excess = len(self._buffer) - self.max_samples
                self._buffer = self._buffer[excess:]

        # Play if enabled
        if self.playback and self._play_backend:
            self._play_backend.play(samples, self.sample_rate)

    def get_audio(self) -> np.ndarray:
        """
        Get all buffered audio as numpy array.

        Returns:
            NumPy array of int16 samples.
        """
        with self._buffer_lock:
            return np.array(self._buffer, dtype=np.int16)

    def get_audio_float(self) -> np.ndarray:
        """
        Get all buffered audio as normalized float32 array.

        Useful for audio processing that expects -1.0 to 1.0 range.

        Returns:
            NumPy array of float32 samples in range [-1.0, 1.0].
        """
        int_audio = self.get_audio()
        return int_audio.astype(np.float32) / 32768.0

    def clear(self) -> None:
        """Clear the audio buffer."""
        with self._buffer_lock:
            self._buffer.clear()

    @property
    def duration(self) -> float:
        """Get duration of buffered audio in seconds."""
        with self._buffer_lock:
            return len(self._buffer) / self.sample_rate

    @property
    def sample_count(self) -> int:
        """Get number of samples in buffer."""
        with self._buffer_lock:
            return len(self._buffer)

    def save_wav(self, filepath: str) -> None:
        """
        Save buffered audio to a WAV file.

        Args:
            filepath: Path to output WAV file.

        Example:
            >>> receiver.save_wav("recording.wav")
        """
        import wave

        audio_data = self.get_audio()

        with wave.open(filepath, 'wb') as wav_file:
            wav_file.setnchannels(DEFAULT_CHANNELS)
            wav_file.setsampwidth(DEFAULT_SAMPLE_WIDTH)
            wav_file.setframerate(self.sample_rate)
            wav_file.writeframes(audio_data.tobytes())
