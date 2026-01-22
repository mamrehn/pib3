"""
Audio subsystem for PIB robot.

This module provides unified audio playback and TTS capabilities that work
across both the real robot and Webots simulation.

Key Components:
- AudioOutput: Enum for specifying playback destination (LOCAL, ROBOT, LOCAL_AND_ROBOT)
- PiperTTS: Text-to-speech synthesis using Piper with Thorsten German voice
- LocalAudioPlayer: Cross-platform local audio playback using simpleaudio
- RobotAudioPlayer: Send audio to robot via /audio_playback ROS topic

Audio Format (standard throughout):
- Sample rate: 16000 Hz (16kHz)
- Channels: 1 (Mono)
- Bit depth: 16-bit signed integers (int16)

Platform-specific requirements:
- Linux: sudo apt-get install libasound2-dev  (before pip install pib3)
- macOS: No additional requirements
- Windows: May need Microsoft Visual C++ Build Tools

Usage:
    >>> with Robot(host="...") as robot:
    ...     # Play audio on robot speaker
    ...     robot.play_audio(audio_data, output=AudioOutput.ROBOT)
    ...
    ...     # Play on both local and robot
    ...     robot.play_file("sound.wav", output=AudioOutput.LOCAL_AND_ROBOT)
    ...
    ...     # Text-to-speech (German by default)
    ...     robot.speak("Hallo, ich bin pib!")
"""

import io
import logging
import os
import threading
import warnings
import wave
from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import Callable, List, Optional, Union

import numpy as np

logger = logging.getLogger(__name__)

# Platform-specific installation help for simpleaudio
import sys
if sys.platform.startswith('linux'):
    _SIMPLEAUDIO_HELP = (
        "simpleaudio is required for audio playback.\n"
        "On Linux, first install ALSA development libraries:\n"
        "    sudo apt-get install libasound2-dev\n"
        "Then install simpleaudio:\n"
        "    pip install simpleaudio"
    )
elif sys.platform == 'darwin':  # macOS
    _SIMPLEAUDIO_HELP = (
        "simpleaudio is required for audio playback.\n"
        "Install with:\n"
        "    pip install simpleaudio"
    )
elif sys.platform == 'win32':  # Windows
    _SIMPLEAUDIO_HELP = (
        "simpleaudio is required for audio playback.\n"
        "On Windows, you may need Microsoft Visual C++ Build Tools.\n"
        "Download from: https://visualstudio.microsoft.com/visual-cpp-build-tools/\n"
        "Then install simpleaudio:\n"
        "    pip install simpleaudio"
    )
else:
    _SIMPLEAUDIO_HELP = (
        "simpleaudio is required for audio playback.\n"
        "Install with: pip install simpleaudio"
    )

# Audio playback imports
try:
    import simpleaudio as sa
    HAS_SIMPLEAUDIO = True
except ImportError as e:
    sa = None
    HAS_SIMPLEAUDIO = False
    logger.warning(f"simpleaudio not available: {e}\n{_SIMPLEAUDIO_HELP}")

try:
    import roslibpy
    HAS_ROSLIBPY = True
except ImportError:
    roslibpy = None
    HAS_ROSLIBPY = False


# ==================== CONSTANTS ====================

DEFAULT_SAMPLE_RATE = 16000
DEFAULT_CHANNELS = 1
DEFAULT_SAMPLE_WIDTH = 2  # 16-bit = 2 bytes
DEFAULT_CHUNK_SIZE = 1024

# Piper TTS settings
PIPER_MODEL_DIR = Path.home() / ".cache" / "pib3" / "piper_models"
DEFAULT_PIPER_VOICE = "de_DE-thorsten-high"  # German Thorsten voice


# ==================== AUDIO OUTPUT ENUM ====================


class AudioOutput(Enum):
    """
    Destination for audio playback.

    Attributes:
        LOCAL: Play on local machine (laptop) speakers only.
        ROBOT: Play on robot speakers only (via /audio_playback topic).
        LOCAL_AND_ROBOT: Play on both simultaneously (best-effort sync).

    Note:
        In Webots simulation, ROBOT and LOCAL_AND_ROBOT both resolve to
        LOCAL-only playback (no duplication).
    """
    LOCAL = "local"
    ROBOT = "robot"
    LOCAL_AND_ROBOT = "local_and_robot"


# ==================== LOCAL AUDIO PLAYER ====================


class LocalAudioPlayer:
    """
    Cross-platform local audio playback using simpleaudio.

    Plays audio through the local machine's speakers.
    """

    def __init__(self):
        if not HAS_SIMPLEAUDIO:
            raise ImportError(_SIMPLEAUDIO_HELP)
        self._play_obj = None
        self._lock = threading.Lock()

    def play(
        self,
        data: Union[bytes, np.ndarray, List[int]],
        sample_rate: int = DEFAULT_SAMPLE_RATE,
        block: bool = True,
    ) -> bool:
        """
        Play audio data through local speakers.

        Args:
            data: Audio data as bytes, numpy array (int16), or list of int16.
            sample_rate: Sample rate in Hz (default: 16000).
            block: If True, wait for playback to complete.

        Returns:
            True if playback started successfully.
        """
        # Convert to bytes
        if isinstance(data, np.ndarray):
            audio_bytes = data.astype(np.int16).tobytes()
        elif isinstance(data, list):
            audio_bytes = np.array(data, dtype=np.int16).tobytes()
        elif isinstance(data, bytes):
            audio_bytes = data
        else:
            raise ValueError(f"Unsupported audio data type: {type(data)}")

        with self._lock:
            self.stop()
            try:
                self._play_obj = sa.play_buffer(
                    audio_bytes,
                    num_channels=DEFAULT_CHANNELS,
                    bytes_per_sample=DEFAULT_SAMPLE_WIDTH,
                    sample_rate=sample_rate,
                )
                if block:
                    self._play_obj.wait_done()
                return True
            except Exception as e:
                logger.warning(f"Local audio playback failed: {e}")
                return False

    def stop(self) -> None:
        """Stop current playback."""
        if self._play_obj is not None:
            try:
                self._play_obj.stop()
            except Exception:
                pass
            self._play_obj = None

    def is_playing(self) -> bool:
        """Check if audio is currently playing."""
        return self._play_obj is not None and self._play_obj.is_playing()


# ==================== ROBOT AUDIO PLAYER ====================


class RobotAudioPlayer:
    """
    Send audio to robot for playback via /audio_playback ROS topic.

    The robot's audio_player node subscribes to this topic and plays
    received audio through the robot's speakers.
    """

    TOPIC_NAME = "/audio_playback"
    TOPIC_TYPE = "std_msgs/msg/Int16MultiArray"

    def __init__(self, client: "roslibpy.Ros"):
        """
        Initialize robot audio player.

        Args:
            client: Connected roslibpy.Ros instance.
        """
        if not HAS_ROSLIBPY:
            raise ImportError("roslibpy is required for robot audio.")

        self._client = client
        self._topic = roslibpy.Topic(
            client,
            self.TOPIC_NAME,
            self.TOPIC_TYPE,
        )

    def play(
        self,
        data: Union[bytes, np.ndarray, List[int]],
        sample_rate: int = DEFAULT_SAMPLE_RATE,
        block: bool = True,
    ) -> bool:
        """
        Send audio to robot for playback.

        Args:
            data: Audio data as bytes, numpy array (int16), or list of int16.
            sample_rate: Sample rate in Hz (default: 16000).
            block: If True, wait estimated playback duration.

        Returns:
            True if audio was sent successfully.
        """
        if not self._client.is_connected:
            logger.warning("Cannot play on robot: not connected")
            return False

        # Convert to list of int16
        if isinstance(data, bytes):
            int_data = np.frombuffer(data, dtype=np.int16).tolist()
        elif isinstance(data, np.ndarray):
            int_data = data.astype(np.int16).tolist()
        elif isinstance(data, list):
            int_data = [int(x) for x in data]
        else:
            raise ValueError(f"Unsupported audio data type: {type(data)}")

        # Construct Int16MultiArray message
        msg = roslibpy.Message({
            "layout": {"dim": [], "data_offset": 0},
            "data": int_data,
        })

        try:
            self._topic.publish(msg)

            if block:
                # Estimate playback duration and wait
                duration = len(int_data) / sample_rate
                import time
                time.sleep(duration)

            return True
        except Exception as e:
            logger.warning(f"Failed to send audio to robot: {e}")
            return False

    def stop(self) -> None:
        """Stop playback (sends empty message to clear queue)."""
        if self._client.is_connected:
            try:
                msg = roslibpy.Message({
                    "layout": {"dim": [], "data_offset": 0},
                    "data": [],
                })
                self._topic.publish(msg)
            except Exception:
                pass


# ==================== PIPER TTS ====================


class PiperTTS:
    """
    Text-to-speech synthesis using Piper.

    Uses the Thorsten German voice by default. Voice models are automatically
    downloaded on first use.

    Piper is a fast, local neural TTS system that works well on
    resource-constrained devices like Raspberry Pi.
    """

    def __init__(self, voice: str = DEFAULT_PIPER_VOICE):
        """
        Initialize Piper TTS.

        Args:
            voice: Piper voice model name (default: German Thorsten).
        """
        self._voice = voice
        self._piper = None
        self._model_path = None
        self._initialized = False

    def _ensure_initialized(self) -> bool:
        """Ensure Piper is initialized and model is downloaded."""
        if self._initialized:
            return True

        try:
            from piper import PiperVoice
        except ImportError:
            raise ImportError(
                "piper-tts is required for text-to-speech. "
                "Install with: pip install piper-tts"
            )

        # Ensure model directory exists
        PIPER_MODEL_DIR.mkdir(parents=True, exist_ok=True)

        # Download model if needed
        model_path = self._download_model_if_needed()
        if model_path is None:
            return False

        # Load the model
        try:
            self._piper = PiperVoice.load(str(model_path))
            self._model_path = model_path
            self._initialized = True
            logger.info(f"Piper TTS initialized with voice: {self._voice}")
            return True
        except Exception as e:
            logger.error(f"Failed to load Piper model: {e}")
            return False

    def _download_model_if_needed(self) -> Optional[Path]:
        """Download Piper voice model if not already cached."""
        model_file = PIPER_MODEL_DIR / f"{self._voice}.onnx"
        config_file = PIPER_MODEL_DIR / f"{self._voice}.onnx.json"

        if model_file.exists() and config_file.exists():
            return model_file

        logger.info(f"Downloading Piper voice model: {self._voice}")

        try:
            import urllib.request

            # Piper model URLs (from official releases)
            base_url = "https://huggingface.co/rhasspy/piper-voices/resolve/main"

            # Map voice name to path
            voice_parts = self._voice.split("-")
            if len(voice_parts) >= 3:
                lang = voice_parts[0]  # e.g., "de_DE"
                name = voice_parts[1]  # e.g., "thorsten"
                quality = voice_parts[2]  # e.g., "high"
                voice_path = f"{lang}/{name}/{quality}"
            else:
                voice_path = self._voice

            model_url = f"{base_url}/{voice_path}/{self._voice}.onnx"
            config_url = f"{base_url}/{voice_path}/{self._voice}.onnx.json"

            # Download model
            logger.info(f"Downloading: {model_url}")
            urllib.request.urlretrieve(model_url, model_file)

            # Download config
            logger.info(f"Downloading: {config_url}")
            urllib.request.urlretrieve(config_url, config_file)

            logger.info(f"Voice model downloaded to: {PIPER_MODEL_DIR}")
            return model_file

        except Exception as e:
            logger.error(f"Failed to download Piper model: {e}")
            # Clean up partial downloads
            if model_file.exists():
                model_file.unlink()
            if config_file.exists():
                config_file.unlink()
            return None

    def synthesize(
        self,
        text: str,
        sample_rate: int = DEFAULT_SAMPLE_RATE,
    ) -> np.ndarray:
        """
        Synthesize speech from text.

        Args:
            text: Text to synthesize.
            sample_rate: Target sample rate (resampling done if needed).

        Returns:
            Audio data as numpy array of int16 samples.

        Raises:
            ImportError: If piper-tts is not installed.
            RuntimeError: If synthesis fails.
        """
        if not self._ensure_initialized():
            raise RuntimeError("Failed to initialize Piper TTS")

        try:
            # Synthesize to WAV in memory
            wav_buffer = io.BytesIO()

            with wave.open(wav_buffer, "wb") as wav_file:
                self._piper.synthesize(text, wav_file)

            # Read back the audio data
            wav_buffer.seek(0)
            with wave.open(wav_buffer, "rb") as wav_file:
                frames = wav_file.readframes(wav_file.getnframes())
                audio = np.frombuffer(frames, dtype=np.int16)
                native_rate = wav_file.getframerate()

            # Resample if needed
            if native_rate != sample_rate:
                # Simple linear resampling
                duration = len(audio) / native_rate
                new_length = int(duration * sample_rate)
                indices = np.linspace(0, len(audio) - 1, new_length)
                audio = np.interp(indices, np.arange(len(audio)), audio).astype(np.int16)

            return audio

        except Exception as e:
            raise RuntimeError(f"TTS synthesis failed: {e}") from e

    @property
    def voice(self) -> str:
        """Get the current voice model name."""
        return self._voice


# ==================== AUDIO STREAM RECEIVER ====================


class AudioStreamReceiver:
    """
    Receives and buffers audio from robot's microphone.

    Designed to work with robot.subscribe_audio_stream() callback.

    Example:
        >>> receiver = AudioStreamReceiver()
        >>> sub = robot.subscribe_audio_stream(receiver.on_audio)
        >>> time.sleep(10)
        >>> sub.unsubscribe()
        >>> audio = receiver.get_audio()
        >>> receiver.save_wav("recording.wav")
    """

    def __init__(
        self,
        sample_rate: int = DEFAULT_SAMPLE_RATE,
        max_buffer_seconds: float = 300.0,
    ):
        """
        Initialize audio stream receiver.

        Args:
            sample_rate: Expected sample rate in Hz.
            max_buffer_seconds: Maximum audio to buffer (default: 5 minutes).
        """
        self.sample_rate = sample_rate
        self.max_samples = int(max_buffer_seconds * sample_rate)
        self._buffer: List[int] = []
        self._lock = threading.Lock()

    def on_audio(self, samples: Union[List[int], np.ndarray]) -> None:
        """
        Callback for incoming audio data.

        Pass this method directly to robot.subscribe_audio_stream().

        Args:
            samples: Audio samples (int16).
        """
        if isinstance(samples, np.ndarray):
            samples = samples.tolist()

        with self._lock:
            self._buffer.extend(samples)
            # Trim if exceeds max buffer
            if len(self._buffer) > self.max_samples:
                excess = len(self._buffer) - self.max_samples
                self._buffer = self._buffer[excess:]

    def get_audio(self) -> np.ndarray:
        """Get buffered audio as numpy array of int16."""
        with self._lock:
            return np.array(self._buffer, dtype=np.int16)

    def get_audio_float(self) -> np.ndarray:
        """Get buffered audio as normalized float32 in range [-1, 1]."""
        audio = self.get_audio()
        return audio.astype(np.float32) / 32768.0

    def clear(self) -> None:
        """Clear the audio buffer."""
        with self._lock:
            self._buffer.clear()

    @property
    def duration(self) -> float:
        """Get duration of buffered audio in seconds."""
        with self._lock:
            return len(self._buffer) / self.sample_rate

    @property
    def sample_count(self) -> int:
        """Get number of samples in buffer."""
        with self._lock:
            return len(self._buffer)

    def save_wav(self, filepath: Union[str, Path]) -> None:
        """
        Save buffered audio to WAV file.

        Args:
            filepath: Output file path.
        """
        audio = self.get_audio()
        with wave.open(str(filepath), "wb") as wav_file:
            wav_file.setnchannels(DEFAULT_CHANNELS)
            wav_file.setsampwidth(DEFAULT_SAMPLE_WIDTH)
            wav_file.setframerate(self.sample_rate)
            wav_file.writeframes(audio.tobytes())


# ==================== UTILITY FUNCTIONS ====================


def load_audio_file(filepath: Union[str, Path]) -> tuple[np.ndarray, int]:
    """
    Load audio from a WAV file.

    Args:
        filepath: Path to WAV file.

    Returns:
        Tuple of (audio_data as int16 numpy array, sample_rate).
    """
    with wave.open(str(filepath), "rb") as wav_file:
        sample_rate = wav_file.getframerate()
        n_channels = wav_file.getnchannels()
        sample_width = wav_file.getsampwidth()
        frames = wav_file.readframes(wav_file.getnframes())

        # Convert to numpy
        if sample_width == 1:
            audio = np.frombuffer(frames, dtype=np.uint8).astype(np.int16) * 256 - 32768
        elif sample_width == 2:
            audio = np.frombuffer(frames, dtype=np.int16)
        elif sample_width == 4:
            audio = (np.frombuffer(frames, dtype=np.int32) / 65536).astype(np.int16)
        else:
            raise ValueError(f"Unsupported sample width: {sample_width}")

        # Convert stereo to mono if needed
        if n_channels == 2:
            audio = audio.reshape(-1, 2).mean(axis=1).astype(np.int16)
        elif n_channels > 2:
            audio = audio.reshape(-1, n_channels)[:, 0]  # Take first channel

        return audio, sample_rate


def resample_audio(
    audio: np.ndarray,
    orig_rate: int,
    target_rate: int = DEFAULT_SAMPLE_RATE,
) -> np.ndarray:
    """
    Resample audio to target sample rate.

    Args:
        audio: Audio data as int16 numpy array.
        orig_rate: Original sample rate.
        target_rate: Target sample rate.

    Returns:
        Resampled audio as int16 numpy array.
    """
    if orig_rate == target_rate:
        return audio

    duration = len(audio) / orig_rate
    new_length = int(duration * target_rate)
    indices = np.linspace(0, len(audio) - 1, new_length)
    return np.interp(indices, np.arange(len(audio)), audio).astype(np.int16)
