"""Robot control backends for pib3 package."""

from .base import RobotBackend
from .webots import WebotsBackend
from .robot import RealRobotBackend, rle_decode
from .audio import (
    # Unified audio system
    AudioOutput,
    LocalAudioPlayer,
    RobotAudioPlayer,
    PiperTTS,
    AudioStreamReceiver,
    load_audio_file,
    resample_audio,
    DEFAULT_SAMPLE_RATE,
)

__all__ = [
    "RobotBackend",
    "WebotsBackend",
    "RealRobotBackend",
    "rle_decode",
    # Unified audio system
    "AudioOutput",
    "LocalAudioPlayer",
    "RobotAudioPlayer",
    "PiperTTS",
    "AudioStreamReceiver",
    "load_audio_file",
    "resample_audio",
    "DEFAULT_SAMPLE_RATE",
]
