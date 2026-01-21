"""Robot control backends for pib3 package."""

from .base import RobotBackend
from .webots import WebotsBackend
from .robot import RealRobotBackend, rle_decode
from .audio import (
    # Device types
    AudioDevice,
    AudioDeviceType,
    AudioDeviceManager,
    ROBOT_MICROPHONE,
    ROBOT_SPEAKER,
    # Output backends
    AudioBackend,
    NoOpAudioBackend,
    SystemAudioBackend,
    ROSAudioBackend,
    # Input backends
    AudioInputBackend,
    NoOpAudioInputBackend,
    SystemAudioInputBackend,
    ROSAudioInputBackend,
    # Convenience
    AudioStreamReceiver,
)

__all__ = [
    "RobotBackend",
    "WebotsBackend",
    "RealRobotBackend",
    "rle_decode",
    # Audio device management
    "AudioDevice",
    "AudioDeviceType",
    "AudioDeviceManager",
    "ROBOT_MICROPHONE",
    "ROBOT_SPEAKER",
    # Audio output backends
    "AudioBackend",
    "NoOpAudioBackend",
    "SystemAudioBackend",
    "ROSAudioBackend",
    # Audio input backends
    "AudioInputBackend",
    "NoOpAudioInputBackend",
    "SystemAudioInputBackend",
    "ROSAudioInputBackend",
    # Convenience
    "AudioStreamReceiver",
]
