"""
pib3 - Inverse kinematics and trajectory generation for the PIB robot.

This package provides tools to:
1. Convert images to 2D drawing strokes
2. Generate robot trajectories using inverse kinematics
3. Execute trajectories on Webots simulation or real robot

Quick Start:
    >>> import pib3
    >>>
    >>> # One-shot: image to trajectory
    >>> trajectory = pib3.generate_trajectory("drawing.png")
    >>> trajectory.to_json("output.json")
    >>>
    >>> # Or step-by-step:
    >>> sketch = pib3.image_to_sketch("drawing.png")
    >>> trajectory = pib3.sketch_to_trajectory(sketch)
    >>>

    >>> with pib3.Robot(host="172.26.34.149") as robot:
    ...     robot.run_trajectory(trajectory)
"""

from pathlib import Path
from typing import Optional, Union

# Core types
from .types import Joint, Sketch, Stroke, HandPose, LEFT_HAND_JOINTS, RIGHT_HAND_JOINTS, AIModel

# Configuration
from .config import (
    ImageConfig,
    IKConfig,
    LowLatencyConfig,
    PaperConfig,
    RobotConfig,
    TrajectoryConfig,
)

# Core functions
from .image import image_to_sketch
from .trajectory import Trajectory, sketch_to_trajectory

# DH kinematics (expert API for vision/IK applications)
from .dh_model import (
    CAMERA_TRANSFORM,
    DEFAULT_TOOL_TRANSFORM,
    MOTOR_GROUPS,
    DEFAULT_MOTOR_SETTINGS,
    camera_to_base,
    base_to_camera,
)

# Backends
from .backends import (
    RobotBackend,
    WebotsBackend,
    RealRobotBackend,
    # Low-latency motor control helpers
    build_motor_mapping,
    PIB_SERVO_CHANNELS,
    # Camera/AI types
    Detection,
    HandLandmarks,
    FingerAngles,
    PoseKeypoints,
    BoundingBox,
    CameraFrameReceiver,
    AIDetectionReceiver,
    COCO_LABELS,
    # Unified audio system - enums
    AudioOutput,
    AudioInput,
    # Device management
    AudioDevice,
    list_audio_devices,
    list_audio_input_devices,
    list_audio_output_devices,
    # Recording/playback utilities
    AudioStreamReceiver,
    PiperTTS,
    load_audio_file,
    save_audio_file,
)

# Convenience aliases
Webots = WebotsBackend
Robot = RealRobotBackend


__version__ = "0.1.4"

__all__ = [
    # Version
    "__version__",
    # Types
    "Joint",
    "Stroke",
    "Sketch",
    "Trajectory",
    "AIModel",
    # Config
    "ImageConfig",
    "IKConfig",
    "LowLatencyConfig",
    "PaperConfig",
    "RobotConfig",
    "TrajectoryConfig",
    # Functions
    "image_to_sketch",
    "sketch_to_trajectory",
    "generate_trajectory",
    # Backends
    "RobotBackend",
    "WebotsBackend",
    "RealRobotBackend",
    # Low-latency motor control
    "build_motor_mapping",
    "PIB_SERVO_CHANNELS",
    # Camera/AI types
    "Detection",
    "HandLandmarks",
    "FingerAngles",
    "PoseKeypoints",
    "BoundingBox",
    "CameraFrameReceiver",
    "AIDetectionReceiver",
    "COCO_LABELS",
    # Aliases
    "Webots",
    "Robot",
    # Hand poses
    "HandPose",
    "LEFT_HAND_JOINTS",
    "RIGHT_HAND_JOINTS",
    # Unified audio system - enums
    "AudioOutput",
    "AudioInput",
    # Device management
    "AudioDevice",
    "list_audio_devices",
    "list_audio_input_devices",
    "list_audio_output_devices",
    # Recording/playback utilities
    "AudioStreamReceiver",
    "PiperTTS",
    "load_audio_file",
    "save_audio_file",
    # DH kinematics (expert API)
    "CAMERA_TRANSFORM",
    "DEFAULT_TOOL_TRANSFORM",
    "MOTOR_GROUPS",
    "DEFAULT_MOTOR_SETTINGS",
    "camera_to_base",
    "base_to_camera",
]


def generate_trajectory(
    image: Union[str, Path, "np.ndarray", "PIL.Image.Image"],
    output_path: Optional[Union[str, Path]] = None,
    config: Optional[TrajectoryConfig] = None,
    initial_q: Optional[Union["np.ndarray", dict, Trajectory]] = None,
) -> Trajectory:
    """
    Convenience function: Convert image directly to robot trajectory.

    This is a one-shot function that combines image_to_sketch() and
    sketch_to_trajectory() for simple use cases.

    Args:
        image: Input image (file path, numpy array, or PIL Image).
        output_path: If provided, save trajectory JSON to this path.
        config: Full configuration (uses sensible defaults if None).
        initial_q: Initial joint configuration to start IK solving from.
            Can be one of:
            - numpy array of shape (n_joints,) with joint positions in radians
            - dict mapping joint names to positions in radians
            - Trajectory object (uses last waypoint)
            - None (default): use fixed initial pose for drawing

            This enables sequential trajectory execution where each new
            trajectory starts from the robot's current position.

    Returns:
        Trajectory object ready for execution.

    Example:
        >>> import pib3
        >>> trajectory = pib3.generate_trajectory("my_drawing.png")
        >>> trajectory.to_json("output.json")

        >>> # With custom config
        >>> from pib3 import TrajectoryConfig, PaperConfig
        >>> config = TrajectoryConfig(
        ...     paper=PaperConfig(size=0.20, drawing_scale=0.9),
        ... )
        >>> trajectory = pib3.generate_trajectory("drawing.png", config=config)

        >>> # Sequential trajectories (robot draws multiple images)
        >>> traj1 = pib3.generate_trajectory("image1.png")
        >>> traj2 = pib3.generate_trajectory("image2.png", initial_q=traj1)
        >>> traj3 = pib3.generate_trajectory("image3.png", initial_q=traj2)
    """
    if config is None:
        config = TrajectoryConfig()

    # Step 1: Image to sketch
    sketch = image_to_sketch(image, config.image)

    # Step 2: Sketch to trajectory
    trajectory = sketch_to_trajectory(
        sketch, config, initial_q=initial_q
    )

    # Optional: Save to file
    if output_path is not None:
        trajectory.to_json(output_path)

    return trajectory
