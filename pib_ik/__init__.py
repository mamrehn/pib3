"""
pib_ik - Inverse kinematics and trajectory generation for the PIB robot.

This package provides tools to:
1. Convert images to 2D drawing strokes
2. Generate robot trajectories using inverse kinematics
3. Execute trajectories on Webots simulation, Swift visualization, or real robot

Quick Start:
    >>> import pib_ik
    >>>
    >>> # One-shot: image to trajectory
    >>> trajectory = pib_ik.generate_trajectory("drawing.png")
    >>> trajectory.to_json("output.json")
    >>>
    >>> # Or step-by-step:
    >>> sketch = pib_ik.image_to_sketch("drawing.png")
    >>> trajectory = pib_ik.sketch_to_trajectory(sketch)
    >>>
    >>> # Execute on backends:
    >>> with pib_ik.Swift() as viz:
    ...     viz.run_trajectory(trajectory)
    >>>
    >>> with pib_ik.Robot(host="172.26.34.149") as robot:
    ...     robot.run_trajectory(trajectory)
"""

from pathlib import Path
from typing import Optional, Union

# Core types
from .types import Sketch, Stroke

# Configuration
from .config import (
    ImageConfig,
    IKConfig,
    PaperConfig,
    RobotConfig,
    TrajectoryConfig,
)

# Core functions
from .image import image_to_sketch
from .trajectory import Trajectory, sketch_to_trajectory

# Backends
from .backends import (
    RobotBackend,
    WebotsBackend,
    RealRobotBackend,
    SwiftBackend,
)

# Convenience aliases
Webots = WebotsBackend
Robot = RealRobotBackend
Swift = SwiftBackend

# Hand poses
from .hand_poses import (
    LEFT_HAND_OPEN,
    LEFT_HAND_CLOSED,
    LEFT_HAND_PENCIL_GRIP,
    LEFT_HAND_JOINTS,
    RIGHT_HAND_OPEN,
    RIGHT_HAND_CLOSED,
    RIGHT_HAND_PENCIL_GRIP,
    RIGHT_HAND_JOINTS,
    left_hand_pose,
    right_hand_pose,
)

__version__ = "0.1.1"

__all__ = [
    # Version
    "__version__",
    # Types
    "Stroke",
    "Sketch",
    "Trajectory",
    # Config
    "ImageConfig",
    "IKConfig",
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
    "SwiftBackend",
    # Aliases
    "Webots",
    "Robot",
    "Swift",
    # Hand poses
    "LEFT_HAND_OPEN",
    "LEFT_HAND_CLOSED",
    "LEFT_HAND_PENCIL_GRIP",
    "LEFT_HAND_JOINTS",
    "RIGHT_HAND_OPEN",
    "RIGHT_HAND_CLOSED",
    "RIGHT_HAND_PENCIL_GRIP",
    "RIGHT_HAND_JOINTS",
    "left_hand_pose",
    "right_hand_pose",
]


def generate_trajectory(
    image: Union[str, Path, "np.ndarray", "PIL.Image.Image"],
    output_path: Optional[Union[str, Path]] = None,
    config: Optional[TrajectoryConfig] = None,
    visualize: bool = False,
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
        visualize: Show Swift visualization during IK solving.
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
        >>> import pib_ik
        >>> trajectory = pib_ik.generate_trajectory("my_drawing.png")
        >>> trajectory.to_json("output.json")

        >>> # With custom config
        >>> from pib_ik import TrajectoryConfig, PaperConfig
        >>> config = TrajectoryConfig(
        ...     paper=PaperConfig(size=0.20, drawing_scale=0.9),
        ... )
        >>> trajectory = pib_ik.generate_trajectory("drawing.png", config=config)

        >>> # Sequential trajectories (robot draws multiple images)
        >>> traj1 = pib_ik.generate_trajectory("image1.png")
        >>> traj2 = pib_ik.generate_trajectory("image2.png", initial_q=traj1)
        >>> traj3 = pib_ik.generate_trajectory("image3.png", initial_q=traj2)
    """
    if config is None:
        config = TrajectoryConfig()

    # Step 1: Image to sketch
    sketch = image_to_sketch(image, config.image)

    # Step 2: Sketch to trajectory
    trajectory = sketch_to_trajectory(
        sketch, config, visualize=visualize, initial_q=initial_q
    )

    # Optional: Save to file
    if output_path is not None:
        trajectory.to_json(output_path)

    return trajectory
