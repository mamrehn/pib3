"""Core data types for pib3 package."""

from dataclasses import dataclass, field
from types import MappingProxyType
from typing import List, Optional, Tuple
import numpy as np
from enum import Enum


class ImuType(str, Enum):
    """Types of IMU data streams available."""
    FULL = "full"
    ACCELEROMETER = "accelerometer"
    GYROSCOPE = "gyroscope"


class AIModel(str, Enum):
    """AI models the pib camera node can load on the OAK-D Lite.

    These mirror the ``AVAILABLE_MODELS`` registry in the backend's
    ``ros_packages/camera/oak_d_lite/stereo.py``. That registry is a hard
    allowlist: the ``switch_ai_model`` service rejects any other name, and the
    ``camera/ai/config`` topic logs an error and ignores it. Adding a model
    therefore requires a backend change, not just a new entry here.

    The weights themselves are pulled from the Luxonis Model Hub on demand
    (``dai.NNModelDescription(slug)``) and cached on the robot, so a name in
    this enum may still take a few seconds to load the first time.

    Use the enum rather than a bare string for IDE completion:
        >>> robot.set_ai_model(AIModel.HAND)
        >>> robot.set_ai_model(AIModel.YOLOV6N)

    Strings still work:
        >>> robot.set_ai_model("hand")  # Also valid
    """

    # Object detection
    YOLOV6N = "yolov6n"          # luxonis/yolov6-nano:r2-coco-512x288, 80 COCO classes
    YOLOV10N = "yolov10n"        # luxonis/yolov10-nano:coco-512x288, 80 COCO classes
    PERSON = "person"            # luxonis/scrfd-person-detection:25g-640x640
    FACE = "face"                # luxonis/yunet:640x480

    # Pose estimation (17 keypoints)
    POSE_YOLO = "pose_yolo"      # luxonis/yolov8-nano-pose-estimation:coco-512x288
    POSE_HRNET = "pose_hrnet"    # luxonis/lite-hrnet:18-coco-288x384

    # Hand tracking
    HAND = "hand"                # luxonis/mediapipe-hand-landmarker:224x224

    # Instance segmentation
    SEGMENTATION = "segmentation"  # luxonis/yolov8-instance-segmentation-nano:coco-512x288

    # Gaze estimation (slow on RVC2: ~4 inf/s)
    GAZE = "gaze"                # luxonis/l2cs-net:448x448

    # Line detection
    LINES = "lines"              # luxonis/m-lsd:512x512


#: Model names this SDK used to expose that the backend never accepted, mapped
#: to the closest model it does accept. ``set_ai_model`` remaps these and emits
#: a DeprecationWarning instead of failing with an opaque timeout.
#:
#: Some of these (``mobilenet-ssd``, ``deeplabv3`` -> ``deeplab-v3-plus``,
#: ``fastsam`` -> ``fastsam-s``) do exist on the Luxonis Model Hub; they are
#: simply absent from the backend registry. Recovering them means adding a slug
#: to ``AVAILABLE_MODELS`` on the robot.
DEPRECATED_MODEL_ALIASES = MappingProxyType({
    "mobilenet-ssd": "yolov6n",
    "yolov8n": "yolov6n",
    "yolo11n": "yolov6n",
    "yolo11s": "yolov10n",
    "pose": "pose_yolo",
    "deeplabv3": "segmentation",
    "yolov8n-seg": "segmentation",
    "fastsam": "segmentation",
})


class Joint(str, Enum):
    """PIB robot joint names for IDE tab completion.

    Use these enum values instead of strings for better IDE support:
        >>> robot.set_joint(Joint.ELBOW_LEFT, 50.0)
        >>> robot.get_joint(Joint.SHOULDER_VERTICAL_RIGHT)

    String values still work for backward compatibility:
        >>> robot.set_joint("elbow_left", 50.0)  # Also valid
    """

    # Head
    TURN_HEAD = "turn_head_motor"
    TILT_HEAD = "tilt_forward_motor"

    # Left arm
    SHOULDER_VERTICAL_LEFT = "shoulder_vertical_left"
    SHOULDER_HORIZONTAL_LEFT = "shoulder_horizontal_left"
    UPPER_ARM_LEFT_ROTATION = "upper_arm_left_rotation"
    ELBOW_LEFT = "elbow_left"
    LOWER_ARM_LEFT_ROTATION = "lower_arm_left_rotation"
    WRIST_LEFT = "wrist_left"

    # Left hand
    THUMB_LEFT_OPPOSITION = "thumb_left_opposition"
    THUMB_LEFT_STRETCH = "thumb_left_stretch"
    INDEX_LEFT = "index_left_stretch"
    MIDDLE_LEFT = "middle_left_stretch"
    RING_LEFT = "ring_left_stretch"
    PINKY_LEFT = "pinky_left_stretch"

    # Right arm
    SHOULDER_VERTICAL_RIGHT = "shoulder_vertical_right"
    SHOULDER_HORIZONTAL_RIGHT = "shoulder_horizontal_right"
    UPPER_ARM_RIGHT_ROTATION = "upper_arm_right_rotation"
    ELBOW_RIGHT = "elbow_right"
    LOWER_ARM_RIGHT_ROTATION = "lower_arm_right_rotation"
    WRIST_RIGHT = "wrist_right"

    # Right hand
    THUMB_RIGHT_OPPOSITION = "thumb_right_opposition"
    THUMB_RIGHT_STRETCH = "thumb_right_stretch"
    INDEX_RIGHT = "index_right_stretch"
    MIDDLE_RIGHT = "middle_right_stretch"
    RING_RIGHT = "ring_right_stretch"
    PINKY_RIGHT = "pinky_right_stretch"


# Joint groups for convenience
LEFT_HAND_JOINTS: List[Joint] = [
    Joint.THUMB_LEFT_OPPOSITION,
    Joint.THUMB_LEFT_STRETCH,
    Joint.INDEX_LEFT,
    Joint.MIDDLE_LEFT,
    Joint.RING_LEFT,
    Joint.PINKY_LEFT,
]

RIGHT_HAND_JOINTS: List[Joint] = [
    Joint.THUMB_RIGHT_OPPOSITION,
    Joint.THUMB_RIGHT_STRETCH,
    Joint.INDEX_RIGHT,
    Joint.MIDDLE_RIGHT,
    Joint.RING_RIGHT,
    Joint.PINKY_RIGHT,
]


class HandPose(Enum):
    """Hand pose presets (values in percent).

    Physical mapping: 0% = bent/closed, 100% = stretched/open

    Use with robot.set_joints_pose():
        >>> robot.set_joints_pose(HandPose.LEFT_OPEN)
        >>> robot.set_joints_pose(HandPose.RIGHT_CLOSED)

    For partial grip, use the joint lists:
        >>> robot.set_joints({j: 50.0 for j in LEFT_HAND_JOINTS})  # 50% = half grip
    """

    # Values are wrapped in MappingProxyType so callers can't mutate the
    # shared pose dict and corrupt every future set_joints_pose(HandPose.X) call.
    LEFT_OPEN = MappingProxyType({
        "thumb_left_opposition": 100.0,
        "thumb_left_stretch": 100.0,
        "index_left_stretch": 100.0,
        "middle_left_stretch": 100.0,
        "ring_left_stretch": 100.0,
        "pinky_left_stretch": 100.0,
    })

    LEFT_CLOSED = MappingProxyType({
        "thumb_left_opposition": 0.0,
        "thumb_left_stretch": 0.0,
        "index_left_stretch": 0.0,
        "middle_left_stretch": 0.0,
        "ring_left_stretch": 0.0,
        "pinky_left_stretch": 0.0,
    })

    RIGHT_OPEN = MappingProxyType({
        "thumb_right_opposition": 100.0,
        "thumb_right_stretch": 100.0,
        "index_right_stretch": 100.0,
        "middle_right_stretch": 100.0,
        "ring_right_stretch": 100.0,
        "pinky_right_stretch": 100.0,
    })

    RIGHT_CLOSED = MappingProxyType({
        "thumb_right_opposition": 0.0,
        "thumb_right_stretch": 0.0,
        "index_right_stretch": 0.0,
        "middle_right_stretch": 0.0,
        "ring_right_stretch": 0.0,
        "pinky_right_stretch": 0.0,
    })


@dataclass
class Stroke:
    """A single continuous drawing stroke (pen-down motion).

    Represents a sequence of 2D points that form a continuous line
    drawn without lifting the pen.

    Attributes:
        points: Array of shape (N, 2) with normalized [0,1] coordinates.
                (0,0) = top-left of drawing area, (1,1) = bottom-right.
        closed: If True, the stroke forms a closed loop (first and last
                points should be connected).
    """
    points: np.ndarray
    closed: bool = False

    def __post_init__(self):
        """Ensure points is a numpy array with correct shape (N, 2)."""
        self.points = np.asarray(self.points, dtype=np.float64)
        if self.points.ndim == 1:
            if self.points.shape[0] % 2 != 0:
                raise ValueError(
                    f"1D points array must have an even number of elements "
                    f"to reshape to (N, 2), got {self.points.shape[0]}"
                )
            self.points = self.points.reshape(-1, 2)
        if self.points.ndim != 2 or self.points.shape[1] != 2:
            raise ValueError(
                f"Points must have shape (N, 2), got {self.points.shape}"
            )

    def __repr__(self) -> str:
        closed_str = ", closed" if self.closed else ""
        return f"Stroke({len(self.points)} points, length={self.length():.3f}{closed_str})"

    def __len__(self) -> int:
        """Return number of points in the stroke."""
        return len(self.points)

    def length(self) -> float:
        """Calculate total arc length of the stroke."""
        if len(self.points) < 2:
            return 0.0
        diffs = np.diff(self.points, axis=0)
        distances = np.linalg.norm(diffs, axis=1)
        return float(np.sum(distances))

    def reverse(self) -> "Stroke":
        """Return a new stroke with reversed point order."""
        return Stroke(points=self.points[::-1].copy(), closed=self.closed)

    def start(self) -> np.ndarray:
        """Return the starting point of the stroke."""
        return self.points[0]

    def end(self) -> np.ndarray:
        """Return the ending point of the stroke."""
        return self.points[-1]


@dataclass
class Sketch:
    """A collection of strokes extracted from an image.

    Represents a complete drawing as a list of individual strokes,
    typically in optimized drawing order to minimize pen-up travel.

    Attributes:
        strokes: List of Stroke objects in drawing order.
        source_size: Original image dimensions (width, height) in pixels.
                     None if not created from an image.
    """
    strokes: List[Stroke] = field(default_factory=list)
    source_size: Optional[Tuple[int, int]] = None

    def __repr__(self) -> str:
        size_str = f", source={self.source_size[0]}x{self.source_size[1]}" if self.source_size else ""
        return f"Sketch({len(self.strokes)} strokes, {self.total_points()} points{size_str})"

    def __len__(self) -> int:
        """Return number of strokes in the sketch."""
        return len(self.strokes)

    def __iter__(self):
        """Iterate over strokes."""
        return iter(self.strokes)

    def __getitem__(self, idx) -> Stroke:
        """Get stroke by index."""
        return self.strokes[idx]

    def total_points(self) -> int:
        """Return total number of points across all strokes."""
        return sum(len(s) for s in self.strokes)

    def total_length(self) -> float:
        """Return total arc length of all strokes."""
        return sum(s.length() for s in self.strokes)

    def bounds(self) -> Tuple[float, float, float, float]:
        """Return bounding box (min_u, min_v, max_u, max_v) of all strokes.

        Raises:
            ValueError: If the sketch is empty (no strokes or no points).
                Callers should check ``total_points() > 0`` first, or handle
                the exception — returning a dummy ``(0, 0, 1, 1)`` silently
                has caused downstream scaling bugs.
        """
        if not self.strokes:
            raise ValueError("Cannot compute bounds of an empty sketch (no strokes)")
        all_points = np.vstack([s.points for s in self.strokes])
        if all_points.size == 0:
            raise ValueError("Cannot compute bounds of a sketch with no points")
        min_coords = all_points.min(axis=0)
        max_coords = all_points.max(axis=0)
        return (
            float(min_coords[0]),
            float(min_coords[1]),
            float(max_coords[0]),
            float(max_coords[1]),
        )

    def add_stroke(self, stroke: Stroke) -> None:
        """Add a stroke to the sketch."""
        self.strokes.append(stroke)

    def to_dict(self) -> dict:
        """Convert sketch to a JSON-serializable dictionary."""
        return {
            "strokes": [
                {
                    "points": s.points.tolist(),
                    "closed": s.closed,
                }
                for s in self.strokes
            ],
            "source_size": self.source_size,
        }

    @classmethod
    def from_dict(cls, data: dict) -> "Sketch":
        """Create a Sketch from a dictionary (e.g., loaded from JSON)."""
        strokes = [
            Stroke(
                points=np.array(s["points"]),
                closed=s.get("closed", False),
            )
            for s in data.get("strokes", [])
        ]
        return cls(
            strokes=strokes,
            source_size=tuple(data["source_size"]) if data.get("source_size") else None,
        )
