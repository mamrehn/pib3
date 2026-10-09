"""
Camera and AI subsystem for PIB robot.

This module provides typed dataclasses and helpers for consuming camera and AI
streams from the robot. It does NOT provide direct DepthAI access - that runs
on the robot itself via the camera ROS node.

Key Components:
- BoundingBox: Normalized bounding box with utility methods
- Detection: Object detection result with label, confidence, bbox
- HandLandmarks: Hand tracking with 21 landmarks and finger angles
- PoseKeypoints: Body pose with 17 COCO keypoints
- CameraFrame: Frame data with timestamp
- AIModelInfo: Model metadata from the robot's model store
- CameraFrameReceiver: Frame buffering helper
- AIDetectionReceiver: Detection buffering with FPS tracking
- AISubsystem: ``robot.ai`` — start/stop models, typed results

The robot's results are ``datatypes/DetectionArray`` messages (pixels, named
keypoints and scalars); see :mod:`pib3.backends.detection_messages`.

Hand Landmark Indices (MediaPipe convention):
    0: WRIST
    1-4: THUMB (CMC, MCP, IP, TIP)
    5-8: INDEX (MCP, PIP, DIP, TIP)
    9-12: MIDDLE (MCP, PIP, DIP, TIP)
    13-16: RING (MCP, PIP, DIP, TIP)
    17-20: PINKY (MCP, PIP, DIP, TIP)

Pose Keypoint Indices (COCO convention):
    0: nose, 1: left_eye, 2: right_eye, 3: left_ear, 4: right_ear
    5: left_shoulder, 6: right_shoulder, 7: left_elbow, 8: right_elbow
    9: left_wrist, 10: right_wrist, 11: left_hip, 12: right_hip
    13: left_knee, 14: right_knee, 15: left_ankle, 16: right_ankle

Usage:
    >>> from pib3.backends.camera import AIDetectionReceiver, Detection
    >>>
    >>> receiver = AIDetectionReceiver()
    >>> sub = robot.subscribe_ai_detections(AIModel.YOLO26S, receiver.on_detection)
    >>> time.sleep(5)
    >>> sub.unsubscribe()
    >>>
    >>> # Get typed detections
    >>> for det in receiver.get_detections():
    ...     print(f"{det.label}: {det.confidence:.2f} at {det.bbox}")
    >>>
    >>> # Check performance
    >>> print(f"FPS: {receiver.fps:.1f}, Latency: {receiver.avg_latency_ms:.1f}ms")
"""

import logging
import math
import threading
import time
from dataclasses import dataclass, field
from enum import Enum
from collections import deque
from typing import Callable, Deque, Dict, List, Optional, Tuple, Union, TYPE_CHECKING

import numpy as np

from .detection_messages import (
    COCO_KEYPOINT_NAMES,
    HAND_KEYPOINT_NAMES,
    detection_topic,
)

if TYPE_CHECKING:
    from .robot import RealRobotBackend
    from ..types import AIModel

logger = logging.getLogger(__name__)


# ==================== CONSTANTS ====================

# COCO class labels (80 classes)
COCO_LABELS = [
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck",
    "boat", "traffic light", "fire hydrant", "stop sign", "parking meter", "bench",
    "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra",
    "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
    "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove",
    "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup",
    "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange",
    "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch",
    "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse",
    "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink",
    "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier",
    "toothbrush"
]

# Hand landmark indices (MediaPipe convention)
HAND_WRIST = 0
HAND_THUMB_CMC = 1
HAND_THUMB_MCP = 2
HAND_THUMB_IP = 3
HAND_THUMB_TIP = 4
HAND_INDEX_MCP = 5
HAND_INDEX_PIP = 6
HAND_INDEX_DIP = 7
HAND_INDEX_TIP = 8
HAND_MIDDLE_MCP = 9
HAND_MIDDLE_PIP = 10
HAND_MIDDLE_DIP = 11
HAND_MIDDLE_TIP = 12
HAND_RING_MCP = 13
HAND_RING_PIP = 14
HAND_RING_DIP = 15
HAND_RING_TIP = 16
HAND_PINKY_MCP = 17
HAND_PINKY_PIP = 18
HAND_PINKY_DIP = 19
HAND_PINKY_TIP = 20

# Pose keypoint indices (COCO convention)
POSE_NOSE = 0
POSE_LEFT_EYE = 1
POSE_RIGHT_EYE = 2
POSE_LEFT_EAR = 3
POSE_RIGHT_EAR = 4
POSE_LEFT_SHOULDER = 5
POSE_RIGHT_SHOULDER = 6
POSE_LEFT_ELBOW = 7
POSE_RIGHT_ELBOW = 8
POSE_LEFT_WRIST = 9
POSE_RIGHT_WRIST = 10
POSE_LEFT_HIP = 11
POSE_RIGHT_HIP = 12
POSE_LEFT_KNEE = 13
POSE_RIGHT_KNEE = 14
POSE_LEFT_ANKLE = 15
POSE_RIGHT_ANKLE = 16


# ==================== ENUMS ====================


class Handedness(str, Enum):
    """Hand classification."""
    LEFT = "left"
    RIGHT = "right"
    UNKNOWN = "unknown"


# ==================== DATACLASSES ====================


@dataclass
class BoundingBox:
    """
    Normalized bounding box in [0, 1] coordinates.

    Attributes:
        xmin: Left edge (0 = left of image)
        ymin: Top edge (0 = top of image)
        xmax: Right edge (1 = right of image)
        ymax: Bottom edge (1 = bottom of image)
    """
    xmin: float
    ymin: float
    xmax: float
    ymax: float

    @property
    def width(self) -> float:
        """Box width in normalized coordinates."""
        return self.xmax - self.xmin

    @property
    def height(self) -> float:
        """Box height in normalized coordinates."""
        return self.ymax - self.ymin

    @property
    def center(self) -> Tuple[float, float]:
        """Center point (x, y) in normalized coordinates."""
        return ((self.xmin + self.xmax) / 2, (self.ymin + self.ymax) / 2)

    @property
    def area(self) -> float:
        """Box area in normalized coordinates (0 to 1)."""
        return self.width * self.height

    def to_pixels(self, img_width: int, img_height: int) -> Tuple[int, int, int, int]:
        """
        Convert to pixel coordinates.

        Returns:
            Tuple of (x1, y1, x2, y2) in pixels.
        """
        return (
            int(self.xmin * img_width),
            int(self.ymin * img_height),
            int(self.xmax * img_width),
            int(self.ymax * img_height),
        )

    def __repr__(self) -> str:
        cx, cy = self.center
        return f"BoundingBox(center=({cx:.2f}, {cy:.2f}), size=({self.width:.2f}x{self.height:.2f}))"


@dataclass
class Detection:
    """
    Single detection result: a box, and for some models more.

    Attributes:
        label_id: Numeric COCO class id, or -1 for labels outside COCO
        confidence: Detection confidence (0.0 to 1.0)
        bbox: Bounding box in normalized coordinates
        label: Class name, for example ``"person"`` or ``"hand"``
        mask_rle: Optional RLE-encoded segmentation mask (simulation only)
        keypoints: Named landmarks the model attaches, in normalized
            coordinates: 17 for body pose, 21 for a hand, 68 or 468 for faces
        scalars: Named values the model attaches: ``yaw``/``pitch``/``roll``
            for head pose, one probability per emotion, ``handedness`` for a
            hand, and so on
    """
    label_id: int
    confidence: float
    bbox: BoundingBox
    label: str = ""
    mask_rle: Optional[Dict] = None
    keypoints: List["Keypoint"] = field(default_factory=list)
    scalars: Dict[str, float] = field(default_factory=dict)

    def __post_init__(self):
        """Auto-resolve label from COCO classes if not provided."""
        if not self.label and 0 <= self.label_id < len(COCO_LABELS):
            self.label = COCO_LABELS[self.label_id]

    @classmethod
    def from_message(cls, data: dict, frame_width: int, frame_height: int) -> "Detection":
        """Create from one ``Detection`` of a ``DetectionArray`` message.

        The robot sends pixels; this normalizes them by the frame size the
        message carries.
        """
        label = str(data.get("label", ""))
        names = list(data.get("keypoint_names", []))
        xs = list(data.get("keypoint_x", []))
        scores = list(data.get("keypoint_score") or [])
        if len(scores) != len(xs):   # none reported (or an older robot): 1.0
            scores = [1.0] * len(xs)
        keypoints = [
            Keypoint(
                x=float(x) / frame_width,
                y=float(y) / frame_height,
                confidence=float(scores[i]),
                name=names[i] if i < len(names) else "",
            )
            for i, (x, y) in enumerate(zip(xs, data.get("keypoint_y", [])))
        ]
        scalars = dict(zip(data.get("scalar_names", []), data.get("scalar_values", [])))
        return cls(
            label_id=COCO_LABELS.index(label) if label in COCO_LABELS else -1,
            confidence=float(data.get("score", 0.0)),
            bbox=BoundingBox(
                xmin=_unit(data.get("x_min", 0) / frame_width),
                ymin=_unit(data.get("y_min", 0) / frame_height),
                xmax=_unit(data.get("x_max", 0) / frame_width),
                ymax=_unit(data.get("y_max", 0) / frame_height),
            ),
            label=label,
            mask_rle=data.get("mask_rle"),
            keypoints=keypoints,
            scalars={str(k): float(v) for k, v in scalars.items()},
        )


def _unit(value: float) -> float:
    """Clamp a normalized coordinate into [0, 1]."""
    return min(1.0, max(0.0, float(value)))


@dataclass
class Keypoint:
    """Single keypoint with position and confidence.

    ``confidence`` is the model's 0..1 for this point: a pose model also
    places keypoints it cannot see (a wrist behind the back), with a low
    value. It is 1.0 when the model reports none (hand landmarks, or a robot
    whose backend predates ``keypoint_score``).
    """
    x: float  # Normalized [0, 1]
    y: float  # Normalized [0, 1]
    confidence: float = 1.0
    name: str = ""  # e.g. "left_shoulder", "index_finger_tip"; "" if unnamed

    def to_pixels(self, img_width: int, img_height: int) -> Tuple[int, int]:
        """Convert to pixel coordinates."""
        return (int(self.x * img_width), int(self.y * img_height))


@dataclass
class FingerAngles:
    """
    Finger bend angles in degrees.

    Calculated from hand landmarks similar to HandTrackerEdge.py.
    Angle of 0° = fully extended, ~90° = bent.

    Attributes:
        thumb_bend: Thumb flexion angle
        thumb_rotation: Thumb rotation relative to palm plane
        index: Index finger bend angle
        middle: Middle finger bend angle
        ring: Ring finger bend angle
        pinky: Pinky finger bend angle
    """
    thumb_bend: float = 0.0
    thumb_rotation: float = 0.0
    index: float = 0.0
    middle: float = 0.0
    ring: float = 0.0
    pinky: float = 0.0

    def to_list(self) -> List[float]:
        """Return angles as list [thumb_bend, thumb_rot, index, middle, ring, pinky]."""
        return [
            self.thumb_bend, self.thumb_rotation,
            self.index, self.middle, self.ring, self.pinky
        ]

    def to_servo_values(self, scale: float = 100.0) -> Dict[str, float]:
        """
        Convert finger angles to servo percentage values.

        Args:
            scale: Maximum servo value (default 100 for percentage)

        Returns:
            Dict mapping finger names to servo values (0 = bent, scale = extended)
        """
        # Invert: 0° extension = 100%, 90° bent = 0%
        def angle_to_pct(angle: float) -> float:
            return max(0, min(scale, scale * (1 - angle / 90.0)))

        return {
            "thumb_opposition": angle_to_pct(self.thumb_rotation),
            "thumb_stretch": angle_to_pct(self.thumb_bend),
            "index": angle_to_pct(self.index),
            "middle": angle_to_pct(self.middle),
            "ring": angle_to_pct(self.ring),
            "pinky": angle_to_pct(self.pinky),
        }


@dataclass
class HandLandmarks:
    """
    Hand tracking result with 21 landmarks.

    Attributes:
        landmarks: Array of shape (21, 2) with normalized [0,1] coordinates
        keypoints: List of 21 Keypoint objects with confidence
        handedness: Left or right hand classification
        confidence: Overall detection confidence
        wrist_position: (x, y) position of wrist in normalized coords
        finger_angles: Calculated finger bend angles
        aspect_ratio: Image width / height the normalized landmarks come
            from. Finger angles are measured in pixel space, so a 16:9 frame
            does not skew them.
    """
    landmarks: np.ndarray  # Shape (21, 2) or (21, 3)
    keypoints: List[Keypoint] = field(default_factory=list)
    handedness: Handedness = Handedness.UNKNOWN
    confidence: float = 1.0
    finger_angles: Optional[FingerAngles] = None
    aspect_ratio: float = 1.0

    def __post_init__(self):
        """Calculate finger angles if not provided."""
        if self.finger_angles is None and len(self.landmarks) >= 21:
            self.finger_angles = self._calculate_finger_angles()

    @property
    def wrist_position(self) -> Tuple[float, float]:
        """Get wrist (landmark 0) position."""
        if len(self.landmarks) > 0:
            return (float(self.landmarks[0][0]), float(self.landmarks[0][1]))
        return (0.0, 0.0)

    def _calculate_finger_angles(self) -> FingerAngles:
        """
        Calculate finger angles from landmarks.

        Algorithm inspired by HandTrackerEdge.py - calculates the angle
        between upper and lower segments of each finger.

        Returns angles where 0° = fully extended, ~90° = bent.
        """
        if len(self.landmarks) < 21:
            return FingerAngles()

        # Normalized x spans the width, y the height; scale x so that both
        # axes have the same unit and an angle is the angle seen in the image.
        lm = np.asarray(self.landmarks)[:, :2] * np.array([self.aspect_ratio, 1.0])

        def angle_between_vectors(v1: np.ndarray, v2: np.ndarray) -> float:
            """Calculate angle in degrees between two vectors."""
            v1_norm = np.linalg.norm(v1)
            v2_norm = np.linalg.norm(v2)
            if v1_norm < 1e-6 or v2_norm < 1e-6:
                return 0.0
            cos_angle = np.dot(v1, v2) / (v1_norm * v2_norm)
            cos_angle = np.clip(cos_angle, -1.0, 1.0)
            return math.degrees(math.acos(cos_angle))

        def bend_angle(mcp: int, pip: int, dip: int, tip: int) -> float:
            """
            Calculate finger bend angle in degrees (0° = straight, 180° = curled).

            Vectors run along the proximal and distal phalanx in the same
            nominal direction (MCP->PIP and DIP->TIP). A straight finger
            makes them parallel (~0°); a fully curled finger makes them
            anti-parallel (~180°).
            """
            v1 = lm[pip][:2] - lm[mcp][:2]
            v2 = lm[tip][:2] - lm[dip][:2]
            return angle_between_vectors(v1, v2)

        try:
            # Thumb bend: angle at IP joint
            thumb_bend = bend_angle(
                HAND_THUMB_CMC, HAND_THUMB_MCP, HAND_THUMB_IP, HAND_THUMB_TIP
            )

            # Thumb rotation relative to palm (2-3 vs 0-9)
            vec_thumb_rot = lm[HAND_THUMB_IP][:2] - lm[HAND_THUMB_MCP][:2]
            vec_palm = lm[HAND_MIDDLE_MCP][:2] - lm[HAND_WRIST][:2]
            thumb_rotation = angle_between_vectors(vec_thumb_rot, vec_palm)

            # Finger bend angles
            index_angle = bend_angle(
                HAND_INDEX_MCP, HAND_INDEX_PIP, HAND_INDEX_DIP, HAND_INDEX_TIP
            )
            middle_angle = bend_angle(
                HAND_MIDDLE_MCP, HAND_MIDDLE_PIP, HAND_MIDDLE_DIP, HAND_MIDDLE_TIP
            )
            ring_angle = bend_angle(
                HAND_RING_MCP, HAND_RING_PIP, HAND_RING_DIP, HAND_RING_TIP
            )
            pinky_angle = bend_angle(
                HAND_PINKY_MCP, HAND_PINKY_PIP, HAND_PINKY_DIP, HAND_PINKY_TIP
            )

            return FingerAngles(
                thumb_bend=thumb_bend,
                thumb_rotation=thumb_rotation,
                index=index_angle,
                middle=middle_angle,
                ring=ring_angle,
                pinky=pinky_angle,
            )
        except Exception as e:
            logger.debug(f"Error calculating finger angles: {e}")
            return FingerAngles()

    @classmethod
    def from_keypoints_list(
        cls,
        keypoints: List[dict],
        handedness: Union[float, int, str, dict, "Handedness", None] = None,
    ) -> "HandLandmarks":
        """
        Create HandLandmarks from robot's keypoints list.

        Args:
            keypoints: List of {"x": float, "y": float, "confidence": float} dicts.
            handedness: Hand classification as published by the robot. Accepts a
                float/int score (MediaPipe/depthai convention: ``> 0.5`` = right,
                ``< 0.5`` = left), a ``"left"``/``"right"`` string, a dict with a
                ``"label"`` and/or ``"score"``/``"value"`` key, an existing
                ``Handedness``, or ``None`` (→ ``UNKNOWN``).
        """
        landmarks = np.array([[kp["x"], kp["y"]] for kp in keypoints])
        kp_objects = [
            Keypoint(x=kp["x"], y=kp["y"], confidence=kp.get("confidence", 1.0))
            for kp in keypoints
        ]

        return cls(
            landmarks=landmarks,
            keypoints=kp_objects,
            handedness=cls._normalize_handedness(handedness),
        )

    @classmethod
    def from_message(cls, data: dict, frame_width: int, frame_height: int) -> "HandLandmarks":
        """Create from one hand ``Detection`` of a ``DetectionArray`` message.

        Uses the ``handedness`` scalar (``> 0.5`` = right, MediaPipe's label
        from the image's point of view) and ``landmark_score`` when present.
        """
        detection = Detection.from_message(data, frame_width, frame_height)
        scalars = detection.scalars
        return cls(
            landmarks=np.array([[kp.x, kp.y] for kp in detection.keypoints]),
            keypoints=detection.keypoints,
            handedness=cls._normalize_handedness(scalars.get("handedness")),
            confidence=scalars.get("landmark_score", detection.confidence),
            aspect_ratio=frame_width / frame_height,
        )

    @staticmethod
    def _normalize_handedness(value) -> Handedness:
        """Normalize a robot-provided handedness value to a Handedness enum.

        Handles the shapes the camera node may emit: a float/int score
        (``> 0.5`` = right, per MediaPipe/depthai), a ``"left"``/``"right"``
        string, a dict with ``"label"`` and/or ``"score"``/``"value"``, an
        existing ``Handedness``, or ``None``.
        """
        if value is None:
            return Handedness.UNKNOWN
        if isinstance(value, Handedness):
            return value
        if isinstance(value, str):
            v = value.strip().lower()
            if v in ("left", "l"):
                return Handedness.LEFT
            if v in ("right", "r"):
                return Handedness.RIGHT
            return Handedness.UNKNOWN
        if isinstance(value, dict):
            if value.get("label") is not None:
                return HandLandmarks._normalize_handedness(value["label"])
            for key in ("value", "score", "handedness", "probability"):
                num = value.get(key)
                if isinstance(num, (int, float)) and not isinstance(num, bool):
                    return HandLandmarks._normalize_handedness(float(num))
            return Handedness.UNKNOWN
        if isinstance(value, bool):
            # bool is an int subclass; treat True = right explicitly.
            return Handedness.RIGHT if value else Handedness.LEFT
        if isinstance(value, (int, float)):
            f = float(value)
            if f > 0.5:
                return Handedness.RIGHT
            if f < 0.5:
                return Handedness.LEFT
            return Handedness.UNKNOWN
        return Handedness.UNKNOWN


@dataclass
class PoseKeypoints:
    """
    Body pose estimation result with 17 COCO keypoints.

    Attributes:
        keypoints: List of 17 Keypoint objects
        confidence: Overall pose confidence
        bbox: Optional bounding box around the person
    """
    keypoints: List[Keypoint]
    confidence: float = 1.0
    bbox: Optional[BoundingBox] = None

    @classmethod
    def from_keypoints_list(cls, keypoints: List[dict], bbox: Optional[dict] = None) -> "PoseKeypoints":
        """Create from robot's keypoints list."""
        kp_objects = [
            Keypoint(x=kp["x"], y=kp["y"], confidence=kp.get("confidence", 1.0))
            for kp in keypoints
        ]

        bbox_obj = None
        if bbox:
            bbox_obj = BoundingBox(
                xmin=bbox.get("xmin", 0),
                ymin=bbox.get("ymin", 0),
                xmax=bbox.get("xmax", 0),
                ymax=bbox.get("ymax", 0),
            )

        return cls(keypoints=kp_objects, bbox=bbox_obj)

    @classmethod
    def from_message(cls, data: dict, frame_width: int, frame_height: int) -> "PoseKeypoints":
        """Create from one person ``Detection`` of a ``DetectionArray`` message.

        Keypoints are ordered by COCO index whatever order the message lists
        them in; names the model does not report stay out of the list.
        """
        detection = Detection.from_message(data, frame_width, frame_height)
        by_name = {kp.name: kp for kp in detection.keypoints if kp.name}
        if all(name in by_name for name in COCO_KEYPOINT_NAMES):
            keypoints = [by_name[name] for name in COCO_KEYPOINT_NAMES]
        else:
            keypoints = detection.keypoints
        return cls(
            keypoints=keypoints,
            confidence=detection.confidence,
            bbox=detection.bbox,
        )

    def get_keypoint(self, index: int) -> Optional[Keypoint]:
        """Get keypoint by COCO index (0-16)."""
        if 0 <= index < len(self.keypoints):
            return self.keypoints[index]
        return None

    @property
    def nose(self) -> Optional[Keypoint]:
        return self.get_keypoint(POSE_NOSE)

    @property
    def left_shoulder(self) -> Optional[Keypoint]:
        return self.get_keypoint(POSE_LEFT_SHOULDER)

    @property
    def right_shoulder(self) -> Optional[Keypoint]:
        return self.get_keypoint(POSE_RIGHT_SHOULDER)


@dataclass
class AIModelInfo:
    """
    A model in the robot's model store, as ``/list_models`` reports it.

    Attributes:
        name: Model id (e.g., "yolo26s_coco_512x288"), the value of the
            matching :class:`~pib3.types.AIModel`
        task: What the model does ("object_detection", "pose_estimation",
            "hand_tracking", "face_detection", "facial_landmarks", ...)
        licence: Licence of the weights; may be "unknown - see source"
        shaves: Camera cores the model uses; the camera has 16 in total
        size_bytes: Size of the compiled model
        available: Whether the model's file is in the store
        active: Whether the model is running now
    """
    name: str
    task: str = ""
    licence: str = ""
    shaves: int = 0
    size_bytes: int = 0
    available: bool = False
    active: bool = False

    @classmethod
    def from_dict(cls, name: str, data: dict) -> "AIModelInfo":
        """Create from one entry of :meth:`RealRobotBackend.get_available_ai_models`."""
        return cls(
            name=name,
            task=data.get("task", ""),
            licence=data.get("licence", ""),
            shaves=int(data.get("shaves", 0)),
            size_bytes=int(data.get("size_bytes", 0)),
            available=bool(data.get("available", False)),
            active=bool(data.get("active", False)),
        )


@dataclass
class CameraFrame:
    """
    Single camera frame with metadata.

    A frame carries either encoded JPEG bytes (the real robot streams MJPEG)
    or an already-decoded BGR array (the Webots camera hands over raw pixels).
    ``to_numpy()`` works either way, so consumer code does not care which
    backend produced the frame.

    Attributes:
        jpeg_bytes: Raw JPEG image data. Empty for frames built by
            :meth:`from_numpy` — use :meth:`to_jpeg` if you need encoded bytes.
        frame_id: Sequential frame number
        timestamp_ns: Timestamp in nanoseconds
        timestamp: Timestamp as float seconds
    """
    jpeg_bytes: bytes
    frame_id: int = 0
    timestamp_ns: int = 0
    # Pre-decoded BGR image, set when the source already had raw pixels.
    # Excluded from repr/eq so frames stay cheap to print and compare.
    array: Optional[np.ndarray] = field(default=None, repr=False, compare=False)

    @classmethod
    def from_numpy(
        cls,
        bgr: np.ndarray,
        frame_id: int = 0,
        timestamp_ns: int = 0,
    ) -> "CameraFrame":
        """
        Build a frame from an already-decoded BGR array.

        Used by backends whose camera is not an MJPEG stream (e.g. Webots),
        so no encode/decode round-trip is needed. ``jpeg_bytes`` stays empty.

        Args:
            bgr: HxWx3 image in BGR channel order (OpenCV convention).
            frame_id: Sequential frame number.
            timestamp_ns: Timestamp in nanoseconds.
        """
        return cls(
            jpeg_bytes=b"",
            frame_id=frame_id,
            timestamp_ns=timestamp_ns,
            array=bgr,
        )

    @property
    def timestamp(self) -> float:
        """Get timestamp as seconds."""
        return self.timestamp_ns / 1e9

    def to_numpy(self) -> Optional[np.ndarray]:
        """
        Get the frame as a BGR numpy array.

        Returns the pre-decoded array if the frame carries one, otherwise
        decodes ``jpeg_bytes`` (requires cv2).

        Returns:
            BGR image as numpy array, or None if decoding fails.
        """
        if self.array is not None:
            return self.array
        try:
            import cv2
            nparr = np.frombuffer(self.jpeg_bytes, np.uint8)
            return cv2.imdecode(nparr, cv2.IMREAD_COLOR)
        except ImportError:
            logger.warning("OpenCV not installed. Cannot decode JPEG.")
            return None
        except Exception as e:
            logger.warning(f"Failed to decode JPEG: {e}")
            return None

    def to_jpeg(self, quality: int = 90) -> Optional[bytes]:
        """
        Get the frame as JPEG bytes, encoding on demand if necessary.

        Args:
            quality: JPEG quality 1-100, used only when encoding is needed.

        Returns:
            JPEG bytes, or None if encoding fails.
        """
        if self.jpeg_bytes:
            return self.jpeg_bytes
        if self.array is None:
            return None
        try:
            import cv2
            ok, buf = cv2.imencode(
                ".jpg", self.array, [int(cv2.IMWRITE_JPEG_QUALITY), quality]
            )
            return buf.tobytes() if ok else None
        except ImportError:
            logger.warning("OpenCV not installed. Cannot encode JPEG.")
            return None
        except Exception as e:
            logger.warning(f"Failed to encode JPEG: {e}")
            return None


# ==================== RECEIVER HELPERS ====================


class CameraFrameReceiver:
    """
    Buffers camera frames for processing.

    Similar to AudioStreamReceiver, this class buffers incoming frames
    and provides easy access to the latest frame or all buffered frames.

    Example:
        >>> receiver = CameraFrameReceiver(max_buffer=30)
        >>> sub = robot.subscribe_camera_image(receiver.on_frame)
        >>> time.sleep(5)
        >>> sub.unsubscribe()
        >>>
        >>> frame = receiver.get_latest()
        >>> if frame:
        ...     print(f"Got frame {frame.frame_id}")
    """

    def __init__(self, max_buffer: int = 30):
        """
        Initialize frame receiver.

        Args:
            max_buffer: Maximum number of frames to buffer.
        """
        self.max_buffer = max_buffer
        self._frames: Deque[CameraFrame] = deque(maxlen=max(1, int(max_buffer)))
        self._lock = threading.Lock()
        self._frame_count = 0

    def on_frame(self, jpeg_bytes: bytes) -> None:
        """
        Callback for incoming frame data.

        Pass this method to robot.subscribe_camera_image().
        """
        with self._lock:
            self._frame_count += 1
            frame = CameraFrame(
                jpeg_bytes=jpeg_bytes,
                frame_id=self._frame_count,
                timestamp_ns=time.time_ns(),
            )
            self._frames.append(frame)

    def get_latest(self) -> Optional[CameraFrame]:
        """Get the most recent frame, or None if no frames received."""
        with self._lock:
            if self._frames:
                return self._frames[-1]
            return None

    def get_all(self) -> List[CameraFrame]:
        """Get all buffered frames and clear the buffer."""
        with self._lock:
            frames = list(self._frames)
            self._frames.clear()
            return frames

    def clear(self) -> None:
        """Clear the frame buffer."""
        with self._lock:
            self._frames.clear()

    @property
    def frame_count(self) -> int:
        """Total number of frames received."""
        return self._frame_count

    @property
    def buffer_size(self) -> int:
        """Current number of buffered frames."""
        with self._lock:
            return len(self._frames)


class AIDetectionReceiver:
    """
    Buffers a model's detection messages with FPS and latency tracking.

    Feed it the ``datatypes/DetectionArray`` messages of one model (rosbridge
    delivers them as dicts, see :mod:`pib3.backends.detection_messages`); the
    getters turn them into typed Detection/HandLandmarks/PoseKeypoints objects.

    Usage:
        >>> from pib3 import Robot, AIModel, AIDetectionReceiver
        >>> with Robot(host="...") as robot:
        ...     robot.start_ai_model(AIModel.HAND)
        ...     receiver = AIDetectionReceiver()
        ...     sub = robot.subscribe_ai_detections(AIModel.HAND, receiver.on_detection)
        ...
        ...     # Waits automatically for results
        ...     for hand in receiver.get_hand_landmarks():
        ...         print(f"{hand.handedness}: index={hand.finger_angles.index:.0f}°")
        ...         servos = hand.finger_angles.to_servo_values()
        ...         robot.set_joints({"index_left_stretch": servos["index"]})
        ...
        ...     print(f"FPS: {receiver.fps:.1f}")
        ...     sub.unsubscribe()
        ...     robot.stop_ai_model(AIModel.HAND)

    Most code uses ``robot.ai`` instead, which manages receivers itself.
    """

    #: Latency samples outside this many milliseconds are ignored: they say the
    #: robot's clock and this computer's clock are not synchronised.
    LATENCY_PLAUSIBLE_MS = (0.0, 10_000.0)

    def __init__(self, max_buffer: int = 100):
        """
        Initialize detection receiver.

        Args:
            max_buffer: Maximum number of detection messages to buffer.
        """
        self.max_buffer = max_buffer
        self._results: List[dict] = []  # Raw DetectionArray messages
        self._lock = threading.Lock()
        self._new_data = threading.Event()
        self._expected_model: Optional[str] = None

        # FPS tracking
        self._frame_times: List[float] = []
        self._latencies: List[float] = []
        self._fps_window = 30  # Calculate FPS over last N frames

    def expect_model(self, model_id: Optional[str]) -> None:
        """
        Accept only messages of ``model_id`` from now on, and clear the buffers.

        Both hand chains publish on one shared topic, and a message names its
        model in ``model_id``; this keeps one chain's results out of the other's
        receiver. A message without a ``model_id`` is kept. ``None`` accepts
        every model again.
        """
        with self._lock:
            self._expected_model = model_id
        self.clear()

    def on_detection(self, message: dict) -> None:
        """
        Callback for an incoming DetectionArray message.

        Pass this method to ``robot.subscribe_ai_detections()``.
        """
        now = time.time()
        with self._lock:
            expected = self._expected_model
            sender = message.get("model_id")
            if expected is not None and sender is not None and sender != expected:
                return

            self._results.append(message)
            if len(self._results) > self.max_buffer:
                self._results.pop(0)

            self._frame_times.append(now)
            if len(self._frame_times) > self._fps_window:
                self._frame_times.pop(0)

            latency_ms = self._latency_ms(message, now)
            low, high = self.LATENCY_PLAUSIBLE_MS
            if latency_ms is not None and low <= latency_ms <= high:
                self._latencies.append(latency_ms)
                if len(self._latencies) > self._fps_window:
                    self._latencies.pop(0)

        # Signal that new data is available
        self._new_data.set()

    @staticmethod
    def _latency_ms(message: dict, now: float) -> Optional[float]:
        """Milliseconds from the message's creation to now, if it says."""
        if "latency_ms" in message:  # the simulation measures it directly
            return float(message["latency_ms"])
        stamp = message.get("header", {}).get("stamp")
        if not stamp:
            return None
        created = float(stamp.get("sec", 0)) + float(stamp.get("nanosec", 0)) * 1e-9
        return (now - created) * 1000.0 if created > 0 else None

    def _wait_for_data(self, timeout: float) -> None:
        """Wait until data is available or timeout."""
        if timeout <= 0:
            return
        deadline = time.time() + timeout
        while time.time() < deadline:
            with self._lock:
                if self._results:
                    return
            self._new_data.clear()
            remaining = deadline - time.time()
            if remaining > 0:
                self._new_data.wait(timeout=min(0.05, remaining))

    @property
    def fps(self) -> float:
        """Messages per second over the last 30 messages."""
        with self._lock:
            if len(self._frame_times) < 2:
                return 0.0
            duration = self._frame_times[-1] - self._frame_times[0]
            if duration < 0.001:
                return 0.0
            return (len(self._frame_times) - 1) / duration

    @property
    def avg_latency_ms(self) -> float:
        """Average age of a message when it arrives, in milliseconds.

        On the robot this is the time since the camera stamped the message and
        includes the network; it is only meaningful when the robot's clock and
        this computer's are synchronised (NTP). Samples that are negative or
        absurdly large are ignored. In the simulation it is the inference time.
        """
        with self._lock:
            if not self._latencies:
                return 0.0
            return sum(self._latencies) / len(self._latencies)

    def get_latest_raw(self) -> Optional[dict]:
        """Get the most recent raw DetectionArray message."""
        with self._lock:
            if self._results:
                return self._results[-1]
            return None

    def _results_snapshot(self, latest_only: bool) -> List[dict]:
        """Copy the buffered results to iterate outside the lock.

        With ``latest_only`` this is at most one entry: the newest frame.
        """
        with self._lock:
            if not self._results:
                return []
            if latest_only:
                return [self._results[-1]]
            return list(self._results)

    def _detections_of(self, latest_only: bool, keep=None):
        """``(detection dict, frame_width, frame_height)`` for buffered results.

        Messages without a frame size (the robot sends 0 when it has no frame
        yet) cannot be normalized and are skipped.
        """
        for message in self._results_snapshot(latest_only):
            width = int(message.get("frame_width", 0))
            height = int(message.get("frame_height", 0))
            if width <= 0 or height <= 0:
                continue
            for det in message.get("detections", []):
                if keep is None or keep(det):
                    yield det, width, height

    def get_detections(
        self,
        timeout: float = 5.0,
        latest_only: bool = False,
    ) -> List[Detection]:
        """
        Get buffered detections: every box of every buffered message.

        Waits automatically if no results are available yet. Pose, hand and
        face models report their persons, hands and faces here too, with
        ``det.keypoints`` and ``det.scalars`` filled in.

        Warning:
            By default this returns detections from **every buffered frame**
            (up to ``max_buffer``), not from the current one. In a polling
            loop the same physical object therefore appears once per buffered
            frame — counting them would count it many times over. Use
            ``latest_only=True`` for "what does the camera see *right now*",
            which is what control loops want, or call :meth:`clear` after each
            poll if you want to consume every frame exactly once.

        Args:
            timeout: How long to wait for results if buffer is empty.
                     Use timeout=0 for immediate (non-blocking) return.
            latest_only: Return results from the newest frame only.

        Returns:
            List of Detection objects (may be empty if timeout=0 and no data).
        """
        self._wait_for_data(timeout)
        return [
            Detection.from_message(det, width, height)
            for det, width, height in self._detections_of(latest_only)
        ]

    def get_hand_landmarks(
        self,
        timeout: float = 5.0,
        latest_only: bool = False,
    ) -> List[HandLandmarks]:
        """
        Get buffered hand tracking results.

        Waits automatically if no results are available yet.

        Warning:
            Returns every buffered frame by default — see
            :meth:`get_detections` for why that double-counts in a loop.

        Args:
            timeout: How long to wait for results if buffer is empty.
                     Use timeout=0 for immediate (non-blocking) return.
            latest_only: Return results from the newest frame only.

        Returns:
            List of HandLandmarks objects with finger angles.
        """
        self._wait_for_data(timeout)
        return [
            HandLandmarks.from_message(det, width, height)
            for det, width, height in self._detections_of(latest_only, _is_hand)
        ]

    def get_poses(
        self,
        timeout: float = 5.0,
        latest_only: bool = False,
    ) -> List[PoseKeypoints]:
        """
        Get buffered pose estimation results.

        Waits automatically if no results are available yet.

        Warning:
            Returns every buffered frame by default — see
            :meth:`get_detections` for why that double-counts in a loop.

        Args:
            timeout: How long to wait for results if buffer is empty.
                     Use timeout=0 for immediate (non-blocking) return.
            latest_only: Return results from the newest frame only.

        Returns:
            List of PoseKeypoints objects.
        """
        self._wait_for_data(timeout)
        return [
            PoseKeypoints.from_message(det, width, height)
            for det, width, height in self._detections_of(latest_only, _is_pose)
        ]

    def clear(self) -> None:
        """Clear all buffers."""
        with self._lock:
            self._results.clear()
            self._frame_times.clear()
            self._latencies.clear()
        self._new_data.clear()

    @property
    def result_count(self) -> int:
        """Total number of results in buffer."""
        with self._lock:
            return len(self._results)


def _is_hand(det: dict) -> bool:
    """Whether a Detection of a message is a hand with its 21 landmarks."""
    names = det.get("keypoint_names") or []
    if names:
        return list(names) == list(HAND_KEYPOINT_NAMES)
    return det.get("label") == "hand" and len(det.get("keypoint_x", [])) == 21


def _is_pose(det: dict) -> bool:
    """Whether a Detection of a message is a person with the COCO keypoints."""
    names = det.get("keypoint_names") or []
    if names:
        return set(COCO_KEYPOINT_NAMES) <= set(names)
    return det.get("label") == "person" and len(det.get("keypoint_x", [])) == 17


def parse_detection_message(message: dict) -> List[Union[Detection, HandLandmarks, PoseKeypoints]]:
    """
    Turn one DetectionArray message into typed objects.

    A hand becomes :class:`HandLandmarks`, a person with the COCO keypoints
    :class:`PoseKeypoints`, everything else :class:`Detection`.

    Args:
        message: A ``datatypes/DetectionArray`` as a dict.

    Returns:
        One object per detection; empty if the message has no frame size.
    """
    width = int(message.get("frame_width", 0))
    height = int(message.get("frame_height", 0))
    if width <= 0 or height <= 0:
        return []
    typed = []
    for det in message.get("detections", []):
        if _is_hand(det):
            typed.append(HandLandmarks.from_message(det, width, height))
        elif _is_pose(det):
            typed.append(PoseKeypoints.from_message(det, width, height))
        else:
            typed.append(Detection.from_message(det, width, height))
    return typed


# ==================== SUBSYSTEM CLASSES ====================


class AISubsystem:
    """
    AI inference of the robot's OAK-D Lite camera, as ``robot.ai``.

    The camera runs the models of pib-backend's model store. A client asks for
    a model with ``/start_model`` and gives an *owner* name; the model runs as
    long as any owner holds it, so scripts, cerebra and the web programs do
    not switch each other's models away. This class starts and stops models
    under this client's owner name and keeps one receiver per model.

    Starting or stopping a model rebuilds the camera pipeline: video and IMU
    pause for a few seconds, unless another owner already runs the model.
    If a start fails, the camera falls back to colour only, which also stops
    the models that ran before; :meth:`start_model` warns when that may have
    happened.

        >>> robot.ai.set_model(AIModel.HAND)
        >>> for hand in robot.ai.get_hand_landmarks(latest_only=True):
        ...     print(f"{hand.handedness}: {hand.finger_angles.index:.0f}°")
        >>> print(f"FPS: {robot.ai.fps:.1f}")
    """

    def __init__(self, robot: "RealRobotBackend"):
        """
        Initialize AI subsystem.

        Args:
            robot: Parent robot backend instance.
        """
        self._robot = robot
        self._receivers: Dict[str, AIDetectionReceiver] = {}
        self._subscriptions: Dict[str, object] = {}
        self._current_model: Optional[str] = None

    # --- model lifecycle ------------------------------------------------

    def start_model(self, model: "Union[AIModel, str]", timeout: float = 30.0) -> bool:
        """
        Start a model without stopping others, and make it the current one.

        The camera can run several models at once as long as their cores
        (``AIModelInfo.shaves``) fit into the camera's 16.

        Args:
            model: AI model to start (AIModel enum or model id).
            timeout: Max seconds for the robot's answer. The call returns
                after the camera pipeline has been rebuilt and delivers
                frames again.

        Returns:
            True if the model is running; False if the robot refused (the
            reason is logged together with the models it offers).
        """
        model_id = self._robot.resolve_ai_model_name(model)
        # A model this client already holds stays held on the robot whatever
        # happens to this call, so its receiver stays too.
        already_held = model_id in self._receivers
        receiver = self._receivers.get(model_id)
        if receiver is None:
            receiver = self._receivers[model_id] = AIDetectionReceiver()
        receiver.expect_model(model_id)
        self._subscribe(model_id)  # before the start, so no result is missed

        others = [m for m in self._receivers if m != model_id]
        try:
            ok, message = self._robot.start_ai_model(model_id, timeout=timeout)
        except BaseException:      # e.g. the connection dropped: leave no half-started model
            if not already_held:
                self._release(model_id)
            raise
        if not ok:
            logger.warning("%s%s", message, self._offered_models_hint())
            if others:
                logger.warning(
                    "A failed start can drop the camera back to colour only, "
                    "which stops the models started before (%s). Check "
                    "robot.available_models() and start them again if needed.",
                    ", ".join(others),
                )
            if not already_held:
                self._release(model_id)
            return False
        self._current_model = model_id
        return True

    def stop_model(
        self, model: "Union[AIModel, str, None]" = None, timeout: float = 30.0
    ) -> bool:
        """
        Release this client's hold on a model (the current one by default).

        The model stops running once no other client holds it either. A model
        named here is released on the robot even if this script did not start
        it, for example one an earlier, crashed run under the same
        :attr:`~pib3.backends.robot.RealRobotBackend.ai_owner` left running;
        the robot ignores a release for a model this owner does not hold.

        Returns:
            True if the robot confirmed. With no model named and none
            started, there is nothing to stop and the result is True.
        """
        model_id = self._robot.resolve_ai_model_name(model) if model else self._current_model
        if model_id is None:
            return True
        ok, message = self._robot.stop_ai_model(model_id, timeout=timeout)
        if not ok:
            logger.warning("Stopping %s failed: %s", model_id, message)
        self._release(model_id)
        return ok

    def set_model(self, model: "Union[AIModel, str]", timeout: float = 30.0) -> bool:
        """
        Run this model and none of the other models started here.

        Stops the models this client started earlier, then starts ``model``.
        It also releases this owner's hold on any other model the robot runs,
        so a model left running by an earlier run that crashed (same
        :attr:`~pib3.backends.robot.RealRobotBackend.ai_owner`) stops too.
        Models held under other owner names keep running; a second script on
        this computer shares the owner name, so give it its own ``ai_owner``
        if both must run models at once. Each stop and start rebuilds
        the camera pipeline (a few seconds each), so a switch is two
        rebuilds; a model that is already running costs nothing. The call
        returns once the robot reports the new model running. Stopping first
        keeps the two models from needing the camera's cores at the same
        time; use :meth:`start_model` to run several models on purpose.

        Args:
            model: AI model to run (AIModel enum or model id).
            timeout: Max seconds for each of the robot's answers.

        Returns:
            True if the model is running; False if the robot refused.

        Example:
            >>> robot.ai.set_model(AIModel.HAND)
            >>> robot.ai.set_model(AIModel.YOLO26S)
        """
        model_id = self._robot.resolve_ai_model_name(model)
        for other in [m for m in self._receivers if m != model_id]:
            self.stop_model(other, timeout=timeout)
        self._release_stale_holds(keep=model_id, timeout=timeout)
        return self.start_model(model_id, timeout=timeout)

    def _release_stale_holds(self, keep: str, timeout: float) -> None:
        """Release this owner's hold on every other running model.

        The robot keeps a hold until its owner releases it, and a script that
        crashed never did. It does not say who holds a model, so this asks it
        to release each running model under this owner; for a model another
        client holds, that changes nothing and costs no rebuild.
        """
        try:
            running = [
                name for name, info in self._robot.get_available_ai_models().items()
                if info.get("active") and name != keep and name not in self._receivers
            ]
        except Exception as exc:
            logger.debug("Could not list the robot's models: %s", exc)
            return
        for name in running:
            ok, message = self._robot.stop_ai_model(name, timeout=timeout)
            if not ok:
                logger.debug("Releasing %s: %s", name, message)

    def _subscribe(self, model_id: str) -> None:
        if model_id not in self._subscriptions and self._robot.is_connected:
            self._subscriptions[model_id] = self._robot.subscribe_ai_detections(
                model_id, self._receivers[model_id].on_detection
            )

    def _release(self, model_id: str) -> None:
        """Forget a model: unsubscribe and drop its receiver."""
        subscription = self._subscriptions.pop(model_id, None)
        if subscription is not None:
            try:
                subscription.unsubscribe()
            except Exception:
                pass
        self._receivers.pop(model_id, None)
        if self._current_model == model_id:
            self._current_model = next(reversed(self._receivers), None)

    def _offered_models_hint(self) -> str:
        """The models the robot can start, for an error message."""
        try:
            offered = sorted(
                name for name, info in self._robot.get_available_ai_models().items()
                if info.get("available")
            )
        except Exception:
            return ""
        return f". The robot offers: {', '.join(offered)}" if offered else ""

    # --- state ----------------------------------------------------------

    @property
    def model(self) -> Optional[str]:
        """Id of the current model: the one the getters read by default."""
        return self._current_model

    @property
    def models(self) -> Tuple[str, ...]:
        """Ids of all models this client started."""
        return tuple(self._receivers)

    def available_models(self) -> List[AIModelInfo]:
        """The models the robot offers, with their state."""
        return [
            AIModelInfo.from_dict(name, info)
            for name, info in self._robot.get_available_ai_models().items()
        ]

    def _receiver(self, model: "Union[AIModel, str, None]") -> AIDetectionReceiver:
        model_id = self._robot.resolve_ai_model_name(model) if model else self._current_model
        if model_id is None or model_id not in self._receivers:
            raise RuntimeError(
                "No AI model started. Call robot.ai.set_model(AIModel.YOLO26S) "
                "(or another model) first."
            )
        return self._receivers[model_id]

    @property
    def fps(self) -> float:
        """Results per second of the current model."""
        receiver = self._receivers.get(self._current_model)
        return receiver.fps if receiver else 0.0

    @property
    def avg_latency_ms(self) -> float:
        """Average age of the current model's results on arrival (see
        :attr:`AIDetectionReceiver.avg_latency_ms` for its limits)."""
        receiver = self._receivers.get(self._current_model)
        return receiver.avg_latency_ms if receiver else 0.0

    # --- results --------------------------------------------------------

    def get_detections(
        self,
        timeout: float = 5.0,
        latest_only: bool = False,
        model: "Union[AIModel, str, None]" = None,
    ) -> List[Detection]:
        """
        Get detections of a model (the current one by default).

        Waits automatically for results if buffer is empty.

        Warning:
            Defaults to **every buffered frame**, not the current one — in a
            polling loop each object reappears once per buffered frame. Pass
            ``latest_only=True`` for "what is in front of the camera right
            now", which is what control loops and counters want.

        Args:
            timeout: How long to wait for results. Use 0 for non-blocking.
            latest_only: Return results from the newest frame only.
            model: Read this model instead of the current one.

        Returns:
            List of Detection objects.

        Raises:
            RuntimeError: if no model has been started.

        Example:
            >>> robot.ai.set_model(AIModel.YOLO26S)
            >>> while True:  # control loop: only ever the current frame
            ...     for det in robot.ai.get_detections(timeout=0, latest_only=True):
            ...         print(f"{det.label}: {det.confidence:.0%}")
        """
        return self._receiver(model).get_detections(timeout, latest_only=latest_only)

    def get_hand_landmarks(
        self,
        timeout: float = 5.0,
        latest_only: bool = False,
        model: "Union[AIModel, str, None]" = None,
    ) -> List[HandLandmarks]:
        """
        Get hand tracking results with finger angles.

        Waits automatically for results if buffer is empty.

        Warning:
            Defaults to every buffered frame — see :meth:`get_detections`.

        Args:
            timeout: How long to wait for results. Use 0 for non-blocking.
            latest_only: Return results from the newest frame only.
            model: Read this model instead of the current one.

        Returns:
            List of HandLandmarks objects with finger angles; empty while no
            hand is in view.

        Example:
            >>> robot.ai.set_model(AIModel.HAND)
            >>> for hand in robot.ai.get_hand_landmarks(latest_only=True):
            ...     print(f"{hand.handedness}: index={hand.finger_angles.index:.0f}°")
            ...     servos = hand.finger_angles.to_servo_values()
            ...     robot.set_joints({"index_left_stretch": servos["index"]})
        """
        return self._receiver(model).get_hand_landmarks(timeout, latest_only=latest_only)

    def get_poses(
        self,
        timeout: float = 5.0,
        latest_only: bool = False,
        model: "Union[AIModel, str, None]" = None,
    ) -> List[PoseKeypoints]:
        """
        Get body pose estimation results.

        Waits automatically for results if buffer is empty.

        Warning:
            Defaults to every buffered frame — see :meth:`get_detections`.

        Args:
            timeout: How long to wait for results. Use 0 for non-blocking.
            latest_only: Return results from the newest frame only.
            model: Read this model instead of the current one.

        Returns:
            List of PoseKeypoints objects.
        """
        return self._receiver(model).get_poses(timeout, latest_only=latest_only)

    def clear(self) -> None:
        """Clear buffered results of every started model."""
        for receiver in self._receivers.values():
            receiver.clear()

    def stop(self) -> None:
        """Release every model this client started and unsubscribe.

        Called on disconnect. A model that other clients hold keeps running.
        """
        for model_id in list(self._receivers):
            try:
                self.stop_model(model_id, timeout=10.0)
            except Exception as exc:  # the connection may already be gone
                logger.debug("Releasing %s failed: %s", model_id, exc)
                self._release(model_id)


class CameraSubsystem:
    """
    RGB camera subsystem for the robot's OAK-D Lite camera.

    Provides access to raw camera frames (separate from AI inference results).

    Accessed via `robot.camera`:
        >>> frame = robot.camera.get_frame()
        >>> if frame:
        ...     img = frame.to_numpy()  # Requires OpenCV
    """

    def __init__(self, robot: "RealRobotBackend"):
        """
        Initialize camera subsystem.

        Args:
            robot: Parent robot backend instance.
        """
        self._robot = robot
        self._receiver = CameraFrameReceiver()
        self._subscription = None
        self._depth_subscription = None

    def _ensure_subscribed(self) -> None:
        """Ensure we're subscribed to camera frames."""
        if self._subscription is None and self._robot.is_connected:
            self._subscription = self._robot.subscribe_camera_image(
                self._receiver.on_frame
            )

    def get_frame(self, timeout: float = 5.0) -> Optional[CameraFrame]:
        """
        Get the latest camera frame.

        Args:
            timeout: How long to wait for a frame if none available.

        Returns:
            CameraFrame object or None if timeout.
        """
        self._ensure_subscribed()
        deadline = time.time() + timeout
        while time.time() < deadline:
            frame = self._receiver.get_latest()
            if frame:
                return frame
            time.sleep(0.05)
        return self._receiver.get_latest()

    def get_frames(self) -> List[CameraFrame]:
        """Get all buffered frames and clear the buffer."""
        self._ensure_subscribed()
        return self._receiver.get_all()

    @property
    def frame_count(self) -> int:
        """Total number of frames received."""
        return self._receiver.frame_count

    def configure(
        self,
        fps: Optional[int] = None,
        quality: Optional[int] = None,
        resolution: Optional[tuple] = None,
    ) -> None:
        """
        Configure camera settings.

        Args:
            fps: Frames per second (e.g., 30).
            quality: JPEG quality 1-100 (e.g., 80).
            resolution: (width, height) tuple (e.g., (1280, 720)).
        """
        self._robot.set_camera_config(fps, quality, resolution)

    def get_depth_frame(self, timeout: float = 5.0) -> Optional["np.ndarray"]:
        """
        Current metric depth frame as a uint16 array of millimetres.

        0 marks an invalid or unknown pixel. Returns None when the robot has
        no depth right now: depth runs while no AI model does, so stop the
        models (``robot.ai.stop()``) first if you get None.
        """
        return self._robot.get_depth_frame(timeout=timeout)

    def get_distance_at_px(
        self,
        x: int,
        y: int,
        timeout: float = 5.0,
    ) -> Optional[float]:
        """
        Distance in millimetres at one pixel, or None if unavailable.

        Combines naturally with detections:
            >>> for det in robot.ai.get_detections():
            ...     cx, cy = det.bbox.center
            ...     mm = robot.camera.get_distance_at_px(int(cx * w), int(cy * h))
        """
        return self._robot.get_distance_at_px(x, y, timeout=timeout)

    def start_depth_stream(self) -> None:
        """
        Have the camera publish its colourised depth preview.

        The depth the services read exists whenever no AI model runs; only the
        colourised preview needs a subscriber. This holds a subscription open
        and discards the frames; use
        :meth:`RealRobotBackend.subscribe_depth_visualization` directly if you
        want to display them.
        """
        if self._depth_subscription is None:
            self._depth_subscription = self._robot.subscribe_depth_visualization(
                lambda _jpeg: None
            )

    def stop_depth_stream(self) -> None:
        """Release the depth subscription started by :meth:`start_depth_stream`."""
        if self._depth_subscription is not None:
            try:
                self._depth_subscription.unsubscribe()
            except Exception:
                pass
            self._depth_subscription = None

    def stop(self) -> None:
        """Stop camera streaming."""
        self.stop_depth_stream()
        if self._subscription is not None:
            try:
                self._subscription.unsubscribe()
            except Exception:
                pass
            self._subscription = None
