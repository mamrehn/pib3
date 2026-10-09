"""Simulated AI inference: OAK-D-shaped results from ordinary RGB frames.

The real robot runs its models on the OAK-D Lite's own accelerator and
publishes ``datatypes/DetectionArray`` messages (pixel boxes, named keypoints
and scalars; see :mod:`pib3.backends.detection_messages`). Webots gives us only
RGB pixels — so this module runs equivalent (or newer) models on the host
CPU/GPU and emits **the same message shape**.

Because the shape is identical, simulated results flow through the same
:class:`~pib3.backends.camera.AIDetectionReceiver` and come out as the same
typed ``Detection`` / ``HandLandmarks`` / ``PoseKeypoints`` objects. Code
written against ``robot.ai`` runs unchanged against ``sim.ai``.

The topologies match by construction, which is why this works at all:

===================  ==============================  ========================
pib3 type            Convention                      Simulated with
===================  ==============================  ========================
``PoseKeypoints``    17 COCO keypoints               ultralytics ``*-pose``
``HandLandmarks``    21 MediaPipe hand landmarks     ``mediapipe`` Hands
``Detection``        box + class name                ultralytics detect/seg
===================  ==============================  ========================

Both backends are optional dependencies, imported lazily::

    pip install ultralytics      # detection, pose, segmentation
    pip install mediapipe        # hand landmarks

The robot's other models (faces, emotion, head pose, QR codes) have no
simulated equivalent; ``set_model`` says so and ``"recognition"`` (Webots
ground truth) needs no model at all.

.. note::
   Running MediaPipe *here* is not a contradiction of the course guidance to
   avoid it on the Raspberry Pi. On the Pi it duplicated work the OAK-D does
   in silicon; on a laptop it is the closest available stand-in for exactly
   that silicon.
"""

import logging
from typing import Any, Dict, List

import numpy as np

from ..types import AIModel, resolve_model_name
from .detection_messages import (
    COCO_KEYPOINT_NAMES,
    HAND_KEYPOINT_NAMES,
    make_detection,
)

logger = logging.getLogger(__name__)


# ==================== RLE ENCODING ====================


def rle_encode(mask: np.ndarray) -> Dict[str, Any]:
    """
    Run-length encode a segmentation mask.

    Inverse of :func:`~pib3.backends.robot.rle_decode`, producing the same
    ``{"runs", "values", "shape"}`` dict the robot's on-board encoder emits,
    so simulated masks decode with the identical helper.

    Args:
        mask: 2-D array of per-pixel class/instance values.

    Returns:
        Dict with ``runs`` (run lengths), ``values`` (value per run) and
        ``shape`` ``[height, width]``.
    """
    flat = np.asarray(mask).ravel()
    if flat.size == 0:
        return {"runs": [], "values": [], "shape": list(np.shape(mask))}

    # Boundaries where the value changes; run lengths are the gaps between.
    change = np.flatnonzero(np.diff(flat)) + 1
    starts = np.concatenate(([0], change))
    ends = np.concatenate((change, [flat.size]))
    return {
        "runs": (ends - starts).astype(int).tolist(),
        "values": flat[starts].astype(int).tolist(),
        "shape": [int(mask.shape[0]), int(mask.shape[1])],
    }


# ==================== MODEL NAME MAPPING ====================

#: Maps the robot's model ids onto weights available off the shelf. Where no
#: host-side equivalent of the OAK-D blob exists, the closest current model is
#: substituted — that is the point of "or more up to date". Deprecated names
#: (``yolov6n``, ``yolo11n``, ``pose`` …) are resolved through
#: :data:`pib3.types.DEPRECATED_MODEL_ALIASES` first, as on the robot.
SIM_MODEL_ALIASES: Dict[str, str] = {
    # Detection — the robot runs the same YOLO26 sizes (its own RVC2 builds).
    AIModel.YOLO26S.value: "yolo26s.pt",
    AIModel.YOLO26N.value: "yolo26n.pt",
    # Pose — both sides are YOLO26 with 17 COCO keypoints.
    AIModel.POSE_YOLO.value: "yolo26s-pose.pt",
    AIModel.POSE_YOLO26N.value: "yolo26n-pose.pt",
    # Hand — handled by MediaPipe, not ultralytics (see MediaPipeHands).
    AIModel.HAND.value: "hand",
    "hand_tracking": "hand",
    # Simulation only: the robot has no segmentation model.
    "segmentation": "yolo26n-seg.pt",
}

#: Model ids with no simulated equivalent: the robot's face, emotion, head
#: pose and QR models, and names of models the robot no longer has.
UNSUPPORTED_IN_SIM = (
    {m.value for m in AIModel} - set(SIM_MODEL_ALIASES)
) | {"gaze", "lines", "person", "facemesh_crop", "facial_landmarks_68_crop"}


def simulated_models() -> Dict[str, str]:
    """Model ids the simulation can run, mapped to their task."""
    return {
        AIModel.YOLO26S.value: "object_detection",
        AIModel.YOLO26N.value: "object_detection",
        AIModel.POSE_YOLO.value: "pose_estimation",
        AIModel.POSE_YOLO26N.value: "pose_estimation",
        AIModel.HAND.value: "hand_tracking",
        "segmentation": "instance_segmentation",   # simulation only
    }


# ==================== RUNNERS ====================


class SimInference:
    """Base class: turn one BGR frame into the robot's detections."""

    def infer(self, bgr: np.ndarray) -> List[dict]:
        """Run the model; return ``Detection`` dicts in pixels of ``bgr``."""
        raise NotImplementedError

    def close(self) -> None:
        """Release any held resources."""


class _UltralyticsBase(SimInference):
    """Shared loading and box conversion for ultralytics models."""

    # conf 0.5 is the threshold of the robot's YOLO archives, so a scene shows
    # the same boxes in Webots as at a station.
    def __init__(self, weights: str, conf: float = 0.5):
        try:
            from ultralytics import YOLO
        except ImportError as exc:
            raise ImportError(
                "ultralytics is required for simulated detection/pose/"
                "segmentation. Install it with:  pip install ultralytics\n"
                "Alternatively use sim.ai.set_model('recognition') for "
                "Webots ground truth, which needs no model at all."
            ) from exc
        self._net = YOLO(weights)
        self._conf = conf
        self.weights = weights
        # nms=False picks YOLO26's end-to-end (NMS-free) head, the one its
        # benchmarks and ONNX exports use; ultralytics otherwise runs its
        # one-to-many head plus NMS. Only passed when that head exists, since
        # older models log a warning for it.
        try:
            head = self._net.model.model[-1]
        except (AttributeError, IndexError, TypeError):
            head = None
        self._extra = (
            {"nms": False} if getattr(head, "one2one_cv2", None) is not None else {}
        )

    def _predict(self, bgr: np.ndarray):
        return self._net(bgr, conf=self._conf, verbose=False, **self._extra)

    @staticmethod
    def _detection(
        box, names: dict, keypoints=(), mask_rle=None, keypoint_scores=None
    ) -> dict:
        """One ultralytics box -> the robot's ``Detection`` dict (pixels)."""
        x1, y1, x2, y2 = (float(v) for v in box.xyxy[0].tolist())
        return make_detection(
            label=str(names.get(int(box.cls[0]), "")),
            score=float(box.conf[0]),
            box=(x1, y1, x2, y2),
            keypoints=keypoints,
            mask_rle=mask_rle,
            keypoint_scores=keypoint_scores,
        )


class UltralyticsDetector(_UltralyticsBase):
    """Object detection — one ``Detection`` per box."""

    def infer(self, bgr: np.ndarray) -> List[dict]:
        detections = []
        for res in self._predict(bgr):
            names = getattr(res, "names", {}) or {}
            for box in getattr(res, "boxes", None) or []:
                detections.append(self._detection(box, names))
        return detections


class UltralyticsSegmenter(_UltralyticsBase):
    """Instance segmentation — boxes plus a ``mask_rle`` per object.

    The robot has no segmentation model, so ``mask_rle`` is an extension of
    the simulation's detections.
    """

    def infer(self, bgr: np.ndarray) -> List[dict]:
        detections = []
        for res in self._predict(bgr):
            names = getattr(res, "names", {}) or {}
            boxes = getattr(res, "boxes", None) or []
            masks = getattr(res, "masks", None)
            mask_data = masks.data if masks is not None else None

            for i, box in enumerate(boxes):
                mask_rle = None
                if mask_data is not None and i < len(mask_data):
                    binary = (mask_data[i].cpu().numpy() > 0.5).astype(np.uint8)
                    mask_rle = rle_encode(binary)
                detections.append(self._detection(box, names, mask_rle=mask_rle))
        return detections


class UltralyticsPose(_UltralyticsBase):
    """Body pose — a person box with the 17 COCO keypoints, named as the robot's."""

    def infer(self, bgr: np.ndarray) -> List[dict]:
        detections = []
        for res in self._predict(bgr):
            names = getattr(res, "names", {}) or {}
            boxes = getattr(res, "boxes", None) or []
            kps = getattr(res, "keypoints", None)
            if kps is None:
                continue

            xy = kps.xy.cpu().numpy()                      # (n, 17, 2) pixels
            conf = kps.conf.cpu().numpy() if kps.conf is not None else None
            for i in range(min(len(xy), len(boxes))):
                keypoints = [
                    (name, float(x), float(y))
                    for name, (x, y) in zip(COCO_KEYPOINT_NAMES, xy[i])
                ]
                scores = None if conf is None else conf[i].tolist()
                detections.append(
                    self._detection(boxes[i], names, keypoints, keypoint_scores=scores)
                )
        return detections


class MediaPipeHands(SimInference):
    """Hand landmarks — 21 points, matching pib3's HAND_* index constants.

    The OAK-D runs a MediaPipe-derived hand model on-device, so using
    MediaPipe on the host reproduces the same landmark topology rather than
    approximating it.
    """

    def __init__(self, max_hands: int = 2, min_confidence: float = 0.5):
        try:
            import mediapipe as mp
        except ImportError as exc:
            raise ImportError(
                "mediapipe is required for simulated hand landmarks. "
                "Install it with:  pip install mediapipe\n"
                "(Laptop only — on the Raspberry Pi use the OAK-D's "
                "on-device hand model instead.)"
            ) from exc
        self._mp = mp
        self._hands = mp.solutions.hands.Hands(
            static_image_mode=False,
            max_num_hands=max_hands,
            min_detection_confidence=min_confidence,
            min_tracking_confidence=min_confidence,
        )

    def infer(self, bgr: np.ndarray) -> List[dict]:
        height, width = bgr.shape[:2]
        # MediaPipe wants RGB; OpenCV/Webots give BGR.
        rgb = bgr[:, :, ::-1]
        result = self._hands.process(np.ascontiguousarray(rgb))

        landmark_sets = result.multi_hand_landmarks or []
        handedness_list = result.multi_handedness or []
        detections = []
        for i, landmarks in enumerate(landmark_sets):
            points = [
                (name, lm.x * width, lm.y * height)
                for name, lm in zip(HAND_KEYPOINT_NAMES, landmarks.landmark)
            ]
            xs = [x for _, x, _ in points]
            ys = [y for _, _, y in points]
            scalars = {}
            score = 1.0
            if i < len(handedness_list):
                top = handedness_list[i].classification[0]
                # The robot's ``handedness`` is the probability of "right".
                # NOTE: MediaPipe labels from the *image's* point of view, i.e.
                # a non-mirrored camera reports the physical left hand as
                # "Right". Kept as published so sim and robot agree; flip
                # downstream if your world uses a mirrored view.
                is_right = top.label.lower() == "right"
                scalars["handedness"] = top.score if is_right else 1.0 - top.score
                score = float(top.score)
            scalars["landmark_score"] = score
            detections.append(make_detection(
                label="hand",
                score=score,
                box=(min(xs), min(ys), max(xs), max(ys)),
                keypoints=points,
                scalars=scalars,
            ))
        return detections

    def close(self) -> None:
        try:
            self._hands.close()
        except Exception:
            pass


# ==================== FACTORY ====================


def build_runner(model_name: str, **kwargs) -> SimInference:
    """
    Create the simulated-inference runner for a robot model id.

    Args:
        model_name: An ``AIModel`` value (``AIModel.YOLO26N``,
            ``AIModel.HAND``, …), ``"segmentation"`` (simulation only) or a
            direct weights filename (``"yolo26s-pose.pt"``). Deprecated names
            are remapped with a DeprecationWarning.
        **kwargs: Forwarded to the runner (e.g. ``conf=0.4``).

    Returns:
        A :class:`SimInference` whose ``infer()`` yields robot-shaped results.

    Raises:
        ValueError: for models with no simulated equivalent (faces, emotion,
            head pose, QR codes).
        ImportError: if the needed optional backend is not installed.
    """
    name = resolve_model_name(model_name, stacklevel=2)

    if name in UNSUPPORTED_IN_SIM:
        raise ValueError(
            f"{name!r} has no simulated equivalent. Use the real OAK-D "
            f"station for it, or pick another model."
        )

    weights = SIM_MODEL_ALIASES.get(name, name)

    if weights == "hand":
        return MediaPipeHands(**kwargs)
    if "-pose" in weights:
        return UltralyticsPose(weights, **kwargs)
    if "-seg" in weights or weights.startswith("FastSAM"):
        return UltralyticsSegmenter(weights, **kwargs)
    return UltralyticsDetector(weights, **kwargs)
