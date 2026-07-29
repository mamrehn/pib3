"""Simulated AI inference: OAK-D-shaped results from ordinary RGB frames.

The real robot runs its models on the OAK-D Lite's own accelerator and
publishes results on ``/camera/ai/detections``. Webots gives us only RGB
pixels — so this module runs equivalent (or newer) models on the host CPU/GPU
and emits **the exact same payload dicts** the robot publishes.

Because the payload shape is identical, simulated results flow through the
same :class:`~pib3.backends.camera.AIDetectionReceiver` and come out as the
same typed ``Detection`` / ``HandLandmarks`` / ``PoseKeypoints`` objects. Code
written against ``robot.ai`` runs unchanged against ``sim.ai``.

The topologies match by construction, which is why this works at all:

===================  ==============================  ========================
pib3 type            Convention                      Simulated with
===================  ==============================  ========================
``PoseKeypoints``    17 COCO keypoints               ultralytics ``*-pose``
``HandLandmarks``    21 MediaPipe hand landmarks     ``mediapipe`` Hands
``Detection``        normalized xyxy + class id      ultralytics detect/seg
===================  ==============================  ========================

Both backends are optional dependencies, imported lazily::

    pip install ultralytics      # detection, pose, segmentation
    pip install mediapipe        # hand landmarks

.. note::
   Running MediaPipe *here* is not a contradiction of the course guidance to
   avoid it on the Raspberry Pi. On the Pi it duplicated work the OAK-D does
   in silicon; on a laptop it is the closest available stand-in for exactly
   that silicon.
"""

import logging
import time
from typing import Any, Dict

import numpy as np

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

#: Maps the robot's AIModel names onto weights available off the shelf.
#: Where no host-side equivalent of the OAK-D blob exists, the closest
#: current model is substituted — that is the point of "or more up to date".
SIM_MODEL_ALIASES: Dict[str, str] = {
    # Detection — the OAK-D runs small blobs; on a host we can afford newer.
    "mobilenet-ssd": "yolo11n.pt",
    "yolov6n": "yolo11n.pt",
    "yolov8n": "yolov8n.pt",
    "yolo11n": "yolo11n.pt",
    "yolo11s": "yolo11s.pt",
    # Pose — both sides are 17-keypoint COCO, so this is a true equivalent.
    "pose": "yolo11n-pose.pt",
    "pose_yolo": "yolo11n-pose.pt",
    # Segmentation
    "yolov8n-seg": "yolov8n-seg.pt",
    "fastsam": "FastSAM-s.pt",
    "deeplabv3": "yolo11n-seg.pt",
    # Hand — handled by MediaPipe, not ultralytics (see MediaPipeHands).
    "hand": "hand",
}

#: Model names with no simulated equivalent yet.
UNSUPPORTED_IN_SIM = {"gaze", "lines"}


# ==================== RUNNERS ====================


class SimInference:
    """Base class: turn one BGR frame into a robot-shaped ``result`` dict."""

    #: Value placed in the payload's ``type`` field.
    model_type: str = "detection"

    def infer(self, bgr: np.ndarray) -> dict:
        """Run the model and return the ``result`` sub-dict of the payload."""
        raise NotImplementedError

    def close(self) -> None:
        """Release any held resources."""


class _UltralyticsBase(SimInference):
    """Shared loading and box conversion for ultralytics models."""

    def __init__(self, weights: str, conf: float = 0.25):
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

    @staticmethod
    def _box_dict(box, width: int, height: int, names: dict) -> dict:
        """One ultralytics box -> the robot's detection dict.

        Mirrors the wire format exactly: ``label`` is the numeric class id and
        ``label_name`` carries the human-readable name, because that is what
        ``Detection.from_dict`` expects.
        """
        x1, y1, x2, y2 = (float(v) for v in box.xyxy[0].tolist())
        cls_id = int(box.cls[0])
        return {
            "label": cls_id,
            "label_name": str(names.get(cls_id, "")),
            "confidence": float(box.conf[0]),
            "bbox": {
                "xmin": x1 / width,
                "ymin": y1 / height,
                "xmax": x2 / width,
                "ymax": y2 / height,
            },
        }


class UltralyticsDetector(_UltralyticsBase):
    """Object detection — emits the robot's ``detection`` payload."""

    model_type = "detection"

    def infer(self, bgr: np.ndarray) -> dict:
        height, width = bgr.shape[:2]
        detections = []
        for res in self._net(bgr, conf=self._conf, verbose=False):
            names = getattr(res, "names", {}) or {}
            for box in getattr(res, "boxes", None) or []:
                detections.append(self._box_dict(box, width, height, names))
        return {"detections": detections}


class UltralyticsSegmenter(_UltralyticsBase):
    """Instance segmentation — detection payload plus ``mask_rle`` per object."""

    model_type = "instance-segmentation"

    def infer(self, bgr: np.ndarray) -> dict:
        height, width = bgr.shape[:2]
        detections = []
        for res in self._net(bgr, conf=self._conf, verbose=False):
            names = getattr(res, "names", {}) or {}
            boxes = getattr(res, "boxes", None) or []
            masks = getattr(res, "masks", None)
            mask_data = masks.data if masks is not None else None

            for i, box in enumerate(boxes):
                det = self._box_dict(box, width, height, names)
                if mask_data is not None and i < len(mask_data):
                    binary = (mask_data[i].cpu().numpy() > 0.5).astype(np.uint8)
                    det["mask_rle"] = rle_encode(binary)
                detections.append(det)
        return {"detections": detections}


class UltralyticsPose(_UltralyticsBase):
    """Body pose — 17 COCO keypoints, the same convention pib3 already uses."""

    model_type = "pose"

    def infer(self, bgr: np.ndarray) -> dict:
        height, width = bgr.shape[:2]
        detections = []
        for res in self._net(bgr, conf=self._conf, verbose=False):
            names = getattr(res, "names", {}) or {}
            boxes = getattr(res, "boxes", None) or []
            kps = getattr(res, "keypoints", None)
            if kps is None:
                continue

            xy = kps.xy.cpu().numpy()                      # (n, 17, 2) pixels
            scores = kps.conf.cpu().numpy() if kps.conf is not None else None

            for i in range(len(xy)):
                person = {
                    "keypoints": [
                        {
                            "x": float(x) / width,
                            "y": float(y) / height,
                            "confidence": (
                                float(scores[i][j]) if scores is not None else 1.0
                            ),
                        }
                        for j, (x, y) in enumerate(xy[i])
                    ]
                }
                if i < len(boxes):
                    person.update(self._box_dict(boxes[i], width, height, names))
                detections.append(person)
        # The robot publishes multi-person pose as detections-with-keypoints.
        return {"detections": detections}


class MediaPipeHands(SimInference):
    """Hand landmarks — 21 points, matching pib3's HAND_* index constants.

    The OAK-D runs a MediaPipe-derived hand model on-device, so using
    MediaPipe on the host reproduces the same landmark topology rather than
    approximating it.
    """

    model_type = "hand"

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

    def infer(self, bgr: np.ndarray) -> dict:
        # MediaPipe wants RGB; OpenCV/Webots give BGR.
        rgb = bgr[:, :, ::-1]
        result = self._hands.process(np.ascontiguousarray(rgb))

        landmark_sets = result.multi_hand_landmarks or []
        if not landmark_sets:
            return {"keypoints": []}

        handedness_list = result.multi_handedness or []
        # The robot publishes one hand per message, so emit the first and note
        # the rest — matching parse_ai_result(), which reads a single set.
        landmarks = landmark_sets[0]
        keypoints = [
            {"x": float(lm.x), "y": float(lm.y), "confidence": 1.0}
            for lm in landmarks.landmark
        ]

        handedness = None
        if handedness_list:
            top = handedness_list[0].classification[0]
            # NOTE: MediaPipe labels from the *image's* point of view, i.e. a
            # non-mirrored camera reports the physical left hand as "Right".
            # Kept as published so sim and robot agree; flip downstream if
            # your world uses a mirrored view.
            handedness = {"label": top.label.lower(), "score": float(top.score)}

        return {"keypoints": keypoints, "handedness": handedness}

    def close(self) -> None:
        try:
            self._hands.close()
        except Exception:
            pass


# ==================== FACTORY ====================


def build_runner(model_name: str, **kwargs) -> SimInference:
    """
    Create the simulated-inference runner for a robot model name.

    Args:
        model_name: An ``AIModel`` value (``"yolov8n"``, ``"hand"``, ``"pose"``,
            …) or a direct weights filename (``"yolo11s-pose.pt"``).
        **kwargs: Forwarded to the runner (e.g. ``conf=0.4``).

    Returns:
        A :class:`SimInference` whose ``infer()`` yields robot-shaped results.

    Raises:
        ValueError: for models with no simulated equivalent (``gaze``, ``lines``).
        ImportError: if the needed optional backend is not installed.
    """
    name = str(getattr(model_name, "value", model_name))

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


def build_payload(
    result: dict,
    model: str,
    model_type: str,
    frame_id: int,
    latency_ms: float,
) -> dict:
    """
    Wrap a runner result in the robot's full ``/camera/ai/detections`` payload.

    Producing the identical envelope is what lets simulated results go through
    the same receiver and parser as real ones.
    """
    return {
        "model": model,
        "type": model_type,
        "frame_id": frame_id,
        "timestamp_ns": time.time_ns(),
        "latency_ms": latency_ms,
        "result": result,
    }
