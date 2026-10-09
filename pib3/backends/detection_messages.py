"""The robot's detection message, as pib3 handles it.

The camera node (pib-backend, ``ros_packages/camera``) publishes one
``datatypes/DetectionArray`` per frame and model. rosbridge delivers it as
nested dicts::

    {"header": {"stamp": {"sec": 1, "nanosec": 5}, "frame_id": ""},
     "model_id": "yolo26n_coco_512x288",
     "frame_width": 1280, "frame_height": 720,
     "detections": [{
         "label": "person", "score": 0.91,
         "x_min": 412, "y_min": 80, "x_max": 700, "y_max": 710,   # pixels
         "keypoint_names": [], "keypoint_x": [], "keypoint_y": [],
         "keypoint_z": [],                                        # mm, 0 = none
         "scalar_names": [], "scalar_values": []}]}

pib3 builds the same dicts for the Webots simulation, so one parser
(:meth:`pib3.backends.camera.Detection.from_message` and friends) serves both.
This module holds the vocabulary and builders and imports nothing from the
rest of pib3.
"""

import time
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

#: The 17 COCO keypoints in the order the pose model outputs them.
COCO_KEYPOINT_NAMES: Tuple[str, ...] = (
    "nose", "left_eye", "right_eye", "left_ear", "right_ear",
    "left_shoulder", "right_shoulder", "left_elbow", "right_elbow",
    "left_wrist", "right_wrist", "left_hip", "right_hip",
    "left_knee", "right_knee", "left_ankle", "right_ankle",
)

#: The 21 hand landmarks in MediaPipe order, with the backend's names.
HAND_KEYPOINT_NAMES: Tuple[str, ...] = (
    "wrist",
    "thumb_cmc", "thumb_mcp", "thumb_ip", "thumb_tip",
    "index_finger_mcp", "index_finger_pip", "index_finger_dip", "index_finger_tip",
    "middle_finger_mcp", "middle_finger_pip", "middle_finger_dip", "middle_finger_tip",
    "ring_finger_mcp", "ring_finger_pip", "ring_finger_dip", "ring_finger_tip",
    "pinky_mcp", "pinky_pip", "pinky_dip", "pinky_tip",
)

#: Models whose results do not arrive on ``detections/<model_id>``: both hand
#: chains publish on one shared topic and tell themselves apart by ``model_id``.
MODEL_TOPICS: Dict[str, str] = {
    "hand_tracking": "detections/hand_tracking",
    "hand_tracking_mp": "detections/hand_tracking",
}


def detection_topic(model_id: str) -> str:
    """ROS topic (without leading slash) a model's DetectionArrays arrive on."""
    return MODEL_TOPICS.get(model_id, f"detections/{model_id}")


def make_detection(
    label: str,
    score: float,
    box: Sequence[float],
    keypoints: Iterable[Tuple[str, float, float]] = (),
    scalars: Optional[Dict[str, float]] = None,
    mask_rle: Optional[dict] = None,
) -> dict:
    """One ``datatypes/Detection`` as a dict.

    Args:
        label: Class name, for example ``"person"``.
        score: Confidence 0..1.
        box: ``(x_min, y_min, x_max, y_max)`` in pixels of the frame.
        keypoints: ``(name, x, y)`` per keypoint, in pixels.
        scalars: Named values, for example ``{"handedness": 0.9}``.
        mask_rle: Simulation only: an RLE segmentation mask. The robot's
            message has no such field.
    """
    points = list(keypoints)
    values = dict(scalars or {})
    detection = {
        "label": str(label),
        "score": float(score),
        "x_min": int(box[0]),
        "y_min": int(box[1]),
        "x_max": int(box[2]),
        "y_max": int(box[3]),
        "keypoint_names": [name for name, _, _ in points],
        "keypoint_x": [float(x) for _, x, _ in points],
        "keypoint_y": [float(y) for _, _, y in points],
        "keypoint_z": [0.0] * len(points),
        "scalar_names": list(values),
        "scalar_values": [float(v) for v in values.values()],
    }
    if mask_rle is not None:
        detection["mask_rle"] = mask_rle
    return detection


def make_detection_array(
    model_id: str,
    detections: List[dict],
    frame_width: int,
    frame_height: int,
    latency_ms: Optional[float] = None,
    stamp: Optional[float] = None,
) -> dict:
    """A ``datatypes/DetectionArray`` as a dict.

    Args:
        model_id: Model that produced the detections.
        detections: Dicts from :func:`make_detection`.
        frame_width: Pixel width the detections refer to.
        frame_height: Pixel height the detections refer to.
        latency_ms: Simulation only: the measured inference time. The robot's
            message carries only ``header.stamp``.
        stamp: Time in seconds since the epoch; default is now.
    """
    now = time.time() if stamp is None else float(stamp)
    message = {
        "header": {
            "stamp": {"sec": int(now), "nanosec": int((now % 1.0) * 1e9)},
            "frame_id": "",
        },
        "model_id": model_id,
        "frame_width": int(frame_width),
        "frame_height": int(frame_height),
        "detections": list(detections),
    }
    if latency_ms is not None:
        message["latency_ms"] = float(latency_ms)
    return message
