"""Camera and AI subsystems for the Webots backend.

The real robot streams MJPEG over rosbridge and runs inference on the OAK-D's
own chip. Webots has neither, so these classes provide the *same public API*
on top of a simulated ``Camera`` device:

    frame = robot.camera.get_frame()          # works on both backends
    dets  = robot.ai.get_detections()         # works on both backends

Two sources of "AI" are available in simulation, chosen with
:meth:`WebotsAISubsystem.set_model`:

``AIModel`` values / model names
    Run a real model (ultralytics) on the simulated frames, on the CPU of the
    machine running Webots. Honest about latency, but COCO-trained detectors
    see very little in an untextured synthetic scene.

``"recognition"`` (the default)
    Use Webots' ``Recognition`` node: ground-truth objects straight from the
    simulator, with no model at all. Deterministic, instant, and perfect —
    which is exactly why it is useful for teaching the *downstream* logic
    (debouncing, state machines, control) without perception noise in the way.
    The contrast against the noisy real camera is the sim-to-real lesson.

    Only Solids that define ``recognitionColors`` are reported, and a Solid's
    ``model`` field becomes the detection label.
"""

import logging
import time
from typing import Any, List, Optional, Union

import numpy as np

from .camera import (
    COCO_LABELS,
    BoundingBox,
    CameraFrame,
    Detection,
    HandLandmarks,
    PoseKeypoints,
)

logger = logging.getLogger(__name__)


#: Model name that selects Webots ground-truth recognition instead of a network.
RECOGNITION_MODEL = "recognition"


def _call_first(obj: Any, *names: str) -> Any:
    """Call the first attribute of ``obj`` that exists, from ``names``.

    Webots' Python bindings have used both ``getPositionOnImage`` and
    ``get_position_on_image`` styles across releases, and some builds expose
    plain attributes instead of methods. Rather than pin one spelling and
    break on the next Webots version, try them in order.

    Raises:
        AttributeError: if none of the names exist on the object.
    """
    for name in names:
        attr = getattr(obj, name, None)
        if attr is None:
            continue
        return attr() if callable(attr) else attr
    raise AttributeError(
        f"None of {names!r} found on {type(obj).__name__}. "
        f"This is most likely a Webots API change — please report it."
    )


class WebotsCameraSubsystem:
    """RGB camera of the simulated robot, mirroring ``CameraSubsystem``.

    Accessed via ``robot.camera`` on :class:`~pib3.backends.WebotsBackend`.

    Example:
        >>> with pib3.Webots() as robot:
        ...     frame = robot.camera.get_frame()
        ...     img = frame.to_numpy()          # BGR, ready for OpenCV
    """

    #: Webots device name of the camera declared in pib.proto.
    DEVICE_NAME = "camera"

    def __init__(self, backend: "Any") -> None:
        self._backend = backend
        self._device = None
        self._frame_id = 0

    # --- device lifecycle ---------------------------------------------

    @property
    def device(self):
        """The underlying Webots ``Camera`` device, or None if absent."""
        return self._device

    @property
    def available(self) -> bool:
        """Whether the simulated robot actually has a camera device."""
        return self._device is not None

    def _ensure_enabled(self) -> bool:
        """Look up and enable the camera device once."""
        if self._device is not None:
            return True
        robot = getattr(self._backend, "_robot", None)
        if robot is None:
            return False
        device = robot.getDevice(self.DEVICE_NAME)
        if device is None:
            logger.warning(
                "No Webots device named %r. The pib proto must contain a "
                "Camera node (see pib3/resources/pib.proto); if you are using "
                "your own world, re-import the proto.",
                self.DEVICE_NAME,
            )
            return False
        timestep = getattr(self._backend, "_timestep", None) or 32
        device.enable(int(timestep))
        self._device = device
        logger.info(
            "Webots camera enabled: %dx%d @ %d ms",
            device.getWidth(), device.getHeight(), int(timestep),
        )
        return True

    @property
    def width(self) -> int:
        """Image width in pixels (0 if no camera)."""
        return self._device.getWidth() if self._ensure_enabled() else 0

    @property
    def height(self) -> int:
        """Image height in pixels (0 if no camera)."""
        return self._device.getHeight() if self._ensure_enabled() else 0

    # --- frames --------------------------------------------------------

    def get_frame(self, timeout: float = 5.0) -> Optional[CameraFrame]:
        """
        Get the current simulated camera frame.

        Unlike the real backend there is no buffer and no waiting: Webots
        renders synchronously, so this always returns the image belonging to
        the current simulation step. ``timeout`` is accepted for API
        compatibility and ignored.

        Returns:
            A :class:`CameraFrame` carrying a decoded BGR array, or None if
            the robot has no camera.
        """
        if not self._ensure_enabled():
            return None

        raw = self._device.getImage()
        if not raw:
            # Normal on the very first step, before the camera has rendered.
            return None

        w, h = self._device.getWidth(), self._device.getHeight()
        # Webots delivers BGRA, 4 bytes per pixel. Drop alpha -> BGR, which is
        # already OpenCV's channel order, so no conversion is needed.
        bgra = np.frombuffer(raw, np.uint8).reshape((h, w, 4))
        bgr = np.ascontiguousarray(bgra[:, :, :3])

        self._frame_id += 1
        return CameraFrame.from_numpy(
            bgr,
            frame_id=self._frame_id,
            timestamp_ns=time.time_ns(),
        )

    def get_frames(self) -> List[CameraFrame]:
        """Current frame as a one-element list (no buffering in simulation)."""
        frame = self.get_frame()
        return [frame] if frame is not None else []

    @property
    def frame_count(self) -> int:
        """Number of frames handed out since connect."""
        return self._frame_id

    def configure(self, **kwargs) -> None:
        """No-op: resolution and FPS are properties of the proto/world."""
        logger.debug(
            "camera.configure() ignored in simulation — set width/height/"
            "fieldOfView on the Camera node in pib.proto instead."
        )

    def stop(self) -> None:
        """Disable the camera device (saves simulation time)."""
        if self._device is not None:
            self._device.disable()
            self._device = None


class WebotsAISubsystem:
    """AI perception for the simulated robot, mirroring ``AISubsystem``.

    Accessed via ``robot.ai`` on :class:`~pib3.backends.WebotsBackend`.

    Defaults to Webots ground-truth ``Recognition``. Switch to a real network
    with :meth:`set_model`.

    Example:
        >>> with pib3.Webots() as robot:
        ...     for det in robot.ai.get_detections(latest_only=True):
        ...         print(det.label, det.confidence, det.bbox.center)
    """

    def __init__(self, backend: "Any") -> None:
        self._backend = backend
        self._model_name: str = RECOGNITION_MODEL
        self._net = None                 # lazily loaded ultralytics model
        self._recognition_on = False
        self._latencies: List[float] = []
        self._frame_times: List[float] = []

    # --- model selection ------------------------------------------------

    @property
    def model(self) -> Optional[str]:
        """Currently selected model name."""
        return self._model_name

    @property
    def uses_recognition(self) -> bool:
        """Whether ground-truth recognition is active (vs. a real network)."""
        return self._model_name == RECOGNITION_MODEL

    def set_model(self, model: "Union[str, Any]", timeout: float = 5.0) -> bool:
        """
        Choose the perception source for the simulation.

        Args:
            model: ``"recognition"`` for Webots ground truth (default), or an
                ``AIModel`` / model name to run a real network on the
                simulated frames (requires ``ultralytics``).
            timeout: Accepted for API compatibility; unused.

        Returns:
            True if the source is ready.

        Note:
            COCO-trained detectors see very little in an untextured synthetic
            scene. If YOLO returns nothing in Webots, that is the world, not a
            bug — either texture the objects or use ``"recognition"``.
        """
        name = getattr(model, "value", model)
        name = str(name)

        if name == RECOGNITION_MODEL:
            self._model_name = name
            self._net = None
            return self._enable_recognition()

        try:
            from ultralytics import YOLO
        except ImportError:
            logger.error(
                "ultralytics is not installed, so only %r is available in "
                "simulation. Install it, or call set_model(%r).",
                RECOGNITION_MODEL, RECOGNITION_MODEL,
            )
            return False

        weights = name if name.endswith(".pt") else f"{name}.pt"
        try:
            self._net = YOLO(weights)
        except Exception as exc:
            logger.error("Could not load %s: %s", weights, exc)
            return False
        self._model_name = name
        logger.info("Webots AI: running %s on simulated frames (CPU)", weights)
        return True

    def _enable_recognition(self) -> bool:
        """Turn on the Recognition node of the simulated camera."""
        camera = self._backend.camera
        if not camera._ensure_enabled():
            return False
        device = camera.device
        has_recognition = getattr(device, "hasRecognition", None)
        if has_recognition is not None and not has_recognition():
            logger.warning(
                "The Camera node has no Recognition child, so ground-truth "
                "detection is unavailable. Add `recognition Recognition {}` "
                "to the Camera in pib.proto, or use a real model."
            )
            return False
        if not self._recognition_on:
            timestep = getattr(self._backend, "_timestep", None) or 32
            device.recognitionEnable(int(timestep))
            self._recognition_on = True
        return True

    # --- results --------------------------------------------------------

    def get_detections(
        self,
        timeout: float = 5.0,
        latest_only: bool = False,
    ) -> List[Detection]:
        """
        Objects visible to the simulated camera.

        Webots renders synchronously and keeps no buffer, so this always
        describes the current simulation step. ``timeout`` and ``latest_only``
        are accepted for API compatibility — on this backend every call is
        already "latest only".

        Returns:
            List of :class:`Detection`. With ground-truth recognition,
            ``confidence`` is always 1.0.
        """
        started = time.perf_counter()
        if self.uses_recognition:
            detections = self._detections_from_recognition()
        else:
            detections = self._detections_from_network()
        self._record_timing(started)
        return detections

    def _detections_from_recognition(self) -> List[Detection]:
        if not self._enable_recognition():
            return []
        camera = self._backend.camera
        device, w, h = camera.device, camera.width, camera.height
        if not w or not h:
            return []

        results: List[Detection] = []
        for obj in device.getRecognitionObjects():
            try:
                cx, cy = _call_first(obj, "getPositionOnImage", "get_position_on_image")
                bw, bh = _call_first(obj, "getSizeOnImage", "get_size_on_image")
                model = _call_first(obj, "getModel", "get_model")
            except AttributeError as exc:
                logger.warning("Skipping recognition object: %s", exc)
                continue

            label = model.decode() if isinstance(model, bytes) else str(model)
            # Webots gives centre + size in pixels; Detection wants normalized
            # corners, matching the real backend's convention.
            bbox = BoundingBox(
                xmin=max(0.0, (cx - bw / 2) / w),
                ymin=max(0.0, (cy - bh / 2) / h),
                xmax=min(1.0, (cx + bw / 2) / w),
                ymax=min(1.0, (cy + bh / 2) / h),
            )
            results.append(Detection(
                label_id=COCO_LABELS.index(label) if label in COCO_LABELS else -1,
                confidence=1.0,          # ground truth: the simulator knows
                bbox=bbox,
                label=label,
            ))
        return results

    def _detections_from_network(self) -> List[Detection]:
        frame = self._backend.camera.get_frame()
        if frame is None or self._net is None:
            return []
        img = frame.to_numpy()
        if img is None:
            return []

        h, w = img.shape[:2]
        results: List[Detection] = []
        for res in self._net(img, verbose=False):
            names = getattr(res, "names", {})
            for box in getattr(res, "boxes", []):
                x1, y1, x2, y2 = (float(v) for v in box.xyxy[0].tolist())
                cls_id = int(box.cls[0])
                results.append(Detection(
                    label_id=cls_id,
                    confidence=float(box.conf[0]),
                    bbox=BoundingBox(x1 / w, y1 / h, x2 / w, y2 / h),
                    label=str(names.get(cls_id, "")),
                ))
        return results

    def get_hand_landmarks(self, timeout: float = 5.0, latest_only: bool = False) -> List[HandLandmarks]:
        """Not available in simulation — always empty.

        Hand landmarks come from a dedicated model on the OAK-D. Use the real
        camera station for gesture work; the simulation covers motion.
        """
        logger.debug("Hand landmarks are not simulated — use the real camera.")
        return []

    def get_poses(self, timeout: float = 5.0, latest_only: bool = False) -> List[PoseKeypoints]:
        """Not available in simulation — always empty. See :meth:`get_hand_landmarks`."""
        logger.debug("Pose keypoints are not simulated — use the real camera.")
        return []

    # --- metrics ---------------------------------------------------------

    def _record_timing(self, started: float) -> None:
        self._latencies.append((time.perf_counter() - started) * 1000.0)
        self._frame_times.append(time.perf_counter())
        del self._latencies[:-30]
        del self._frame_times[:-30]

    @property
    def fps(self) -> float:
        """Rate at which *you* are polling detections (not a camera FPS)."""
        if len(self._frame_times) < 2:
            return 0.0
        span = self._frame_times[-1] - self._frame_times[0]
        return (len(self._frame_times) - 1) / span if span > 1e-3 else 0.0

    @property
    def avg_latency_ms(self) -> float:
        """Average time spent producing detections, in milliseconds."""
        return sum(self._latencies) / len(self._latencies) if self._latencies else 0.0

    def clear(self) -> None:
        """No-op: simulation keeps no result buffer."""

    def stop(self) -> None:
        """Disable recognition and drop any loaded network."""
        camera = self._backend.camera
        if self._recognition_on and camera.device is not None:
            try:
                camera.device.recognitionDisable()
            except Exception:
                pass
        self._recognition_on = False
        self._net = None
