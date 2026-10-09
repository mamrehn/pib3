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
from typing import Any, Dict, List, Optional, Tuple, Union

import numpy as np

from ..types import resolve_model_name
from .hints import hint
from .detection_messages import make_detection, make_detection_array
from .camera import (
    AIDetectionReceiver,
    AIModelInfo,
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

    #: Consecutive reads within one simulation step before we point out
    #: that the loop is missing sim.step(). High enough that legitimately
    #: polling a few getters per step never trips it.
    STUCK_READ_LIMIT = 25

    def __init__(self, backend: "Any") -> None:
        self._backend = backend
        self._device = None
        self._frame_id = 0
        self._cached_frame: Optional[CameraFrame] = None
        self._cached_time: float = -1.0
        self._display = None          # Webots Display, set by show_on_display()
        self._reads_without_step = 0

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

        The frame is cached per simulation step, so calling this repeatedly
        without a :meth:`~pib3.backends.WebotsBackend.step` returns the same
        object (and the same ``frame_id``) instead of re-copying pixels. That
        also lets ``ai`` skip re-running inference on an image it has already
        seen.

        Returns:
            A :class:`CameraFrame` carrying a decoded BGR array, or None if
            the robot has no camera.
        """
        if not self._ensure_enabled():
            return None

        now = self._sim_time()
        if self._cached_frame is not None and now == self._cached_time:
            # Reading repeatedly without stepping means the image cannot have
            # changed — the classic "my loop does nothing" bug.
            self._reads_without_step += 1
            if self._reads_without_step == self.STUCK_READ_LIMIT:
                hint(
                    "no-step-loop",
                    f"You have read the camera {self.STUCK_READ_LIMIT} times "
                    "without advancing the simulation, so it is the SAME image "
                    "every time and nothing can ever change.\n"
                    "  Your loop needs a step:\n"
                    "      while sim.step():\n"
                    "          frame = sim.camera.get_frame()\n"
                    "  sim.step() advances time, renders the next image, and "
                    "returns False when Webots closes.",
                )
            return self._cached_frame
        self._reads_without_step = 0

        try:
            raw = self._device.getImage()
        except ValueError:
            # Webots returns a NULL image pointer until the camera has
            # rendered at least once, and the Python binding dereferences it
            # unconditionally — so this raises rather than returning empty.
            # Enabling happens inside the first _ensure_enabled() call above,
            # meaning the very first get_frame() lands in exactly this case.
            # Not an error: step the simulation and ask again.
            logger.debug("Camera image not ready yet (needs a step after enable).")
            return None
        if not raw:
            return None

        w, h = self._device.getWidth(), self._device.getHeight()
        # Webots delivers BGRA, 4 bytes per pixel. Drop alpha -> BGR, which is
        # already OpenCV's channel order, so no conversion is needed.
        bgra = np.frombuffer(raw, np.uint8).reshape((h, w, 4))
        bgr = np.ascontiguousarray(bgra[:, :, :3])

        self._frame_id += 1
        self._cached_frame = CameraFrame.from_numpy(
            bgr,
            frame_id=self._frame_id,
            timestamp_ns=time.time_ns(),
        )
        self._cached_time = now
        return self._cached_frame

    def _sim_time(self) -> float:
        """Current simulation time, used to invalidate the frame cache."""
        robot = getattr(self._backend, "_robot", None)
        return robot.getTime() if robot is not None else 0.0

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

    # --- live view in the 3D window ------------------------------------

    #: Webots device name of the Display declared in pib.proto.
    DISPLAY_NAME = "camera_display"

    def show_on_display(self, name: Optional[str] = None) -> bool:
        """
        Mirror the camera onto a Webots ``Display``, shown in the 3D window.

        Call once, before your loop. Webots then keeps the panel filled with
        the live camera image on its own — no per-step work and no cost in
        your control loop. Use :meth:`draw_detections` to paint on top of it.

        This is the quickest way to make "what the robot sees" visible to
        somebody watching the simulation, rather than only to the code.

        Args:
            name: Display device name. Defaults to ``DISPLAY_NAME``.

        Returns:
            True if the display was found and attached.
        """
        if not self._ensure_enabled():
            return False
        robot = getattr(self._backend, "_robot", None)
        if robot is None:
            return False

        display = robot.getDevice(name or self.DISPLAY_NAME)
        if display is None:
            logger.warning(
                "No Webots device named %r, so the live view is unavailable. "
                "Add a Display node to the robot (see pib.proto); an existing "
                "world keeps its own copy of the proto and may need re-importing.",
                name or self.DISPLAY_NAME,
            )
            return False

        display.attachCamera(self._device)
        self._display = display
        return True

    @property
    def display(self):
        """The attached Webots ``Display``, or None if never attached."""
        return self._display

    def draw_detections(self, detections, color: int = 0xFFFF00) -> None:
        """
        Draw detection boxes and labels onto the attached display.

        No-op when :meth:`show_on_display` was never called, so it is safe to
        leave in code that also runs headless.

        Call it once per step, also when nothing was detected. The attached
        camera only refreshes the panel's background; boxes live on a drawing
        layer above it that Webots never clears, so each call first erases the
        previous boxes — otherwise they smear across the panel as objects move.

        Args:
            detections: Iterable of :class:`Detection` (normalized boxes).
            color: Line/text colour as 0xRRGGBB.
        """
        display = self._display
        if display is None:
            return

        w, h = display.getWidth(), display.getHeight()
        # Transparent pixels let the camera background show through again.
        display.setAlpha(0.0)
        display.fillRectangle(0, 0, w, h)
        display.setAlpha(1.0)
        display.setColor(color)
        for det in detections:
            box = det.bbox
            x1, y1, x2, y2 = box.to_pixels(w, h)
            display.drawRectangle(x1, y1, max(1, x2 - x1), max(1, y2 - y1))
            label = f"{det.label} {det.confidence:.0%}" if det.label else ""
            if label:
                # Keep the text inside the panel when the box hugs the top.
                display.drawText(label, x1, max(0, y1 - 12))

    def stop(self) -> None:
        """Disable the camera device (saves simulation time)."""
        if self._display is not None:
            try:
                self._display.detachCamera()
            except Exception:
                pass
            self._display = None
        if self._device is not None:
            self._device.disable()
            self._device = None


class WebotsAISubsystem:
    """AI perception for the simulated robot, mirroring ``AISubsystem``.

    Accessed via ``robot.ai`` on :class:`~pib3.backends.WebotsBackend`, with
    the same methods as on the real robot: ``set_model``, ``start_model``,
    ``stop_model``, ``models``, ``available_models`` and the getters with their
    ``model=`` argument.

    Starts out with Webots ground-truth ``Recognition`` as the one running
    model. Run a real network with :meth:`set_model` or :meth:`start_model`;
    several can run together, each infers on every rendered frame.

    Example:
        >>> with pib3.Webots() as robot:
        ...     for det in robot.ai.get_detections(latest_only=True):
        ...         print(det.label, det.confidence, det.bbox.center)
    """

    def __init__(self, backend: "Any") -> None:
        self._backend = backend
        # Started models: id -> SimInference (None for ground-truth recognition).
        self._runners: Dict[str, Optional[Any]] = {}
        # Results go through the SAME receiver the real robot feeds, so
        # buffering, fps and latency behave identically on both backends.
        self._receivers: Dict[str, AIDetectionReceiver] = {}
        self._current_model: Optional[str] = None
        self._recognition_on = False
        self._last_inferred_frame = -1
        self._add(RECOGNITION_MODEL, None)

    # --- model selection ------------------------------------------------

    def _add(self, name: str, runner: Optional[Any]) -> None:
        receiver = AIDetectionReceiver()
        receiver.expect_model(name)
        self._runners[name] = runner
        self._receivers[name] = receiver
        self._current_model = name
        self._last_inferred_frame = -1

    @property
    def model(self) -> Optional[str]:
        """Id of the current model: the one the getters read by default."""
        return self._current_model

    @property
    def models(self) -> Tuple[str, ...]:
        """Ids of all running models."""
        return tuple(self._runners)

    @property
    def uses_recognition(self) -> bool:
        """Whether ground-truth recognition is the current model."""
        return self._current_model == RECOGNITION_MODEL

    def available_models(self) -> List[AIModelInfo]:
        """The models the simulation can run, with their state.

        Narrower than the robot's list: faces, emotion, head pose and QR codes
        have no simulated equivalent.
        """
        from .sim_ai import simulated_models

        tasks = {RECOGNITION_MODEL: "ground_truth", **simulated_models()}
        return [
            AIModelInfo(name=name, task=task, available=True, active=name in self._runners)
            for name, task in tasks.items()
        ]

    def start_model(self, model: "Union[str, Any]", timeout: float = 5.0) -> bool:
        """
        Start a model without stopping the others, and make it the current one.

        Args:
            model: ``"recognition"`` for Webots ground truth, or any
                ``AIModel`` / model id to run a real network on the simulated
                frames. Detection, pose and segmentation need
                ``ultralytics``; ``AIModel.HAND`` needs ``mediapipe``.
            timeout: Accepted for API compatibility; unused.

        Returns:
            True if the model is running. A model that cannot be loaded leaves
            everything as it was.

        Note:
            COCO-trained detectors see very little in an untextured synthetic
            scene. If YOLO returns nothing in Webots, that is the world, not a
            bug — either texture the objects or use ``"recognition"``.
        """
        name = resolve_model_name(model, stacklevel=2)

        if name == RECOGNITION_MODEL:
            # Recognition is the starting model, so it may already be listed;
            # either way the node is (re-)enabled, which is idempotent. A world
            # without a Recognition node keeps it listed: the getters then
            # return nothing and the warning says why.
            added = name not in self._runners
            if added:
                self._add(name, None)
            self._current_model = name
            if self._enable_recognition():
                return True
            if added:
                self._remove(name)
            return False

        if name in self._runners:
            self._current_model = name
            return True

        from .sim_ai import build_runner

        try:
            runner = build_runner(name)
        except (ImportError, ValueError) as exc:
            logger.error("Cannot use %r in simulation: %s", name, exc)
            return False
        except Exception as exc:
            logger.error("Could not load a simulated model for %r: %s", name, exc)
            return False

        self._add(name, runner)
        logger.info(
            "Webots AI: %r simulated with %s on host hardware",
            name, type(runner).__name__,
        )
        return True

    def _remove(self, name: str) -> None:
        runner = self._runners.pop(name, None)
        self._receivers.pop(name, None)
        if runner is not None:
            runner.close()
        if name == RECOGNITION_MODEL:
            self._disable_recognition()
        if self._current_model == name:
            self._current_model = next(reversed(self._runners), None)

    def stop_model(
        self, model: "Union[str, Any, None]" = None, timeout: float = 5.0
    ) -> bool:
        """Stop a model (the current one by default). Always succeeds."""
        name = resolve_model_name(model, stacklevel=2) if model else self._current_model
        if name in self._runners:
            self._remove(name)
        return True

    def set_model(self, model: "Union[str, Any]", timeout: float = 5.0) -> bool:
        """
        Run this model and none of the others.

        The new model is loaded before the old ones are stopped, so a model
        that cannot be loaded (a missing package, a model the simulation does
        not have) leaves the old ones running. On the robot a failed start can
        stop them; code must not rely on either.

        Returns:
            True if the model is running.
        """
        name = resolve_model_name(model, stacklevel=2)
        if not self.start_model(name, timeout):
            return False
        for other in [m for m in self._runners if m != name]:
            self._remove(other)
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

    def _disable_recognition(self) -> None:
        camera = self._backend.camera
        if self._recognition_on and camera.device is not None:
            try:
                camera.device.recognitionDisable()
            except Exception:
                pass
        self._recognition_on = False

    # --- results --------------------------------------------------------

    def _infer_started_models(self) -> None:
        """Produce one robot-shaped message per running model for this step.

        Runs at most once per rendered frame: Webots is synchronous, so
        polling three getters in one step must not pay for three inferences.
        """
        camera = self._backend.camera
        frame = camera.get_frame()
        if frame is None or frame.frame_id == self._last_inferred_frame:
            return
        self._last_inferred_frame = frame.frame_id

        image = None
        for name, runner in list(self._runners.items()):
            started = time.perf_counter()
            if runner is None:                                # recognition
                width, height = camera.width, camera.height
                detections = self._recognition_detections()
            else:
                if image is None:
                    image = frame.to_numpy()
                    if image is None:
                        return
                height, width = image.shape[:2]
                detections = runner.infer(image)
            latency_ms = (time.perf_counter() - started) * 1000.0

            # Feed the real receiver exactly as rosbridge would on the robot.
            self._receivers[name].on_detection(make_detection_array(
                model_id=name,
                detections=detections,
                frame_width=width,
                frame_height=height,
                latency_ms=latency_ms,
            ))

    def _recognition_detections(self) -> List[dict]:
        """Webots ground-truth objects as the robot's ``Detection`` dicts."""
        if not self._enable_recognition():
            return []
        camera = self._backend.camera
        device, w, h = camera.device, camera.width, camera.height
        if not w or not h:
            return []

        detections = []
        for obj in device.getRecognitionObjects():
            try:
                cx, cy = _call_first(obj, "getPositionOnImage", "get_position_on_image")
                bw, bh = _call_first(obj, "getSizeOnImage", "get_size_on_image")
                model = _call_first(obj, "getModel", "get_model")
            except AttributeError as exc:
                logger.warning("Skipping recognition object: %s", exc)
                continue

            label = model.decode() if isinstance(model, bytes) else str(model)
            # Webots gives centre + size in pixels; the message wants corners.
            detections.append(make_detection(
                label=label,
                score=1.0,                # ground truth: the simulator knows
                box=(
                    max(0.0, cx - bw / 2), max(0.0, cy - bh / 2),
                    min(float(w), cx + bw / 2), min(float(h), cy + bh / 2),
                ),
            ))
        return detections

    def _receiver(self, model: "Union[str, Any, None]") -> AIDetectionReceiver:
        name = resolve_model_name(model, stacklevel=3) if model else self._current_model
        if name is None or name not in self._receivers:
            raise RuntimeError(
                "No AI model started. Call robot.ai.set_model(AIModel.YOLO26N) "
                "(or another model, or 'recognition') first."
            )
        return self._receivers[name]

    def _check_stale_buffer(
        self, receiver: AIDetectionReceiver, latest_only: bool, what: str
    ) -> None:
        """Point out a multi-frame read that the caller almost certainly
        did not intend.

        Reading without ``latest_only`` returns every buffered frame, so the
        same physical object appears once per frame. Counting them counts it
        many times over, and a loop that takes the first match keeps acting on
        the OLDEST frame — which never moves. Both look like "my code does
        nothing" rather than like a bug in the read.
        """
        if latest_only:
            return
        buffered = receiver.result_count
        if buffered > 1:
            hint(
                "stale-buffer",
                f"{what}() just returned results from {buffered} different "
                "frames, not just the current one.\n"
                "  In a loop this means: the same object is counted once per "
                "frame, and the FIRST result is the oldest — it never moves.\n"
                f"  Fix:  sim.ai.{what}(latest_only=True)\n"
                "  Leave it off only when you deliberately want every frame "
                "since the last call.",
            )

    def get_detections(
        self,
        timeout: float = 5.0,
        latest_only: bool = False,
        model: "Union[str, Any, None]" = None,
    ) -> List[Detection]:
        """
        Objects visible to the simulated camera.

        Webots renders synchronously, so results always describe the current
        step; ``timeout`` is accepted for API compatibility and ignored.

        Returns:
            List of :class:`Detection`. With ground-truth recognition,
            ``confidence`` is always 1.0.
        """
        receiver = self._receiver(model)
        self._infer_started_models()
        self._check_stale_buffer(receiver, latest_only, "get_detections")
        return receiver.get_detections(timeout=0.0, latest_only=latest_only)

    def get_hand_landmarks(
        self,
        timeout: float = 5.0,
        latest_only: bool = False,
        model: "Union[str, Any, None]" = None,
    ) -> List[HandLandmarks]:
        """
        Hand landmarks from the simulated camera — 21 points with finger angles.

        Requires ``set_model(AIModel.HAND)`` and the ``mediapipe`` package; the
        landmark topology matches the OAK-D's on-device hand model, so
        ``hand.finger_angles`` behaves the same on both backends.

        Returns an empty list under ``"recognition"``, which knows about
        objects but not about hands.
        """
        receiver = self._receiver(model)
        self._infer_started_models()
        self._check_stale_buffer(receiver, latest_only, "get_hand_landmarks")
        return receiver.get_hand_landmarks(timeout=0.0, latest_only=latest_only)

    def get_poses(
        self,
        timeout: float = 5.0,
        latest_only: bool = False,
        model: "Union[str, Any, None]" = None,
    ) -> List[PoseKeypoints]:
        """
        Body poses from the simulated camera — 17 COCO keypoints.

        Requires ``set_model(AIModel.POSE_YOLO)`` and the ``ultralytics``
        package. The keypoint order is the same COCO convention the robot
        publishes, so ``pose.left_shoulder`` and friends work unchanged.
        """
        receiver = self._receiver(model)
        self._infer_started_models()
        self._check_stale_buffer(receiver, latest_only, "get_poses")
        return receiver.get_poses(timeout=0.0, latest_only=latest_only)

    # --- metrics ---------------------------------------------------------

    @property
    def fps(self) -> float:
        """Inference rate of the current model, measured as on the robot."""
        receiver = self._receivers.get(self._current_model)
        return receiver.fps if receiver else 0.0

    @property
    def avg_latency_ms(self) -> float:
        """
        Average inference latency of the current model in milliseconds.

        Honest host-hardware timing — usually *slower* than the OAK-D's
        dedicated accelerator. That gap is worth showing rather than hiding:
        it is the whole argument for edge AI.
        """
        receiver = self._receivers.get(self._current_model)
        return receiver.avg_latency_ms if receiver else 0.0

    def clear(self) -> None:
        """Drop buffered results of every running model."""
        for receiver in self._receivers.values():
            receiver.clear()
        self._last_inferred_frame = -1

    def stop(self) -> None:
        """Stop every model, disable recognition and release loaded models."""
        for name in list(self._runners):
            self._remove(name)
        self._disable_recognition()
