"""Unit tests for the simulated camera / AI subsystems.

These run **without Webots**: the Camera device and its recognition objects
are faked, which is enough to pin down the parts that are easy to get wrong
and impossible to eyeball —

* BGRA -> BGR conversion (channel order and alpha removal),
* pixel centre+size -> normalized corner bbox,
* the per-simulation-step frame cache,
* CameraFrame carrying raw arrays instead of JPEG.

The things that genuinely need a simulator (camera orientation, whether the
Camera node is parented to the head) are covered by
``examples/webots_camera_check.py`` instead.
"""

import time

import numpy as np
import pytest

from pib3.backends.camera import BoundingBox, CameraFrame, COCO_LABELS, Detection
from pib3.backends.webots_camera import (
    RECOGNITION_MODEL,
    WebotsAISubsystem,
    WebotsCameraSubsystem,
    _call_first,
)


# ==================== fakes ====================


class FakeRecognitionObject:
    """Mimics Webots' CameraRecognitionObject (camelCase spelling)."""

    def __init__(self, center, size, model):
        self._center, self._size, self._model = center, size, model

    def getPositionOnImage(self):
        return self._center

    def getSizeOnImage(self):
        return self._size

    def getModel(self):
        return self._model


class SnakeCaseRecognitionObject:
    """Same data, snake_case spelling — other Webots releases expose this."""

    def __init__(self, center, size, model):
        self._center, self._size, self._model = center, size, model

    def get_position_on_image(self):
        return self._center

    def get_size_on_image(self):
        return self._size

    def get_model(self):
        return self._model


class FakeCameraDevice:
    def __init__(self, width=640, height=400, image=None):
        self._w, self._h = width, height
        self._image = image
        self.enabled_with = None
        self.recognition_enabled_with = None
        self.objects = []
        self.has_recognition_flag = True

    # device API
    def enable(self, period):
        self.enabled_with = period

    def disable(self):
        self.enabled_with = None

    def getWidth(self):
        return self._w

    def getHeight(self):
        return self._h

    def getImage(self):
        return self._image

    # recognition API
    def hasRecognition(self):
        return self.has_recognition_flag

    def recognitionEnable(self, period):
        self.recognition_enabled_with = period

    def recognitionDisable(self):
        self.recognition_enabled_with = None

    def getRecognitionObjects(self):
        return self.objects


class FakeWebotsRobot:
    def __init__(self, device):
        self._device = device
        self.time = 0.0

    def getDevice(self, name):
        return self._device if name == "camera" else None

    def getTime(self):
        return self.time


class FakeBackend:
    """Stands in for WebotsBackend: only what the subsystems actually touch."""

    def __init__(self, device=None, timestep=32):
        self._device = device if device is not None else FakeCameraDevice()
        self._robot = FakeWebotsRobot(self._device)
        self._timestep = timestep
        self._camera = None

    @property
    def camera(self):
        if self._camera is None:
            self._camera = WebotsCameraSubsystem(self)
        return self._camera

    def advance(self, dt=0.032):
        self._robot.time += dt


def bgra_bytes(width, height, b, g, r, a=255):
    """A solid-colour BGRA buffer, the layout Webots' getImage() returns."""
    buf = np.zeros((height, width, 4), dtype=np.uint8)
    buf[:, :, 0] = b
    buf[:, :, 1] = g
    buf[:, :, 2] = r
    buf[:, :, 3] = a
    return buf.tobytes()


# ==================== _call_first ====================


def test_call_first_prefers_earlier_name():
    class Both:
        def a(self):
            return "a"

        def b(self):
            return "b"

    assert _call_first(Both(), "a", "b") == "a"
    assert _call_first(Both(), "b", "a") == "b"


def test_call_first_falls_through_to_later_name():
    class OnlyB:
        def b(self):
            return "b"

    assert _call_first(OnlyB(), "a", "b") == "b"


def test_call_first_accepts_plain_attributes():
    class Attr:
        value = 7

    assert _call_first(Attr(), "value") == 7


def test_call_first_raises_with_helpful_message():
    with pytest.raises(AttributeError, match="Webots API change"):
        _call_first(object(), "nope", "also_nope")


# ==================== CameraFrame ====================


def test_camera_frame_from_numpy_roundtrip():
    img = np.random.randint(0, 255, (4, 6, 3), dtype=np.uint8)
    frame = CameraFrame.from_numpy(img, frame_id=3, timestamp_ns=1_000_000_000)

    assert frame.frame_id == 3
    assert frame.timestamp == pytest.approx(1.0)
    assert frame.jpeg_bytes == b""
    np.testing.assert_array_equal(frame.to_numpy(), img)


def test_camera_frame_to_jpeg_encodes_on_demand():
    cv2 = pytest.importorskip("cv2")
    img = np.zeros((8, 8, 3), dtype=np.uint8)
    frame = CameraFrame.from_numpy(img)

    data = frame.to_jpeg()
    assert data[:2] == b"\xff\xd8"                      # JPEG SOI marker
    assert cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR).shape == (8, 8, 3)


def test_camera_frame_to_jpeg_passes_through_existing_bytes():
    frame = CameraFrame(jpeg_bytes=b"\xff\xd8already")
    assert frame.to_jpeg() == b"\xff\xd8already"


def test_camera_frame_array_excluded_from_equality():
    """Two frames with identical metadata compare equal despite big arrays."""
    a = CameraFrame.from_numpy(np.zeros((2, 2, 3), np.uint8), frame_id=1)
    b = CameraFrame.from_numpy(np.ones((2, 2, 3), np.uint8), frame_id=1)
    assert a == b


# ==================== camera subsystem ====================


def test_get_frame_converts_bgra_to_bgr():
    """Alpha is dropped and channel order preserved — no accidental RGB swap."""
    device = FakeCameraDevice(4, 3, bgra_bytes(4, 3, b=10, g=20, r=30))
    cam = WebotsCameraSubsystem(FakeBackend(device))

    img = cam.get_frame().to_numpy()

    assert img.shape == (3, 4, 3)
    assert img.dtype == np.uint8
    assert tuple(img[0, 0]) == (10, 20, 30)


def test_get_frame_enables_device_with_timestep():
    device = FakeCameraDevice(2, 2, bgra_bytes(2, 2, 1, 2, 3))
    cam = WebotsCameraSubsystem(FakeBackend(device, timestep=64))

    cam.get_frame()
    assert device.enabled_with == 64


def test_get_frame_returns_none_before_first_render():
    device = FakeCameraDevice(2, 2, image=None)
    assert WebotsCameraSubsystem(FakeBackend(device)).get_frame() is None


def test_get_frame_returns_none_without_device():
    backend = FakeBackend()
    backend._robot._device = None
    assert WebotsCameraSubsystem(backend).get_frame() is None


def test_frame_is_cached_within_one_simulation_step():
    device = FakeCameraDevice(2, 2, bgra_bytes(2, 2, 1, 2, 3))
    backend = FakeBackend(device)
    cam = WebotsCameraSubsystem(backend)

    first = cam.get_frame()
    again = cam.get_frame()
    assert again is first, "same step must not re-copy pixels"
    assert again.frame_id == first.frame_id

    backend.advance()
    later = cam.get_frame()
    assert later is not first
    assert later.frame_id == first.frame_id + 1


def test_frame_count_tracks_distinct_frames():
    device = FakeCameraDevice(2, 2, bgra_bytes(2, 2, 0, 0, 0))
    backend = FakeBackend(device)
    cam = WebotsCameraSubsystem(backend)

    for _ in range(3):
        cam.get_frame()
        backend.advance()

    assert cam.frame_count == 3


def test_stop_disables_device():
    device = FakeCameraDevice(2, 2, bgra_bytes(2, 2, 0, 0, 0))
    cam = WebotsCameraSubsystem(FakeBackend(device))
    cam.get_frame()

    cam.stop()
    assert device.enabled_with is None
    assert cam.device is None


# ==================== AI subsystem: recognition ====================


def _ai_with(objects, width=640, height=400):
    device = FakeCameraDevice(width, height, bgra_bytes(width, height, 0, 0, 0))
    device.objects = objects
    backend = FakeBackend(device)
    ai = WebotsAISubsystem(backend)
    ai.set_model(RECOGNITION_MODEL)
    return ai, device, backend


def test_recognition_is_the_default_model():
    ai = WebotsAISubsystem(FakeBackend())
    assert ai.model == RECOGNITION_MODEL
    assert ai.uses_recognition


def test_recognition_bbox_is_normalized_from_pixel_center_and_size():
    # centred object, a quarter of the width and half the height
    ai, _, backend = _ai_with(
        [FakeRecognitionObject(center=(320, 200), size=(160, 200), model="ball")],
        width=640, height=400,
    )

    det = ai.get_detections()[0]

    assert det.label == "ball"
    assert det.confidence == pytest.approx(1.0)
    assert det.bbox.xmin == pytest.approx(0.375)   # (320 - 80) / 640
    assert det.bbox.xmax == pytest.approx(0.625)   # (320 + 80) / 640
    assert det.bbox.ymin == pytest.approx(0.25)    # (200 - 100) / 400
    assert det.bbox.ymax == pytest.approx(0.75)
    assert det.bbox.center == pytest.approx((0.5, 0.5))


def test_recognition_bbox_is_clamped_to_image():
    """Partly off-screen objects must not produce negative or >1 coordinates."""
    ai, _, backend = _ai_with(
        [FakeRecognitionObject(center=(10, 5), size=(100, 100), model="ball")],
        width=640, height=400,
    )

    box = ai.get_detections()[0].bbox
    assert box.xmin == 0.0
    assert box.ymin == 0.0
    assert 0.0 <= box.xmax <= 1.0
    assert box.width > 0


def test_recognition_accepts_snake_case_bindings():
    """Both Webots method spellings must work — see _call_first."""
    ai, _, backend = _ai_with(
        [SnakeCaseRecognitionObject(center=(320, 200), size=(64, 40), model="cup")],
        width=640, height=400,
    )

    det = ai.get_detections()[0]
    assert det.label == "cup"
    assert det.bbox.center == pytest.approx((0.5, 0.5))


def test_recognition_decodes_bytes_model_names():
    ai, _, backend = _ai_with(
        [FakeRecognitionObject(center=(320, 200), size=(64, 40), model=b"ball")],
        width=640, height=400,
    )
    assert ai.get_detections()[0].label == "ball"


def test_known_coco_label_gets_its_class_id():
    ai, _, backend = _ai_with(
        [FakeRecognitionObject(center=(320, 200), size=(64, 40), model="cup")],
        width=640, height=400,
    )
    assert ai.get_detections()[0].label_id == COCO_LABELS.index("cup")


def test_unknown_label_survives_with_negative_id():
    """A custom world label must not be silently renamed by COCO resolution."""
    ai, _, backend = _ai_with(
        [FakeRecognitionObject(center=(320, 200), size=(64, 40), model="testball")],
        width=640, height=400,
    )

    det = ai.get_detections()[0]
    assert det.label == "testball"
    assert det.label_id == -1


def test_results_are_stable_within_one_simulation_step():
    """Inference must not re-run while the image cannot have changed.

    Perception is keyed on frame_id, so moving an object without advancing
    simulated time is invisible — as it must be, since nothing can move
    between steps.
    """
    obj = FakeRecognitionObject(center=(200, 200), size=(80, 80), model="ball")
    ai, _, backend = _ai_with([obj], width=640, height=400)

    first = ai.get_detections()[0].bbox.center[0]
    obj._center = (440, 200)
    assert ai.get_detections()[0].bbox.center[0] == pytest.approx(first)


def test_head_turn_moves_target_across_the_image():
    """The closed loop, in miniature: a moving object shifts bbox.center[0]."""
    obj = FakeRecognitionObject(center=(200, 200), size=(80, 80), model="ball")
    ai, _, backend = _ai_with([obj], width=640, height=400)

    left_x = ai.get_detections(latest_only=True)[0].bbox.center[0]

    # What sim.step() does for a real controller: new time -> new frame.
    obj._center = (440, 200)
    backend.advance()

    right_x = ai.get_detections(latest_only=True)[0].bbox.center[0]

    assert right_x > left_x
    assert left_x == pytest.approx(200 / 640)
    assert right_x == pytest.approx(440 / 640)


def test_no_objects_yields_empty_list():
    ai, _, backend = _ai_with([])
    assert ai.get_detections() == []


def test_recognition_enable_uses_timestep_once():
    ai, device, backend = _ai_with([])
    assert device.recognition_enabled_with == 32

    device.recognition_enabled_with = "untouched"
    ai.get_detections()
    assert device.recognition_enabled_with == "untouched", "must not re-enable"


def test_missing_recognition_node_is_reported_not_raised():
    device = FakeCameraDevice(64, 64, bgra_bytes(64, 64, 0, 0, 0))
    device.has_recognition_flag = False
    ai = WebotsAISubsystem(FakeBackend(device))

    assert ai.set_model(RECOGNITION_MODEL) is False
    assert ai.get_detections() == []


def test_hand_and_pose_are_empty_in_simulation():
    ai, _, backend = _ai_with([])
    assert ai.get_hand_landmarks() == []
    assert ai.get_poses() == []


def test_stop_disables_recognition():
    ai, device, backend = _ai_with([])
    ai.stop()
    assert device.recognition_enabled_with is None


def test_metrics_are_populated_across_steps():
    """FPS counts inferences, so it only advances when frames do."""
    ai, _, backend = _ai_with([FakeRecognitionObject((10, 10), (4, 4), "ball")])

    for _ in range(4):
        ai.get_detections()
        backend.advance()
        time.sleep(0.002)      # give the wall-clock window a measurable span

    assert ai.avg_latency_ms >= 0.0
    assert ai.fps > 0.0


# ==================== live view on a Display ====================


class FakeDisplay:
    def __init__(self, width=320, height=200):
        self._w, self._h = width, height
        self.attached = None
        self.color = None
        self.alpha = 1.0
        self.rects = []
        self.texts = []
        self.ops = []                    # ordered log: ("clear"|"rect", ...)

    def getWidth(self): return self._w
    def getHeight(self): return self._h
    def attachCamera(self, cam): self.attached = cam
    def detachCamera(self): self.attached = None
    def setColor(self, c): self.color = c
    def setAlpha(self, a): self.alpha = a

    def fillRectangle(self, x, y, w, h):
        if self.alpha == 0.0 and (x, y, w, h) == (0, 0, self._w, self._h):
            self.ops.append(("clear",))

    def drawRectangle(self, x, y, w, h):
        self.rects.append((x, y, w, h))
        self.ops.append(("rect", x, y, w, h, self.alpha))

    def drawText(self, t, x, y): self.texts.append((t, x, y))


def _backend_with_display(display, cam_w=640, cam_h=400):
    device = FakeCameraDevice(cam_w, cam_h, bgra_bytes(cam_w, cam_h, 0, 0, 0))
    backend = FakeBackend(device)
    real_get = backend._robot.getDevice

    def get(name):
        return display if name == "camera_display" else real_get(name)

    backend._robot.getDevice = get
    return backend


def test_show_on_display_attaches_the_camera():
    d = FakeDisplay()
    cam = WebotsCameraSubsystem(_backend_with_display(d))

    assert cam.show_on_display() is True
    assert d.attached is cam.device
    assert cam.display is d


def test_show_on_display_reports_missing_device():
    backend = FakeBackend()          # getDevice("camera_display") -> None
    cam = WebotsCameraSubsystem(backend)
    assert cam.show_on_display() is False
    assert cam.display is None


def test_draw_detections_scales_boxes_to_display_pixels():
    d = FakeDisplay(width=320, height=200)
    cam = WebotsCameraSubsystem(_backend_with_display(d))
    cam.show_on_display()

    det = Detection(label_id=-1, confidence=0.5,
                    bbox=BoundingBox(0.25, 0.5, 0.75, 1.0), label="ball")
    cam.draw_detections([det])

    # 0.25..0.75 of 320 -> x 80, width 160;  0.5..1.0 of 200 -> y 100, height 100
    assert d.rects == [(80, 100, 160, 100)]
    assert d.texts and d.texts[0][0] == "ball 50%"


def test_draw_detections_keeps_label_inside_the_panel():
    d = FakeDisplay()
    cam = WebotsCameraSubsystem(_backend_with_display(d))
    cam.show_on_display()

    top = Detection(label_id=-1, confidence=1.0,
                    bbox=BoundingBox(0.0, 0.0, 0.1, 0.05), label="ball")
    cam.draw_detections([top])
    assert d.texts[0][2] >= 0, "label must not be drawn above the panel"


def test_draw_detections_erases_the_previous_boxes():
    """The overlay persists across steps; stale boxes smear as the ball moves.

    Webots keeps Display drawings on a layer above the attached camera image
    and never clears it, so every call must wipe that layer first — also when
    nothing was detected, or the last box stays behind.
    """
    d = FakeDisplay(width=320, height=200)
    cam = WebotsCameraSubsystem(_backend_with_display(d))
    cam.show_on_display()

    det = Detection(label_id=-1, confidence=1.0,
                    bbox=BoundingBox(0.25, 0.5, 0.75, 1.0), label="ball")
    cam.draw_detections([det])
    cam.draw_detections([])

    assert d.ops == [("clear",), ("rect", 80, 100, 160, 100, 1.0), ("clear",)]


def test_draw_detections_is_a_noop_without_a_display():
    cam = WebotsCameraSubsystem(FakeBackend())
    det = Detection(label_id=-1, confidence=1.0,
                    bbox=BoundingBox(0, 0, 1, 1), label="ball")
    cam.draw_detections([det])          # must not raise


def test_stop_detaches_the_display():
    d = FakeDisplay()
    cam = WebotsCameraSubsystem(_backend_with_display(d))
    cam.show_on_display()

    cam.stop()
    assert d.attached is None
    assert cam.display is None


def test_get_frame_survives_null_image_pointer():
    """Webots raises ValueError until the camera has rendered once.

    wb_camera_get_image() returns NULL in the step that enables the device,
    and the Python binding dereferences it unconditionally. get_frame() must
    treat that as "not ready", not propagate a crash.
    """
    class NotReadyCamera(FakeCameraDevice):
        def getImage(self):
            raise ValueError("NULL pointer access")

    cam = WebotsCameraSubsystem(FakeBackend(NotReadyCamera(4, 4)))
    assert cam.get_frame() is None
    assert cam.frame_count == 0


def test_get_frame_recovers_once_the_image_appears():
    class LateCamera(FakeCameraDevice):
        def __init__(self, *a, **kw):
            super().__init__(*a, **kw)
            self.ready = False

        def getImage(self):
            if not self.ready:
                raise ValueError("NULL pointer access")
            return self._image

    device = LateCamera(4, 3, bgra_bytes(4, 3, 10, 20, 30))
    backend = FakeBackend(device)
    cam = WebotsCameraSubsystem(backend)

    assert cam.get_frame() is None          # not rendered yet
    device.ready = True
    backend.advance()                        # a step happens
    frame = cam.get_frame()
    assert frame is not None
    assert tuple(frame.to_numpy()[0, 0]) == (10, 20, 30)


# ==================== novice safeguards ====================

from pib3.backends.hints import already_hinted, reset_hints


@pytest.fixture(autouse=True)
def _fresh_hints():
    reset_hints()
    yield
    reset_hints()


def test_hint_fires_once_only(caplog):
    from pib3.backends.hints import hint
    with caplog.at_level("WARNING"):
        hint("demo", "first")
        hint("demo", "second")
    assert caplog.text.count("pib3 hint") == 1


def test_reading_without_stepping_is_flagged(caplog):
    """The 'my loop does nothing' bug: same frame forever."""
    device = FakeCameraDevice(4, 4, bgra_bytes(4, 4, 1, 2, 3))
    cam = WebotsCameraSubsystem(FakeBackend(device))

    with caplog.at_level("WARNING"):
        for _ in range(WebotsCameraSubsystem.STUCK_READ_LIMIT + 5):
            cam.get_frame()          # never advancing sim time

    assert already_hinted("no-step-loop")
    assert "sim.step()" in caplog.text


def test_stepping_normally_never_triggers_the_hint():
    device = FakeCameraDevice(4, 4, bgra_bytes(4, 4, 1, 2, 3))
    backend = FakeBackend(device)
    cam = WebotsCameraSubsystem(backend)

    for _ in range(WebotsCameraSubsystem.STUCK_READ_LIMIT * 3):
        cam.get_frame()
        backend.advance()

    assert not already_hinted("no-step-loop"), "false positive on a healthy loop"


def test_a_few_reads_per_step_never_triggers_the_hint():
    """Polling camera + ai + a getter in one step must stay silent."""
    device = FakeCameraDevice(4, 4, bgra_bytes(4, 4, 1, 2, 3))
    backend = FakeBackend(device)
    cam = WebotsCameraSubsystem(backend)

    for _ in range(50):
        for _ in range(4):           # four reads per step
            cam.get_frame()
        backend.advance()

    assert not already_hinted("no-step-loop")


def test_multi_frame_read_is_flagged(caplog):
    """Forgetting latest_only returns several frames' worth of results."""
    obj = FakeRecognitionObject(center=(100, 100), size=(20, 20), model="ball")
    ai, _, backend = _ai_with([obj])

    with caplog.at_level("WARNING"):
        for _ in range(4):
            ai.get_detections()      # no latest_only
            backend.advance()

    assert already_hinted("stale-buffer")
    assert "latest_only=True" in caplog.text


def test_latest_only_reads_stay_silent():
    obj = FakeRecognitionObject(center=(100, 100), size=(20, 20), model="ball")
    ai, _, backend = _ai_with([obj])

    for _ in range(10):
        ai.get_detections(latest_only=True)
        backend.advance()

    assert not already_hinted("stale-buffer")


# ==================== sim model names ====================


def test_every_robot_model_id_resolves_in_sim():
    """Each AIModel either maps to host weights or fails with a clear error."""
    from pib3.backends.sim_ai import SIM_MODEL_ALIASES, UNSUPPORTED_IN_SIM
    from pib3.types import AIModel

    unresolved = [
        m.value for m in AIModel
        if m.value not in SIM_MODEL_ALIASES and m.value not in UNSUPPORTED_IN_SIM
    ]
    assert unresolved == []


def test_robot_models_run_yolo26_in_sim():
    from pib3.backends.sim_ai import SIM_MODEL_ALIASES
    from pib3.types import AIModel

    assert SIM_MODEL_ALIASES[AIModel.YOLO26S.value] == "yolo26s.pt"
    assert SIM_MODEL_ALIASES[AIModel.YOLO26N.value] == "yolo26n.pt"
    assert SIM_MODEL_ALIASES[AIModel.POSE_YOLO.value] == "yolo26s-pose.pt"
    assert SIM_MODEL_ALIASES[AIModel.POSE_YOLO26N.value] == "yolo26n-pose.pt"
    assert SIM_MODEL_ALIASES[AIModel.HAND.value] == "hand"
    assert SIM_MODEL_ALIASES["segmentation"] == "yolo26n-seg.pt"


@pytest.mark.parametrize("name", [m for m in
                                  ["face_detection_yunet_160x120", "facemesh_crop",
                                   "facial_landmarks_68_crop",
                                   "emotion_recognition_crop", "qr_code_detection_384x384",
                                   "person", "gaze"]])
def test_models_without_a_simulated_equivalent_say_so(name):
    from pib3.backends.sim_ai import build_runner

    with pytest.raises(ValueError, match="no simulated equivalent"):
        build_runner(name)


def test_deprecated_names_point_at_current_models():
    from pib3.types import AIModel, DEPRECATED_MODEL_ALIASES

    current = {m.value for m in AIModel}
    # "segmentation" is the one simulation-only name the aliases may produce.
    assert set(DEPRECATED_MODEL_ALIASES.values()) <= current | {"segmentation"}
    assert not set(DEPRECATED_MODEL_ALIASES) & current


@pytest.mark.parametrize("old", ["yolov6n", "yolov10n", "yolo11n", "yolov8n", "yolo26n"])
def test_old_yolo_names_resolve_to_yolo26n_with_a_warning(old):
    from pib3.types import AIModel, resolve_model_name

    with pytest.warns(DeprecationWarning, match=old):
        assert resolve_model_name(old) == AIModel.YOLO26N.value


@pytest.mark.parametrize("old,new", [
    ("pose_yolo", "POSE_YOLO"), ("pose", "POSE_YOLO"), ("pose_yolov8", "POSE_YOLO26N"),
    ("hand", "HAND"), ("face", "FACE"), ("yolo11s", "YOLO26S"), ("yolo26s", "YOLO26S"),
    ("mobilenet-ssd", "YOLO26S"),
])
def test_old_short_names_resolve_to_model_ids(old, new):
    from pib3.types import AIModel, resolve_model_name

    with pytest.warns(DeprecationWarning):
        assert resolve_model_name(old) == AIModel[new].value


def test_current_names_resolve_silently(recwarn):
    from pib3.types import AIModel, resolve_model_name

    assert resolve_model_name(AIModel.YOLO26N) == "yolo26n_coco_512x288"
    assert resolve_model_name(AIModel.YOLO26S) == "yolo26s_coco_512x288"
    assert resolve_model_name(AIModel.POSE_YOLO26S) == "yolo26s_pose_coco_512x288"
    assert resolve_model_name("yolov6n_coco_640x640") == "yolov6n_coco_640x640"
    assert resolve_model_name("recognition") == "recognition"
    assert not recwarn.list


def test_sim_set_model_resolves_old_names(monkeypatch):
    """sim.ai.set_model("yolov6n") loads YOLO26n and reports that id."""
    import pib3.backends.sim_ai as sim_ai
    from pib3.types import AIModel

    loaded = []

    class FakeRunner:
        def close(self):
            pass

    monkeypatch.setattr(sim_ai, "build_runner", lambda name: loaded.append(name) or FakeRunner())
    ai = WebotsAISubsystem(FakeBackend())
    with pytest.warns(DeprecationWarning):
        assert ai.set_model("yolov6n") is True
    assert loaded == [AIModel.YOLO26N.value]
    assert ai.model == AIModel.YOLO26N.value


# ==================== sim.ai has robot.ai's surface ====================


class _FakeRunner:
    """A model that finds one 'cup' in every frame."""

    def __init__(self, label="cup"):
        self.label, self.closed, self.calls = label, False, 0

    def infer(self, bgr):
        from pib3.backends.detection_messages import make_detection
        self.calls += 1
        h, w = bgr.shape[:2]
        return [make_detection(self.label, 0.9, (w * 0.1, h * 0.1, w * 0.3, h * 0.4))]

    def close(self):
        self.closed = True


def _runner_ai(monkeypatch, runners):
    """A sim AI whose build_runner hands out ``runners`` by model id."""
    import pib3.backends.sim_ai as sim_ai
    monkeypatch.setattr(sim_ai, "build_runner", lambda name: runners[name]
                        if not isinstance(runners[name], Exception) else (_ for _ in ()).throw(runners[name]))
    ai, device, backend = _ai_with([])
    return ai, backend


def test_sim_runs_several_models_at_once_and_reads_each_by_id(monkeypatch):
    from pib3.types import AIModel
    yolo, pose = _FakeRunner("cup"), _FakeRunner("person")
    ai, backend = _runner_ai(monkeypatch, {AIModel.YOLO26N.value: yolo,
                                           AIModel.POSE_YOLO.value: pose})

    assert ai.start_model(AIModel.YOLO26N) and ai.start_model(AIModel.POSE_YOLO)

    assert ai.models == ("recognition", AIModel.YOLO26N.value, AIModel.POSE_YOLO.value)
    assert ai.model == AIModel.POSE_YOLO.value
    assert [d.label for d in ai.get_detections(latest_only=True)] == ["person"]
    assert [d.label for d in ai.get_detections(latest_only=True, model=AIModel.YOLO26N)] == ["cup"]
    assert yolo.calls == 1 and pose.calls == 1     # one inference per model and frame


def test_sim_set_model_leaves_only_that_model(monkeypatch):
    from pib3.types import AIModel
    yolo = _FakeRunner()
    ai, _ = _runner_ai(monkeypatch, {AIModel.YOLO26N.value: yolo})

    assert ai.set_model(AIModel.YOLO26N)

    assert ai.models == (AIModel.YOLO26N.value,)


def test_sim_failed_switch_keeps_the_running_model(monkeypatch, caplog):
    """The old code closed the running model first and then failed to load."""
    from pib3.types import AIModel
    yolo = _FakeRunner()
    ai, _ = _runner_ai(monkeypatch, {AIModel.YOLO26N.value: yolo,
                                     AIModel.HAND.value: ImportError("no mediapipe")})
    ai.set_model(AIModel.YOLO26N)

    assert ai.set_model(AIModel.HAND) is False

    assert ai.models == (AIModel.YOLO26N.value,) and not yolo.closed
    assert ai.model == AIModel.YOLO26N.value
    assert len(ai.get_detections(latest_only=True)) == 1


def test_sim_stop_model_closes_it_and_hands_over_to_the_previous(monkeypatch):
    from pib3.types import AIModel
    yolo = _FakeRunner()
    ai, _ = _runner_ai(monkeypatch, {AIModel.YOLO26N.value: yolo})
    ai.start_model(AIModel.YOLO26N)

    assert ai.stop_model() is True

    assert yolo.closed and ai.model == RECOGNITION_MODEL
    assert ai.stop_model("never-started") is True        # like the robot


def test_sim_reading_with_no_model_is_an_error_like_on_the_robot():
    ai, _, _ = _ai_with([])
    ai.stop_model()

    with pytest.raises(RuntimeError, match="No AI model started"):
        ai.get_detections()


def test_sim_lists_what_it_can_run():
    from pib3.types import AIModel
    ai, _, _ = _ai_with([])

    models = {m.name: m for m in ai.available_models()}

    assert models[RECOGNITION_MODEL].active
    assert models[AIModel.YOLO26N.value].task == "object_detection" and not models[AIModel.YOLO26N.value].active
    assert AIModel.FACE.value not in models                 # no simulated equivalent


def test_the_small_yolo26_models_are_the_defaults():
    from pib3.types import AIModel

    assert AIModel.YOLO26S.value == "yolo26s_coco_512x288"
    assert AIModel.POSE_YOLO.value == "yolo26s_pose_coco_512x288"
    assert AIModel.POSE_YOLO26S is AIModel.POSE_YOLO       # one model, two names
    assert AIModel.POSE_YOLO26N.value == "yolo26n_pose_coco_512x288"
    # pib-backend withdrew these from the camera model list (PR-1957)
    assert "facemesh_crop" not in {m.value for m in AIModel}
    assert "facial_landmarks_68_crop" not in {m.value for m in AIModel}


def test_simulated_pose_keeps_each_keypoints_confidence():
    """ultralytics reports a confidence per keypoint; a hidden one is low."""
    from types import SimpleNamespace

    import numpy as np

    from pib3.backends.camera import parse_detection_message
    from pib3.backends.detection_messages import COCO_KEYPOINT_NAMES, make_detection_array
    from pib3.backends.sim_ai import UltralyticsPose

    class Tensor:
        def __init__(self, value):
            self.value = np.asarray(value, dtype=float)

        def cpu(self):
            return self

        def numpy(self):
            return self.value

        def tolist(self):
            return self.value.tolist()

        def __getitem__(self, index):
            return Tensor(self.value[index])

        def __float__(self):
            return float(self.value)

        def __int__(self):
            return int(self.value)

    conf = [0.99] * 15 + [0.02, 0.01]            # both ankles hidden
    box = SimpleNamespace(xyxy=Tensor([[10, 20, 110, 220]]), cls=Tensor([0]),
                          conf=Tensor([0.9]))
    result = SimpleNamespace(
        names={0: "person"}, boxes=[box],
        keypoints=SimpleNamespace(xy=Tensor([[[50, 60]] * 17]), conf=Tensor([conf])),
    )
    runner = UltralyticsPose.__new__(UltralyticsPose)
    runner._predict = lambda bgr: [result]

    [detection] = runner.infer(np.zeros((240, 320, 3), np.uint8))
    [pose] = parse_detection_message(make_detection_array("pose", [detection], 320, 240))

    assert detection["keypoint_names"] == list(COCO_KEYPOINT_NAMES)
    assert detection["keypoint_score"] == pytest.approx(conf)
    assert pose.keypoints[16].confidence == pytest.approx(0.01)
    assert pose.nose.confidence == pytest.approx(0.99)


def test_the_medium_models_run_only_in_the_simulation():
    from pib3.backends.sim_ai import SIM_MODEL_ALIASES, simulated_models
    from pib3.types import LAPTOP_ONLY_MODELS, AIModel

    assert SIM_MODEL_ALIASES[AIModel.YOLO26M.value] == "yolo26m.pt"
    assert SIM_MODEL_ALIASES[AIModel.POSE_YOLO26M.value] == "yolo26m-pose.pt"
    assert LAPTOP_ONLY_MODELS == {AIModel.YOLO26M.value, AIModel.POSE_YOLO26M.value}
    assert simulated_models()[AIModel.YOLO26M.value] == "object_detection"


def test_weights_are_found_offline_before_ultralytics_downloads(tmp_path, monkeypatch):
    from pib3.backends import sim_ai

    folder, script_dir, cwd = (tmp_path / name for name in ("weights", "script", "cwd"))
    for path in (folder, script_dir, cwd):
        path.mkdir()
    monkeypatch.setattr(sim_ai.sys, "argv", [str(script_dir / "run.py")])
    monkeypatch.chdir(cwd)
    monkeypatch.delenv(sim_ai.WEIGHTS_DIR_VARIABLE, raising=False)

    assert sim_ai.resolve_weights("yolo26s.pt") == "yolo26s.pt"     # nothing local
    (cwd / "yolo26s.pt").write_bytes(b"x")
    assert sim_ai.resolve_weights("yolo26s.pt") == str(cwd / "yolo26s.pt")
    (script_dir / "yolo26s.pt").write_bytes(b"x")
    assert sim_ai.resolve_weights("yolo26s.pt") == str(script_dir / "yolo26s.pt")
    (folder / "yolo26s.pt").write_bytes(b"x")
    monkeypatch.setenv(sim_ai.WEIGHTS_DIR_VARIABLE, str(folder))
    assert sim_ai.resolve_weights("yolo26s.pt") == str(folder / "yolo26s.pt")
    assert sim_ai.resolve_weights("/elsewhere/yolo26s.pt") == "/elsewhere/yolo26s.pt"


def test_inference_ms_times_the_model_after_a_warm_up(monkeypatch):
    from pib3.backends import sim_ai

    calls = []

    class Runner:
        def infer(self, image):
            calls.append(image.shape)
            return []

        def close(self):
            calls.append("closed")

    monkeypatch.setattr(sim_ai, "build_runner", lambda name: Runner())

    ms = sim_ai.inference_ms("yolo26s_coco_512x288", repeats=3)

    assert ms >= 0.0
    assert calls == [(360, 640, 3)] * 4 + ["closed"]      # warm-up + 3 timed
