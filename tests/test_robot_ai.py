"""The robot's AI interface: pib-backend's model store, without a robot.

The camera node runs models on request (``/start_model`` with an owner name)
and publishes each model's results as ``datatypes/DetectionArray`` on
``detections/<topic>``. These tests pin down how pib3 asks for models and how
it turns the messages into typed objects.

Where the messages come from, since that decides what the tests prove:

- ``tests/data/detection_arrays``: YOLO26n and YOLO26n-pose results from
  ultralytics on one photo each, run through pib-backend's own
  ``translate_detections`` (so field names, pixel mapping and keypoint naming
  are the backend's code). They are not packets captured on the camera; the
  same models were checked on the OAK-D separately.
- ``hand_message()`` below: built from reading the backend's hand code
  (``label "hand"``, 21 named keypoints, four scalars). No real hand message
  was captured, so how pib3 reads a hand from a robot is untested against one.
"""

import json
import logging
import time
import types
from pathlib import Path

import numpy as np
import pytest

from pib3.backends import robot as robot_module
from pib3.backends.camera import (
    AIDetectionReceiver,
    AIModelInfo,
    AISubsystem,
    Detection,
    Handedness,
    HandLandmarks,
    PoseKeypoints,
    parse_detection_message,
)
from pib3.backends.detection_messages import (
    COCO_KEYPOINT_NAMES,
    HAND_KEYPOINT_NAMES,
    detection_topic,
    make_detection,
    make_detection_array,
)
from pib3.backends.robot import RealRobotBackend
from pib3.types import AIModel, ImuType

DATA = Path(__file__).parent / "data" / "detection_arrays"


def load(name):
    return json.loads((DATA / f"{name}.json").read_text())


def hand_message(landmarks_px, handedness=0.73, frame=(1280, 720), model_id="hand_tracking_mp"):
    """A hand the way the backend publishes it (label, 21 names, four scalars)."""
    xs = [x for x, _ in landmarks_px]
    ys = [y for _, y in landmarks_px]
    detection = make_detection(
        label="hand",
        score=0.96,
        box=(min(xs), min(ys), max(xs), max(ys)),
        keypoints=[(n, x, y) for n, (x, y) in zip(HAND_KEYPOINT_NAMES, landmarks_px)],
        scalars={"handedness": handedness, "palm_score": 0.9, "landmark_score": 0.96,
                 "z_source": 1.0},
    )
    return make_detection_array(model_id, [detection], *frame)


def open_hand(wrist=(640.0, 600.0), scale=1.0):
    """21 landmarks of a flat hand pointing up: every finger straight."""
    points = [wrist]
    x0, y0 = wrist
    # thumb leans left, the four fingers fan out slightly
    for dx, step in [(-0.5, 55)] + [(d, 80) for d in (-0.25, 0.0, 0.2, 0.4)]:
        base_x = x0 + dx * 120 * scale
        for k in range(1, 5 if dx == -0.5 else 5):
            points.append((base_x + dx * 20 * k * scale, y0 - (100 + step * (k - 1)) * scale))
    return points[:21]


def curled_hand(wrist=(640.0, 600.0)):
    """The fingers folded back onto the palm: the tip points down again."""
    points = open_hand(wrist)
    for finger in range(1, 5):
        base = 1 + (finger - 1) * 4 + 4  # skip the thumb's four points
        mcp, pip = points[base], points[base + 1]
        points[base + 2] = (pip[0], pip[1] + 40)   # dip below the pip
        points[base + 3] = (pip[0], pip[1] + 75)   # tip even lower
    return points


# ==================== messages -> typed objects ====================


def test_detector_message_becomes_normalized_detections():
    message = load("yolo26n_coco_512x288")

    detections = [Detection.from_message(d, message["frame_width"], message["frame_height"])
                  for d in message["detections"]]

    assert {d.label for d in detections} == {"person", "bus"}
    bus = next(d for d in detections if d.label == "bus")
    assert bus.label_id == 5
    assert bus.confidence == pytest.approx(0.83, abs=0.01)
    assert (bus.bbox.xmin, bus.bbox.ymin) == pytest.approx((14 / 1280, 152 / 720))
    assert all(0.0 <= v <= 1.0 for d in detections
               for v in (d.bbox.xmin, d.bbox.ymin, d.bbox.xmax, d.bbox.ymax))
    assert detections[0].keypoints == [] and detections[0].scalars == {}


def test_pose_message_orders_keypoints_by_coco_index_and_names_them():
    message = load("yolo26n_pose_coco_512x288")
    [det] = message["detections"]
    # a backend that listed the keypoints in another order must still work
    order = list(reversed(range(17)))
    for key in ("keypoint_names", "keypoint_x", "keypoint_y", "keypoint_z"):
        det[key] = [det[key][i] for i in order]

    pose = PoseKeypoints.from_message(det, message["frame_width"], message["frame_height"])

    assert [kp.name for kp in pose.keypoints] == list(COCO_KEYPOINT_NAMES)
    assert pose.nose.name == "nose"
    assert 0.0 < pose.left_shoulder.x < 1.0 and 0.0 < pose.left_shoulder.y < 1.0
    assert pose.bbox.width > 0.3  # a person fills much of this frame
    assert pose.confidence == pytest.approx(0.88, abs=0.02)


def test_message_with_typed_objects_picks_the_right_class():
    pose = parse_detection_message(load("yolo26n_pose_coco_512x288"))
    boxes = parse_detection_message(load("yolo26n_coco_512x288"))
    hands = parse_detection_message(hand_message(open_hand()))

    assert [type(o) for o in pose] == [PoseKeypoints]
    assert {type(o) for o in boxes} == {Detection}
    assert [type(o) for o in hands] == [HandLandmarks]


def test_messages_without_a_frame_size_are_skipped():
    message = load("yolo26n_coco_512x288")
    message["frame_width"] = 0  # the service answers so before the first frame

    assert parse_detection_message(message) == []


def test_hand_message_gives_handedness_and_landmarks():
    [hand] = parse_detection_message(hand_message(open_hand(), handedness=0.73))
    [left] = parse_detection_message(hand_message(open_hand(), handedness=0.2))

    assert hand.handedness is Handedness.RIGHT
    assert left.handedness is Handedness.LEFT
    assert hand.landmarks.shape == (21, 2)
    assert hand.confidence == pytest.approx(0.96)
    assert hand.keypoints[8].name == "index_finger_tip"
    assert hand.finger_angles.index < 15 and hand.finger_angles.middle < 15


def test_curled_fingers_measure_as_bent():
    [hand] = parse_detection_message(hand_message(curled_hand()))

    assert hand.finger_angles.index > 150
    assert hand.finger_angles.middle > 150


def test_finger_angles_are_measured_in_pixels_not_normalized_units():
    """On a 16:9 frame a right angle in the image must read as 90 degrees."""
    points = open_hand()
    # index finger: proximal segment up-right, distal segment down-right,
    # perpendicular in the image (100 px, -100 px) and (100 px, 100 px)
    points[5], points[6] = (500.0, 500.0), (600.0, 400.0)
    points[7], points[8] = (600.0, 400.0), (700.0, 500.0)

    [hand] = parse_detection_message(hand_message(points))

    assert hand.finger_angles.index == pytest.approx(90.0, abs=0.5)


def test_make_detection_round_trips_through_the_parser():
    det = make_detection("cup", 0.5, (128, 72, 256, 144),
                         keypoints=[("a", 640, 360)], scalars={"yaw": 12.5})
    message = make_detection_array("m", [det], 1280, 720)

    [parsed] = parse_detection_message(message)

    assert parsed.label == "cup" and parsed.label_id == 41
    assert parsed.bbox.center == pytest.approx((0.15, 0.15))
    assert (parsed.keypoints[0].x, parsed.keypoints[0].y) == (0.5, 0.5)
    assert parsed.scalars == {"yaw": 12.5}


# ==================== the receiver ====================


def test_receiver_returns_typed_results_and_the_latest_frame_only():
    receiver = AIDetectionReceiver()
    first = load("yolo26n_coco_512x288")
    second = load("yolo26n_coco_512x288")
    second["detections"] = second["detections"][:1]
    receiver.on_detection(first)
    receiver.on_detection(second)

    assert len(receiver.get_detections(timeout=0)) == len(first["detections"]) + 1
    assert len(receiver.get_detections(timeout=0, latest_only=True)) == 1
    assert receiver.get_poses(timeout=0) == [] and receiver.get_hand_landmarks(timeout=0) == []


def test_receiver_ignores_other_models_on_a_shared_topic():
    """Both hand chains publish on detections/hand_tracking."""
    receiver = AIDetectionReceiver()
    receiver.expect_model("hand_tracking_mp")
    receiver.on_detection(hand_message(open_hand(), model_id="hand_tracking"))
    assert receiver.result_count == 0

    receiver.on_detection(hand_message(open_hand(), model_id="hand_tracking_mp"))
    assert receiver.result_count == 1
    assert len(receiver.get_hand_landmarks(timeout=0)) == 1


def test_latency_is_the_message_age_when_the_clocks_agree():
    receiver = AIDetectionReceiver()
    receiver.on_detection(make_detection_array("m", [], 1280, 720, stamp=time.time() - 0.080))

    assert receiver.avg_latency_ms == pytest.approx(80, abs=30)


@pytest.mark.parametrize("offset", [-5.0, 4000.0])
def test_latency_ignores_stamps_from_an_unsynchronised_clock(offset):
    receiver = AIDetectionReceiver()
    receiver.on_detection(make_detection_array("m", [], 1280, 720, stamp=time.time() - offset))

    assert receiver.avg_latency_ms == 0.0


def test_receiver_measures_fps():
    receiver = AIDetectionReceiver()
    for _ in range(5):
        receiver.on_detection(make_detection_array("m", [], 1280, 720))
        time.sleep(0.02)

    assert 10 < receiver.fps < 100


def test_a_message_without_a_model_id_is_kept():
    receiver = AIDetectionReceiver()
    receiver.expect_model("yolo26n_coco_512x288")
    message = load("yolo26n_coco_512x288")
    del message["model_id"]

    receiver.on_detection(message)

    assert receiver.result_count == 1


# ==================== robot services and topics ====================


class FakeTopic:
    created = []

    def __init__(self, client, name, message_type):
        self.name, self.message_type = name, message_type
        self.callback = None
        self.subscribed = False
        self.published = []
        FakeTopic.created.append(self)

    def subscribe(self, callback):
        self.callback, self.subscribed = callback, True

    def unsubscribe(self):
        self.subscribed = False

    def publish(self, message):
        self.published.append(message)


class FakeService:
    handlers = {}
    calls = []

    def __init__(self, client, name, service_type):
        self.name, self.service_type = name, service_type

    def call(self, request, timeout=None):
        FakeService.calls.append((self.name, self.service_type, dict(request), timeout))
        handler = FakeService.handlers.get(self.name)
        if handler is None:
            raise RuntimeError("service not available")
        return handler(dict(request))


@pytest.fixture
def robot(monkeypatch):
    FakeTopic.created, FakeService.calls, FakeService.handlers = [], [], {}
    fake_roslibpy = types.SimpleNamespace(
        Topic=FakeTopic, Service=FakeService, ServiceRequest=dict,
    )
    monkeypatch.setattr(robot_module, "roslibpy", fake_roslibpy)
    backend = RealRobotBackend(host="robot.local", motor_mode="ros")
    backend._client = types.SimpleNamespace(is_connected=True)
    return backend


def topics(name):
    return [t for t in FakeTopic.created if t.name == name]


def test_default_owner_is_stable_and_named_after_user_and_computer(robot):
    other = RealRobotBackend(host="elsewhere", motor_mode="ros")

    assert robot.ai_owner.startswith("pib3-") and "@" in robot.ai_owner
    assert robot.ai_owner == other.ai_owner  # a rerun after a crash reuses it
    assert RealRobotBackend(ai_owner="group-3", motor_mode="ros").ai_owner == "group-3"
    with pytest.raises(ValueError):
        robot.ai_owner = "  "


def test_list_models_becomes_a_dict_keyed_by_model_id(robot):
    FakeService.handlers["/list_models"] = lambda request: {"models": [
        {"model_id": "yolo26n_coco_512x288", "task": "object_detection", "licence": "AGPL-3.0",
         "shaves": 4, "size_bytes": 5472216, "available": True, "active": False},
        {"model_id": "hand_tracking_mp", "task": "hand_tracking", "licence": "x",
         "shaves": 8, "size_bytes": 1, "available": False, "active": False},
    ]}

    models = robot.get_available_ai_models()

    assert FakeService.calls[0][:2] == ("/list_models", "datatypes/srv/ListModels")
    assert models["yolo26n_coco_512x288"]["shaves"] == 4
    assert "model_id" not in models["yolo26n_coco_512x288"]
    info = AIModelInfo.from_dict("hand_tracking_mp", models["hand_tracking_mp"])
    assert info.available is False and info.task == "hand_tracking"


def test_start_and_stop_send_the_model_id_and_the_owner(robot):
    FakeService.handlers["/start_model"] = lambda r: {"success": True, "message": "started"}
    FakeService.handlers["/stop_model"] = lambda r: {"success": True, "message": "stopped"}

    assert robot.start_ai_model(AIModel.POSE_YOLO, timeout=12) == (True, "started")
    assert robot.stop_ai_model("yolo26n_pose_coco_512x288") == (True, "stopped")

    start, stop = FakeService.calls
    assert start == ("/start_model", "datatypes/srv/StartModel",
                     {"model_id": "yolo26n_pose_coco_512x288", "shaves": 0,
                      "owner": robot.ai_owner}, 12)
    assert stop[2] == {"model_id": "yolo26n_pose_coco_512x288", "owner": robot.ai_owner}


def test_start_reports_why_the_robot_refused(robot):
    FakeService.handlers["/start_model"] = lambda r: {
        "success": False, "message": "Unknown model: yolo26n_coco_512x288"}

    ok, message = robot.start_ai_model(AIModel.YOLO26N)

    assert ok is False and "Unknown model" in message


def test_a_dead_service_is_a_failed_start_not_an_exception(robot):
    robot._MODEL_POLL_INTERVAL = 0.01

    ok, message = robot.start_ai_model(AIModel.YOLO26N, timeout=0.05)

    assert ok is False and "/start_model" in message


def test_a_start_that_outlasts_rosbridges_timeout_counts_if_the_model_runs(robot):
    """A rebuild takes longer than rosbridge waits for a service answer."""
    robot._MODEL_POLL_INTERVAL = 0.01
    listed = iter([False, False, True])    # /list_models: becomes active
    FakeService.handlers["/list_models"] = lambda r: {"models": [
        {"model_id": "yolo26n_coco_512x288", "task": "object_detection", "licence": "",
         "shaves": 4, "size_bytes": 1, "available": True, "active": next(listed)}]}
    # /start_model has no handler: the call raises, as roslibpy does on a timeout

    ok, message = robot.start_ai_model(AIModel.YOLO26N, timeout=5)

    assert ok is True and "no answer" in message


def test_a_start_that_got_no_answer_and_never_runs_is_a_failure(robot):
    robot._MODEL_POLL_INTERVAL = 0.01
    FakeService.handlers["/list_models"] = lambda r: {"models": [
        {"model_id": "yolo26n_coco_512x288", "available": True, "active": False}]}

    ok, message = robot.start_ai_model(AIModel.YOLO26N, timeout=0.1)

    assert ok is False


def test_a_refusal_is_not_second_guessed_by_polling(robot):
    """An answered "no" stays "no", even if /list_models would say active."""
    FakeService.handlers["/start_model"] = lambda r: {"success": False, "message": "Model is unavailable"}

    ok, message = robot.start_ai_model(AIModel.YOLO26N)

    assert (ok, message) == (False, "Model is unavailable")
    assert [c[0] for c in FakeService.calls] == ["/start_model"]


def test_old_names_are_resolved_before_they_reach_the_robot(robot):
    FakeService.handlers["/start_model"] = lambda r: {"success": True, "message": ""}

    with pytest.warns(DeprecationWarning):
        robot.start_ai_model("yolov6n")

    assert FakeService.calls[0][2]["model_id"] == "yolo26n_coco_512x288"


@pytest.mark.parametrize("model,topic", [
    (AIModel.YOLO26N, "/detections/yolo26n_coco_512x288"),
    (AIModel.POSE_YOLO, "/detections/yolo26n_pose_coco_512x288"),
    (AIModel.HAND, "/detections/hand_tracking"),          # shared hand topic
    (AIModel.FACE, "/detections/face_detection_yunet_160x120"),
    ("yolov6n_coco_640x640", "/detections/yolov6n_coco_640x640"),  # any model id
])
def test_results_arrive_on_the_models_topic(robot, model, topic):
    got = []

    sub = robot.subscribe_ai_detections(model, got.append)

    assert sub.name == topic and sub.message_type == "datatypes/msg/DetectionArray"
    sub.callback({"model_id": "x"})
    assert got == [{"model_id": "x"}]


def test_detection_topics_follow_the_backends_naming():
    assert detection_topic("facemesh_crop") == "detections/facemesh_crop"
    assert detection_topic("hand_tracking") == detection_topic("hand_tracking_mp")


def test_get_detections_service_returns_the_message(robot):
    message = load("yolo26n_coco_512x288")
    FakeService.handlers["/get_detections"] = lambda r: {"detections": message}

    assert robot.get_ai_detections(AIModel.YOLO26N) == message
    assert FakeService.calls[0][2] == {"model_id": "yolo26n_coco_512x288"}


def test_status_topic(robot):
    sub = robot.subscribe_ai_status(lambda msg: None)

    assert (sub.name, sub.message_type) == ("/models_status", "datatypes/msg/ModelStatusArray")


# ==================== robot.ai ====================


class FakeRobot:
    """What AISubsystem uses of the backend, recording what it is asked."""

    is_connected = True

    def __init__(self, refuse=(), offered=None):
        self.refuse = set(refuse)
        self.offered = offered or {}
        self.calls = []
        self.subscriptions = {}

    def resolve_ai_model_name(self, model):
        return model.value if isinstance(model, AIModel) else model

    def start_ai_model(self, model, shaves=0, timeout=30.0):
        self.calls.append(("start", model))
        if model in self.refuse:
            return False, f"Unknown model: {model}"
        return True, "started"

    def stop_ai_model(self, model, timeout=30.0):
        self.calls.append(("stop", model))
        return True, "stopped"

    def subscribe_ai_detections(self, model, callback):
        sub = types.SimpleNamespace(callback=callback, subscribed=True)
        sub.unsubscribe = lambda: setattr(sub, "subscribed", False)
        self.subscriptions[model] = sub
        return sub

    def get_available_ai_models(self, timeout=5.0):
        return self.offered


def test_set_model_stops_the_old_model_then_starts_the_new_one():
    robot = FakeRobot()
    ai = AISubsystem(robot)

    assert ai.set_model(AIModel.HAND)
    assert ai.set_model(AIModel.YOLO26N)

    assert robot.calls == [
        ("start", "hand_tracking_mp"),
        ("stop", "hand_tracking_mp"),
        ("start", "yolo26n_coco_512x288"),
    ]
    assert ai.model == "yolo26n_coco_512x288" and ai.models == ("yolo26n_coco_512x288",)
    assert robot.subscriptions["hand_tracking_mp"].subscribed is False


def test_setting_the_running_model_again_does_not_restart_the_camera_twice():
    robot = FakeRobot()
    ai = AISubsystem(robot)
    ai.set_model(AIModel.YOLO26N)

    ai.set_model(AIModel.YOLO26N)

    assert [c[0] for c in robot.calls] == ["start", "start"]  # the robot says "already"


def test_start_model_keeps_other_models_running():
    robot = FakeRobot()
    ai = AISubsystem(robot)

    ai.start_model(AIModel.YOLO26N)
    ai.start_model(AIModel.HAND)

    assert not any(c[0] == "stop" for c in robot.calls)
    assert set(ai.models) == {"yolo26n_coco_512x288", "hand_tracking_mp"}
    assert ai.model == "hand_tracking_mp"


def test_results_reach_the_receiver_of_their_model():
    robot = FakeRobot()
    ai = AISubsystem(robot)
    ai.start_model(AIModel.YOLO26N)
    ai.start_model(AIModel.POSE_YOLO)

    robot.subscriptions["yolo26n_coco_512x288"].callback(load("yolo26n_coco_512x288"))
    robot.subscriptions["yolo26n_pose_coco_512x288"].callback(load("yolo26n_pose_coco_512x288"))

    assert ai.get_detections(timeout=0, model=AIModel.YOLO26N)[0].label in ("person", "bus")
    assert len(ai.get_poses(timeout=0)) == 1      # the current model is the pose model
    assert ai.get_poses(timeout=0, model=AIModel.YOLO26N) == []


def test_a_refused_model_is_reported_with_what_the_robot_offers(caplog):
    robot = FakeRobot(refuse={"yolo26n_coco_512x288"},
                      offered={"yolov6n_coco_640x640": {"available": True},
                               "hand_tracking_mp": {"available": False}})
    ai = AISubsystem(robot)

    with caplog.at_level(logging.WARNING):
        assert ai.set_model(AIModel.YOLO26N) is False

    assert "Unknown model" in caplog.text and "yolov6n_coco_640x640" in caplog.text
    assert "hand_tracking_mp" not in caplog.text          # unavailable: not offered
    assert ai.model is None and ai.models == ()
    assert robot.subscriptions["yolo26n_coco_512x288"].subscribed is False


def test_an_exception_while_starting_leaves_no_half_started_model():
    robot = FakeRobot()

    def dropped(*args, **kwargs):
        raise ConnectionError("rosbridge gone")
    robot.start_ai_model = dropped
    ai = AISubsystem(robot)

    with pytest.raises(ConnectionError):
        ai.start_model(AIModel.YOLO26N)

    assert ai.models == () and ai.model is None
    assert robot.subscriptions["yolo26n_coco_512x288"].subscribed is False


def test_reading_before_any_model_is_an_error_not_an_empty_list():
    ai = AISubsystem(FakeRobot())

    with pytest.raises(RuntimeError, match="No AI model started"):
        ai.get_detections(timeout=0)


def test_stop_releases_every_model_and_unsubscribes():
    robot = FakeRobot()
    ai = AISubsystem(robot)
    ai.start_model(AIModel.YOLO26N)
    ai.start_model(AIModel.HAND)

    ai.stop()

    assert sorted(c[1] for c in robot.calls if c[0] == "stop") == [
        "hand_tracking_mp", "yolo26n_coco_512x288"]
    assert all(not s.subscribed for s in robot.subscriptions.values())
    assert ai.models == () and ai.model is None


def test_stop_survives_a_lost_connection():
    robot = FakeRobot()
    ai = AISubsystem(robot)
    ai.start_model(AIModel.YOLO26N)

    def broken(*args, **kwargs):
        raise ConnectionError("gone")
    robot.stop_ai_model = broken

    ai.stop()  # must not raise on disconnect

    assert ai.models == ()


def test_set_ai_model_on_the_backend_is_robot_ai_set_model(robot):
    FakeService.handlers["/start_model"] = lambda r: {"success": True, "message": ""}
    FakeService.handlers["/stop_model"] = lambda r: {"success": True, "message": ""}

    assert robot.set_ai_model(AIModel.HAND)
    assert robot.set_ai_model(AIModel.YOLO26N)

    assert [(c[0], c[2]["model_id"]) for c in FakeService.calls] == [
        ("/start_model", "hand_tracking_mp"),
        ("/stop_model", "hand_tracking_mp"),
        ("/start_model", "yolo26n_coco_512x288"),
    ]


# ==================== image, IMU and camera settings ====================


def test_camera_image_comes_from_camera_topic_as_base64_jpeg(robot):
    import base64
    got = []

    sub = robot.subscribe_camera_image(got.append)

    assert (sub.name, sub.message_type) == ("/camera_topic", "std_msgs/msg/String")
    sub.callback({"data": base64.b64encode(b"\xff\xd8jpeg").decode()})
    sub.callback({"data": ""})
    assert got == [b"\xff\xd8jpeg"]


def test_camera_settings_use_the_nodes_own_topics(robot):
    robot.set_camera_config(fps=10, quality=70, resolution=(640, 360))

    published = {t.name: t.published[0] for t in FakeTopic.created}
    assert published["/quality_factor_topic"] == {"data": 70}
    assert published["/size_topic"]["data"] == [640, 360]
    assert published["/timer_period_topic"] == {"data": pytest.approx(0.1)}


def test_imu_comes_from_one_topic_at_the_fixed_rate(robot):
    imu = {"header": {"frame_id": "oak_imu_frame"},
           "linear_acceleration": {"x": 0.1, "y": 0.2, "z": 9.8},
           "angular_velocity": {"x": 0.0, "y": 0.0, "z": 0.5}}
    got = {}

    for kind in ("full", "accelerometer", ImuType.GYROSCOPE):
        key = kind.value if isinstance(kind, ImuType) else kind
        sub = robot.subscribe_imu(lambda d, key=key: got.setdefault(key, d), data_type=kind)
        assert (sub.name, sub.message_type) == ("/imu", "sensor_msgs/msg/Imu")
        sub.callback(imu)

    assert got["full"]["linear_acceleration"]["z"] == 9.8
    assert got["full"]["angular_velocity"]["z"] == 0.5
    assert got["accelerometer"]["vector"] == imu["linear_acceleration"]
    assert got["gyroscope"]["vector"] == imu["angular_velocity"]
    with pytest.raises(ValueError):
        robot.subscribe_imu(print, data_type="magnetometer")


def test_the_imu_rate_cannot_be_set(robot):
    with pytest.raises(NotImplementedError, match="100 Hz"):
        robot.set_imu_frequency(50)
