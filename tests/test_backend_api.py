"""Behaviour of the motion API and the backends' internals, without hardware.

Covers the fixes that are easy to break again and hard to notice on a robot:
argument validation for novices, clamping, speed handling, the ROS message
layout the pib backend actually parses, the robot's own invert/range
settings in direct mode, the emergency freeze, and the Webots twin.
"""

import math
import threading
import time
import types

import numpy as np
import pytest

from pib3 import Joint, Trajectory
from pib3.backends.base import normalize_unit
from pib3.backends.camera import AIDetectionReceiver
from pib3.backends.hints import already_hinted, reset_hints
from pib3.backends.robot import (
    RealRobotBackend,
    joint_trajectory_message,
    parse_robot_motor_config,
)
from pib3.backends.webots import WebotsBackend
from pib3.image import _normalize_coordinates
from pib3.trajectory import _interpolate_stroke_points, _within_motor_limits, _wrap_angles
from pib3.types import Stroke

from .test_safety import MemoryBackend


@pytest.fixture
def robot():
    reset_hints()
    backend = MemoryBackend()
    yield backend
    backend._deactivate_safety()


# ==================== arguments ====================


def test_typo_in_joint_name_suggests_the_right_one(robot):
    with pytest.raises(ValueError, match="elbow_left"):
        robot.set_joint("elbow_lft", 50.0, async_=True)


def test_enum_member_names_are_accepted(robot):
    robot.set_joint("TURN_HEAD", 50.0, async_=True)
    robot.set_joint("index_left", 50.0, async_=True)
    sent = [list(p) for p, _ in robot.sent]
    assert sent == [["turn_head_motor"], ["index_left_stretch"]]


def test_unit_spellings():
    assert normalize_unit("degrees") == "deg"
    assert normalize_unit("°") == "deg"
    assert normalize_unit("%") == "percent"
    assert normalize_unit("Radians") == "rad"
    with pytest.raises(ValueError, match="unit"):
        normalize_unit("grad")


def test_unknown_unit_no_longer_falls_back_to_percent(robot):
    with pytest.raises(ValueError):
        robot.set_joint(Joint.ELBOW_LEFT, 30.0, unit="degre", async_=True)
    with pytest.raises(ValueError):
        robot.get_joint(Joint.ELBOW_LEFT, unit="degre")


def test_non_numbers_are_rejected(robot):
    with pytest.raises(ValueError):
        robot.set_joint(Joint.ELBOW_LEFT, float("nan"), async_=True)
    with pytest.raises(TypeError):
        robot.set_joint(Joint.ELBOW_LEFT, "50", async_=True)


def test_speed_zero_is_not_full_speed(robot):
    with pytest.raises(ValueError, match="positive"):
        robot.set_joint(Joint.ELBOW_LEFT, 50.0, speed=0, async_=True)
    with pytest.raises(ValueError):
        robot.default_speed = -10


def test_get_joints_accepts_a_single_joint(robot):
    assert list(robot.get_joints(Joint.ELBOW_LEFT)) == ["elbow_left"]


# ==================== clamping and speed ====================


def test_out_of_range_is_clamped_with_a_hint(robot):
    robot.default_speed = 1000.0
    ok = robot.set_joint(Joint.ELBOW_LEFT, 120.0)
    (positions, _), = robot.sent
    assert positions["elbow_left"] == pytest.approx(math.pi / 2)
    assert ok is False                       # not where it was asked to go
    assert already_hinted("clamp-elbow_left")


def test_default_speed_is_sent_with_every_command(robot):
    robot.set_joint(Joint.ELBOW_LEFT, 10.0, async_=True)
    robot.go_home(async_=True)                 # slow, explicit homing speed
    robot.set_joint(Joint.ELBOW_LEFT, 20.0, async_=True)
    velocities = [v for _, v in robot.sent]
    assert velocities[0] == 9000               # MemoryBackend.DEFAULT_SPEED
    assert velocities[-1] == 9000, "go_home's slow speed must not stick"


def test_slow_blocking_move_is_not_reported_as_failure(robot):
    assert robot.set_joint(Joint.ELBOW_LEFT, -40.0, unit="deg", speed=1000.0)
    # 120 deg at 50 deg/s = 2.4 s: the old fixed 2 s timeout returned False.
    assert robot.set_joint(Joint.ELBOW_LEFT, 80.0, unit="deg", speed=50.0)


def test_run_trajectory_first_moves_to_the_start(robot):
    traj = Trajectory(joint_names=["elbow_left"], waypoints=np.array([[0.3], [0.31], [0.32]]))
    robot.default_speed = 1000.0
    assert robot.run_trajectory(traj, rate_hz=200, approach_speed=500.0)
    approach, velocity = robot.sent[0]
    assert approach == {"elbow_left": pytest.approx(0.3)}
    assert velocity == 50000
    assert len(robot.executed) == 3


def test_run_trajectory_clamps_out_of_range_waypoints(robot):
    traj = Trajectory(joint_names=["elbow_left"], waypoints=np.array([[0.0], [3.0]]))
    robot.run_trajectory(traj, rate_hz=200, approach_speed=0)
    assert robot.executed[-1][0] == pytest.approx(math.pi / 2)


# ==================== real robot: ROS message layout ====================


def test_ros_message_has_one_point_per_joint():
    msg = joint_trajectory_message(["a", "b", "c"], [100, 200, 300])
    jt = msg["joint_trajectory"]
    assert jt["joint_names"] == ["a", "b", "c"]
    assert [p["positions"] for p in jt["points"]] == [[100.0], [200.0], [300.0]]


class FakeService:
    def __init__(self):
        self.requests = []

    def call(self, request, timeout=None):
        self.requests.append(dict(request))
        return {"successful": True}


def ros_robot(monkeypatch):
    import pib3.backends.robot as robot_module
    monkeypatch.setattr(robot_module.roslibpy, "ServiceRequest", lambda d: d)
    r = RealRobotBackend(host="test", motor_mode="ros", estop_keys=False, stop_button=False)
    r._client = types.SimpleNamespace(is_connected=True)
    r._service = FakeService()
    return r


def test_ros_mode_sends_all_joints_in_one_call(monkeypatch):
    r = ros_robot(monkeypatch)
    assert r.set_joints({"elbow_left": 0.5, "wrist_left": -0.2}, unit="rad", async_=True)
    (request,) = r._service.requests
    jt = request["joint_trajectory"]
    assert jt["joint_names"] == ["elbow_left", "wrist_left"]
    assert [p["positions"][0] for p in jt["points"]] == [
        round(math.degrees(0.5) * 100), round(math.degrees(-0.2) * 100)]


def test_ros_mode_trajectory_moves_every_joint(monkeypatch):
    r = ros_robot(monkeypatch)
    waypoints = np.array([[100, 200], [110, 210]])
    assert r._execute_waypoints(["elbow_left", "wrist_left"], waypoints, 200.0, None)
    for request in r._service.requests:
        points = request["joint_trajectory"]["points"]
        assert len(points) == 2, "one point per joint, or the robot moves only the first"


# ==================== real robot: direct mode ====================


class FakeServo:
    def __init__(self):
        self.position = {}
        self.ramp = {}
        self.motion = {}
        self.enabled = {}
        self.writes = []

    def set_enable(self, ch, on):
        self.enabled[ch] = on

    def get_enabled(self, ch):
        return self.enabled.get(ch, False)

    def get_motion_configuration(self, ch):
        v, a, d = self.motion.get(ch, (15000, 15000, 15000))
        return types.SimpleNamespace(velocity=v, acceleration=a, deceleration=d)

    def set_motion_configuration(self, ch, v, a, d):
        self.motion[ch] = (v, a, d)
        self.writes.append(("motion", ch, (v, a, d)))

    def set_position(self, ch, pos):
        self.position[ch] = pos
        self.writes.append(("position", ch, pos))

    def get_current_position(self, ch):
        return self.ramp.get(ch, self.position.get(ch, 0))


def direct_robot(table=None):
    """A robot in direct mode with fake bricklets.

    Returns (robot, servo) where ``servo`` is the bricklet of the left arm
    (U1 without a table, B0 with one: the first ten motors of the table).
    """
    r = RealRobotBackend(host="test", estop_keys=False, stop_button=False)
    r._client = types.SimpleNamespace(is_connected=True)
    r._tinkerforge_conn = object()
    if table is not None:
        r._apply_robot_motor_config(table)
        r._tinkerforge_servos = {uid: FakeServo() for uid in ("B0", "B1", "B2")}
        return r, r._tinkerforge_servos["B0"]
    servo = FakeServo()
    r._tinkerforge_servos = {"U1": servo}
    r._tinkerforge_motor_map = {"elbow_left": ("U1", 8), "wrist_left": ("U1", 6)}
    return r, servo


def full_table(**overrides):
    motors = []
    for i, name in enumerate(RealRobotBackend.MOTOR_NAMES):
        motor = {"name": name, "invert": False, "rotationRangeMin": -9000,
                 "rotationRangeMax": 9000,
                 "brickletPins": [{"pin": i % 10, "invert": False, "bricklet": f"B{i // 10}"}]}
        motor.update(overrides.get(name, {}))
        motors.append(motor)
    return parse_robot_motor_config({"motors": motors})


def test_parse_robot_motor_config_skips_junk():
    table = parse_robot_motor_config({"motors": [
        {"name": "elbow_left", "invert": True, "rotationRangeMin": -4500,
         "rotationRangeMax": 9000, "brickletPins": [{"pin": 8, "invert": False, "bricklet": "2cPQ"}]},
        {"name": "wrist_left", "brickletPins": [{"pin": 6, "bricklet": None}]},
        "garbage",
    ]})
    assert table["elbow_left"] == {"pins": [("2cPQ", 8, False)], "invert": True,
                                   "range": (-4500, 9000)}
    assert table["wrist_left"]["pins"] == []


def test_complete_robot_table_gives_the_exact_pin_map():
    r, _ = direct_robot(full_table())
    assert r._tinkerforge_motor_map["turn_head_motor"] == ("B0", 0)
    assert len(r._tinkerforge_motor_map) == 26


def test_incomplete_table_keeps_auto_discovery_for_pins():
    table = full_table(elbow_left={"brickletPins": []})
    r, _ = direct_robot(None)
    r._tinkerforge_motor_map = {}
    r._apply_robot_motor_config(table)
    assert r._tinkerforge_motor_map == {}          # pins left to auto-discovery
    assert "elbow_left" in r._motor_range_cd       # ranges still used


def test_direct_mode_applies_motor_invert_and_range():
    table = full_table(
        elbow_left={"invert": True},
        tilt_forward_motor={"rotationRangeMin": -4500, "rotationRangeMax": 4500},
        wrist_left={"brickletPins": [{"pin": 3, "invert": True, "bricklet": "B0"}]},
    )
    r, _ = direct_robot(table)

    def written(name):
        uid, pin = r._tinkerforge_motor_map[name]
        return r._tinkerforge_servos[uid].position[pin]

    r.set_joint(Joint.ELBOW_LEFT, 30.0, unit="deg", async_=True)
    assert written("elbow_left") == -3000
    r.set_joint(Joint.TILT_HEAD, 60.0, unit="deg", async_=True)
    assert written("tilt_forward_motor") == 4500   # robot's own range, not +60
    r.set_joint(Joint.WRIST_LEFT, 20.0, unit="deg", async_=True)
    assert r._tinkerforge_servos["B0"].position[3] == -2000   # pin-level invert


def test_direct_reads_undo_the_invert():
    r, _ = direct_robot(full_table(elbow_left={"invert": True}))
    uid, pin = r._tinkerforge_motor_map["elbow_left"]
    r._tinkerforge_servos[uid].ramp[pin] = -2500
    assert r.get_joint(Joint.ELBOW_LEFT, unit="deg") == pytest.approx(25.0)


def test_direct_commands_apply_their_speed_once():
    r, servo = direct_robot()
    r.set_joint(Joint.ELBOW_LEFT, 10.0, unit="deg", async_=True, speed=20)
    r.set_joint(Joint.ELBOW_LEFT, 12.0, unit="deg", async_=True, speed=20)
    motion_writes = [w for w in servo.writes if w[0] == "motion"]
    assert motion_writes == [("motion", 8, (2000, 15000, 15000))]


def test_emergency_freeze_holds_every_channel_without_overshoot():
    r, servo = direct_robot()
    r.set_joint(Joint.ELBOW_LEFT, 80.0, unit="deg", async_=True)
    servo.ramp[8] = 3100                           # half way there
    servo.writes.clear()
    r.stop(reason="test")
    assert ("motion", 8, (15000, 15000, 0)) in servo.writes   # no braking ramp
    assert servo.position[8] == 3100                           # target := here
    with pytest.raises(Exception):
        r.set_joint(Joint.ELBOW_LEFT, 10.0, async_=True)
    # After resume the next command restores the normal ramp.
    r.resume()
    r.set_joint(Joint.ELBOW_LEFT, 10.0, unit="deg", async_=True)
    assert servo.motion[8] == (15000, 15000, 15000)


def test_remote_stop_latches(monkeypatch):
    r, _ = direct_robot()
    r._on_remote_estop({"data": '{"action": "stop", "source": "teacher"}'})
    deadline = time.monotonic() + 2
    while not r.stopped and time.monotonic() < deadline:
        time.sleep(0.01)
    assert r.stopped and "teacher" in r.stop_reason


def test_bad_constructor_arguments():
    with pytest.raises(ValueError):
        RealRobotBackend(motor_mode="tinkerforge")
    with pytest.raises(ValueError):
        RealRobotBackend(stop_button="yes")


# ==================== Webots twin ====================


class FakeSensor:
    def __init__(self):
        self.value = 0.0

    def getValue(self):
        return self.value


class FakeMotor:
    def __init__(self, vmax=20.0):
        self.vmax = vmax
        self.sensor = FakeSensor()
        self.target = None
        self.velocity = None

    def getMinPosition(self):
        return -3.0

    def getMaxPosition(self):
        return 3.0

    def getMaxVelocity(self):
        return self.vmax

    def setVelocity(self, v):
        self.velocity = v

    def setPosition(self, p):
        self.target = p

    def getPositionSensor(self):
        return self.sensor


class FakeWebotsRobot:
    def __init__(self):
        self.steps = 0

    def step(self, ms):
        self.steps += 1
        return 0


def webots(realistic=True):
    sim = WebotsBackend(realistic_motion=realistic)
    sim._robot = FakeWebotsRobot()
    sim._timestep = 32
    sim._motors = {"elbow_left": FakeMotor(), "wrist_left": FakeMotor(vmax=1.0)}
    return sim


def test_webots_honours_speed_and_caps_at_motor_maximum():
    sim = webots()
    sim.set_joints({"elbow_left": 0.5, "wrist_left": 0.5}, unit="rad", async_=True, speed=90)
    assert sim._motors["elbow_left"].velocity == pytest.approx(math.radians(90))
    assert sim._motors["wrist_left"].velocity == pytest.approx(1.0)


def test_webots_default_speed_matches_the_real_robot():
    sim = webots()
    sim.set_joint(Joint.ELBOW_LEFT, 0.5, unit="rad", async_=True)
    assert sim._motors["elbow_left"].velocity == pytest.approx(math.radians(150))
    assert WebotsBackend.DEFAULT_SPEED == RealRobotBackend.DEFAULT_SPEED


def test_webots_can_keep_the_old_instant_motors():
    sim = webots(realistic=False)
    sim.set_joint(Joint.ELBOW_LEFT, 0.5, unit="rad", async_=True)
    assert sim._motors["elbow_left"].velocity == pytest.approx(20.0)


def test_webots_stop_from_another_thread_is_applied_on_the_main_thread():
    sim = webots()
    sim._motors["elbow_left"].sensor.value = 0.25
    sim.set_joint(Joint.ELBOW_LEFT, 1.0, unit="rad", async_=True)
    t = threading.Thread(target=sim.stop)
    t.start()
    t.join()
    assert sim._pending_halt
    assert sim._motors["elbow_left"].target == pytest.approx(1.0)   # untouched yet
    sim.step()
    assert sim._motors["elbow_left"].target == pytest.approx(0.25)  # frozen here
    assert not sim._pending_halt


# ==================== perception, image, IK ====================


def test_segmentation_results_are_detections():
    receiver = AIDetectionReceiver()
    receiver.on_detection({
        "model": "segmentation", "type": "instance-segmentation", "latency_ms": 5,
        "result": {"detections": [{"label": 41, "confidence": 0.8,
                                   "bbox": {"xmin": 0.1, "ymin": 0.1, "xmax": 0.3, "ymax": 0.4}}]},
    })
    (det,) = receiver.get_detections(timeout=0)
    assert det.label == "cup"


def test_wide_images_keep_their_proportions():
    contour = np.array([[0.0, 0.0], [200.0, 100.0]])
    ((pts, _),) = _normalize_coordinates([(contour, True)], 200, 100, margin=0.0)
    width, height = pts[1] - pts[0]
    assert width == pytest.approx(2 * height)
    assert pts[:, 1].mean() == pytest.approx(0.5)            # centred


def test_closed_strokes_are_closed():
    square = Stroke(points=np.array([[0, 0], [1, 0], [1, 1], [0, 1]], dtype=float), closed=True)
    path = _interpolate_stroke_points(square, density=0.25)
    assert path[0] == pytest.approx(path[-1])
    assert len(path) == len({tuple(np.round(p, 9)) for p in path}) + 1   # no duplicates


def test_ik_rejects_angles_the_servos_cannot_reach():
    names = ["shoulder_vertical_left", "shoulder_horizontal_left", "upper_arm_left_rotation",
             "elbow_left", "lower_arm_left_rotation", "wrist_left"]
    assert _within_motor_limits(np.zeros(6), names)
    assert not _within_motor_limits(np.array([0, 0, 0, -1.0, 0, 0]), names)   # elbow < -45 deg
    assert _wrap_angles(np.array([2 * np.pi + 0.1]))[0] == pytest.approx(0.1)


def test_webots_instant_read_does_not_step():
    sim = webots()
    sim._motors["elbow_left"].sensor.value = 0.3
    steps = sim._robot.steps
    assert sim.get_joint(Joint.ELBOW_LEFT, unit="rad", timeout=0) == pytest.approx(0.3)
    assert sim.get_joints([Joint.ELBOW_LEFT], unit="rad", timeout=0) == {
        "elbow_left": pytest.approx(0.3)}
    assert sim._robot.steps == steps
