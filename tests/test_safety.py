"""Emergency stop ("Notaus"), without a robot.

The stop must work on every laptop and must really stop: freeze the motors,
latch, and refuse further motion until a deliberate resume(). These tests
pin that down with an in-memory backend and stand-ins for pynput keys.
"""

import signal
import sys
import textwrap
import threading
import time
import types

import numpy as np
import pytest

from pib3 import EmergencyStopError, Joint
from pib3.backends.base import RobotBackend
from pib3.backends.hints import reset_hints
from pib3.safety import (
    DEFAULT_STOP_KEYS,
    KeyboardHook,
    SigintGuard,
    StopButton,
    coerce_keys,
    key_names,
    keyboard_hook_problem,
    normalize_key_name,
    parse_estop_message,
)


# ==================== an in-memory robot ====================


class MemoryBackend(RobotBackend):
    """Joints that move towards their target at ``speed`` deg/s in real time."""

    MOTOR_NAMES = RobotBackend.MOTOR_NAMES
    DEFAULT_SPEED = 90.0
    VERIFY_MIN_SETTLE_SECONDS = 0.0

    def __init__(self):
        super().__init__()
        self.connected = True
        self.target = {n: 0.0 for n in self.MOTOR_NAMES}
        self.start = dict(self.target)
        self.t0 = time.monotonic()
        self.speed_rad = np.radians(90.0)
        self.sent = []
        self.halts = 0
        self.executed = []

    # --- motion model ---
    def _pos(self, name):
        t = time.monotonic() - self.t0
        a, b = self.start[name], self.target[name]
        step = self.speed_rad * t
        return b if abs(b - a) <= step else a + np.sign(b - a) * step

    def _set_joints_impl(self, positions_radians, velocity_centideg=None):
        now = {n: self._pos(n) for n in self.MOTOR_NAMES}
        self.start, self.t0 = now, time.monotonic()
        self.target.update(positions_radians)
        if velocity_centideg is not None:
            self.speed_rad = np.radians(velocity_centideg / 100.0)
        self.sent.append((dict(positions_radians), velocity_centideg))
        return True

    def _halt_motion(self):
        self.halts += 1
        now = {n: self._pos(n) for n in self.MOTOR_NAMES}
        self.start, self.target, self.t0 = now, dict(now), time.monotonic()

    def _get_joint_radians(self, motor_name, timeout=None):
        return self._pos(motor_name)

    def _get_joints_radians(self, motor_names=None, timeout=None):
        return {n: self._pos(n) for n in (motor_names or self.MOTOR_NAMES)}

    def _execute_waypoints(self, joint_names, waypoints, rate_hz, progress_callback):
        for i, row in enumerate(waypoints):
            if self._stopped:
                return False
            self.executed.append(list(row))
            if self._stop_event.wait(1.0 / rate_hz):
                return False
        return True

    def _to_backend_format(self, radians):
        return radians

    def _from_backend_format(self, values):
        return values

    def connect(self):
        self.connected = True

    def disconnect(self):
        self._deactivate_safety()
        self.connected = False

    @property
    def is_connected(self):
        return self.connected


@pytest.fixture
def robot():
    reset_hints()
    backend = MemoryBackend()
    yield backend
    backend._deactivate_safety()


# ==================== keys ====================


def key(**attrs):
    return types.SimpleNamespace(**attrs)


def test_space_and_esc_match_on_every_platform():
    for platform in ("win32", "darwin", "linux"):
        assert "space" in key_names(key(name="space"), platform)
        assert "esc" in key_names(key(name="esc"), platform)


def test_numpad_zero_matches_per_platform():
    # What pynput really delivers for Numpad-0 on each system.
    assert "kp_0" in key_names(key(vk=96, char="0"), "win32")
    assert "kp_0" in key_names(key(vk=82, char="0"), "darwin")
    assert "kp_0" in key_names(key(vk=None, char="0"), "linux")
    # NumLock off: Windows and X11 report Insert.
    assert "kp_0" in key_names(key(name="insert"), "win32")


def test_keys_that_used_to_trigger_by_accident_no_longer_do():
    # The old KeyCode.from_vk(96) matched F5 on macOS (vk 96) ...
    assert "kp_0" not in key_names(key(vk=96, char=None), "darwin")
    # ... and the top-row 0 must not count as Numpad-0 on Linux.
    assert "kp_0" not in key_names(key(vk=48, char="0"), "linux")


def test_key_name_aliases():
    assert normalize_key_name("Escape") == "esc"
    assert normalize_key_name("KP_0") == "kp_0"
    assert normalize_key_name("numpad0") == "kp_0"
    assert normalize_key_name("F12") == "f12"
    assert normalize_key_name(" ") == "space"
    assert normalize_key_name("q") == "q"
    with pytest.raises(ValueError):
        normalize_key_name("hyper_meta")


def test_default_keys_include_laptop_keys():
    assert {"space", "esc"} <= set(DEFAULT_STOP_KEYS)
    assert coerce_keys(None) == DEFAULT_STOP_KEYS
    assert coerce_keys("F12") == ("f12",)
    assert coerce_keys(False) == ()


def test_hook_problems_are_detected_before_starting():
    assert keyboard_hook_problem("linux", {"XDG_SESSION_TYPE": "wayland",
                                           "DISPLAY": ":0"})
    assert keyboard_hook_problem("linux", {}) is not None          # no display
    assert keyboard_hook_problem("linux", {"DISPLAY": ":0",
                                           "XDG_SESSION_TYPE": "x11"}) is None
    assert keyboard_hook_problem("win32", {}) is None


# ==================== latch ====================


def test_stop_freezes_and_latches(robot):
    robot.stop(reason="test")
    assert robot.stopped and robot.halts == 1
    assert robot.stop_reason == "test"
    with pytest.raises(EmergencyStopError, match="resume"):
        robot.set_joint(Joint.ELBOW_LEFT, 60.0, async_=True)
    with pytest.raises(EmergencyStopError):
        robot.go_home(async_=True)
    with pytest.raises(EmergencyStopError):
        robot.set_joints_sequence([{Joint.ELBOW_LEFT: 10.0}])


def test_resume_is_needed_and_enough(robot):
    robot.stop()
    robot.stop()            # a second press does NOT resume
    assert robot.stopped
    robot.resume()
    assert robot.set_joint(Joint.ELBOW_LEFT, 40.0, async_=True)


def test_stop_ends_a_blocking_move_at_once(robot):
    robot.default_speed = 5.0           # a long, slow move
    timer = threading.Timer(0.2, robot.stop, kwargs={"reason": "key"})
    timer.start()
    t0 = time.monotonic()
    reached = robot.set_joint(Joint.ELBOW_LEFT, 100.0)
    elapsed = time.monotonic() - t0
    timer.join()
    assert reached is False
    assert elapsed < 1.0, "the wait must end with the stop, not with the timeout"
    frozen = robot.get_joint(Joint.ELBOW_LEFT, unit="deg")
    time.sleep(0.15)
    assert robot.get_joint(Joint.ELBOW_LEFT, unit="deg") == pytest.approx(frozen)


def test_stop_ends_a_sequence(robot):
    timer = threading.Timer(0.15, robot.stop)
    timer.start()
    ok = robot.set_joints_sequence(
        [{Joint.ELBOW_LEFT: float(v)} for v in range(0, 100, 5)], rate_hz=10)
    timer.join()
    assert ok is False
    assert len(robot.sent) < 10


def test_run_trajectory_refuses_while_stopped(robot):
    from pib3 import Trajectory
    traj = Trajectory(joint_names=["elbow_left"], waypoints=np.zeros((3, 1)))
    robot.stop()
    with pytest.raises(EmergencyStopError):
        robot.run_trajectory(traj)


def test_crash_inside_with_block_freezes_motors(robot):
    with pytest.raises(ZeroDivisionError):
        with robot:
            robot.set_joint(Joint.ELBOW_LEFT, 90.0, async_=True)
            1 / 0
    assert robot.halts == 1


def test_normal_exit_lets_motion_finish(robot):
    with robot:
        robot.set_joint(Joint.ELBOW_LEFT, 90.0, async_=True)
    assert robot.halts == 0


# ==================== keyboard hook ====================


class FakeListener(threading.Thread):
    instances = []

    def __init__(self, on_press):
        super().__init__(daemon=True)
        self.on_press = on_press
        self._ready = False
        self._stop_flag = threading.Event()
        FakeListener.instances.append(self)

    def run(self):
        self._ready = True
        self._stop_flag.wait()

    def stop(self):
        self._stop_flag.set()


@pytest.fixture
def fake_pynput(monkeypatch):
    FakeListener.instances.clear()
    keyboard = types.SimpleNamespace(Listener=FakeListener)
    pynput = types.ModuleType("pynput")
    pynput.keyboard = keyboard
    monkeypatch.setitem(sys.modules, "pynput", pynput)
    monkeypatch.setitem(sys.modules, "pynput.keyboard", keyboard)
    monkeypatch.setattr("pib3.backends.base.keyboard_hook_problem", lambda: None)
    yield
    KeyboardHook._subscribers.clear()
    if KeyboardHook._listener is not None:
        KeyboardHook._listener.stop()
    KeyboardHook._listener = None


def test_space_key_stops_and_second_press_does_not_resume(robot, fake_pynput):
    assert robot.enable_estop_key() is True
    listener = FakeListener.instances[-1]
    listener.on_press(key(name="space"))
    deadline = time.monotonic() + 2
    while not robot.stopped and time.monotonic() < deadline:
        time.sleep(0.01)
    assert robot.stopped and "Space" in robot.stop_reason
    listener.on_press(key(name="space"))
    time.sleep(0.1)
    assert robot.stopped


def test_other_keys_are_ignored(robot, fake_pynput):
    robot.enable_estop_key()
    FakeListener.instances[-1].on_press(key(char="a", vk=65))
    time.sleep(0.1)
    assert not robot.stopped


def test_two_robots_share_one_hook(fake_pynput):
    a, b = MemoryBackend(), MemoryBackend()
    a.enable_estop_key()
    b.enable_estop_key(["f12"])
    assert len(FakeListener.instances) == 1
    FakeListener.instances[0].on_press(key(name="f12"))
    time.sleep(0.2)
    assert b.stopped and not a.stopped
    a.disable_estop_key()
    b.disable_estop_key()
    assert KeyboardHook._listener is None


# ==================== Ctrl+C ====================


def test_ctrl_c_freezes_before_the_interrupt(robot):
    previous = signal.getsignal(signal.SIGINT)
    try:
        assert SigintGuard.add(robot._estop_token, lambda: robot.stop(reason="Ctrl+C"))
        with pytest.raises(KeyboardInterrupt):
            SigintGuard._handler(signal.SIGINT, None)
        deadline = time.monotonic() + 2
        while not robot.stopped and time.monotonic() < deadline:
            time.sleep(0.01)
        assert robot.stopped and robot.stop_reason == "Ctrl+C"
    finally:
        SigintGuard.remove(robot._estop_token)
    assert signal.getsignal(signal.SIGINT) == previous


# ==================== remote stop ====================


def test_remote_stop_messages():
    assert parse_estop_message({"data": '{"action": "stop", "source": "Herr A"}'}) == "Herr A"
    assert parse_estop_message({"data": ""}) == "remote"
    assert parse_estop_message({"data": '{"action": "resume"}'}) is None


# ==================== on-screen button (protocol only) ====================


def test_stop_button_protocol(tmp_path):
    script = tmp_path / "fake_button.py"
    script.write_text(textwrap.dedent("""
        import sys
        print("READY", flush=True)
        print("STOP", flush=True)
        for line in sys.stdin:
            print("GOT " + line.strip(), file=sys.stderr, flush=True)
    """))
    hits = []
    button = StopButton(on_stop=lambda: hits.append(1), title="t", script=str(script))
    assert button.start(timeout=10)
    deadline = time.monotonic() + 5
    while not hits and time.monotonic() < deadline:
        time.sleep(0.02)
    assert hits == [1]
    button.notify_stopped("x")
    button.close()
    assert not button.running


def test_stop_button_reports_a_failing_window(tmp_path):
    script = tmp_path / "broken.py"
    script.write_text('print("ERROR tkinter is not available", flush=True)\n')
    button = StopButton(on_stop=lambda: None, script=str(script))
    assert button.start(timeout=10) is False
    assert "tkinter" in button.failure


# ==================== arming ====================


def test_connecting_arms_nothing_until_the_first_move(robot, monkeypatch):
    robot._activate_safety()                      # what connect() does
    assert not robot.estop_armed
    robot.get_joints()                            # reading does not arm
    assert not robot.estop_armed
    robot.set_joint(Joint.ELBOW_LEFT, 20.0, async_=True)
    assert robot.estop_armed


def test_observer_crash_does_not_freeze_other_programs_motors(robot):
    with pytest.raises(ZeroDivisionError):
        with robot:
            robot.get_joints()                    # camera/observer use only
            1 / 0
    assert robot.halts == 0


def test_button_opens_by_itself_when_keys_cannot_work(robot, monkeypatch, caplog):
    opened = []
    monkeypatch.setattr("pib3.backends.base.keyboard_hook_problem",
                        lambda: "this desktop runs Wayland")
    monkeypatch.setattr(KeyboardHook, "subscribe", classmethod(lambda cls, *a: None))
    monkeypatch.setattr(StopButton, "start", lambda self, timeout=8.0, wait=True: opened.append(wait) or True)
    monkeypatch.setattr(StopButton, "close", lambda self: None)
    robot._estop_keys_setting = True
    robot._estop_button_setting = "auto"
    robot.set_joint(Joint.ELBOW_LEFT, 20.0, async_=True)
    assert opened == [True]          # only way besides Ctrl+C: wait for it
    banner = [r.getMessage() for r in caplog.records
              if r.getMessage().startswith("Emergency stop armed:")]
    assert banner and "Space" not in banner[-1] and "STOP button" in banner[-1]
    robot._deactivate_safety()


def test_no_button_when_keys_work(robot, monkeypatch):
    opened = []
    monkeypatch.setattr("pib3.backends.base.keyboard_hook_problem", lambda: None)
    monkeypatch.setattr(KeyboardHook, "subscribe", classmethod(lambda cls, *a: None))
    monkeypatch.setattr(KeyboardHook, "is_subscribed", classmethod(lambda cls, t: True))
    monkeypatch.setattr(StopButton, "start", lambda self, timeout=8.0, wait=True: opened.append(wait) or True)
    robot._estop_keys_setting = True
    robot._estop_button_setting = "auto"
    robot.set_joint(Joint.ELBOW_LEFT, 20.0, async_=True)
    assert opened == []
    assert robot.estop_keys == DEFAULT_STOP_KEYS
    robot._deactivate_safety()


# ==================== teacher tool ====================


def test_freeze_servo_stops_without_braking_ramp():
    from pib3.safety import freeze_servo

    class Servo:
        def __init__(self):
            self.calls = []

        def get_enabled(self, ch):
            return True

        def get_motion_configuration(self, ch):
            return types.SimpleNamespace(velocity=15000, acceleration=15000, deceleration=15000)

        def set_motion_configuration(self, ch, v, a, d):
            self.calls.append(("motion", v, a, d))

        def get_current_position(self, ch):
            return 1234

        def set_position(self, ch, p):
            self.calls.append(("position", p))

        def set_enable(self, ch, on):
            self.calls.append(("enable", on))

    servo = Servo()
    assert freeze_servo(servo, 3)
    assert servo.calls == [("motion", 15000, 15000, 0), ("position", 1234)]
    relaxed = Servo()
    freeze_servo(relaxed, 3, relax=True)
    assert relaxed.calls == [("enable", False)]


def test_estop_cli_stops_every_robot(monkeypatch, capsys):
    from pib3.tools import estop

    frozen, latched = [], []
    monkeypatch.setattr(estop, "freeze_robot_servos",
                        lambda host, relax=False: frozen.append((host, relax)) or 26)
    monkeypatch.setattr(estop, "broadcast_stop",
                        lambda host, source="teacher": latched.append(host) or True)
    assert estop.main(["--host", "pib-01", "--host", "pib-02"]) == 0
    assert sorted(frozen) == [("pib-01", False), ("pib-02", False)]
    assert sorted(latched) == ["pib-01", "pib-02"]
    assert "26 servo channels frozen" in capsys.readouterr().out


def test_estop_cli_reports_an_unreachable_robot(monkeypatch, capsys):
    from pib3.tools import estop

    def unreachable(host, relax=False):
        raise OSError("connection refused")

    monkeypatch.setattr(estop, "freeze_robot_servos", unreachable)
    monkeypatch.setattr(estop, "broadcast_stop", lambda host, source="teacher": False)
    assert estop.main(["--host", "pib-03"]) == 1
    assert "FAILED" in capsys.readouterr().out


def test_stop_button_names_the_old_tk_build(tmp_path):
    script = tmp_path / "crash.py"
    script.write_text(
        "import sys\n"
        "sys.stderr.write('[xcb] Unknown sequence number while appending request\\n')\n"
        "sys.stderr.write(\"python: xcb_io.c:166: append_pending_request: Assertion failed.\\n\")\n"
        "sys.exit(134)\n")
    button = StopButton(on_stop=lambda: None, script=str(script))
    assert button.start(timeout=10) is False
    assert "uv python upgrade" in button.failure


def test_real_ctrl_c_still_cancels_the_program(tmp_path):
    """Ctrl+C keeps its meaning (KeyboardInterrupt); pib3 only freezes first."""
    import os
    import subprocess

    script = tmp_path / "probe.py"
    script.write_text(textwrap.dedent("""
        import os, signal, sys, threading, time
        from tests.test_safety import MemoryBackend
        from pib3 import Joint
        robot = MemoryBackend()
        if sys.argv[1] == "moved":
            robot.set_joint(Joint.ELBOW_LEFT, 80.0, async_=True)
        threading.Timer(0.3, os.kill, args=(os.getpid(), signal.SIGINT)).start()
        try:
            time.sleep(10)
            print("NOT INTERRUPTED")
        except KeyboardInterrupt:
            time.sleep(0.3)
            print(f"INTERRUPTED halts={robot.halts}")
    """))
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    env = dict(os.environ, PYTHONPATH=os.pathsep.join([root] + sys.path))
    for mode, halts in (("moved", 1), ("observer", 0)):
        out = subprocess.run([sys.executable, str(script), mode], cwd=root, env=env,
                             capture_output=True, text=True, timeout=30).stdout
        assert f"INTERRUPTED halts={halts}" in out, out



def test_the_button_shows_the_stop_is_armed_and_lists_what_works(robot, monkeypatch):
    started = []
    monkeypatch.setattr("pib3.backends.base.keyboard_hook_problem", lambda: None)
    monkeypatch.setattr(KeyboardHook, "subscribe", classmethod(lambda cls, *a: None))
    monkeypatch.setattr(KeyboardHook, "is_subscribed", classmethod(lambda cls, t: True))
    monkeypatch.setattr(StopButton, "start",
                        lambda self, timeout=8.0, wait=True: started.append((self._triggers, wait)) or True)
    monkeypatch.setattr(StopButton, "close", lambda self: None)
    robot._estop_keys_setting = True
    robot._estop_button_setting = True
    robot.get_joints()
    assert started == []                          # not armed: no window
    robot.set_joint(Joint.ELBOW_LEFT, 20.0, async_=True)
    (triggers, wait), = started
    assert triggers == ["click", "space", "esc", "ctrl_c"]
    assert wait is False                          # keys work: do not delay the move
    robot._deactivate_safety()


def test_window_language_follows_the_system():
    from pib3.safety import ui_language
    assert ui_language({"PIB3_LANG": "de"}, "linux") == "de"
    assert ui_language({"PIB3_LANG": "en", "LANG": "de_DE.UTF-8"}, "linux") == "en"
    assert ui_language({"LANG": "de_DE.UTF-8"}, "linux") == "de"
    assert ui_language({"LANG": "en_US.UTF-8"}, "linux") in ("en", "de")   # getlocale may add more
    assert ui_language({"LC_ALL": "German_Germany.1252"}, "win32") == "de"


def test_stop_window_script_texts_exist_for_every_trigger():
    from pib3.tools.stop_button import TEXTE
    for lang in TEXTE.values():
        for token in ("click", "space", "space3d", "esc", "ctrl_c", "scharf", "gestoppt"):
            assert token in lang



def test_german_window_translates_stop_reasons():
    from pib3.tools.stop_button import _grund
    assert _grund("Space key", "de") == "Leertaste"
    assert _grund("stop from laptop-3: Space key", "de") == "Stopp von laptop-3: Leertaste"
    assert _grund("Space key", "en") == "Space key"
