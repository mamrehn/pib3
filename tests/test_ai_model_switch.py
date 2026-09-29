"""Model switching on the robot's camera, without a robot.

The model belongs to the camera, not to one script: a switch restarts the
OAK-D (about 4 s), frames of the old model may still be in flight, and another
client can switch the shared camera at any time. These tests pin down how
``robot.ai`` copes: it waits for the new model's first result and ignores
results of any other model.
"""

import logging
import threading
import time
import types

import pytest

from pib3.backends.camera import AIDetectionReceiver, AISubsystem
from pib3.types import resolve_model_name


def payload(model, label=0, type_="detection"):
    return {
        "model": model,
        "type": type_,
        "frame_id": 1,
        "timestamp_ns": 0,
        "latency_ms": 10.0,
        "result": {"detections": [{
            "label": label, "confidence": 0.9,
            "bbox": {"xmin": 0.1, "ymin": 0.1, "xmax": 0.2, "ymax": 0.2},
        }]},
    }


# ==================== receiver ====================


def test_stale_results_before_the_switch_are_dropped_silently(caplog):
    r = AIDetectionReceiver()
    r.expect_model("yolo26n")
    with caplog.at_level(logging.WARNING):
        r.on_detection(payload("hand", type_="hand"))
    assert r.result_count == 0
    assert not r.wait_for_expected_model(0)
    assert not caplog.records

    r.on_detection(payload("yolo26n"))
    assert r.result_count == 1
    assert r.wait_for_expected_model(0)


def test_a_foreign_switch_is_warned_once_and_ignored(caplog):
    r = AIDetectionReceiver()
    r.expect_model("yolo26n")
    r.on_detection(payload("yolo26n"))
    with caplog.at_level(logging.WARNING):
        r.on_detection(payload("person"))
        r.on_detection(payload("person"))
    assert r.result_count == 1
    warnings = [rec for rec in caplog.records if "another client" in rec.getMessage()]
    assert len(warnings) == 1
    assert "'person'" in warnings[0].getMessage()


def test_without_an_expected_model_everything_is_kept():
    r = AIDetectionReceiver()
    r.on_detection(payload("yolo26n"))
    r.on_detection(payload("person"))
    assert r.result_count == 2


def test_payloads_without_a_model_name_are_kept():
    r = AIDetectionReceiver()
    r.expect_model("yolo26n")
    data = payload("x")
    del data["model"]
    r.on_detection(data)
    assert r.result_count == 1


# ==================== robot.ai.set_model ====================


class FakeRobot:
    """What AISubsystem touches: the switch service and the detection topic.

    ``stream`` is a list of (delay_s, payload) sent from a thread once
    something subscribes, like rosbridge delivering the camera's messages.
    """

    is_connected = True

    def __init__(self, accepts=True, stream=()):
        self.accepts = accepts
        self.stream = list(stream)

    def resolve_ai_model_name(self, model):
        return resolve_model_name(model, stacklevel=3)

    def set_ai_model(self, name, timeout):
        return self.accepts

    def subscribe_ai_detections(self, callback):
        def feed():
            for delay, data in self.stream:
                time.sleep(delay)
                callback(data)
        threading.Thread(target=feed, daemon=True).start()
        return types.SimpleNamespace(unsubscribe=lambda: None)


def test_set_model_returns_once_the_new_model_delivers():
    robot = FakeRobot(stream=[(0.0, payload("hand", type_="hand")),
                              (0.2, payload("yolo26n", label=0))])
    ai = AISubsystem(robot)
    t0 = time.monotonic()
    assert ai.set_model("yolo26n", timeout=5) is True
    assert time.monotonic() - t0 >= 0.2
    assert ai.model == "yolo26n"
    assert [d.label for d in ai.get_detections(timeout=0)] == ["person"]


def test_set_model_fails_when_no_result_arrives(caplog):
    robot = FakeRobot(stream=[(0.0, payload("hand", type_="hand"))])
    ai = AISubsystem(robot)
    with caplog.at_level(logging.WARNING):
        assert ai.set_model("yolo26n", timeout=0.3) is False
    assert any("sent no result" in rec.getMessage() for rec in caplog.records)


def test_a_refused_switch_keeps_the_previous_model():
    robot = FakeRobot(stream=[(0.0, payload("yolo26n"))])
    ai = AISubsystem(robot)
    assert ai.set_model("yolo26n", timeout=5)
    robot.accepts = False
    assert ai.set_model("hand", timeout=1) is False
    assert ai.model == "yolo26n"


def test_old_names_switch_to_yolo26n():
    robot = FakeRobot(stream=[(0.0, payload("yolo26n"))])
    ai = AISubsystem(robot)
    with pytest.warns(DeprecationWarning):
        assert ai.set_model("yolov6n", timeout=5)
    assert ai.model == "yolo26n"
