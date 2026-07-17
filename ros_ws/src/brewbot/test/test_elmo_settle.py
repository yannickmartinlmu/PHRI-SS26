"""Check the _move_elmo settle loop without a robot, an Elmo, or an rclpy init.

The method is called unbound against a fake self — the loop only touches
_elmo_pub / _elmo_pos / get_logger, so no node is needed.
"""

import pytest

from brewbot import arm_controller as m


class FakeElmo:
    """Reports `arrives_at` on the next feedback poll, or never if None."""

    def __init__(self, arrives_at=None):
        self.published = []
        self._elmo_pos = {axis: None for axis in m.ELMO_AXES}
        self._elmo_pub = {axis: self for axis in m.ELMO_AXES}
        self._arrives_at = arrives_at

    def publish(self, msg):  # stands in for the rclpy publisher
        self.published.append(msg.data)
        if self._arrives_at is not None:
            self._elmo_pos["carriage"] = self._arrives_at
            self._elmo_pos["lift"] = self._arrives_at

    def get_logger(self):
        return self

    def info(self, _msg):
        pass


def test_returns_once_feedback_reaches_target():
    elmo = FakeElmo(arrives_at=1.0)
    m.ArmController._move_elmo(elmo, "carriage", 1.0)
    assert elmo.published == [1.0]


def test_accepts_string_target_from_the_cli():
    # `ros2 run brewbot arm_controller move_rail 1.0` hands the target over as str.
    elmo = FakeElmo(arrives_at=1.0)
    m.ArmController._move_elmo(elmo, "carriage", "1.0")
    assert elmo.published == [1.0]


def test_raises_when_feedback_never_arrives(monkeypatch):
    monkeypatch.setattr(m, "ELMO_TIMEOUT", 0.3)
    with pytest.raises(RuntimeError, match="did not reach"):
        m.ArmController._move_elmo(FakeElmo(), "lift", 1.0)


def test_raises_when_axis_stops_short(monkeypatch):
    # Silent creep-to-a-halt must fail loudly, not report a move that never landed.
    monkeypatch.setattr(m, "ELMO_TIMEOUT", 0.3)
    elmo = FakeElmo(arrives_at=0.5)
    with pytest.raises(RuntimeError, match="did not reach"):
        m.ArmController._move_elmo(elmo, "carriage", 1.0)
