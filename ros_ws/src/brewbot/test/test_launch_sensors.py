"""sensors:=fake|real|none picks the right sensor include and the right trigger.

The three-way is easy to get wrong silently — a bad condition just launches
nothing and looks like a quiet startup.
"""

import importlib.util
import os

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument

LAUNCH = os.path.join(
    os.path.dirname(__file__), "..", "launch", "interaction.launch.py")


def _live(sensors, exclude_trigger="false"):
    spec = importlib.util.spec_from_file_location("interaction_launch", LAUNCH)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    ld = mod.generate_launch_description()

    ctx = LaunchContext()
    for entity in ld.entities:
        if isinstance(entity, DeclareLaunchArgument):
            entity.visit(ctx)
    ctx.launch_configurations["sensors"] = sensors
    ctx.launch_configurations["exclude_trigger"] = exclude_trigger

    on = [e for e in ld.entities
          if e.condition is None or e.condition.evaluate(ctx)]
    execs = {e.node_executable for e in on if hasattr(e, "node_executable")}
    includes = sum(1 for e in on
                   if type(e).__name__ == "IncludeLaunchDescription")
    return execs, includes


def test_sensor_includes():
    assert _live("fake")[1] == 1
    assert _live("real")[1] == 1
    assert _live("none")[1] == 0     # no sensors at all — the by-hand case


def test_trigger_rides_with_real_sensors():
    assert "trigger" in _live("real")[0]
    assert "trigger" not in _live("fake")[0]
    assert "trigger" not in _live("none")[0]
    assert "trigger" not in _live("real", exclude_trigger="true")[0]


def test_actuators_always_up():
    for sensors in ("fake", "real", "none"):
        execs = _live(sensors)[0]
        assert {"light_actuator", "coffee_machine_actuator"} <= execs


if __name__ == "__main__":
    test_sensor_includes()
    test_trigger_rides_with_real_sensors()
    test_actuators_always_up()
    print("OK")
