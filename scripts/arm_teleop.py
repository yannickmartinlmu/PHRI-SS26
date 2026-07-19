#!/usr/bin/env python3
"""Keyboard teleop for the ceiling-mounted Kinova Gen3.

Each keypress reads the arm's LIVE pose from TF, nudges it by one step
(translation or a coarse rotation), and sends a blocking MoveGroup goal
(absolute pose). One key = one completed move.

Reading the live pose every keypress means a failed/unreachable move can't
poison the next command: the arm didn't move, so the next read is still the
true current pose. It also means no hardcoded start pose -- the arm can begin
anywhere (straight down, tucked, tilted).

Run in a sourced ROS2 env (WSL+venv):  python3 scripts/arm_teleop.py
Self-test the quaternion math (no ROS): python3 scripts/arm_teleop.py test

The arm hangs from the ceiling, so base_link Z points down: UP = decreasing z.
"""

import sys
from math import sin, cos, radians

STEP = 0.1
ROT_STEP = radians(30)  # coarse; tune to taste

# translation: key -> (axis, delta). Arrows arrive as 3-byte escape sequences.
MOVES = {
    "\x1b[A": ("y", +STEP),  # up arrow
    "\x1b[B": ("y", -STEP),  # down arrow
    "\x1b[C": ("x", -STEP),  # right arrow
    "\x1b[D": ("x", +STEP),  # left arrow
    "r": ("z", -STEP),       # up   (ceiling flip)
    "f": ("z", +STEP),       # down
}

# rotation: key -> (base_link-fixed axis, angle). No roll -- useless to hand-control.
ROT = {
    "w": ((0.0, 1.0, 0.0), +ROT_STEP),  # pitch toward horizontal
    "s": ((0.0, 1.0, 0.0), -ROT_STEP),  # pitch back toward down
    "a": ((0.0, 0.0, 1.0), +ROT_STEP),  # yaw (approach side)
    "d": ((0.0, 0.0, 1.0), -ROT_STEP),  # yaw (other side)
}

HELP = ("arm_teleop  |  arrows: x/y   r/f: up/down   "
        "w/s: pitch   a/d: yaw   h: horizontal   q: quit")


def qmul(a, b):
    """Hamilton product; quats as (x, y, z, w)."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw*bx + ax*bw + ay*bz - az*by,
            aw*by - ax*bz + ay*bw + az*bx,
            aw*bz + ax*by - ay*bx + az*bw,
            aw*bw - ax*bx - ay*by - az*bz)


def _from_axis_angle(axis, ang):
    s = sin(ang / 2)
    return (axis[0]*s, axis[1]*s, axis[2]*s, cos(ang / 2))


def rotate(quat, key):
    """Rotate quat by the key's delta about a base_link-fixed axis."""
    axis, ang = ROT[key]
    return qmul(_from_axis_angle(axis, ang), quat)  # world-fixed: delta on left


# Glass-holding pose: gripper horizontal (approach along base_link +X) plus a
# 90 deg wrist roll so the fingers close HORIZONTALLY around an upright glass
# (side grasp, glass stays level) -- not top-to-bottom. Works out to
# (0.5, -0.5, 0.5, -0.5). Tight on all axes; steer facing with a/d/w/s after.
# Flip the roll sign or tune on the real arm if the tool frame convention differs.
HORIZ = qmul(qmul(_from_axis_angle((0.0, 1.0, 0.0), radians(90)),
                  (0.0, 0.0, 1.0, 0.0)),
             _from_axis_angle((0.0, 0.0, 1.0), radians(90)))
HORIZ_TOL = (0.1, 0.05, 0.1)

# PILZ PTP moves joint-to-joint from the current config (nearest IK, no random
# OMPL sampling) so a small pose nudge = a small joint change -- no wrist flips.
# Auto-falls back to OMPL (empty pipeline) if PILZ isn't installed/configured.
USE_PILZ = True


def read_key():
    import termios, tty
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
        if ch == "\x1b":  # escape -> arrow key, grab the next two bytes
            ch += sys.stdin.read(2)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return ch


def lookup_pose(rclpy, node, buffer):
    """Live base_link -> end_effector_link pose: ({x,y,z}, (x,y,z,w))."""
    import tf2_ros
    from rclpy.time import Time
    for _ in range(50):
        rclpy.spin_once(node, timeout_sec=0.1)
        try:
            t = buffer.lookup_transform("base_link", "end_effector_link", Time())
            tr, ro = t.transform.translation, t.transform.rotation
            return ({"x": tr.x, "y": tr.y, "z": tr.z},
                    (ro.x, ro.y, ro.z, ro.w))
        except tf2_ros.TransformException:
            continue
    raise RuntimeError("no TF base_link->end_effector_link")


def make_goal(pos, quat, tol=(0.1, 0.1, 0.1), pilz=True):
    from moveit_msgs.action import MoveGroup
    from moveit_msgs.msg import (Constraints, PositionConstraint,
                                 OrientationConstraint, BoundingVolume)
    from shape_msgs.msg import SolidPrimitive
    from geometry_msgs.msg import Pose

    goal = MoveGroup.Goal()
    req = goal.request
    req.group_name = "manipulator"
    if pilz:  # else empty pipeline_id -> configured default (OMPL)
        req.pipeline_id = "pilz_industrial_motion_planner"
        req.planner_id = "PTP"
    req.num_planning_attempts = 5
    req.allowed_planning_time = 5.0
    req.max_velocity_scaling_factor = 0.3
    req.max_acceleration_scaling_factor = 0.3

    sphere = SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.01])
    region_pose = Pose()
    region_pose.position.x = pos["x"]
    region_pose.position.y = pos["y"]
    region_pose.position.z = pos["z"]
    region_pose.orientation.w = 1.0

    pc = PositionConstraint()
    pc.header.frame_id = "base_link"
    pc.link_name = "end_effector_link"
    pc.constraint_region = BoundingVolume(primitives=[sphere],
                                          primitive_poses=[region_pose])
    pc.weight = 1.0

    oc = OrientationConstraint()
    oc.header.frame_id = "base_link"
    oc.link_name = "end_effector_link"
    oc.orientation.x, oc.orientation.y, oc.orientation.z, oc.orientation.w = quat
    oc.absolute_x_axis_tolerance = tol[0]
    oc.absolute_y_axis_tolerance = tol[1]
    oc.absolute_z_axis_tolerance = tol[2]
    oc.weight = 1.0

    req.goal_constraints.append(
        Constraints(position_constraints=[pc], orientation_constraints=[oc]))
    goal.planning_options.plan_only = False
    return goal


def send(rclpy, node, client, goal):
    """Returns (ok, error_code); ok is True only on SUCCESS (code 1)."""
    fut = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, fut)
    handle = fut.result()
    if not handle.accepted:
        return False, None
    res_fut = handle.get_result_async()
    rclpy.spin_until_future_complete(node, res_fut)
    code = res_fut.result().result.error_code.val
    return code == 1, code


def main():
    global USE_PILZ
    import rclpy
    from rclpy.action import ActionClient
    from moveit_msgs.action import MoveGroup
    import tf2_ros

    rclpy.init()
    node = rclpy.create_node("arm_teleop")
    buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buffer, node)  # noqa: F841 (keep ref alive)
    client = ActionClient(node, MoveGroup, "/move_action")
    print("waiting for /move_action ...")
    client.wait_for_server()

    print(HELP)
    try:
        while True:
            key = read_key()
            if key in ("q", "\x03"):  # q or Ctrl-C
                break
            if key not in MOVES and key not in ROT and key != "h":
                continue
            pos, quat = lookup_pose(rclpy, node, buffer)  # fresh ground truth
            tol = (0.1, 0.1, 0.1)
            if key == "h":  # snap horizontal, keep coords, free the rotation
                quat, tol = HORIZ, HORIZ_TOL
            elif key in MOVES:
                axis, delta = MOVES[key]
                pos[axis] = round(pos[axis] + delta, 3)
            else:
                quat = rotate(quat, key)
            print(f"  target pos={pos} quat={tuple(round(c, 3) for c in quat)}")
            ok, code = send(rclpy, node, client, make_goal(pos, quat, tol, USE_PILZ))
            if not ok and USE_PILZ:  # PILZ missing/misconfigured -> OMPL, once
                USE_PILZ = False
                print(f"  PILZ failed (code={code}) -> falling back to OMPL")
                ok, code = send(rclpy, node, client, make_goal(pos, quat, tol, False))
            print("  done" if ok else f"  FAILED error_code={code}")
    finally:
        rclpy.shutdown()


def _selftest():
    from math import isclose
    assert qmul((0, 0, 0, 1), (0, 0, 0, 1)) == (0, 0, 0, 1)  # identity
    q = rotate((0, 0, 0, 1), "a")  # +30 deg yaw about Z
    assert isclose(q[2], sin(radians(15))) and isclose(q[3], cos(radians(15)))
    q = rotate((0, 0, 0, 1), "w")  # +30 deg pitch about Y
    assert isclose(q[1], sin(radians(15))) and isclose(q[3], cos(radians(15)))
    q90 = (0, 0, sin(radians(45)), cos(radians(45)))  # 90 deg about Z
    q = qmul(q90, q90)  # composes to 180 deg -> (0,0,1,0)
    assert isclose(q[2], 1.0, abs_tol=1e-9) and isclose(q[3], 0.0, abs_tol=1e-9)
    assert all(isclose(a, b, abs_tol=1e-9)  # glass-holding pose
               for a, b in zip(HORIZ, (0.5, -0.5, 0.5, -0.5)))
    print("selftest ok")


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "test":
        _selftest()
    else:
        main()
