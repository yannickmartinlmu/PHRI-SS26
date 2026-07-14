#!/usr/bin/env python3
"""Keyboard teleop for the ceiling-mounted Kinova Gen3.

Each keypress nudges a locally-tracked target pose by one step and sends a
blocking MoveGroup goal (absolute position, fixed orientation). One key = one
completed move.

Run in a sourced ROS2 env (WSL+venv):  python3 scripts/arm_teleop.py

The arm hangs from the ceiling, so base_link Z is flipped: UP = decreasing z.
Target is dead-reckoned locally, seeded from tf2_echo base_link end_effector_link.
"""

import sys
import rclpy
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup

STEP = 0.1

# key -> (axis, delta). Arrow keys arrive as 3-byte escape sequences.
MOVES = {
    "\x1b[A": ("y", +STEP),  # up arrow    -> +y
    "\x1b[B": ("y", -STEP),  # down arrow  -> -y
    "\x1b[C": ("x", -STEP),  # right arrow -> +x
    "\x1b[D": ("x", +STEP),  # left arrow  -> -x
    "u": ("z", -STEP),       # u = up   -> -z (ceiling flip)
    "j": ("z", +STEP),       # j = down -> +z
}

# Fixed end-effector orientation (xyzw) from tf: 180 deg about Z, pointing down.
ORIENT = (0.0, 0.0, 1.0, 0.0)

HELP = """arm_teleop  |  arrows: x/y   u: up   j: down   q: quit"""


def apply(pose, key):
    axis, delta = MOVES[key]
    pose[axis] = round(pose[axis] + delta, 3)
    return pose


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


def make_goal(pose):
    from moveit_msgs.action import MoveGroup
    from moveit_msgs.msg import (Constraints, PositionConstraint,
                                 OrientationConstraint, BoundingVolume)
    from shape_msgs.msg import SolidPrimitive
    from geometry_msgs.msg import Pose

    goal = MoveGroup.Goal()
    req = goal.request
    req.group_name = "manipulator"
    req.num_planning_attempts = 5
    req.allowed_planning_time = 5.0
    req.max_velocity_scaling_factor = 0.3
    req.max_acceleration_scaling_factor = 0.3

    sphere = SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.01])
    region_pose = Pose()
    region_pose.position.x = pose["x"]
    region_pose.position.y = pose["y"]
    region_pose.position.z = pose["z"]
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
    oc.orientation.x, oc.orientation.y, oc.orientation.z, oc.orientation.w = ORIENT
    oc.absolute_x_axis_tolerance = 0.1
    oc.absolute_y_axis_tolerance = 0.1
    oc.absolute_z_axis_tolerance = 0.1
    oc.weight = 1.0

    req.goal_constraints.append(
        Constraints(position_constraints=[pc], orientation_constraints=[oc]))
    goal.planning_options.plan_only = False
    return goal


def send(rclpy, node, client, goal):
    fut = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, fut)
    handle = fut.result()
    if not handle.accepted:
        print("  goal REJECTED")
        return
    res_fut = handle.get_result_async()
    rclpy.spin_until_future_complete(node, res_fut)
    code = res_fut.result().result.error_code.val
    print("  done" if code == 1 else f"  FAILED error_code={code}")


def main():
    

    pose = {"x": 0.0, "y": 0.0, "z": 1.177}  # seed from tf2_echo

    rclpy.init()
    node = rclpy.create_node("arm_teleop")
    client = ActionClient(node, MoveGroup, "/move_action")
    print("waiting for /move_action ...")
    client.wait_for_server()

    print(HELP)
    print(f"  target = {pose}")
    try:
        while True:
            key = read_key()
            if key in ("q", "\x03"):  # q or Ctrl-C
                break
            if key not in MOVES:
                continue
            apply(pose, key)
            print(f"  target = {pose}")
            send(rclpy, node, client, make_goal(pose))
    finally:
        rclpy.shutdown()



if __name__ == "__main__":
    main()
