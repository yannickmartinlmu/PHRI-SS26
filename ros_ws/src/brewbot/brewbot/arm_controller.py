#!/usr/bin/env python3
"""Robot arm controller — owns the arm, gripper, and Elmo rail as one resource.

BringDrink action server. Each brew step is a *method* (skill), not a node: one
arm / rail / gripper means strictly sequential use, so methods in order beat
cross-node arbitration. Skills are stubs here — fill each with a canned motion.

Every hardware path is funnelled through one primitive (_move_arm / _gripper /
_move_elmo) so the joint-constraint-vs-pose-goal question — teleop is only proven
in sim so far — is decided in exactly one place without touching the skills.

_move_arm has two interchangeable backends, both driven from the SAME joint-angle
table (MoveIt takes joint_constraints, not just pose goals), so switching can never
land the arm somewhere different — MoveIt only adds collision-aware planning on the
way to an identical target:
    -p use_moveit:=true   MoveGroup /move_action        (planned, collision-aware)
    -p use_moveit:=false  FollowJointTrajectory         (dumb, direct, always works)
"""

import os
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import Float32
from control_msgs.action import FollowJointTrajectory, GripperCommand
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from brewbot_interfaces.action import BringDrink

# Kitchen collision scene lives in scripts/kitchen_scene.py (single source of truth,
# user-edited). Import it by path; --symlink-install makes realpath resolve to the real
# source tree. A miss disables the scene but must NOT ground the arm.
_SCRIPTS = os.path.normpath(os.path.join(
    os.path.dirname(os.path.realpath(__file__)), *[".."] * 4, "scripts"))
try:
    if _SCRIPTS not in sys.path:
        sys.path.insert(0, _SCRIPTS)
    from kitchen_scene import build_scene
except ImportError:
    build_scene = None

# Elmo setpoints (Float32). carriage = base_link -X, lift = Z. See elmo-axis-mapping.
# Both axes speak the identical topic pair, so one primitive drives them both.
ELMO_AXES = ("carriage", "lift")
ELMO_SET = "/elmo/id1/{axis}/position/set"
ELMO_GET = "/elmo/id1/{axis}/position/get"

# Rail carriage targets. Positions assumed
RAIL_KITCHEN = -0.5    # drink-filling station
RAIL_HANDOVER = 1.0       # handover position

# Lift height targets
LIFT_HOME = 0.35      # same default as elmo sim
LIFT_PICK_GLASS = 0.588
LIFT_HANDOVER = 0.588


ELMO_TOLERANCE = 0.01   # units; "arrived" window — widen if the axis creeps forever
ELMO_TIMEOUT = 30.0     # sec; raise rather than block the whole BringDrink goal
ELMO_POLL = 0.1         # sec between feedback checks

FK_ACTION = "/joint_trajectory_controller/follow_joint_trajectory"
MOVEIT_ACTION = "/move_action"
MOVE_GROUP = "manipulator"

JOINTS = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]

# Per-joint |limit|, from kortex_description gen3/6dof/urdf/gen3_macro.xacro.
# Continuous joints (1/4/6 without use_external_cable) get 2*pi — a safe bound either way.
JOINT_LIMITS = [6.28, 2.24, 2.57, 6.28, 2.09, 6.28]

FK_MOVE_TIME = 8          # sec; matches the hand-tested commands in info/ros_commands.txt
JOINT_TOLERANCE = 0.01    # rad, MoveIt goal window

GRIPPER_ACTION = "/robotiq_gripper_controller/gripper_cmd"
GRIPPER_LIMIT = 0.8       # 2F-140 mechanical close limit, per info/phri-reference-guide.md
GRIPPER_OPEN = 0.0
GRIPPER_CLOSED = 0.6      # tune against the real glass — GRIPPER_LIMIT crushes it
GRIPPER_MAX_EFFORT = 10.0  # N; lower if the glass complains

# Named arm poses in JOINT SPACE — the single table both backends consume.
# None = not teached yet: jog the arm, then `ros2 topic echo /joint_states`.
POSES = {
    "home":        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "tuck":        [-1.57, 0.0, 1.57, 0.0, 0.0, 1.57],  # glass-transport pose, sim-tested
    "above_glass": [-3.14, 0.0, 1.57, 0.0, 0.0, 1.57],
    "at_glass":    [-3.14, 0.0, 1.57, 0.0, 0.0, 1.57],
    "fill_coffee": None,
    "fill_water":  None,
    "handover":    [0, -1.57, 0, 0.0, 0.0, 1.57]
}


def _check_poses():
    # Runs at import: a typo'd table refuses to start the node rather than
    # commanding joint_6 to -26 rad (the wrist-bounds trap, see kinova-sim).
    for name, angles in POSES.items():
        if angles is None:
            continue
        assert len(angles) == len(JOINTS), f"POSES[{name}]: need {len(JOINTS)} angles"
        for joint, angle, limit in zip(JOINTS, angles, JOINT_LIMITS):
            assert abs(angle) <= limit, f"POSES[{name}]: {joint}={angle} exceeds ±{limit}"
    # Same idea for the hand-tuned gripper knobs: a typo jams the fingers.
    for name, pos in [("GRIPPER_OPEN", GRIPPER_OPEN), ("GRIPPER_CLOSED", GRIPPER_CLOSED)]:
        assert 0.0 <= pos <= GRIPPER_LIMIT, f"{name}={pos} outside 0.0..{GRIPPER_LIMIT}"


_check_poses()


class ArmController(Node):

    def __init__(self):
        super().__init__("arm_controller")
        cb = ReentrantCallbackGroup()

        # Elmo: setpoint out, feedback in, per axis.
        self._elmo_pub = {}
        self._elmo_pos = {}
        for axis in ELMO_AXES:
            self._elmo_pub[axis] = self.create_publisher(
                Float32, ELMO_SET.format(axis=axis), 10)
            self._elmo_pos[axis] = None
            self.create_subscription(
                Float32, ELMO_GET.format(axis=axis),
                lambda msg, a=axis: self._on_elmo(a, msg), 10, callback_group=cb
            )

        # Both arm backends stay wired; the parameter only picks which one sends.
        self.use_moveit = self.declare_parameter("use_moveit", True).value
        self._fk_client = ActionClient(
            self, FollowJointTrajectory, FK_ACTION, callback_group=cb)
        self._moveit_client = ActionClient(
            self, MoveGroup, MOVEIT_ACTION, callback_group=cb)

        self._gripper_client = ActionClient(
            self, GripperCommand, GRIPPER_ACTION, callback_group=cb)

        self._busy = False
        self._server = ActionServer(
            self, BringDrink, "bring_drink", self._execute,
            goal_callback=self._on_goal, callback_group=cb
        )

        # Kitchen collision scene: re-publish after every Elmo move + seed once at startup.
        # base_link rides the rail and MoveIt won't re-transform a cached scene. Best-effort.
        if build_scene is not None:
            self._scene_client = self.create_client(
                ApplyPlanningScene, "/apply_planning_scene", callback_group=cb)
            self._scene_timer = self.create_timer(1.0, self._seed_scene, callback_group=cb)
        else:
            self.get_logger().warn(
                f"kitchen_scene not importable from {_SCRIPTS} — collision scene disabled")

        self.get_logger().info("Arm controller ready")

    def _on_goal(self, goal_request):
        # One arm, one goal at a time — reject rather than queue or run concurrently.
        # Simple flag, not a hard lock. Two goals landing in the same instant could both pass
        if self._busy:
            self.get_logger().warn("[bring_drink] busy — rejecting goal")
            return GoalResponse.REJECT
        self._busy = True
        return GoalResponse.ACCEPT

    def _on_elmo(self, axis, msg):
        self._elmo_pos[axis] = msg.data

    def _seed_scene(self):
        # Seed the home scene once, as soon as both Elmo axes have reported, then stop.
        if all(v is not None for v in self._elmo_pos.values()):
            self._scene_timer.cancel()
            self._publish_scene()

    def _publish_scene(self):
        # Re-cache the kitchen boxes for wherever the rail is NOW. Best-effort: a scene
        # failure logs and returns — it must never abort a drink. See project-kitchen-scene.
        if build_scene is None:
            return
        carriage, lift = self._elmo_pos["carriage"], self._elmo_pos["lift"]
        if carriage is None or lift is None:
            self.get_logger().warn(f"[scene] no Elmo feedback yet — skipped")
            return
        if not self._scene_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("[scene] /apply_planning_scene unavailable — skipped")
            return
        ps = PlanningScene(is_diff=True)
        ps.world.collision_objects = build_scene(carriage, lift)
        result = self._scene_client.call(ApplyPlanningScene.Request(scene=ps))
        ok = result is not None and result.success
        self.get_logger().info(f"[scene] {len(ps.world.collision_objects)} boxes @ "
                               f"carriage={carriage} lift={lift} -> {ok}")

    # ---- motion primitives: the ONE place each hardware path gets implemented ----

    def _move_arm(self, target_pose_name):
        angles = POSES[target_pose_name]
        if angles is None:
            raise RuntimeError(
                f"pose '{target_pose_name}' not teached — jog the arm, read /joint_states, "
                f"put the 6 angles in POSES")
        how = "moveit" if self.use_moveit else "fk"
        self.get_logger().info(f"[arm] -> {target_pose_name} via {how}")
        if self.use_moveit:
            self._move_arm_moveit(angles)
        else:
            self._move_arm_fk(angles)

    def _send(self, client, goal):
        # Blocking send. Safe only because a MultiThreadedExecutor keeps spinning in
        # another thread — spin_until_future_complete (as in scripts/arm_teleop.py)
        # would deadlock here.
        client.wait_for_server()
        response = client.send_goal(goal)
        if response is None:
            raise RuntimeError("arm goal REJECTED by the action server")
        return response.result

    def _move_arm_fk(self, angles):
        # No IK, no collision checking: the joints go exactly where told.
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = JointTrajectory(
            joint_names=JOINTS,
            points=[JointTrajectoryPoint(
                positions=[float(a) for a in angles],
                time_from_start=Duration(sec=FK_MOVE_TIME))])
        result = self._send(self._fk_client, goal)
        if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            raise RuntimeError(
                f"FK trajectory failed: {result.error_code} {result.error_string}")

    def _move_arm_moveit(self, angles):
        # Same targets as _move_arm_fk, but planned around collisions.
        goal = MoveGroup.Goal()
        request = goal.request
        request.group_name = MOVE_GROUP
        request.num_planning_attempts = 5
        request.allowed_planning_time = 5.0
        request.max_velocity_scaling_factor = 0.3
        request.max_acceleration_scaling_factor = 0.3
        request.goal_constraints.append(Constraints(joint_constraints=[
            JointConstraint(joint_name=name, position=float(angle),
                            tolerance_above=JOINT_TOLERANCE,
                            tolerance_below=JOINT_TOLERANCE, weight=1.0)
            for name, angle in zip(JOINTS, angles)]))
        goal.planning_options.plan_only = False
        result = self._send(self._moveit_client, goal)
        if result.error_code.val != 1:  # MoveItErrorCodes.SUCCESS
            raise RuntimeError(f"MoveIt planning failed: error_code={result.error_code.val}")

    def _gripper(self, target_pos):
        self.get_logger().info(f"[gripper] -> {target_pos}")
        return
        goal = GripperCommand.Goal()
        goal.command.position = float(target_pos)
        goal.command.max_effort = GRIPPER_MAX_EFFORT
        result = self._send(self._gripper_client, goal)
        # Holding a glass means stalled at max effort BEFORE reaching the setpoint —
        # that is a successful grasp, so reached_goal alone would raise on every pick.
        if not (result.reached_goal or result.stalled):
            raise RuntimeError(f"gripper stuck at {result.position}")

    def _move_elmo(self, axis, target_position):
        # float(): skills reach here from the CLI, where every arg is still a string.
        target = float(target_position)
        self.get_logger().info(f"[elmo] {axis} -> {target}")
        self._elmo_pub[axis].publish(Float32(data=target))

        # Poll the feedback topic — the Elmo reports position but has no done signal.
        # Blocking is fine: another thread spins, same as the action clients.
        deadline = time.monotonic() + ELMO_TIMEOUT
        while time.monotonic() < deadline:
            time.sleep(ELMO_POLL)
            position = self._elmo_pos[axis]
            # None = no feedback yet; arriving is only knowable once a reading lands.
            if position is not None and abs(position - target) <= ELMO_TOLERANCE:
                self._publish_scene()  # base_link moved along the rail -> re-cache boxes
                return
        raise RuntimeError(
            f"elmo {axis} did not reach {target} within {ELMO_TIMEOUT}s "
            f"(last feedback: {self._elmo_pos[axis]})")

    # ---- skills: individually callable (ros2 run brewbot arm_controller <skill>) ----

    def home(self):
        self._move_arm("home")

    def tuck(self):
        self._move_arm("tuck")

    def move_rail(self, target_carriage_position):
        self.tuck()  # INVARIANT: arm safe-by-construction before ANY rail move
        self._move_elmo("carriage", target_carriage_position)

    def move_lift(self, target_lift_position):
        self._move_elmo("lift", target_lift_position)

    def open_gripper(self):
        self._gripper(GRIPPER_OPEN)

    def close_gripper(self):
        self._gripper(GRIPPER_CLOSED)

    def pick_glass(self):
        self._move_arm("above_glass"); 
        self.open_gripper()
        self.move_lift(LIFT_PICK_GLASS)
        self.close_gripper()
        self.move_lift(LIFT_HOME)

    def fill(self, drink):
        self._move_arm(f"fill_{drink}")

    def handover(self):
        self._move_arm("handover"); 
        self.move_lift(LIFT_HANDOVER)
        self.open_gripper()
        self.move_lift(LIFT_HOME)

    # ---- orchestration: BringDrink = skills in sequence ----

    def _execute(self, goal_handle):
        drink = goal_handle.request.drink
        self.get_logger().info(f"[bring_drink] {drink}")
        try:
            self.tuck()
            # self.move_rail(RAIL_STATION)
            self.pick_glass()
            self.fill(drink)
            # self.move_rail(RAIL_USER)
            self.handover()
            goal_handle.succeed()
            return BringDrink.Result(success=True)
        except Exception as e:
            # A raised motion must abort the goal, not vanish into the callback.
            self.get_logger().error(f"[bring_drink] failed: {e}")
            goal_handle.abort()
            return BringDrink.Result(success=False)
        finally:
            self._busy = False  # never leave the controller wedged as busy


def main():
    rclpy.init()
    node = ArmController()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    # Address one skill directly: `ros2 run brewbot arm_controller pick_glass` (or `fill coffee`).
    # Cut at --ros-args first, or `-p use_moveit:=false` gets read as a skill name.
    argv = sys.argv[1:sys.argv.index("--ros-args")] if "--ros-args" in sys.argv else sys.argv[1:]
    args = [a for a in argv if not a.startswith("-")]
    if args:
        # Spin in the background so skills get action results and topic callbacks
        # exactly as they do under the action server — one waiting style everywhere.
        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()
        try:
            getattr(node, args[0])(*args[1:])
        finally:
            # Stop the executor and join BEFORE tearing the context down, or rclpy
            # aborts ("terminate called without an active exception").
            executor.shutdown()
            spin_thread.join()
            node.destroy_node()
            rclpy.shutdown()
        return

    try:
        executor.spin()
    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
