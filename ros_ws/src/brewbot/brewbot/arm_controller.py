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
from moveit_msgs.srv import ApplyPlanningScene, GetStateValidity
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from tf2_ros import StaticTransformBroadcaster
from brewbot_interfaces.action import BringDrink

# Kitchen collision scene lives in scripts/kitchen_scene.py (single source of truth,
# user-edited). Import it by path; --symlink-install makes realpath resolve to the real
# source tree. A miss disables the scene but must NOT ground the arm.
_SCRIPTS = os.path.normpath(os.path.join(
    os.path.dirname(os.path.realpath(__file__)), *[".."] * 4, "scripts"))
try:
    if _SCRIPTS not in sys.path:
        sys.path.insert(0, _SCRIPTS)
    from kitchen_scene import build_scene, make_anchor_tf
except ImportError:
    build_scene = make_anchor_tf = None

# Elmo setpoints (Float32). carriage = base_link -X, lift = Z. See elmo-axis-mapping.
# Both axes speak the identical topic pair, so one primitive drives them both.
ELMO_AXES = ("carriage", "lift")
ELMO_SET = "/elmo/id1/{axis}/position/set"
ELMO_GET = "/elmo/id1/{axis}/position/get"

# Rail carriage targets. Positions assumed
RAIL_KITCHEN = -0.6       # drink-filling station
RAIL_HANDOVER = 1.1       # handover position

# Lift height targets
LIFT_HOME = 0.35      # travel waypoint only — the frame anchor lives in kitchen_scene
LIFT_PICK_GLASS = 0.57
LIFT_HANDOVER = 0.43
LIFT_MIN = 0.235
LIFT_COLLISION_STEP = 0.05  # m; virtual sweep resolution for the lift safety check.
# ponytail: tunneling knob — arm width catches thinner boxes, shrink if one slips through.


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
GRIPPER_CLOSED = 0.5      # tune against the real glass — GRIPPER_LIMIT crushes it
GRIPPER_MAX_EFFORT = 10.0  # N; lower if the glass complains

# Named arm poses in JOINT SPACE — the single table both backends consume.
# None = not teached yet: jog the arm, then `ros2 topic echo /joint_states`.
POSES = {
    "home":         [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "tuck":         [-1.57, 0.0, 1.57, 0.0, 0.0, 1.57],  # glass-transport pose, sim-tested
    "above_glass":  [-3.14, -0.9, 0.0, 0.0, 0.8, 1.57],
    "at_glass":     [-3.14, -0.9, 0.0, 0.0, 0.8, 1.57],
    "above_glass_old":  [-3.14, -0.78, 0.0, 0.0, 0.78, 1.57],
    # Placeholder values. Confirm in the real world by aproaching slowly
    "fill_coffee":  [2.3, 0.3, 1.05, 0, 0.8, 1.57],
    "fill_water":   [-3.53, 0.3, 0.75, 0.0, 1.15, 1.57],
    "handover":     [-0.2, -0.9, -0.2, 0.0, 0.9, 1.57], 
    "handover_old": [0, -0.78, 0.0, 0.0, 0.78, 1.57]

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

        # Kitchen collision scene: boxes are constants in the fixed 'lab' frame; we
        # broadcast lab->base_link from Elmo feedback and re-send the boxes after every
        # Elmo move (move_group bakes them at receive time, it won't re-transform).
        if build_scene is not None:
            self._tf_broadcaster = StaticTransformBroadcaster(self)
            self._scene_client = self.create_client(
                ApplyPlanningScene, "/apply_planning_scene", callback_group=cb)
            self._validity_client = self.create_client(
                GetStateValidity, "/check_state_validity", callback_group=cb)
            self._scene_timer = self.create_timer(1.0, self._seed_scene, callback_group=cb)
        else:
            self.get_logger().warn(
                f"kitchen_scene not importable from {_SCRIPTS} — collision scene disabled")

        self.get_logger().info("Arm controller ready")

    def wait_ready(self, timeout=20.0):
        # DDS discovery over the lab network can take seconds. Publishing an Elmo
        # setpoint before its subscriber is matched silently drops it (volatile
        # QoS) — the one-shot CLI mode hit this constantly. Actions already gate
        # on wait_for_server(); this is the same gate for the raw Elmo pub/sub.
        deadline = time.monotonic() + timeout
        def pending():
            return [a for a in ELMO_AXES
                    if self._elmo_pub[a].get_subscription_count() == 0
                    or self._elmo_pos[a] is None]
        while pending():
            if time.monotonic() > deadline:
                raise RuntimeError(
                    f"elmo axes {pending()} not discovered within {timeout}s — "
                    f"is the Elmo node up?")
            time.sleep(0.1)
        self.get_logger().info("[elmo] discovery complete — both axes matched")

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

    def _broadcast_anchor(self, carriage, lift):
        # lab->world from Elmo readings (world = URDF root, identity to base_link).
        # Static TF is latched; re-sending replaces the old value in every listener's
        # buffer. We are the ONLY writer of this pair.
        self._tf_broadcaster.sendTransform(
            make_anchor_tf(carriage, lift, self.get_clock().now().to_msg()))

    def _publish_scene(self, lift=None):
        # Broadcast lab->base_link, then re-send the (constant) kitchen boxes so
        # move_group re-bakes them against the fresh TF. `lift` override = the virtual
        # sweep in _lift_path_clear lying about height; None = where Elmo really is.
        # Best-effort: a scene failure logs and returns — it must never abort a drink.
        if build_scene is None:
            return
        carriage = self._elmo_pos["carriage"]
        lift = lift if lift is not None else self._elmo_pos["lift"]
        if carriage is None or lift is None:
            self.get_logger().warn(f"[scene] no Elmo feedback yet — skipped")
            return
        if not self._scene_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("[scene] /apply_planning_scene unavailable — skipped")
            return
        self._broadcast_anchor(carriage, lift)
        time.sleep(0.2)  # ponytail: TF must land before boxes bake; raise if ever stale
        ps = PlanningScene(is_diff=True)
        ps.world.collision_objects = build_scene()
        result = self._scene_client.call(ApplyPlanningScene.Request(scene=ps))
        ok = result is not None and result.success
        self.get_logger().info(f"[scene] {len(ps.world.collision_objects)} boxes @ "
                               f"carriage={carriage} lift={lift} -> {ok}")

    # ---- motion primitives: the ONE place each hardware path gets implemented ----

    def move_arm(self, target_pose_name):
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

    def move_arm_through_poses(self, target_pose_names):
        for pose in target_pose_names:
            self._move_arm_fk(POSES[pose])

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
        request.num_planning_attempts = 16
        request.allowed_planning_time = 10.0
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
        # Sim does not have a working gripper. Do a check, then skip. 
        if not self._gripper_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn(
                f"[gripper] no action server — skipping -> {target_pos} (sim?)")
            return
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
        self.move_arm("home")

    def tuck(self):
        self.move_arm("tuck")

    def move_rail(self, target_carriage_position):
        self.tuck()  # INVARIANT: arm safe-by-construction before ANY rail move
        self._move_elmo("carriage", target_carriage_position)

    def move_lift(self, target_lift_position):
        if not self._lift_path_clear(float(target_lift_position)):
            raise RuntimeError(
                f"lift path to {target_lift_position} blocked by collision — aborting")
        self._move_elmo("lift", target_lift_position)

    def _lift_path_clear(self, target):
        # Elmo isn't a MoveIt joint, so MoveIt can't plan the lift. Instead: freeze the arm
        # and virtually ride base_link through each height by LYING on the lab anchor TF —
        # _publish_scene(lift=h) re-bakes the constant boxes against the fake anchor — then
        # ask /check_state_validity if the current arm state collides. is_diff=True + empty
        # state means "the arm where it is right now". /apply_planning_scene is a service
        # (synchronous apply), so the scene is live before each check. Dynamic obstacles
        # enter the same scene and get checked for free. RViz flickers during the sweep;
        # the final _publish_scene() restores TF + scene to reality.
        # FAIL-OPEN: if the check can't run, warn loudly and allow.
        current, carriage = self._elmo_pos["lift"], self._elmo_pos["carriage"]
        if build_scene is None or current is None or carriage is None:
            self.get_logger().warn("[lift-check] no scene/feedback — SKIPPING collision check")
            return True
        if not self._validity_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("[lift-check] /check_state_validity unavailable — SKIPPING")
            return True

        # Sampled heights: every step from current toward target, plus target itself (a
        # partial final step must still be checked — a 0.08 m move checks 0.05 AND 0.08).
        step = LIFT_COLLISION_STEP if target >= current else -LIFT_COLLISION_STEP
        heights, h = [], current + step
        while (step > 0 and h < target) or (step < 0 and h > target):
            heights.append(h)
            h += step
        heights.append(target)

        clear = True
        for lift in heights:
            self._publish_scene(lift=lift)
            req = GetStateValidity.Request()
            req.robot_state.is_diff = True  # empty state + is_diff = current monitored arm
            req.group_name = MOVE_GROUP
            result = self._validity_client.call(req)
            if result is not None and not result.valid:
                self.get_logger().warn(f"[lift-check] collision at lift={lift:.3f} — blocked")
                clear = False
                break

        self._publish_scene()  # restore TF + scene to where the arm actually is
        return clear

    def open_gripper(self):
        self._gripper(GRIPPER_OPEN)

    def close_gripper(self):
        self._gripper(GRIPPER_CLOSED)

    def pick_glass(self):
        self.move_lift(LIFT_HOME)
        self.move_rail(RAIL_HANDOVER)
        self.move_arm("above_glass"); 
        self.open_gripper()
        self.move_lift(LIFT_PICK_GLASS)
        self.close_gripper()
        self.move_lift(LIFT_HOME)

    def fill(self, drink):
        self.move_rail(RAIL_KITCHEN)
        self.move_arm(f"fill_{drink}")

    def handover(self):
        self.move_rail(RAIL_HANDOVER)
        self.move_lift(LIFT_HOME)
        self.move_arm("handover"); 
        self.move_lift(LIFT_HANDOVER)
        self.open_gripper()
        self.move_lift(LIFT_MIN)
        self.tuck()
        self.move_lift(LIFT_HOME)
        
    def bring_bottle_simple(self):
        self.pick_glass()
        self.fill("water")
        self.handover()

    def retrieve_bottle_simple(self): 
        self.move_lift(LIFT_MIN)
        self.move_arm("handover"); 
        self.move_lift(LIFT_HANDOVER)
        self.close_gripper()
        self.move_lift(LIFT_HOME)
        self.move_arm("above_glass")
        self.move_lift(LIFT_PICK_GLASS)
        self.open_gripper()
        self.move_lift(LIFT_HOME)
        self.tuck()


    # ---- orchestration: BringDrink = skills in sequence ----

    def _execute(self, goal_handle):
        drink = goal_handle.request.drink
        self.get_logger().info(f"[bring_drink] {drink}")
        try:
            if(drink == "water"):
                self.bring_bottle_simple()
            else: 
                self.retrieve_bottle_simple()
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
    # Drop flags (-p, --ros-args) but keep negative numbers like -0.3.
    def _is_flag(a):
        try:
            float(a)
            return False
        except ValueError:
            return a.startswith("-")
    args = [a for a in argv if not _is_flag(a)]
    if args:
        # Spin in the background so skills get action results and topic callbacks
        # exactly as they do under the action server — one waiting style everywhere.
        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()
        try:
            node.wait_ready()  # one-shot mode: block until DDS discovery is done
            node.get_logger().info(f"[cli] {args[0]}({', '.join(args[1:])})")
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
