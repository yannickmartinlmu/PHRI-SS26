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

import math
import os
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from tf2_ros import Buffer, TransformListener

from std_msgs.msg import Bool, Empty, Float32, String
from std_srvs.srv import SetBool
from control_msgs.action import FollowJointTrajectory, GripperCommand
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (Constraints, JointConstraint, PlanningScene,
                             PositionConstraint, OrientationConstraint, BoundingVolume)
from moveit_msgs.srv import ApplyPlanningScene, GetStateValidity
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from brewbot_interfaces.action import BringDrink, DispenseDrink
from brewbot_interfaces.srv import AskForWater
from brewbot.drinks import MENU
from brewbot.gestures import (
    EVENT_TOPIC as GESTURE_EVENT_TOPIC,
    UP as GESTURE_UP,
    PALM as GESTURE_PALM,
)

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


# Every hand the robot can see — both recognizers feed this one topic. The names
# and the reason it is the EVENT topic live in brewbot/gestures.py, because
# interaction_manager watches the same topic for the open-palm barge-in.
# thumbs_down needs no special case here — see query_all_drinks.
GESTURE_TIMEOUT = 10.0    # sec the arm holds the questioning pose waiting for an answer

# Sec the arm holds the glass under the tap waiting to be told "done". Bounded so a
# dead transcriber parks the arm at the sink for a minute, not forever.
WATER_CONFIRM_TIMEOUT = 60.0

# Waiting under a tap looks like a hang, so the wait escalates: first just hold,
# then a wiggle, then the lab light. Both nudges are pure feedback and both fail
# open — no light node, no planner, the water still happens.
WIGGLE_AFTER = 20.0       # sec of silence before the arm nudges the user
BLINK_AFTER = 40.0        # sec before the light joins in
ANSWER_GRACE = 10.0       # sec past the timeout before we give up on the reply
                          # itself — interaction_manager stops listening at
                          # WATER_CONFIRM_TIMEOUT and answers then, so cancelling
                          # at exactly 60 would race its own response.
WAIT_POLL = 0.1           # sec between "did they answer yet" checks

# Ends on fill_water, so one wiggle puts the glass back level by itself.
WIGGLE_POSES = ("fill_water_wiggled_1", "fill_water_wiggled_2", "fill_water")
WIGGLE_MOVE_TIME = 1.5    # sec per leg. One wiggle is ~4.5s of servo noise inside
                          # a 60s listening window — the ear stays open throughout.
BLINK_SERVICE = "/lab_light/blink"

# Rail carriage targets. Positions assumed
RAIL_KITCHEN = -0.6       # drink-filling station
RAIL_HANDOVER = 1.1       # handover position

# Lift height targets
LIFT_HOME = 0.35      # same default as elmo sim
LIFT_PICK_GLASS = 0.58
LIFT_HANDOVER = 0.44
LIFT_MIN = 0.235
LIFT_COLLISION_STEP = 0.05  # m; virtual sweep resolution for the lift safety check.
# ponytail: tunneling knob — arm width catches thinner boxes, shrink if one slips through.


# Base sags under load; the kitchen seen from the tilted base_link is rotated by -TILT,
# which build_scene applies so MoveIt's level-FK collision check matches reality. One
# measured number for now (jog to a worst-case left reach, tilt ~= tip_drop / reach);
# swap in the live per-config C*tau model later. See project-tilt-compensation.
TILT_DEG = 0.0     # 0 until measured; single constant, per-config model later
TILT_AXIS = "y"    # sag plane: reach along X (rail) dips in Z

# Sag model (offline fit, NOT wired yet — feeds build_scene later once TILT_LINE is real).
# Sag is a pitch about Y; torque about Y ~ weight * EE x-offset, so tilt is linear in x alone:
# tilt(x) = a*x + c. The measured drop = tilt*x = a*x^2 + c*x (quadratic — why drop looked nonlinear).
FLOOR_BELOW_BASE_HOME = 2.006  # m; floor depth under base_link at lift=LIFT_HOME (reference_lift_calibration)
TILT_LINE = (0.0, 0.0)         # (a, c): tilt_rad = a*x + c. 0 until calibrate_tilt fills it.

ELMO_TOLERANCE = 0.01   # units; "arrived" window — widen if the axis creeps forever
ELMO_TIMEOUT = 30.0     # sec; raise rather than block the whole BringDrink goal
ELMO_POLL = 0.1         # sec between feedback checks

FK_ACTION = "/joint_trajectory_controller/follow_joint_trajectory"
MOVEIT_ACTION = "/move_action"
MOVE_GROUP = "manipulator"
EE_LINK = "end_effector_link"

# Cartesian pose goals (move_to_point). Frame E = fixed, origin at carriage=0 / lift=0
# (the unreachable rail zero), axes parallel to base_link. Elmo feedback is metres 1:1
# (see elmo-axis-mapping), so E->base_link is pure translation and the carriage nulls X.
RAIL_MIN, RAIL_MAX = -0.6, 1.1      # ponytail: physical rail travel; widen if it reaches further
GRIPPER_HORIZ_QUAT = (0.5, -0.5, 0.5, -0.5)   # horizontal side grasp; proven as arm_teleop HORIZ
# ponytail: verify — jog arm to a level side grasp, `tf2_echo base_link end_effector_link`
POSE_POS_TOL = 0.01      # m; IK position window (sphere radius)
POSE_LEVEL_TOL = 0.01     # rad; how far the wrist may tilt off level (pitch/roll)
POSE_YAW_TOL = 3.15      # rad; ~free azimuth about vertical — round upright bottle, approach from any side

JOINTS = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]

# Per-joint |limit|, from kortex_description gen3/6dof/urdf/gen3_macro.xacro.
# Continuous joints (1/4/6 without use_external_cable) get 2*pi — a safe bound either way.
JOINT_LIMITS = [6.28, 2.24, 2.57, 6.28, 2.09, 6.28]

FK_MOVE_TIME = 8          # sec; matches the hand-tested commands in info/ros_commands.txt
JOINT_TOLERANCE = 0.01    # rad, MoveIt goal window

GRIPPER_ACTION = "/robotiq_gripper_controller/gripper_cmd"
GRIPPER_LIMIT = 0.8       # 2F-140 mechanical close limit, per info/phri-reference-guide.md
GRIPPER_OPEN = 0.0
GRIPPER_CLOSED = 0.39      # tune against the real glass — GRIPPER_LIMIT crushes it
GRIPPER_MAX_EFFORT = 10.0  # N; lower if the glass complains

# Named arm poses in JOINT SPACE — the single table both backends consume.
# None = not teached yet: jog the arm, then `ros2 topic echo /joint_states`.
POSES = {
    "home":         [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "tuck":         [-1.57, 0.0, 1.57, 0.0, 0.0, 1.57],  # glass-transport pose, sim-tested
    "above_glass":  [-3.14, -0.9, 0.0, 0.0, 0.8, 1.57],
    "at_glass":     [-3.14, -0.9, 0.0, 0.0, 0.8, 1.57],
    "fill_coffee":  [-3.09, -0.6, 0.1, 0, 1, 1.57],
    "fill_water":   [-3.53, 0.3, 0.75, 0.0, 1.15, 1.57],
    "fill_water_wiggled_1":   [-3.53, 0.3, 0.75, 0.0, 1.15, 1.40],
    "fill_water_wiggled_2":   [-3.53, 0.3, 0.75, 0.0, 1.15, 1.80],
    "handover":     [-0.2, -0.9, -0.2, 0.0, 0.9, 1.57],
    "apriltag":     [-2, -0.5, 1.3, 1.9, -2, 0],
    "look_inside_cup":  [0, -1.57, 0, 0.0, -1.4, 0],
    "find_cup_pose": [-1.57, -2.3, -2.3, 0, 0, 1.57],
    "trashcan":     [0.4, 0.2, 1.77, 0.0, 0.0, 1.57], # rail 0.8
    # ------------- Drink Gestures -------------
    "look_at_user":                  [-0.1, 0.0, 1.77, 0.0, -0.2, 1.57],
    "look_at_user_question":         [-0.1, 0.0, 1.77, 0.0, -0.2, 1.4],  # intended as a slight tilting-head move after taking the position. 
    "look_at_user_kitchen":          [-2, 0.0, 1.77, 0.0, -0.2, 1.57],
    "look_at_user_question_kitchen": [-2, 0.0, 1.77, 0.0, -0.2, 1.4],
    "look_at_entrance":               [-2, 0.0, 1.57, -1.57, 0.6, 3.14],
    "gesture_coffee_1": [-1.9, -0.5, 1.9, 1.57, 1.8, -0.9],
    "gesture_coffee_2": [-1.9, -0.6, 2.3, 1.57, 1.8, -0.2],
    "gesture_water_1":  [-1.57, 0.2, 1.57, 0.0, 0.2, 1.57],
    "gesture_water_2":  [-1.57, -0.3, 2.3, 0.0, 0.8, 1.57],
    "gesture_tea_1":    [-1.57, -0.3, 1.57, 0.0, -0.6, 1.57],
    "gesture_tea_2":    [-1.57, -0.3, 1.57, 0.0, -0.3, 1.57],
    "feed_beer_1":        [-1.57, -1.57, -1.57, 0.0, 1, 1.57],
    "feed_beer_2":      [-1.57, -1.8, -1, 0.0, -1.2, 1.57]
    }

# Which fixed camera can see a person. NOT the arm camera: it rides the wrist,
# so what it sees says where the arm is pointed, not where the user is.
# Published by arm_camera_gesture_recognizer under its own node namespace.
PERSON_TOPICS = {
    "kitchen": "/kitchen_camera/person_present",
    "pc": "/usb_camera/person_present",
}

GRIPPER_POSES = {
    "coffee": 0.5,
    "tea": 0.7,
    "water": 0.0
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
    # Every drink on the menu must be mimeable, or do_gesture raises KeyError
    # halfway through a motion instead of before the node starts.
    for drink in MENU:
        for suffix in ("_1", "_2"):
            assert f"gesture_{drink}{suffix}" in POSES, \
                f"MENU has '{drink}' but POSES has no gesture_{drink}{suffix}"
        # .get default is deliberately out of range: missing and mistyped fail here.
        assert 0.0 <= GRIPPER_POSES.get(drink, -1.0) <= GRIPPER_LIMIT, \
            f"GRIPPER_POSES['{drink}'] missing or outside 0.0..{GRIPPER_LIMIT}"
    # ask_user builds its pose names from these; a missing one is a KeyError with
    # the arm already holding a glass. The derived yaw is checked by the joint-limit
    # loop above, which is the whole reason the _kitchen poses go into POSES.
    for name in ("look_at_user", "look_at_user_question"):
        assert name in POSES and name + "_kitchen" in POSES, \
            f"POSES needs {name}[_kitchen]"
    assert POSES["look_at_user_kitchen"] != POSES["look_at_user"], \
        "kitchen and PC poses are identical — one of the two directions is wrong"


#_check_poses()


def _closest_carriage(px):
    # "Closest" carriage = the one that nulls the point's X, clamped to the rail. The
    # sign is negative because the rail axis is inverted vs the arm (elmo-axis-mapping).
    return max(RAIL_MIN, min(RAIL_MAX, -px))


def _point_to_base(px, py, pz, carriage, lift):
    # Frame E (origin carriage=0/lift=0) -> base_link. Pure translation, 1:1 metres. A
    # clamped carriage leaves a nonzero X residual for the arm to stretch to.
    return (px + carriage, py, pz - lift)


def _check_transform():
    # In-range point: carriage nulls X, so the arm's X residual is ~0.
    c = _closest_carriage(0.3)
    assert abs(c + 0.3) < 1e-9, c
    x, y, z = _point_to_base(0.3, 0.5, 0.2, c, 0.35)
    assert abs(x) < 1e-9 and y == 0.5 and abs(z - (0.2 - 0.35)) < 1e-9, (x, y, z)
    # Clamp at both ends; a clamped point leaves the arm a nonzero X to reach.
    assert _closest_carriage(-999) == RAIL_MAX
    assert _closest_carriage(999) == RAIL_MIN
    assert abs(_point_to_base(5.0, 0, 0, _closest_carriage(5.0), 0)[0]) > 1e-6


_check_transform()


def fit_tilt_line(rows):
    # rows: (x, y, z, lift, measured_floor_height) tuples from measure() + a tape. Sag is a
    # pitch about Y and the arm sits below the mount, so the Y offset makes no torque about Y
    # and drops out. tilt(x) = a*x + c, and drop = tilt*x = a*x^2 + c*x. Fit on drop
    # (well-conditioned), NOT drop/x (blows up near the forward pose where x ~= 0).
    import numpy as np
    A, drops = [], []
    for x, y, z, lift, m in rows:
        base_h = FLOOR_BELOW_BASE_HOME + (lift - LIFT_HOME)   # base_link height above floor
        A.append([x * x, x])
        drops.append((base_h + z) - m)                        # predicted tip floor-height - measured
    (a, c), *_ = np.linalg.lstsq(np.array(A), np.array(drops), rcond=None)
    return float(a), float(c)


def _check_tilt_fit():
    # Round-trip: synthesize drops from a known (a, c), confirm the fit recovers them.
    a, c = 0.03, 0.005
    rows = []
    for x, z, lift in [(0.8, -0.6, 0.35), (0.4, -0.6, 0.35), (-0.7, -0.6, 0.35)]:
        drop = a * x * x + c * x
        m = (FLOOR_BELOW_BASE_HOME + (lift - LIFT_HOME) + z) - drop
        rows.append((x, 0.0, z, lift, m))
    fa, fc = fit_tilt_line(rows)
    assert abs(fa - a) < 1e-9 and abs(fc - c) < 1e-9, (fa, fc)


_check_tilt_fit()


# The order query_all_drinks mimes when the user waves the talking away. The
# suggestion goes first: they already heard it named, so it is the shortest path
# to a thumbs-up. MENU order for the rest — no ranking exists to do better.
def menu_from(drink):
    return (drink,) + tuple(d for d in MENU if d != drink)


def _check_menu_order():
    for drink in MENU:
        order = menu_from(drink)
        assert order[0] == drink, f"menu_from({drink}) put {order[0]} first"
        # Nothing dropped and nothing offered twice — the arm mimes this list.
        assert sorted(order) == sorted(MENU), f"menu_from({drink}) -> {order}"

_check_menu_order()


# ponytail: a schedule, not a state machine — each (at, action) fires once, when
# the clock passes it, and any answer ends the whole thing. Clock and sleep are
# arguments so _check_escalation below can run it on a fake clock, instantly.
# Ceiling: actions are checked for `done` only between steps, so a wiggle already
# under way finishes before we notice the answer. Deliberate — a half-move looks
# like a fault, and WIGGLE_MOVE_TIME keeps the overshoot to seconds.
def escalate(steps, done, now, sleep):
    for at, action in steps:
        while not done() and now() < at:
            sleep(WAIT_POLL)
        if done():
            return
        action()


def _check_escalation():
    for pose in WIGGLE_POSES:
        assert POSES.get(pose) is not None, f"WIGGLE_POSES: '{pose}' not teached"
    assert WIGGLE_AFTER < BLINK_AFTER < WATER_CONFIRM_TIMEOUT, \
        "escalation steps must be in ascending order"

    # Fake clock: sleep() is the only thing that moves it, so this runs instantly.
    clock = [0.0]

    def run(answer_at):
        clock[0] = 0.0
        fired = []
        escalate([(10.0, lambda: fired.append("wiggle")),
                  (20.0, lambda: fired.append("blink")),
                  (30.0, lambda: fired.append("give_up"))],
                 lambda: clock[0] >= answer_at,
                 lambda: clock[0],
                 lambda s: clock.__setitem__(0, clock[0] + s))
        return fired

    assert run(5.0) == [], "answered early — nothing should have fired"
    assert run(15.0) == ["wiggle"], "answered after the wiggle — no light"
    assert run(99.0) == ["wiggle", "blink", "give_up"], "silence runs the whole ladder"


_check_escalation()


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

        self._coffee_client = ActionClient(
            self, DispenseDrink, "dispense_drink", callback_group=cb)

        # Both arm backends stay wired; the parameter only picks which one sends.
        self.use_moveit = self.declare_parameter("use_moveit", True).value
        self._fk_client = ActionClient(
            self, FollowJointTrajectory, FK_ACTION, callback_group=cb)
        self._moveit_client = ActionClient(
            self, MoveGroup, MOVEIT_ACTION, callback_group=cb)

        self._gripper_client = ActionClient(
            self, GripperCommand, GRIPPER_ACTION, callback_group=cb)

        self._water_client = self.create_client(
            AskForWater, "/ask_for_water", callback_group=cb)
        self._light_client = self.create_client(
            SetBool, BLINK_SERVICE, callback_group=cb)

        # The wrist camera is the eye. The Event is what ask_user blocks on from a
        # skill thread while the executor spins; the string says which way it went.
        self._answered = threading.Event()
        self._gesture = ""
        self.create_subscription(
            String, GESTURE_EVENT_TOPIC, self._on_gesture, 10, callback_group=cb)

        # ...but the eye has to be pointed the right way first. False until a
        # recognizer says otherwise, which degrades to "assume kitchen" — the
        # direction the arm looked before any of this existed.
        self._person = {where: False for where in PERSON_TOPICS}
        for where, topic in PERSON_TOPICS.items():
            self.create_subscription(
                Bool, topic,
                lambda msg, w=where: self._person.__setitem__(w, msg.data),
                10, callback_group=cb
            )

        self._busy = False
        self._server = ActionServer(
            self, BringDrink, "bring_drink", self._execute,
            goal_callback=self._on_goal, callback_group=cb
        )

        # Look up when someone walks in. interaction_manager says the words on
        # this same event; the two halves are independent on purpose.
        self.create_subscription(
            Empty, "/user_arrived", self._on_arrival, 10, callback_group=cb
        )

        # Kitchen collision scene: re-publish after every Elmo move + seed once at startup.
        # base_link rides the rail and MoveIt won't re-transform a cached scene. Best-effort.
        if build_scene is not None:
            self._scene_client = self.create_client(
                ApplyPlanningScene, "/apply_planning_scene", callback_group=cb)
            self._validity_client = self.create_client(
                GetStateValidity, "/check_state_validity", callback_group=cb)
            self._scene_timer = self.create_timer(1.0, self._seed_scene, callback_group=cb)
        else:
            self.get_logger().warn(
                f"kitchen_scene not importable from {_SCRIPTS} — collision scene disabled")

        # Live TF for FK readback (measure). Pure rclpy — no numpy (dodges the numpy trap).
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

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
        # Off-menu goals are refused here, not routed to some default skill: a
        # wrong drink that LOOKS like it worked costs more than a rejected goal.
        if goal_request.drink not in MENU:
            self.get_logger().warn(
                f"[bring_drink] not on the menu: '{goal_request.drink}' "
                f"(have {', '.join(MENU)})")
            return GoalResponse.REJECT
        # One arm, one goal at a time — reject rather than queue or run concurrently.
        # Simple flag, not a hard lock. Two goals landing in the same instant could both pass
        if self._busy:
            self.get_logger().warn("[bring_drink] busy — rejecting goal")
            return GoalResponse.REJECT
        self._busy = True
        return GoalResponse.ACCEPT

    def _on_arrival(self, _msg):
        # Busy means a drink is in progress — quite possibly a full glass under
        # the tap. Swinging to the entrance mid-fetch is the one way this can do
        # damage, and an arrival while the arm is already serving someone is a
        # reconnecting E4 far more often than a person. Drop it, don't queue it.
        if self._busy:
            self.get_logger().info("[arrival] busy — not looking up")
            return
        self.move_arm("look_at_entrance")

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
        ps.world.collision_objects = build_scene(carriage, lift, TILT_DEG, TILT_AXIS)
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

    def _move_arm_fk(self, angles, seconds=FK_MOVE_TIME):
        # No IK, no collision checking: the joints go exactly where told.
        # `seconds` is the one timing knob in this node — MoveIt has none, it only
        # scales velocity, which is why the wiggle calls this directly.
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = JointTrajectory(
            joint_names=JOINTS,
            points=[JointTrajectoryPoint(
                positions=[float(a) for a in angles],
                time_from_start=Duration(sec=int(seconds),
                                         nanosec=int(seconds % 1 * 1e9)))])
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

    def _move_arm_pose(self, x, y, z):
        # The ONE place Cartesian IK lives. MoveIt only — FK has no IK. Position: a small
        # sphere at (x,y,z) in base_link. Orientation: gripper horizontal (side grasp, fingers
        # close level around the upright bottle), approach azimuth ~free because the bottle is
        # round. Mirrors the proven message shape in scripts/arm_teleop.py make_goal().
        if not self.use_moveit:
            raise RuntimeError("_move_arm_pose needs use_moveit:=true (FK has no IK)")
        self.get_logger().info(f"[arm] -> pose ({x:.3f}, {y:.3f}, {z:.3f}) horizontal")
        region = Pose()
        region.position.x, region.position.y, region.position.z = float(x), float(y), float(z)
        region.orientation.w = 1.0
        pc = PositionConstraint()
        pc.header.frame_id = "base_link"
        pc.link_name = EE_LINK
        pc.constraint_region = BoundingVolume(
            primitives=[SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[POSE_POS_TOL])],
            primitive_poses=[region])
        pc.weight = 1.0
        oc = OrientationConstraint()
        oc.header.frame_id = "base_link"
        oc.link_name = EE_LINK
        (oc.orientation.x, oc.orientation.y,
         oc.orientation.z, oc.orientation.w) = GRIPPER_HORIZ_QUAT
        # ponytail: HORIZ_QUAT maps EE y -> base vertical (math, not measured), so the free
        # azimuth rides the Y tolerance. Wrong tilt in a test? swap which axis gets POSE_YAW_TOL.
        oc.absolute_x_axis_tolerance = POSE_LEVEL_TOL
        oc.absolute_y_axis_tolerance = POSE_YAW_TOL
        oc.absolute_z_axis_tolerance = POSE_LEVEL_TOL
        oc.weight = 1.0
        goal = MoveGroup.Goal()
        req = goal.request
        req.group_name = MOVE_GROUP
        req.num_planning_attempts = 16
        req.allowed_planning_time = 10.0
        req.max_velocity_scaling_factor = 0.3
        req.max_acceleration_scaling_factor = 0.3
        req.goal_constraints.append(
            Constraints(position_constraints=[pc], orientation_constraints=[oc]))
        # Hold the SAME level orientation for the whole path so a held glass never tips.
        # ponytail: the arm must ALREADY be level at the start or constrained planning fails
        # instantly — call this only from a level pose. Constrained planning is slower; the
        # 10s / 16 attempts above is the budget, bump it if this starts timing out.
        # req.path_constraints = Constraints(orientation_constraints=[oc])
        goal.planning_options.plan_only = False
        result = self._send(self._moveit_client, goal)
        if result.error_code.val != 1:
            raise RuntimeError(f"MoveIt pose planning failed: error_code={result.error_code.val}")

    def _gripper(self, target_pos):
        # Sim does not have a working gripper. Do a check, then skip. 
        if not self._gripper_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn(
                f"[gripper] no action server — skipping -> {target_pos} (sim?)")
            return
        self.get_logger().info(f"[gripper] -> {target_pos}")
        goal = GripperCommand.Goal()
        goal.command.position = float(target_pos)
        goal.command.max_effort = GRIPPER_MAX_EFFORT
        result = self._send(self._gripper_client, goal)
        # Holding a glass means stalled at max effort BEFORE reaching the setpoint —
        # that is a successful grasp, so reached_goal alone would raise on every pick.
        if not (result.reached_goal or result.stalled):
            raise RuntimeError(f"gripper stuck at {result.position}")

    def _on_gesture(self, msg):
        # No filter: the recognizer publishes an event only for the three gestures
        # above, and all three end the wait — sitting out the rest of the timeout
        # after the user has already answered reads as a hang.
        self.get_logger().info(f"[gesture] {msg.data}")
        self._gesture = msg.data
        self._answered.set()

    def _wiggle(self):
        # FK directly, on purpose: this is a wrist twist inside a pose the arm is
        # already holding, so there is nothing for MoveIt to plan around — and FK
        # is the only backend that takes the duration, which is the whole point.
        self.get_logger().info("[water] nudging the user")
        for pose in WIGGLE_POSES:
            self._move_arm_fk(POSES[pose], WIGGLE_MOVE_TIME)

    def _blink(self, on):
        # Fail-open like the prompt itself, and fire-and-forget: nothing downstream
        # depends on the light having actually changed.
        if not self._light_client.service_is_ready():
            if on:
                self.get_logger().warn(f"[water] {BLINK_SERVICE} unavailable — no light")
            return
        self._light_client.call_async(SetBool.Request(data=on))

    def _ask_for_water(self):
        # Hold the pose under the tap while interaction_manager talks to the user,
        # nudging harder the longer they stay quiet — see escalate().
        # Blocking is fine: another thread spins, same as the action clients.
        # FAIL-OPEN — no dialog stack (sim, ASR down) must not make water goals
        # impossible; the caller hands over an empty glass rather than wedging.
        if not self._water_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("[water] /ask_for_water unavailable — skipping the prompt")
            return False

        # async, unlike every other call here: the whole feature is doing something
        # while this is pending. The ear belongs to interaction_manager and stays
        # open the entire time, wiggle or not.
        future = self._water_client.call_async(
            AskForWater.Request(timeout=WATER_CONFIRM_TIMEOUT))
        start = time.monotonic()
        try:
            escalate(
                [(WIGGLE_AFTER, self._wiggle),
                 (BLINK_AFTER, lambda: self._blink(True)),
                 (WATER_CONFIRM_TIMEOUT + ANSWER_GRACE, lambda: None)],
                future.done,
                lambda: time.monotonic() - start,
                time.sleep,
            )
        finally:
            self._blink(False)   # never leave the lab light flashing

        if not future.done():
            # Only reachable if interaction_manager died holding the request — its
            # own timeout answers first in every normal case.
            future.cancel()
            self.get_logger().warn("[water] no answer at all — giving up")
            return False

        result = future.result()
        confirmed = result is not None and result.confirmed
        self.get_logger().info(f"[water] confirmed={confirmed}")
        return confirmed

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

    def move_to_point(self, px, py, pz):
        # Reach a point in frame E (origin = carriage 0 / lift 0, the rail zero), gripper
        # horizontal. Carriage slides to null X (bounded = "closest"); lift is READ, not moved
        # (grab by dropping the lift afterwards, as a separate skill); the arm IKs the rest.
        # CLI args arrive as strings -> float().
        px, py, pz = float(px), float(py), float(pz)
        carriage = _closest_carriage(px)
        self.move_rail(carriage)              # tucks first, collision-safe
        lift = self._elmo_pos["lift"]
        if lift is None:
            raise RuntimeError("no lift feedback — cannot place the point in base_link")
        self._move_arm_pose(*_point_to_base(px, py, pz, carriage, lift))

    def _lift_path_clear(self, target):
        # Elmo isn't a MoveIt joint, so MoveIt can't plan the lift. Instead: freeze the arm,
        # virtually move the kitchen boxes to each height the arm sweeps through, and ask
        # /check_state_validity if the current arm state collides. is_diff=True + empty state
        # means "the arm where it is right now". /apply_planning_scene is a service (synchronous
        # apply), so the scene is live before each check. Dynamic obstacles enter the same scene
        # and get checked for free. FAIL-OPEN: if the check can't run, warn loudly and allow.
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
            ps = PlanningScene(is_diff=True)
            ps.world.collision_objects = build_scene(carriage, lift, TILT_DEG, TILT_AXIS)
            self._scene_client.call(ApplyPlanningScene.Request(scene=ps))
            req = GetStateValidity.Request()
            req.robot_state.is_diff = True  # empty state + is_diff = current monitored arm
            req.group_name = MOVE_GROUP
            result = self._validity_client.call(req)
            if result is not None and not result.valid:
                self.get_logger().warn(f"[lift-check] collision at lift={lift:.3f} — blocked")
                clear = False
                break

        self._publish_scene()  # restore the scene to where the arm actually is
        return clear

    def test_tilt(self, deg, axis="x", parent="base_link"):
        # POC: broadcast `<parent>_tilted`, rotated `deg` about `axis`, held live at 10 Hz
        # so it shows in RViz. NOT the real fix — a lone triad, the arm does not follow it
        # (arm is parented to base_link by robot_state_publisher). Proves the plumbing +
        # lets you eyeball the angle. Ctrl-C to stop. See memory project_tilt_compensation.
        import math
        from tf2_ros import TransformBroadcaster
        from geometry_msgs.msg import TransformStamped
        br = TransformBroadcaster(self)
        half = math.radians(float(deg)) / 2.0
        t = TransformStamped()
        t.header.frame_id = parent
        t.child_frame_id = f"{parent}_tilted"
        q = t.transform.rotation
        setattr(q, axis, math.sin(half))   # axis in {x,y,z}
        q.w = math.cos(half)
        self.get_logger().info(f"[tilt-test] {parent} -> {parent}_tilted: {deg} deg about {axis}. Ctrl-C.")
        while rclpy.ok():
            t.header.stamp = self.get_clock().now().to_msg()
            br.sendTransform(t)
            time.sleep(0.1)

    def measure(self):
        # FK of the EE in base_link RIGHT NOW, no motion. Call it in any pose you care about
        # (full_extend, above_glass, ...) then tape the tip's floor-height -> one calibration
        # tuple for the sag fit. This is the *nominal/level* FK (the sag lives above base_link,
        # not in the joints), i.e. the predicted tip to diff against your measured one.
        # Print x,y (plane offset) + z & lift (floor-height -> drop -> angle). See project-tilt-compensation.
        from rclpy.time import Time
        from rclpy.duration import Duration as RclDuration
        tf = self._tf_buffer.lookup_transform(
            "base_link", EE_LINK, Time(), timeout=RclDuration(seconds=2.0))
        t = tf.transform.translation
        lift = self._elmo_pos["lift"]
        self.get_logger().info(
            f"[measure] ee=({t.x:.4f}, {t.y:.4f}, {t.z:.4f}) lift={lift}  # tape tip floor-height now")
        return (t.x, t.y, t.z, lift)

    def move_and_measure(self, joint_1):
        # Sweep only joint_1 across the fully-extended pose (= full_extend with a swept base),
        # then measure. FK backend: direct + collision-bypassed on purpose — clear the
        # workspace, this is a calibration reach, not a production move.
        a = float(joint_1)
        assert abs(a) <= JOINT_LIMITS[0], f"joint_1={a} exceeds +/-{JOINT_LIMITS[0]}"
        self._move_arm_fk([a, -1.57, 0.0, 0.0, 0.0, 1.57])
        return self.measure()

    def calibrate_tilt(self, *rows):
        # OFFLINE, run once with the measure() tuples. Each arg: "x,y,z,lift,measured".
        # Prints (a, c) -> paste into TILT_LINE. Take a 4th, held-out pose to confirm: if the
        # drop at a near-forward (x ~= 0) pose isn't ~0, a 2nd axis is leaking in. NOT WIRED yet.
        parsed = [tuple(float(v) for v in r.split(",")) for r in rows]
        a, c = fit_tilt_line(parsed)
        self.get_logger().info(f"[calibrate] TILT_LINE = ({a:.5f}, {c:.5f}) from {len(rows)} rows")
        return a, c

    def _tilt_now(self):
        # Live pitch (deg) about Y for the CURRENT pose, from the fitted TILT_LINE. Will feed
        # build_scene (TILT_AXIS='y') at its two call sites once TILT_LINE is real. NOT WIRED yet.
        from rclpy.time import Time
        from rclpy.duration import Duration as RclDuration
        tf = self._tf_buffer.lookup_transform(
            "base_link", EE_LINK, Time(), timeout=RclDuration(seconds=2.0))
        x = tf.transform.translation.x
        a, c = TILT_LINE
        return math.degrees(a * x + c)

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
        self.move_arm(f"fill_{drink}")

    def _request_coffee_machine(self, drink):
        if not self._coffee_client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError(
                "/dispense_drink unavailable - is coffee_machine_actuator up?"
            )
        self.get_logger().info(f"[coffee] dispensing -> {drink}")
        goal = DispenseDrink.Goal(
            beverage=str(drink),
            reason="arm at fill_coffee",
        )
        result = self._send(self._coffee_client, goal)
        if not result.success:
            raise RuntimeError(f"coffee machine failed: {result.status}")
        self.get_logger().info(f"[coffee] {result.status}")

    def do_gesture(self, drink):
        self._gripper(GRIPPER_POSES[drink])
        pose = f"gesture_{drink}_"
        for _ in range(2):
            self.move_arm(pose + "1")
            self.move_arm(pose + "2")
        self.open_gripper() # For unobstructed viewing of user

    def ask_user(self, timeout=GESTURE_TIMEOUT):
        # Face the user, tilt the wrist ("well?"), then wait. The two moves are
        # part of the question, not setup: the camera rides the wrist, so until it
        # points at the user no thumb can be seen — which is also why WHICH way it
        # turns has to be decided here, off the fixed cameras.
        # Clearing here and not before the moves means a thumb given during the
        # mime (camera facing away, nothing detectable) cannot leak in as an answer.
        # Returns the gesture; "" = nobody answered. Three answers do not fit in a
        # bool, and "" is the failure value everywhere else in the dialog layer.
        self._answered.clear()
        self._gesture = ""
        # The PC pose is the default and the tie-break — it is the one the arm has
        # always used, so it is the known-safe swing. Turning to the kitchen needs
        # positive evidence: someone there AND nobody at the PC. Two people, or a
        # camera that never came up, and the arm stays with what it did before.
        at_kitchen = self._person["kitchen"] and not self._person["pc"]
        self.get_logger().info(
            f"[gesture] asking towards {'kitchen' if at_kitchen else 'pc'}")
        for pose in ("look_at_user", "look_at_user_question"):
            self.move_arm(pose + "_kitchen" if at_kitchen else pose)
        self._answered.wait(float(timeout))   # return redundant: "" already says it
        self.get_logger().info(f"[gesture] answer={self._gesture or 'none'}")
        return self._gesture

    def query_all_drinks(self, *drinks):
        # menu order — put the estimator's likeliest drink first once wired.
        self.move_rail(0.8)
        for drink in drinks or MENU:
            self.do_gesture(drink)
            answer = self.ask_user()
            if answer == GESTURE_UP:
                return drink
            if answer == GESTURE_PALM:
                self.get_logger().info("[gesture] open palm — done offering")
                break
            # thumbs_down, silence, and anything a future recognizer adds: keep
            # going. Falling through to the next drink is the safe default.
        return ""


    def handover(self):
        self.move_rail(RAIL_HANDOVER)
        self.move_lift(LIFT_HOME)
        self.move_arm("handover"); 
        self.move_lift(LIFT_HANDOVER)
        self.open_gripper()
        self.move_lift(LIFT_MIN)
        self.tuck()
        self.move_lift(LIFT_HOME)
        
    def bring_water_simple(self):
        self.pick_glass()
        self.move_rail(RAIL_KITCHEN)
        self.fill("water")
        self._ask_for_water()
        self.handover()

    def coffee_machine_approach(self):
        self.move_rail(RAIL_KITCHEN + 0.1)
        self.move_lift(LIFT_HOME + 0.01)
        self.move_arm("fill_coffee")
        # DANGEROUS Move without tuck. use with CAUTION!
        self._move_elmo("carriage", RAIL_KITCHEN)
        

    def coffee_machine_departure(self):
        # DANGEROUS Move without tuck. use with CAUTION!
        self._move_elmo("carriage", RAIL_KITCHEN + 0.1)
        self.move_lift(LIFT_HOME)
        self.tuck()


    def bring_coffee_machine_drink_simple(self, drink):
        self.pick_glass()
        self.coffee_machine_approach()
        self._request_coffee_machine(drink)
        self.coffee_machine_departure()
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
            if goal_handle.request.offer_menu:
                # The user waved the talking away, so the arm asks instead — mime
                # the menu and bring whatever gets a thumbs-up. Inside the try: the
                # mime is motion and a raised move must abort like any other.
                drink = self.query_all_drinks(*menu_from(drink))
                if not drink:
                    # Palmed again, or thumbed everything down. Asked and answered.
                    self.get_logger().info("[bring_drink] nothing chosen")
                    goal_handle.succeed()
                    return BringDrink.Result(success=False)

            # Two routes, MENU says which: sink (None) or coffee machine.
            # _on_goal already rejected anything not in MENU.
            beverage = MENU[drink]
            if beverage is None:
                self.bring_water_simple()
            else:
                self.bring_coffee_machine_drink_simple(beverage)
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
