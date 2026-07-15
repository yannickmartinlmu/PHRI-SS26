#!/usr/bin/env python3
"""Robot arm controller — owns the arm, gripper, and Elmo rail as one resource.

BringDrink action server. Each brew step is a *method* (skill), not a node: one
arm / rail / gripper means strictly sequential use, so methods in order beat
cross-node arbitration. Skills are stubs here — fill each with a canned motion.

Every hardware path is funnelled through one primitive (_move_arm / _gripper /
_move_elmo) so the joint-constraint-vs-pose-goal question — teleop is only proven
in sim so far — is decided in exactly one place without touching the skills.
"""

import sys
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import Float32
from brewbot_interfaces.action import BringDrink

# Elmo rail setpoints (Float32). carriage = base_link -X, lift = Z. See elmo-axis-mapping.
ELMO_CARRIAGE_SET = "/elmo/id1/carriage/position/set"
ELMO_CARRIAGE_GET = "/elmo/id1/carriage/position/get"

# Rail carriage targets. Metric scale and positions ASSUMED 1 unit = 1 m — unconfirmed.
RAIL_STATION = 0.0    # drink-filling station
RAIL_USER = 1.0       # handover position


class ArmController(Node):

    def __init__(self):
        super().__init__("arm_controller")
        cb = ReentrantCallbackGroup()

        # Elmo rail: setpoint out, feedback in.
        self._elmo_pub = self.create_publisher(Float32, ELMO_CARRIAGE_SET, 10)
        self._elmo_pos = None
        self.create_subscription(
            Float32, ELMO_CARRIAGE_GET, self._on_elmo, 10, callback_group=cb
        )

        # TODO: arm = MoveGroup client (/move_action); gripper = GripperCommand
        # (/robotiq_gripper_controller/gripper_cmd). Wire when the motion path is chosen.

        self._busy = False
        self._server = ActionServer(
            self, BringDrink, "bring_drink", self._execute,
            goal_callback=self._on_goal, callback_group=cb
        )

        self.get_logger().info("Arm controller ready")

    def _on_goal(self, goal_request):
        # One arm, one goal at a time — reject rather than queue or run concurrently.
        # Simple flag, not a hard lock. Two goals landing in the same instant could both pass
        if self._busy:
            self.get_logger().warn("[bring_drink] busy — rejecting goal")
            return GoalResponse.REJECT
        self._busy = True
        return GoalResponse.ACCEPT

    def _on_elmo(self, msg):
        self._elmo_pos = msg.data

    # ---- motion primitives: the ONE place each hardware path gets implemented ----

    def _move_arm(self, target_pose_name):
        # Joint-constraint vs pose-goal undecided (teleop only tested in  sim).
        # Reuse arm_teleop's /move_action client; both brew poses live here so
        # the skills below never change when the path is chosen.
        self.get_logger().info(f"[arm] -> {target_pose_name} (stub)")

    def _gripper(self, target_pos):
        # TODO: GripperCommand action, 0.0 open / 0.8 close (2F-140).
        self.get_logger().info(f"[gripper] {target_pos} (stub)")

    def _move_elmo(self, target_carriage_position):
        # TODO: after publishing, wait for _elmo_pos to settle near target.
        self.get_logger().info(f"[elmo] carriage -> {target_carriage_position} (stub)")
        self._elmo_pub.publish(Float32(data=float(target_carriage_position)))

    # ---- skills: individually callable (ros2 run brewbot arm_controller <skill>) ----

    def home(self):
        self._move_arm("home")

    def tuck(self):
        self._move_arm("tuck")

    def move_rail(self, target_carriage_position):
        self.tuck()  # INVARIANT: arm safe-by-construction before ANY rail move
        self._move_elmo(target_carriage_position)

    def open_gripper(self):
        self._gripper(0.0)

    def close_gripper(self):
        self._gripper(0.8)

    def pick_glass(self):
        self._move_arm("above_glass"); self.open_gripper()
        self._move_arm("at_glass");    self.close_gripper()
        self._move_arm("above_glass")

    def fill(self, drink):
        self._move_arm(f"fill_{drink}")

    def handover(self):
        self._move_arm("handover"); self.open_gripper()

    # ---- orchestration: BringDrink = skills in sequence ----

    def _execute(self, goal_handle):
        drink = goal_handle.request.drink
        self.get_logger().info(f"[bring_drink] {drink}")
        try:
            self.tuck()
            self.move_rail(RAIL_STATION)
            self.pick_glass()
            self.fill(drink)
            self.move_rail(RAIL_USER)
            self.handover()
            goal_handle.succeed()
            return BringDrink.Result(success=True)
        finally:
            self._busy = False  # never leave the controller wedged as busy


def main():
    rclpy.init()
    node = ArmController()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    # Address one skill directly: `ros2 run brewbot arm_controller pick_glass` (or `fill coffee`).
    args = [a for a in sys.argv[1:] if not a.startswith("-")]
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
