#!/usr/bin/env python3
"""Trigger — decides WHEN to interrupt the user.

state_estimator answers what the user needs but never initiates; this node is
the one that initiates. Four ways in, all landing on _fire():
  - keyboard: Enter (ask the estimator), a drink name (skip it, force one), or
    a drink and a reason ("water hot") to force the greeting's angle too
  - auto:=true: the same query on a 60s loop
  - /user_arrived: someone put the E4 on (see sensor_e4)
  - /user_spike: the estimator's reading crossed into "worth an offer"

All of them are dead until /user_arrived lands — nobody is there to talk to
before that. Typing "arrived" publishes it, same message the E4 sends, so the
arm looks up and the manager welcomes exactly as it would with a real band.

Keeping the policy here is why the estimator stays a passive service — swap
this node for an arrival/gesture trigger and nothing downstream changes.
"""

import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import Empty
from brewbot_interfaces.action import SuggestDrink
from brewbot_interfaces.srv import GetUserState
from brewbot.drinks import MENU

# Doubles as the cooldown after a declined offer.
LOOP_PERIOD = 60.0


class TriggerNode(Node):

    def __init__(self):
        super().__init__("trigger")

        cb = ReentrantCallbackGroup()

        self._state_client = self.create_client(
            GetUserState, "/get_user_state", callback_group=cb
        )
        self._suggest_client = ActionClient(
            self, SuggestDrink, "suggest_drink", callback_group=cb
        )

        # _fire blocks for the whole conversation; without this the timer would
        # stack blocked threads behind a user who is still being talked to.
        self._busy = False

        # ponytail: one-way latch — no /user_left exists anywhere in the package,
        # so re-arming has nothing to hang off. Reset it here if one shows up.
        self._arrived = False

        # Own publication, own subscription: "arrived" from the keyboard goes out
        # on the wire so arm_controller and interaction_manager see it too.
        self._arrival_pub = self.create_publisher(Empty, "/user_arrived", 10)

        # Reentrant + _busy: _fire blocks here for the whole conversation, and
        # the estimator query it makes is a service call on this same executor.
        self.create_subscription(
            Empty, "/user_arrived", self._on_arrival, 10, callback_group=cb
        )
        # Empty, not the drink: _fire() re-queries so the greeting still gets
        # the `reason` string. One redundant service call, no new interface.
        self.create_subscription(
            Empty, "/user_spike", lambda _: self._fire(), 10, callback_group=cb
        )

        self.declare_parameter("auto", False)
        if self.get_parameter("auto").get_parameter_value().bool_value:
            self.create_timer(LOOP_PERIOD, self._fire, callback_group=cb)
            self.get_logger().info(f"auto mode: querying every {LOOP_PERIOD}s")

        threading.Thread(target=self._keyboard, daemon=True).start()

        self.get_logger().info(
            "Trigger ready — waiting for /user_arrived (type 'arrived' to fake it). "
            "Then Enter to ask the estimator, or type a drink (optionally + a "
            "reason, e.g. 'water hot'): " + ", ".join(MENU)
        )

    def _keyboard(self):
        # Line-based, not raw keys. First word is the drink, the rest is the
        # reason — free text, it only ever lands in the LLM greeting prompt.
        for line in sys.stdin:
            drink, _, reason = line.strip().partition(" ")
            if drink == "arrived":
                # Publish rather than set the latch: _on_arrival runs off the
                # subscription, so the fake path is the real path.
                self._arrival_pub.publish(Empty())
                continue
            self._fire(drink or None, reason.strip())

    def _on_arrival(self, _msg):
        self._arrived = True
        self._fire()

    # reason defaults empty: a hand-typed drink has no state behind it unless
    # the typist supplied one.
    def _fire(self, drink=None, reason=""):
        if not self._arrived:
            self.get_logger().warn("[trigger] nobody has arrived yet — ignored")
            return
        if self._busy:
            self.get_logger().warn("[trigger] already running — ignored")
            return
        self._busy = True
        try:
            if drink is None:
                if not self._state_client.wait_for_service(timeout_sec=2.0):
                    self.get_logger().error(
                        "[trigger] /get_user_state unavailable — is state_estimator up?"
                    )
                    return
                state = self._state_client.call(GetUserState.Request())
                drink = state.drink
                reason = state.state   # rides along so the greeting can use it
                self.get_logger().info(f"[trigger] state={state.state} drink={drink or '-'}")

            if not drink:
                self.get_logger().info("[trigger] nothing worth suggesting")
                return

            # Covers both ways in: a typo'd line and, if decide() ever drifts,
            # the estimator too. Cheaper than starting a conversation about a
            # drink the arm will refuse to fetch.
            if drink not in MENU:
                self.get_logger().warn(
                    f"[trigger] not on the menu: '{drink}' — have {', '.join(MENU)}")
                return

            self._suggest(drink, reason)
        finally:
            self._busy = False

    def _suggest(self, drink, reason=""):
        if not self._suggest_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error(
                "[trigger] suggest_drink unavailable — is interaction_manager up?"
            )
            return

        goal_future = self._suggest_client.send_goal_async(
            SuggestDrink.Goal(drink=drink, reason=reason)
        )
        while not goal_future.done():
            time.sleep(0.05)

        handle = goal_future.result()
        if not handle.accepted:
            # IM rejects while it is mid-conversation — expected, not an error.
            self.get_logger().warn("[trigger] goal rejected — manager is busy")
            return

        result_future = handle.get_result_async()
        while not result_future.done():
            time.sleep(0.05)

        accepted = result_future.result().result.accepted
        self.get_logger().info(f"[trigger] user accepted={accepted}")


def main():
    rclpy.init()
    node = TriggerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
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
