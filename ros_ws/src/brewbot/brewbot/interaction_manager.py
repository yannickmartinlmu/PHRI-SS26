#!/usr/bin/env python3
"""Interaction manager — owns the mouth (TTS) and the ear (ASR) as one resource.

Two layers, same split as arm_controller:
  _ask(text, timeout)  the dialog leaf: say it, wait, classify. One at a time.
  action/service       orchestration: what to say and what to do with the answer.

Both layers are reachable from other nodes, so the leaf carries its own lock.
INVARIANT: never hold the mouth across an arm call — arm_controller calls
/ask_for_water mid-motion while _execute is parked waiting on BringDrink.

To query the Interaction Manager from a terminal, use
ros2 action send_goal -f /suggest_drink brewbot_interfaces/action/SuggestDrink "{drink: 'water'}"
"""

import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import String
from brewbot_interfaces.action import SuggestDrink, BringDrink
from brewbot_interfaces.srv import ClassifyText, AskForWater

SPEECH_TIMEOUT = 30.0

# The caller's timeout bounds the LISTENING. This bounds the classifier, which is a
# separate way to hang — and _ask holds the mouth throughout, so an unbounded wait
# here would wedge every later ask, not just this one. Encoding takes well under 1s.
CLASSIFY_TIMEOUT = 5.0

WATER_PROMPT = ("I am ready. Please fill as much water as you like, "
                "then tell me when you are done.")
WATER_GIVEUP = "No answer. I will put the glass back."


class SuggestionHandlerNode(Node):

    def __init__(self):
        super().__init__("interaction_manager")

        cb = ReentrantCallbackGroup()

        self._tts_pub = self.create_publisher(String, "/tts_text", 10)

        self._speech_sub = self.create_subscription(
            String, "/speech_text", self._on_speech, 10, callback_group=cb
        )

        self._classify_client = self.create_client(
            ClassifyText, "/classify_yes_no", callback_group=cb
        )

        self._bring_client = ActionClient(
            self, BringDrink, "bring_drink", callback_group=cb
        )

        self._action_server = ActionServer(
            self, SuggestDrink, "suggest_drink", self._execute,
            goal_callback=self._on_goal, callback_group=cb
        )

        # The arm calls this from under the tap, while _execute below is parked
        # in its BringDrink wait — hence ReentrantCallbackGroup on both.
        self.create_service(
            AskForWater, "/ask_for_water", self._on_ask_for_water, callback_group=cb
        )

        self._mouth = threading.Lock()   # one mouth, one ear, one user
        self._busy = False
        self._speech_event = threading.Event()
        self._speech_text = None

        self.get_logger().info("Interaction manager ready")

    def _on_goal(self, goal_request):
        # One conversation at a time. Rejecting beats letting a second goal fall
        # through _ask and report "user declined" when we never actually asked.
        if self._busy:
            self.get_logger().warn("[suggest_drink] busy — rejecting goal")
            return GoalResponse.REJECT
        self._busy = True
        return GoalResponse.ACCEPT

    def _on_speech(self, msg):
        # The lock IS the "we are listening" flag — no separate state field to
        # keep in sync, and nothing for a nested ask to clobber on its way out.
        if self._mouth.locked():
            self.get_logger().info(f"[SPEECH] Received: '{msg.data}'")
            self._speech_text = msg.data
            self._speech_event.set()
        else:
            self.get_logger().debug(f"[SPEECH] Ignored (not asking): '{msg.data}'")

    # ---- the dialog leaf: the ONE place the robot talks and listens ----

    def _ask(self, text, timeout):
        # Returns the classifier's own vocabulary ("YES"/"NO"/...); "" = nobody
        # answered. Three-valued because a silent user and a refusing user are
        # different outcomes to every caller.
        if not self._mouth.acquire(blocking=False):
            self.get_logger().warn(f"[ASK] already talking — dropped: '{text}'")
            return ""
        try:
            self._speech_event.clear()
            self._speech_text = None

            self.get_logger().info(f"[ASK] TTS: '{text}' (up to {timeout}s)")
            self._tts_pub.publish(String(data=text))

            if not self._speech_event.wait(timeout=timeout):
                self.get_logger().warn("[ASK] timed out — no response")
                return ""

            self.get_logger().info(f"[ASK] classifying: '{self._speech_text}'")
            request = ClassifyText.Request()
            request.data = self._speech_text
            future = self._classify_client.call_async(request)
            deadline = time.monotonic() + CLASSIFY_TIMEOUT
            while not future.done():
                if time.monotonic() > deadline:
                    future.cancel()
                    self.get_logger().error("[ASK] classifier did not answer — is nlp up?")
                    return ""
                time.sleep(0.05)

            answer = future.result().result
            self.get_logger().info(f"[ASK] -> {answer}")
            return answer
        finally:
            self._mouth.release()

    # ---- orchestration ----

    def _on_ask_for_water(self, request, response):
        response.confirmed = self._ask(WATER_PROMPT, request.timeout) == "YES"
        if not response.confirmed:
            self._tts_pub.publish(String(data=WATER_GIVEUP))
        return response

    def _execute(self, goal_handle):
        drink = goal_handle.request.drink
        self.get_logger().info(f"[GOAL] Received suggestion for: '{drink}'")
        try:
            goal_handle.publish_feedback(SuggestDrink.Feedback(status="asking_user"))
            answer = self._ask(f"Would you like a {drink}?", SPEECH_TIMEOUT)

            if not answer:
                self.get_logger().warn("[GOAL] no response — aborting")
                goal_handle.abort()
                return SuggestDrink.Result(accepted=False)

            if answer != "YES":
                self.get_logger().info("[GOAL] user declined")
                goal_handle.succeed()
                return SuggestDrink.Result(accepted=False)

            # Mouth already released — the sink prompt needs it during this call.
            self.get_logger().info(f"[BRINGING] sending BringDrink goal for '{drink}'")
            goal_handle.publish_feedback(SuggestDrink.Feedback(status="bringing"))

            if not self._bring_client.wait_for_server(timeout_sec=3.0):
                self.get_logger().warn("[BRINGING] No bring_drink server found, skipping")
            else:
                bring_future = self._bring_client.send_goal_async(
                    BringDrink.Goal(drink=drink)
                )
                while not bring_future.done():
                    time.sleep(0.05)

                result_future = bring_future.result().get_result_async()
                while not result_future.done():
                    time.sleep(0.05)

                success = result_future.result().result.success
                self.get_logger().info(f"[BRINGING] BringDrink completed, success={success}")

            self.get_logger().info("[DONE] Goal succeeded, accepted=True")
            goal_handle.succeed()
            return SuggestDrink.Result(accepted=True)
        finally:
            self._busy = False  # never leave the manager wedged as busy


def main():
    rclpy.init()
    node = SuggestionHandlerNode()
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
