#!/usr/bin/env python3

import time
import threading
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import String
from brewbot_interfaces.action import SuggestDrink, BringDrink
from brewbot_interfaces.srv import ClassifyText

SPEECH_TIMEOUT = 30.0


class State(Enum):
    IDLE = auto()
    ASKING = auto()
    CLASSIFYING = auto()
    BRINGING = auto()


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
            self, SuggestDrink, "suggest_drink", self._execute, callback_group=cb
        )

        self._state = State.IDLE
        self._speech_event = threading.Event()
        self._speech_text = None

        self.get_logger().info("Suggestion handler ready")

    def _on_speech(self, msg):
        if self._state == State.ASKING:
            self.get_logger().info(f"[SPEECH] Received: '{msg.data}'")
            self._speech_text = msg.data
            self._speech_event.set()
        else:
            self.get_logger().debug(f"[SPEECH] Ignored (state={self._state.name}): '{msg.data}'")

    def _execute(self, goal_handle):
        drink = goal_handle.request.drink
        self.get_logger().info(f"[GOAL] Received suggestion for: '{drink}'")

        # ASKING — publish TTS, wait for speech
        self._state = State.ASKING
        self._speech_event.clear()
        self._speech_text = None

        self.get_logger().info(f"[ASKING] Publishing TTS: 'Would you like a {drink}?'")
        goal_handle.publish_feedback(SuggestDrink.Feedback(status="asking_user"))
        self._tts_pub.publish(String(data=f"Would you like a {drink}?"))
        self.get_logger().info(f"[ASKING] Waiting up to {SPEECH_TIMEOUT}s for user response...")

        if not self._speech_event.wait(timeout=SPEECH_TIMEOUT):
            self.get_logger().warn("[ASKING] Timed out — no response received, aborting")
            self._state = State.IDLE
            goal_handle.abort()
            return SuggestDrink.Result(accepted=False)

        # CLASSIFYING — call text_classification service
        self._state = State.CLASSIFYING
        self.get_logger().info(f"[CLASSIFYING] Classifying: '{self._speech_text}'")
        goal_handle.publish_feedback(SuggestDrink.Feedback(status="classifying"))

        req = ClassifyText.Request()
        req.data = self._speech_text
        future = self._classify_client.call_async(req)
        while not future.done():
            time.sleep(0.05)

        classification = future.result().result
        self.get_logger().info(f"[CLASSIFYING] Result: {classification}")

        if classification != "YES":
            self.get_logger().info("[CLASSIFYING] User declined, goal succeeded with accepted=False")
            self._state = State.IDLE
            goal_handle.succeed()
            return SuggestDrink.Result(accepted=False)

        # BRINGING — send BringDrink action goal
        self._state = State.BRINGING
        self.get_logger().info(f"[BRINGING] User accepted — sending BringDrink goal for '{drink}'")
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

        self._state = State.IDLE
        self.get_logger().info("[DONE] Goal succeeded, accepted=True")
        goal_handle.succeed()
        return SuggestDrink.Result(accepted=True)


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
