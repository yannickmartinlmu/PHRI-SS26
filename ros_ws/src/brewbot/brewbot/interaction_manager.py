#!/usr/bin/env python3
"""Interaction manager — owns TTS+ASR as one resource; _ask() is the dialog leaf.

Never hold the mouth across an arm call: the arm calls /ask_for_water mid-motion
while _execute is parked waiting on BringDrink.

  ros2 action send_goal -f /suggest_drink brewbot_interfaces/action/SuggestDrink "{drink: 'water'}"
"""

import sys
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import Empty, String
from brewbot_interfaces.action import SuggestDrink, BringDrink
from brewbot_interfaces.srv import AskLLM, AskForWater
from brewbot.drinks import MENU
from brewbot import gestures

SPEECH_TIMEOUT = 30.0

# Bounds the LLM, not the listening; _ask holds the mouth throughout, so an
# unbounded wait here wedges every later ask. A warm generation is ~2s.
LLM_TIMEOUT = 10.0

# What may be offered or counter-offered. drinks.py is the only place to add one.
DRINKS = tuple(MENU)

# Classifier personas — one bare token, not the barista. Callers still validate.
DRINK_SYSTEM = (
    "You decide how a person replied to a drink offer. Answer with ONE of: "
    "YES if they accepted, NO if they declined and named nothing else, or the "
    "name of the drink they asked for instead, exactly as written here: "
    + ", ".join(DRINKS) + ". Output only that. No punctuation, no explanation."
)

WATER_SYSTEM = (
    "A person is filling a glass at a tap while a robot holds it. Decide from "
    "what they said whether they are finished. Answer with ONE word: DONE if "
    "they said they are finished, or WAIT for anything else — including "
    "background chatter that was not addressed to the robot. Output only that."
)

GREET_SYSTEM = ""   # "" = the llm node's barista persona

# Canned on purpose: identical every time reads as a ritual. Also fires when the
# estimator has nothing to offer, which on arrival is most of the time.
WELCOME = "Welcome back, {name}."

WATER_PROMPT = ("I am ready. Please fill as much water as you like, "
                "then tell me when you are done.")
WATER_GIVEUP = "No answer. I will put the glass back."

# Fourth _ask outcome: waved off mid-sentence. Not a label — no classifier ran.
PALM = "PALM"

# Which hands end an _ask. Palm always; the rest opt-in, because a thumb means
# "done" at the tap and "yes" to an offer.
GESTURE_ANSWERS = {gestures.PALM: PALM}
WATER_GESTURES = {gestures.PALM: "DONE", gestures.UP: "DONE"}


def _check_gestures():
    # Drift guard — both failures are silent at runtime.
    for table in (GESTURE_ANSWERS, WATER_GESTURES):
        for name in table:
            assert name in (gestures.UP, gestures.DOWN, gestures.PALM), \
                f"'{name}' is not a gesture the recognizer publishes"
    assert set(WATER_GESTURES.values()) <= {"DONE", "WAIT"}, \
        "water gestures must answer with a label _on_ask_for_water accepts"
    assert GESTURE_ANSWERS[gestures.PALM] == PALM, \
        "open palm is the barge-in — _execute tests for exactly this value"

_check_gestures()


def _match(answer, labels):
    # An LLM eventually replies "Coffee." — fold case and punctuation here, not
    # at every == downstream. "" for anything unrecognised: callers fall back.
    clean = answer.strip().strip('.!?,;:"\'').lower()
    return next((l for l in labels if l.lower() == clean), "")


class SuggestionHandlerNode(Node):

    def __init__(self):
        super().__init__("interaction_manager")

        cb = ReentrantCallbackGroup()

        self._tts_pub = self.create_publisher(String, "/tts_text", 10)
        self._tts_stop_pub = self.create_publisher(Empty, "/tts_stop", 10)

        self._speech_sub = self.create_subscription(
            String, "/speech_text", self._on_speech, 10, callback_group=cb
        )
        # Same topic the arm watches for thumbs during its mime; the mouth lock
        # keeps the two from stealing each other's hands.
        self.create_subscription(
            String, gestures.TOPIC, self._on_gesture, 10, callback_group=cb
        )

        # One client, two jobs: classifying replies and writing the greeting.
        self._llm_client = self.create_client(
            AskLLM, "/ask_llm", callback_group=cb
        )

        self.declare_parameter("user_name", "David")
        self._user_name = (
            self.get_parameter("user_name").get_parameter_value().string_value
        )

        # Greeting and offering are separate events. arm_controller listens to the
        # same topic and turns to the entrance; neither waits for the other.
        self.create_subscription(
            Empty, "/user_arrived", self._on_arrival, 10, callback_group=cb
        )

        self._bring_client = ActionClient(
            self, BringDrink, "bring_drink", callback_group=cb
        )

        self._action_server = ActionServer(
            self, SuggestDrink, "suggest_drink", self._execute,
            goal_callback=self._on_goal, callback_group=cb
        )

        # The arm calls this from under the tap while _execute is parked in its
        # BringDrink wait — hence ReentrantCallbackGroup on both.
        self.create_service(
            AskForWater, "/ask_for_water", self._on_ask_for_water, callback_group=cb
        )

        self._mouth = threading.Lock()   # one mouth, one ear, one user
        self._busy = False
        self._speech_event = threading.Event()
        self._speech_text = None
        self._gestures = {}      # what a hand means to the ask that is running
        self._gesture_answer = None

        self.get_logger().info("Interaction manager ready")

    def _on_goal(self, goal_request):
        # One conversation at a time. Rejecting beats reporting "user declined"
        # for a question we never asked.
        if self._busy:
            self.get_logger().warn("[suggest_drink] busy — rejecting goal")
            return GoalResponse.REJECT
        self._busy = True
        return GoalResponse.ACCEPT

    def _on_speech(self, msg):
        # The lock IS the "we are listening" flag — no separate state to sync.
        if self._mouth.locked():
            self.get_logger().info(f"[SPEECH] Received: '{msg.data}'")
            self._speech_text = msg.data
            self._speech_event.set()
        else:
            self.get_logger().debug(f"[SPEECH] Ignored (not asking): '{msg.data}'")

    def _on_gesture(self, msg):
        # Same gate: without it a thumbs-up meant for the arm's mime lands here.
        answer = self._gestures.get(msg.data) if self._mouth.locked() else None
        if not answer:
            return
        self.get_logger().info(f"[GESTURE] {msg.data} -> {answer}")
        self._tts_stop_pub.publish(Empty())   # they have heard enough
        self._gesture_answer = answer
        self._speech_event.set()

    # ---- the dialog leaf: the ONE place the robot talks and listens ----

    def _llm(self, prompt, system):
        # Blocking; "" on every failure — unavailable, slow, or nothing to say.
        if not self._llm_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("[LLM] /ask_llm unavailable — is llm up?")
            return ""

        request = AskLLM.Request()
        request.prompt = prompt
        request.system = system
        future = self._llm_client.call_async(request)
        deadline = time.monotonic() + LLM_TIMEOUT
        while not future.done():
            if time.monotonic() > deadline:
                future.cancel()
                self.get_logger().error(f"[LLM] no answer within {LLM_TIMEOUT}s")
                return ""
            time.sleep(0.05)
        return future.result().response

    def _ask(self, text, timeout, system, labels, gesture_answers=GESTURE_ANSWERS):
        # Returns one of `labels`, or whatever `gesture_answers` maps a hand to.
        # "" = nobody answered or the LLM went off-menu; both are the caller's
        # fallback, and a silent user is a different outcome from a refusing one.
        if not self._mouth.acquire(blocking=False):
            self.get_logger().warn(f"[ASK] already talking — dropped: '{text}'")
            return ""
        try:
            self._speech_event.clear()
            self._speech_text = None
            # Set under the lock, so a hand raised during the previous ask
            # cannot answer this one.
            self._gesture_answer = None
            self._gestures = gesture_answers

            # "" = listen without speaking, so a re-listen does not read the
            # same sentence at someone who is already pouring.
            self.get_logger().info(f"[ASK] TTS: '{text}' (up to {timeout}s)")
            if text:
                self._tts_pub.publish(String(data=text))

            if not self._speech_event.wait(timeout=timeout):
                self.get_logger().warn("[ASK] timed out — no response")
                return ""

            if self._gesture_answer:
                # A hand, not a sentence: nothing for the classifier to read.
                return self._gesture_answer

            self.get_logger().info(f"[ASK] classifying: '{self._speech_text}'")
            # The question is half the meaning — "no, a coffee instead" only
            # parses with the offer in context. Which is why llm stays stateless.
            answer = self._llm(
                f'The robot asked: "{text}"\n'
                f'The person said: "{self._speech_text}"',
                system,
            )
            label = _match(answer, labels)
            if answer and not label:
                self.get_logger().warn(f"[ASK] off-menu answer: '{answer}'")
            self.get_logger().info(f"[ASK] -> {label or '(none)'}")
            return label
        finally:
            self._mouth.release()

    # ---- orchestration ----

    def _greet(self, drink, reason):
        # "" on failure and the caller falls back to the canned question.
        # No reason, no call: asked to greet without a state the model invents
        # one ("I've noticed that you are tired") — a fabricated sensor reading.
        if not reason:
            return ""
        line = self._llm(
            "\n".join((
                f"name: {self._user_name}",
                f"drink to offer: {drink}",
                f"what you noticed about them: {reason}",
                "Write the greeting.",
            )),
            GREET_SYSTEM,
        )
        if not line:
            self.get_logger().warn("[GREET] no line from LLM — using canned")
        return line

    def _on_arrival(self, _msg):
        # Straight to the mouth: nothing is being asked, so there is no answer to
        # wait for. Mid-conversation we skip — trigger follows with an offer.
        if self._mouth.locked():
            self.get_logger().info("[ARRIVAL] mid-conversation — no welcome")
            return
        self.get_logger().info("[ARRIVAL] welcoming")
        self._tts_pub.publish(String(data=WELCOME.format(name=self._user_name)))

    def _on_ask_for_water(self, request, response):
        # WAIT costs a round, not the goal: _ask ends at the first thing it hears,
        # and at a running tap that is usually background chatter. "" still gives
        # up — mouth busy or classifier down, re-asking would spin.
        # Any hand means DONE: someone mid-pour won't free one to interrupt.
        deadline = time.monotonic() + request.timeout
        prompt = WATER_PROMPT
        response.confirmed = False
        while (remaining := deadline - time.monotonic()) > 0:
            answer = self._ask(prompt, remaining, WATER_SYSTEM,
                               ("DONE", "WAIT"), WATER_GESTURES)
            if answer != "WAIT":
                response.confirmed = answer == "DONE"
                break
            self.get_logger().info("[WATER] not done yet — still listening")
            prompt = ""   # already said once; the arm's wiggle carries the rest
        if not response.confirmed:
            self._tts_pub.publish(String(data=WATER_GIVEUP))
        return response

    def _execute(self, goal_handle):
        drink = goal_handle.request.drink
        reason = goal_handle.request.reason
        self.get_logger().info(f"[GOAL] Received suggestion for: '{drink}' ({reason or '-'})")
        try:
            goal_handle.publish_feedback(SuggestDrink.Feedback(status="asking_user"))
            question = self._greet(drink, reason) or f"Would you like a {drink}?"
            answer = self._ask(question, SPEECH_TIMEOUT, DRINK_SYSTEM,
                               ("YES", "NO") + DRINKS)

            if not answer:
                self.get_logger().warn("[GOAL] no response — aborting")
                goal_handle.abort()
                return SuggestDrink.Result(accepted=False)

            if answer == PALM:
                # They declined being talked to, not the drink — so the arm mimes.
                self.get_logger().info("[GOAL] open palm — arm will mime the menu")
            elif answer in DRINKS:
                # Naming a drink is an acceptance (the classifier answers "water"
                # as readily as "YES"); a different one is a counter-offer.
                if answer != drink:
                    self.get_logger().info(f"[GOAL] switched '{drink}' -> '{answer}'")
                    drink = answer
            elif answer != "YES":
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
                    BringDrink.Goal(drink=drink, offer_menu=(answer == PALM))
                )
                while not bring_future.done():
                    time.sleep(0.05)

                handle = bring_future.result()
                if not handle.accepted:
                    # get_result_async on a rejected handle raises, so this branch
                    # is required. Arm busy, or the drink is off-menu there.
                    self.get_logger().warn(f"[BRINGING] arm rejected '{drink}'")
                    goal_handle.succeed()
                    return SuggestDrink.Result(accepted=False)

                result_future = handle.get_result_async()
                while not result_future.done():
                    time.sleep(0.05)

                success = result_future.result().result.success
                self.get_logger().info(f"[BRINGING] BringDrink completed, success={success}")

            self.get_logger().info("[DONE] Goal succeeded, accepted=True")
            goal_handle.succeed()
            return SuggestDrink.Result(accepted=True)
        finally:
            self._busy = False  # never leave the manager wedged as busy


def demo(host="http://10.163.18.109:11434", model="llama3.2"):
    # The prompts are the fragile part, not the plumbing. Needs a reachable
    # Ollama, no ROS graph:  interaction_manager.py --selfcheck
    from brewbot.llm import generate

    assert _match(" Coffee. ", ("YES", "NO") + DRINKS) == "coffee"
    assert _match("I think yes", ("YES", "NO")) == ""   # off-menu, not a guess

    def ask(question, said, system, labels):
        return _match(generate(
            host, model,
            f'The robot asked: "{question}"\nThe person said: "{said}"',
            system, 30.0,
        ), labels)

    q = "Would you like a water?"
    drink_labels = ("YES", "NO") + DRINKS
    assert ask(q, "Yes please", DRINK_SYSTEM, drink_labels) == "YES"
    assert ask(q, "No thanks, I'm fine", DRINK_SYSTEM, drink_labels) == "NO"
    # the whole reason UC1 needs an LLM and not a yes/no classifier
    assert ask(q, "No, but I'd like a coffee instead",
               DRINK_SYSTEM, drink_labels) == "coffee"

    w = WATER_PROMPT
    water_labels = ("DONE", "WAIT")
    assert ask(w, "Okay, I'm done", WATER_SYSTEM, water_labels) == "DONE"
    assert ask(w, "hang on, not yet", WATER_SYSTEM, water_labels) == "WAIT"
    # overheard lab chatter must not stop the pour
    assert ask(w, "so then I told him the build was broken anyway",
               WATER_SYSTEM, water_labels) == "WAIT"

    # Wording moves between runs (temp 0.6), so only invariants are asserted.
    # Dana/tea share nothing with the exemplar in llm.SYSTEM_PROMPT
    # (Sam/thirsty/juice), so a copied word is a fabricated fact.
    for reason, drink in (("hot", "water"), ("stressed", "tea")):
        line = generate(host, model, "\n".join((
            "name: Dana",
            f"drink to offer: {drink}",
            f"what you noticed about them: {reason}",
            "Write the greeting.",
        )), GREET_SYSTEM, 30.0)
        low = line.lower()
        assert len(line.split()) <= 25, f"greeting too long: {line}"
        assert '"' not in line, f"greeting is quoted: {line}"
        assert "dana" in low and drink in low, f"greeting lost a fact: {line}"
        assert reason in low, f"greeting lost the state: {line}"
        assert "sam" not in low and "juice" not in low, \
            f"greeting leaked the example: {line}"
    print("prompt self-check passed")


def main():
    if "--selfcheck" in sys.argv:
        # Trailing args are host, then model.
        demo(*sys.argv[sys.argv.index("--selfcheck") + 1:])
        return
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
