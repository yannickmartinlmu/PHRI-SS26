#!/usr/bin/env python3
"""LLM — the robot's talking brain, behind one service.

  ros2 service call /ask_llm brewbot_interfaces/srv/AskLLM "{prompt: 'hi'}"

Empty `system` = the barista persona below; filled in = a classifier instead.
Talks to Ollama on the lab PC. Every failure comes back as "" and is logged here.
"""

import rclpy
from rclpy.node import Node
import json
import urllib.request


from brewbot_interfaces.srv import AskLLM


# The example is the instruction: told to "be brief" a 3B model writes 30 words
# of ad copy, shown one sentence in the target shape it copies it and stops.
# Sam/thirsty/juice match no real case, so a leak is obvious in the logs.
SYSTEM_PROMPT = (
    "You are BrewBot, a robot barista in a university lab. "
    "Greet the person by name, say what you noticed about them, then offer "
    "the drink as a question. Exactly this shape:\n"
    "Hello Sam. I've noticed that you are thirsty. "
    "Would you like a cup of juice?\n"
    "At most 25 words. Use ONLY the facts given — never invent a state, "
    "a name or a drink. No quotation marks, no lists, no markdown, no emoji."
)

# Temperature 0: the same utterance must classify the same way every time.
# num_predict is the hard length cap that keeps TTS from reading an essay.
OPTIONS = {"temperature": 0, "num_predict": 60}

# The greeting is the one call that WANTS to vary — the same sentence three
# times is what makes a robot sound broken. Measured on llama3.2: 0.6 rewords,
# 0.9 starts narrating in the third person.
# Keyed off an empty `system` rather than a per-call options field, which would
# mean changing AskLLM.srv and rebuilding brewbot_interfaces.
GREET_OPTIONS = {"temperature": 0.6, "num_predict": 40}
KEEP_ALIVE = "30m"    # holds the model in the lab PC's memory between calls


def generate(host, model, prompt, system, timeout):
    # Module level, not a method: the prompt wording is the fragile part, and a
    # self-check should be able to hit Ollama without a ROS graph.
    req = urllib.request.Request(
        f"{host}/api/generate",
        data=json.dumps({
            "model": model,
            "prompt": prompt,
            # Resolved HERE, not at the caller — the options below read the
            # same emptiness.
            "system": system or SYSTEM_PROMPT,
            "stream": False,
            "keep_alive": KEEP_ALIVE,
            "options": OPTIONS if system else GREET_OPTIONS,
        }).encode(),
        headers={"Content-Type": "application/json"},
    )
    # stream:False means one body at the end, so this read blocks for the whole
    # generation — timeout has to cover all of it.
    with urllib.request.urlopen(req, timeout=timeout) as r:
        # Small models wrap spoken lines in quotes however firmly the prompt
        # says not to, and TTS reads them out.
        return json.loads(r.read())["response"].strip().strip('"').strip()


class LLMNode(Node):

    def __init__(self):
        super().__init__("llm")

        self.declare_parameter("host", "http://10.163.18.109:11434")
        self.declare_parameter("model", "llama3.2")
        # Generous because the FIRST call also pays for loading the model.
        # Warm calls are ~1-2s; drop this once the lab PC stays warm.
        self.declare_parameter("timeout", 30.0)

        self._host = self.get_parameter("host").get_parameter_value().string_value
        self._model = self.get_parameter("model").get_parameter_value().string_value
        self._timeout = self.get_parameter("timeout").get_parameter_value().double_value

        self._srv = self.create_service(AskLLM, "ask_llm", self._handle)

        # One throwaway generation, so the first real interaction is not the one
        # that waits 20s for the model to load.
        self._warmup_timer = self.create_timer(0.1, self._warmup)

        self.get_logger().info(
            f"ask_llm ready (model={self._model} host={self._host})"
        )

    def _warmup(self):
        self._warmup_timer.cancel()
        try:
            self._generate("hi", "")
            self.get_logger().info("model warm")
        except Exception as e:
            # Not fatal — the lab PC may come up after us, and calls then pay
            # the load themselves.
            self.get_logger().warn(f"warmup failed, model still cold: {e}")

    def _handle(self, request, response):
        if not request.prompt.strip():
            self.get_logger().warn("empty prompt")
            response.response = ""
            return response

        try:
            response.response = self._generate(request.prompt, request.system)
        except Exception as e:
            # Caller only needs "no answer"; the why stays in this log line.
            self.get_logger().error(f"generation failed: {e}")
            response.response = ""

        if not response.response:
            self.get_logger().warn("no usable answer — caller should fall back")
        return response

    def _generate(self, prompt, system):
        return generate(
            self._host, self._model, prompt, system, self._timeout
        )


def main():
    rclpy.init()
    node = LLMNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
