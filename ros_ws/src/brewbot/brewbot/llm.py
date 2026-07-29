#!/usr/bin/env python3
"""LLM — the robot's talking brain, behind one service.

  ros2 service call /ask_llm brewbot_interfaces/srv/AskLLM "{prompt: 'hi'}"

Leave `system` empty and you get the barista persona below. Fill it in and the
same service is a classifier instead — that is how interaction_manager reads
yes/no/counter-offer out of a sentence without a second model on this laptop.

Talks to Ollama on the lab PC. Every failure — unreachable, timed out, empty
generation — comes back as "" and is logged here; callers just fall back.
"""

import rclpy
from rclpy.node import Node
import json
import urllib.request


from brewbot_interfaces.srv import AskLLM


SYSTEM_PROMPT = (
    "You are BrewBot, a friendly robot barista in a university lab. "
    "Answer in one or two short spoken sentences. No lists, no markdown."
)

# Temperature 0: the same utterance must classify the same way every time, or
# debugging a bad interaction is guesswork. Costs nothing for the chat path —
# a reproducible barista is a feature during demos.
# keep_alive holds the model in the lab PC's memory, so only the very first
# request after an ollama restart pays the load; num_predict is the hard length
# cap that keeps TTS from reading an essay.
OPTIONS = {"temperature": 0, "num_predict": 60}
KEEP_ALIVE = "30m"


def generate(host, model, prompt, system, timeout):
    # Module level, not a method: the prompt wording is the fragile part of this
    # feature, and a self-check should be able to hit Ollama without a ROS graph.
    req = urllib.request.Request(
        f"{host}/api/generate",
        data=json.dumps({
            "model": model,
            "prompt": prompt,
            "system": system,
            "stream": False,
            "keep_alive": KEEP_ALIVE,
            "options": OPTIONS,
        }).encode(),
        headers={"Content-Type": "application/json"},
    )
    # stream:False means Ollama sends one body at the end, so this read
    # blocks for the whole generation — timeout has to cover all of it.
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return json.loads(r.read())["response"].strip()


class LLMNode(Node):

    def __init__(self):
        super().__init__("llm")

        self.declare_parameter("host", "http://10.163.18.109:11434")
        self.declare_parameter("model", "llama3.2")
        # Generous because the FIRST call also pays for loading the model into
        # memory. Warm calls are ~1-2s; drop this once the lab PC stays warm.
        self.declare_parameter("timeout", 30.0)

        self._host = self.get_parameter("host").get_parameter_value().string_value
        self._model = self.get_parameter("model").get_parameter_value().string_value
        self._timeout = self.get_parameter("timeout").get_parameter_value().double_value

        self._srv = self.create_service(AskLLM, "ask_llm", self._handle)

        # One throwaway generation to pull the model into memory now, so the
        # first real interaction is not the one that waits 20s for the load.
        self._warmup_timer = self.create_timer(0.1, self._warmup)

        self.get_logger().info(
            f"ask_llm ready (model={self._model} host={self._host})"
        )

    def _warmup(self):
        self._warmup_timer.cancel()
        try:
            self._generate("hi", SYSTEM_PROMPT)
            self.get_logger().info("model warm")
        except Exception as e:
            # Not fatal — the lab PC may come up after us. Calls just pay the
            # load themselves, which is what the generous timeout is for.
            self.get_logger().warn(f"warmup failed, model still cold: {e}")

    def _handle(self, request, response):
        if not request.prompt.strip():
            self.get_logger().warn("empty prompt")
            response.response = ""
            return response

        try:
            response.response = self._generate(
                request.prompt, request.system or SYSTEM_PROMPT
            )
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
