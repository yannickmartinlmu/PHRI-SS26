#!/usr/bin/env python3
"""LLM — the robot's talking brain, behind one service.

  ros2 service call /ask_llm brewbot_interfaces/srv/AskLLM "{prompt: 'hi'}"

Talks to Ollama on the lab PC. Every failure — unreachable, timed out, empty
generation — comes back as "" and is logged here; callers just fall back.
"""

import rclpy
from rclpy.node import Node
import json
import urllib.request


from brewbot_interfaces.srv import AskLLM


SYSTEM_PROMPT = (
    "You are BrewBot, a friendly robot barista in a university lab."
)


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

        self.get_logger().info(
            f"ask_llm ready (model={self._model} host={self._host})"
        )

    def _handle(self, request, response):
        if not request.prompt.strip():
            self.get_logger().warn("empty prompt")
            response.response = ""
            return response

        try:
            response.response = self._generate(request.prompt)
        except Exception as e:
            # Caller only needs "no answer"; the why stays in this log line.
            self.get_logger().error(f"generation failed: {e}")
            response.response = ""

        if not response.response:
            self.get_logger().warn("no usable answer — caller should fall back")
        return response

    def _generate(self, prompt):
        req = urllib.request.Request(
            f"{self._host}/api/generate",
            data=json.dumps({
                "model": self._model,
                "prompt": prompt,
                "system": SYSTEM_PROMPT,
                "stream": False,
            }).encode(),
            headers={"Content-Type": "application/json"},
        )
        # stream:False means Ollama sends one body at the end, so this read
        # blocks for the whole generation — timeout has to cover all of it.
        with urllib.request.urlopen(req, timeout=self._timeout) as r:
            return json.loads(r.read())["response"].strip()


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
