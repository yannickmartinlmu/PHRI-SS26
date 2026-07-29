#!/usr/bin/env python3
"""LLM — the robot's talking brain, behind one service.

  ros2 service call /ask_llm brewbot_interfaces/srv/AskLLM "{prompt: 'hi'}"

Shell only: _generate() is not wired to Ollama yet, it returns "". Everything
around it is real, so callers can be written against it today — an empty
response is exactly what they will see when Ollama is down anyway.
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

        self.declare_parameter("host", "http://localhost:11434")
        self.declare_parameter("model", "llama3.2")
        # Anything slower than this and the conversation has already died.
        self.declare_parameter("timeout", 10.0)

        self._host = self.get_parameter("host").get_parameter_value().string_value
        self._model = self.get_parameter("model").get_parameter_value().string_value
        self._timeout = self.get_parameter("timeout").get_parameter_value().double_value

        self._srv = self.create_service(AskLLM, "ask_llm", self._handle)

        self.get_logger().info(
            f"ask_llm ready (model={self._model} host={self._host}) — STUB, returns ''"
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
        """
        Request might look something like this:
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
        with urllib.request.urlopen(req, timeout=self._timeout) as r:
            return json.loads(r.read())["response"].strip()
        """
        return ""


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
