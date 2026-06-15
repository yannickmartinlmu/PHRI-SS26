#!/usr/bin/env python3

import os
import rclpy
from rclpy.node import Node
from brewbot_interfaces.srv import ClassifyText

from sentence_transformers import SentenceTransformer
from sklearn.metrics.pairwise import cosine_similarity

_HERE = os.path.dirname(os.path.abspath(__file__))
_DEFAULT_MODEL = os.path.join(_HERE, "models", "all-mpnet-base-v2")


YES_EXAMPLES = [
    "yes", "yeah", "sure", "absolutely", "correct",
    "that's right", "of course", "certainly", "I think so", "definitely",
]

NO_EXAMPLES = [
    "no", "nope", "not at all", "absolutely not", "incorrect",
    "that's wrong", "I don't think so", "negative", "certainly not", "never",
]


class TextClassificationNode(Node):

    def __init__(self):
        super().__init__("nlp")

        print("Initializing...")
        self.declare_parameter(
            "model_path",
            _DEFAULT_MODEL,
        )
        model_path = self.get_parameter("model_path").get_parameter_value().string_value
        self._model = SentenceTransformer(model_path, local_files_only=True)

        self._yes_emb = self._model.encode(YES_EXAMPLES)
        self._no_emb = self._model.encode(NO_EXAMPLES)

        self._srv = self.create_service(
            ClassifyText,
            "classify_yes_no",
            self._handle,
        )

        self.get_logger().info("text_classification service ready")
        print("text_classification service ready")

    def _handle(self, request, response):
        response.result = self._classify(request.data)
        return response

    def _classify(self, text, threshold=0.15):
        emb = self._model.encode([text])
        yes_score = cosine_similarity(emb, self._yes_emb).mean()
        no_score = cosine_similarity(emb, self._no_emb).mean()
        
        print("Text:", text, ", Yes Score:", yes_score, ", No Score:", no_score)

        return "YES" if yes_score > no_score else "NO"


def main():
    rclpy.init()
    print(__file__)
    node = TextClassificationNode()
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
