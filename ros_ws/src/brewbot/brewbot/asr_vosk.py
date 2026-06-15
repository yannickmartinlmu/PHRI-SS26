#!/usr/bin/env python3

import json
import os
import queue

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import sounddevice as sd
from vosk import Model, KaldiRecognizer

_HERE = os.path.dirname(os.path.abspath(__file__))
_DEFAULT_MODEL = os.path.join(_HERE, "models", "vosk-model-small-en-us-0.15")


class VoskNode(Node):

    def __init__(self):
        super().__init__("asr_vosk")

        self.pub = self.create_publisher(
            String,
            "/speech_text",
            10
        )

        self.q = queue.Queue()

        self.model = Model(_DEFAULT_MODEL)

        self.rec = KaldiRecognizer(
            self.model,
            16000
        )

        self.stream = sd.RawInputStream(
            samplerate=16000,
            blocksize=8000,
            dtype="int16",
            channels=1,
            callback=self.audio_callback
        )

        self.stream.start()

        self.timer = self.create_timer(
            0.05,
            self.process_audio
        )

    def audio_callback(self, indata, frames,
                       time_info, status):
        self.q.put(bytes(indata))

    def process_audio(self):

        while not self.q.empty():

            data = self.q.get()

            if self.rec.AcceptWaveform(data):

                result = json.loads(
                    self.rec.Result()
                )

                text = result.get("text", "")

                if text:

                    msg = String()
                    msg.data = text

                    self.pub.publish(msg)

                    self.get_logger().info(
                        f"ASR: {text}"
                    )


def main():
    rclpy.init()
    node = VoskNode()
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
