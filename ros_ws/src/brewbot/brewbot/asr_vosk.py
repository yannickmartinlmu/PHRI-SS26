#!/usr/bin/env python3

import json
import os
import queue
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

import sounddevice as sd
from vosk import Model, KaldiRecognizer

_HERE = os.path.dirname(os.path.abspath(__file__))
_DEFAULT_MODEL = os.path.join(_HERE, "models", "vosk-model-small-en-us-0.15")

# Speakers and mic share a room in the lab, so the mic goes down while /tts_speaking
# is true. Barge-in is lost for the seconds the robot is actually talking; the rest
# of a 60s prompt is still live.
UNMUTE_DELAY = 0.2   # sec after TTS reports done — covers the ROS hop + speaker tail
MAX_MUTE = 30.0      # sec; a tts node that dies mid-sentence must not deafen us forever


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

        # None = mic live. Otherwise the monotonic time to unmute at — one field
        # gives both the trailing delay and the stuck-muted cap.
        self._unmute_at = None

        self.create_subscription(
            Bool, "/tts_speaking", self._on_tts_speaking, 10
        )

        self.timer = self.create_timer(
            0.05,
            self.process_audio
        )

    def audio_callback(self, indata, frames,
                       time_info, status):
        self.q.put(bytes(indata))

    def _set_mic(self, on):
        # ponytail: stops OUR capture stream, not the system device — no mixer, no
        # device names, same on the laptop and in WSL. Swap the body for
        # `pactl set-source-mute @DEFAULT_SOURCE@ 1/0` if something else records too.
        if on:
            self.stream.start()
        else:
            self.stream.stop()
        self.get_logger().info(f"[mic] {'live' if on else 'muted'}")

    def _on_tts_speaking(self, msg):
        if msg.data:
            self._unmute_at = time.monotonic() + MAX_MUTE
            self._set_mic(False)
        elif self._unmute_at is not None:
            self._unmute_at = time.monotonic() + UNMUTE_DELAY

    def process_audio(self):

        if self._unmute_at is not None:
            if time.monotonic() < self._unmute_at:
                return
            # Audio captured before the stop is still queued and vosk still holds a
            # partial utterance. Without dropping both, the first thing published
            # after unmute is the tail of the robot's own sentence — muted or not.
            while not self.q.empty():
                self.q.get_nowait()
            self.rec.Reset()
            self._unmute_at = None
            self._set_mic(True)

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
