#!/usr/bin/env python3

import queue
import tempfile

import numpy as np
import sounddevice as sd
import soundfile as sf

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from faster_whisper import WhisperModel


class WhisperNode(Node):

    def __init__(self):

        super().__init__("asr_whisper")

        self.pub = self.create_publisher(
            String,
            "/speech_text",
            10
        )

        print("Loading Whisper model...")
        self.model = WhisperModel(
            "small.en",
            device="cpu",
            compute_type="int8"
        )
        print("Whisper model loaded.")

        self.audio_q = queue.Queue()

        self.stream = sd.InputStream(
            samplerate=16000,
            channels=1,
            callback=self.audio_callback
        )

        self.stream.start()
        print("Microphone stream started.")

        self.timer = self.create_timer(
            3.0,
            self.transcribe_chunk
        )
        print("Whisper ASR node ready.")

    def audio_callback(
        self,
        indata,
        frames,
        time_info,
        status
    ):
        self.audio_q.put(indata.copy())

    def transcribe_chunk(self):

        chunks = []

        while not self.audio_q.empty():
            chunks.append(
                self.audio_q.get()
            )

        if not chunks:
            return

        audio = np.concatenate(
            chunks,
            axis=0
        )

        print(f"Transcribing {len(audio) / 16000:.1f}s of audio...")

        with tempfile.NamedTemporaryFile(
            suffix=".wav"
        ) as f:

            sf.write(
                f.name,
                audio,
                16000
            )

            segments, _ = self.model.transcribe(
                f.name,
                beam_size=1
            )

            text = " ".join(
                s.text for s in segments
            ).strip()

        if text:
            print(f"Detected: \"{text}\"")

            msg = String()
            msg.data = text

            self.pub.publish(msg)

            self.get_logger().info(
                f"ASR: {text}"
            )


def main():
    rclpy.init()
    node = WhisperNode()
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
