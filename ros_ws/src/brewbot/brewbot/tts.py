#!/usr/bin/env python3

import os
import queue
import subprocess
import tempfile
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Empty, String

# --- espeak-ng backend ---
# Install: sudo apt install espeak-ng

def speak_espeak(text: str, voice: str = ""):
    subprocess.run(["espeak-ng", text], check=True, capture_output=True)  # one voice only


# --- Piper backend ---
# Install: pip install piper-tts
# Models:  https://github.com/rhasspy/piper/blob/master/VOICES.md
# Download a model .onnx + .onnx.json, set PIPER_MODEL_PATH below

# The voice travels with the text ("hal: I'm sorry Dave"), not as a separate
# /tts_voice topic: the speak queue is async, so a set-voice message would race
# with lines already queued and hand the wrong voice the wrong sentence.
VOICES = {
    "": os.path.expanduser("~/piper/en_GB-alan-medium.onnx"),
    "hal": os.path.expanduser("~/piper/hal.onnx"),
}

_piper_voices = {}   # name -> loaded PiperVoice; loaded on first use, kept

_playing = None   # the aplay child, so /tts_stop can cut a sentence short


def stop():
    # Best-effort: the child may already have exited, which is the same outcome.
    # Only playback is interruptible, not synthesis — a palm during piper's few
    # hundred ms of generation lets that one sentence through. Not worth a second
    # kill path; move synthesis into the child if it ever becomes audible.
    if _playing is not None and _playing.poll() is None:
        _playing.terminate()


def speak_piper(text: str, voice: str = ""):
    global _playing
    if voice not in _piper_voices:
        from piper import PiperVoice
        _piper_voices[voice] = PiperVoice.load(VOICES[voice])
    _piper_voice = _piper_voices[voice]

    import wave
    import numpy as np

    with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as f:
        tmp_path = f.name

    try:
        with wave.open(tmp_path, "wb") as wav_file:
            wav_file.setnchannels(1)
            wav_file.setsampwidth(2)
            wav_file.setframerate(_piper_voice.config.sample_rate)
            for chunk in _piper_voice.synthesize(text):
                audio_int16 = (chunk.audio_float_array * 32767).astype(np.int16)
                wav_file.writeframes(audio_int16.tobytes())
        # Popen, not run(): stop() needs a handle on the child.
        _playing = subprocess.Popen(["aplay", tmp_path], stderr=subprocess.PIPE, text=True)
        _, err = _playing.communicate()
        if _playing.returncode < 0:
            return   # negative rc = killed by a signal, i.e. stop(). Not a failure.
        if _playing.returncode != 0:
            raise RuntimeError(f"aplay failed (rc={_playing.returncode}): {err.strip()}")
    finally:
        os.unlink(tmp_path)


# --- Active backend ---
# Swap speak_espeak for speak_piper once Piper is installed and model is downloaded
_speak = speak_piper


class TtsNode(Node):

    def __init__(self):
        super().__init__("tts")

        self._queue = queue.Queue()

        # Speakers and mic share a room: asr_vosk mutes on this or it transcribes
        # the robot answering its own question. Created BEFORE the worker starts —
        # the worker publishes on its very first item.
        self._speaking_pub = self.create_publisher(Bool, "/tts_speaking", 10)

        self._worker = threading.Thread(
            target=self._speak_worker, daemon=True
        )
        self._worker.start()

        self._sub = self.create_subscription(
            String, "/tts_text", self._on_text, 10
        )

        # Barge-in. Deliberately unconditional: every line is interruptible, so
        # nothing has to carry a per-message "may be cut off" flag. The only
        # publisher is interaction_manager's open-palm handler.
        self.create_subscription(Empty, "/tts_stop", self._on_stop, 10)

        self.get_logger().info("TTS node ready")

    def _on_text(self, msg):
        self.get_logger().info(f"[TTS] Queued: '{msg.data}'")
        self._queue.put(msg.data)

    def _on_stop(self, _):
        # Drain BEFORE killing: the worker only unmutes the mic once the queue is
        # empty, so a queued follow-up sentence would both start playing anyway
        # and hold the mic muted through it.
        dropped = 0
        while not self._queue.empty():
            self._queue.get_nowait()
            dropped += 1
        stop()
        self.get_logger().info(f"[TTS] stopped ({dropped} queued dropped)")

    def _speak_worker(self):
        while True:
            text = self._queue.get()
            # "hal: line" picks a voice; a bare colon anywhere else is left alone
            # because the prefix has to match a known voice name.
            name, _, rest = text.partition(":")
            voice, text = (name, rest.strip()) if name in VOICES and name else ("", text)
            # True before synthesis, not just before playback: muting early is free,
            # unmuting early is the whole bug.
            self._speaking_pub.publish(Bool(data=True))
            try:
                _speak(text, voice)
            except FileNotFoundError as e:
                self.get_logger().error(f"TTS binary not found: {e}")
            except Exception as e:
                self.get_logger().error(f"TTS failed: {e}")
            finally:
                # Only once the queue drains — per-message would unmute the mic in
                # the gap between two back-to-back sentences.
                if self._queue.empty():
                    self._speaking_pub.publish(Bool(data=False))


def main():
    rclpy.init()
    node = TtsNode()
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
