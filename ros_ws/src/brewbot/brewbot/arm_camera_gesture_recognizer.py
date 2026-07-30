#!/usr/bin/env python3
#Recognize thumbs-up / thumbs-down gestures from image stream

#The node uses MediaPipe's pretrained Gesture Recognizer and adds a small state machine on top:

#only Thumb_Up, Thumb_Down and Open_Palm are accepted as application gestures
#a gesture must be seen in several consecutive processed frames
#a confirmed gesture expires when it has not been observed recently
#state topics are transient-local, so ros2 topic echo --once receives the latest value immediately
#an event topic is published only when a new confirmed gesture starts.
#No robot motion is commanded by this node. It is a perception component only.

from __future__ import annotations

import os
import time
from pathlib import Path
from typing import Optional, Tuple

import cv2
import mediapipe as mp
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32, String


NONE = "none"
THUMBS_UP = "thumbs_up"
THUMBS_DOWN = "thumbs_down"
OPEN_PALM = "open_palm"

MEDIAPIPE_TO_ROS = {
    "Thumb_Up": THUMBS_UP,
    "Thumb_Down": THUMBS_DOWN,
    "Open_Palm": OPEN_PALM,
}


class ArmCameraGestureRecognizer(Node):
   

    def __init__(self) -> None:
        super().__init__("gesture_recognizer")

        models = os.path.join(get_package_share_directory("brewbot"), "models")
        default_model = os.path.join(models, "gesture_recognizer.task")
        default_person_model = os.path.join(models, "efficientdet_lite0.tflite")

        self.image_topic = str(
            self.declare_parameter("image_topic", "/camera/color/image_raw").value
        )
        self.output_namespace = str(
            self.declare_parameter(
                "output_namespace", "/brewbot/perception/arm_camera"
            ).value
        ).rstrip("/")
        self.model_path = str(
            self.declare_parameter("model_path", default_model).value
        )

        self.max_processing_rate_hz = float(
            self.declare_parameter("max_processing_rate_hz", 8.0).value
        )
        self.max_input_width = int(
            self.declare_parameter("max_input_width", 960).value
        )
        self.rotation_degrees = int(
            self.declare_parameter("rotation_degrees", 0).value
        )

        self.minimum_gesture_confidence = float(
            self.declare_parameter("minimum_gesture_confidence", 0.65).value
        )
        self.minimum_hand_detection_confidence = float(
            self.declare_parameter("minimum_hand_detection_confidence", 0.5).value
        )
        self.minimum_hand_presence_confidence = float(
            self.declare_parameter("minimum_hand_presence_confidence", 0.5).value
        )
        self.minimum_tracking_confidence = float(
            self.declare_parameter("minimum_tracking_confidence", 0.5).value
        )
        self.required_consecutive_frames = int(
            self.declare_parameter("required_consecutive_frames", 3).value
        )
        self.lost_timeout_sec = float(
            self.declare_parameter("lost_timeout_sec", 0.7).value
        )
        self.event_cooldown_sec = float(
            self.declare_parameter("event_cooldown_sec", 1.0).value
        )
        self.publish_rate_hz = float(
            self.declare_parameter("publish_rate_hz", 10.0).value
        )

        # Presence, not gestures: is a person in front of THIS camera at all, so the
        # arm knows which way to turn to ask a question. Off by default, because it
        # only means anything on a camera that does not move — what the wrist camera
        # sees is a fact about the arm, not about where the user is standing.
        self.detect_person = bool(
            self.declare_parameter("detect_person", False).value
        )
        self.person_model_path = str(
            self.declare_parameter("person_model_path", default_person_model).value
        )
        self.minimum_person_confidence = float(
            self.declare_parameter("minimum_person_confidence", 0.4).value
        )
        # Deliberately far below max_processing_rate_hz: which side of the room
        # someone stands on does not change ten times a second, and this is what
        # keeps a second model off the CPU budget.
        self.person_rate_hz = float(
            self.declare_parameter("person_rate_hz", 2.0).value
        )
        # Much longer than lost_timeout_sec: a hand leaves the frame between
        # gestures, but someone who turns to face the sink has not left the room.
        self.person_lost_timeout_sec = float(
            self.declare_parameter("person_lost_timeout_sec", 5.0).value
        )

        self._validate_parameters()

        if not Path(self.model_path).is_file():
            raise RuntimeError(
                "MediaPipe gesture model not found at "
                f"'{self.model_path}'. Run scripts/download_gesture_model.py "
                "and rebuild brewbot, or pass model_path:=/absolute/path/model.task."
            )

        self._bridge = CvBridge()
        self._last_processed_monotonic = 0.0
        self._last_mediapipe_timestamp_ms = -1

        # Raw candidate / temporal confirmation state.
        self._candidate = NONE
        self._candidate_count = 0
        self._stable_gesture = NONE
        self._stable_confidence = 0.0
        self._stable_handedness = "unknown"
        self._last_valid_gesture_monotonic = 0.0
        self._last_hand_seen_monotonic = 0.0
        self._last_event_monotonic = -float("inf")
        self._last_person_seen_monotonic = 0.0
        self._last_person_check_monotonic = 0.0

        BaseOptions = mp.tasks.BaseOptions
        GestureRecognizer = mp.tasks.vision.GestureRecognizer
        GestureRecognizerOptions = mp.tasks.vision.GestureRecognizerOptions
        RunningMode = mp.tasks.vision.RunningMode

        options = GestureRecognizerOptions(
            base_options=BaseOptions(model_asset_path=self.model_path),
            running_mode=RunningMode.VIDEO,
            num_hands=1,
            min_hand_detection_confidence=self.minimum_hand_detection_confidence,
            min_hand_presence_confidence=self.minimum_hand_presence_confidence,
            min_tracking_confidence=self.minimum_tracking_confidence,
        )
        self._recognizer = GestureRecognizer.create_from_options(options)

        self._person_detector = None
        if self.detect_person:
            if not Path(self.person_model_path).is_file():
                raise RuntimeError(
                    "MediaPipe person model not found at "
                    f"'{self.person_model_path}'. Run "
                    "scripts/download_gesture_model.py and rebuild brewbot."
                )
            ObjectDetector = mp.tasks.vision.ObjectDetector
            ObjectDetectorOptions = mp.tasks.vision.ObjectDetectorOptions
            self._person_detector = ObjectDetector.create_from_options(
                ObjectDetectorOptions(
                    base_options=BaseOptions(model_asset_path=self.person_model_path),
                    # IMAGE, not VIDEO: at 2 Hz there is nothing to track between
                    # frames, and it saves the timestamp bookkeeping.
                    running_mode=RunningMode.IMAGE,
                    # The model knows 80 COCO classes. We want one bit.
                    category_allowlist=["person"],
                    score_threshold=self.minimum_person_confidence,
                    max_results=1,
                )
            )

        # State publishers are transient-local, so late subscribers immediately
        # receive the latest known state.
        state_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        # For live image processing, only the newest available frame matters.
        # A queue depth of 1 prevents old frames from being processed later.
        image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        # Gesture events are short-lived events and are not latched.
        event_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self._gesture_pub = self.create_publisher(
            String, f"{self.output_namespace}/gesture", state_qos
        )
        self._confidence_pub = self.create_publisher(
            Float32, f"{self.output_namespace}/gesture_confidence", state_qos
        )
        self._handedness_pub = self.create_publisher(
            String, f"{self.output_namespace}/gesture_handedness", state_qos
        )
        self._hand_present_pub = self.create_publisher(
            Bool, f"{self.output_namespace}/hand_present", state_qos
        )
        self._thumbs_up_pub = self.create_publisher(
            Bool, f"{self.output_namespace}/thumbs_up", state_qos
        )
        self._thumbs_down_pub = self.create_publisher(
            Bool, f"{self.output_namespace}/thumbs_down", state_qos
        )
        self._open_palm_pub = self.create_publisher(
            Bool, f"{self.output_namespace}/open_palm", state_qos
        )
        self._event_pub = self.create_publisher(
            String, f"{self.output_namespace}/gesture_event", event_qos
        )

        # Relative name on purpose — it resolves into the node's OWN namespace
        # (/kitchen_camera/person_present, /usb_camera/person_present). The gesture
        # topics above are deliberately merged into one output_namespace, so this is
        # what keeps the fixed cameras distinguishable without a second parameter.
        self._person_pub = (
            self.create_publisher(Bool, "person_present", state_qos)
            if self._person_detector is not None
            else None
        )

        self._image_subscription = self.create_subscription(
            Image,
            self.image_topic,
            self._on_image,
            image_qos,
        )
        self._publish_timer = self.create_timer(
            1.0 / self.publish_rate_hz,
            self._publish_state,
        )

        self.get_logger().info(
            "Arm-camera gesture recognizer ready: "
            f"image={self.image_topic}, output={self.output_namespace}, "
            f"rate<={self.max_processing_rate_hz:.1f} Hz, "
            f"confirm={self.required_consecutive_frames} frames, "
            f"person_present={'on' if self.detect_person else 'off'}"
        )

    def _validate_parameters(self) -> None:
        if self.max_processing_rate_hz <= 0.0:
            raise ValueError("max_processing_rate_hz must be > 0")
        if self.max_input_width < 0:
            raise ValueError("max_input_width must be >= 0")
        if self.rotation_degrees not in (0, 90, 180, 270):
            raise ValueError("rotation_degrees must be one of 0, 90, 180, 270")
        if not 0.0 <= self.minimum_gesture_confidence <= 1.0:
            raise ValueError("minimum_gesture_confidence must be in [0, 1]")
        for name, value in (
            ("minimum_hand_detection_confidence", self.minimum_hand_detection_confidence),
            ("minimum_hand_presence_confidence", self.minimum_hand_presence_confidence),
            ("minimum_tracking_confidence", self.minimum_tracking_confidence),
        ):
            if not 0.0 <= value <= 1.0:
                raise ValueError(f"{name} must be in [0, 1]")
        if self.required_consecutive_frames < 1:
            raise ValueError("required_consecutive_frames must be >= 1")
        if self.lost_timeout_sec <= 0.0:
            raise ValueError("lost_timeout_sec must be > 0")
        if self.event_cooldown_sec < 0.0:
            raise ValueError("event_cooldown_sec must be >= 0")
        if self.publish_rate_hz <= 0.0:
            raise ValueError("publish_rate_hz must be > 0")
        if not 0.0 <= self.minimum_person_confidence <= 1.0:
            raise ValueError("minimum_person_confidence must be in [0, 1]")
        if self.person_rate_hz <= 0.0:
            raise ValueError("person_rate_hz must be > 0")
        if self.person_lost_timeout_sec <= 0.0:
            raise ValueError("person_lost_timeout_sec must be > 0")

    def _on_image(self, msg: Image) -> None:
        now = time.monotonic()
        minimum_period = 1.0 / self.max_processing_rate_hz
        if now - self._last_processed_monotonic < minimum_period:
            return
        self._last_processed_monotonic = now

        try:
            # MediaPipe expects SRGB. CvBridge handles common camera encodings
            # such as rgb8, bgr8 and bgra8 here.
            rgb = self._bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
        except CvBridgeError as exc:
            self.get_logger().error(f"cv_bridge conversion failed: {exc}")
            return

        rgb = self._rotate_and_resize(np.asarray(rgb))
        rgb = np.ascontiguousarray(rgb)

        timestamp_ms = time.monotonic_ns() // 1_000_000
        if timestamp_ms <= self._last_mediapipe_timestamp_ms:
            timestamp_ms = self._last_mediapipe_timestamp_ms + 1
        self._last_mediapipe_timestamp_ms = timestamp_ms

        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)

        # Before the gesture call, which returns early on failure. The two answer
        # different questions and neither should silence the other.
        if self._person_detector is not None:
            self._detect_person(mp_image, now)

        try:
            result = self._recognizer.recognize_for_video(mp_image, timestamp_ms)
        except Exception as exc:  # MediaPipe raises several native exception types.
            self.get_logger().error(f"MediaPipe recognition failed: {exc}")
            return

        gesture, confidence, handedness, hand_present = self._read_result(result)
        self._update_stability(
            gesture=gesture,
            confidence=confidence,
            handedness=handedness,
            hand_present=hand_present,
            now=now,
        )

    def _detect_person(self, mp_image, now: float) -> None:
        if now - self._last_person_check_monotonic < 1.0 / self.person_rate_hz:
            return
        self._last_person_check_monotonic = now
        try:
            result = self._person_detector.detect(mp_image)
        except Exception as exc:  # MediaPipe raises several native exception types.
            self.get_logger().error(f"MediaPipe person detection failed: {exc}")
            return
        # Only "seen" is recorded; "gone" is the timeout in _publish_state, so a
        # single missed frame cannot make the arm swing round to the other camera.
        if result.detections:
            self._last_person_seen_monotonic = now

    def _rotate_and_resize(self, rgb: np.ndarray) -> np.ndarray:
        if self.rotation_degrees == 90:
            rgb = cv2.rotate(rgb, cv2.ROTATE_90_CLOCKWISE)
        elif self.rotation_degrees == 180:
            rgb = cv2.rotate(rgb, cv2.ROTATE_180)
        elif self.rotation_degrees == 270:
            rgb = cv2.rotate(rgb, cv2.ROTATE_90_COUNTERCLOCKWISE)

        height, width = rgb.shape[:2]
        if self.max_input_width > 0 and width > self.max_input_width:
            scale = self.max_input_width / float(width)
            rgb = cv2.resize(
                rgb,
                (self.max_input_width, max(1, int(round(height * scale)))),
                interpolation=cv2.INTER_AREA,
            )
        return rgb

    def _read_result(self, result) -> Tuple[str, float, str, bool]:
        hand_present = bool(result.hand_landmarks)
        if not hand_present:
            return NONE, 0.0, "unknown", False

        # The canned classifiers top result can also be Open_Palm, Victory,
        # None, etc. Only the two application gestures are accepted here.
        best_gesture = NONE
        best_confidence = 0.0
        best_handedness = "unknown"

        for hand_index, categories in enumerate(result.gestures):
            if not categories:
                continue
            top = categories[0]
            ros_name = MEDIAPIPE_TO_ROS.get(top.category_name, NONE)
            score = float(top.score)
            if ros_name == NONE or score < self.minimum_gesture_confidence:
                continue
            if score > best_confidence:
                best_gesture = ros_name
                best_confidence = score
                if hand_index < len(result.handedness) and result.handedness[hand_index]:
                    best_handedness = str(
                        result.handedness[hand_index][0].category_name
                    ).lower()

        return best_gesture, best_confidence, best_handedness, True

    def _update_stability(
        self,
        *,
        gesture: str,
        confidence: float,
        handedness: str,
        hand_present: bool,
        now: float,
    ) -> None:
        if hand_present:
            self._last_hand_seen_monotonic = now

        if gesture == self._candidate:
            self._candidate_count += 1
        else:
            self._candidate = gesture
            self._candidate_count = 1

        if gesture not in (THUMBS_UP, THUMBS_DOWN, OPEN_PALM):
            return
        if self._candidate_count < self.required_consecutive_frames:
            return

        previous = self._stable_gesture
        self._stable_gesture = gesture
        self._stable_confidence = confidence
        self._stable_handedness = handedness
        self._last_valid_gesture_monotonic = now

        if previous != gesture:
            self.get_logger().info(
                f"Gesture confirmed: {gesture} "
                f"({confidence:.2f}, {handedness})"
            )
            if now - self._last_event_monotonic >= self.event_cooldown_sec:
                self._event_pub.publish(String(data=gesture))
                self._last_event_monotonic = now

    def _publish_state(self) -> None:
        now = time.monotonic()
        if (
            self._stable_gesture != NONE
            and now - self._last_valid_gesture_monotonic > self.lost_timeout_sec
        ):
            self.get_logger().info(f"Gesture expired: {self._stable_gesture}")
            self._stable_gesture = NONE
            self._stable_confidence = 0.0
            self._stable_handedness = "unknown"
            self._candidate = NONE
            self._candidate_count = 0

        hand_present = (
            self._last_hand_seen_monotonic > 0.0
            and now - self._last_hand_seen_monotonic <= self.lost_timeout_sec
        )

        self._gesture_pub.publish(String(data=self._stable_gesture))
        self._confidence_pub.publish(Float32(data=float(self._stable_confidence)))
        self._handedness_pub.publish(String(data=self._stable_handedness))
        self._hand_present_pub.publish(Bool(data=hand_present))
        self._thumbs_up_pub.publish(Bool(data=self._stable_gesture == THUMBS_UP))
        self._thumbs_down_pub.publish(Bool(data=self._stable_gesture == THUMBS_DOWN))
        self._open_palm_pub.publish(Bool(data=self._stable_gesture == OPEN_PALM))

        if self._person_pub is not None:
            self._person_pub.publish(Bool(data=(
                self._last_person_seen_monotonic > 0.0
                and now - self._last_person_seen_monotonic
                <= self.person_lost_timeout_sec
            )))

    def destroy_node(self) -> bool:
        try:
            self._recognizer.close()
            if self._person_detector is not None:
                self._person_detector.close()
        finally:
            return super().destroy_node()


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node: Optional[ArmCameraGestureRecognizer] = None
    try:
        node = ArmCameraGestureRecognizer()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        if node is not None:
            node.get_logger().fatal(str(exc))
        else:
            print(f"Failed to start gesture recognizer: {exc}")
        raise
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
