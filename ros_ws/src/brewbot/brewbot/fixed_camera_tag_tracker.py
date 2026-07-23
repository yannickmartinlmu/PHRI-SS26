#!/usr/bin/env python3

#AprilTag Tracker for the fixed cam in the kitchen
#The actual processing is performed by apriltag_ros. This node only adds:
#a semantic name for every configured tag,
#a small stability gate (several consecutive detections, mininmum visible time to be detected),
# everything configurable in YAML file 
from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List

import rclpy
from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from std_msgs.msg import Bool, Int32MultiArray
from tf2_ros import Buffer, TransformException, TransformListener


@dataclass
class TagState:
    consecutive_detections: int = 0 
    last_seen_ns: int = 0
    last_decision_margin: float = 0.0
    visible: bool = False
    source_frame: str = "" #frame_id des Kameraframes von dem die Detection kommt eher umbenennen zu detection_frame oder camera_frameweil unten nochmal name verwendet
    #zu camera_frame

class FixedCameraTagTracker(Node):

    def __init__(self) -> None:
        super().__init__("tag_tracker")

        #Declare Configurable Parameters
        self.declare_parameter("detections_topic", "/fixed_camera/detections") #Topic name on which we expect the apriltag_ros detection
        self.declare_parameter("output_namespace", "/brewbot/perception/fixed_camera")
        self.declare_parameter("target_frame", "") #In which coordinate system we want to publish the tag pose e.g base_link
        self.declare_parameter("tag_ids", [0]) #TagIds to be taken into account (ignores all others)
        self.declare_parameter("tag_names", ["reference"]) #Human readable names for tags
        self.declare_parameter("tag_frames", ["kitchen_reference_tag"]) #TF names that apriltag_ros published for the tags
        self.declare_parameter("minimum_decision_margin", 15.0)#minimum detection quality (accepting)
        self.declare_parameter("minimum_consecutive_detections", 3)#min number of detections to actually count as detected
        self.declare_parameter("lost_timeout_sec", 0.6) #how long a tag still counts as detected even though its not there anymore (to avoid "flickering")
        self.declare_parameter("maximum_detection_gap_sec", 0.25) #max time between detections to still count as consecutive
        self.declare_parameter("publish_rate_hz", 10.0)

        #Get Parameters out of the YAML
        detections_topic = str(self.get_parameter("detections_topic").value)
        self.output_namespace = str(self.get_parameter("output_namespace").value).rstrip("/")
        self.target_frame = str(self.get_parameter("target_frame").value).strip()
        tag_ids = [int(value) for value in self.get_parameter("tag_ids").value]
        tag_names = [str(value) for value in self.get_parameter("tag_names").value]
        tag_frames = [str(value) for value in self.get_parameter("tag_frames").value]

        if not (len(tag_ids) == len(tag_names) == len(tag_frames)):
            raise ValueError("tag_ids, tag_names and tag_frames must have equal lengths")
        if len(set(tag_ids)) != len(tag_ids):
            raise ValueError("tag_ids must be unieque")
        if len(set(tag_names)) != len(tag_names):
            raise ValueError("tag_names must be unieque")

        self.minimum_decision_margin = float(self.get_parameter("minimum_decision_margin").value)
        self.minimum_consecutive_detections = int(self.get_parameter("minimum_consecutive_detections").value)
        self.lost_timeout_ns = int(float(self.get_parameter("lost_timeout_sec").value) * 1e9)
        self.maximum_detection_gap_ns = int(float(self.get_parameter("maximum_detection_gap_sec").value) * 1e9)
        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        
        if publish_rate_hz <= 0.0:
            raise ValueError("publish_rate_hz must be > 0")

        self.tag_names: Dict[int, str] = dict(zip(tag_ids, tag_names))
        self.tag_frames: Dict[int, str] = dict(zip(tag_ids, tag_frames))
        self.states: Dict[int, TagState] = {tag_id: TagState() for tag_id in tag_ids} #Erstellt für jeden Tag im YAML eine eigene TagState Instanz

        #Für jeden Tag wird ein Publisher erstellt unter dem definierten [namespace]/[tag name] (u.a pose, visible
        self.pose_publishers = {
            tag_id: self.create_publisher(
                PoseStamped,
                f"{self.output_namespace}/{self.tag_names[tag_id]}/pose",
                10,
            )
            for tag_id in tag_ids
        }
        self.visible_publishers = {
            tag_id: self.create_publisher(
                Bool,
                f"{self.output_namespace}/{self.tag_names[tag_id]}/visible",
                10,
            )
            for tag_id in tag_ids
        }

        #Ein alleiniger publisher der die Liste aller sichtbaren Ids veröffentlich
        self.visible_ids_publisher = self.create_publisher(
            Int32MultiArray,
            f"{self.output_namespace}/visible_ids",
            10,
        )

        #Buffer für die tansformationen (raff ich ned)
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        #Hier wird apriltag_ros subscribed (frag nicht)
        self.subscription = self.create_subscription(
            AprilTagDetectionArray,
            detections_topic,
            self._on_detections,
            qos_profile_sensor_data, 
        )
        self.timer = self.create_timer(
            1.0 / publish_rate_hz,
            self._publish_state,
        )

        configured = ", ".join(
            f"{tag_id}:{self.tag_names[tag_id]}->{self.tag_frames[tag_id]}"
            for tag_id in tag_ids
        )
        frame_description = self.target_frame or "<camera frame from message>"
        self.get_logger().info(
            f"Listening on {detections_topic}; target_frame={frame_description}"
        )
        self.get_logger().info(f"Configured tags: {configured}")

    def _on_detections(self, msg: AprilTagDetectionArray) -> None:
        now_ns = self.get_clock().now().nanoseconds

        for detection in msg.detections:
            tag_id = int(detection.id)
            if tag_id not in self.states:
                continue
            if float(detection.decision_margin) < self.minimum_decision_margin:
                continue

            state = self.states[tag_id]
            gap_ns = now_ns - state.last_seen_ns if state.last_seen_ns else 0

            if (
                state.last_seen_ns == 0
                or gap_ns <= self.maximum_detection_gap_ns
            ):
                state.consecutive_detections += 1
            else:
                state.consecutive_detections = 1

            state.last_seen_ns = now_ns
            state.last_decision_margin = float(detection.decision_margin)
            state.source_frame = msg.header.frame_id #zu camera_frame

    def _publish_state(self) -> None:
        now_ns = self.get_clock().now().nanoseconds
        visible_ids: List[int] = []

        for tag_id, state in self.states.items():
            recently_seen = (
                state.last_seen_ns > 0
                and now_ns - state.last_seen_ns <= self.lost_timeout_ns
            )
            should_be_visible = (
                recently_seen
                and state.consecutive_detections
                >= self.minimum_consecutive_detections
            )

            if should_be_visible != state.visible:
                state.visible = should_be_visible
                status = "VISIBLE" if state.visible else "LOST"
                self.get_logger().info(
                    f"{self.tag_names[tag_id]} (ID {tag_id}) {status}; "
                    f"margin={state.last_decision_margin:.1f}"
                )

            self.visible_publishers[tag_id].publish(
                Bool(data=state.visible)
            )

            if not state.visible:
                if not recently_seen:
                    state.consecutive_detections = 0
                continue

            visible_ids.append(tag_id)
            self._publish_pose(tag_id, state)

        self.visible_ids_publisher.publish(
            Int32MultiArray(data=sorted(visible_ids))
        )

    def _publish_pose(self, tag_id: int, state: TagState) -> None:
        source_frame = self.tag_frames[tag_id] #zu tag_frames
        target_frame = self.target_frame or state.source_frame

        if not target_frame:
            self.get_logger().warn(
                f"No target frame available for tag ID {tag_id}",
                throttle_duration_sec=2.0,
            )
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame, #zu camera_frame
                source_frame, #zu tag_frame
                Time(),  # latest available tag transform
                timeout=Duration(seconds=0.05),
            )
        except TransformException as exc:
            self.get_logger().warn(
                f"Cannot transform {source_frame} -> {target_frame}: {exc}", #sf zu tag_frame, tf zu camera_frame
                throttle_duration_sec=2.0,
            )
            return

        pose = PoseStamped()
        pose.header = transform.header
        pose.pose.position.x = transform.transform.translation.x
        pose.pose.position.y = transform.transform.translation.y
        pose.pose.position.z = transform.transform.translation.z
        pose.pose.orientation = transform.transform.rotation
        self.pose_publishers[tag_id].publish(pose)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FixedCameraTagTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
