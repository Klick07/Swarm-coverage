"""OpenCV 5 front end for one robot: raw stereo -> rectified, quality-scored input for Swarm-SLAM.

Subscribes  /rN/stereo_camera/{left,right}/image_raw
Publishes   /rN/stereo_camera/{left,right}/image_rect_color   (mono8, Swarm-SLAM input)
            /rN/stereo_camera/{left,right}/camera_info
            /rN/quality                                       (std_msgs/String, JSON)
Frames failing the quality gate are logged but not forwarded.

Usage: python -m frontend.node --robot 0 --calib calib/cam0.yaml --log runs/x/quality_r0.jsonl
"""
import argparse
import json
import os
import time

import cv2
import message_filters
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

from frontend.core import QualityScorer, StereoRectifier, load_calib, passes_gate
from frontend.rosimg import camera_info, from_msg, to_msg


class FrontEnd(Node):
    def __init__(self, args):
        super().__init__(f"frontend_{args.robot}")
        self.robot = args.robot
        self.rect = StereoRectifier(load_calib(args.calib))
        self.scorer = QualityScorer()
        self.min_blur, self.min_features = args.min_blur, args.min_features
        self.log = open(args.log, "a", buffering=1)
        # Rectified left images, named by stamp, so the coordinator can re-verify overlaps.
        self.frames_dir = args.frames_dir
        if self.frames_dir:
            os.makedirs(self.frames_dir, exist_ok=True)
            os.makedirs(f"{self.frames_dir}_right", exist_ok=True)  # Also save right frames for dense reconstruction
        ns = f"/r{args.robot}/stereo_camera"
        self.pub = {side: self.create_publisher(Image, f"{ns}/{side}/image_rect_color", 10)
                    for side in ("left", "right")}
        self.pub_info = {side: self.create_publisher(CameraInfo, f"{ns}/{side}/camera_info", 10)
                         for side in ("left", "right")}
        self.pub_q = self.create_publisher(String, f"/r{args.robot}/quality", 10)
        subs = [message_filters.Subscriber(self, Image, f"{ns}/{side}/image_raw") for side in ("left", "right")]
        self.sync = message_filters.TimeSynchronizer(subs, 10)
        self.sync.registerCallback(self.on_pair)
        self.get_logger().info(f"OpenCV {cv2.__version__}, baseline {self.rect.baseline:.4f} m")

    def on_pair(self, msg_l, msg_r):
        t0 = time.perf_counter()
        left, right = self.rect.rectify(from_msg(msg_l), from_msg(msg_r))
        scores = self.scorer.score(left, right)
        passed = passes_gate(scores, self.min_blur, self.min_features)
        if passed:
            h = msg_l.header
            self.pub["left"].publish(to_msg(left, h))
            self.pub["right"].publish(to_msg(right, h))
            self.pub_info["left"].publish(camera_info(self.rect.P1, self.rect.size, h))
            self.pub_info["right"].publish(camera_info(self.rect.P2, self.rect.size, h))
        stamp_ns = msg_l.header.stamp.sec * 10**9 + msg_l.header.stamp.nanosec
        if passed and self.frames_dir:
            cv2.imwrite(f"{self.frames_dir}/{stamp_ns}.png", left)  # lossless: JPEG artifacts hurt stereo matching
            cv2.imwrite(f"{self.frames_dir}_right/{stamp_ns}.png", right)
        record = {"robot": self.robot, "stamp_ns": stamp_ns,
                  **scores, "passed": passed, "ms": round((time.perf_counter() - t0) * 1000, 1)}
        self.pub_q.publish(String(data=json.dumps(record)))
        self.log.write(json.dumps(record) + "\n")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--robot", type=int, required=True)
    ap.add_argument("--calib", required=True)
    ap.add_argument("--log", required=True)
    ap.add_argument("--frames-dir", default="")
    # Placeholder gates, deliberately permissive. Tune on recorded runs (spec Phase 5 step 4).
    ap.add_argument("--min-blur", type=float, default=0.0)
    ap.add_argument("--min-features", type=int, default=0)
    args = ap.parse_args()
    # The venv can silently fall back to the system OpenCV 4 (python3-opencv); refuse that.
    assert cv2.__version__.startswith("5."), f"need OpenCV 5, got {cv2.__version__} from {cv2.__file__}"
    rclpy.init()
    rclpy.spin(FrontEnd(args))


if __name__ == "__main__":
    main()
