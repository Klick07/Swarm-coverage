"""Record everything the coordinator acts on into one events.jsonl per run.

Perception (ROS) -> events.jsonl -> coordinator. The coordinator never imports rclpy,
so it runs live (following the file) or as a replay of a recorded run.

Event types:
  quality   OpenCV 5 front-end scores for one frame          (/rN/quality)
  keyframe  Swarm-SLAM keyframe id <-> image stamp, odom pose  (/rN/cslam/keyframe_odom)
  lc        inter-robot loop closure, with cslam's success    (/cslam/inter_robot_loop_closure)
  poses     merged-frame keyframe positions, throttled        (/rN/cslam/optimized_estimates)

Usage: python -m coordinator.recorder --robots 3 --out runs/<run>/events.jsonl
"""
import argparse
import json
import time

import rclpy
from cslam_common_interfaces.msg import InterRobotLoopClosure, KeyframeOdom, OptimizationResult
from rclpy.node import Node
from std_msgs.msg import String


def stamp_ns(stamp):
    return stamp.sec * 10**9 + stamp.nanosec


class Recorder(Node):
    def __init__(self, robots, out, poses_period_s):
        super().__init__("event_recorder")
        self.out = open(out, "a", buffering=1)
        self.poses_period = poses_period_s
        self.last_poses = {}
        for r in range(robots):
            self.create_subscription(String, f"/r{r}/quality", self.on_quality, 100)
            self.create_subscription(KeyframeOdom, f"/r{r}/cslam/keyframe_odom",
                                     lambda m, r=r: self.on_keyframe(r, m), 100)
            # Each robot's topic carries only that robot's merged-frame poses, so throttle per robot.
            self.create_subscription(OptimizationResult, f"/r{r}/cslam/optimized_estimates",
                                     lambda m, r=r: self.on_estimates(r, m), 10)
        self.create_subscription(InterRobotLoopClosure, "/cslam/inter_robot_loop_closure", self.on_lc, 100)

    def write(self, kind, **data):
        self.out.write(json.dumps({"t": round(time.time(), 3), "type": kind, **data}) + "\n")

    def on_quality(self, msg):
        self.write("quality", **json.loads(msg.data))

    def on_keyframe(self, robot, msg):
        p = msg.odom.pose.pose.position
        self.write("keyframe", robot=robot, kf=msg.id, stamp_ns=stamp_ns(msg.odom.header.stamp),
                   odom=[round(p.x, 3), round(p.y, 3), round(p.z, 3)])

    def on_lc(self, msg):
        t = msg.transform.translation
        self.write("lc", a=msg.robot0_id, ka=msg.robot0_keyframe_id, b=msg.robot1_id, kb=msg.robot1_keyframe_id,
                   success=msg.success, translation=[round(t.x, 3), round(t.y, 3), round(t.z, 3)])

    def on_estimates(self, robot, msg):
        now = time.time()
        if not msg.success or now - self.last_poses.get(robot, 0.0) < self.poses_period:
            return
        self.last_poses[robot] = now
        poses = [[e.key.keyframe_id, round(e.pose.position.x, 3), round(e.pose.position.y, 3),
                  round(e.pose.position.z, 3)] for e in msg.estimates]
        self.write("poses", robot=robot, origin=msg.origin_robot_id, poses=poses)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--robots", type=int, default=3)
    ap.add_argument("--out", required=True)
    ap.add_argument("--poses-period", type=float, default=5.0)
    args = ap.parse_args()
    rclpy.init()
    rclpy.spin(Recorder(args.robots, args.out, args.poses_period))


if __name__ == "__main__":
    main()
