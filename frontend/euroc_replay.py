"""Replay a EuRoC sequence as raw stereo topics for one robot.

Stamps are wall-clock at publish time; the mapping to EuRoC timestamps is written
to --stamps so trajectories can be compared with ground truth later.

Usage: python -m frontend.euroc_replay --robot 0 --mav0 <seq>/mav0 --rate 0.5 --skip 2
"""
import argparse
import csv
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header

from frontend.rosimg import to_msg


def read_frames(mav0):
    def rows(cam):
        with open(f"{mav0}/{cam}/data.csv") as f:
            return {int(r[0]): r[1] for r in csv.reader(f) if not r[0].startswith("#")}
    left, right = rows("cam0"), rows("cam1")
    return [(ts, left[ts], right[ts]) for ts in sorted(left) if ts in right]


def parse_perturb(spec, robot):
    """'r1:blank:0.40:0.47,r2:shake:0.60:0.66' -> [(kind, start, end)] for this robot,
    with start/end as fractions of the sequence. Used for the spec's failure cases."""
    out = []
    for item in filter(None, spec.split(",")):
        r, kind, a, b = item.split(":")
        if int(r.lstrip("r")) == robot:
            assert kind in ("blank", "shake"), kind
            out.append((kind, float(a), float(b)))
    return out


def perturb(kind, left, right, rng):
    """blank: a textureless wall filling the view. shake: strong motion blur plus jitter."""
    if kind == "blank":
        h, w = left.shape
        wall = np.full((h, w), 140, np.uint8) + np.linspace(-12, 12, w, dtype=np.float32).astype(np.int8)
        wall = np.clip(wall.astype(np.int16) + rng.normal(0, 2, (h, w)), 0, 255).astype(np.uint8)
        return wall, wall.copy()
    length = int(rng.integers(15, 35))
    k = np.zeros((length, length), np.float32)
    k[length // 2, :] = 1.0 / length
    rot = cv2.getRotationMatrix2D((length / 2 - 0.5, length / 2 - 0.5), float(rng.uniform(0, 180)), 1.0)
    k = cv2.warpAffine(k, rot, (length, length))
    k /= k.sum()
    shift = np.float32([[1, 0, rng.uniform(-15, 15)], [0, 1, rng.uniform(-15, 15)]])
    out = [cv2.warpAffine(cv2.filter2D(img, -1, k), shift, img.shape[::-1], borderMode=cv2.BORDER_REFLECT)
           for img in (left, right)]
    return out[0], out[1]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--robot", type=int, required=True)
    ap.add_argument("--mav0", required=True)
    ap.add_argument("--rate", type=float, default=0.5, help="playback speed vs real time")
    ap.add_argument("--skip", type=int, default=2, help="publish every Nth frame")
    ap.add_argument("--start-delay", type=float, default=20.0)
    ap.add_argument("--stamps", required=True, help="CSV: ros_stamp_ns,euroc_ts_ns[,perturbation]")
    ap.add_argument("--perturb", default="", help="failure injection, e.g. r1:blank:0.40:0.47")
    args = ap.parse_args()
    windows = parse_perturb(args.perturb, args.robot)
    rng = np.random.default_rng(args.robot)

    rclpy.init()
    node = Node(f"euroc_replay_{args.robot}")
    ns = f"/r{args.robot}/stereo_camera"
    pub_l = node.create_publisher(Image, f"{ns}/left/image_raw", 10)
    pub_r = node.create_publisher(Image, f"{ns}/right/image_raw", 10)
    frames = read_frames(args.mav0)[::args.skip]
    node.get_logger().info(f"{len(frames)} stereo pairs, starting in {args.start_delay}s")
    time.sleep(args.start_delay)

    t0_wall, t0_data = time.monotonic(), frames[0][0]
    with open(args.stamps, "w") as out:
        out.write("ros_stamp_ns,euroc_ts_ns,perturbation\n")
        for n, (ts, fl, fr) in enumerate(frames):
            target = t0_wall + (ts - t0_data) / 1e9 / args.rate
            time.sleep(max(0.0, target - time.monotonic()))
            left = cv2.imread(f"{args.mav0}/cam0/data/{fl}", cv2.IMREAD_GRAYSCALE)
            right = cv2.imread(f"{args.mav0}/cam1/data/{fr}", cv2.IMREAD_GRAYSCALE)
            frac = n / len(frames)
            kind = next((k for k, a, b in windows if a <= frac < b), "")
            if kind:
                left, right = perturb(kind, left, right, rng)
            header = Header(frame_id="camera_link", stamp=node.get_clock().now().to_msg())
            pub_l.publish(to_msg(left, header))
            pub_r.publish(to_msg(right, header))
            out.write(f"{header.stamp.sec * 10**9 + header.stamp.nanosec},{ts},{kind}\n")
    node.get_logger().info("replay finished")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
