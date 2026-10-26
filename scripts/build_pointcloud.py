"""Dense point cloud of the merged map: each Swarm-SLAM keyframe's stereo depth at its optimized pose.

Every keyframe's rectified pair (frames/rN/<stamp>.jpg + frames/rN_right/<stamp>.jpg) goes through
SGBM once; its camera-frame points are cached and re-placed on every pass with the latest optimized
pose graph, so loop closures straighten the cloud as well. Camera frame = robot base frame (see the
launch file), so a keyframe pose maps its points straight into the merged frame.
Writes <run>/pointcloud.pcd (binary, fields x y z rgb robot t), colored by camera, shaded by image
intensity, leveled so the floor is z = 0 with z up (found once by RANSAC, then fixed so the view doesn't
jump). t = seconds after the header's t0_ns at which that camera first saw the voxel, so the dashboard
can grow the cloud along with a replay; <run>/pointcloud_traj.json holds the keyframe paths, same frame.

Usage: python -m scripts.build_pointcloud ~/runs/<run> [--watch 10]
"""
import argparse
import glob
import json
import os
import re
import time
from datetime import datetime, timedelta

import cv2
import numpy as np

from frontend.core import StereoRectifier, load_calib

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ROBOT_RGB = np.array([[0x39, 0x87, 0xe5], [0xd9, 0x59, 0x26], [0x19, 0x9e, 0x70]]) / 255.0  # dashboard palette
MAX_DEPTH_M = 3.0   # depth error grows with z^2: ~5 cm per 1/4 px of disparity at 3 m; beyond that stereo guesses
STRIDE = 4          # keep every 4th pixel in each direction before voxelizing
VOXEL_M = 0.04


def read_frame(path_no_ext):
    """Frames are PNG in new runs, JPG in older ones."""
    for ext in (".png", ".jpg"):
        img = cv2.imread(path_no_ext + ext, cv2.IMREAD_GRAYSCALE)
        if img is not None:
            return img
    return None


def run_sequences(run):
    for tok in open(f"{run}/params.txt").read().split():
        if tok.startswith("seqs="):
            return tok[5:].split(",")
    return ["MH_01_easy", "MH_02_easy", "MH_03_medium"]


def latest_graph(run):
    """Newest optimized global pose graph written since this run started.

    Swarm-SLAM writes snapshots to ~/runs/cslam/<experiment>/<dd-mm-yyyy_HH-MM-SS>/ during the run;
    run_euroc.sh copies that whole folder, older experiments included, to <run>/cslam_logs at the
    end. Copies lose their mtimes, so go by the snapshot folder's name and the start/end in params.txt."""
    params = open(f"{run}/params.txt").read()
    stamp = lambda k, default: (datetime.fromisoformat(m.group(1)).replace(tzinfo=None)
                                if (m := re.search(rf"{k}=(\S+)", params)) else default)
    start, end = stamp("start", datetime.min), stamp("end", datetime.max)  # end: later runs share ~/runs/cslam
    best, best_t = None, start - timedelta(seconds=5)
    for root in (f"{run}/cslam_logs", os.path.expanduser("~/runs/cslam")):
        for path in glob.glob(f"{root}/*/*/optimized_global_pose_graph.g2o"):
            try:
                t = datetime.strptime(os.path.basename(os.path.dirname(path)), "%d-%m-%Y_%H-%M-%S")
            except ValueError:
                continue
            if best_t < t <= end:
                best, best_t = path, t
    return best


def read_graph(path):
    """{(robot, kf): 4x4 pose}. Keys are gtsam LabeledSymbols: label 'A'+robot in bits 48-55."""
    poses = {}
    for line in open(path):
        if not line.startswith("VERTEX_SE3:QUAT"):
            continue
        tok = line.split()
        key = int(tok[1])
        x, y, z, qx, qy, qz, qw = map(float, tok[2:9])
        T = np.eye(4)
        T[:3, :3] = quat_to_rot(qx, qy, qz, qw)
        T[:3, 3] = (x, y, z)
        poses[(((key >> 48) & 0xFF) - ord("A"), key & ((1 << 48) - 1))] = T
    return poses


def quat_to_rot(x, y, z, w):
    n = np.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class Builder:
    def __init__(self, run):
        self.run = run
        # upload jobs keep each robot's calibration next to the run; EuRoC runs use the repo's calib/
        calib = lambda s: f"{run}/calib/{s}.yaml" if os.path.exists(f"{run}/calib/{s}.yaml") else f"{REPO}/calib/{s}.yaml"
        self.rect = [StereoRectifier(load_calib(calib(s))) for s in run_sequences(run)]
        self.sgbm = cv2.StereoSGBM_create(minDisparity=0, numDisparities=64, blockSize=7,
                                          P1=8 * 7 * 7, P2=32 * 7 * 7, disp12MaxDiff=1, uniquenessRatio=18,
                                          speckleWindowSize=200, speckleRange=2,
                                          mode=cv2.STEREO_SGBM_MODE_SGBM)
        # WLS filter smooths disparity while keeping edges; needs the right-view matcher for confidence.
        self.right_matcher = cv2.ximgproc.createRightMatcher(self.sgbm)
        self.wls = cv2.ximgproc.createDisparityWLSFilter(self.sgbm)
        self.kf_stamp = {}   # (robot, kf) -> image stamp_ns
        self.local = {}      # (robot, kf) -> (Nx3 camera-frame points, N intensities)
        self.events_pos = 0
        self.level = None    # 4x4 merged frame -> floor frame

    def read_keyframes(self):
        if not os.path.exists(f"{self.run}/events.jsonl"):
            return
        with open(f"{self.run}/events.jsonl", "rb") as f:
            f.seek(self.events_pos)
            for line in f:
                if not line.endswith(b"\n"):
                    break  # recorder is mid-write; pick it up next pass
                self.events_pos += len(line)
                ev = json.loads(line)
                if ev["type"] == "keyframe":
                    self.kf_stamp[(ev["robot"], ev["kf"])] = ev["stamp_ns"]

    def keyframe_points(self, robot, kf):
        stamp = self.kf_stamp.get((robot, kf))
        left = read_frame(f"{self.run}/frames/r{robot}/{stamp}")
        right = read_frame(f"{self.run}/frames/r{robot}_right/{stamp}")
        if stamp is None or left is None or right is None:
            return None
        disp_l = self.sgbm.compute(left, right)
        disp_r = self.right_matcher.compute(right, left)
        disp = self.wls.filter(disp_l, left, disparity_map_right=disp_r).astype(np.float32) / 16.0  # x16 fixed-point
        P = self.rect[robot].P1
        f, cx, cy, b = P[0, 0], P[0, 2], P[1, 2], self.rect[robot].baseline
        d = disp[::STRIDE, ::STRIDE]
        v, u = np.mgrid[0:left.shape[0]:STRIDE, 0:left.shape[1]:STRIDE]
        ok = d > 1.0
        z = np.zeros_like(d)
        z[ok] = f * b / d[ok]
        ok &= (z > 0.3) & (z < MAX_DEPTH_M)
        z, u, v = z[ok], u[ok], v[ok]
        pts = np.column_stack([(u - cx) * z / f, (v - cy) * z / f, z])
        return pts, left[::STRIDE, ::STRIDE][ok]

    def build(self):
        self.read_keyframes()
        graph = latest_graph(self.run)
        if not graph:
            return None
        clouds, colors, robots, stamps, kfs = [], [], [], [], []
        poses = read_graph(graph)
        for (robot, kf), T in poses.items():
            if robot >= len(self.rect):
                continue
            if (robot, kf) not in self.local:
                got = self.keyframe_points(robot, kf)
                if got is None:
                    continue  # keyframe event or its frames not written yet; retry next pass
                self.local[(robot, kf)] = got
            pts, gray = self.local[(robot, kf)]
            clouds.append(pts @ T[:3, :3].T + T[:3, 3])
            colors.append(ROBOT_RGB[robot] * (0.35 + 0.65 * gray[:, None] / 255.0))
            robots.append(np.full(len(pts), robot))
            stamps.append(np.full(len(pts), self.kf_stamp[(robot, kf)]))
            kfs.append(np.full(len(pts), kf))
        if not clouds:
            return None
        t0 = min(self.kf_stamp.values())
        pts, rgb, robot, t = voxelize(np.vstack(clouds), np.vstack(colors), np.concatenate(robots),
                                      np.concatenate(kfs), (np.concatenate(stamps) - t0) / 1e9, VOXEL_M)
        if self.level is None and len(clouds) >= 100:
            self.level = floor_frame(pts, np.array([T[:3, 3] for T in poses.values()]))
        level = self.level if self.level is not None else np.eye(4)
        pts = pts @ level[:3, :3].T + level[:3, 3]
        write_pcd(f"{self.run}/pointcloud.pcd", pts, rgb, robot, t, t0)
        paths = {}
        for (r, kf), T in sorted(poses.items(), key=lambda item: self.kf_stamp.get(item[0], 0)):
            if (r, kf) in self.kf_stamp:
                p = level[:3, :3] @ T[:3, 3] + level[:3, 3]
                paths.setdefault(r, []).append([round((self.kf_stamp[(r, kf)] - t0) / 1e9, 3), *np.round(p, 3).tolist()])
        with open(f"{self.run}/pointcloud_traj.json.tmp", "w") as f:
            json.dump({"t0_ns": t0, "paths": paths}, f)
        os.replace(f"{self.run}/pointcloud_traj.json.tmp", f"{self.run}/pointcloud_traj.json")
        return len(pts)


def floor_frame(pts, cams, iters=300, tol=0.03):
    """Rigid transform putting the biggest roughly-horizontal plane at z = 0, cameras on the +z side.

    Horizontal = normal within 45 deg of the camera y axis (image down), which the merged frame
    inherits from robot 0's first keyframe."""
    rng = np.random.default_rng(0)
    sample = pts[rng.choice(len(pts), min(len(pts), 50000), replace=False)]
    best_n, best_d, best_in = None, 0.0, 0
    for _ in range(iters):
        a, b, c = sample[rng.choice(len(sample), 3, replace=False)]
        n = np.cross(b - a, c - a)
        if np.linalg.norm(n) < 1e-9:
            continue
        n /= np.linalg.norm(n)
        if abs(n[1]) < np.cos(np.radians(45)):
            continue
        inliers = np.count_nonzero(np.abs((sample - a) @ n) < tol)
        if inliers > best_in:
            best_n, best_d, best_in = n, -a @ n, inliers
    if best_n is None:
        return np.eye(4)
    if np.median(cams @ best_n + best_d) < 0:
        best_n, best_d = -best_n, -best_d
    z = best_n
    x = np.cross([0.0, 0.0, 1.0], z) if abs(z[2]) < 0.9 else np.cross([1.0, 0.0, 0.0], z)
    x /= np.linalg.norm(x)
    T = np.eye(4)
    T[:3, :3] = np.vstack([x, np.cross(z, x), z])
    T[2, 3] = best_d
    return T


def voxelize(pts, rgb, robot, kf, t, size):
    """One point per occupied voxel and camera, keeping voxels seen by at least 2 DIFFERENT keyframes
    (drops single-frame stereo noise). Position and color are averaged; t is the earliest sighting."""
    ijk = np.floor(pts / size).astype(np.int64) + (1 << 19)  # one int64 key: 20 bits per axis + camera
    keys = (ijk[:, 0] << 42) | (ijk[:, 1] << 22) | (ijk[:, 2] << 2) | robot
    _, first, inv, counts = np.unique(keys, return_index=True, return_inverse=True, return_counts=True)
    pair = np.unique(np.column_stack([keys, kf]), axis=0)  # distinct (voxel, keyframe) sightings
    _, kf_counts = np.unique(pair[:, 0], return_counts=True)
    keep = kf_counts >= 2
    mean = lambda a: np.column_stack([np.bincount(inv, a[:, i]) for i in range(3)]) / counts[:, None]
    tmin = np.full(len(counts), np.inf)
    np.minimum.at(tmin, inv, t)
    return mean(pts)[keep], mean(rgb)[keep], robot[first][keep], tmin[keep]


def write_pcd(path, pts, rgb, robot, t, t0_ns):
    """Binary PCD with packed float rgb, written to a temp file and renamed so readers never see half a file."""
    c = np.clip(rgb * 255, 0, 255).astype(np.uint32)
    packed = ((c[:, 0] << 16) | (c[:, 1] << 8) | c[:, 2]).view(np.float32)
    data = np.column_stack([pts.astype(np.float32), packed, robot.astype(np.float32),
                            t.astype(np.float32)]).astype(np.float32)
    header = (f"# .PCD v0.7 - Point Cloud Data file format\n# t0_ns {t0_ns}\nVERSION 0.7\n"
              f"FIELDS x y z rgb robot t\nSIZE 4 4 4 4 4 4\nTYPE F F F F F F\nCOUNT 1 1 1 1 1 1\n"
              f"WIDTH {len(data)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(data)}\nDATA binary\n")
    with open(f"{path}.tmp", "wb") as f:
        f.write(header.encode())
        f.write(data.tobytes())
    os.replace(f"{path}.tmp", path)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run")
    ap.add_argument("--watch", type=float, default=0.0, help="rebuild every N seconds until killed")
    args = ap.parse_args()
    builder = Builder(os.path.expanduser(args.run))
    while True:
        t0 = time.perf_counter()
        n = builder.build()
        print(f"pointcloud: {n or 0} points from {len(builder.local)} keyframes "
              f"in {time.perf_counter() - t0:.1f} s", flush=True)
        if not args.watch:
            break
        time.sleep(args.watch)


if __name__ == "__main__":
    main()
