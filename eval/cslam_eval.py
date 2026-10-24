"""Turn a Swarm-SLAM run into TUM trajectories and ATE numbers against EuRoC ground truth.

Usage: python -m eval.cslam_eval <run_dir> <gt_dir>
  run_dir: ~/runs/<run> copied from EC2 (stamps_rN.csv, cslam_logs/.../*.g2o, pose_timestamps*.csv)
  gt_dir:  holds MH_01_easy.csv ... (EuRoC state_groundtruth_estimate0/data.csv) and calib/euroc_cam0.yaml

Writes <run_dir>/eval/{initial,optimized}_rN.tum, gt_rN.tum and metrics.json.
"Initial" = each robot's odometry chained in the global graph before optimization;
"optimized" = the merged Swarm-SLAM estimate. Joint ATE aligns all robots with ONE
SE(3) transform, so it only stays low if the robots' maps are merged consistently.
"""
import csv
import glob
import json
import os
import sys

import numpy as np
import yaml
from scipy.spatial.transform import Rotation

SEQUENCES = ["MH_01_easy", "MH_02_easy", "MH_03_medium"]  # default, and for runs made before seqs= was recorded


def run_sequences(run_dir):
    """EuRoC sequence per robot, as recorded by scripts/run_euroc.sh in params.txt."""
    try:
        for tok in open(f"{run_dir}/params.txt").read().split():
            if tok.startswith("seqs="):
                return tok[5:].split(",")
    except FileNotFoundError:
        pass
    return SEQUENCES


def body_to_cam0(gt_dir, seq):
    """EuRoC T_BS of cam0 for this sequence (all sequences share one rig; older gt dirs hold one file)."""
    for path in (f"{gt_dir}/calib/{seq}.yaml", f"{gt_dir}/calib/euroc_cam0.yaml"):
        if os.path.exists(path):
            with open(path) as f:
                return np.array(yaml.safe_load(f)["T_B_c0"])
    raise FileNotFoundError(f"no calibration for {seq} in {gt_dir}/calib")
# camera_link -> camera optical frame. Identity since run euroc3_r025_c (see launch file);
# runs euroc3_r025 and _b used roll -90 / yaw -90 and need that rotation here instead.
T_LINK_OPT = np.eye(4)


def decode_key(key):
    """gtsam::LabeledSymbol -> (robot, index); cslam uses label 'A'+robot."""
    return ((key >> 48) & 0xFF) - ord("A"), key & 0xFFFFFFFFFFFF


def read_g2o(path):
    poses = {}
    with open(path) as f:
        for line in f:
            p = line.split()
            if p and p[0] == "VERTEX_SE3:QUAT":
                T = np.eye(4)
                T[:3, 3] = [float(v) for v in p[2:5]]
                T[:3, :3] = Rotation.from_quat([float(v) for v in p[5:9]]).as_matrix()
                poses[decode_key(int(p[1]))] = T
    return poses


def read_pose_times(cslam_logs):
    """Each robot writes its own pose_timestamps<N>.csv into its own log folder; keys are unique."""
    times = {}
    for path in sorted(glob.glob(f"{cslam_logs}/**/pose_timestamps*.csv", recursive=True)):
        with open(path) as f:
            for r in csv.DictReader(f):
                times[decode_key(int(r["vertice_id"]))] = int(r["sec"]) * 10**9 + int(r["nanosec"])
    return times


def read_stamp_map(run_dir, robot):
    with open(f"{run_dir}/stamps_r{robot}.csv") as f:
        return {int(r["ros_stamp_ns"]): int(r["euroc_ts_ns"]) for r in csv.DictReader(f)}


def read_gt(path, T_B_c0):
    """EuRoC body poses -> camera_link poses, keyed by timestamp (ns)."""
    gt = {}
    with open(path) as f:
        for r in csv.reader(f):
            if r[0].startswith("#"):
                continue
            T = np.eye(4)
            T[:3, 3] = [float(v) for v in r[1:4]]
            w, x, y, z = (float(v) for v in r[4:8])
            T[:3, :3] = Rotation.from_quat([x, y, z, w]).as_matrix()
            gt[int(r[0])] = T @ T_B_c0 @ np.linalg.inv(T_LINK_OPT)
    return gt


def nearest(gt, ts, tol_ns=5_000_000):
    keys = np.fromiter(gt.keys(), dtype=np.int64)
    k = keys[np.abs(keys - ts).argmin()]
    return gt[int(k)] if abs(int(k) - ts) <= tol_ns else None


def umeyama_se3(src, dst):
    """Rigid transform minimizing ||R src + t - dst|| (no scale: stereo is metric)."""
    mu_s, mu_d = src.mean(0), dst.mean(0)
    U, _, Vt = np.linalg.svd((dst - mu_d).T @ (src - mu_s))
    S = np.diag([1, 1, np.sign(np.linalg.det(U @ Vt))])
    R = U @ S @ Vt
    return R, mu_d - R @ mu_s


def ate(est, ref):
    R, t = umeyama_se3(est, ref)
    err = np.linalg.norm((est @ R.T + t) - ref, axis=1)
    return float(np.sqrt((err ** 2).mean())), R, t


def write_tum(path, rows):
    with open(path, "w") as f:
        for ts, T in rows:
            q = Rotation.from_matrix(T[:3, :3]).as_quat()
            f.write(f"{ts / 1e9:.9f} {T[0,3]:.6f} {T[1,3]:.6f} {T[2,3]:.6f} {q[0]:.6f} {q[1]:.6f} {q[2]:.6f} {q[3]:.6f}\n")


def main(run_dir, gt_dir):
    out = f"{run_dir}/eval"
    os.makedirs(out, exist_ok=True)
    seqs = run_sequences(run_dir)
    result_dirs = sorted(glob.glob(f"{run_dir}/cslam_logs/**/optimized_global_pose_graph.g2o", recursive=True))
    if not result_dirs:
        sys.exit(f"no optimized_global_pose_graph.g2o under {run_dir}/cslam_logs")
    logs = os.path.dirname(result_dirs[-1])  # latest write_logs() snapshot
    times = read_pose_times(f"{run_dir}/cslam_logs")
    metrics = {"cslam_log_folder": os.path.relpath(logs, run_dir)}

    pairs = {}  # kind -> robot -> list of (euroc_ts, est T, gt T)
    for kind in ("initial", "optimized"):
        poses = read_g2o(f"{logs}/{kind}_global_pose_graph.g2o")
        pairs[kind] = {}
        for robot, seq in enumerate(seqs):
            stamp_map = read_stamp_map(run_dir, robot)
            gt = read_gt(f"{gt_dir}/{seq}.csv", body_to_cam0(gt_dir, seq))
            rows = []
            for (r, idx), T in sorted(poses.items()):
                if r != robot or (r, idx) not in times or times[(r, idx)] not in stamp_map:
                    continue
                ts = stamp_map[times[(r, idx)]]
                G = nearest(gt, ts)
                if G is not None:
                    rows.append((ts, T, G))
            pairs[kind][robot] = rows
            write_tum(f"{out}/{kind}_r{robot}.tum", [(ts, T) for ts, T, _ in rows])
            if kind == "optimized":
                write_tum(f"{out}/gt_r{robot}.tum", [(ts, G) for ts, _, G in rows])

        per_robot = {}
        for robot, rows in pairs[kind].items():
            if len(rows) >= 3:
                est = np.array([T[:3, 3] for _, T, _ in rows])
                ref = np.array([G[:3, 3] for _, _, G in rows])
                per_robot[f"r{robot}"] = {"poses": len(rows), "ate_rmse_m": round(ate(est, ref)[0], 3)}
        all_rows = [row for rows in pairs[kind].values() for row in rows]
        joint = None
        if len(all_rows) >= 3 and sum(1 for rows in pairs[kind].values() if rows) > 1:
            est = np.array([T[:3, 3] for _, T, _ in all_rows])
            ref = np.array([G[:3, 3] for _, _, G in all_rows])
            joint = round(ate(est, ref)[0], 3)
        metrics[kind] = {"per_robot_ate": per_robot, "joint_ate_rmse_m": joint}

    with open(f"{out}/metrics.json", "w") as f:
        json.dump(metrics, f, indent=2)
    print(json.dumps(metrics, indent=2))


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
