"""AirMuseum ground truth -> EuRoC-style csv for eval/cslam_eval.py.

The dataset gives world poses of the cam100 frame (the RIGHT eye) at 10 Hz in seconds
(timestamp tx ty tz qx qy qz qw). We need the LEFT eye (cam101) at our image timestamps:
  T_w_left = T_w_cam100 @ inv(T_cam101_cam100)      (Kalibr cam1.T_cn_cnm1 = cam100 -> cam101)
Interpolated (lerp position, slerp rotation) to each converted pair's timestamp.
Writes <gt_dir>/AM_<R>.csv (#timestamp,px,py,pz,qw,qx,qy,qz) and <gt_dir>/calib/AM_<R>.yaml with T_B_c0 = identity
(the csv is already the left camera pose, so body == cam0).
Usage: python scripts/airmuseum_gt.py <clips_dir> <sensors_dir> <gt_dir>
"""
import os
import sys

import numpy as np
import yaml
from scipy.spatial.transform import Rotation, Slerp


def main(clips, sensors, gt_dir):
    os.makedirs(f"{gt_dir}/calib", exist_ok=True)
    for r in "ABC":
        with open(f"{sensors}/robot{r}_cameras_calib.yaml") as f:
            T_c1_c0 = np.array(yaml.safe_load(f)["cam1"]["T_cn_cnm1"], dtype=float)
        T_c0_c1 = np.linalg.inv(T_c1_c0)  # left(cam101) pose = right(cam100) pose @ this
        g = np.loadtxt(f"{clips}/{r}/cam100_stamped_groundtruth.txt")
        ts = np.array([int(l.split(",")[0]) for l in open(f"{clips}/{r}/mav0/cam0/data.csv") if not l.startswith("#")])
        t = ts / 1e9
        ok = (t >= g[0, 0]) & (t <= g[-1, 0])
        t, ts = t[ok], ts[ok]
        rot = Slerp(g[:, 0], Rotation.from_quat(g[:, 4:8]))(t)
        pos = np.stack([np.interp(t, g[:, 0], g[:, i]) for i in (1, 2, 3)], 1)
        rows = []
        for tn, p, R in zip(ts, pos, rot.as_matrix()):
            T = np.eye(4); T[:3, :3] = R; T[:3, 3] = p
            L = T @ T_c0_c1
            x, y, z, w = Rotation.from_matrix(L[:3, :3]).as_quat()
            rows.append(f"{tn},{L[0,3]:.6f},{L[1,3]:.6f},{L[2,3]:.6f},{w:.8f},{x:.8f},{y:.8f},{z:.8f}")
        with open(f"{gt_dir}/AM_{r}.csv", "w") as f:
            f.write("#timestamp [ns],p_x,p_y,p_z,q_w,q_x,q_y,q_z\n" + "\n".join(rows) + "\n")
        with open(f"{gt_dir}/calib/AM_{r}.yaml", "w") as f:
            yaml.safe_dump({"T_B_c0": np.eye(4).tolist(), "note": "GT csv is already the left-eye (cam101) pose"}, f)
        print(f"robot{r}: {len(rows)} gt poses (of {len(ok)} frames), baseline offset {np.linalg.norm(T_c0_c1[:3,3]):.4f} m, "
              f"rot offset {np.degrees(np.linalg.norm(Rotation.from_matrix(T_c0_c1[:3,:3]).as_rotvec())):.2f} deg")


if __name__ == "__main__":
    main(*sys.argv[1:4])
