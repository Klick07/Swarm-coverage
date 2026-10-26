"""Failure case "two similar-looking rooms": find cross-camera keyframe pairs that are far apart
in ground truth but look most alike to the coordinator's OpenCV 5 check, and write them as
fake inter-robot loop closures (success=true, as if Swarm-SLAM had wrongly accepted them).

Usage: python -m eval.similar_places <run_dir> <gt_dir> [n_samples] [top_k]
Writes <run_dir>/injected_false_lc.jsonl and prints the pairs.
"""
import json
import random
import sys

import cv2
import numpy as np

from coordinator.verify import verify_pair
from eval.cslam_eval import body_to_cam0, nearest, read_gt, read_stamp_map, run_sequences

MIN_GT_DIST_M = 5.0


def main(run, gt_dir, n_samples=1500, top_k=6):
    n_samples, top_k = int(n_samples), int(top_k)
    events = [json.loads(line) for line in open(f"{run}/events.jsonl")]
    kf_stamp = {(e["robot"], e["kf"]): e["stamp_ns"] for e in events if e["type"] == "keyframe"}
    seqs = run_sequences(run)
    stamp_map = {r: read_stamp_map(run, r) for r in range(3)}
    gt = {r: read_gt(f"{gt_dir}/{seqs[r]}.csv", body_to_cam0(gt_dir, seqs[r])) for r in range(3)}

    pos = {}
    for (r, kf), s in kf_stamp.items():
        if s in stamp_map[r]:
            G = nearest(gt[r], stamp_map[r][s])
            if G is not None:
                pos[(r, kf)] = G[:3, 3]
    keys = sorted(pos)
    rng = random.Random(0)
    scored, tried = [], set()
    while len(tried) < n_samples:
        (a, ka), (b, kb) = rng.sample(keys, 2)
        if a == b or (a, ka, b, kb) in tried or np.linalg.norm(pos[(a, ka)] - pos[(b, kb)]) < MIN_GT_DIST_M:
            continue
        tried.add((a, ka, b, kb))
        from scripts.build_pointcloud import read_frame
        ia = read_frame(f"{run}/frames/r{a}/{kf_stamp[(a, ka)]}")
        ib = read_frame(f"{run}/frames/r{b}/{kf_stamp[(b, kb)]}")
        if ia is None or ib is None:
            continue
        res = verify_pair(ia, ib)
        scored.append((res["inliers"], a, ka, b, kb, float(np.linalg.norm(pos[(a, ka)] - pos[(b, kb)]))))
    scored.sort(reverse=True)
    inl = np.array([s[0] for s in scored])
    print(f"{len(scored)} far-apart pairs (>{MIN_GT_DIST_M} m in ground truth): inliers median {np.median(inl):.0f}, "
          f"p99 {np.percentile(inl, 99):.0f}, max {inl.max()}, >=50: {(inl >= 50).sum()}")
    t_last = events[-1]["t"]
    with open(f"{run}/injected_false_lc.jsonl", "w") as f:
        for i, (n, a, ka, b, kb, d) in enumerate(scored[:top_k]):
            print(f"  camera {a} kf {ka} <-> camera {b} kf {kb}: {d:.1f} m apart, {n} inliers")
            f.write(json.dumps({"t": t_last + 1 + i, "type": "lc", "a": a, "ka": ka, "b": b, "kb": kb, "success": True,
                                "translation": [0, 0, 0], "injected": "false_closure", "gt_dist_m": round(d, 2)}) + "\n")


if __name__ == "__main__":
    main(*sys.argv[1:])
