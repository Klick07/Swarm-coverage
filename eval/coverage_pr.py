"""Precision / recall of "area covered" decisions against EuRoC ground truth.

Usage: python -m eval.coverage_pr <run_dir> <gt_dir> [coverage_db]
Precision: a covered claim (overlap applied) is correct if the two keyframes were within
           TRUE_DIST_M of each other in ground truth.
Recall:    ground-truth 2 m floor cells (x, y in the Leica frame) visited by >= 2 cameras,
           and the share of them that received a correct covered claim.
"""
import json
import sqlite3
import sys

import numpy as np

from eval.cslam_eval import body_to_cam0, nearest, read_gt, read_stamp_map, run_sequences

TRUE_DIST_M = 3.0
CELL_M = 2.0


def main(run, gt_dir, db=None):
    db = db or f"{run}/coverage.db"
    events = [json.loads(line) for line in open(f"{run}/events.jsonl")]
    kf_stamp = {(e["robot"], e["kf"]): e["stamp_ns"] for e in events if e["type"] == "keyframe"}
    seqs = run_sequences(run)
    stamp_map = {r: read_stamp_map(run, r) for r in range(3)}
    gt = {r: read_gt(f"{gt_dir}/{seqs[r]}.csv", body_to_cam0(gt_dir, seqs[r])) for r in range(3)}

    def gt_pos(r, kf):
        s = kf_stamp.get((r, kf))
        G = nearest(gt[r], stamp_map[r][s]) if s in stamp_map[r] else None
        return None if G is None else G[:3, 3]

    cell = lambda p: (int(p[0] // CELL_M), int(p[1] // CELL_M))
    visits = {}
    for (r, kf) in kf_stamp:
        p = gt_pos(r, kf)
        if p is not None:
            visits.setdefault(cell(p), set()).add(r)
    co_visited = {c for c, robots in visits.items() if len(robots) >= 2}

    con = sqlite3.connect(db)
    claims = con.execute("SELECT overlap_id, a, ka, b, kb, resolved_by FROM overlaps WHERE state='applied'").fetchall()
    correct, wrong, hit_cells = 0, [], set()
    for oid, a, ka, b, kb, by in claims:
        pa, pb = gt_pos(a, ka), gt_pos(b, kb)
        if pa is None or pb is None:
            continue
        d = float(np.linalg.norm(pa - pb))
        if d <= TRUE_DIST_M:
            correct += 1
            hit_cells.add(cell(pb))
        else:
            wrong.append((oid, a, b, round(d, 1), by or "auto"))
    n = correct + len(wrong)
    print(f"claims with ground truth: {n}   correct {correct}   wrong {len(wrong)} {wrong}")
    print(f"precision {correct / n:.3f}" if n else "precision n/a")
    print(f"ground-truth cells visited by >=2 cameras: {len(co_visited)}   with a correct claim: "
          f"{len(hit_cells & co_visited)}   recall {len(hit_cells & co_visited) / len(co_visited):.3f}")


if __name__ == "__main__":
    main(*sys.argv[1:4])
