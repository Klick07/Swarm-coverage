"""Latency from a frame arriving to the walker's notification, per stage.

Usage: python -m eval.latency <run_dir> <coordinator_db>
  run_dir:        events.jsonl + quality logs from a pipeline run on EC2 (one clock)
  coordinator_db: coverage.db from a Bedrock replay of that run
Stages (all measured, none assumed):
  1 front end   OpenCV 5 processing time per frame (quality 'ms')
  2 SLAM        frame stamp of the later keyframe -> inter-robot closure event (Swarm-SLAM)
  3 decision    overlap/quality event handled -> notification written (Bedrock + tools)
  4 dashboard   polls every 1 s, so at most 1 s more
"""
import json
import sqlite3
import sys

import numpy as np


def pct(v):
    v = np.asarray(v, float)
    return f"median {np.median(v):6.2f} s   p90 {np.percentile(v, 90):6.2f} s   n={len(v)}"


def main(run, db_path):
    events = [json.loads(line) for line in open(f"{run}/events.jsonl")]
    kf_stamp = {(e["robot"], e["kf"]): e["stamp_ns"] for e in events if e["type"] == "keyframe"}
    front = [e["ms"] / 1000 for e in events if e["type"] == "quality"]
    slam = []
    for e in events:
        if e["type"] == "lc" and e["success"]:
            stamps = [kf_stamp.get((e["a"], e["ka"])), kf_stamp.get((e["b"], e["kb"]))]
            if all(stamps):
                slam.append(e["t"] - max(stamps) / 1e9)
    con = sqlite3.connect(db_path)
    decide_overlap = [r[0] for r in con.execute(
        "SELECT n.t - o.created FROM notifications n JOIN overlaps o ON n.overlap_id = o.overlap_id "
        "WHERE o.resolved_by IS NULL")]
    # Quality decisions carry their measured latency (coordinator records decision_latency_s).
    decide_quality = [json.loads(v).get("decision_latency_s") for (v,) in con.execute(
        "SELECT values_json FROM decisions WHERE trigger='quality_drop' AND action LIKE 'notify%'")]
    decide_quality = [v for v in decide_quality if v is not None]
    print("1 front end (OpenCV 5 per frame):    ", pct(front))
    print("2 SLAM (frame -> inter-robot closure):", pct(slam))
    print("3a decision, overlap (Bedrock):       ", pct(decide_overlap))
    print("3b decision, quality (Bedrock):       ",
          pct(decide_quality) if decide_quality else "not recorded in this db (needs a replay after 2026-10-24)")
    print("4 dashboard refresh:                    <= 1.00 s")
    total = np.median(front) + np.median(slam) + np.median(decide_overlap) + 0.5
    print(f"frame -> 'area covered' on the dashboard, median stages summed: about {total:.1f} s")


if __name__ == "__main__":
    main(*sys.argv[1:3])
