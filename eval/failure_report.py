"""Failure-case timeline: injected window -> coordinator warnings -> SLAM loss and recovery.

Usage: python -m eval.failure_report <run_dir> <coverage_db>
Times are relative to the start of each robot's injected window, in the EC2 run's clock
(replay frame stamps), and in EuRoC data time (wall x rate; the replay ran at quarter speed).
Recovery = first Swarm-SLAM keyframe after the longest keyframe gap that starts in the window.
"""
import csv
import json
import re
import sqlite3
import sys


def main(run, db):
    rate = float(re.search(r"rate=([\d.]+)", open(f"{run}/params.txt").read()).group(1))
    log = open(f"{run}/launch.log").read()
    lost = {}
    for t, r in re.findall(r"\[WARN\] \[(\d+\.\d+)\] \[r(\d)\.cslam_map_manager\]: Odom tracking failed", log):
        lost.setdefault(int(r), []).append(float(t))
    events = [json.loads(line) for line in open(f"{run}/events.jsonl")]
    con = sqlite3.connect(db)
    alerts = {}
    for cam, trig, v in con.execute("SELECT camera, trigger, values_json FROM decisions WHERE action LIKE 'notify%' "
                                    "AND trigger IN ('quality_drop', 'slam_tracking_lost')"):
        alerts.setdefault((cam, trig), []).append(json.loads(v))

    for r in range(3):
        with open(f"{run}/stamps_r{r}.csv") as f:
            rows = list(csv.DictReader(f))
        win = [int(x["ros_stamp_ns"]) / 1e9 for x in rows if x.get("perturbation")]
        kind = next((x["perturbation"] for x in rows if x.get("perturbation")), "none")
        kfs = sorted(e["stamp_ns"] / 1e9 for e in events if e["type"] == "keyframe" and e["robot"] == r)
        print(f"robot {r} ({kind}): keyframes {len(kfs)}, odometry-lost frames {len(lost.get(r, []))}, "
              f"quality warnings {len(alerts.get((r, 'quality_drop'), []))}, "
              f"tracking-lost alerts {len(alerts.get((r, 'slam_tracking_lost'), []))}")
        if not win:
            continue
        t0, t1 = win[0], win[-1]
        rel = lambda t: f"{t - t0:+7.1f} s wall ({(t - t0) * rate:+6.1f} s data)"
        print(f"   injected window            0.0 .. {t1 - t0:.1f} s wall ({(t1 - t0) * rate:.1f} s data)")
        q = [a["frame_stamp_ns"] / 1e9 for a in alerts.get((r, "quality_drop"), []) if a.get("frame_stamp_ns")]
        first_q = min((t for t in q if t >= t0), default=None)
        print(f"   first 'slow down' warning  {rel(first_q) if first_q else 'none'}")
        # Keyframes lag the frames they come from, so the gap can start a few seconds before t0.
        gaps = [(b - a, a, b) for a, b in zip(kfs, kfs[1:]) if t0 - 15 <= a <= t1]
        if gaps:
            _, a, b = max(gaps)
            print(f"   last keyframe before gap   {rel(a)}")
            print(f"   SLAM recovered (keyframe)  {rel(b)}   -> {b - t1:.1f} s wall after the window ended")
        else:
            print("   no keyframe gap in the window")


if __name__ == "__main__":
    main(*sys.argv[1:3])
