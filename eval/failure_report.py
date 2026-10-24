"""Failure-case timeline: injected window -> coordinator warning -> odometry loss and recovery.

Usage: python -m eval.failure_report <run_dir> <coverage_db>
All times are in the EC2 run's clock (replay frame stamps and ROS log stamps), in
seconds relative to the start of each robot's injected window, and also in EuRoC
data time (wall / rate) because the replay ran at quarter speed.
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
    for r, t in re.findall(r"\[r(\d)\.cslam_map_manager\]: Odom tracking failed", log) and \
            re.findall(r"\[WARN\] \[(\d+\.\d+)\] \[r(\d)\.cslam_map_manager\]: Odom tracking failed", log) or []:
        pass
    for t, r in re.findall(r"\[WARN\] \[(\d+\.\d+)\] \[r(\d)\.cslam_map_manager\]: Odom tracking failed", log):
        lost.setdefault(int(r), []).append(float(t))
    con = sqlite3.connect(db)
    warns = {}
    for cam, v in con.execute("SELECT camera, values_json FROM decisions WHERE trigger='quality_drop' AND action LIKE 'notify%'"):
        s = json.loads(v).get("frame_stamp_ns")
        if s:
            warns.setdefault(cam, []).append(s / 1e9)

    for r in range(3):
        with open(f"{run}/stamps_r{r}.csv") as f:
            rows = list(csv.DictReader(f))
        win = [int(x["ros_stamp_ns"]) / 1e9 for x in rows if x.get("perturbation")]
        kind = next((x["perturbation"] for x in rows if x.get("perturbation")), "none")
        L = sorted(lost.get(r, []))
        W = sorted(warns.get(r, []))
        print(f"robot {r} ({kind}): {len(win)} injected frames, odometry-lost frames {len(L)}, warnings {len(W)}")
        if not win:
            continue
        t0, t1 = win[0], win[-1]
        rel = lambda t: f"{t - t0:+7.1f} s wall ({(t - t0) * rate:+6.1f} s data)"
        print(f"   injected window          0.0 .. {t1 - t0:.1f} s wall ({(t1 - t0) * rate:.1f} s data)")
        first_w = next((w for w in W if w >= t0), None)
        print(f"   first warning in window  {rel(first_w) if first_w else 'none'}")
        in_l = [t for t in L if t >= t0 - 1]
        if in_l:
            # recovery = end of the lost run that starts in the window (gaps > 2 s split runs)
            end = in_l[0]
            for t in in_l[1:]:
                if t - end > 2:
                    break
                end = t
            print(f"   odometry lost from       {rel(in_l[0])}")
            print(f"   odometry recovered after {rel(end)}  (window ended at {t1 - t0:.1f} s wall)")
        print(f"   warnings outside window  {sum(1 for w in W if not (t0 <= w <= t1 + 3))}")


if __name__ == "__main__":
    main(*sys.argv[1:3])
