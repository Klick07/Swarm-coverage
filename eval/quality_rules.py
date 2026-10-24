"""Compare weak-tracking rules across environments: warnings inside injected failure windows
(should fire) vs everywhere else (should stay quiet).

Usage: python -m eval.quality_rules <run_dir> [<run_dir> ...]
Simulates the coordinator's 6-frame median window and 20 s cooldown on each run's quality events.
"""
import csv
import json
import statistics
import sys
from collections import deque

WINDOW, COOLDOWN_S = 6, 20


def rules():
    # name -> fn(window_medians, baseline_blur) -> weak?
    return {
        "absolute (blur<90 or track<0.5)": lambda m, base: m["blur"] < 90 or m["track"] < 0.5,
        "track only (<0.5)": lambda m, base: m["track"] < 0.5,
        "relative (blur<0.35x own median or track<0.5)": lambda m, base: m["blur"] < 0.35 * base or m["track"] < 0.5,
        "relative (blur<0.25x own median or track<0.5)": lambda m, base: m["blur"] < 0.25 * base or m["track"] < 0.5,
    }


def windows(run, r):
    try:
        rows = list(csv.DictReader(open(f"{run}/stamps_r{r}.csv")))
    except FileNotFoundError:
        return None
    w = [int(x["ros_stamp_ns"]) / 1e9 for x in rows if x.get("perturbation")]
    return (w[0], w[-1] + 3) if w else None


def simulate(run, rule):
    events = [json.loads(line) for line in open(f"{run}/events.jsonl")]
    inside = outside = 0
    hit_windows = set()
    for r in range(3):
        q = [e for e in events if e["type"] == "quality" and e["robot"] == r]
        win = windows(run, r)
        recent, history, last = deque(maxlen=WINDOW), deque(maxlen=150), -1e9
        for e in q:
            recent.append(e)
            if e["track_inliers"] is None and e["features"] > 0:
                continue
            history.append(e["blur"])
            if len(recent) < WINDOW or len(history) < 30:
                continue
            tr = [x["track_inliers"] for x in recent if x["track_inliers"] is not None]
            m = {"blur": statistics.median(x["blur"] for x in recent),
                 "track": statistics.median(tr) if tr else 0.0}
            base = statistics.median(history)
            t = e["stamp_ns"] / 1e9
            if rule(m, base) and t - last >= COOLDOWN_S:
                last = t
                if win and win[0] <= t <= win[1]:
                    inside += 1
                    hit_windows.add(r)
                else:
                    outside += 1
    n_windows = sum(1 for r in range(3) if windows(run, r))
    return inside, outside, len(hit_windows), n_windows


def main(*runs):
    for name, rule in rules().items():
        parts = []
        for run in runs:
            ins, out, hit, nwin = simulate(run, rule)
            parts.append(f"{run.split('/')[-1]}: outside {out:2d}" + (f", failures caught {hit}/{nwin}" if nwin else ""))
        print(f"{name:48s} | " + " | ".join(parts))


if __name__ == "__main__":
    main(*sys.argv[1:])
