"""Uploaded footage: store it, then run the OpenCV 5 front end's quality scoring over it.

A single uploaded video has no stereo partner and no calibration, so it cannot enter Swarm-SLAM
yet. What it gets is the per-frame quality analysis the coordinator acts on: tracking inliers,
ORB features and blur over the whole clip, with the weak segments marked by the coordinator's rule.
"""
import json
import os
import time

import cv2
import numpy as np

from frontend.core import QualityScorer

MAX_SAMPLES = 400          # frames scored per video (evenly spread)
SCORE_WIDTH = 960          # downscale wider frames: thresholds were tuned at 752x480
WEAK_TRACK, MIN_FEATURES = 0.50, 100   # same rule as coordinator/main.py
MIN_SEGMENT_S = 1.0


def write_meta(out_dir, **fields):
    path = f"{out_dir}/meta.json"
    meta = json.load(open(path)) if os.path.exists(path) else {}
    meta.update(fields)
    with open(path + ".tmp", "w") as f:
        json.dump(meta, f)
    os.replace(path + ".tmp", path)  # atomic: the dashboard may be reading it
    return meta


def analyze_video(video_path, out_dir):
    """Fill out_dir/meta.json (status, video info, quality summary, weak segments) and thumb.jpg."""
    t0 = time.time()
    write_meta(out_dir, status="processing", opencv=cv2.__version__)
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        return write_meta(out_dir, status="failed", error="OpenCV could not open this file (unsupported codec?)")
    fps = cap.get(cv2.CAP_PROP_FPS) or 0.0
    n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT) or 0)
    w, h = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    if fps <= 0 or n <= 0:
        return write_meta(out_dir, status="failed", error="no frames or no frame rate in this file")
    step = max(1, n // MAX_SAMPLES)
    write_meta(out_dir, fps=round(fps, 2), frames=n, width=w, height=h, duration_s=round(n / fps, 2),
               sample_every=step)

    scorer, rows, idx = QualityScorer(), [], 0
    thumb_at = n // 3
    while True:
        ok = cap.grab()
        if not ok:
            break
        if idx % step == 0:
            ok, frame = cap.retrieve()
            if not ok:
                break
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
            if gray.shape[1] > SCORE_WIDTH:
                gray = cv2.resize(gray, (SCORE_WIDTH, int(gray.shape[0] * SCORE_WIDTH / gray.shape[1])),
                                  interpolation=cv2.INTER_AREA)
            s = scorer.score(gray, gray)  # mono: the stereo check is meaningless and is dropped below
            weak = s["features"] < MIN_FEATURES or (s["track_inliers"] is not None and s["track_inliers"] < WEAK_TRACK)
            rows.append({"t": round(idx / fps, 2), "blur": s["blur"], "features": s["features"],
                         "track": s["track_inliers"], "weak": weak})
            if idx >= thumb_at and not os.path.exists(f"{out_dir}/thumb.jpg"):
                cv2.imwrite(f"{out_dir}/thumb.jpg", cv2.resize(frame, (480, int(480 * h / max(w, 1)))),
                            [cv2.IMWRITE_JPEG_QUALITY, 80])
            if len(rows) % 25 == 0:
                write_meta(out_dir, progress=round(idx / n, 2))
        idx += 1
    cap.release()
    if not rows:
        return write_meta(out_dir, status="failed", error="no frame could be decoded")

    def med(k):
        v = [r[k] for r in rows if r[k] is not None]
        return round(float(np.median(v)), 3) if v else None

    segments, start = [], None   # contiguous weak stretches of at least MIN_SEGMENT_S
    # The sentinel sits one sample after the last frame, so a stretch running to the end of the
    # clip keeps its full length.
    for r in rows + [{"t": rows[-1]["t"] + step / fps, "weak": False}]:
        if r["weak"] and start is None:
            start = r["t"]
        elif not r["weak"] and start is not None:
            if r["t"] - start >= MIN_SEGMENT_S:
                segments.append([round(start, 2), round(r["t"], 2)])
            start = None
    return write_meta(out_dir, status="done", progress=1.0, samples=len(rows),
                      median_blur=med("blur"), median_features=med("features"), median_track=med("track"),
                      weak_share=round(sum(r["weak"] for r in rows) / len(rows), 3),
                      weak_segments=segments, timeline=rows, analysis_s=round(time.time() - t0, 1))
