import json

import cv2
import numpy as np

from dashboard.uploads import analyze_video


def make_video(path, frames=60, fps=20, blank_from=40):
    """Textured frames drifting sideways, then a blank stretch (like pointing at a wall)."""
    rng = np.random.default_rng(0)
    base = cv2.resize((rng.random((60, 80)) * 255).astype(np.uint8), (640, 480), interpolation=cv2.INTER_NEAREST)
    out = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*"MJPG"), fps, (640, 480))
    for i in range(frames):
        g = np.full((480, 640), 140, np.uint8) if i >= blank_from else np.roll(base, 3 * i, axis=1)
        out.write(cv2.cvtColor(g, cv2.COLOR_GRAY2BGR))
    out.release()


def test_analyze_marks_blank_stretch_as_weak(tmp_path):
    video = tmp_path / "clip.avi"
    make_video(video)
    meta = analyze_video(str(video), str(tmp_path))
    assert meta["status"] == "done" and meta["frames"] == 60 and meta["width"] == 640
    assert abs(meta["duration_s"] - 3.0) < 0.01
    assert 0.2 < meta["weak_share"] < 0.5                  # the last third is blank
    assert meta["weak_segments"] and meta["weak_segments"][0][0] >= 1.9
    assert (tmp_path / "thumb.jpg").exists()
    assert json.load(open(tmp_path / "meta.json"))["status"] == "done"


def test_unreadable_file_fails_cleanly(tmp_path):
    bad = tmp_path / "not_a_video.mp4"
    bad.write_bytes(b"hello")
    assert analyze_video(str(bad), str(tmp_path))["status"] == "failed"
