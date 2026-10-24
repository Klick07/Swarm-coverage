import json

import cv2
import numpy as np

from coordinator.main import LOST_FRAMES, MIN_INLIERS, WEAK_BLUR, WEAK_TRACK, WINDOW, Coordinator


def make_run(tmp_path, same_view):
    """A tiny run dir: two keyframe images (same view or unrelated) and their keyframe events."""
    rng = np.random.default_rng(0)
    base = cv2.resize((rng.random((60, 94)) * 255).astype(np.uint8), (752, 480), interpolation=cv2.INTER_NEAREST)
    base = cv2.GaussianBlur(base, (3, 3), 0)
    other = np.roll(base, 12, axis=1) if same_view else cv2.GaussianBlur(
        cv2.resize((np.random.default_rng(9).random((60, 94)) * 255).astype(np.uint8), (752, 480),
                   interpolation=cv2.INTER_NEAREST), (3, 3), 0)
    for r, img in ((0, base), (1, other)):
        (tmp_path / "frames" / f"r{r}").mkdir(parents=True)
        cv2.imwrite(str(tmp_path / "frames" / f"r{r}" / f"{100 + r}.jpg"), img)
    c = Coordinator(str(tmp_path), use_bedrock=False)
    c.on_event({"type": "keyframe", "robot": 0, "kf": 5, "stamp_ns": 100, "odom": [0, 0, 0]})
    c.on_event({"type": "keyframe", "robot": 1, "kf": 7, "stamp_ns": 101, "odom": [0, 0, 0]})
    return c


def lc(c):
    c.on_event({"type": "lc", "a": 0, "ka": 5, "b": 1, "kb": 7, "success": True, "translation": [0, 0, 0]})
    return c.store.rows("SELECT * FROM overlaps")[0]


def test_verified_overlap_is_covered_and_second_camera_notified(tmp_path):
    c = make_run(tmp_path, same_view=True)
    o = lc(c)
    assert o["cv_inliers"] >= MIN_INLIERS and o["state"] == "applied"
    notes = c.store.rows("SELECT * FROM notifications")
    assert notes[0]["camera"] == 1 and notes[0]["message"] == "Area covered by camera 0, camera 1 move on."


def test_low_confidence_overlap_blocks_coverage_until_human(tmp_path):
    c = make_run(tmp_path, same_view=False)
    o = lc(c)
    assert o["confidence"] == "low" and o["state"] == "pending"
    assert c.mark_covered(o["overlap_id"])["refused"]                   # guardrail, whoever asks
    assert not c.store.rows("SELECT * FROM places WHERE status='covered'")
    c.store.set_overlap(o["overlap_id"], state="approved", resolved_by="human")  # dashboard answer
    c._apply_approvals()
    assert c.store.overlap(o["overlap_id"])["state"] == "applied"
    assert c.store.rows("SELECT * FROM places WHERE status='covered'")
    assert c.store.rows("SELECT source FROM decisions WHERE trigger='human_answer'")[0]["source"] == "human"


def test_swarm_slam_rejected_closure_is_logged_not_acted_on(tmp_path):
    c = make_run(tmp_path, same_view=True)
    c.on_event({"type": "lc", "a": 0, "ka": 5, "b": 1, "kb": 7, "success": False, "translation": [0, 0, 0]})
    assert not c.store.rows("SELECT * FROM overlaps")
    assert c.store.rows("SELECT action FROM decisions")[0]["action"] == "none"


def test_slam_lost_with_good_images_notifies_then_recovers(tmp_path):
    c = make_run(tmp_path, same_view=True)
    good = {"type": "quality", "robot": 0, "blur": 250.0, "features": 1000, "stereo_inliers": 0.9,
            "track_inliers": 0.8, "passed": True, "ms": 1}
    for i in range(LOST_FRAMES + 5):  # sharp frames keep coming, but no new keyframe
        c.on_event({**good, "stamp_ns": 200 + i})
    c.on_event({"type": "keyframe", "robot": 0, "kf": 6, "stamp_ns": 999, "odom": [0, 0, 0]})
    msgs = [n["message"] for n in c.store.rows("SELECT * FROM notifications WHERE camera=0 ORDER BY id")]
    assert len(msgs) == 2 and msgs[0].startswith("Camera 0, tracking lost") and "recovered" in msgs[1]


def test_quality_drop_notifies_once_per_cooldown(tmp_path):
    c = make_run(tmp_path, same_view=True)
    for i in range(WINDOW * 3):
        c.on_event({"type": "quality", "robot": 2, "stamp_ns": int(1e9 * (1 + i * 0.4)), "blur": 250.0,
                    "features": 900, "stereo_inliers": 0.9, "track_inliers": WEAK_TRACK / 2, "passed": True, "ms": 1})
    notes = c.store.rows("SELECT * FROM notifications")
    assert [n["message"] for n in notes] == ["Camera 2, slow down, tracking is weak."]
    values = json.loads(c.store.rows("SELECT values_json FROM decisions")[0]["values_json"])
    assert values["opencv"]["track_inliers"] < WEAK_TRACK            # the trace carries the OpenCV numbers


def test_low_blur_alone_does_not_warn(tmp_path):
    """A dark scene with low blur scores but healthy tracking (Vicon Room) must not nag walkers."""
    c = make_run(tmp_path, same_view=True)
    for i in range(WINDOW * 3):
        c.on_event({"type": "quality", "robot": 1, "stamp_ns": int(1e9 * (1 + i * 0.4)), "blur": WEAK_BLUR / 2,
                    "features": 900, "stereo_inliers": 0.9, "track_inliers": 0.7, "passed": True, "ms": 1})
    assert not c.store.rows("SELECT * FROM notifications")


def test_blank_wall_warns_without_track_score(tmp_path):
    c = make_run(tmp_path, same_view=True)
    for i in range(WINDOW * 2):  # blank wall: no features, so no tracking score at all
        c.on_event({"type": "quality", "robot": 1, "stamp_ns": int(1e9 * (1 + i * 0.4)), "blur": 20.0,
                    "features": 0, "stereo_inliers": 0.0, "track_inliers": None, "passed": True, "ms": 1})
    assert [n["message"] for n in c.store.rows("SELECT * FROM notifications")] == ["Camera 1, slow down, tracking is weak."]
