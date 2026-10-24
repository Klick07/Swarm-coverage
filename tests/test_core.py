import cv2
import numpy as np
import pytest

from frontend.core import QualityScorer, StereoRectifier, load_calib, passes_gate, save_calib

W, H = 752, 480


def make_calib():
    """A deliberately imperfect stereo rig: distortion plus a small relative rotation."""
    K = np.array([[458.0, 0, 367.0], [0, 457.0, 248.0], [0, 0, 1]])
    D = np.array([-0.28, 0.07, 0.0002, 0.00002, 0.0])
    R, _ = cv2.Rodrigues(np.array([0.01, -0.02, 0.005]))
    T = np.array([[-0.11], [0.001], [0.0005]])
    return {"K1": K, "D1": D, "K2": K.copy(), "D2": D.copy(), "R": R, "T": T,
            "image_size": (W, H), "model": "pinhole"}


def textured(seed=0):
    rng = np.random.default_rng(seed)
    img = (rng.random((H // 8, W // 8)) * 255).astype(np.uint8)
    return cv2.resize(img, (W, H), interpolation=cv2.INTER_NEAREST)


def test_rectified_epipolar_lines_are_horizontal():
    calib = make_calib()
    rect = StereoRectifier(calib)
    rng = np.random.default_rng(1)
    pts3d = np.column_stack([rng.uniform(-2, 2, 200), rng.uniform(-1.5, 1.5, 200), rng.uniform(2, 8, 200)])
    left, _ = cv2.projectPoints(pts3d, np.zeros(3), np.zeros(3), calib["K1"], calib["D1"])
    rvec, _ = cv2.Rodrigues(calib["R"])
    right, _ = cv2.projectPoints(pts3d, rvec, calib["T"], calib["K2"], calib["D2"])
    rl = cv2.undistortPoints(left, calib["K1"], calib["D1"], R=rect.R1, P=rect.P1).reshape(-1, 2)
    rr = cv2.undistortPoints(right, calib["K2"], calib["D2"], R=rect.R2, P=rect.P2).reshape(-1, 2)
    assert np.abs(rl[:, 1] - rr[:, 1]).max() < 0.5  # same row after rectification
    assert (rl[:, 0] - rr[:, 0] > 0).all()           # positive disparity
    assert rect.baseline == pytest.approx(0.11, abs=0.002)


def test_rectify_keeps_image_shape():
    rect = StereoRectifier(make_calib())
    l, r = rect.rectify(textured(0), textured(1))
    assert l.shape == r.shape == (H, W)


def test_blurred_frame_scores_lower():
    sharp = textured(2)
    blurred = cv2.GaussianBlur(sharp, (15, 15), 5)
    s_sharp = QualityScorer().score(sharp, sharp)
    s_blur = QualityScorer().score(blurred, blurred)
    assert s_blur["blur"] < s_sharp["blur"] / 5
    assert s_blur["features"] < s_sharp["features"]


def test_stereo_inliers_detect_shifted_pair():
    img = textured(3)
    shifted = np.roll(img, -20, axis=1)        # pure horizontal disparity
    vertical = np.roll(img, 15, axis=0)        # breaks the same-row rule
    good = QualityScorer().score(img, shifted)["stereo_inliers"]
    bad = QualityScorer().score(img, vertical)["stereo_inliers"]
    assert good > 0.8 and bad < 0.2


def test_track_inliers_none_on_first_frame_then_high():
    q = QualityScorer()
    img = textured(4)
    assert q.score(img, img)["track_inliers"] is None
    assert q.score(np.roll(img, 5, axis=1), img)["track_inliers"] > 0.8


def test_gate():
    assert passes_gate({"blur": 100, "features": 300}, min_blur=50, min_features=100)
    assert not passes_gate({"blur": 10, "features": 300}, min_blur=50, min_features=100)


def test_calib_roundtrip(tmp_path):
    calib = make_calib()
    p = tmp_path / "cam0.yaml"
    save_calib(p, calib)
    back = load_calib(p)
    for k in ("K1", "D1", "R"):
        assert np.allclose(back[k], calib[k])
    assert back["T"].shape == (3, 1) and back["image_size"] == (W, H)
