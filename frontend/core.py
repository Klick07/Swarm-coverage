"""OpenCV 5 stereo front end: rectification, quality scores, geometric verification.

No ROS imports, so this module is unit-testable on any machine with OpenCV 5.
"""
import cv2
import numpy as np
import yaml

CALIB_KEYS = ("K1", "D1", "K2", "D2", "R", "T")


def load_calib(path):
    """Read calib/camN.yaml: per-lens intrinsics and the cam0->cam1 extrinsic."""
    with open(path) as f:
        raw = yaml.safe_load(f)
    calib = {k: np.array(raw[k], dtype=np.float64) for k in CALIB_KEYS}
    calib["T"] = calib["T"].reshape(3, 1)
    calib["image_size"] = tuple(int(v) for v in raw["image_size"])  # (width, height)
    calib["model"] = raw.get("model", "pinhole")
    return calib


def save_calib(path, calib, extra=None):
    out = {k: np.asarray(calib[k]).tolist() for k in CALIB_KEYS}
    out["image_size"] = list(calib["image_size"])
    out["model"] = calib.get("model", "pinhole")
    out.update(extra or {})
    with open(path, "w") as f:
        yaml.safe_dump(out, f, sort_keys=False)


class StereoRectifier:
    """Precomputes rectification maps once; rectify() is then two cv2.remap calls."""

    def __init__(self, calib):
        size = calib["image_size"]
        args = (calib["K1"], calib["D1"], calib["K2"], calib["D2"], size, calib["R"], calib["T"])
        if calib["model"] == "fisheye":
            R1, R2, P1, P2, _ = cv2.fisheye.stereoRectify(*args, flags=cv2.CALIB_ZERO_DISPARITY)
            init = cv2.fisheye.initUndistortRectifyMap
        else:
            R1, R2, P1, P2, _, _, _ = cv2.stereoRectify(*args, flags=cv2.CALIB_ZERO_DISPARITY, alpha=0)
            init = cv2.initUndistortRectifyMap
        self.maps = (init(calib["K1"], calib["D1"], R1, P1, size, cv2.CV_16SC2),
                     init(calib["K2"], calib["D2"], R2, P2, size, cv2.CV_16SC2))
        self.R1, self.R2, self.P1, self.P2, self.size = R1, R2, P1, P2, size
        self.baseline = -P2[0, 3] / P2[0, 0]

    def rectify(self, left, right):
        (l1, l2), (r1, r2) = self.maps
        return (cv2.remap(left, l1, l2, cv2.INTER_LINEAR),
                cv2.remap(right, r1, r2, cv2.INTER_LINEAR))


class QualityScorer:
    """Per-frame scores the coordinator acts on.

    blur:            variance of the Laplacian of the left image (higher = sharper)
    features:        ORB keypoints in the left image
    stereo_inliers:  share of left-right ORB matches that sit on the same row
                     with positive disparity (geometric check of rectification)
    track_inliers:   share of matches to the previous frame that are inliers of
                     a RANSAC fundamental matrix (geometric check of tracking)
    """

    def __init__(self, n_features=1000, row_tol_px=2.0, max_disparity_px=200.0):
        self.orb = cv2.ORB_create(nfeatures=n_features)
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        self.row_tol = row_tol_px
        self.max_disp = max_disparity_px
        self.prev = None  # (keypoints, descriptors) of the previous left image

    def score(self, left, right):
        blur = float(cv2.Laplacian(left, cv2.CV_64F).var())
        kl, dl = self.orb.detectAndCompute(left, None)
        kr, dr = self.orb.detectAndCompute(right, None)
        stereo = self._stereo_inliers(kl, dl, kr, dr)
        track = self._track_inliers(kl, dl)
        self.prev = (kl, dl)
        return {"blur": round(blur, 2), "features": len(kl),
                "stereo_inliers": stereo, "track_inliers": track}

    def _stereo_inliers(self, kl, dl, kr, dr):
        if dl is None or dr is None:
            return 0.0
        matches = self.matcher.match(dl, dr)
        if not matches:
            return 0.0
        ok = 0
        for m in matches:
            (xl, yl), (xr, yr) = kl[m.queryIdx].pt, kr[m.trainIdx].pt
            if abs(yl - yr) <= self.row_tol and 0.0 < xl - xr <= self.max_disp:
                ok += 1
        return round(ok / len(matches), 3)

    def _track_inliers(self, kl, dl):
        if self.prev is None or dl is None or self.prev[1] is None:
            return None  # first frame: nothing to track against
        kp, dp = self.prev
        matches = self.matcher.match(dp, dl)
        if len(matches) < 8:
            return 0.0
        p0 = np.float32([kp[m.queryIdx].pt for m in matches])
        p1 = np.float32([kl[m.trainIdx].pt for m in matches])
        _, mask = cv2.findFundamentalMat(p0, p1, cv2.FM_RANSAC, 1.0, 0.999)
        return 0.0 if mask is None else round(float(mask.sum()) / len(matches), 3)


def passes_gate(scores, min_blur, min_features):
    """Frames failing this are dropped before Swarm-SLAM sees them."""
    return scores["blur"] >= min_blur and scores["features"] >= min_features
