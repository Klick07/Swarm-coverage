"""Evidence for the rectification step on real frames: score raw vs rectified pairs.

Usage: python scripts/check_rectification.py <seq>/mav0 calib/euroc_cam0.yaml [n_frames]
"""
import sys

import cv2
import numpy as np

from frontend.core import QualityScorer, StereoRectifier, load_calib
from frontend.euroc_replay import read_frames


def main(mav0, calib_path, n=60):
    rect = StereoRectifier(load_calib(calib_path))
    frames = read_frames(mav0)
    picks = frames[len(frames) // 4: len(frames) * 3 // 4: max(1, len(frames) // (2 * n))][:n]
    raw_scores, rect_scores, dy = [], [], []
    for _, fl, fr in picks:
        left = cv2.imread(f"{mav0}/cam0/data/{fl}", cv2.IMREAD_GRAYSCALE)
        right = cv2.imread(f"{mav0}/cam1/data/{fr}", cv2.IMREAD_GRAYSCALE)
        raw_scores.append(QualityScorer().score(left, right)["stereo_inliers"])
        rl, rr = rect.rectify(left, right)
        rect_scores.append(QualityScorer().score(rl, rr)["stereo_inliers"])
        orb, bf = cv2.ORB_create(1000), cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        kl, dl = orb.detectAndCompute(rl, None)
        kr, dr = orb.detectAndCompute(rr, None)
        ms = bf.match(dl, dr)
        p0 = np.float32([kl[m.queryIdx].pt for m in ms])
        p1 = np.float32([kr[m.trainIdx].pt for m in ms])
        _, mask = cv2.findFundamentalMat(p0, p1, cv2.FM_RANSAC, 1.0, 0.999)
        inl = mask.ravel().astype(bool)
        dy.append(np.median(np.abs(p0[inl, 1] - p1[inl, 1])))
    print(f"OpenCV {cv2.__version__}, {len(picks)} pairs from {mav0}")
    print(f"same-row match share  raw: {np.mean(raw_scores):.3f}   rectified: {np.mean(rect_scores):.3f}")
    print(f"median |dy| of RANSAC inliers after rectification: {np.median(dy):.3f} px")


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2], *(int(a) for a in sys.argv[3:4]))
