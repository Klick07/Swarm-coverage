"""Evidence for the failure-case inputs: front-end scores on clean vs perturbed frames.

Usage: python scripts/check_perturb.py <seq>/mav0 calib/euroc_camN.yaml [out_dir]
"""
import sys

import cv2
import numpy as np

from frontend.core import QualityScorer, StereoRectifier, load_calib
from frontend.euroc_replay import perturb, read_frames


def main(mav0, calib, out_dir=None):
    frames = read_frames(mav0)
    rect = StereoRectifier(load_calib(calib))
    rng = np.random.default_rng(0)
    for kind in ("none", "blank", "shake"):
        q, rows = QualityScorer(), []
        for i, (_, fl, fr) in enumerate(frames[600:640:2]):
            left = cv2.imread(f"{mav0}/cam0/data/{fl}", cv2.IMREAD_GRAYSCALE)
            right = cv2.imread(f"{mav0}/cam1/data/{fr}", cv2.IMREAD_GRAYSCALE)
            if kind != "none":
                left, right = perturb(kind, left, right, rng)
            rl, rr = rect.rectify(left, right)
            rows.append(q.score(rl, rr))
            if out_dir and i == 5:
                cv2.imwrite(f"{out_dir}/perturb_{kind}.jpg", rl)

        def med(k):
            return np.median([r[k] for r in rows if r[k] is not None])
        print(f"{kind:6s} blur {med('blur'):8.1f}  features {med('features'):6.0f}  "
              f"stereo_inliers {med('stereo_inliers'):.2f}  track_inliers {med('track_inliers'):.2f}")


if __name__ == "__main__":
    main(*sys.argv[1:4])
