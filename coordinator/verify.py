"""OpenCV 5 re-verification of a Swarm-SLAM inter-robot loop closure from the two keyframe images.

Swarm-SLAM already checks closures with RTAB-Map PnP; this is an independent check the
coordinator can cite: ORB + ratio test, then a MAGSAC fundamental matrix.
"""
import cv2
import numpy as np

ORB = cv2.ORB_create(nfeatures=1500)
MATCHER = cv2.BFMatcher(cv2.NORM_HAMMING)


def verify_pair(img_a, img_b, ratio=0.8):
    ka, da = ORB.detectAndCompute(img_a, None)
    kb, db = ORB.detectAndCompute(img_b, None)
    if da is None or db is None or len(ka) < 8 or len(kb) < 8:
        return {"matches": 0, "inliers": 0, "inlier_ratio": 0.0}
    good = [m for m, n in (p for p in MATCHER.knnMatch(da, db, k=2) if len(p) == 2) if m.distance < ratio * n.distance]
    if len(good) < 8:
        return {"matches": len(good), "inliers": 0, "inlier_ratio": 0.0}
    pa = np.float32([ka[m.queryIdx].pt for m in good])
    pb = np.float32([kb[m.trainIdx].pt for m in good])
    _, mask = cv2.findFundamentalMat(pa, pb, cv2.USAC_MAGSAC, 1.0, 0.999, 10000)
    inliers = 0 if mask is None else int(mask.sum())
    return {"matches": len(good), "inliers": inliers, "inlier_ratio": round(inliers / len(good), 3)}
