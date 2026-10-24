"""OpenCV 5 re-verification of a Swarm-SLAM inter-robot loop closure from the two keyframe images.

Swarm-SLAM already checks closures with RTAB-Map PnP; this is an independent check the
coordinator can cite: ORB + ratio test, then a MAGSAC fundamental matrix.
"""
import cv2
import numpy as np

ORB = cv2.ORB_create(nfeatures=1500)
MATCHER = cv2.BFMatcher(cv2.NORM_HAMMING)


def verify_pair(img_a, img_b, ratio=0.8, draw_path=None):
    """Returns match/inlier counts; with draw_path, also saves both images side by side
    with the inlier matches drawn, which is what a human approver looks at."""
    ka, da = ORB.detectAndCompute(img_a, None)
    kb, db = ORB.detectAndCompute(img_b, None)
    good, inlier_matches = [], []
    res = {"matches": 0, "inliers": 0, "inlier_ratio": 0.0}
    if da is not None and db is not None and len(ka) >= 8 and len(kb) >= 8:
        good = [m for m, n in (p for p in MATCHER.knnMatch(da, db, k=2) if len(p) == 2) if m.distance < ratio * n.distance]
        res["matches"] = len(good)
    if len(good) >= 8:
        pa = np.float32([ka[m.queryIdx].pt for m in good])
        pb = np.float32([kb[m.trainIdx].pt for m in good])
        # Fixed seed: MAGSAC samples randomly, and the same pair once gave 23 inliers / 2.4% spread
        # on one run and 26 / 11% on another. Decisions in the log must be reproducible.
        cv2.setRNGSeed(0)
        _, mask = cv2.findFundamentalMat(pa, pb, cv2.USAC_MAGSAC, 1.0, 0.999, 10000)
        if mask is not None:
            inlier_matches = [m for m, keep in zip(good, mask.ravel()) if keep]
        res.update(inliers=len(inlier_matches), inlier_ratio=round(len(inlier_matches) / len(good), 3))
        if len(inlier_matches) >= 3:
            # Share of each image covered by the inliers' convex hull (the smaller of the two).
            # Low = the matches crowd into one patch, e.g. the same object seen from 5+ m away.
            keep = mask.ravel().astype(bool)
            area = img_a.shape[0] * img_a.shape[1]
            res["spread"] = round(min(cv2.contourArea(cv2.convexHull(p[keep])) / area for p in (pa, pb)), 3)
    res.setdefault("spread", 0.0)
    if draw_path:
        vis = cv2.drawMatches(img_a, ka, img_b, kb, inlier_matches, None, matchColor=(80, 220, 120),
                              flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
        cv2.imwrite(draw_path, cv2.resize(vis, None, fx=0.6, fy=0.6, interpolation=cv2.INTER_AREA),
                    [cv2.IMWRITE_JPEG_QUALITY, 85])
    return res
