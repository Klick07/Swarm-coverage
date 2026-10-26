"""Stereo upload -> EuRoC-style robot folder that the Swarm-SLAM launch reads.

Input per robot: a left video, a right video, a stereo calibration (our calib yaml or a Kalibr
camchain yaml), and optionally one timestamp file per video. No ROS imports; runs anywhere with OpenCV.

Output <out>/mav0/cam0|cam1/data/<ns>.png + data.csv, <out>/calib.yaml and <out>/ingest_report.json.

Synchronisation is checked, never assumed:
  * with timestamp files: each left frame is paired with the nearest right frame; the pair is kept
    only if the skew is under a quarter frame period, and the skew statistics go in the report.
  * without them: frame counts and fps must agree, and the motion energy of the two videos is
    cross-correlated. A clear non-zero lag is an error. If the correlation is weak the report says
    "unverified" (equal fps and frame count are NOT proof of sync).
The left/right order is also checked with the rectified-pair row test; a clearly swapped pair is an error.
"""
import csv
import json
import os

import cv2
import numpy as np
import yaml

from frontend.core import QualityScorer, StereoRectifier, load_calib, save_calib

MAX_LAG = 15


class IngestError(ValueError):
    pass


# ---------- calibration ----------
def load_calibration(path, swap_eyes=False):
    """Our calib yaml (K1 D1 K2 D2 R T image_size model) or a Kalibr camchain (cam0, cam1).

    Kalibr: cam0 is taken as the LEFT lens, T_cn_cnm1 of cam1 maps cam0 points into cam1. Some rigs name
    the right lens cam0 (AirMuseum does): swap_eyes=True exchanges them and inverts the transform."""
    with open(path) as f:
        raw = yaml.safe_load(f)
    if "K1" in raw:
        return load_calib(path)
    if "cam0" not in raw or "cam1" not in raw:
        raise IngestError("calibration must be our yaml (K1,D1,K2,D2,R,T,image_size) or a Kalibr camchain (cam0, cam1)")
    cams = []
    for k in ("cam0", "cam1"):
        c = raw[k]
        fu, fv, cu, cv_ = c["intrinsics"]
        dm = c.get("distortion_model", "radtan")
        d = list(c["distortion_coeffs"])
        if dm in ("radtan", "radial-tangential"):
            model = "pinhole"
            d = (d + [0.0] * 5)[:5]
        elif dm == "equidistant":
            model = "fisheye"
            if len(d) != 4:
                raise IngestError("equidistant distortion needs 4 coefficients")
        else:
            raise IngestError(f"unsupported distortion_model {dm!r} (supported: radtan, equidistant)")
        cams.append(dict(K=np.array([[fu, 0, cu], [0, fv, cv_], [0, 0, 1.0]]), D=np.array(d, float).reshape(-1, 1),
                         model=model, size=tuple(int(v) for v in c["resolution"])))
    if cams[0]["model"] != cams[1]["model"] or cams[0]["size"] != cams[1]["size"]:
        raise IngestError("the two lenses must share distortion model and resolution")
    T = np.array(raw["cam1"]["T_cn_cnm1"], float)
    if T.shape != (4, 4):
        raise IngestError("cam1.T_cn_cnm1 must be a 4x4 matrix")
    if swap_eyes:
        cams.reverse()
        T = np.linalg.inv(T)
    return {"K1": cams[0]["K"], "D1": cams[0]["D"], "K2": cams[1]["K"], "D2": cams[1]["D"],
            "R": np.ascontiguousarray(T[:3, :3]), "T": np.ascontiguousarray(T[:3, 3:]), "image_size": cams[0]["size"], "model": cams[0]["model"]}


def swapped(calib):
    T = np.eye(4)
    T[:3, :3], T[:3, 3:] = calib["R"], calib["T"]
    Ti = np.linalg.inv(T)
    out = dict(calib)
    out.update(K1=calib["K2"], D1=calib["D2"], K2=calib["K1"], D2=calib["D1"],
               R=np.ascontiguousarray(Ti[:3, :3]), T=np.ascontiguousarray(Ti[:3, 3:]))
    return out


# ---------- timestamps ----------
def read_stamps(path):
    """One timestamp per row (first column; '#' header rows skipped). Unit guessed by magnitude -> ns."""
    vals = []
    with open(path) as f:
        for r in csv.reader(f):
            if not r or r[0].strip().startswith("#"):
                continue
            try:
                vals.append(float(r[0]))
            except ValueError:
                continue  # a text header row
    if len(vals) < 2:
        raise IngestError(f"{os.path.basename(path)}: fewer than 2 timestamps")
    v = np.array(vals)
    m = np.median(np.abs(v))
    scale = 1.0 if m > 1e17 else 1e3 if m > 1e14 else 1e6 if m > 1e11 else 1e9  # ns, us, ms, s
    ns = np.round(v * scale).astype(np.int64)
    if np.any(np.diff(ns) <= 0):
        raise IngestError(f"{os.path.basename(path)}: timestamps are not strictly increasing")
    return ns


def pair_by_stamps(tl, tr):
    """Nearest-right for each left; keep pairs whose skew < a quarter of the left frame period. Returns (il, ir, skew_ms)."""
    period = np.median(np.diff(tl))
    idx = np.clip(np.searchsorted(tr, tl), 1, len(tr) - 1)
    near = np.where(np.abs(tr[idx - 1] - tl) <= np.abs(tr[idx] - tl), idx - 1, idx)
    skew = np.abs(tr[near] - tl)
    keep = skew < period / 4
    # a right frame must not be used twice
    seen, il, ir, sk = set(), [], [], []
    for i in np.flatnonzero(keep):
        if near[i] in seen:
            continue
        seen.add(near[i]); il.append(i); ir.append(near[i]); sk.append(skew[i] / 1e6)
    return np.array(il), np.array(ir), np.array(sk)


def motion_lag(el, er):
    """Frame lag of the right video vs the left from motion energy; returns (lag, corr). lag>0: right is late."""
    el, er = np.asarray(el, float), np.asarray(er, float)
    n = min(len(el), len(er))
    if n < 3 * MAX_LAG or el.std() < 1e-6 or er.std() < 1e-6:
        return 0, 0.0
    best = (0, -2.0)
    for lag in range(-MAX_LAG, MAX_LAG + 1):
        a, b = (el[lag:n], er[:n - lag]) if lag >= 0 else (el[:n + lag], er[-lag:n])
        if len(a) < 20 or a.std() < 1e-9 or b.std() < 1e-9:
            continue
        c = float(np.corrcoef(a, b)[0, 1])
        if c > best[1]:
            best = (lag, c)
    return best


# ---------- video ----------
def _open(path):
    cap = cv2.VideoCapture(path)
    if not cap.isOpened():
        raise IngestError(f"cannot open video {os.path.basename(path)}")
    return cap


def _gray(frame):
    return frame if frame.ndim == 2 else cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)


def read_all(path, max_frames=None):
    cap = _open(path)
    fps = cap.get(cv2.CAP_PROP_FPS) or 0.0
    frames = []
    while True:
        ok, f = cap.read()
        if not ok or (max_frames and len(frames) >= max_frames):
            break
        frames.append(_gray(f))
    cap.release()
    return frames, fps


def motion_energy(frames):
    small = [cv2.resize(f, (80, int(80 * f.shape[0] / f.shape[1])), interpolation=cv2.INTER_AREA).astype(np.float32) for f in frames]
    return [float(np.abs(b - a).mean()) for a, b in zip(small, small[1:])]


def stereo_ratio(calib, lefts, rights, n=8):
    rect, sc = StereoRectifier(calib), QualityScorer()
    idx = np.linspace(0, len(lefts) - 1, min(n, len(lefts))).astype(int)
    vals = []
    for i in idx:
        l, r = rect.rectify(lefts[i], rights[i])
        vals.append(sc.score(l, r)["stereo_inliers"])
    return float(np.median(vals))


def ingest_robot(left, right, out_dir, calib, left_stamps=None, right_stamps=None, target_hz=10.0,
                 max_seconds=None, progress=None):
    """Convert one robot's uploads. `calib` is a dict from load_calibration. Returns the report dict."""
    frames_l, fps_l = read_all(left)
    frames_r, fps_r = read_all(right)
    if not frames_l or not frames_r:
        raise IngestError("a video has no readable frames")
    h, w = frames_l[0].shape
    if (w, h) != tuple(calib["image_size"]) or frames_r[0].shape != (h, w):
        raise IngestError(f"video size {w}x{h} (right {frames_r[0].shape[1]}x{frames_r[0].shape[0]}) does not match "
                          f"calibration image_size {tuple(calib['image_size'])}")
    report = {"video_size": [w, h], "frames_left": len(frames_l), "frames_right": len(frames_r)}
    if left_stamps and right_stamps:
        tl, tr = read_stamps(left_stamps), read_stamps(right_stamps)
        if len(tl) != len(frames_l) or len(tr) != len(frames_r):
            raise IngestError(f"timestamp rows ({len(tl)}, {len(tr)}) do not match frame counts ({len(frames_l)}, {len(frames_r)})")
        il, ir, skew = pair_by_stamps(tl, tr)
        if len(il) < 0.9 * len(tl):
            raise IngestError(f"only {len(il)} of {len(tl)} left frames have a right frame within a quarter frame period; streams are not synchronised")
        stamps = tl[il]
        report["sync"] = {"method": "timestamps", "status": "verified", "pairs": int(len(il)),
                          "skew_ms_mean": round(float(skew.mean()), 3), "skew_ms_max": round(float(skew.max()), 3)}
    elif left_stamps or right_stamps:
        raise IngestError("give timestamp files for both videos, or for neither")
    else:
        if abs(fps_l - fps_r) > 0.001 * max(fps_l, 1):
            raise IngestError(f"frame rates differ ({fps_l:.3f} vs {fps_r:.3f}); upload timestamp files or re-encode at one rate")
        if abs(len(frames_l) - len(frames_r)) > 1:
            raise IngestError(f"frame counts differ ({len(frames_l)} vs {len(frames_r)}); upload timestamp files or trim to the same start/end")
        if fps_l <= 0:
            raise IngestError("video has no frame rate; upload timestamp files")
        n = min(len(frames_l), len(frames_r))
        il = ir = np.arange(n)
        stamps = np.round(np.arange(n) / fps_l * 1e9).astype(np.int64)
        lag, corr = motion_lag(motion_energy(frames_l[:n]), motion_energy(frames_r[:n]))
        if lag != 0 and corr > 0.6:
            raise IngestError(f"right video appears {lag:+d} frames out of sync with the left (motion correlation {corr:.2f}); "
                              "trim it or upload timestamp files")
        report["sync"] = {"method": "nominal fps and frame count", "motion_lag_frames": lag, "motion_corr": round(corr, 3),
                          "status": "consistent (motion check)" if corr > 0.5 else "UNVERIFIED (not enough motion to check; equal fps and frame count are not proof)"}
    # eye order
    sample = np.linspace(0, len(il) - 1, min(8, len(il))).astype(int)
    L = [frames_l[il[i]] for i in sample]; R = [frames_r[ir[i]] for i in sample]
    asis, sw = stereo_ratio(calib, L, R), stereo_ratio(calib, R, L)  # hypothesis: the two videos are exchanged
    report["stereo_inlier_ratio"] = {"as_given": round(asis, 3), "if_eyes_swapped": round(sw, 3)}
    if asis < 0.2 and sw > 0.5:
        raise IngestError(f"left/right look swapped (stereo row-match {asis:.2f} as given vs {sw:.2f} swapped); swap the videos or the calibration order")
    if asis < 0.2:
        raise IngestError(f"left/right do not form a valid rectified pair with this calibration (row-match {asis:.2f}); check calibration and sync")
    # decimate to target_hz and write
    dt = 1e9 / target_hz
    keep, last = [], None
    for k, t in enumerate(stamps):
        if max_seconds and (t - stamps[0]) / 1e9 > max_seconds:
            break
        if last is None or t - last >= dt * 0.98:
            keep.append(k); last = t
    for cam in ("cam0", "cam1"):
        os.makedirs(f"{out_dir}/mav0/{cam}/data", exist_ok=True)
    rows = []
    for j, k in enumerate(keep):
        name = f"{int(stamps[k])}.png"
        cv2.imwrite(f"{out_dir}/mav0/cam0/data/{name}", frames_l[il[k]])
        cv2.imwrite(f"{out_dir}/mav0/cam1/data/{name}", frames_r[ir[k]])
        rows.append((int(stamps[k]), name))
        if progress and j % 50 == 0:
            progress(j, len(keep))
    for cam in ("cam0", "cam1"):
        with open(f"{out_dir}/mav0/{cam}/data.csv", "w") as f:
            f.write("#timestamp [ns],filename\n" + "".join(f"{t},{n}\n" for t, n in rows))
    save_calib(f"{out_dir}/calib.yaml", calib, extra={"source": "upload"})
    report.update(pairs_written=len(rows), target_hz=target_hz,
                  duration_s=round((rows[-1][0] - rows[0][0]) / 1e9, 2) if len(rows) > 1 else 0.0,
                  baseline_m=round(float(np.linalg.norm(calib["T"])), 4), model=calib["model"])
    with open(f"{out_dir}/ingest_report.json", "w") as f:
        json.dump(report, f, indent=1)
    return report
