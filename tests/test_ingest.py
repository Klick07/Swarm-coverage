import csv
import os
import sys

import cv2
import numpy as np
import pytest

from jobs import ingest as ing

CLIP = os.path.expanduser("~/data/airmuseum/clips/A")
pytestmark = pytest.mark.skipif(not os.path.isdir(CLIP), reason="AirMuseum clip A not on this machine")


def frames(n=90, start=400):
    rows = [r for r in csv.reader(open(f"{CLIP}/mav0/cam0/data.csv")) if not r[0].startswith("#")][start:start + n]
    L = [cv2.imread(f"{CLIP}/mav0/cam0/data/{r[1]}", 0) for r in rows]
    R = [cv2.imread(f"{CLIP}/mav0/cam1/data/{r[1]}", 0) for r in rows]
    return L, R, [int(r[0]) for r in rows]


def write_video(path, fr, fps=10):
    h, w = fr[0].shape
    vw = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*"mp4v"), fps, (w, h), True)
    for f in fr:
        vw.write(cv2.cvtColor(f, cv2.COLOR_GRAY2BGR))
    vw.release()


@pytest.fixture(scope="module")
def vids(tmp_path_factory):
    d = tmp_path_factory.mktemp("v")
    L, R, ts = frames()
    write_video(f"{d}/l.mp4", L); write_video(f"{d}/r.mp4", R)
    write_video(f"{d}/r_late.mp4", R[4:] + R[-1:] * 4)       # right 4 frames early => lag
    write_video(f"{d}/l_trim.mp4", L[:60])
    np.savetxt(f"{d}/l.csv", ts, fmt="%d", header="#timestamp [ns]", comments="")
    np.savetxt(f"{d}/r.csv", ts, fmt="%d", header="#timestamp [ns]", comments="")
    np.savetxt(f"{d}/r_off.csv", np.array(ts) + 50_000_000, fmt="%d")   # 50 ms offset = half period: no right frame within a quarter period
    return str(d)


def calib():
    return ing.load_calibration(f"{CLIP}/calib.yaml")


def test_ok_with_timestamps(vids, tmp_path):
    rep = ing.ingest_robot(f"{vids}/l.mp4", f"{vids}/r.mp4", str(tmp_path), calib(), f"{vids}/l.csv", f"{vids}/r.csv")
    assert rep["sync"]["status"] == "verified" and rep["sync"]["skew_ms_max"] < 1
    assert rep["pairs_written"] == 90 and rep["stereo_inlier_ratio"]["as_given"] > 0.4
    assert len(os.listdir(f"{tmp_path}/mav0/cam0/data")) == 90


def test_ok_nominal_reports_motion_check(vids, tmp_path):
    rep = ing.ingest_robot(f"{vids}/l.mp4", f"{vids}/r.mp4", str(tmp_path), calib())
    assert rep["sync"]["method"].startswith("nominal") and rep["sync"]["motion_lag_frames"] == 0


def test_lag_detected(vids, tmp_path):
    with pytest.raises(ing.IngestError, match="out of sync"):
        ing.ingest_robot(f"{vids}/l.mp4", f"{vids}/r_late.mp4", str(tmp_path), calib())


def test_skewed_timestamps_rejected(vids, tmp_path):
    with pytest.raises(ing.IngestError, match="not synchronised"):
        ing.ingest_robot(f"{vids}/l.mp4", f"{vids}/r.mp4", str(tmp_path), calib(), f"{vids}/l.csv", f"{vids}/r_off.csv")


def test_swapped_eyes_rejected(vids, tmp_path):
    with pytest.raises(ing.IngestError, match="swapped"):
        ing.ingest_robot(f"{vids}/r.mp4", f"{vids}/l.mp4", str(tmp_path), calib())


def test_count_mismatch_rejected(vids, tmp_path):
    with pytest.raises(ing.IngestError, match="frame counts differ"):
        ing.ingest_robot(f"{vids}/l_trim.mp4", f"{vids}/r.mp4", str(tmp_path), calib())


def test_size_mismatch_rejected(vids, tmp_path):
    c = calib(); c["image_size"] = (752, 480)
    with pytest.raises(ing.IngestError, match="does not match"):
        ing.ingest_robot(f"{vids}/l.mp4", f"{vids}/r.mp4", str(tmp_path), c)


def test_kalibr_parse_and_swap():
    p = os.path.expanduser("~/data/airmuseum/sensors/robotA_cameras_calib.yaml")
    if not os.path.exists(p):
        pytest.skip("no sensors")
    c = ing.load_calibration(p)
    s = ing.load_calibration(p, swap_eyes=True)
    assert c["model"] == "fisheye" and c["image_size"] == (640, 512)
    assert abs(np.linalg.norm(c["T"]) - 0.262) < 0.01 and np.allclose(np.linalg.norm(s["T"]), np.linalg.norm(c["T"]))
    assert s["T"][0, 0] < 0 < c["T"][0, 0]


def test_stamp_units_and_errors(tmp_path):
    for scale in (1, 1e3, 1e6, 1e9):
        np.savetxt(f"{tmp_path}/t.csv", np.arange(5) * 0.1 * scale + 1.6e9 * scale, fmt="%.0f" if scale > 1 else "%.3f")
        assert np.allclose(np.diff(ing.read_stamps(f"{tmp_path}/t.csv")), 1e8, rtol=1e-3)
    np.savetxt(f"{tmp_path}/bad.csv", [3, 2, 1])
    with pytest.raises(ing.IngestError):
        ing.read_stamps(f"{tmp_path}/bad.csv")
