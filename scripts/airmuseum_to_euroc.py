"""Convert one AirMuseum robot (ROS1 bags + Kalibr calib) into a EuRoC-style mav0 folder
plus a calib yaml that frontend.core.StereoRectifier reads (model: fisheye).

Usage: python scripts/airmuseum_to_euroc.py <robotdir with cam100_imu.bag,cam101.bag>
           <robot letter> <sensors dir> <out_mav0> <out_calib.yaml>
           [--start S] [--duration D] [--every N]
Timestamps: ROS header stamps in ns, as cam0/data/<ns>.png and data.csv (EuRoC layout).
cam0 (left) = /robotX/cam101, cam1 (right) = /robotX/cam100 (AirMuseum quirk, verified by rectification test).
"""
import argparse
import os
from pathlib import Path

import cv2
import numpy as np
import yaml
from rosbags.highlevel import AnyReader

from frontend.core import save_calib


def read_kalibr(path):
    with open(path) as f:
        c = yaml.safe_load(f)
    out = []
    for k in ("cam0", "cam1"):
        fu, fv, cu, cv_ = c[k]["intrinsics"]
        assert c[k]["distortion_model"] == "equidistant", c[k]["distortion_model"]
        out.append((np.array([[fu, 0, cu], [0, fv, cv_], [0, 0, 1.0]]),
                    np.array(c[k]["distortion_coeffs"], dtype=np.float64).reshape(4, 1),
                    tuple(c[k]["resolution"])))
    T = np.array(c["cam1"]["T_cn_cnm1"], dtype=np.float64)  # cam0 -> cam1
    return out, T


def frames(bag, topic, t0, t1, every):
    keep = []
    with AnyReader([Path(bag)]) as r:
        conns = [x for x in r.connections if x.topic == topic]
        for i, (conn, ts, raw) in enumerate(r.messages(connections=conns)):
            m = r.deserialize(raw, conn.msgtype)
            stamp = m.header.stamp.sec * 10**9 + m.header.stamp.nanosec
            if stamp < t0:
                continue
            if stamp > t1:
                break
            keep.append((stamp, m))
    out = []
    for stamp, m in keep:
        a = np.frombuffer(m.data, np.uint8)
        if m.encoding in ("mono8", "8UC1"):
            img = a.reshape(m.height, m.step)[:, :m.width]
        elif m.encoding in ("bgr8", "rgb8"):
            img = cv2.cvtColor(a.reshape(m.height, m.width, 3),
                               cv2.COLOR_BGR2GRAY if m.encoding == "bgr8" else cv2.COLOR_RGB2GRAY)
        else:
            raise SystemExit(f"unhandled encoding {m.encoding}")
        out.append((stamp, img.copy()))
    return out


def first_stamp(bag, topic):
    with AnyReader([Path(bag)]) as r:
        conns = [x for x in r.connections if x.topic == topic]
        for conn, ts, raw in r.messages(connections=conns):
            m = r.deserialize(raw, conn.msgtype)
            return m.header.stamp.sec * 10**9 + m.header.stamp.nanosec


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("robotdir"); ap.add_argument("letter"); ap.add_argument("sensors")
    ap.add_argument("out"); ap.add_argument("calib")
    ap.add_argument("--start", type=float, default=0); ap.add_argument("--duration", type=float, default=1e9)
    ap.add_argument("--every", type=int, default=1)
    a = ap.parse_args()
    ((K1, D1, s1), (K2, D2, s2)), T = read_kalibr(f"{a.sensors}/robot{a.letter}_cameras_calib.yaml")
    assert s1 == s2
    # AirMuseum quirk (verified by rectification test): cam101 is the LEFT eye, cam100 the RIGHT.
    # Kalibr gives T = cam0->cam1 with +x baseline, i.e. cam1 is to the left. Swap so cam0 = left.
    Tm = np.linalg.inv(T)
    K1, D1, K2, D2 = K2, D2, K1, D1
    T = np.ascontiguousarray(Tm)
    save_calib(a.calib, {"K1": K1, "D1": D1, "K2": K2, "D2": D2, "R": T[:3, :3], "T": T[:3, 3:],
                         "image_size": s1, "model": "fisheye"},
               extra={"source": f"AirMuseum robot{a.letter} Kalibr calib"})
    t_first = first_stamp(f"{a.robotdir}/cam100_imu.bag", f"/robot{a.letter}/cam100/image_raw")
    t0 = t_first + int(a.start * 1e9)
    t1 = t0 + int(a.duration * 1e9) if a.duration < 1e8 else 2**62
    # after the swap: left = cam101 bag, right = cam100 bag
    L = frames(f"{a.robotdir}/cam101.bag", f"/robot{a.letter}/cam101/image_raw", t0, t1, a.every)
    R = frames(f"{a.robotdir}/cam100_imu.bag", f"/robot{a.letter}/cam100/image_raw", t0, t1, a.every)
    rts = np.array([s for s, _ in R])
    os.makedirs(f"{a.out}/cam0/data", exist_ok=True); os.makedirs(f"{a.out}/cam1/data", exist_ok=True)
    rows, n = [], 0
    for i, (ts, img) in enumerate(L):
        if i % a.every:
            continue
        j = int(np.argmin(np.abs(rts - ts)))
        if abs(int(rts[j]) - ts) > 15_000_000:  # >15 ms apart: skip
            continue
        cv2.imwrite(f"{a.out}/cam0/data/{ts}.png", img)
        cv2.imwrite(f"{a.out}/cam1/data/{ts}.png", R[j][1])
        rows.append(f"{ts},{ts}.png"); n += 1
    for cam in ("cam0", "cam1"):
        with open(f"{a.out}/{cam}/data.csv", "w") as f:
            f.write("#timestamp [ns],filename\n" + "\n".join(rows) + "\n")
    print(f"robot{a.letter}: {len(L)} left, {len(R)} right in window, wrote {n} pairs to {a.out}; baseline {np.linalg.norm(T[:3,3]):.4f} m; size {s1}")


if __name__ == "__main__":
    main()
