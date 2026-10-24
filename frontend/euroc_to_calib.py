"""Convert a EuRoC sequence's cam0/cam1 sensor.yaml into our calib/camN.yaml format.

Usage: python -m frontend.euroc_to_calib <sequence>/mav0 <out.yaml>
"""
import sys

import numpy as np
import yaml

from frontend.core import save_calib


def read_sensor(path):
    with open(path) as f:
        text = "".join(line for line in f if not line.startswith("%"))
    s = yaml.safe_load(text)
    fu, fv, cu, cv = s["intrinsics"]
    K = np.array([[fu, 0, cu], [0, fv, cv], [0, 0, 1]])
    D = np.array(list(s["distortion_coefficients"]) + [0.0])  # radtan k1 k2 p1 p2 (+k3=0)
    T_BS = np.array(s["T_BS"]["data"]).reshape(4, 4)          # sensor -> body
    assert s["distortion_model"] == "radial-tangential", s["distortion_model"]
    return K, D, T_BS, tuple(s["resolution"])


def main(mav0, out):
    K1, D1, T_B_c0, size = read_sensor(f"{mav0}/cam0/sensor.yaml")
    K2, D2, T_B_c1, size2 = read_sensor(f"{mav0}/cam1/sensor.yaml")
    assert size == size2
    T_c1_c0 = np.linalg.inv(T_B_c1) @ T_B_c0  # maps cam0 points into cam1, as cv2.stereoRectify expects
    calib = {"K1": K1, "D1": D1, "K2": K2, "D2": D2, "R": T_c1_c0[:3, :3],
             "T": T_c1_c0[:3, 3:], "image_size": size, "model": "pinhole"}
    save_calib(out, calib, extra={"source": f"EuRoC {mav0}", "T_B_c0": T_B_c0.tolist()})
    print(f"{out}: baseline {np.linalg.norm(calib['T']):.4f} m, size {size}")


if __name__ == "__main__":
    main(*sys.argv[1:3])
