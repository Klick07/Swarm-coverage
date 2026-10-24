"""sensor_msgs <-> NumPy without cv_bridge (cv_bridge links the system OpenCV 4)."""
import numpy as np
from sensor_msgs.msg import CameraInfo, Image


def to_msg(img, header):
    msg = Image(header=header, height=img.shape[0], width=img.shape[1],
                encoding="mono8", is_bigendian=0, step=img.shape[1])
    msg.data = img.tobytes()
    return msg


def from_msg(msg):
    assert msg.encoding in ("mono8", "8UC1"), msg.encoding
    return np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)[:, :msg.width]


def camera_info(P, size, header):
    """CameraInfo for an already-rectified image: no distortion, R = I."""
    info = CameraInfo(header=header, width=int(size[0]), height=int(size[1]),
                      distortion_model="plumb_bob")
    info.d = [0.0] * 5
    info.k = [float(v) for v in P[:3, :3].ravel()]
    info.r = [1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0]
    info.p = [float(v) for v in P.ravel()]
    return info
