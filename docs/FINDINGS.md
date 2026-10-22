# Findings

Facts verified before building. Each item says how it was checked and what is still open.
Swarm-SLAM sources inspected on 2026-10-22 at these commits:

| Repo | Commit | Date |
|---|---|---|
| MISTLab/Swarm-SLAM | `af17c4b` | 2025-04-10 |
| lajoiepy/cslam | `fccf887` | 2025-04-10 |
| lajoiepy/cslam_interfaces | `0e6bb91` | 2024-03-04 |
| lajoiepy/cslam_experiments | `0b5fc6c` | 2025-02-06 |

Pin these commits in our own `.repos` file so the EC2 build is reproducible.

## 1. Camera stream format — ASSUMED (no camera available yet)

No camera is on hand, so the pipeline is built against this assumption and
everything camera-specific lives in `config/cameras.yaml`:

| Property | Assumed value |
|---|---|
| Layout | One side-by-side stream per camera (left half = left eye) |
| Protocol | HTTP MJPEG, e.g. `http://<ip>:8080/stream` |
| Resolution | 1280x480 total (640x480 per eye) |
| Frame rate | 15 fps |
| Hardware timestamps | None |

`capture.py` must also accept two separate URLs per camera and RTSP, since
`cv2.VideoCapture` reads both the same way. When a real camera arrives, test it
with `ffprobe <url>` (installed on the Mac) and replace this section with the
measured values.

## 2. Swarm-SLAM stereo inputs — VERIFIED from source

Closest example: `cslam_experiments/launch/datasets_experiments/kitti_stereo.launch.py`,
which includes `launch/cslam/cslam_stereo.launch.py` once per robot, plus
`config/kitti_stereo.yaml`.

**Nodes per robot** (all in namespace `/r<N>`): `loop_closure_detection_node.py`,
`map_manager`, `pose_graph_manager`.

**Subscribed topics** (relative to `/r<N>`, set in the yaml under `frontend.*`):

| Parameter | Value in kitti_stereo.yaml | Message type |
|---|---|---|
| `frontend.left_image_topic` | `stereo_camera/left/image_rect_color` | `sensor_msgs/msg/Image` |
| `frontend.right_image_topic` | `stereo_camera/right/image_rect_color` | `sensor_msgs/msg/Image` |
| `frontend.left_camera_info_topic` | `stereo_camera/left/camera_info` | `sensor_msgs/msg/CameraInfo` |
| `frontend.right_camera_info_topic` | `stereo_camera/right/camera_info` | `sensor_msgs/msg/CameraInfo` |
| `frontend.odom_topic` | `odom` | `nav_msgs/msg/Odometry` |

- The five topics are joined with `message_filters` **ApproximateTime**, QoS
  best-effort (`stereo_handler.cpp:17-44`).
- Images must already be **rectified**: `alreadyRectified = true` is hard-coded
  (`stereo_handler.cpp:120`). Accepted encodings: mono8, mono16, bgr8, rgb8,
  bgra8, rgba8; mono8 is recommended.
- Frames whose odometry has `pose.covariance[0] > 1000` are dropped as
  tracking failures (`stereo_handler.cpp:55`). Our odometry must set this when it
  loses tracking.
- A TF from `frontend.sensor_base_frame_id` (`camera_link`) to the image
  `header.frame_id` is looked up with a 0.1 s timeout. KITTI publishes static
  transforms for this; we need the same.

**Odometry source:** external. Swarm-SLAM does not compute odometry. The KITTI
example runs RTAB-Map `rtabmap_odom/stereo_odometry` per robot
(`launch/odometry/rtabmap_kitti_stereo_odometry.launch.py`), publishing
`/r<N>/odom`. Two options for us:
- **A (default):** reuse RTAB-Map stereo odometry. It's already a dependency and is the
  tested path.
- **B:** our own OpenCV 5 odometry (ORB + stereo depth + `solvePnPRansac`),
  publishing `nav_msgs/Odometry`. This gives more OpenCV 5 credit but is more risk.
  Build it only if A misbehaves on our footage, or after the main pipeline works.

**Keyframes:** Swarm-SLAM selects them itself. With
`frontend.keyframe_generation_ratio_threshold` > 0.99 (KITTI uses 1.0), every
synchronized frame becomes a keyframe. Below that, it uses an inlier-ratio test
(`rgbd_handler.cpp:314-350`). So our front end must **not** duplicate keyframe
selection. Instead, it should drop bad frames (blur, too few features) before
they reach Swarm-SLAM. This matches spec Phase 3 step 4.

**Robot count and IDs:** launch arguments `robot_id` (0, 1, 2) and
`max_nb_robots` (3), passed as node parameters. Namespace `/r<N>`.

**Place recognition:** `frontend.global_descriptor_technique: cosplace`.
`frontend.nn_checkpoint` is resolved relative to the installed `cslam` share
directory (`cosplace.py:44-47`). The code defaults to ResNet-18 with dim 64
(`models/download.sh`); KITTI overrides this to resnet101/512. We set
`backbone: resnet18`, `descriptor_dim: 64`. The weights come from
github.com/gmberton/CosPlace. The exact download method still needs checking
on EC2 (torch.hub `get_trained_model` is the likely route).

**Inter-robot settings worth tuning later:** `similarity_threshold` (0.8),
`pnp_min_inliers` (20), `inter_robot_detection_period_sec` (5),
`inter_robot_loop_closure_budget` (1).

**Multi-robot transport:** the Zenoh bridge plus `ROS_DOMAIN_ID=robot_id`
(`experiment_lidar.launch.py:30-38`) is only for robots on separate machines.
All three of our robots run on one EC2 instance in one ROS domain, as in the
KITTI dataset launch. **Zenoh is not needed.**

**Outputs we can use for coverage:** per-robot `cslam/keyframe_data` and
`cslam/keyframe_odom`. Visualization topics are under `/cslam/viz/*` when
`visualization.enable: true`. Exact loop-closure message names are to be
recorded in Phase 4 from `ros2 topic list` on the running system.

## 3. ROS 2 distribution — STRONG EVIDENCE, build not yet run

The spec assumed Foxy. That is out of date:
`Swarm-SLAM/build-everything.sh` says "tested with Humble and Jazzy" and uses
GTSAM 4.2, not 4.1.1.

**Decision: ROS 2 Jazzy on Ubuntu 24.04.** Fallback: Humble on Ubuntu 22.04.
Confirm by running the build on EC2 in Phase 1.

Things to watch in that script:
- It installs `ros-rolling-cv-bridge` regardless of distro. This looks like a bug;
  use `ros-$ROS_DISTRO-cv-bridge`.
- It needs `intel-mkl-full`, which is x86-only. That rules out Graviton for the
  SLAM host; Graviton stays limited to the optional COOL benchmark.
- It clones cslam repos at `main` HEAD. Use the commits pinned above.

## 4. OpenCV 5 — VERIFIED on the Mac, EC2 pending

- PyPI has `opencv-python` / `opencv-contrib-python` **5.0.0.93**.
- Installed in the project venv (`.venv`, Python 3.15.0):
  `cv2.__version__ == "5.0.0"`, numpy 2.5.4. ORB, StereoSGBM, `cv2.fisheye` and
  `solvePnPRansac` are all present.
- cslam itself uses `cv_bridge` (C++ and Python) against the system OpenCV 4.
  That's fine, because it runs in its own process. **Our nodes never import
  `cv_bridge`.** They convert `sensor_msgs/Image` <-> NumPy by hand.
- EC2 risk: our nodes need `rclpy` (system Python 3.12 on Jazzy) and cv2 5 in
  one interpreter. Plan: a venv with `--system-site-packages` so that rclpy is
  visible and the venv's cv2 5 shadows the system python3-opencv. Check that
  the numpy version is compatible with rclpy's message code. If it's not, pin numpy in
  that venv.

## 5. Time synchronization — DECIDED

The assumed camera has no hardware timestamps, and with a side-by-side stream
left and right arrive in the **same frame**, so they're synchronized by construction.
- Stamp each frame on arrival at the Mac relay with `time.time_ns()`, plus a
  per-camera sequence number.
- Use the same stamp for left, right and both CameraInfo messages. This satisfies
  Swarm-SLAM's ApproximateTime join exactly.
- If a camera turns out to deliver two separate streams: pair by nearest arrival
  time, **reject pairs more than 15 ms apart** (under a quarter of a frame at
  15 fps), and log the offset of each accepted pair.
- Cross-camera alignment uses the Mac's single clock for all three cameras. Add
  an NTP check on the Mac before each recording.

## Status

| Item | Status |
|---|---|
| 1. Camera format | Assumed; re-check when hardware arrives |
| 2. Swarm-SLAM inputs | Verified from source |
| 3. ROS 2 distro | Jazzy chosen; prove in Phase 1 build |
| 4. OpenCV 5 | Verified on Mac; check on EC2 |
| 5. Time sync | Decided |

Blocked on: AWS credentials (`aws configure`) for Phase 1.
