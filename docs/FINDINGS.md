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

## 6. AWS account restrictions — VERIFIED 2026-10-22

IAM user `<iam-user>` in account <aws-account-id>, default region us-east-1. The account
belongs to AWS Organization `<org-id>`, and its SCP `<scp-id>` **explicitly
denies** some services in every region tested (us-east-1/2, us-west-2,
eu-west-1, eu-central-1, ap-south-1, ap-southeast-1). IAM policies on the user
cannot override an SCP.

| Works | Denied by SCP | Not tested (user has no policy for it) |
|---|---|---|
| S3, CloudWatch Logs, CloudWatch metrics, ACM, Bedrock (model listing; Anthropic models visible), Elastic Beanstalk (describe only) | EC2 (incl. DescribeVpcs), DynamoDB, ELB, SSM, Lambda, SNS, Bedrock Agents | ECS, ECR, App Runner, Lightsail, SageMaker, Batch, API Gateway, CloudFront, SQS, IoT |

As things stand, the spec's AWS design (EC2 + DynamoDB + ALB) **cannot be deployed**
in this account. We need the hackathon organizers to say which compute service
participants are meant to use.

**Update:** the account is on the AWS **Free plan** ($100 credits, ends 2027-04-09).
The owner (console user) can use EC2; the IAM user `<iam-user>` cannot. Workaround:
the owner launches instances in the console, and the agent works over SSH.

## 7. EC2 instance — VERIFIED 2026-10-22

| Property | Value |
|---|---|
| Instance | `<instance-id>` ("Cloude"), ap-southeast-2a (Sydney) |
| Type | m7i-flex.large: 2 vCPU, 7.6 GiB RAM, about $0.12/h (from credits) |
| OS | Ubuntu 24.04.5 LTS, x86_64 |
| Disk | 30 GB gp3; 4 GB swap file added |
| Access | `ssh -i ~/.ssh/swarm-slam.pem ubuntu@<ec2-public-ip>` (IP changes on stop/start) |
| IAM role | none attached yet |

ROS Jazzy apt has **`ros-jazzy-gtsam` 4.2.0**, the same version build-everything.sh
compiles, so we skip the GTSAM source build. cslam uses GTSAM only from C++.
RTAB-Map 0.23.13 (apt) links the system OpenCV 4.6. Our OpenCV 5 nodes stay in a
separate venv.

Setup script: `scripts/ec2_setup.sh`.

## 8. Phase 1 build — VERIFIED 2026-10-22

- `colcon build`: all 3 packages built in 4 min 36 s (cslam 3 min 49 s).
- Linker warning: librtabmap_core links liboctomap 1.9 while ROS links 1.10,
  so both get loaded. The nodes run, but watch for crashes in octomap code.
- Smoke test: `map_manager`, `pose_graph_manager` and
  `loop_closure_detection_node.py` all start and keep running ("Initialization
  done", CosPlace selected).
- Fixed: numba 0.68 crashed on Ubuntu's coverage 7.4
  (`coverage.types.Tracer` missing). A user-level coverage 7.16.2 solves it.
- Python stack (system python 3.12 + `~/.local`): torch 2.14.1+cpu,
  torchvision 0.29.1, numpy 1.26.4 (apt), numba 0.68.0, open3d 0.20.0, system
  cv2 4.6.0.
- CosPlace ResNet-18/64 from the CosPlace v1.0 release (sha256 `5535824d…`)
  loads into cslam's `GeoLocalizationNet` with `strict=True` and gives 64-dim
  descriptors. It's installed at `install/cslam/share/cslam/models/resnet18_64.pth`.
  Config: `nn_checkpoint: models/resnet18_64.pth`, `backbone: resnet18`,
  `descriptor_dim: 64`.
- Disk after build: 16 GB used, 13 GB free.

Still open in Phase 1: the AMI snapshot (step 9; console only, done by the owner).

## 9. EuRoC 3-robot runs — MEASURED 2026-10-24

Data: EuRoC MH_01_easy / MH_02_easy / MH_03_medium (ASL format) as robots 0/1/2,
from the unofficial Hugging Face mirror `GlowBond/EuRoC_MAV_Dataset` (ETH's
Research Collection blocks scripted downloads). Replay at rate 0.25, every 2nd
frame (2.5 stereo pairs/s per robot) on the 2-vCPU instance.

Rectification check (`scripts/check_rectification.py`, 60 MH_01 pairs): share of
left-right ORB matches on the same row with positive disparity is 0.000 raw
(the right image sits about 13 px higher) and 0.900 after OpenCV 5 rectification.

| Run | Inter-robot LC | Joint ATE, odometry only | Joint ATE, Swarm-SLAM | Per-robot ATE optimized (r0/r1/r2) |
|---|---|---|---|---|
| euroc3_r025 | 26 | — (no timestamps logged) | — | — |
| euroc3_r025_b | 23 | 0.383 m | **1.662 m** (bad) | 0.059 / 0.048 / 0.081 m |
| euroc3_r025_c | 28 | 0.274 m | **0.090 m** | 0.088 / 0.071 / 0.071 m |

- **Run b bug:** robots were merged rotated by about 120°, which is exactly the
  rotated camera_link->camera_optical TF. All 13 verifiable inter-robot closures were
  true matches (0.01–1.6 m apart in ground truth). Fix (run c): stamp images in
  the base frame (identity transform), as Swarm-SLAM's KITTI launch does.
- `evaluation.enable_pose_timestamps_recording: true` is required, or no
  pose can be matched to ground truth (run a).
- **The "odometry only" joint ATE is flattering:** all MH flights take off
  from the same spot, so unmerged odometry frames line up by coincidence. The
  fair comparison is per-robot ATE plus joint ATE after merging.
- Joint ATE = one SE(3) alignment for all 3 robots against Leica ground truth (the
  shared frame across MH sequences). Ground truth is converted from body to cam0 with
  EuRoC's T_BS. The small rectification rotation R1 is ignored (less than 1°).
