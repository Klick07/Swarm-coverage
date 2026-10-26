# Findings

> **Phase note (2026-10-26).** This is the chronological working log from the earlier build phase, when the coordinator used Amazon Nova on Bedrock and the EuRoC recordings were the test input. Statements such as "real time" or storage sizes refer to that phase and are superseded by `docs/REPORT.md` and the README (recorded footage only, near real time not demonstrated). Results from the team's final stereo footage are pending and not included.


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
- Figure: `docs/figures/merge_before_after.png` (`python -m eval.plot_merge`).
- Joint ATE = one SE(3) alignment for all 3 robots against Leica ground truth (the
  shared frame across MH sequences). Ground truth is converted from body to cam0 with
  EuRoC's T_BS. The small rectification rotation R1 is ignored (less than 1°).

## 10. Phase 5 coordinator — MEASURED 2026-10-24 (replay of run euroc3_r025_d)

**Bedrock on this account:** Anthropic models are refused ("not available for this
account"; free plan). Amazon Nova works (Micro, Lite, 2 Lite, all with tool calling).
The coordinator uses `us.amazon.nova-2-lite-v1:0`.

Pipeline: ROS -> `coordinator/recorder.py` -> `events.jsonl` -> `coordinator/main.py`
(Bedrock agent + tools) -> `coverage.db` (SQLite; DynamoDB is blocked) -> `dashboard/`.

Run d events: 4,711 quality, 1,384 keyframes, 261 merged-pose snapshots, 177
inter-robot closure candidates (Swarm-SLAM: 24 accepted, 153 rejected).

**Thresholds** (tuned on recorded runs, not copied from elsewhere):
- Weak tracking: 6-frame median blur < 90 (about the 5th percentile, run c) or track_inliers
  < 0.50 (between the 1st and 5th percentile); 20 s cooldown per camera.
- Overlap re-verification (OpenCV 5 ORB + MAGSAC F-matrix on the two keyframes):
  confidence high if inliers >= 50. On run d, Swarm-SLAM-accepted closures have median
  210 inliers (83% >= 50); 150 random cross-robot pairs >5 m apart have median 12, and
  0.7% are >= 50.
- Frame-dropping gate before SLAM: off. Even the worst frames keep 300+ features,
  and dropping frames breaks odometry, so the coordinator acts on quality instead.

**Result (final Bedrock replay):** 24 overlaps -> 20 covered automatically, 4 held for
human approval (25–47 inliers); 15 "slow down" notifications (camera 2: 7, cameras 0/1: 4
each); 153 Swarm-SLAM-rejected closures logged, not acted on; 9 areas covered, 28 seen
by one camera. Overlap notifications identical to the rule-based reference. 121 Nova
calls, about 201k input / 6.7k output tokens, 0 fallbacks.

**Agent failures found and fixed (keep for the report):**
1. Wrong recipient: in 2 of 24 overlaps where camera 2 arrived before camera 1, the model
   notified camera 2. Fix: verify_overlap returns explicit arrived_first/arrived_second
   and the exact notification. Afterwards: 0 of 24 wrong.
2. Invented thresholds in reasons ("inlier ratio 0.49 < 0.5"); no such rule exists.
   Fix: the tool returns confidence_rule; the prompt names min_inliers as the only
   threshold. Afterwards: no reason cites a ratio threshold.
3. A Bedrock call hung for 17 min in an SSL read (network drop). Fix: 10 s connect,
   30 s read timeouts, 3 retries; on failure that event falls back to the rules and
   the log says so.

**Human control:** `mark_covered` refuses low-confidence overlaps in code, so neither
the model nor the rules can change coverage without approval (test:
`tests/test_coordinator.py`). Verified on the dashboard: Approve #5 -> coverage updated,
camera 2 notified, logged as source=human; Reject #10 -> coverage unchanged, logged.

## 11. Phase 6 failure cases and evaluation — MEASURED 2026-10-24

No house walks exist (no cameras), so the spec's failure cases were created from EuRoC
(`frontend/euroc_replay.py --perturb`; each perturbed frame is tagged in stamps_rN.csv):
blank wall = textureless frames on camera 1 for 40–47% of MH_02 (10.6 s data); quick shake =
random 15–35 px motion blur plus jitter on camera 2 for 60–66% of MH_03 (8.0 s data).
Front-end check (`scripts/check_perturb.py`, MH_02): clean blur 275 / 1000 features /
track 0.94; blank blur 20 / 0 features; shake blur 10 / track 0.20.

| | Run e (Odom/ResetCountdown=0, default) | Run f (ResetCountdown=3) |
|---|---|---|
| First "slow down", blank / shake | +0.3 s / +0.2 s data | +0.3 s / +0.2 s data |
| SLAM back after window ended, blank | 147 s wall (never during the walk) | 1.6 s wall |
| Odometry-lost frames, blank / shake | 202 / 179 | 63 / 15 |
| Keyframes cams 1/2 (clean run d: 440/405) | 324 / 337 | 442 / 403 |
| Inter-robot closures | 20 | 34 |
| Joint ATE merged | 0.069 m (post-failure parts missing) | 0.336 m |
| Per-robot ATE cams 1/2, odometry only / merged | 0.041, 0.133 / 0.030, 0.097 | 2.022, 6.453 / 0.409, 0.418 |
| Coverage precision / recall | — | 1.00 (23/23) / 0.375 (6/16) |

- Without auto-reset, RTAB-Map odometry never recovered after either failure, and the
  coordinator kept saying "Tracking OK" because the images were fine again. Two fixes:
  (1) `Odom/ResetCountdown=3` (now the launch default); (2) the coordinator alerts when 30 good
  frames arrive without a Swarm-SLAM keyframe (clean max 21; 0 false alarms on runs d/e/f
  healthy robots) and gives the all-clear when keyframes resume.
- Reset trade-off: odometry resumes from the last pose, so motion during the outage is
  lost (2.0 / 6.5 m odometry error); Swarm-SLAM loop closures cut it to about 0.41 m. That
  gives a complete map with worse accuracy, instead of an accurate map that silently stops.
- Similar-looking places (`eval/similar_places.py`, run d): 1,500 cross-camera pairs more
  than 5 m apart in ground truth; 7 had >= 50 OpenCV inliers. The worst were the same
  object seen from 5–6 m away (matches crowded in one patch), not look-alike rooms. Added a
  spread rule (inliers cover >= 10% of each image). With 6 injected false closures
  (success=true): 4 held for a human, 2 auto-covered (75 inl/14%, 51 inl/20%).
  Real overlaps: 19 auto, 5 to a human. Precision with injection 0.83, without 1.00.

**Coverage precision/recall** (`eval/coverage_pr.py`; correct = the two keyframes <= 3 m
apart in ground truth; recall over 2 m ground-truth cells visited by >= 2 cameras):
run d 1.00 (10/10) / 0.22 (4/18); run f 1.00 (23/23) / 0.375 (6/16). Recall is low
because coverage is only claimed at inter-robot closures (cslam budget: 1 per 5 s).
Map-based coverage from merged trajectories would raise it (not built).

**Latency** (`eval/latency.py`, run d, quarter-speed replay on 2 vCPU):
front end 0.19 s median per frame (p90 0.33); Swarm-SLAM frame -> inter-robot closure
27 s median wall (p90 227 s, about 7 s data time; detection runs every 5 s with a budget of 1);
Bedrock overlap decision 2.2 s median (p90 2.5); dashboard polls every 1 s. Frame -> "area
covered" on the dashboard is about 30 s median, dominated by loop-closure detection.

**Trajectory, merged vs single:** see item 9 (run c 0.090 m joint; run d 0.079 m joint).

## 12. Vicon Room 1 and the weak-tracking rule — MEASURED 2026-10-24

Data: EuRoC V1_01_easy / V1_02_medium / V1_03_difficult (145 / 85 / 107 s), same stereo rig as
the Machine Hall (same calibration, 0.1101 m baseline), Vicon ground truth, same mirror
(`scripts/fetch_euroc.py vicon_room1 ...`). Unpacked: 2.1 + 1.3 + 1.6 GB. Run `v1_r025_a`, quarter speed.

| ATE vs Vicon | cam 0 | cam 1 | cam 2 | joint |
|---|---|---|---|---|
| odometry only | 0.286 m | 0.317 m | 0.458 m | 0.895 m |
| Swarm-SLAM merged | 0.243 m | 0.059 m | 0.159 m | **0.207 m** |

- These flights don't share a start pose, so here the odometry-only joint ATE is a fair
  "not merged" baseline: merging cuts it from 0.895 to 0.207 m.
- 12 inter-robot closures accepted, 167 rejected by Swarm-SLAM (483 measurement log lines).
- Coverage: precision 1.00 (7/7), recall 0.667 (6/9 co-visited cells). The small room
  overlaps constantly. Figure: `docs/figures/merge_vicon_room1.png`.

**The blur threshold did not transfer.** Tuned at 90 in the bright hall, blur < 90 held for
31% / 71% / 83% of normal Vicon frames (median blur 128 / 59 / 48), while tracking was healthy
(median track_inliers 0.65–0.69; 2 lost-odometry frames in total). Comparison
(`eval/quality_rules.py`, 6-frame median, 20 s cooldown):

| Rule | hall d | hall e / f (failures caught) | Vicon v1 |
|---|---|---|---|
| blur < 90 or track < 0.5 (old) | 15 | 16 / 16 (2/2) | 52 |
| **track < 0.5 or features < 100 (new)** | 7 | 7 / 7 (2/2) | 14 |
| blur < 0.25x own median or track < 0.5 | 10 | 10 / 10 (2/2) | 19 |

New rule adopted. Blur is shown but doesn't trigger. Failure timing is unchanged (run f:
"slow down" +0.3 s blank, +0.2 s shake). Tests: low blur alone doesn't warn; a blank wall
(0 features, no tracking score) does.
