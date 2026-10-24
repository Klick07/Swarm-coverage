# Swarm coverage

Three stereo cameras map a building together. **OpenCV 5** cleans and scores every frame,
**Swarm-SLAM** merges the three maps, and an **Amazon Bedrock** agent tells each walker what
to do next: *"Area covered by camera 0, camera 2 move on"*, *"Camera 1, slow down, tracking
is weak"*. It asks a human whenever the visual evidence is weak.

Built for the OpenCV 5 hackathon, Agentic Vision Award track.

![Architecture](docs/figures/architecture.png)

**What runs today.** No cameras were available, so the three "walkers" are EuRoC MAV flights:
real stereo footage with motion-capture or laser-tracker ground truth, replayed through the full
pipeline on AWS EC2. Two environments: an indoor room (**Vicon Room 1**, V1_01–03, the demo) and an
industrial hall (**Machine Hall**, MH_01–03). The camera
capture and calibration path (spec Phases 2–3 on real hardware) is designed but not built.
See [Limitations](#limitations).

## Results (all measured; details in [docs/REPORT.md](docs/REPORT.md))

| | Result |
|---|---|
| Merged map, 3 robots, one alignment vs ground truth | Vicon Room: **0.895 m unmerged → 0.207 m merged**. Machine Hall: **7.9–9.0 cm** (robot 2: 23.7 → 7.1 cm). |
| OpenCV 5 rectification on real EuRoC pairs | same-row matches **0% raw, 90% rectified** (raw right image sits ~13 px higher) |
| "Area covered" precision / recall vs ground truth | **1.00** in every run / 0.667 (room), 0.22–0.375 (hall) |
| Weak-tracking warning after a blank wall / quick shake | **+0.3 s / +0.2 s** |
| SLAM recovery after a blank wall | 147 s → **1.6 s** with odometry auto-reset (accuracy trade-off: 0.34 m) |
| Look-alike false closures (6 injected) | **4 held for a human**, 2 auto-covered |
| Coordinator (Bedrock), final replays | room: 11 auto-covered, 1 to a human; hall: 19 / 5. 0 wrong recipients, 0 fallbacks |
| Weak-tracking rule across environments | blur threshold gave 52 nuisance warnings in the room → tracking-based rule: 15, both failures still caught |
| Frame → "area covered" on the dashboard | ~30 s median, almost all of it Swarm-SLAM loop-closure detection |

![Merged map, Vicon Room 1](docs/figures/merge_vicon_room1.png)
![Merged map before and after the frame fix](docs/figures/merge_before_after.png)

## How it works

1. **Front end (`frontend/`, OpenCV 5.0.0).** It rectifies each stereo pair (`stereoRectify`,
   `remap`) and scores it: Laplacian-variance blur, ORB features, a stereo same-row check, and
   tracking inliers from a RANSAC fundamental matrix. It publishes rectified mono8 images to
   Swarm-SLAM, and does not use `cv_bridge`.
2. **SLAM.** RTAB-Map stereo odometry, then Swarm-SLAM (CosPlace ResNet-18 place
   recognition and a GTSAM 4.2 pose graph) on three robots in one ROS 2 Jazzy domain.
3. **Recorder (`coordinator/recorder.py`).** Writes quality scores, keyframes, inter-robot
   closures (with Swarm-SLAM's success flag) and merged poses to one `events.jsonl` per run.
4. **Coordinator (`coordinator/`).** Amazon Nova 2 Lite on Bedrock, using tool calls. For each
   overlap it re-verifies the two keyframes with OpenCV 5 (ORB + MAGSAC). With ≥ 50 inliers
   that span ≥ 10% of each image, it marks the area covered and tells the second camera to move
   on. Otherwise it requests human approval. It also warns on weak tracking (tracking inliers
   < 0.5 or fewer than 100 features; blur is shown but doesn't trigger, because it depends on the
   scene), and on SLAM tracking loss while the images still look fine. Guardrails live in code: `mark_covered`
   refuses a low-confidence overlap until a human has approved it. Every action is logged
   with the values that triggered it.
5. **Dashboard (`dashboard/`).** Live camera views, the merged map with coverage cells,
   approval cards showing both keyframes and the matches, the notification feed, and the
   decision log. It is served on localhost and published through a Cloudflare quick tunnel.
   Answering an approval needs a token.

## Repository

| Path | What |
|---|---|
| `frontend/core.py` | OpenCV 5 rectification, quality scores, quality gate (no ROS) |
| `frontend/node.py`, `rosimg.py` | ROS 2 front-end node; Image/CameraInfo without cv_bridge |
| `frontend/euroc_replay.py`, `euroc_to_calib.py` | EuRoC replay (with failure injection); calibration import |
| `launch/euroc_swarm.launch.py`, `config/cslam_stereo.yaml` | 3-robot launch; our copy of the Swarm-SLAM config |
| `coordinator/` | recorder, coordinator loop, Bedrock agent, OpenCV 5 re-verification, SQLite store |
| `dashboard/` | FastAPI app and single-page dashboard |
| `eval/` | ATE vs ground truth, coverage precision/recall, latency, failure timeline, look-alike search, plots |
| `scripts/` | EC2 setup, OpenCV 5 env, EuRoC download, run, demo server, evidence checks |
| `tests/` | 15 unit tests (rectification, quality, evaluation, coordinator guardrails) |
| `docs/FINDINGS.md` | every verified fact and measurement, in the order it was found |
| `docs/REPORT.md` | technical report |
| `docs/VIDEO_SCRIPT.md` | ≤ 5 minute demo video script |
| `swarm.repos` | Swarm-SLAM packages pinned to the tested commits |

## Build and run

**Hardware used:** one EC2 m7i-flex.large (2 vCPU, 8 GB) with Ubuntu 24.04 (x86_64) and a
30 GB disk, plus an IAM role with Bedrock access. The coordinator and the evaluation also run
on a Mac.

```bash
# 1. ROS 2 Jazzy, RTAB-Map, GTSAM 4.2 (apt), Swarm-SLAM at pinned commits, CosPlace weights
scp scripts/ec2_setup.sh swarm.repos ubuntu@<ec2>:~/
ssh ubuntu@<ec2> 'bash ~/ec2_setup.sh ~/swarm.repos'          # ~15 min

# 2. OpenCV 5 venv next to ROS (rclpy + cv2 5.0.0 in one interpreter)
ssh ubuntu@<ec2> 'cd ~/OpenCVH && bash scripts/setup_cv5env.sh'

# 3. EuRoC footage (Vicon Room 1: 5 GB unpacked; Machine Hall MH_01-03: 6.6 GB)
ssh ubuntu@<ec2> 'pip install --user --break-system-packages remotezip==0.12.3 &&
  python3 ~/OpenCVH/scripts/fetch_euroc.py vicon_room1 V1_01_easy V1_02_medium V1_03_difficult'

# 4. Run 3 robots (~15 min at quarter speed); calibration is generated per sequence.
#    Optional failure injection as the 5th argument, e.g. "r1:blank:0.40:0.47,r2:shake:0.60:0.66"
ssh ubuntu@<ec2> 'cd ~/OpenCVH && SEQS=V1_01_easy,V1_02_medium,V1_03_difficult bash scripts/run_euroc.sh myrun 0.25 2 900'

# 5. Evaluate (copy ~/runs/myrun and the EuRoC ground-truth CSVs to gt/)
python -m eval.cslam_eval runs/myrun gt          # ATE per robot and joint
python -m eval.plot_merge out.png runs/myrun:title
python -m eval.coverage_pr runs/myrun gt         # precision / recall
python -m eval.latency runs/myrun runs/myrun/coverage.db
python -m eval.failure_report runs/myrun runs/myrun/coverage.db

# 6. Coordinator: replay a run through Bedrock (or --rules-only, free)
python -m coordinator.main runs/myrun [--speed 2] [--rules-only] [--wait-approvals]

# 7. Public demo on EC2: coordinator replay + dashboard + HTTPS tunnel (prints URL and token)
ssh ubuntu@<ec2> 'cd ~/OpenCVH && setsid bash scripts/serve_demo.sh ~/runs/myrun 2 < /dev/null'
ssh ubuntu@<ec2> 'cd ~/OpenCVH && setsid bash scripts/serve_demo.sh restart ~/runs/myrun 2 < /dev/null'  # same URL
ssh ubuntu@<ec2> 'bash ~/OpenCVH/scripts/serve_demo.sh stop'

# Tests
python -m pytest -q tests
```

**Pinned versions** (verified on 2026-10-22/24): OpenCV 5.0.0 (`opencv-contrib-python-headless==5.0.0.93`),
ROS 2 Jazzy, RTAB-Map 0.23.13, GTSAM 4.2.0 (`ros-jazzy-gtsam`), PyTorch 2.14.1+cpu, numpy 1.26.4 on
EC2, Swarm-SLAM commits in `swarm.repos`, Bedrock `us.amazon.nova-2-lite-v1:0`, cloudflared 2026.10.0.
Python packages for the cslam nodes are pinned in `scripts/ec2_setup.sh`.

## Limitations

- **No real cameras or house.** All results come from EuRoC footage (a drone in a room and in an
  industrial hall, grayscale, from an unofficial Hugging Face mirror). The live Wi-Fi capture, the Mac relay
  and checkerboard calibration (spec Phase 2) are not built.
- **AWS free plan.** DynamoDB, the load balancer and Lambda are blocked on this account, and
  Anthropic models are refused on Bedrock. So: SQLite instead of DynamoDB, a Cloudflare tunnel
  instead of an ALB, Amazon Nova instead of Claude. The COOL / Graviton benchmark was not run.
- **2 vCPU.** Three robots only keep up at quarter speed. Latency in wall-clock time is about 4×
  the data time.
- **Recall is low in the hall (0.22–0.375; 0.667 in the room).** Coverage is only claimed at inter-robot
  loop closures, which Swarm-SLAM reports at most once per 5 s. The dashboard shows where paths cross, but doesn't claim them.
- **2 of 6 planted look-alike closures still auto-cover.** Odometry auto-reset keeps the map
  complete after a failure, but loses the motion during the outage (0.34 m joint ATE vs 0.08 m).
- The quick-tunnel URL changes when the tunnel restarts.

## Credit and license

This project builds on **Swarm-SLAM** (P.-Y. Lajoie and G. Beltrame, *Swarm-SLAM: Sparse
Decentralized Collaborative Simultaneous Localization and Mapping Framework for Multi-Robot
Systems*, IEEE RA-L 2024; MIT license, https://github.com/MISTLab/Swarm-SLAM), **RTAB-Map**,
**GTSAM**, **CosPlace** (Berton et al., CVPR 2022) and the **EuRoC MAV dataset** (Burri et al.,
IJRR 2016). This team built the OpenCV 5 front end, the ROS bridge without cv_bridge, the event
recorder, the Bedrock coordinator and its guardrails, the dashboard, the failure injection and
all evaluation code. Swarm-SLAM itself is used unmodified.
