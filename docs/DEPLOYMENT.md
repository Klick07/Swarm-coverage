# Deployment guide: Swarm coverage from scratch

For anyone who starts with this repository, a laptop and an
AWS account, and ends with the full system running: 3-robot Swarm-SLAM on EuRoC, the Bedrock
coordinator, and the public dashboard with uploads. Follow the steps in order. Each step ends
with a **check**. Don't continue until the check passes.

**Owner actions** are steps only the account owner can take (credentials, spending,
console clicks, making things public). Confirm each one before continuing.

Time: about 1.5 h, mostly unattended (builds, download, a 15 min run). Cost: about $0.12/h for
EC2 plus cents for Bedrock. Repository: https://github.com/Klick07/Swarm-coverage

---

## 0. What you need

| Item | Notes |
|---|---|
| Laptop: macOS or Linux with `git`, `ssh`, `python3` (3.11+), `curl` | Windows: use WSL |
| AWS account you own | Free plan works with the workarounds below |
| AWS CLI v2 on the laptop | `brew install awscli` or https://aws.amazon.com/cli/ |
| About 1.5 h | |

Fixed names used below (keep them; scripts assume these paths on EC2):

| Thing | Value |
|---|---|
| EC2 user / repo path | `ubuntu` / `~/OpenCVH` |
| SSH key file (laptop) | `~/.ssh/swarm-slam.pem` |
| IAM role on the instance | `swarm-slam-ec2-role` |
| Bedrock model / region | `us.amazon.nova-2-lite-v1:0` / `us-east-1` (set in `coordinator/agent.py`) |
| Data / runs / uploads on EC2 | `~/data/euroc`, `~/runs`, `~/uploads` |

---

## 1. AWS access — OWNER ACTION

The account owner:

1. **Create an IAM user** for the deployment (not root) with the least privilege the steps below need. Do not attach account-wide `*FullAccess` policies. The repository code calls only Bedrock (`bedrock-runtime`, from the coordinator, and only when Nova is enabled). It does not use S3 or CloudWatch. The deployment user needs:
   - (Untested, see the note in item 3.) EC2 actions for launching and describing one instance, security group and key pair in the deployment region (`ec2:RunInstances`, `ec2:Describe*`, `ec2:CreateSecurityGroup`, `ec2:AuthorizeSecurityGroupIngress`, `ec2:CreateKeyPair`, `ec2:CreateTags`, `ec2:StopInstances`, `ec2:StartInstances`, `ec2:TerminateInstances`), or the console path (Path B in step 2) with no AWS CLI access at all.
   - `ssm:GetParameter` limited to `arn:aws:ssm:<region>::parameter/aws/service/canonical/*`, only for the AMI lookup in step 2. Skip it if you pick the AMI in the console.
   - Permission to hand the instance role to EC2:
   ```json
   {"Version": "2012-10-17", "Statement": [{"Effect": "Allow", "Action": ["iam:PassRole", "iam:GetInstanceProfile"],
     "Resource": ["arn:aws:iam::<account-id>:role/swarm-slam-ec2-role", "arn:aws:iam::<account-id>:instance-profile/swarm-slam-ec2-role"]}]}
   ```
2. **Create an access key** for that user (type "CLI"), then run `aws configure` **themselves**
   in the deployer's terminal, with region `us-east-1` and output `json`. Never paste the key into chat or a shared document.
3. **Create the instance role**: IAM → Roles → Create role → AWS service → **EC2**, attach
   a policy that allows only `bedrock:InvokeModel` (the Converse call is authorised by this action) on the model the coordinator uses, `us.amazon.nova-2-lite-v1:0` and its underlying foundation-model ARNs, name it `swarm-slam-ec2-role`. Skip this role entirely if you run the rules-only coordinator. **These policies are untested.** They come from reading the code, not from a deployment that used them. Check the current Bedrock model-access and ARN requirements in the AWS console, and expect to adjust them.
   (It must be a **role**, not a user. Only roles appear in EC2's "Modify IAM role" list.)
4. Optionally set a budget alert (Billing → Budgets).

**Check:**
```bash
aws sts get-caller-identity                                   # prints the IAM user ARN
aws bedrock-runtime converse --region us-east-1 --model-id us.amazon.nova-2-lite-v1:0 \
  --messages '[{"role":"user","content":[{"text":"Reply with: ready"}]}]' \
  --inference-config '{"maxTokens":10}' --query 'output.message.content[0].text'   # "ready"
aws ec2 describe-vpcs --region <region> --query 'length(Vpcs)'                     # see below
```
- If Bedrock says "**not available for this account**" for Anthropic models, that's expected on the
  AWS Free plan. The coordinator uses **Amazon Nova**, which works.
- If `describe-vpcs` fails with "**explicit deny in a service control policy**", the account's
  organization blocks EC2 for IAM users (this happens on Free-plan accounts). Use **path B** in step 2.

---

## 2. Launch the EC2 instance — OWNER ACTION (spends money)

Specs: **Ubuntu Server 24.04 LTS, x86_64** (not 26.04, not ARM) · **m7i-flex.large** (2 vCPU, 8 GB;
works on the Free plan; larger is faster) · **30 GB or more gp3 (48 GB was used for the later runs)** · security group: **SSH from the owner's IP only**
and **HTTPS** · key pair **`swarm-slam`** (RSA, .pem) · IAM role **`swarm-slam-ec2-role`**.

**Path A: the deployer's IAM user can use EC2.** With the owner's approval of the spend:
```bash
REGION=ap-southeast-2   # any region; Bedrock is called in us-east-1 regardless
aws ec2 create-key-pair --region $REGION --key-name swarm-slam --query KeyMaterial --output text > ~/.ssh/swarm-slam.pem
chmod 400 ~/.ssh/swarm-slam.pem
AMI=$(aws ssm get-parameter --region $REGION --query Parameter.Value --output text \
  --name /aws/service/canonical/ubuntu/server/24.04/stable/current/amd64/hvm/ebs-gp3/ami-id)
SG=$(aws ec2 create-security-group --region $REGION --group-name swarm-slam --description swarm-slam --query GroupId --output text)
aws ec2 authorize-security-group-ingress --region $REGION --group-id $SG --protocol tcp --port 22 --cidr $(curl -s https://checkip.amazonaws.com)/32
aws ec2 authorize-security-group-ingress --region $REGION --group-id $SG --protocol tcp --port 443 --cidr 0.0.0.0/0
aws ec2 run-instances --region $REGION --image-id $AMI --instance-type m7i-flex.large --key-name swarm-slam \
  --security-group-ids $SG --iam-instance-profile Name=swarm-slam-ec2-role \
  --block-device-mappings 'DeviceName=/dev/sda1,Ebs={VolumeSize=30,VolumeType=gp3}' \
  --tag-specifications 'ResourceType=instance,Tags=[{Key=Name,Value=swarm-slam}]' --query 'Instances[0].InstanceId'
```
(If `--iam-instance-profile` fails because the profile doesn't exist yet, the console-created role
has one with the same name; retry a minute later, or attach the role afterwards as in path B, step 3.)

**Path B: EC2 is blocked for the IAM user** (the SCP case in step 1). The owner does it in the console:
1. EC2 → Launch instance with the specs above; create key pair `swarm-slam` (.pem); set SSH source to **My IP**; tick HTTPS.
2. Move the key: `mv ~/Downloads/swarm-slam.pem ~/.ssh/ && chmod 400 ~/.ssh/swarm-slam.pem`
3. Select the instance → Actions → Security → **Modify IAM role** → `swarm-slam-ec2-role` → Update.
4. Note the **Public IPv4 address**.

**Check** (export the IP once; it changes on every stop/start):
```bash
export EC2=<public-ip>
ssh -i ~/.ssh/swarm-slam.pem -o StrictHostKeyChecking=accept-new ubuntu@$EC2 \
  'lsb_release -ds; uname -m; nproc; TOKEN=$(curl -s -X PUT http://169.254.169.254/latest/api/token -H "X-aws-ec2-metadata-token-ttl-seconds: 60"); curl -s -H "X-aws-ec2-metadata-token: $TOKEN" http://169.254.169.254/latest/meta-data/iam/security-credentials/'
# Expect: Ubuntu 24.04.x LTS · x86_64 · 2 · swarm-slam-ec2-role
```
If SSH times out, the instance is stopped, the IP changed, or the owner's own IP changed (security
group → edit SSH rule → My IP).

---

## 3. Install on EC2 (~20 min, unattended)

All commands from the laptop; `$EC2` as above. Run long steps detached so a dropped SSH session
can't kill them, and **always pass `< /dev/null`** so the local shell returns.

```bash
ssh -i ~/.ssh/swarm-slam.pem ubuntu@$EC2 'git clone https://github.com/Klick07/Swarm-coverage.git ~/OpenCVH' < /dev/null
ssh -i ~/.ssh/swarm-slam.pem ubuntu@$EC2 'setsid nohup bash -c "bash ~/OpenCVH/scripts/ec2_setup.sh ~/OpenCVH/swarm.repos || echo SETUP_FAILED" < /dev/null > ~/setup.log 2>&1 &' < /dev/null
# poll every ~60 s until the last line is SETUP_DONE or SETUP_FAILED:
ssh -i ~/.ssh/swarm-slam.pem ubuntu@$EC2 'tail -1 ~/setup.log' < /dev/null
```
`ec2_setup.sh` adds 4 GB swap and the ROS 2 apt source, installs ROS 2 Jazzy (ros-base), RTAB-Map,
`ros-jazzy-gtsam` 4.2.0 and CPU-only PyTorch, builds Swarm-SLAM at the commits in `swarm.repos`, and
downloads the CosPlace weights (checked by sha256).

Then the OpenCV 5 environment:
```bash
ssh -i ~/.ssh/swarm-slam.pem ubuntu@$EC2 'bash ~/OpenCVH/scripts/setup_cv5env.sh' < /dev/null
# Expect the last line: cv2 5.0.0 numpy 1.26.4 rclpy ok
```

**Check:**
```bash
ssh -i ~/.ssh/swarm-slam.pem ubuntu@$EC2 'source /opt/ros/jazzy/setup.bash; source ~/swarm_ws/install/setup.bash
  ros2 pkg executables cslam; cd ~/OpenCVH && ~/cv5env/bin/python -m pytest -q tests 2>&1 | tail -1' < /dev/null
# Expect: map_manager, pose_graph_manager, loop_closure_detection_node.py, ... and "26 passed"
```

---

## 4. Get the footage (~10 min)

EuRoC MAV Vicon Room 1 (the demo environment), about 5 GB unpacked. ETH's server blocks scripted
downloads, so the script reads only the needed parts of the Hugging Face mirror:
```bash
ssh -i ~/.ssh/swarm-slam.pem ubuntu@$EC2 'python3 -m venv ~/dlenv && ~/dlenv/bin/pip install -q remotezip==0.12.3 &&
  setsid nohup ~/dlenv/bin/python ~/OpenCVH/scripts/fetch_euroc.py vicon_room1 V1_01_easy V1_02_medium V1_03_difficult \
  < /dev/null > ~/data_fetch.log 2>&1 &' < /dev/null
ssh -i ~/.ssh/swarm-slam.pem ubuntu@$EC2 'tail -2 ~/data_fetch.log; df -h / | tail -1' < /dev/null   # until FETCH_DONE
```
Optional second environment: `fetch_euroc.py machine_hall MH_01_easy MH_02_easy MH_03_medium` (6.6 GB).
**Watch the disk** (about 16 GB used after step 3 on the original 30 GB disk; later runs used a 48 GB disk and filled it). Keep at least 2 GB free.

**Check:** `ls ~/data/euroc/V1_01_easy/mav0` shows `cam0 cam1 state_groundtruth_estimate0 ...`

---

## 5. Run the 3-robot pipeline (~15 min, unattended)

```bash
ssh -i ~/.ssh/swarm-slam.pem ubuntu@$EC2 'cd ~/OpenCVH && SEQS=V1_01_easy,V1_02_medium,V1_03_difficult \
  setsid nohup bash scripts/run_euroc.sh v1_run 0.25 2 900 < /dev/null > ~/runs_v1_run.out 2>&1 &' < /dev/null
# poll until RUN_DONE (it always runs to the 900 s limit):
ssh -i ~/.ssh/swarm-slam.pem ubuntu@$EC2 'tail -1 ~/runs_v1_run.out' < /dev/null
```
The run replays each sequence at quarter speed through: OpenCV 5 front end → RTAB-Map odometry →
Swarm-SLAM, with the event recorder writing `~/runs/v1_run/events.jsonl`. Calibration files are generated
per sequence. Optional failure injection as the 5th argument: `"r1:blank:0.40:0.47,r2:shake:0.60:0.66"`.

**Check:**
```bash
ssh -i ~/.ssh/swarm-slam.pem ubuntu@$EC2 'R=~/runs/v1_run; grep -c "process started" $R/launch.log;
  grep -c "New inter-robot loop closure" $R/launch.log; wc -l < $R/events.jsonl; ls $R/frames' < /dev/null
# Expect: 19 processes · inter-robot closures > 0 (Vicon Room 1 gave 36 log lines = 12 per robot) · thousands of events · r0 r1 r2
```
If `launch.log` holds a single error line, read it. For example, a malformed launch argument means the
script was edited; `git checkout scripts/run_euroc.sh`.

---

## 6. Public dashboard — OWNER ACTION (makes a URL public)

Starting the tunnel publishes the dashboard (camera frames, map, decisions) on a public HTTPS URL.
**Get the owner's go-ahead before running this.** Some restricted environments block it; then the owner runs the command.

The setup script doesn't install `cloudflared`. Install it once, checksum-verified:
```bash
ssh -i ~/.ssh/swarm-slam.pem ubuntu@$EC2 'cd /tmp && curl -sSfL -o cf.deb https://github.com/cloudflare/cloudflared/releases/download/2026.10.0/cloudflared-linux-amd64.deb &&
  echo "9721287c28f78cb623e77c7b95a63feea87e909054d75666a0ec9eff20e5d548  cf.deb" | sha256sum -c && sudo dpkg -i cf.deb && rm cf.deb' < /dev/null
```
Start, switch, stop:
```bash
# first start: prints "dashboard: https://<random>.trycloudflare.com" and the operator token
ssh -i ~/.ssh/swarm-slam.pem ubuntu@$EC2 'cd ~/OpenCVH && setsid bash scripts/serve_demo.sh ~/runs/v1_run 2 < /dev/null' < /dev/null
# switch run or replay from the start, SAME URL:
ssh -i ~/.ssh/swarm-slam.pem ubuntu@$EC2 'cd ~/OpenCVH && setsid bash scripts/serve_demo.sh restart ~/runs/v1_run 2 < /dev/null' < /dev/null
# stop everything (the URL dies):
ssh -i ~/.ssh/swarm-slam.pem ubuntu@$EC2 'bash ~/OpenCVH/scripts/serve_demo.sh stop' < /dev/null
```
Share the **URL** with the owner. Treat the **token** (`~/demo_logs/approval_token`) as a password. It's
needed to approve, reject, upload and delete.

**Check:**
```bash
curl -s -o /dev/null -w "%{http_code}\n" https://<url>/                                     # 200
curl -s https://<url>/api/state | python3 -c "import json,sys; print(json.load(sys.stdin)['run'])"   # v1_run
curl -s -o /dev/null -w "%{http_code}\n" -X POST -F "file=@/dev/null;filename=x.mp4" https://<url>/api/uploads   # 403
```

---

## 7. Evaluate (laptop)

```bash
git clone https://github.com/Klick07/Swarm-coverage.git && cd Swarm-coverage
python3 -m venv .venv && .venv/bin/pip install opencv-contrib-python==5.0.0.93 numpy scipy matplotlib pyyaml pytest \
  boto3 fastapi==0.121.1 uvicorn==0.38.0 python-multipart==0.0.20 httpx==0.28.1
.venv/bin/python -m pytest -q tests                      # 26 passed
mkdir -p runs gt/calib
rsync -az -e "ssh -i ~/.ssh/swarm-slam.pem" ubuntu@$EC2:runs/v1_run runs/
for s in V1_01_easy V1_02_medium V1_03_difficult; do
  scp -i ~/.ssh/swarm-slam.pem ubuntu@$EC2:data/euroc/$s/mav0/state_groundtruth_estimate0/data.csv gt/$s.csv
  scp -i ~/.ssh/swarm-slam.pem ubuntu@$EC2:OpenCVH/calib/$s.yaml gt/calib/; done
.venv/bin/python -m eval.cslam_eval runs/v1_run gt                   # ATE per robot + joint
.venv/bin/python -m coordinator.main runs/v1_run --rules-only        # free; drop --rules-only to use Bedrock
.venv/bin/python -m eval.coverage_pr runs/v1_run gt                  # precision / recall
.venv/bin/python -m eval.latency runs/v1_run runs/v1_run/coverage.db
.venv/bin/python -m eval.plot_merge out.png "runs/v1_run:Vicon Room 1"
```
Reference values (docs/FINDINGS.md item 12): joint ATE about 0.21 m merged vs 0.90 m unmerged; coverage
precision 1.00, recall about 0.67. Small differences between runs are normal (timing on 2 vCPU).

---

## 8. Known pitfalls (already handled in the code; don't undo the fixes)

| Symptom | Cause → fix in place |
|---|---|
| `pip install opencv 5` fails with numpy conflict, or cv2 shows 4.6 | The OpenCV 5 wheel declares numpy ≥ 2 while rclpy needs 1.26. `setup_cv5env.sh` uses `--no-deps`; `frontend/node.py` asserts cv2 5. |
| `No module named rclpy` in our nodes | Never set `PYTHONPATH=.`; use `python -m` from the repo, or `PYTHONPATH=.:$PYTHONPATH`. |
| `'BedrockRuntime' object has no attribute 'converse'` | Ubuntu's boto3 is too old; `setup_cv5env.sh` installs current boto3 into the venv. |
| cslam Python node: `coverage.types has no attribute Tracer` | numba vs Ubuntu's coverage; `ec2_setup.sh` pins coverage 7.16.2. |
| Robots merged ~120° apart | Image frame ≠ base frame. Images are stamped in `camera_link` (identity); don't add a rotated TF. |
| `pose_timestamps*.csv` empty, eval finds nothing | `evaluation.enable_pose_timestamps_recording: true` in `config/cslam_stereo.yaml`. |
| Odometry never recovers after blur or a blank wall | `Odom/ResetCountdown=3` in the launch file. |
| A Bedrock call hangs | Timeouts plus per-event rule fallback in `coordinator/agent.py`; the log says `source=rules`. |
| Uploads over 100 MB fail | Cloudflare quick-tunnel limit; the app caps at 95 MB per video. |
| Dashboard link dead | The instance was stopped. Quick-tunnel URLs change on every start; run step 6 again. |
| Two demos or two tunnels running | `serve_demo.sh` refuses a second start; use `restart`. |
| Approval images missing in the browser | Old cached page; the page is served `no-store` now. Hard-refresh once. |
| Disk full during download | V1 is about 5 GB, MH about 6.6 GB. Delete `~/swarm_ws/build`, run `sudo apt-get clean`, remove unneeded `~/data/euroc/*`. |

---

## 9. Done when

- [ ] `aws sts get-caller-identity` works; Bedrock Nova answers
- [ ] SSH to EC2 works; the instance role shows `swarm-slam-ec2-role`
- [ ] `26 passed` on EC2 and on the laptop
- [ ] A Vicon run finished with inter-robot closures > 0 and `events.jsonl` present
- [ ] `eval.cslam_eval` prints a joint ATE well below the odometry-only value
- [ ] The public URL returns 200, shows the run, and refuses an upload without the token
- [ ] The owner has the URL and the token

**Cleanup when finished:** `serve_demo.sh stop`; stop the instance (EC2 → Instance state → Stop) to end
the hourly charge, or terminate it to delete everything. Deactivate the IAM access key after the event.
