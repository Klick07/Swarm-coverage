#!/usr/bin/env bash
# Run the 3-robot EuRoC experiment on EC2 and keep everything under ~/runs/<run>/.
# Usage: [SEQS=V1_01_easy,V1_02_medium,V1_03_difficult] bash scripts/run_euroc.sh <run> [rate] [skip] [max_seconds] [perturb]
set -u
RUN=$1; RATE=${2:-0.25}; SKIP=${3:-2}; MAX=${4:-900}; PERTURB=${5:-}
SEQS=${SEQS:-MH_01_easy,MH_02_easy,MH_03_medium}
OUT=$HOME/runs/$RUN
mkdir -p "$OUT"
set +u; source /opt/ros/jazzy/setup.bash; source "$HOME/swarm_ws/install/setup.bash"; set -u
cd "$(dirname "$0")/.."
mkdir -p calib
for s in ${SEQS//,/ }; do  # calibration per sequence, from its own sensor.yaml
  [ -f "calib/$s.yaml" ] || ~/cv5env/bin/python -m frontend.euroc_to_calib "$HOME/data/euroc/$s/mav0" "calib/$s.yaml"
done
CSLAM_CFG=${CSLAM_CFG:-cslam_stereo.yaml}
cp config/$CSLAM_CFG "$OUT/"
git rev-parse --short HEAD > "$OUT/commit.txt" 2>/dev/null || true
echo "run=$RUN rate=$RATE skip=$SKIP seqs=$SEQS perturb=$PERTURB start=$(date -Is)" > "$OUT/params.txt"

( while sleep 30; do echo "== $(date +%T)"; top -b -n1 -o %CPU | sed -n 8,14p | awk '{print $9, $10, $12}'; free -m | sed -n 2,3p; done ) > "$OUT/cpu.log" &
MON=$!
# ros2 launch rejects an empty "perturb:=", so only pass it when set.
EXTRA=(); [ -n "$PERTURB" ] && EXTRA=(perturb:="$PERTURB")
timeout "$MAX" ros2 launch launch/euroc_swarm.launch.py run:="$RUN" rate:="$RATE" skip:="$SKIP" robots:=3 \
  sequences:="$SEQS" cslam_config:="$CSLAM_CFG" "${EXTRA[@]}" > "$OUT/launch.log" 2>&1
kill $MON
cp -r "$HOME/runs/cslam" "$OUT/cslam_logs" 2>/dev/null || true
echo "end=$(date -Is)" >> "$OUT/params.txt"
echo RUN_DONE
