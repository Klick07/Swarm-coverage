#!/usr/bin/env bash
# Run the 3-robot EuRoC experiment on EC2 and keep everything under ~/runs/<run>/.
# Usage: bash scripts/run_euroc.sh <run> [rate] [skip] [max_seconds] [perturb]
set -u
RUN=$1; RATE=${2:-0.25}; SKIP=${3:-2}; MAX=${4:-900}; PERTURB=${5:-}
OUT=$HOME/runs/$RUN
mkdir -p "$OUT"
set +u; source /opt/ros/jazzy/setup.bash; source "$HOME/swarm_ws/install/setup.bash"; set -u
cd "$(dirname "$0")/.."
cp config/cslam_stereo.yaml "$OUT/"
git rev-parse --short HEAD > "$OUT/commit.txt" 2>/dev/null || true
echo "run=$RUN rate=$RATE skip=$SKIP perturb=$PERTURB start=$(date -Is)" > "$OUT/params.txt"

( while sleep 30; do echo "== $(date +%T)"; top -b -n1 -o %CPU | sed -n 8,14p | awk '{print $9, $10, $12}'; free -m | sed -n 2,3p; done ) > "$OUT/cpu.log" &
MON=$!
timeout "$MAX" ros2 launch launch/euroc_swarm.launch.py run:="$RUN" rate:="$RATE" skip:="$SKIP" robots:=3 \
  perturb:="$PERTURB" > "$OUT/launch.log" 2>&1
kill $MON
cp -r "$HOME/runs/cslam" "$OUT/cslam_logs" 2>/dev/null || true
echo "end=$(date -Is)" >> "$OUT/params.txt"
echo RUN_DONE
