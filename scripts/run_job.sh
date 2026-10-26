#!/usr/bin/env bash
# Compute for one upload job. Usage: DATA_ROOT=.. CALIB_DIR=.. CSLAM_CFG=.. bash scripts/run_job.sh <run> <rate> <max_s> <seqs>
# Same stack as run_euroc.sh; also starts the rules-only coordinator (--follow). No Bedrock/Nova.
set -u
RUN=$1; RATE=$2; MAX=$3; SEQS=$4
OUT=${RUNS_DIR:-$HOME/runs}/$RUN
mkdir -p "$OUT"
set +u; source /opt/ros/jazzy/setup.bash; source "$HOME/swarm_ws/install/setup.bash"; set -u
cd "$(dirname "$0")/.."
cp "config/$CSLAM_CFG" "$OUT/"
echo "run=$RUN rate=$RATE skip=1 seqs=$SEQS live=1 start=$(date -Is)" > "$OUT/params.txt"
~/cv5env/bin/python -m coordinator.main "$OUT" --follow --rules-only > "$OUT/coordinator.log" 2>&1 &
COORD=$!
timeout "$MAX" ros2 launch launch/euroc_swarm.launch.py run:="$RUN" rate:="$RATE" skip:=1 robots:=$(echo "$SEQS" | tr , '\n' | wc -l) \
  sequences:="$SEQS" cslam_config:="$CSLAM_CFG" data_root:="$DATA_ROOT" calib_dir:="$CALIB_DIR" > "$OUT/launch.log" 2>&1
RC=$?   # 124 = the wall-clock cap (timeout) killed the stack, anything else = the launch ended by itself
kill $COORD 2>/dev/null
cp -r "$HOME/runs/cslam" "$OUT/cslam_logs" 2>/dev/null || true
echo "end=$(date -Is) exit=$RC" >> "$OUT/params.txt"
echo "exit=$RC" > "$OUT/run.done"
