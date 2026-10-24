#!/usr/bin/env bash
# Public demo on EC2: replay a recorded run through the Bedrock coordinator, serve the
# dashboard on localhost only, and expose it through a Cloudflare quick tunnel (HTTPS).
# Usage: bash scripts/serve_demo.sh <run_dir> [speed]
# Prints the public https://*.trycloudflare.com URL. Stop with: bash scripts/serve_demo.sh stop
set -eu
cd "$(dirname "$0")/.."
LOGS=$HOME/demo_logs
if [ "${1:-}" = "stop" ]; then
  pkill -f "uvicorn dashboard.app" || true; pkill -f "coordinator.main" || true; pkill -f "cloudflared tunnel" || true
  echo stopped; exit 0
fi
# restart: new code / fresh replay behind the SAME tunnel, so the public URL doesn't change.
RESTART=0
if [ "${1:-}" = "restart" ]; then RESTART=1; shift; fi
RUN=$1; SPEED=${2:-1}
mkdir -p "$LOGS"
[ -f "$LOGS/approval_token" ] || head -c 18 /dev/urandom | base64 | tr -d '/+=' > "$LOGS/approval_token"
if [ $RESTART = 1 ]; then pkill -f "uvicorn dashboard.app" || true; pkill -f "coordinator.main" || true; sleep 1
elif pgrep -f "cloudflared tunnel" > /dev/null; then
  # A second start would add another tunnel and coordinator next to the running ones.
  echo "a demo is already running: use 'restart' to switch runs (same URL), or 'stop' first"
  grep -oE "https://[a-z0-9-]+\.trycloudflare\.com" "$LOGS/tunnel.log" | head -1
  exit 1
fi
rm -f "$RUN/coverage.db" "$RUN/coverage.db-wal" "$RUN/coverage.db-shm"

# Low priority so a live SLAM run on the same 2 vCPUs always wins.
RUN_DIR=$RUN APPROVAL_TOKEN=$(cat "$LOGS/approval_token") nohup nice -n 10 \
  ~/cv5env/bin/uvicorn dashboard.app:app --host 127.0.0.1 --port 8000 > "$LOGS/dashboard.log" 2>&1 &
nohup nice -n 10 ~/cv5env/bin/python -m coordinator.main "$RUN" --speed "$SPEED" --wait-approvals \
  > "$LOGS/coordinator.log" 2>&1 &
if [ $RESTART = 1 ]; then
  echo "restarted dashboard + coordinator; tunnel unchanged"; exit 0
fi
nohup cloudflared tunnel --no-autoupdate --url http://127.0.0.1:8000 > "$LOGS/tunnel.log" 2>&1 &

for _ in $(seq 30); do
  URL=$(grep -oE "https://[a-z0-9-]+\.trycloudflare\.com" "$LOGS/tunnel.log" | head -1 || true)
  [ -n "$URL" ] && break; sleep 1
done
echo "dashboard: ${URL:-not ready, see $LOGS/tunnel.log}"
echo "approval token: $(cat "$LOGS/approval_token")"
