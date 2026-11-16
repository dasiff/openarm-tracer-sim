#!/bin/bash
# usage: bash scripts/safety_filter_speed_run.sh <log name (no folder, no .log)> <command...>
# One heavy job of the safety filter speed work, under the task's memory rules: waits until no other heavy job runs and free swap is at
# least MIN_SWAP_MB (gives up after WAIT_MIN minutes, exit 75), then runs the command through experiments/run_capped.sh (6 GB cap)
# with a time limit of CAP_MIN minutes. The log goes to data/logs/<name>_<stamp>.log; prints the log path, the exit code and the last 20 lines.
MIN_SWAP_MB=${MIN_SWAP_MB:-1500}   # MB of free swap needed before a run starts (task rule: 1.5 GB)
WAIT_MIN=${WAIT_MIN:-30}           # minutes to wait for memory or for another job before giving up
CAP_MIN=${CAP_MIN:-45}             # minutes: time limit of one run (task rule)
cd "$(dirname "$0")/.."
NAME=$1; shift
mkdir -p data/logs
LOG=data/logs/${NAME}_$(date +%Y%m%d_%H%M%S).log
for i in $(seq 1 $((WAIT_MIN * 2))); do
  SWAP=$(free -m | awk '/^Swap/{print $4}')
  BUSY=$(pgrep -fc '^python.*(safety_filter|demo_video|_test\.py)')
  if [ "$SWAP" -ge "$MIN_SWAP_MB" ] && [ "$BUSY" -eq 0 ]; then break; fi
  if [ "$i" -eq $((WAIT_MIN * 2)) ]; then echo "NOT STARTED: free swap $SWAP MB, other jobs $BUSY ($NAME)"; exit 75; fi
  sleep 30
done
free -h | sed -n 2,3p
bash experiments/run_capped.sh "$LOG" timeout $((CAP_MIN * 60)) "$@"
RC=$(grep -o '=== exit [0-9]*' "$LOG" | tail -1 | awk '{print $3}')
echo "LOG $LOG"; echo "EXIT $RC"; tail -20 "$LOG" | cut -c1-300
exit "${RC:-1}"
