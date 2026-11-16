#!/bin/bash
# Runs one command alone under the 6 GB cap with GNU time -v and a 5 s sampler (process-tree RSS of python and ffmpeg, swap, system CPU busy %), as run_export.sh.
#
#   bash experiments/video_export_timing/run_any.sh <log> <command...>
cd "$(dirname "$0")/../.."
source .venv/bin/activate
export MUJOCO_GL=egl
LOG=$1; shift
mkdir -p "$(dirname "$LOG")"
{ echo "=== started $(date '+%F %T')"; echo "=== git $(git rev-parse --short HEAD) $(git status --short | wc -l) dirty files"; echo "=== free -m before"; free -m; echo "=== command: $*"; } > "$LOG"
sampler() {
    local prev_idle=0 prev_total=0
    while true; do
        read -r _ u n s i w q sq st _ < /proc/stat
        local total=$((u + n + s + i + w + q + sq + st)) idle=$((i + w)) busy="n/a"
        if [ "$prev_total" -gt 0 ]; then
            local dt=$((total - prev_total)) di=$((idle - prev_idle))
            [ "$dt" -gt 0 ] && busy=$(( (dt - di) * 100 / dt ))
        fi
        prev_total=$total; prev_idle=$idle
        local rss=$(ps -eo rss,args | grep -E "python.*experiments/|ffmpeg-linux" | grep -v grep | awk '{s+=$1} END {printf "%.0f", s/1024}')
        echo "SAMPLE $(date +%T) tree_rss_mb=${rss:-0} swap_used_mb=$(free -m | awk '/Swap/ {print $3}') cpu_busy_pct=$busy" >> "$LOG"
        sleep 5
    done
}
sampler &
SAMPLER=$!
systemd-run --user --scope -q -p MemoryMax=6G -p ManagedOOMPreference=omit /usr/bin/time -v "$@" >> "$LOG" 2>&1
RC=$?
kill $SAMPLER 2>/dev/null
{ echo "=== exit $RC at $(date '+%F %T')"; echo "=== free -m after"; free -m
  echo "=== peak tree RSS MB: $(grep SAMPLE "$LOG" | sed 's/.*tree_rss_mb=\([0-9]*\).*/\1/' | sort -n | tail -1)"
  echo "=== mean system CPU busy %: $(grep SAMPLE "$LOG" | sed 's/.*cpu_busy_pct=\([0-9]*\).*/\1/' | grep -E '^[0-9]+$' | awk '{s+=$1; n++} END {if (n) printf "%.1f", s/n}')"
  echo JOB_DONE; } >> "$LOG"
