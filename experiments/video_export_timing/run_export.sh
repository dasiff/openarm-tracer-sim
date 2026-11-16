#!/bin/bash
# One timed demo-video export, run alone (one MuJoCo process at a time, 6 GB cap as experiments/run_capped.sh).
#
#   bash experiments/video_export_timing/run_export.sh <out dir> <log> [search_video.py arguments after the run folder]
#
# Writes <log> with: swap and free memory before and after, the command, GNU time -v (wall, CPU, the Python process's peak RSS), and
# a sampler line every 5 s (process-tree RSS of the Python process and its ffmpeg children in MB, swap used in MB, system CPU busy
# percent since the previous sample). The export is `python experiments/search_video.py data/demo_record/rerun_moved_20261113_001429
# --out <out dir> --name moved`, the lab's demo-video command of 2026-11-13 (docs/reports/20261113_0745_search_display_videos.md).
cd "$(dirname "$0")/../.."
source .venv/bin/activate
export MUJOCO_GL=egl
OUT=$1; LOG=$2; shift 2
RUN=data/demo_record/rerun_moved_20261113_001429
mkdir -p "$OUT" "$(dirname "$LOG")"
{ echo "=== started $(date '+%F %T')"; echo "=== git $(git rev-parse --short HEAD) $(git status --short | wc -l) dirty files"; echo "=== free -m before"; free -m; echo "=== command: python experiments/search_video.py $RUN --out $OUT --name moved $*"; } > "$LOG"

sampler() {
    local prev_idle=0 prev_total=0
    while true; do
        read -r _ u n s i w q sq st _ < /proc/stat
        local total=$((u + n + s + i + w + q + sq + st)) idle=$((i + w))
        local busy="n/a"
        if [ "$prev_total" -gt 0 ]; then
            local dt=$((total - prev_total)) di=$((idle - prev_idle))
            [ "$dt" -gt 0 ] && busy=$(( (dt - di) * 100 / dt ))
        fi
        prev_total=$total; prev_idle=$idle
        local rss=$(ps -eo rss,args | grep -E "experiments/search_video.py|ffmpeg-linux|export_timed.py|multiprocessing" | grep -v grep | awk '{s+=$1} END {printf "%.0f", s/1024}')
        local swap=$(free -m | awk '/Swap/ {print $3}')
        echo "SAMPLE $(date +%T) tree_rss_mb=${rss:-0} swap_used_mb=$swap cpu_busy_pct=$busy" >> "$LOG"
        sleep 5
    done
}
sampler &
SAMPLER=$!
systemd-run --user --scope -q -p MemoryMax=${MEMCAP:-6G} -p ManagedOOMPreference=omit /usr/bin/time -v python experiments/search_video.py "$RUN" --out "$OUT" --name moved "$@" >> "$LOG" 2>&1
RC=$?
kill $SAMPLER 2>/dev/null
{ echo "=== exit $RC at $(date '+%F %T')"; echo "=== free -m after"; free -m
  echo "=== peak tree RSS MB: $(grep SAMPLE "$LOG" | sed 's/.*tree_rss_mb=\([0-9]*\).*/\1/' | sort -n | tail -1)"
  echo "=== mean system CPU busy %: $(grep SAMPLE "$LOG" | sed 's/.*cpu_busy_pct=\([0-9]*\).*/\1/' | grep -E '^[0-9]+$' | awk '{s+=$1; n++} END {if (n) printf "%.1f", s/n}')"
  echo JOB_DONE; } >> "$LOG"
