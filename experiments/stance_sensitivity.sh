#!/bin/bash
# Runs the D2 sensitivity cases one after the other (one MuJoCo process at a time); a case whose summary file exists is skipped.
cd "$(dirname "$0")/.." || exit 1
source .venv/bin/activate
for c in "$@"; do
  if [ -f "data/d2/sens/${c}_summary.json" ]; then echo "case $c already done"; continue; fi
  echo "=== case $c start $(date +%T)"
  MUJOCO_GL=egl python experiments/stance_sensitivity.py --case "$c" --resume
  echo "=== case $c exit $? $(date +%T)"
done
echo "ALL DONE $(date +%T)"
