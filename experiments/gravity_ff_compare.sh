#!/bin/bash
# Part C step 1: the 8 comparison episodes, one after the other (one MuJoCo process at a time). usage: bash experiments/gravity_ff_compare.sh TAG
cd "$(dirname "$0")/.." && source .venv/bin/activate
TAG=${1:-$(date +%Y%m%d_%H%M%S)}
for variant in plain zombie_held; do
  for layout in normal moved; do
    for mode in physics feedforward; do
      name=${layout}_${variant}_${mode}
      echo "=== $name $(date +%H:%M:%S)" >> data/logs/gravity_ff_compare_${TAG}_summary.txt
      OPENARM_GRAVITY_COMP=$mode MUJOCO_GL=egl python experiments/gravity_ff_compare.py --layout $layout --variant $variant --out data/gravity_ff/${TAG}/$name > data/logs/gravity_ff_compare_${TAG}_$name.log 2>&1
      echo "exit $? $(tail -2 data/logs/gravity_ff_compare_${TAG}_$name.log | cut -c1-300)" >> data/logs/gravity_ff_compare_${TAG}_summary.txt
    done
  done
done
echo "ALL DONE $(date +%H:%M:%S)" >> data/logs/gravity_ff_compare_${TAG}_summary.txt
