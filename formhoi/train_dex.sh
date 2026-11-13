#!/bin/bash
# Train holosoma's whole-body tracking (with object) on one FORM-HOI clip, G1 + Inspire DFQ.
# One policy per clip, as OmniRetarget does.
#
#   formhoi/train_dex.sh <task> [extra holosoma args...]
#   e.g. formhoi/train_dex.sh fd00_baseball_bat_balance_06 logger:wandb --training.num-envs=4096
#
# <task> is a line of formhoi/data/rl_clips.txt (made by formhoi/prepare_rl_data.sh, or unpacked
# from the data bundle). Run from an environment set up by scripts/setup_isaacsim.sh.
set -eu
REPO=$(cd "$(dirname "$0")/.." && pwd)
t=$1; shift
line=$(grep "^$t " "$REPO/formhoi/data/rl_clips.txt") || { echo "no clip '$t' in formhoi/data/rl_clips.txt" >&2; exit 1; }
read -r _ motion object <<< "$line"
cd "$REPO/src/holosoma"
exec python holosoma/train_agent.py \
  exp:g1-inspire-wbt-w-object \
  --training.name "g1_inspire_wbt_$t" \
  --command.setup_terms.motion_command.params.motion_config.motion_file="$motion" \
  --scene.rigid-objects.object.urdf-file "$object" \
  "$@"
