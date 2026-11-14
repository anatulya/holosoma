#!/bin/bash
# Train holosoma's whole-body tracking (with object) on all clips of one FORM-HOI object at once,
# G1 + Inspire DFQ: one policy per object, as our mjlab runs (instead of train_dex.sh's one per clip).
#
#   formhoi/train_dex_object.sh <name> <motion dir> <object urdf> [extra holosoma args...]
#   e.g. formhoi/train_dex_object.sh box10 holosoma/data/motions/g1_inspire/formhoi_box10 \
#          holosoma/data/scene_objects/formhoi/giant_cardboard_box/giant_cardboard_box.urdf \
#          logger:wandb --training.num-envs=4096
#
# Paths are relative to src/holosoma. Every clip in <motion dir> must use the same object.
# Run from an environment set up by scripts/setup_isaacsim.sh.
set -eu
REPO=$(cd "$(dirname "$0")/.." && pwd)
name=$1; motion_dir=$2; object=$3; shift 3
cd "$REPO/src/holosoma"
[ -n "$(ls $motion_dir/*.npz 2>/dev/null)" ] || { echo "no .npz in $motion_dir (from $PWD)" >&2; exit 1; }
[ -f "$object" ] || { echo "no $object (from $PWD)" >&2; exit 1; }
exec python holosoma/train_agent.py \
  exp:g1-inspire-wbt-w-object \
  --training.name "g1_inspire_wbt_$name" \
  --command.setup_terms.motion_command.params.motion_config.motion_dir="$motion_dir" \
  --scene.rigid-objects.object.urdf-file "$object" \
  "$@"
