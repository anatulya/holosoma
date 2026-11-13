#!/bin/bash
# Training data for g1_inspire_wbt_w_object from the dexterous retargets:
#   - each demo_results/g1_dex/.../<task>_original.npz -> holosoma motion file (50 fps, holosoma's
#     own data_conversion/convert_data_format_mj.py with --robot g1_dex)
#     at src/holosoma/holosoma/data/motions/g1_inspire/formhoi/<task>.npz
#   - its object (models/<obj>/<obj>.{urdf,obj}, the URDF the retargeter used)
#     at src/holosoma/holosoma/data/scene_objects/formhoi/<obj>/
#   - formhoi/data/rl_clips.txt: "<task> <motion file> <object urdf>" per clip (train_dex.sh reads it)
# usage: prepare_rl_data.sh [-j jobs]
set -u
REPO=$(cd "$(dirname "$0")/.." && pwd)
J=${2:-4}
PKG=$REPO/src/holosoma_retargeting/holosoma_retargeting
DATA=$REPO/src/holosoma/holosoma/data
MOT=$DATA/motions/g1_inspire/formhoi; OBJ=$DATA/scene_objects/formhoi; mkdir -p $MOT $OBJ $REPO/formhoi/data
PY=$REPO/.venv-rt/bin/python
export OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2 MKL_NUM_THREADS=2 MUJOCO_GL=egl
one() {
  t=$1; o=$(grep "^$t " $REPO/formhoi/jobs_dex.txt | awk '{print $2}')
  [ -f $MOT/$t.npz ] || (cd $PKG && timeout 3600 $PY data_conversion/convert_data_format_mj.py --robot g1_dex \
    --input_file demo_results/g1_dex/object_interaction/formhoi/${t}_original.npz --output_fps 50 \
    --output_name $MOT/$t.npz --data_format smplh_dex --object_name $o --has_dynamic_object --once --headless \
    > $REPO/formhoi/data/convert_$t.log 2>&1)
  mkdir -p $OBJ/$o && cp $PKG/models/$o/$o.urdf $PKG/models/$o/$o.obj $OBJ/$o/
  [ -f $MOT/$t.npz ] && echo "$t holosoma/data/motions/g1_inspire/formhoi/$t.npz holosoma/data/scene_objects/formhoi/$o/$o.urdf" \
    || echo "CONVERT_FAIL $t" >&2
}
export -f one; export REPO PKG MOT OBJ PY
ls $PKG/demo_results/g1_dex/object_interaction/formhoi/ | sed 's/_original.npz$//' | sort \
  | xargs -P$J -I{} bash -c 'one {}' | sort > $REPO/formhoi/data/rl_clips.txt
echo "$(wc -l < $REPO/formhoi/data/rl_clips.txt) clips -> formhoi/data/rl_clips.txt"
