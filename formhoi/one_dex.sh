#!/bin/bash
# dexterous OmniRetarget (our G1 + Inspire) for one clip, then render it
# full-body and close-up: one_dex.sh <task fdNN_...> <object_name>
REPO=$(cd "$(dirname "$0")/.." && pwd); A=$REPO/formhoi
REACTOR=${REACTOR:-/home/ant/reactor-mujoco-formhoi}; RPY=${RPY:-/home/ant/reactor-mujoco/.venv/bin/python}
cd $REPO/src/holosoma_retargeting/holosoma_retargeting
t=$1; o=$2; L=$A/logs_dex; mkdir -p $L
O=$REACTOR/videos/formhoi/omniretarget_kin_dex; mkdir -p $O/full $O/closeup
out=demo_results/g1_dex/object_interaction/formhoi/${t}_original.npz
export OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2 MKL_NUM_THREADS=2 MUJOCO_GL=egl
[ -f $out ] || timeout 10800 ../../../.venv-rt/bin/python examples/robot_retarget.py --robot g1_dex --data_path demo_data/FORMHOI --task-type object_interaction --task-name $t --data_format smplh_dex --task-config.object-name $o --save_dir demo_results/g1_dex/object_interaction/formhoi > $L/$t.retarget.log 2>&1
[ -f $out ] || { echo "RETARGET_FAIL $t"; exit 0; }
$RPY $A/render_kin.py $t $o $O/full/$t.mp4 g1_dex 2.2 > $L/$t.render.log 2>&1
$RPY $A/render_kin.py $t $o $O/closeup/$t.mp4 g1_dex 0.9 >> $L/$t.render.log 2>&1
[ -f $O/closeup/$t.mp4 ] && echo "ok $t" || echo "RENDER_FAIL $t"
