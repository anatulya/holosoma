#!/bin/bash
# retarget + convert one exported clip: one.sh <task> <object_name>
REPO=$(cd "$(dirname "$0")/.." && pwd); A=$REPO/formhoi
REACTOR=${REACTOR:-/home/ant/reactor-mujoco-formhoi}; RPY=${RPY:-/home/ant/reactor-mujoco/.venv/bin/python}
cd $REPO/src/holosoma_retargeting/holosoma_retargeting
t=$1; o=$2; L=$A/logs; mkdir -p $L
out=demo_results/g1/object_interaction/formhoi/${t}_original.npz
conv=converted_res/object_interaction/${t}_mj_fps50.npz
export OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2 MKL_NUM_THREADS=2 MUJOCO_GL=egl
[ -f $out ] || timeout 7200 ../../../.venv-rt/bin/python examples/robot_retarget.py --data_path demo_data/FORMHOI --task-type object_interaction --task-name $t --data_format smplh --task-config.object-name $o --save_dir demo_results/g1/object_interaction/formhoi > $L/$t.retarget.log 2>&1
[ -f $out ] || { echo "RETARGET_FAIL $t"; exit 0; }
[ -f $conv ] || timeout 1800 ../../../.venv-rt/bin/python data_conversion/convert_data_format_mj.py --input_file $out --output_fps 50 --output_name $conv --data_format smplh --object_name $o --has_dynamic_object --once --headless > $L/$t.convert.log 2>&1
[ -f $conv ] && echo "ok $t" || echo "CONVERT_FAIL $t"
