# FORM-HOI with G1 + Inspire DFQ hands

OmniRetarget (holosoma) on FORM-HOI clips, extended from the 29-dof G1 to our G1 + Inspire DFQ
hands. Two parts: dexterous kinematic retargeting, and holosoma's whole-body tracking RL with
the same robot.

## Training (Isaac Sim)

1. Set up holosoma's Isaac Sim environment as upstream describes (`scripts/setup_isaacsim.sh`,
   then `source scripts/source_isaacsim_setup.sh`).
2. Unpack the data bundle (`formhoi_rl_data.tar.gz`) at the repo root. It fills
   `src/holosoma/holosoma/data/motions/g1_inspire/formhoi/` (38 motion files),
   `src/holosoma/holosoma/data/scene_objects/formhoi/` (object URDFs + meshes) and
   `formhoi/data/rl_clips.txt` (task, motion file, object URDF per clip).
3. Train one clip (one policy per clip, as in OmniRetarget):

   ```bash
   formhoi/train_dex.sh fd00_baseball_bat_balance_06 logger:wandb --training.num-envs=4096
   ```

   Extra arguments go straight to `holosoma/train_agent.py`.

   One policy per object over all its clips (as our mjlab runs), e.g. the 10 box clips
   (`formhoi_box10_rl_data.tar.gz`, see `formhoi/box10_MANIFEST.md`):

       formhoi/train_dex_object.sh box10 holosoma/data/motions/g1_inspire/formhoi_box10 \
         holosoma/data/scene_objects/formhoi/giant_cardboard_box/giant_cardboard_box.urdf \
         logger:wandb --training.num-envs=4096

### What `g1_inspire_wbt_w_object` changes from `g1_29dof_wbt_w_object`

Everything holosoma trains with stays: PPO settings, rewards, observations, terminations,
randomization, curriculum, the object's URDF template (0.1 kg, as holosoma's box). Only the robot
differs:

- **URDF** `data/robots/g1/g1_29dof_inspire_dfq.urdf`, built by `formhoi/build_inspire_urdf.py`:
  holosoma's with-object G1 with its sphere hands replaced by Unitree's Inspire DFQ hands
  (`formhoi/assets/unitree_g1_inspire_dfq`, unitree_ros 41dec58). Forward kinematics match the
  MuJoCo G1 + Inspire the retargeter and our mjlab training use (< 0.01 mm).
- **Coupled fingers.** Each hand has 12 joints; 6 follow another joint
  (`q = multiplier x q_parent`, the URDF's `<mimic>`). The URDF keeps them as plain revolute
  joints and `managers/action/terms/coupled_joint_control.py` gives each follower its parent's
  action with the coupled default and scale, so the PD targets stay coupled.
  Actions: 29 + 12 = 41.
- **Finger actuators** as in our mjlab setup: kp 3.0, kd 0.15, effort 3.0, armature 0.002
  (velocity limit 10 rad/s, the URDF's 0.5 rad/s would cap the fingers).
- **Tracked bodies**: holosoma's 14 plus the 5 distal finger links of each hand.

The scene has the robot, the floor and the object only, as upstream: clips that start with the
object on a desk have no desk.

## Retargeting (MuJoCo, CPU)

`.venv-rt` is the retargeting venv (`uv venv .venv-rt && uv pip install -e src/holosoma_retargeting`).
Paths to our mjlab repo (`REACTOR`, default `/home/ant/reactor-mujoco-formhoi`) are only needed
to rebuild inputs from FORM-HOI.

- `export.py`, `floor_anchor.py`, `heights.py`: FORM-HOI -> OmniRetarget inputs (`demo_data/FORMHOI`),
  object meshes at real size, floor-anchored heights for clips that start off the floor.
- `build_dex_model.py`: MuJoCo G1 + Inspire scene models (`models/g1_dex`).
- `export_dex.py`: adds the 10 fingertips to each input (`smplh_dex`).
- `one.sh` (29 dof) / `one_dex.sh` (G1 + Inspire): retarget one clip (+ convert / render).
- `prepare_rl_data.sh`: dexterous retargets -> holosoma motion files and objects (the data bundle).

Changes to holosoma_retargeting itself: the `g1_dex` robot and `smplh_dex` format (finger
mapping), joint couplings as linear equalities in the QP (the only solver change), and the
`g1_dex` joint order for `data_conversion`. 38 of 51 clips retarget; the 13 others are infeasible
under the non-penetration constraint (thin handles: hatchet, cane, bat, racket, and three boxes).
