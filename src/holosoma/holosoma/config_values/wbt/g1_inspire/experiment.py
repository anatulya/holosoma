"""Whole-body tracking with an object for the G1 + Inspire DFQ hands (FORM-HOI).

g1_inspire_wbt_w_object is g1_29dof_wbt_w_object (same PPO, rewards, observations,
terminations, randomization, curriculum) with the robot swapped for the G1 + Inspire built by
formhoi/build_inspire_urdf.py. What the hands add:

- 24 finger joints (12 per hand). The policy drives the 12 independent ones; the 12 coupled
  joints follow through CoupledJointPositionActionTerm, so actions are 29 + 12 = 41.
- Finger actuators as in our mjlab G1 + Inspire: kp 3.0, kd 0.15, effort 3.0, armature 0.002.
- The five distal finger links of each hand join the tracked bodies.

The motion file and the object URDF are per clip: formhoi/train_dex.py sets both.
"""

from __future__ import annotations

import xml.etree.ElementTree as ET
from dataclasses import replace
from pathlib import Path

from holosoma.config_types.action import ActionManagerCfg, ActionTermCfg
from holosoma.config_types.command import CommandManagerCfg
from holosoma.config_types.scene import RigidObjectConfig
from holosoma.config_values import robot as robot_values
from holosoma.config_values.wbt.g1 import command as g1_command
from holosoma.config_values.wbt.g1.experiment import g1_29dof_wbt_w_object

URDF = "g1/g1_29dof_inspire_dfq.urdf"
COUPLINGS = "@holosoma/data/robots/g1/g1_29dof_inspire_dfq.couplings.json"

_HAND = ["thumb_proximal_yaw", "thumb_proximal_pitch", "thumb_intermediate", "thumb_distal", "index_proximal",
         "index_intermediate", "middle_proximal", "middle_intermediate", "ring_proximal", "ring_intermediate",
         "pinky_proximal", "pinky_intermediate"]  # fmt: skip
_HAND_LINKS = ["thumb_proximal_base", "thumb_proximal", "thumb_intermediate", "thumb_distal", "index_proximal",
               "index_intermediate", "middle_proximal", "middle_intermediate", "ring_proximal",
               "ring_intermediate", "pinky_proximal", "pinky_intermediate"]  # fmt: skip
_TRACKED_FINGER_LINKS = ["thumb_distal", "index_intermediate", "middle_intermediate", "ring_intermediate",
                         "pinky_intermediate"]  # fmt: skip
FINGER_KP, FINGER_KD, FINGER_EFFORT, FINGER_ARMATURE = 3.0, 0.15, 3.0, 0.002
# The URDF's 0.5 rad/s would cap the fingers well below what the hand does; our mjlab setup has no
# velocity limit.
FINGER_VEL_LIMIT = 10.0


def _hand(side: str) -> list[str]:
    return [f"{side}_{j}_joint" for j in _HAND]


def _limits() -> dict[str, tuple[float, float]]:
    root = ET.parse(Path(__file__).resolve().parents[3] / "data/robots" / URDF).getroot()
    out = {}
    for j in root.findall("joint"):
        lim = j.find("limit")
        if lim is not None and j.get("name") in _hand("L") + _hand("R"):
            out[j.get("name")] = (float(lim.get("lower")), float(lim.get("upper")))
    return out


def _insert_hands(g1: list, hand_left: list, hand_right: list) -> list:
    """G1 joint lists run legs, waist, left arm (7), right arm (7); each hand follows its arm."""
    assert len(g1) == 29, len(g1)
    return g1[:22] + hand_left + g1[22:] + hand_right


_base = robot_values.g1_29dof_w_object
_lim = _limits()
_n = len(_HAND)

g1_29dof_inspire_dfq_w_object = replace(
    _base,
    num_bodies=_base.num_bodies + 2 * len(_HAND_LINKS),
    dof_obs_size=29 + 2 * _n,
    actions_dim=29 + 2 * 6,  # 6 independent joints per hand
    dof_names=_insert_hands(_base.dof_names, _hand("L"), _hand("R")),
    upper_dof_names=_base.upper_dof_names + _hand("L") + _hand("R"),
    dof_pos_lower_limit_list=_insert_hands(
        _base.dof_pos_lower_limit_list, [_lim[j][0] for j in _hand("L")], [_lim[j][0] for j in _hand("R")]
    ),
    dof_pos_upper_limit_list=_insert_hands(
        _base.dof_pos_upper_limit_list, [_lim[j][1] for j in _hand("L")], [_lim[j][1] for j in _hand("R")]
    ),
    dof_vel_limit_list=_insert_hands(_base.dof_vel_limit_list, [FINGER_VEL_LIMIT] * _n, [FINGER_VEL_LIMIT] * _n),
    dof_effort_limit_list=_insert_hands(_base.dof_effort_limit_list, [FINGER_EFFORT] * _n, [FINGER_EFFORT] * _n),
    dof_armature_list=_insert_hands(_base.dof_armature_list, [FINGER_ARMATURE] * _n, [FINGER_ARMATURE] * _n),
    dof_joint_friction_list=_insert_hands(_base.dof_joint_friction_list, [0.0] * _n, [0.0] * _n),
    body_names=_base.body_names + [f"L_{b}" for b in _HAND_LINKS] + [f"R_{b}" for b in _HAND_LINKS],
    init_state=replace(
        _base.init_state,
        default_joint_angles={**_base.init_state.default_joint_angles, **{j: 0.0 for j in _hand("L") + _hand("R")}},
    ),
    symmetry_joint_names={
        **(_base.symmetry_joint_names or {}),
        **{f"L_{j}_joint": f"R_{j}_joint" for j in _HAND},
        **{f"R_{j}_joint": f"L_{j}_joint" for j in _HAND},
    },
    control=replace(
        _base.control,
        stiffness={**_base.control.stiffness, **{j: FINGER_KP for j in _HAND}},
        damping={**_base.control.damping, **{j: FINGER_KD for j in _HAND}},
    ),
    asset=replace(_base.asset, urdf_file=URDF),
)

g1_inspire_joint_pos = ActionManagerCfg(
    terms={
        "joint_control": ActionTermCfg(
            func="holosoma.managers.action.terms.coupled_joint_control:CoupledJointPositionActionTerm",
            params={"couplings_file": COUPLINGS},
            scale=1.0,
            clip=None,
        ),
    }
)

motion_config_inspire = replace(
    g1_command.motion_config_w_object,
    body_names_to_track=g1_command.motion_config_w_object.body_names_to_track
    + [f"L_{b}" for b in _TRACKED_FINGER_LINKS]
    + [f"R_{b}" for b in _TRACKED_FINGER_LINKS],
)


def _with_motion_config(cfg: CommandManagerCfg) -> CommandManagerCfg:
    term = cfg.setup_terms["motion_command"]
    return replace(
        cfg,
        setup_terms={
            **cfg.setup_terms,
            "motion_command": replace(term, params={**term.params, "motion_config": motion_config_inspire}),
        },
    )


g1_inspire_wbt_w_object = replace(
    g1_29dof_wbt_w_object,
    training=replace(g1_29dof_wbt_w_object.training, name="g1_inspire_wbt_w_object"),
    robot=replace(
        g1_29dof_inspire_dfq_w_object,
        asset=replace(g1_29dof_inspire_dfq_w_object.asset, enable_self_collisions=True),
        init_state=replace(g1_29dof_inspire_dfq_w_object.init_state, pos=[0.0, 0.0, 0.76]),
    ),
    action=g1_inspire_joint_pos,
    command=_with_motion_config(g1_29dof_wbt_w_object.command),
    scene=replace(
        g1_29dof_wbt_w_object.scene,
        rigid_objects={"object": RigidObjectConfig(urdf_file="holosoma/data/scene_objects/boxes/large_box.urdf")},
    ),
)

__all__ = ["g1_29dof_inspire_dfq_w_object", "g1_inspire_joint_pos", "g1_inspire_wbt_w_object"]
