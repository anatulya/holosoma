"""Joint position control for robots with coupled (mimic) joints, e.g. G1 + Inspire hands.

The policy acts on the independent joints only; each coupled joint follows its parent as
q_follower = offset + multiplier * q_parent (the URDF <mimic> relation). The URDF keeps the
followers as plain revolute joints (no mimic constraint), so the coupling is applied to the PD
targets here: a follower's default angle and action scale are set to the coupling of its
parent's, which makes its action equal its parent's. PD control, clipping, delay and torque
randomization are JointPositionActionTerm's, unchanged, over all joints.

params:
    couplings_file: JSON list of {"joint", "parent", "multiplier", "offset"} (written by
        formhoi/build_inspire_urdf.py next to the URDF), resolved like other holosoma paths.
"""

from __future__ import annotations

import json
from typing import TYPE_CHECKING, Any

import torch

from holosoma.managers.action.terms.joint_control import JointPositionActionTerm
from holosoma.utils.path import resolve_path

if TYPE_CHECKING:
    from holosoma.config_types.action import ActionTermCfg


class CoupledJointPositionActionTerm(JointPositionActionTerm):
    def __init__(self, cfg: ActionTermCfg, env: Any):
        super().__init__(cfg, env)
        with open(resolve_path(cfg.params["couplings_file"])) as f:
            couplings = json.load(f)
        names = list(env.dof_names)
        followers = {c["joint"]: c for c in couplings if c["joint"] in names}
        self._independent = torch.tensor(
            [i for i, n in enumerate(names) if n not in followers], dtype=torch.long, device=env.device
        )
        # Column of the policy action each joint reads.
        col = {int(j): k for k, j in enumerate(self._independent.tolist())}
        self._source = torch.tensor(
            [col.get(i, -1) if n not in followers else col[names.index(followers[n]["parent"])]
             for i, n in enumerate(names)],
            dtype=torch.long, device=env.device,
        )  # fmt: skip
        default = env.robot_config.init_state.default_joint_angles
        for n, c in followers.items():
            i, p = names.index(n), names.index(c["parent"])
            self.action_scales[i] = c["multiplier"] * self.action_scales[p]
            expected = c["offset"] + c["multiplier"] * default[c["parent"]]
            actual = default[n]
            if abs(expected - actual) > 1e-6:
                raise ValueError(
                    f"Default angle of coupled joint '{n}' is {actual}, its coupling gives {expected}; "
                    "set it in init_state.default_joint_angles."
                )

    @property
    def action_dim(self) -> int:
        return len(self._independent)

    def process_actions(self, actions: torch.Tensor) -> None:
        super().process_actions(actions[:, self._source])
