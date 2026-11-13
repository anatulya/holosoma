"""(.venv-soma) Standing height of each clip's subject: the SOMA mesh at rest
pose with the clip's identity, top minus bottom (SOMA is Y-up)."""
import os
import sys, json, yaml, numpy as np, torch
sys.path.insert(0, os.environ.get("REACTOR", "/home/ant/reactor-mujoco-formhoi"))
from motion.bake_formhoi import SomaFK
seqs = open(sys.argv[1]).read().split()
fk = SomaFK("cpu")
out = {}
for s in seqs:
    d = np.load(f"/home/ant/form_hoi/{s}/soma_params.npz")
    meta = yaml.safe_load(open(f"/home/ant/form_hoi/{s}/hoi_metadata.yaml"))
    fk.layer.prepare_identity(torch.as_tensor(d["identity_coeffs"][:1], device="cpu"), scale_params=torch.as_tensor(d["scale_params"][:1], device="cpu"))
    v = fk.layer.pose(torch.zeros(1, 77, 3, device="cpu")).vertices[0].detach().cpu().numpy()
    out[s] = {"person": str(meta["person"]["id"]), "height": float(v[:, 1].max() - v[:, 1].min())}
json.dump(out, open(sys.argv[2], "w"), indent=1)
by = {}
for v in out.values(): by.setdefault(v["person"], set()).add(round(v["height"], 3))
print(by)
