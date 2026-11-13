"""Export FORM-HOI clips (our SOMA raw bake) for Holosoma's OmniRetarget
`object_interaction` pipeline, in the InterMimic/OMOMO layout it reads.

Per clip i (task name fh<ii>_<clip>):
  demo_data/FORMHOI/<task>.pt   (T, 591) float: cols 162:318 the 52 SMPL-H joint
                                world positions (InterMimic order == our bake's
                                body_pos_w), cols 318:325 object pos + quat xyzw
  demo_data/height_dict.pkl     + fh<ii>: subject height (SOMA rest mesh)
  models/fh<ii>obj/             object mesh (human scale) + urdf/xml/template
                                made from their largebox files
  models/g1/g1_29dof_w_fh<ii>obj.xml   robot + object scene, the object mesh
                                scaled by smpl_scale = 1.32 / height (their
                                pipeline scales the human and object trajectory by it)

usage: export.py <holosoma_retargeting pkg dir> <seq list> <heights.json> [only_index...]
"""
import os
import json, pickle, re, shutil, sys
from pathlib import Path
import numpy as np, torch

PKG, SEQS, HEIGHTS = Path(sys.argv[1]), Path(sys.argv[2]), Path(sys.argv[3])
only = {int(a) for a in sys.argv[4:]}
RAW = Path(os.environ.get("REACTOR", "/home/ant/reactor-mujoco-formhoi")) / Path("dataset/formhoi_soma_raw")
ROBOT_HEIGHT = 1.32
seqs = SEQS.read_text().split()
heights = json.loads(HEIGHTS.read_text())

hd_path = PKG / "demo_data/height_dict.pkl"
backup = PKG / "demo_data/height_dict.orig.pkl"
if not backup.exists():
    shutil.copy(hd_path, backup)
hd = pickle.load(open(backup, "rb"))
(PKG / "demo_data/FORMHOI").mkdir(parents=True, exist_ok=True)
lb = PKG / "models/largebox"
scene_lb = (PKG / "models/g1/g1_29dof_w_largebox.xml").read_text()
manifest = []
for i, s in enumerate(seqs):
    key, obj_key = f"fh{i:02d}", f"fh{i:02d}obj"
    task = f"{key}_{s[20:]}"
    hd[key] = np.float32(heights[s]["height"])
    scale = ROBOT_HEIGHT / heights[s]["height"]
    manifest.append({"index": i, "seq": s, "task": task, "object_name": obj_key, "height": heights[s]["height"], "smpl_scale": scale})
    if only and i not in only:
        continue
    d = np.load(RAW / s / f"{s}.npz")
    assert [str(n) for n in d["body_names"]][:5] == ["Pelvis", "L_Hip", "L_Knee", "L_Ankle", "L_Toe"]
    T = d["body_pos_w"].shape[0]
    a = np.zeros((T, 591), np.float32)
    a[:, 162:318] = d["body_pos_w"].reshape(T, -1)
    a[:, 318:321] = d["obj_pos"]
    a[:, 321:325] = d["obj_quat"][:, [1, 2, 3, 0]]  # wxyz -> xyzw
    torch.save(torch.from_numpy(a), PKG / "demo_data/FORMHOI" / f"{task}.pt")
    od = PKG / "models" / obj_key
    od.mkdir(parents=True, exist_ok=True)
    shutil.copy(RAW / s / "objects" / f"{d['obj_name']}.obj", od / f"{obj_key}.obj")
    for f in ("largebox.urdf", "largebox.xml"):
        (od / f.replace("largebox", obj_key)).write_text((lb / f).read_text().replace("largebox", obj_key))
    tpl = PKG / "models/templates/largebox.urdf.jinja"
    (PKG / "models/templates" / f"{obj_key}.urdf.jinja").write_text(tpl.read_text().replace("largebox", obj_key))
    # object stays at real size, as in their largebox scene: object_interaction
    # matches the robot against the UNscaled mesh points (only the human-side
    # demo points are scaled by smpl_scale), and keeps the object's frame-0 height
    (PKG / "models/g1" / f"g1_29dof_w_{obj_key}.xml").write_text(scene_lb.replace("largebox", obj_key))
    print(f"{task}: {T} frames, height {heights[s]['height']:.3f}, scale {scale:.3f}")
pickle.dump(hd, open(hd_path, "wb"))
json.dump(manifest, open(PKG / "demo_data/FORMHOI/manifest.json", "w"), indent=1)
