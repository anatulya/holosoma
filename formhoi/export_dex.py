"""Dexterous inputs for every clip: fd<ii>_<clip>.pt = that clip's final
OmniRetarget export (jobs_final.txt: fa<ii> floor-anchored for elevated starts,
else fh<ii>) + the 10 SOMA fingertips (*End) at cols 325:355 (smplh_dex), and
height_dict key fd<ii>. Writes jobs_dex.txt (task object) for one_dex.sh.

usage: export_dex.py <holosoma_retargeting pkg dir>
"""
import os
import json, pickle, sys
from pathlib import Path
import numpy as np, torch

PKG = Path(sys.argv[1])
HERE = Path(__file__).resolve().parent
RAW = Path(os.environ.get("REACTOR", "/home/ant/reactor-mujoco-formhoi")) / Path("dataset/formhoi_soma_raw")
SEL = Path(os.environ.get("REACTOR", "/home/ant/reactor-mujoco-formhoi")) / Path("dataset/hoi_sel51")
manifest = {m["index"]: m for m in json.loads((PKG / "demo_data/FORMHOI/manifest.json").read_text())}
hd_path = PKG / "demo_data/height_dict.pkl"
hd = pickle.load(open(hd_path, "rb"))
jobs = []
for line in (HERE / "jobs_final.txt").read_text().split("\n"):
  if not line.strip():
    continue
  task, obj = line.split()
  i = int(task[2:4])
  seq = manifest[i]["seq"]
  raw = np.load(RAW / seq / f"{seq}.npz")
  names = [str(n) for n in np.load(SEL / f"{seq}.npz")["body_names"]]
  a = torch.load(PKG / "demo_data/FORMHOI" / f"{task}.pt").numpy().copy()
  assert np.allclose(a[:, 162:165], raw["body_pos_w"][:, 0]) and not np.abs(a[:, 325:355]).any()
  tips = np.stack([raw["soma_pos_w"][:, names.index(f"{s}Hand{f}End")]
                   for s in ("Left", "Right") for f in ("Thumb", "Index", "Middle", "Ring", "Pinky")], 1)
  a[:, 325:355] = tips.reshape(len(a), -1)
  out = f"fd{task[2:]}"
  torch.save(torch.from_numpy(a), PKG / "demo_data/FORMHOI" / f"{out}.pt")
  hd[f"fd{i:02d}"] = hd[f"fh{i:02d}"]
  jobs.append(f"{out} {obj}")
pickle.dump(hd, open(hd_path, "wb"))
(HERE / "jobs_dex.txt").write_text("\n".join(jobs) + "\n")
print(len(jobs), "dexterous inputs;", sum(j.split()[0] != "fd" + j.split()[0][2:] for j in jobs), "renamed")
