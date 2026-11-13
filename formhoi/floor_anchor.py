"""Floor-rest anchor for OmniRetarget's object height scaling.

Their preprocessing maps object z as  z' = z0 + s (z - z0)  (frame-0 height kept),
which assumes the object starts resting on the floor. We want the anchor at the
object's floor-rest height h instead:  z' = s z + (1 - s) h,  where h = centre
height above its lowest point at the frame-0 orientation (from the mesh).
Feeding z_in = z + (1 - s)(h - z0) into their rule gives exactly that.

usage: floor_anchor.py <holosoma_retargeting pkg dir> <seq list> <heights.json> [write_index...]
  no indices: report h vs z0 for every clip (floor-start clips should have z0 ~ h)
  indices:    write demo_data/FORMHOI/fa<ii>_<clip>.pt (+ height_dict key fa<ii>)
"""
import os
import json, pickle, sys
from pathlib import Path
import numpy as np, torch

PKG, SEQS, HEIGHTS = Path(sys.argv[1]), Path(sys.argv[2]), Path(sys.argv[3])
write = [int(a) for a in sys.argv[4:]]
RAW = Path(os.environ.get("REACTOR", "/home/ant/reactor-mujoco-formhoi")) / Path("dataset/formhoi_soma_raw")
seqs = SEQS.read_text().split()
heights = json.loads(HEIGHTS.read_text())


def obj_verts(path):
    return np.array([[float(x) for x in l.split()[1:4]] for l in open(path) if l.startswith("v ")])


def quat_mat(q):  # wxyz
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def clip_h(d, s):
    v = obj_verts(RAW / s / "objects" / f"{d['obj_name']}.obj")
    return -(v @ quat_mat(d["obj_quat"][0]).T)[:, 2].min()


if not write:
    print(f"{'clip':55s} {'z0':>6s} {'h':>6s} {'z0-h':>6s} {'shift':>6s}")
    for i, s in enumerate(seqs):
        d = np.load(RAW / s / f"{s}.npz")
        sc = 1.32 / heights[s]["height"]
        h, z0 = clip_h(d, s), d["obj_pos"][0, 2]
        print(f"{s[20:]:55s} {z0:6.3f} {h:6.3f} {z0 - h:6.3f} {(1 - sc) * (h - z0):6.3f}")
    sys.exit()

hd_path = PKG / "demo_data/height_dict.pkl"
hd = pickle.load(open(hd_path, "rb"))
for i in write:
    s = seqs[i]
    d = np.load(RAW / s / f"{s}.npz")
    sc = 1.32 / heights[s]["height"]
    h, z0 = clip_h(d, s), d["obj_pos"][0, 2]
    a = torch.load(PKG / "demo_data/FORMHOI" / f"fh{i:02d}_{s[20:]}.pt").numpy().copy()
    a[:, 320] += (1 - sc) * (h - z0)
    torch.save(torch.from_numpy(a), PKG / "demo_data/FORMHOI" / f"fa{i:02d}_{s[20:]}.pt")
    hd[f"fa{i:02d}"] = hd[f"fh{i:02d}"]
    print(f"fa{i:02d}_{s[20:]}: z0 {z0:.3f} h {h:.3f} shift {(1 - sc) * (h - z0):+.3f}")
pickle.dump(hd, open(hd_path, "wb"))
