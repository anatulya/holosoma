"""Our training robot (G1 + Inspire DFQ, config/g1_inspire_cfg.get_inspire_spec)
as a Holosoma retargeter model: models/g1_dex/g1_dex_53dof.xml (robot + ground)
and g1_dex_53dof_w_<obj>.xml per exported object (robot + ground + object,
laid out like their g1_29dof_w_<obj>.xml).

Adds what their retargeter expects of a G1 and our model lacks: massless
marker bodies (pelvis_contour_link, *_ankle_intermediate_1_link,
*_ankle_roll_sphere_{1..5}_link, at their XML's offsets: same Unitree G1
kinematics), an unnamed floating base (they read limits from every named
joint), a ground plane named "ground" and the object body last in qpos.
Robot collision geoms keep ours (contype = conaffinity = 1 like theirs).

usage: build_dex_model.py <holosoma_retargeting pkg dir> <object_name> [...]
"""
import os
import sys
from pathlib import Path

sys.path.insert(0, os.environ.get("REACTOR", "/home/ant/reactor-mujoco-formhoi"))
import mujoco
from mjlab.asset_zoo.robots.unitree_g1 import g1_constants
from config.g1_inspire_cfg import get_inspire_spec

PKG = Path(sys.argv[1])
objects = sys.argv[2:]
OUT = PKG / "models/g1_dex"
OUT.mkdir(parents=True, exist_ok=True)
G1_MESH_DIR = Path(g1_constants.G1_XML).parent / "assets"
MARKER_INERTIA = dict(mass=0.001, inertia=[1e-7, 1e-7, 1e-7])


def marker(parent, name, pos, size=0.005):
  b = parent.add_body(name=name, pos=pos, **MARKER_INERTIA)
  b.add_geom(name=name, type=mujoco.mjtGeom.mjGEOM_SPHERE, size=[size, 0, 0], contype=0, conaffinity=0,
             group=1, density=0, rgba=[0.9, 0.2, 0.2, 1])


def robot_spec() -> mujoco.MjSpec:
  spec = get_inspire_spec()
  for m in spec.meshes:  # absolute mesh paths, the XML lands elsewhere
    if not Path(m.file).is_absolute():
      m.file = str(G1_MESH_DIR / m.file)
  spec.meshdir = ""
  spec.joint("floating_base_joint").name = ""
  for k in list(spec.keys):
    spec.delete(k)
  for s in list(spec.sensors):
    spec.delete(s)
  marker(spec.body("pelvis"), "pelvis_contour_link", [0, 0, 0])
  for side, sy in (("left", 1), ("right", -1)):
    marker(spec.body(f"{side}_knee_link"), f"{side}_ankle_intermediate_1_link", [0, 0, -0.28], 0.01)
    foot = spec.body(f"{side}_ankle_roll_link")
    for i, (x, y) in enumerate(((-0.05, 0.025), (-0.05, -0.025), (0.12, 0.03), (0.12, -0.03), (0.14, 0.0)), 1):
      marker(foot, f"{side}_ankle_roll_sphere_{i}_link", [x, y, -0.03])
  w = spec.worldbody
  tex = spec.add_texture(name="texplane", builtin=mujoco.mjtBuiltin.mjBUILTIN_CHECKER, type=mujoco.mjtTexture.mjTEXTURE_2D,
                         width=512, height=512, rgb1=[0.2, 0.3, 0.4], rgb2=[0.1, 0.15, 0.2])
  mat = spec.add_material(name="MatPlane", reflectance=0.2, texuniform=True)
  mat.textures[mujoco.mjtTextureRole.mjTEXROLE_RGB] = "texplane"
  w.add_geom(name="ground", type=mujoco.mjtGeom.mjGEOM_PLANE, size=[10, 10, 0.1], material="MatPlane", condim=1,
             contype=1, conaffinity=15)
  w.add_light(name="sun", pos=[0, 0, 5], dir=[0, 0, -1], diffuse=[1, 1, 1], ambient=[0.2, 0.2, 0.2])
  return spec


def to_xml(spec: mujoco.MjSpec) -> str:
  """spec.to_xml(), minus the empty <default/> the attached hands leave (an
  unnamed class the XML parser rejects; nothing references it)."""
  return "\n".join(l for l in spec.to_xml().splitlines() if l.strip() != "<default/>")


def check(model: mujoco.MjModel, n_obj: int):
  named = [model.joint(i).name for i in range(model.njnt) if model.joint(i).name]
  assert model.nq == 7 + 53 + 7 * n_obj, model.nq
  assert len(named) == 53, len(named)
  assert model.jnt_type[0] == mujoco.mjtJoint.mjJNT_FREE and not model.joint(0).name


spec = robot_spec()
check(spec.compile(), 0)
(OUT / "g1_dex_53dof.xml").write_text(to_xml(spec))
for obj in objects:
  spec = robot_spec()
  mesh = spec.add_mesh(name=f"{obj}_mesh", file=str((PKG / "models" / obj / f"{obj}.obj").resolve()))
  b = spec.worldbody.add_body(name=f"{obj}_link", mass=0.1, inertia=[0.002, 0.002, 0.002])
  b.add_freejoint()
  b.add_geom(name=obj, type=mujoco.mjtGeom.mjGEOM_MESH, meshname=mesh.name, contype=1, conaffinity=1,
             rgba=[0.7, 0.8, 0.9, 0.7], friction=[0.9, 0.5, 0.5], solref=[0.02, 1], solimp=[0.9, 0.95, 0.001, 0.5, 2])
  check(spec.compile(), 1)
  (OUT / f"g1_dex_53dof_w_{obj}.xml").write_text(to_xml(spec))
  print("wrote", OUT / f"g1_dex_53dof_w_{obj}.xml")
# round-trip check: the written XML loads standalone
m = mujoco.MjModel.from_xml_path(str(OUT / (f"g1_dex_53dof_w_{objects[-1]}.xml" if objects else "g1_dex_53dof.xml")))
print("loads:", m.nq, "qpos,", m.nbody, "bodies,", m.neq, "joint couplings")
