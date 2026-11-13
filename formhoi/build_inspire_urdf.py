"""Build holosoma's G1 + Inspire DFQ URDF for whole-body tracking with objects.

Starts from holosoma's own with-object G1 (data/robots/g1/main_mesh_collision_halfspherehand.urdf,
the robot g1_29dof_w_object trains with), removes its two sphere hands, and attaches Unitree's
Inspire DFQ hands (formhoi/assets/unitree_g1_inspire_dfq, from unitreerobotics/unitree_ros
41dec58, robots/g1_description/g1_29dof_rev_1_0_with_inspire_hand_DFQ.urdf, BSD-3). The arm
kinematics and wrist inertials of both URDFs are identical, and the hand mount is the same
0.0415 m wrist offset our MuJoCo G1 + Inspire uses.

The URDF's <mimic> tags are dropped: every finger joint stays an independent revolute joint, and
the 6 finger couplings per hand are enforced by the action term instead
(holosoma.managers.action.terms.coupled_joint_control, followers get c1 x the parent's target).
The ratios are written next to the URDF as <name>.couplings.json.

    python formhoi/build_inspire_urdf.py
"""
import copy
import json
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
SRC = REPO / "formhoi/assets/unitree_g1_inspire_dfq"
ROBOTS = REPO / "src/holosoma/holosoma/data/robots/g1"
BASE = ROBOTS / "main_mesh_collision_halfspherehand.urdf"
OUT = ROBOTS / "g1_29dof_inspire_dfq.urdf"
MESH_DIR = "meshes_inspire_dfq"


def main() -> None:
    robot = ET.parse(BASE).getroot()
    dfq = ET.parse(SRC / "g1_29dof_rev_1_0_with_inspire_hand_DFQ.urdf").getroot()

    # Drop holosoma's sphere hands (fixed links on the wrist yaw links).
    sphere = {"left_sphere_hand_link", "right_sphere_hand_link"}
    for el in list(robot):
        if el.tag == "link" and el.get("name") in sphere:
            robot.remove(el)
        elif el.tag == "joint" and el.find("child").get("link") in sphere:
            robot.remove(el)

    hand_links = {l.get("name") for l in dfq.findall("link") if l.get("name")[:2] in ("L_", "R_")}
    couplings = []
    for el in dfq:
        if el.tag == "link" and el.get("name") in hand_links:
            link = copy.deepcopy(el)
            for mesh in link.iter("mesh"):
                mesh.set("filename", f"{MESH_DIR}/{Path(mesh.get('filename')).name}")
            robot.append(link)
        elif el.tag == "joint" and el.find("child").get("link") in hand_links:
            joint = copy.deepcopy(el)
            mimic = joint.find("mimic")
            if mimic is not None:
                couplings.append({
                    "joint": joint.get("name"),
                    "parent": mimic.get("joint"),
                    "multiplier": float(mimic.get("multiplier", 1.0)),
                    "offset": float(mimic.get("offset", 0.0)),
                })
                joint.remove(mimic)
            robot.append(joint)

    ET.indent(robot, space="    ")
    OUT.write_text('<?xml version="1.0"?>\n' + ET.tostring(robot, encoding="unicode") + "\n")
    # Hand meshes: data/robots/g1/meshes_inspire_dfq (committed). To refresh them, put unitree_ros'
    # robots/g1_description/meshes at formhoi/assets/unitree_g1_inspire_dfq/meshes.
    if (SRC / "meshes").is_dir():
        (OUT.parent / MESH_DIR).mkdir(exist_ok=True)
        for link in hand_links:
            for mesh in dfq.find(f"link[@name='{link}']").iter("mesh"):
                name = Path(mesh.get("filename")).name
                shutil.copy(SRC / "meshes" / name, OUT.parent / MESH_DIR / name)
    OUT.with_suffix(".couplings.json").write_text(json.dumps(couplings, indent=1) + "\n")
    actuated = [j.get("name") for j in robot.findall("joint") if j.get("type") != "fixed"]
    print(f"{OUT}: {len(actuated)} joints, {len(couplings)} couplings, {len(hand_links)} hand links")


if __name__ == "__main__":
    main()
