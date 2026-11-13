"""Play an OmniRetarget kinematic result (qpos (T, 43) = G1 free root + 29 joints +
object free joint) on its own robot+object scene, with the scaled human joints
drawn as red dots. Offscreen, tracking camera on the pelvis.

usage (from holosoma_retargeting/): render_kin.py <task> <object_name> <out.mp4>
"""
import sys
import imageio, mujoco, numpy as np

task, obj, out = sys.argv[1:4]
robot = sys.argv[4] if len(sys.argv) > 4 else "g1"  # g1 (29 dof) or g1_dex (G1 + Inspire, 53 dof)
zoom = float(sys.argv[5]) if len(sys.argv) > 5 else 2.2
d = np.load(f"demo_results/{robot}/object_interaction/formhoi/{task}_original.npz")
qpos, hj, fps = d["qpos"], d["human_joints"], float(d["fps"])
dof = {"g1": 29, "g1_dex": 53}[robot]
m = mujoco.MjModel.from_xml_path(f"models/{robot}/{robot}_{dof}dof_w_{obj}.xml")
m.vis.global_.offwidth, m.vis.global_.offheight = 960, 720
data = mujoco.MjData(m)
r = mujoco.Renderer(m, 720, 960)
cam = mujoco.MjvCamera()
cam.distance, cam.elevation, cam.azimuth = zoom, -15.0, 135.0
stride = max(1, round(fps / 30))
w = imageio.get_writer(out, fps=fps / stride, codec="libx264", quality=7)
for t in range(0, len(qpos), stride):
    data.qpos[:] = qpos[t]
    mujoco.mj_forward(m, data)
    cam.lookat[:] = qpos[t, -7:-4] if zoom < 1.5 else [qpos[t, 0], qpos[t, 1], 0.8]  # close-up: follow the object
    r.update_scene(data, cam)
    s = r.scene
    for p in hj[t]:
        if s.ngeom >= s.maxgeom:
            break
        mujoco.mjv_initGeom(s.geoms[s.ngeom], mujoco.mjtGeom.mjGEOM_SPHERE, [0.02 if hj.shape[1] <= 52 else 0.008, 0, 0], p, np.eye(3).ravel(), [1, 0.2, 0.2, 0.8])
        s.ngeom += 1
    w.append_data(r.render())
w.close()
print(f"{task}: {len(qpos)} frames @ {fps:g} fps, cost {float(d['cost']):.3f} -> {out}")
