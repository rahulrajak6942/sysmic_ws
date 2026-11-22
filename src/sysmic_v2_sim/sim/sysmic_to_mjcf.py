#!/usr/bin/env python3
"""SYSMIC 2.0: sysmic.urdf -> MuJoCo model with motors (the "physical" model).

    python sysmic_to_mjcf.py             # convert and check
    python sysmic_to_mjcf.py --view      # convert, then open the MuJoCo viewer
    python sysmic_to_mjcf.py --export    # also refresh ../sysmic_v2_robocasa/assets

The other scripts call convert() themselves when the URDF has changed. The working
copies live in ~/.cache/sysmic_v2_sim and contain absolute paths for this machine.

The URDF file is never edited. A few simulation-only things are added in memory
before conversion (see sim_urdf()):
  * lighter copies of the meshes that have more than 60000 triangles (MuJoCo
    refuses STL files above 200000; the arm meshes include every screw)
  * colours (the URDF has no materials), from sysmic_colors.py, including the
    "SYSMIC" lettering on the lift as its own darker visual
  * contact cylinders for the 2 drive wheels and 4 caster wheels, sized from their
    meshes (a mesh hull is a polygon and rolls badly)
  * a sprung slide joint above each drive wheel. In the URDF the drive wheels
    (radius 0.085 m, axle at 0.081 m) reach 4 mm below the casters; on the real
    robot that is spring preload. Without the spring the base would rock.
"""
import math
import os
import struct
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from sysmic_colors import link_color, split_decals
from sysmic_paths import DESCRIPTION, GENERATED, ROBOT_XML, SCENE_XML, URDF

_MARKER = os.path.join(GENERATED, "built_from.txt")

LIFT_FORCE = 2000       # N, lift motor limit (placeholder; the load alone is 368 N)
ARM_KP, ARM_KV = 200, 3  # arm / gripper position servos; with kp 25 the arms sagged 5 deg
ARM_TORQUE = 2.9        # N m, STS3215
SUSP_TRAVEL = 0.012     # m, drive wheel spring travel
SUSP_STIFFNESS = 34000  # N/m; reference 0 gives the 4 mm preload (~136 N per wheel)


def read_stl(path):
    """Vertices (n, 3) of an ASCII or binary STL file."""
    with open(path, "rb") as f:
        data = f.read()
    if data[:5] == b"solid" and b"facet" in data[:300]:
        return np.array([[float(x) for x in line.split()[1:4]]
                         for line in data.decode(errors="ignore").splitlines()
                         if line.strip().startswith("vertex")])
    n = struct.unpack("<I", data[80:84])[0]
    rec = np.frombuffer(data[84:84 + 50 * n], dtype=np.dtype(
        [("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")]))
    return rec["v"].reshape(-1, 3).astype(float)


MAX_FACES = 60000  # per mesh for simulation (MuJoCo's hard limit is 200000)


def write_stl(path, verts, faces):
    tri = verts[faces].astype("<f4")
    n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    n /= np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-12)
    rec = np.zeros(len(faces), dtype=np.dtype([("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")]))
    rec["n"], rec["v"] = n, tri.reshape(-1, 9)
    with open(path, "wb") as f:
        f.write(b"sysmic sim mesh".ljust(80, b" "))
        f.write(struct.pack("<I", len(faces)))
        f.write(rec.tobytes())


def sim_mesh(src):
    """Path of a mesh MuJoCo can load quickly: src itself if small, else a lighter copy.

    Big meshes (the arm parts include every screw) are reduced by vertex clustering
    on a grid that is made coarser until the mesh has at most MAX_FACES triangles.
    Copies go to the cache folder; the URDF's meshes are not touched.
    """
    v = read_stl(src)
    if len(v) // 3 <= MAX_FACES:
        return src
    out_dir = os.path.join(GENERATED, "meshes")
    os.makedirs(out_dir, exist_ok=True)
    out = os.path.join(out_dir, os.path.basename(src))
    if os.path.exists(out) and os.path.getmtime(out) >= os.path.getmtime(src):
        return out
    cell = 0.0005
    while True:
        keys = np.floor(v / cell).astype(np.int64)
        _, inv = np.unique(keys, axis=0, return_inverse=True)
        inv = inv.reshape(-1)
        faces = inv.reshape(-1, 3)
        ok = (faces[:, 0] != faces[:, 1]) & (faces[:, 1] != faces[:, 2]) & (faces[:, 0] != faces[:, 2])
        faces = np.unique(np.sort(faces[ok], axis=1), axis=0, return_index=True)[1]
        faces = inv.reshape(-1, 3)[ok][np.sort(faces)]
        if len(faces) <= MAX_FACES:
            break
        cell *= 1.4
    verts = np.zeros((inv.max() + 1, 3))
    np.add.at(verts, inv, v)
    verts /= np.bincount(inv)[:, None]
    write_stl(out, verts, faces)
    return out


def link_mesh(link):
    """Mesh file (absolute) of a URDF link's visual, or None."""
    m = link.find("visual/geometry/mesh")
    return os.path.join(DESCRIPTION, m.get("filename")) if m is not None else None


def is_wheel(name):
    return name.startswith("wheel_") or (name.startswith("caster_") and "_wheel_" in name)


def sim_urdf():
    """The URDF as an XML tree with the simulation-only additions described above."""
    root = ET.parse(URDF).getroot()
    for gz in root.findall("gazebo"):
        root.remove(gz)
    links = {l.get("name"): l for l in root.findall("link")}

    # meshes: absolute paths, reduced copies where the original is too big
    for mesh in root.iter("mesh"):
        mesh.set("filename", sim_mesh(os.path.join(DESCRIPTION, mesh.get("filename"))))

    # colours. A link with a decal (the "SYSMIC" lettering) gets the rest of its mesh
    # as its visual here; the decal itself is a thin open surface, which the URDF
    # importer cannot take, so convert() adds it to the MJCF afterwards.
    decals = []  # (link, mesh file, rgba)
    for name, link in links.items():
        for vis in link.findall("visual"):
            mesh = vis.find("geometry/mesh")
            parts = split_decals(name, mesh.get("filename")) if mesh is not None \
                else [(None, link_color(name))]
            if parts[0][0] is not None:
                mesh.set("filename", parts[0][0])
            mat = ET.SubElement(vis, "material", name=f"{name}_mat")
            ET.SubElement(mat, "color", rgba=parts[0][1])
            decals += [(name, path, rgba) for path, rgba in parts[1:]]

    # wheel contact cylinders (wheels spin about their link's y axis)
    for name, link in links.items():
        if not is_wheel(name):
            continue
        v = read_stl(link_mesh(link))
        lo, hi = v.min(0), v.max(0)
        radius = float(max(hi[0] - lo[0], hi[2] - lo[2]) / 2)
        for col in link.findall("collision"):
            link.remove(col)
        col = ET.SubElement(link, "collision", name=f"{name}_contact")
        c = (lo + hi) / 2
        ET.SubElement(col, "origin", xyz=f"{c[0]:.6f} {c[1]:.6f} {c[2]:.6f}", rpy="1.5707963 0 0")
        geo = ET.SubElement(col, "geometry")
        ET.SubElement(geo, "cylinder", radius=f"{radius:.6f}", length=f"{hi[1] - lo[1]:.6f}")

    # drive wheel suspension: base_link -> wheel_X_susp_link (slide z) -> wheel_X_link
    for side in ("left", "right"):
        joint = root.find(f"joint[@name='wheel_{side}_joint']")
        susp = f"wheel_{side}_susp_link"
        link = ET.SubElement(root, "link", name=susp)
        inert = ET.SubElement(link, "inertial")
        ET.SubElement(inert, "origin", xyz="0 0 0", rpy="0 0 0")
        ET.SubElement(inert, "mass", value="0.3")
        ET.SubElement(inert, "inertia", ixx="1e-4", iyy="1e-4", izz="1e-4",
                      ixy="0", ixz="0", iyz="0")
        sj = ET.SubElement(root, "joint", name=f"wheel_{side}_suspension", type="prismatic")
        ET.SubElement(sj, "parent", link=joint.find("parent").get("link"))
        ET.SubElement(sj, "child", link=susp)
        o = joint.find("origin")
        ET.SubElement(sj, "origin", xyz=o.get("xyz"), rpy=o.get("rpy", "0 0 0"))
        ET.SubElement(sj, "axis", xyz="0 0 1")
        ET.SubElement(sj, "limit", lower="0", upper=str(SUSP_TRAVEL), effort="2000", velocity="1")
        joint.find("parent").set("link", susp)
        o.set("xyz", "0 0 0")
    return root, decals


def add_decals(path, decals):
    """Add decal visuals (thin surfaces, shell inertia, no mass) to a saved MJCF."""
    if not decals:
        return
    t = ET.parse(path)
    r = t.getroot()
    asset = r.find("asset")
    bodies = {b.get("name"): b for b in r.iter("body")}
    for link, mesh_file, rgba in decals:
        name = os.path.splitext(os.path.basename(mesh_file))[0]
        ET.SubElement(asset, "mesh", name=name, file=mesh_file, inertia="shell")
        body = bodies[link]
        first_geom = next(i for i, el in enumerate(body) if el.tag == "geom")
        body.insert(first_geom + 1, ET.Element(
            "geom", name=name, type="mesh", mesh=name, rgba=rgba, contype="0",
            conaffinity="0", group="1", density="0"))
    t.write(path, xml_declaration=True, encoding="utf-8")


def check_motors(m):
    """Drive every motor of the physical model and check the joint follows.
    Returns a list of problems (empty = all good)."""
    def fresh():
        d = mujoco.MjData(m)
        for side in ("left", "right"):  # arms in the ready pose, like on the robot
            for k, v in (("shoulder", 1.0), ("elbow", -1.2), ("wrist_flex", 0.4)):
                n = f"arm_{side}_{k}_joint"
                d.ctrl[m.actuator(n).id] = v
                d.qpos[m.joint(n).qposadr[0]] = v
        mujoco.mj_forward(m, d)
        return d

    def run(d, seconds):
        for _ in range(int(seconds / m.opt.timestep)):
            mujoco.mj_step(m, d)

    problems = []
    for i in range(m.nu):
        a = m.actuator(i)
        if m.actuator_biastype[i] != mujoco.mjtBias.mjBIAS_AFFINE or a.name.startswith("wheel"):
            continue  # position servos only
        j = m.joint(m.actuator_trnid[i, 0])
        lo, hi = m.actuator_ctrlrange[i]
        target = lo + 0.6 * (hi - lo)
        d = fresh()
        run(d, 0.5)
        d.ctrl[i] = target
        run(d, 3.0)
        err = abs(d.qpos[j.qposadr[0]] - target)
        if err > max(0.05, 0.05 * (hi - lo)):
            problems.append(f"{a.name}: commanded {target:.3f}, joint at "
                            f"{d.qpos[j.qposadr[0]]:.3f}")
    d = fresh()
    run(d, 0.5)
    base = m.body("base_link").id
    start = d.xpos[base].copy()
    heading = lambda: math.atan2(d.xmat[base][3], d.xmat[base][0])  # noqa: E731
    h0 = heading()
    for side in ("left", "right"):
        d.ctrl[m.actuator(f"wheel_{side}").id] = 5.0
    run(d, 3.0)
    moved = d.xpos[base][:2] - start[:2]
    forward = moved @ np.array([math.cos(h0), math.sin(h0)])
    turned = abs((heading() - h0 + math.pi) % (2 * math.pi) - math.pi)
    if forward < 0.8 or turned > 0.15:  # 5 rad/s * 0.085 m * 3 s = 1.3 m
        problems.append(f"wheels +5/+5 rad/s for 3 s: moved {forward:.2f} m forward, "
                        f"turned {math.degrees(turned):.0f} deg")
    return problems


def is_stale():
    """True if the generated model is missing or older than the URDF / this script."""
    if not (os.path.exists(ROBOT_XML) and os.path.exists(SCENE_XML)):
        return True
    try:  # built from a different copy of the workspace? then the mesh paths are wrong
        with open(_MARKER) as f:
            if f.read().strip() != DESCRIPTION:
                return True
    except OSError:
        return True
    built = min(os.path.getmtime(ROBOT_XML), os.path.getmtime(SCENE_XML))
    return built < max(os.path.getmtime(URDF), os.path.getmtime(__file__))


def convert(verbose=True):
    """Build sysmic_v2.xml (plain conversion) and sysmic_v2_scene.xml (with motors)."""
    root, decals = sim_urdf()
    mj = ET.Element("mujoco")
    ET.SubElement(mj, "compiler", meshdir=DESCRIPTION, strippath="false",
                  discardvisual="false", balanceinertia="true", fusestatic="false")
    root.insert(0, mj)
    urdf_joints = {j.get("name"): j for j in root.findall("joint")}

    os.makedirs(GENERATED, exist_ok=True)
    model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))
    mujoco.mj_saveLastXML(ROBOT_XML, model)
    add_decals(ROBOT_XML, decals)

    # Wrap the robot in a scene: free base, floor, light, actuators
    t = ET.parse(ROBOT_XML)
    r = t.getroot()
    wb = r.find("worldbody")
    base = ET.Element("body", name="sysmic_root", pos="0 0 0.02")
    ET.SubElement(base, "freejoint", name="root")
    for child in list(wb):
        wb.remove(child)
        base.append(child)
    wb.append(base)
    ET.SubElement(wb, "light", pos="0 0 4", dir="0 0 -1", diffuse="0.8 0.8 0.8")
    ET.SubElement(wb, "geom", name="floor", type="plane", size="10 10 0.1",
                  rgba="0.8 0.8 0.8 1", friction="1 0.005 0.0001")
    vis = r.find("visual")
    if vis is None:
        vis = ET.SubElement(r, "visual")
    ET.SubElement(vis, "global", offwidth="1600", offheight="1200")

    opt = r.find("option")
    if opt is None:
        opt = ET.SubElement(r, "option")
    opt.set("integrator", "implicitfast")
    opt.set("timestep", "0.002")

    for j in r.iter("joint"):
        n = j.get("name", "")
        if n.startswith("arm_"):  # geared servos: reflected rotor inertia + damping
            j.set("armature", "0.01")
            j.set("damping", "0.2")
        elif "caster" in n:
            j.set("armature", "0.0005")
        elif n.endswith("_suspension"):
            j.set("stiffness", str(SUSP_STIFFNESS))
            j.set("springref", "0")
            j.set("damping", "400")
        elif n == "lift_joint":
            # The URDF's effort="200" is a placeholder. MuJoCo turns it into a joint
            # limit (actuatorfrcrange) that clamps the lift motor below the weight it
            # carries (carriage + both arms = 37.5 kg = 368 N), so the lift never moved.
            # Let the motor's own limit (LIFT_FORCE) decide instead.
            j.set("actuatorfrcrange", f"-{LIFT_FORCE:g} {LIFT_FORCE:g}")

    # base_link's mesh is concave (box + tower); its convex hull swallows the
    # wheels and arms. Ignore base_link <-> rest-of-robot contacts.
    contact = ET.SubElement(r, "contact")
    for b in base.iter("body"):
        if b.get("name") not in ("sysmic_root", "base_link"):
            ET.SubElement(contact, "exclude", body1="base_link", body2=b.get("name"))

    # Actuators: position for lift + arm joints, velocity for the drive wheels
    act = ET.SubElement(r, "actuator")
    lift = urdf_joints["lift_joint"].find("limit")
    ET.SubElement(act, "position", name="lift", joint="lift_joint", kp="200000", kv="10000",
                  ctrlrange=f"{lift.get('lower')} {lift.get('upper')}",
                  forcerange=f"-{LIFT_FORCE:g} {LIFT_FORCE:g}")
    for name, j in urdf_joints.items():
        if name.startswith("arm_") and j.get("type") == "revolute":
            lim = j.find("limit")
            ET.SubElement(act, "position", name=name, joint=name, kp=str(ARM_KP),
                          kv=str(ARM_KV), ctrlrange=f"{lim.get('lower')} {lim.get('upper')}",
                          forcerange=f"-{ARM_TORQUE:g} {ARM_TORQUE:g}")
    for side in ("left", "right"):
        # The URDF mirrors the right wheel (axis -y). gear -1 makes a positive command
        # mean "forward" on both wheels. Torque limit = the URDF effort.
        j = urdf_joints[f"wheel_{side}_joint"]
        axis_y = float(j.find("axis").get("xyz").split()[1])
        effort = float(j.find("limit").get("effort"))
        ET.SubElement(act, "velocity", name=f"wheel_{side}", joint=f"wheel_{side}_joint",
                      kv="20", ctrlrange="-27 27", forcerange=f"-{effort:g} {effort:g}",
                      gear="-1" if axis_y < 0 else "1")
    t.write(SCENE_XML, xml_declaration=True, encoding="utf-8")

    # Sanity check: load and simulate 2 s
    m = mujoco.MjModel.from_xml_path(SCENE_XML)
    d = mujoco.MjData(m)
    mujoco.mj_resetData(m, d)
    for _ in range(int(2.0 / m.opt.timestep)):
        mujoco.mj_step(m, d)
    if abs(d.qpos[2]) > 0.05:
        raise RuntimeError(f"Physical model does not stand still (base height {d.qpos[2]:.3f} m)")
    problems = check_motors(m)
    if problems:
        raise RuntimeError("Physical model motors do not work:\n  " + "\n  ".join(problems))
    with open(_MARKER, "w") as f:
        f.write(DESCRIPTION)
    if verbose:
        print(f"OK: {m.nbody} bodies, {m.njnt} joints, {m.nu} actuators, {m.nmesh} meshes")
        print(f"Base height after 2 s: {d.qpos[2]:.3f} m (should stay near 0)")
        print(f"Scene saved: {SCENE_XML}")
    return SCENE_XML


def export(verbose=True):
    """Copy the models into the standalone folder ../sysmic_v2_robocasa/assets, with
    every mesh they need and relative mesh paths, so that folder can be copied
    anywhere and used without this workspace:
      robot.xml, base.xml, *_gripper.xml, controller.json   robosuite / RoboCasa robot
      sysmic_v2_scene.xml   physical MuJoCo model: floor, motors, real wheels
      sysmic_v2.xml         plain URDF -> MJCF conversion
    All of them share assets/meshes/."""
    import shutil

    import sysmic_robosuite  # builds the robosuite files if they are missing or stale
    from sysmic_paths import ROBOCASA_DIR, ROBOSUITE_DIR

    if is_stale():
        convert(verbose=False)

    out = os.path.join(ROBOCASA_DIR, "assets")
    mesh_out = os.path.join(out, "meshes")
    # Empty the folders but keep them: a shell or viewer sitting in assets/ keeps working.
    for d in (mesh_out, out):
        if os.path.isdir(d):
            for name in os.listdir(d):
                p = os.path.join(d, name)
                if os.path.isfile(p) or os.path.islink(p):
                    os.remove(p)
                elif p != mesh_out:
                    shutil.rmtree(p)
    os.makedirs(mesh_out, exist_ok=True)
    copied = {}

    def copy_model(src, dst_name):
        tree = ET.parse(src)
        root = tree.getroot()
        compiler = root.find("compiler")
        meshdir = os.path.dirname(src)
        if compiler is not None and compiler.get("meshdir"):
            meshdir = os.path.join(os.path.dirname(src), compiler.get("meshdir"))
            del compiler.attrib["meshdir"]
        for mesh in root.iter("mesh"):
            f = mesh.get("file")
            if not f:
                continue  # inline mesh (vertex=...), nothing to copy
            path = os.path.normpath(f if os.path.isabs(f) else os.path.join(meshdir, f))
            base = os.path.basename(path)
            if copied.setdefault(base, path) != path:
                raise RuntimeError(f"two different meshes are called {base}")
            if not os.path.exists(os.path.join(mesh_out, base)):
                shutil.copyfile(path, os.path.join(mesh_out, base))
            mesh.set("file", f"meshes/{base}")
        tree.write(os.path.join(out, dst_name), xml_declaration=True, encoding="utf-8")

    for name in sorted(os.listdir(ROBOSUITE_DIR)):
        src = os.path.join(ROBOSUITE_DIR, name)
        if name.endswith(".xml"):
            copy_model(src, name)
        else:
            shutil.copyfile(src, os.path.join(out, name))
    copy_model(SCENE_XML, "sysmic_v2_scene.xml")
    copy_model(ROBOT_XML, "sysmic_v2.xml")
    for name in os.listdir(out):  # every exported model must load from its new place
        if name.endswith(".xml"):
            mujoco.MjModel.from_xml_path(os.path.join(out, name))
    if verbose:
        size = sum(os.path.getsize(os.path.join(mesh_out, f)) for f in os.listdir(mesh_out))
        print(f"Exported to {out}: {len(os.listdir(out)) - 1} model files, "
              f"{len(copied)} meshes ({size / 1e6:.1f} MB)")
    return out


if __name__ == "__main__":
    import argparse

    ap = argparse.ArgumentParser()
    ap.add_argument("--view", action="store_true", help="open the MuJoCo viewer afterwards")
    ap.add_argument("--export", action="store_true",
                    help="write the RoboCasa model into ../sysmic_v2_robocasa/assets")
    args = ap.parse_args()
    scene = convert()
    if args.export:
        export()
    if args.view:
        import mujoco.viewer

        mujoco.viewer.launch_from_path(scene)
