#!/usr/bin/env python3
"""SYSMIC V2: URDF -> MuJoCo model with motors (the "physical" model).

    python sysmic_to_mjcf.py            # convert and check
    python sysmic_to_mjcf.py --view     # convert, then open the MuJoCo viewer

The other scripts call convert() themselves when the URDF has changed, so running
this by hand is only needed to look at the physical model.
"""
import os
import xml.etree.ElementTree as ET

import mujoco

from sysmic_paths import DESCRIPTION, GENERATED, ROBOT_XML, SCENE_XML, URDF


_MARKER = os.path.join(GENERATED, "built_from.txt")


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
    # 1. Clean URDF: drop Gazebo-only tags, add MuJoCo compiler hints
    tree = ET.parse(URDF)
    root = tree.getroot()
    for gz in root.findall("gazebo"):
        root.remove(gz)
    mj = ET.Element("mujoco")
    ET.SubElement(mj, "compiler", meshdir=DESCRIPTION, strippath="false",
                  discardvisual="false", balanceinertia="true", fusestatic="false")
    root.insert(0, mj)
    urdf_str = ET.tostring(root, encoding="unicode")

    # 2. URDF -> MJCF. The saved file keeps the absolute mesh folder, which is fine
    # because it is generated on this machine and never shipped.
    os.makedirs(GENERATED, exist_ok=True)
    model = mujoco.MjModel.from_xml_string(urdf_str)
    mujoco.mj_saveLastXML(ROBOT_XML, model)

    # 3. Wrap robot in a scene: floating base, floor, light, actuators
    t = ET.parse(ROBOT_XML)
    r = t.getroot()
    wb = r.find("worldbody")

    # Make the robot free to drive: move all worldbody children into a free-joint body
    base = ET.Element("body", name="sysmic_root", pos="0 0 0.02")
    ET.SubElement(base, "freejoint", name="root")
    for child in list(wb):
        wb.remove(child)
        base.append(child)
    wb.append(base)

    ET.SubElement(wb, "light", pos="0 0 4", dir="0 0 -1", diffuse="0.8 0.8 0.8")
    ET.SubElement(wb, "geom", name="floor", type="plane", size="10 10 0.1",
                  rgba="0.8 0.8 0.8 1", friction="1 0.005 0.0001")

    vis = r.find("visual") or ET.SubElement(r, "visual")
    ET.SubElement(vis, "global", offwidth="1600", offheight="1200")

    # Stable integration for stiff position servos on very light links
    opt = r.find("option")
    if opt is None:
        opt = ET.SubElement(r, "option")
    opt.set("integrator", "implicitfast")
    opt.set("timestep", "0.002")

    # Geared servos have reflected rotor inertia; without it the tiny wrist and
    # gripper links (inertia ~1e-4) make the joints jitter.
    for j in r.iter("joint"):
        n = j.get("name", "")
        if n.startswith(("left_", "right_")):
            j.set("armature", "0.01")
            j.set("damping", "0.2")
        if "caster" in n:
            j.set("armature", "0.0005")

    # Suspension springs: Gazebo's springStiffness=34000 / springReference=0 are lost in
    # conversion. Reference 0 gives the 4 mm preload (~136 N on each drive wheel).
    for j in r.iter("joint"):
        if j.get("name", "").endswith("_suspension"):
            j.set("stiffness", "34000")
            j.set("springref", "0")

    # MuJoCo turns every collision mesh into its convex hull. base_link's mesh
    # (box + tower) is concave, so its hull swallows the wheels and arms and
    # pushes them apart at t=0. Ignore contacts between base_link and the rest
    # of the robot (self-collision); the floor still collides with everything.
    contact = ET.SubElement(r, "contact")
    robot_bodies = [b.get("name") for b in base.iter("body")
                    if b.get("name") not in ("sysmic_root", "base_link")]
    for b in robot_bodies:
        ET.SubElement(contact, "exclude", body1="base_link", body2=b)

    # Actuators: position control for lift + arms, velocity for drive wheels
    act = ET.SubElement(r, "actuator")
    ET.SubElement(act, "position", name="lift", joint="lift_joint",
                  kp="100000", kv="5000", ctrlrange="0 0.4477", forcerange="-1000 1000")
    for side in ("left", "right"):
        for jn, lo, hi in [("shoulder_pan", -1.91986, 1.91986),
                           ("shoulder_lift", 0, 3.49066),
                           ("elbow_flex", -3.38594, 0),
                           ("wrist_flex", -1.65806, 1.65806),
                           ("wrist_roll", -2.74017, 2.84489),
                           ("gripper", -0.174533, 1.74533)]:
            ET.SubElement(act, "position", name=f"{side}_{jn}", joint=f"{side}_{jn}",
                          kp="25", kv="0.5", ctrlrange=f"{lo} {hi}", forcerange="-2.9 2.9")
    for side in ("left", "right"):
        ET.SubElement(act, "velocity", name=f"wheel_{side}", joint=f"wheel_{side}_joint",
                      kv="10", ctrlrange="-27 27", forcerange="-16 16")

    t.write(SCENE_XML, xml_declaration=True, encoding="utf-8")

    # 4. Sanity check: load and simulate 2 s
    m = mujoco.MjModel.from_xml_path(SCENE_XML)
    d = mujoco.MjData(m)
    mujoco.mj_resetData(m, d)
    for _ in range(int(2.0 / m.opt.timestep)):
        mujoco.mj_step(m, d)
    if abs(d.qpos[2]) > 0.05:
        raise RuntimeError(f"Physical model does not stand still (base height {d.qpos[2]:.3f} m)")
    with open(_MARKER, "w") as f:
        f.write(DESCRIPTION)
    if verbose:
        print(f"OK: {m.nbody} bodies, {m.njnt} joints, {m.nu} actuators, {m.nmesh} meshes")
        print(f"Base height after 2 s: {d.qpos[2]:.3f} m (should stay near 0)")
        print(f"Scene saved: {SCENE_XML}")
    return SCENE_XML


if __name__ == "__main__":
    import argparse

    ap = argparse.ArgumentParser()
    ap.add_argument("--view", action="store_true", help="open the MuJoCo viewer afterwards")
    args = ap.parse_args()
    scene = convert()
    if args.view:
        import mujoco.viewer

        mujoco.viewer.launch_from_path(scene)
