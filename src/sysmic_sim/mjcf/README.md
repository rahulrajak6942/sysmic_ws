# SYSMIC V2 MuJoCo models

Generated from `../description/sysmic_v2_gazebo.urdf` by
`python sim/sysmic_to_mjcf.py --export`. Do not edit these files by hand:
change the URDF, then export again.

| File | What it is |
|---|---|
| `sysmic_v2_scene.xml` | Physical model: floor, light, 15 motors, real wheels, casters and suspension. Open this one |
| `sysmic_v2.xml` | Plain URDF to MJCF conversion, without motors or floor |
| `robosuite/robot.xml` | Body, lift and arms of the robosuite / RoboCasa model |
| `robosuite/base.xml` | Planar base joints, passive wheels and casters |
| `robosuite/left_gripper.xml`, `robosuite/right_gripper.xml` | Gripper models |
| `robosuite/controller.json` | robosuite controller settings |

View the physical model (needs only `pip install mujoco`):

```bash
python -m mujoco.viewer --mjcf=sysmic_v2_scene.xml
```

Mesh paths point into `../description/models/`, so keep this folder inside the
package. The `robosuite/` files are parts that robosuite assembles into one robot.
Each one loads on its own in MuJoCo, but use `sim/sysmic_robosuite.py` to run them
as a robot.
