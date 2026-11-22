# SYSMIC 2.0 for RoboCasa

A self-contained robosuite / RoboCasa robot: SYSMIC 2.0, a mobile base with a lift
and two 5-joint arms with single-jaw grippers. Copy this folder anywhere. It does
not need ROS, the URDF or the rest of the workspace.

```
sysmic_v2_robocasa/
├── sysmic_robocasa.py      registers the robot "Sysmic"; make_env, joint_action, get_joints
├── sensors.yaml            which sensors are on and their parameters (cameras, lidars, IMUs)
├── sysmic_sensors.py       mounts and reads the sensors
├── control_panel.py        a slider for every joint + keyboard driving, in a kitchen
├── demo.py                 moves every joint in a kitchen (scripted)
└── assets/                 MuJoCo XML / MJCF files + 28 meshes (32 MB), see below
```

| File in `assets/` | What it is |
|---|---|
| `robot.xml`, `base.xml`, `left_gripper.xml`, `right_gripper.xml`, `controller.json` | the robosuite / RoboCasa robot; `sysmic_robocasa.py` loads these |
| `sysmic_v2_scene.xml` | physical MuJoCo model on an empty floor (15 motors, real wheels, casters, suspension); opens in plain MuJoCo: `python -m mujoco.viewer --mjcf=assets/sysmic_v2_scene.xml`. Control tab: `lift` (m), arm and gripper joints (rad), `wheel_left` / `wheel_right` (rad/s, positive = forward on both) |
| `sysmic_v2.xml` | plain URDF to MJCF conversion, without motors or floor |
| `meshes/` | every mesh the files above use (arm meshes reduced to 60k triangles for MuJoCo) |

## Requirements

robosuite 1.5 and RoboCasa with the kitchen assets downloaded (tested with
robosuite 1.5.2, RoboCasa 1.0.1, MuJoCo 3.3.1, Python 3.11).

## Try it

Run these with the Python that has robosuite (e.g. `conda activate robocasa`), not
the system `python3`.

```bash
conda activate robocasa
python control_panel.py               # viewer + "SYSMIC 2.0 control" panel with sliders
python demo.py                        # kitchen viewer: lift, arms, wrists, grippers, base
python demo.py --layout 5 --style 12  # another kitchen
python demo.py --no-viewer            # without a window
```

### Control panel

Two windows open: the MuJoCo viewer and the "SYSMIC 2.0 control" panel.

- **Sliders:** one per joint, labelled with the URDF names: `lift_joint` (m), the
  5 joints of each arm (rad), and each gripper's jaw angle (-0.17 open ... 1.75
  closed).
- **Driving:** click the panel, then hold W/S (or Up/Down) to drive and A/D (or
  Left/Right) to turn. Releasing the key stops the robot; Space or the red STOP
  button stops it at once.
- **Buttons:** Ready pose, Reset scene, Quit. Closing either window also quits.

The sliders in the MuJoCo viewer's own "Control" tab do not move this robot:
robosuite's controllers overwrite them every step. Use the panel instead.

## Sensors

`sensors.yaml` decides which sensors are on the robot and their parameters: switch
any sensor (or one stream of the D435i) with `enabled`, and change position,
angle, resolution, field of view, rate, range and noise. `show: true` draws a
sensor as a small dark box (visual only, no collision) of `size` in the simulation
and in RViz: on by default for the D435i (in the face window) and the three
webcams (the gripper ones sunk flush into the flat palm). Restart the simulation
after editing; nothing has to be rebuilt.

| Sensor | Where | Default |
|---|---|---|
| `head_camera`: RealSense D435i | face window, moves with the lift | colour 640x360 69 deg + depth 640x360 87 deg (0.3 to 10 m, 16-bit mm), 30 Hz; built-in IMU present but off |
| `tray_camera`: webcam | under the "SYSMIC" plate, 65 deg down at the tray | 640x480, 75 deg, 30 Hz. Sees the tray from about lift 0.15 m up |
| `left_gripper_camera`, `right_gripper_camera`: webcams | flat palm face between the fixed finger and the jaw | 640x480, 75 deg, 30 Hz |
| `lidar_top`, `lidar_base`: YDLIDAR | the pucks on the head and on the base deck | 360 deg, 0.5 deg steps, 0.12 to 10 m, 10 Hz |
| `base_imu` | base deck | accel + gyro + orientation, 30 Hz |

```python
env = S.make_env("Kitchen", layout_ids=1, style_ids=1)    # or sensors="my_sensors.yaml"
env.reset()
sensors = S.Sensors(env)              # create after env.reset()
env.step(S.joint_action(env, {}))
for name in sensors.due():            # streams whose rate says "now"
    r = sensors.read(name)
# r["ranges"] (lidar, m, inf = no hit), r["accel"] / r["gyro"] / r["orientation"] (IMU),
# r["image"] (camera: H x W x 3 uint8, or H x W uint16 mm for depth) + r["intrinsics"]
```

How they are simulated: cameras are MuJoCo cameras rendered offscreen (about 10 ms
per image in a kitchen); lidar beams are rays cast against the visual meshes (the
lidar's own housing is ignored, the rest of the robot blocks beams); IMUs are
MuJoCo accelerometer, gyro and orientation sensors with Gaussian noise. Rates are
counted in simulation time, and cameras and IMUs can be at most `sim.control_freq`
(30 Hz by default).

## Use it in your own code

```python
import sys
sys.path.insert(0, "/path/to/sysmic_v2_robocasa")
import sysmic_robocasa as S                     # registers "Sysmic" with robosuite

env = S.make_env("Kitchen", layout_ids=1, style_ids=1)   # or any RoboCasa task, e.g. "PnPCounterToCab"
env.reset()

print(S.get_joints(env))                        # current positions by URDF joint name

targets = {"lift_joint": 0.35,                  # m
           "arm_right_shoulder_joint": 1.4,     # rad
           "arm_right_elbow_joint": -1.0,
           "arm_right_gripper_joint": 1.7}      # jaw angle: -0.17 open ... 1.75 closed
for _ in range(60):                             # 20 steps = 1 s
    env.step(S.joint_action(env, targets, base=(0.0, 0.0)))
```

`joint_action(env, targets, base)` builds the 16-value robosuite action:

- `targets` holds absolute positions by URDF name. Joints left out keep their
  current position, and values are clipped to the limits below.
- `base=(forward, turn)` takes each value from -1 to 1 as a fraction of top speed
  (0.5 m/s, 1.0 rad/s). Send it every step; the base is differential drive.

`S.make_env` is `robosuite.make` with `robots="Sysmic"` and this robot's controller
config, so you can pass any other `robosuite.make` argument as well:
`has_renderer=True, renderer="mjviewer", render_camera=None` gives a viewer with a
free camera. `S.make_action(...)` is the lower-level version that takes arm
vectors and gripper actions from -1 to 1.

## Joints

| Joint | Unit | Range |
|---|---|---|
| `lift_joint` | m | 0 to 0.4477 |
| `arm_{left,right}_pan_joint` | rad | -1.92 to 1.92 |
| `arm_{left,right}_shoulder_joint` | rad | 0 to 3.49 |
| `arm_{left,right}_elbow_joint` | rad | -3.39 to 0 |
| `arm_{left,right}_wrist_flex_joint` | rad | -1.66 to 1.66 |
| `arm_{left,right}_wrist_roll_joint` | rad | -2.74 to 2.84 |
| `arm_{left,right}_gripper_joint` | rad | -0.17 (open) to 1.75 (closed) |

Ready pose (start of every episode): lift 0.2 m, each arm
`[pan, shoulder, elbow, wrist_flex, wrist_roll] = [0, 1.0, -1.2, 0.4, 0]`.

Inside the MuJoCo model the joints are called `robot0_{left,right}_{shoulder_pan,
shoulder_lift,elbow_flex,wrist_flex,wrist_roll}`, `robot0_torso_lift_joint` and
`gripper0_{left,right}_jaw_joint`; the base is `mobilebase0_joint_mobile_{forward,
side,yaw}`.

## Limitations

- **Arm mass.** The masses come from the CAD export: 3.5 kg per arm. Holding the
  arm against gravity already takes 2.2 to 2.7 N m of the servos' 2.9 N m in
  common poses, so poses that reach far out sag. Check the masses against the
  real robot.
- **Start position.** RoboCasa places the robot right at a counter, so back away
  or turn before stretching the arms.
- **Base and wheels.** The base is moved by planar joints like RoboCasa's own
  mobile robots. Wheels and casters are passive and roll on a ground plane that
  belongs to the robot, so wheel slip has no effect.

## Regenerating

The `assets/` folder is generated from `sysmic_v2_sim/urdf/sysmic.urdf`:

```bash
cd sysmic_v2_sim/sim
python sysmic_to_mjcf.py --export
```
