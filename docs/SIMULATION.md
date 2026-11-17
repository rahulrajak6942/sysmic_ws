# SYSMIC V2 Simulation

ROS 2 workspace for simulating SYSMIC V2, our dual-arm mobile manipulator for
hospitality tasks: a differential-drive base with four casters and sprung drive
wheels, a 448 mm vertical lift, and two 5-DOF arms with single-jaw grippers
(STS3215 servos).

One launch file starts the robot in a RoboCasa kitchen, connects it to ROS 2
Humble and opens RViz:

```bash
ros2 launch sysmic_sim sim.launch.py
```

The same URDF is used in three simulators:

| Simulator | Used for |
|---|---|
| **RoboCasa** (MuJoCo) + ROS 2 | Manipulation in kitchens, driven from ROS: `/cmd_vel`, `/joint_states`, `/odom`, TF |
| **MuJoCo** physical model | Checking the robot itself: real wheels, casters, suspension, joint limits |
| **Gazebo Harmonic** | Original setup, see [docs/gazebo.md](src/sysmic_sim/docs/gazebo.md) |

## Workspace layout

```
sysmic_ws/
├── README.md
├── install_sim.sh                  one-command setup
└── src/
    └── sysmic_sim/                 ROS 2 package
        ├── package.xml, CMakeLists.txt
        ├── launch/
        │   └── sim.launch.py       simulation + bridge + robot_state_publisher + RViz
        ├── scripts/
        │   └── ros2_bridge.py      ROS 2 node (system Python 3.10)
        ├── sim/                    simulation side (conda Python 3.11)
        │   ├── sysmic_sim_server.py    RoboCasa kitchen that the bridge talks to
        │   ├── sysmic_robosuite.py     SYSMIC as a robosuite / RoboCasa robot, self-test
        │   ├── sysmic_teleop.py        control panel (sliders + keyboard)
        │   ├── sysmic_to_mjcf.py       URDF -> MuJoCo physical model
        │   └── sysmic_paths.py
        ├── description/
        │   ├── sysmic_v2_gazebo.urdf   robot description (source of truth)
        │   ├── models/sysmic_v2/       meshes, Gazebo model
        │   └── sysmic_world.sdf        Gazebo world
        ├── rviz/sysmic.rviz
        └── docs/gazebo.md
```

The MuJoCo and robosuite model files are generated from the URDF into
`~/.cache/sysmic_sim/` the first time anything runs, and again whenever the URDF
or the scripts change. Nothing generated is kept in the workspace.

## Requirements

- Ubuntu 22.04 with ROS 2 Humble (`ros-humble-desktop`)
- About 20 GB of free disk (the RoboCasa kitchen assets are about 10 GB)
- No GPU is needed to run the simulation. A GPU is needed for policy training.

Developed on a Ryzen 5 4600H laptop with 14 GB RAM and a GTX 1650.

## Install

### Quick

```bash
unzip sysmic_ws.zip -d ~
cd ~/sysmic_ws
bash install_sim.sh
```

The script installs Miniconda (if missing), creates a `robocasa` conda env with
Python 3.11, installs robosuite and RoboCasa into `~/robocasa_ws`, downloads the
kitchen assets, runs the SYSMIC self-test and builds this workspace with colcon.
It skips anything already installed, so it is safe to run again if a download
breaks.

Note: the script has not been run on a completely clean machine yet. If it
stops somewhere, continue with the manual steps below from that point.

### Manual

Run one command at a time and stop at the first error.

```bash
# 1. Miniconda (skip if `conda --version` works)
wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh
bash Miniconda3-latest-Linux-x86_64.sh -b
~/miniconda3/bin/conda init && source ~/.bashrc
conda tos accept --override-channels --channel https://repo.anaconda.com/pkgs/main
conda tos accept --override-channels --channel https://repo.anaconda.com/pkgs/r
conda config --set auto_activate_base false

# 2. Environment. Python 3.11 is required; newer versions fail on numba.
conda create -c conda-forge -n robocasa python=3.11 -y
conda activate robocasa
python --version            # must print 3.11.x

# 3. robosuite and RoboCasa
mkdir -p ~/robocasa_ws && cd ~/robocasa_ws
git clone https://github.com/ARISE-Initiative/robosuite
cd robosuite && pip install -e . && cd ..
git clone https://github.com/robocasa/robocasa
cd robocasa && pip install -e .

# 4. Macros and kitchen assets (about 10 GB; re-run if the download stops)
python -m robocasa.scripts.setup_macros
python -m robocasa.scripts.download_kitchen_assets

# 5. Check RoboCasa on its own: a kitchen with the default robot should open
python -m robocasa.demos.demo_kitchen_scenes
conda deactivate

# 6. Build this workspace (new terminal, no conda env active)
source /opt/ros/humble/setup.bash
cd ~/sysmic_ws
colcon build --symlink-install
```

Optional, for driving from the keyboard:

```bash
sudo apt install ros-humble-teleop-twist-keyboard
```

### Versions

Built against robosuite 1.5.2 (commit `5ce6643`), RoboCasa 1.0.1 (commit
`3443cdf`), MuJoCo 3.3.1 and numpy 2.2.5, as cloned on 10 Nov 2026. If a newer
checkout breaks something, `git checkout` those commits in `~/robocasa_ws`.

## Run

```bash
source /opt/ros/humble/setup.bash
source ~/sysmic_ws/install/setup.bash
ros2 launch sysmic_sim sim.launch.py
```

Do not activate conda in this terminal. The launch file starts the simulation
with the Python of the `robocasa` conda env by itself, and everything else with
the normal ROS Python.

It starts:

1. the RoboCasa kitchen with SYSMIC (MuJoCo viewer window)
2. the ROS 2 bridge
3. `robot_state_publisher` with the SYSMIC URDF
4. RViz, showing the robot model and odometry

Closing the MuJoCo viewer, or Ctrl+C in the terminal, stops everything.

| Launch argument | Default | Meaning |
|---|---|---|
| `layout` | `1` | RoboCasa kitchen layout id |
| `style` | `1` | RoboCasa kitchen style id |
| `rviz` | `true` | Start RViz |
| `viewer` | `true` | Show the MuJoCo viewer; `false` runs the simulation headless |
| `conda_env` | `robocasa` | Name of the conda env that has RoboCasa |
| `conda_python` | auto | Full path to that env's `python`, if it is not found automatically |

```bash
ros2 launch sysmic_sim sim.launch.py layout:=5 style:=12
ros2 launch sysmic_sim sim.launch.py rviz:=false
```

### Drive it

In a second terminal (ROS sourced, no conda):

```bash
ros2 topic list
ros2 topic echo /odom --once

# base, from the keyboard
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# lift, arms and grippers
ros2 topic pub --once /sysmic/joint_command sensor_msgs/msg/JointState \
  "{name: [lift_joint, left_shoulder_lift, right_gripper], position: [0.35, 1.4, 1.2]}"
```

## ROS 2 interface

| Topic | Type | Direction | Notes |
|---|---|---|---|
| `/clock` | `rosgraph_msgs/Clock` | out | Simulation time. Use `use_sim_time:=true` in your nodes |
| `/joint_states` | `sensor_msgs/JointState` | out | All 25 movable URDF joints, 20 Hz |
| `/odom` | `nav_msgs/Odometry` | out | Pose since start in `odom`, twist in `base_link` |
| `/tf` | | out | `odom -> base_link`; link frames come from `robot_state_publisher` |
| `/cmd_vel` | `geometry_msgs/Twist` | in | `linear.x` up to 0.5 m/s, `angular.z` up to 1.0 rad/s |
| `/sysmic/joint_command` | `sensor_msgs/JointState` | in | `name[]` and `position[]`; a joint holds its last target |

Joints accepted on `/sysmic/joint_command`:

| Joint | Unit | Range |
|---|---|---|
| `lift_joint` | m | 0 to 0.4477 |
| `{left,right}_shoulder_pan` | rad | -1.92 to 1.92 |
| `{left,right}_shoulder_lift` | rad | 0 to 3.49 |
| `{left,right}_elbow_flex` | rad | -3.39 to 0 |
| `{left,right}_wrist_flex` | rad | -1.66 to 1.66 |
| `{left,right}_wrist_roll` | rad | -2.74 to 2.84 |
| `{left,right}_gripper` | rad | -0.17 (open) to 1.75 (closed) |

Behaviour:

- The base stops if no `/cmd_vel` arrives for 0.5 s. Keep publishing while driving.
- `/cmd_vel` is differential drive; `linear.y` is ignored.
- Odometry is ground truth from the simulator: no wheel slip, no noise.
- The simulation prints its speed relative to real time every 5 s.

### Why the simulation is a separate process

ROS 2 Humble is tied to Python 3.10 and RoboCasa needs 3.11, so `rclpy` cannot
be imported inside the simulation. `sim/sysmic_sim_server.py` (conda Python) and
`scripts/ros2_bridge.py` (ROS Python) exchange small JSON packets over UDP on
localhost: state on port 9870, commands on port 9871. The packet format is
documented at the top of `sysmic_sim_server.py`, which makes it easy to add more
data such as a laser scan or camera images.

## Other tools

These run directly in the conda env, without ROS:

```bash
conda activate robocasa
cd ~/sysmic_ws/src/sysmic_sim/sim

python sysmic_robosuite.py --no-viewer     # self-test
python sysmic_robosuite.py                 # self-test, then a kitchen viewer
python sysmic_teleop.py                    # control panel in a kitchen
python sysmic_to_mjcf.py --view            # physical model in the MuJoCo viewer
```

**Self-test.** Expected output:

```
  jitter at rest (rad)            : about 0.001
  hold error (rad)                : below 0.001
  arm tracking error (rad)        : below 0.001
  lift error (m)                  : 0.0000
  closed jaw angle (rad)          : 1.7453
  reversed in 2 s (m)             : about 0.94
  heading error after turn (rad)  : below 0.01
SYSMIC registered. Action size: 16
```

**Control panel.** Click the panel, then hold `W` / `S` (or Up / Down) to drive
and `A` / `D` (or Left / Right) to turn. Releasing the key stops the robot;
`Space` or the red STOP button stops it immediately. Sliders set the lift and
every arm joint, tick boxes close the grippers. Add `--layout 5 --style 12` for
another kitchen.

**Physical model.** The robot stands on its own wheels on an empty floor, with
all 25 movable URDF joints including caster swivels and wheel suspension. Use
the Control sliders in the viewer for the lift, arms, grippers and drive wheels.

**Viewer camera.** Left-drag rotates, right-drag pans, scroll zooms.

## Python API

```python
import sys
sys.path.insert(0, "/path/to/sysmic_ws/src/sysmic_sim/sim")
import sysmic_robosuite as S

env = S.make_env("Kitchen", layout_ids=1, style_ids=1)
env.reset()

action = S.make_action(
    env,
    base=(0.5, 0.0),                         # forward, turn: fractions of top speed
    lift=0.3,                                # metres
    right_arm=[0.3, 1.4, -1.0, 0.2, 0.5],    # rad
    left_arm=None,                           # None = hold the current pose
    right_gripper=1.0,                       # -1 open ... +1 closed
)
env.step(action)
```

The action vector has 16 values: right arm 5, left arm 5, lift 1, base 3
(forward, sideways, yaw), right gripper 1, left gripper 1. Arms and lift take
absolute targets. `S.gripper_action(angle)` converts a jaw angle in rad to the
-1..+1 gripper action.

Always build base commands with `S.make_action`. robosuite's base controller
does not keep "forward" pointing along the robot once it has turned (the stock
PandaOmron behaves the same way). `make_action` corrects for this, and the
self-test checks it.

## How the models differ from the URDF

The URDF is never edited. Two models are generated from it.

### Physical model

All 25 movable URDF joints plus a free joint for the base, so the robot drives on
its real wheels.

| Change | Reason |
|---|---|
| `<gazebo>` tags dropped | MuJoCo cannot read them |
| Free joint on the base, floor and light added | A URDF has no world |
| 15 motors added: lift, 12 arm and gripper joints (position), 2 drive wheels (velocity) | A URDF has no actuators |
| Suspension spring 34000 N/m with reference 0 | Was defined in the Gazebo tags; gives the 4 mm preload |
| Armature 0.01 and damping 0.2 on the arm joints | Gearbox inertia; the light wrist links jitter without it |
| `implicitfast` integrator, 2 ms step | Stable with stiff position motors |
| Contacts between `base_link` and the rest of the robot disabled | MuJoCo collides the convex hull of each mesh, and the base hull encloses the wheels and arms |

### RoboCasa model

| Change | Reason |
|---|---|
| Base driven by three planar joints (forward, sideways, yaw) | RoboCasa's placement, tasks and controllers expect this, as on its PandaOmron |
| Wheels, casters and suspension are passive and roll on an invisible ground plane that belongs to the robot | They roll and swivel but do not propel the robot. RoboCasa rejects start poses in which the robot touches the kitchen floor |
| Base collision is a platform box and a tower box | The mesh hull would block the arms and the counters |
| Arm joints use torque motors (2.9 N m limit) under robosuite's joint-position controller | robosuite controllers output torques |
| Each gripper is its own model: moving jaw plus fixed finger, as convex shapes | robosuite needs separate gripper models, and a hull of the whole wrist would fill the gap between the jaws |
| Joints renamed (`torso_lift_joint`, `joint_mobile_*`, `passive_*`), right arm listed first | robosuite groups joints by name. The ROS bridge maps them back to URDF names |

Because the base is moved by planar joints in RoboCasa, wheel slip, caster
behaviour and suspension have no effect on driving there. Use the physical model
or Gazebo when those matter.

## Parameters still to be tuned on hardware

These are estimates and have not been measured on the real robot:

- Arm gearbox inertia (armature 0.01) and joint damping (0.2)
- Motor gains for the arms, the lift and the base
- Base top speed (0.5 m/s, 1.0 rad/s)
- Start pose `ARM_READY = [0, 1.0, -1.2, 0.4, 0]`
- Caster spring (20000 N/m, 4 mm travel). This is a modelling aid and does not exist on the robot

## Simulation results

Measured in a robosuite test scene with MuJoCo 3.3.1.

| Check | Result |
|---|---|
| Arm joints, each driven to both limits (collisions off) | Error below 0.0004 rad |
| Peak arm motor torque while holding poses | 0.5 N m (limit 2.9) |
| Lift | Full travel, 0 to 0.448 m |
| Grippers | Full travel, -0.175 to 1.745 rad |
| Grasp and lift 12 cm: 12 mm / 30 g box and 30 mm / 50 g box, both grippers, 6 tries each | Held 24 of 24, box shifts 9 to 15 mm in the jaws |
| Grasp and lift 12 cm: 40 mm / 200 g box, 6 tries each | Right gripper 4 of 6, left gripper 2 of 6 |
| Base at 0.5 m/s command | 0.49 m/s, stops within 3.2 cm |
| Base at 1.0 rad/s command | 0.98 rad/s, stops within 0.6 deg |
| Driving after a 2 rad turn | Heading error 0.0015 rad |
| Physical model: load on the floor | 162 N per drive wheel, 40 to 77 N per caster, 557 N total |
| Physical model: 4 s forward / reverse at 5 rad/s wheel speed | 1.645 m both ways; casters flip 180 deg in reverse |
| ROS bridge: `/cmd_vel` 0.25 m/s and 0.40 rad/s | Odometry reports 0.248 m/s and 0.391 rad/s |

## Known limitations

- From the ready pose, `shoulder_lift` above about 2.5 rad and `elbow_flex`
  near 0 are blocked by the robot's own torso. The joints reach their full range
  when the arm is clear of the body.
- Grasping is only reliable for small, light objects so far. A 40 mm, 200 g box
  slips out of the jaws in about half of the tries, more often on the left
  gripper. Finger friction and contact shapes need work before heavier objects.
- Gripper zero: in the URDF, 0 rad is about 90 degrees open and about 1.6 rad is
  closed. This still has to be checked against the real gripper.
- RoboCasa's cameras are still placed for the Panda. The default view is blocked
  by SYSMIC's tower, so the scripts open a free camera instead.
- The stand-off distance from counters has not been tuned for SYSMIC's shorter arms.
- In RoboCasa the wheels do not collide with kitchen objects; the platform box does.
- The URDF has no lidar, camera or IMU.

## Roadmap

- Sensors in the URDF, with laser scan and camera topics over the ROS bridge
- `ros2_control` / `FollowJointTrajectory` interface for MoveIt 2
- Map and Nav2 configuration
- Cameras and stand-off distance for RoboCasa tasks, then demo recording and policy training
- A hospitality environment (only kitchens so far)

## Troubleshooting

| Message | Cause | Fix |
|---|---|---|
| `Could not find the Python of conda env 'robocasa'` | Env missing or in an unusual place | Run `bash install_sim.sh`, or pass `conda_python:=/path/to/envs/robocasa/bin/python` |
| `Package 'sysmic_sim' not found` | Workspace not built or not sourced | `colcon build --symlink-install`, then `source install/setup.bash` |
| `colcon build` fails with Python errors | Conda env active in that terminal | `conda deactivate`, open a new terminal |
| `No module named 'rclpy'` | ROS not sourced, or conda is active | `conda deactivate`, then `source /opt/ros/humble/setup.bash` |
| `No module named 'robocasa'` (running a `sim/` script by hand) | Conda env not active | `conda activate robocasa` |
| `Cannot install on Python version 3.14` (numba) | pip ran in the conda `base` env | `conda activate robocasa`, check `python --version` |
| `CondaToSNonInteractiveError` | Conda terms not accepted | Run the two `conda tos accept` lines |
| `No module named 'tkinter'` | Tk missing in the env | `conda install -c conda-forge tk` |
| Bridge prints `No data from the simulation yet` | Simulation still loading, or it crashed | Kitchens take some seconds to load; otherwise read the simulation's error above |
| Robot does not move from `/cmd_vel` | Publisher stopped | Publish continuously; the base stops after 0.5 s of silence |
| RViz shows no robot | Display settings | Fixed Frame `odom`; RobotModel topic `/robot_description` |
| Meshes not found after moving the workspace | Stale generated models | `rm -rf ~/.cache/sysmic_sim` |
| `No private macro file`, `robosuite_models`, `mink`, `mimicgen` warnings | Optional extras | Harmless |
| `Wayland: The platform does not provide the window position` | Viewer on Wayland | Harmless |
| `Geom with duplicate name ''` | URDF geoms are unnamed | Harmless |
