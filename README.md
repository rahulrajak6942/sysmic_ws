# SYSMIC 2.0 Simulation (RoboCasa + ROS 2)

SYSMIC 2.0 is a dual-arm mobile manipulator for hospitality tasks: a
differential-drive base with four casters, a 448 mm lift and two 5-joint SO-101
arms with grippers. This branch simulates it in RoboCasa kitchens (MuJoCo), with a
ROS 2 Humble bridge, cameras, lidars and an IMU.

Everything is in one ROS 2 package, [`src/sysmic_v2_sim`](src/sysmic_v2_sim):

```
src/sysmic_v2_sim/
├── urdf/                    SYSMIC 2.0 URDF + meshes (Fusion export, never edited)
├── sysmic_v2_robocasa/      standalone RoboCasa robot, usable without ROS:
│   ├── assets/              MuJoCo XML / MJCF files + meshes
│   ├── sysmic_robocasa.py   registers the robot "Sysmic" with robosuite
│   ├── sensors.yaml         sensors on/off and their parameters
│   ├── control_panel.py     slider for every joint + keyboard driving
│   └── demo.py
├── sim/                     URDF -> MuJoCo / RoboCasa model, simulation server
├── scripts/ros2_bridge.py   ROS 2 node
├── launch/sim.launch.py     one-command start
└── rviz/sysmic.rviz
```

## Use the robot in RoboCasa (no ROS)

The `sysmic_v2_robocasa` folder is self-contained: copy it anywhere.

```bash
conda activate robocasa
cd src/sysmic_v2_sim/sysmic_v2_robocasa
python control_panel.py      # kitchen viewer + a slider for every joint
python demo.py               # scripted: lift, arms, wrists, grippers, base
```

```python
import sys
sys.path.insert(0, "path/to/sysmic_v2_robocasa")
import sysmic_robocasa as S                       # registers "Sysmic" with robosuite

env = S.make_env("Kitchen", layout_ids=1, style_ids=1)
env.reset()
env.step(S.joint_action(env, {"lift_joint": 0.35, "arm_right_shoulder_joint": 1.4,
                              "arm_right_gripper_joint": 1.7}, base=(0.0, 0.0)))
```

MuJoCo files in `sysmic_v2_robocasa/assets/` (they share `assets/meshes/`):

| File | What it is |
|---|---|
| `robot.xml`, `base.xml`, `left_gripper.xml`, `right_gripper.xml`, `controller.json` | robosuite / RoboCasa robot |
| `sysmic_v2_scene.xml` | physical MuJoCo model on a floor: motors, real wheels, casters, suspension. `python -m mujoco.viewer --mjcf=sysmic_v2_scene.xml`, then the Control tab: lift, every arm / gripper joint, both wheels (positive = forward) |
| `sysmic_v2.xml` | plain URDF to MJCF conversion |

Details: [sysmic_v2_robocasa/README.md](src/sysmic_v2_sim/sysmic_v2_robocasa/README.md).

## Run with ROS 2

```bash
source /opt/ros/humble/setup.bash
cd ~/sysmic_ws && colcon build --symlink-install
source install/setup.bash
ros2 launch sysmic_v2_sim sim.launch.py          # MuJoCo viewer + bridge + RViz
```

Do not activate conda in that terminal; the launch file finds the `robocasa` env's
Python by itself.

| Topic | Type | |
|---|---|---|
| `/joint_states`, `/odom`, `/tf`, `/clock` | | 23 URDF joints, ground-truth odometry, sim time |
| `/cmd_vel` | `Twist` | base, up to 0.5 m/s and 1.0 rad/s |
| `/sysmic/joint_command` | `JointState` | lift, arms, grippers by URDF joint name |
| `/head_camera/color/image_raw`, `/head_camera/depth/image_raw` | `Image` (+ `camera_info`) | RealSense D435i in the face window |
| `/tray_camera/image_raw` | `Image` (+ `camera_info`) | webcam looking down at the tray |
| `/left_gripper_camera/image_raw`, `/right_gripper_camera/image_raw` | `Image` (+ `camera_info`) | webcams in the gripper palms |
| `/scan_top`, `/scan_base` | `LaserScan` | YDLIDARs on the head and the base, 10 m, 10 Hz |
| `/imu` | `Imu` | base IMU |

Move the joints:

```bash
ros2 topic pub --once /sysmic/joint_command sensor_msgs/msg/JointState \
  "{name: [lift_joint, arm_left_shoulder_joint, arm_right_gripper_joint], position: [0.35, 1.4, 1.7]}"
ros2 run teleop_twist_keyboard teleop_twist_keyboard          # base
```

Sliders for all joints (joint_state_publisher_gui, remapped to the command topic):

```bash
ros2 run joint_state_publisher_gui joint_state_publisher_gui --ros-args \
  -r joint_states:=/sysmic/joint_command -p zeros.lift_joint:=0.2 \
  -p zeros.arm_left_shoulder_joint:=1.0 -p zeros.arm_right_shoulder_joint:=1.0 \
  -p zeros.arm_left_elbow_joint:=-1.2 -p zeros.arm_right_elbow_joint:=-1.2 \
  -p zeros.arm_left_wrist_flex_joint:=0.4 -p zeros.arm_right_wrist_flex_joint:=0.4 \
  -p zeros.arm_left_gripper_joint:=-0.17 -p zeros.arm_right_gripper_joint:=-0.17
```

### Sensors

Every sensor that is `enabled` in
[`sensors.yaml`](src/sysmic_v2_sim/sysmic_v2_robocasa/sensors.yaml) starts with the
launch file; nothing else is needed. RViz shows both lidar scans and the head
camera. Tick the other Image displays in RViz (head depth, tray, left / right
gripper) or use `ros2 run rqt_image_view rqt_image_view` to see the other cameras.

```bash
ros2 topic hz /scan_top          # about 10 Hz
ros2 topic echo /imu --once
ros2 launch sysmic_v2_sim sim.launch.py sensors:=/path/to/my_sensors.yaml   # another config
```

`sensors.yaml` switches each sensor (or one D435i stream) on or off and sets its
position, resolution, field of view, rate, range, noise and visible box. Restart
the launch after editing; nothing has to be rebuilt. A camera is only
rendered while its topic has a subscriber. Each watched camera costs about 10 ms
per frame, so with all five at 30 Hz the simulation runs at about 0.4x real time.
Full details: [src/sysmic_v2_sim/README.md](src/sysmic_v2_sim/README.md).

## Install

Ubuntu 22.04 with ROS 2 Humble (`ros-humble-desktop`), about 20 GB of free disk
(RoboCasa kitchen assets are about 10 GB). No GPU is needed to run the simulation.

```bash
# 1. Miniconda (skip if `conda --version` works)
wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh
bash Miniconda3-latest-Linux-x86_64.sh -b
~/miniconda3/bin/conda init && source ~/.bashrc

# 2. Environment. Python 3.11 is required.
conda create -c conda-forge -n robocasa python=3.11 -y
conda activate robocasa

# 3. robosuite and RoboCasa
mkdir -p ~/robocasa_ws && cd ~/robocasa_ws
git clone https://github.com/ARISE-Initiative/robosuite
cd robosuite && pip install -e . && cd ..
git clone https://github.com/robocasa/robocasa
cd robocasa && pip install -e .
python -m robocasa.scripts.setup_macros
python -m robocasa.scripts.download_kitchen_assets     # about 10 GB
conda deactivate

# 4. This workspace (new terminal, no conda env active)
source /opt/ros/humble/setup.bash
git clone -b v2 https://github.com/rahulrajak6942/sysmic_ws.git ~/sysmic_ws
cd ~/sysmic_ws && colcon build --symlink-install
```

Optional: `sudo apt install ros-humble-teleop-twist-keyboard ros-humble-joint-state-publisher-gui`.

Tested with robosuite 1.5.2 (commit `5ce6643`), RoboCasa 1.0.1 (commit `3443cdf`),
MuJoCo 3.3.1, numpy 2.2.5, Python 3.11 (conda) and 3.10 (ROS 2 Humble), on a
Ryzen 5 4600H laptop with a GTX 1650.
