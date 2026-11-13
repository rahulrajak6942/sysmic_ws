# SYSMIC V2 in Gazebo (Harmonic, gz-sim 8)

Tested headless with Gazebo Harmonic 8.10 (DART physics). All units are SI (m, kg, rad). Z is up and X is forward.

## Run
Linux:
    cd sysmic_ws/src/sysmic_sim/description
    export GZ_SIM_RESOURCE_PATH=$PWD/models
    gz sim -r sysmic_world.sdf

Windows: Gazebo is best run in WSL2 (Ubuntu 24.04) using the same commands.

## Drive and move (in a second terminal)
Send each command 2 times. A single one-shot publish can be lost before Gazebo connects to the topic.

    gz topic -t /sysmic/cmd_vel -m gz.msgs.Twist -p 'linear: {x: 0.4}'      # forward 0.4 m/s
    gz topic -t /sysmic/cmd_vel -m gz.msgs.Twist -p 'angular: {z: 1.0}'     # spin
    gz topic -t /sysmic/cmd_vel -m gz.msgs.Twist -p 'linear: {x: 0.0}'      # stop
    gz topic -t /sysmic/lift -m gz.msgs.Double -p 'data: 0.30'              # lift 0..0.4477 m
    gz topic -t /sysmic/left_shoulder_lift -m gz.msgs.Double -p 'data: 1.2' # any arm joint, radians

Arm joint topics: /sysmic/{left,right}_{shoulder_pan,shoulder_lift,elbow_flex,wrist_flex,wrist_roll,gripper}
Readouts: /sysmic/odom and /sysmic/joint_states

## Joint limits
| Joint | Limit |
|---|---|
| shoulder_pan | ±110° |
| shoulder_lift | 0 to 200° |
| elbow_flex | −194° to 0° |
| wrist_flex | ±95° |
| wrist_roll | −157° to 163° |
| gripper | −10° to 100° |
| lift | 0 to 447.7 mm, 1000 N |

Arm joints are limited to 2.9 N·m each (STS3215 servo).

## Differences from the plain URDF (sysmic_v2.urdf)
- Each drive wheel hangs on a sprung slider: 34 kN/m spring, 0 to 12 mm travel, with 4 mm of preload. In the sim the spring compresses fully and the casters touch the floor. Each drive wheel carries about 136 N.
- Wheel and caster collisions are simple cylinders, which gives stable floor contact.
- Includes plugins for differential drive, the joint controllers and joint-state publishing.

## Test results (scripted run, real torque limits)
- Stands level on all 6 wheels.
- Lift reaches 447.7 mm and holds it.
- Arm tracking error stays under 0.12 rad.
- Drives at 0.503 m/s (commanded 0.5) and spins at 0.99 rad/s (commanded 1.0), drifting 10 mm while spinning.
- Hard stop from 0.5 m/s with the lift fully up and the arms out: pitches 1.15°, stops in 41 mm, does not tip.
- Battery and electronics are not modelled yet (robot mass 56 kg without them).
