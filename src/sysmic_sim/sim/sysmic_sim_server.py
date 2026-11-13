#!/usr/bin/env python3
"""SYSMIC V2 in a RoboCasa kitchen, as a server for the ROS 2 bridge.

Normally started by the launch file:

    ros2 launch sysmic_sim sim.launch.py

By hand, in the robocasa conda env (Python 3.11):

    python sysmic_sim_server.py                    # kitchen layout 1, style 1
    python sysmic_sim_server.py --layout 5 --style 12
    python sysmic_sim_server.py --no-viewer        # headless

The two sides talk over UDP on this machine only (JSON text):

    state    server -> 127.0.0.1:9870   every control step (20 Hz)
    command  bridge -> 127.0.0.1:9871   whenever ROS sends something

State packet
    {"t": sim time (s), "seq": n,
     "names": [...], "pos": [...], "vel": [...],        joints, URDF names
     "odom": {"x","y","yaw","vx","vy","wz"},            pose since start, twist in base frame
     "world": {"x","y","yaw"}}                          pose in the kitchen's frame

Command packet (any subset of the keys)
    {"cmd_vel": [v m/s, w rad/s],
     "joints": {"lift_joint": 0.3, "left_elbow_flex": -1.0, "right_gripper": 1.2, ...},
     "stop": true}

Safety: if no cmd_vel arrives for CMD_VEL_TIMEOUT seconds the base stops.
"""
import argparse
import json
import socket
import time

import mujoco
import numpy as np

import sysmic_robosuite as S

STATE_ADDR = ("127.0.0.1", 9870)
COMMAND_ADDR = ("127.0.0.1", 9871)
CMD_VEL_TIMEOUT = 0.5  # s of wall time
MAX_SPEED, MAX_TURN = 0.5, 1.0  # m/s, rad/s: top speeds of the base motors
# The base motors run slightly slow against joint friction; add it back.
SPEED_TRIM, TURN_TRIM = 0.003, 0.012

ARM = [f"{side}_{j}" for side in ("left", "right") for j in S.ARM_JOINTS]
CASTERS = ("FL", "FR", "RL", "RR")


def joint_map(env):
    """URDF joint name -> joint name in the RoboCasa model."""
    robot = env.robots[0]
    r = robot.robot_model.naming_prefix  # robot0_
    b = robot.robot_model.base.naming_prefix  # mobilebase0_
    m = {"lift_joint": f"{r}torso_lift_joint"}
    for name in ARM:
        m[name] = r + name
    for side in ("left", "right"):
        m[f"{side}_gripper"] = f"{robot.gripper[side].naming_prefix}jaw_joint"
        m[f"wheel_{side}_joint"] = f"{b}passive_wheel_{side}_spin"
        m[f"wheel_{side}_suspension"] = f"{b}passive_wheel_{side}_susp"
    for c in CASTERS:
        m[f"caster_{c}_swivel_joint"] = f"{b}passive_caster_{c}_swivel"
        m[f"caster_{c}_wheel_joint"] = f"{b}passive_caster_{c}_wheel"
    return m


class Commands:
    """Latest targets received from ROS; arms, lift and grippers hold their last value."""

    def __init__(self):
        self.reset()

    def reset(self):
        self.cmd_vel = (0.0, 0.0)
        self.cmd_vel_time = 0.0
        self.lift = 0.2
        self.arm = {name: S.ARM_READY[i % 5] for i, name in enumerate(ARM)}
        self.gripper = {"left": S.GRIPPER_OPEN, "right": S.GRIPPER_OPEN}

    def apply(self, msg, now):
        """Merge one command packet. Returns a list of names that were not understood."""
        unknown = []
        if msg.get("stop"):
            self.cmd_vel = (0.0, 0.0)
        if "cmd_vel" in msg:
            v, w = msg["cmd_vel"]
            self.cmd_vel = (float(v), float(w))
            self.cmd_vel_time = now
        for name, value in (msg.get("joints") or {}).items():
            value = float(value)
            if not np.isfinite(value):
                unknown.append(name)
            elif name == "lift_joint":
                self.lift = float(np.clip(value, *S.LIFT_RANGE))
            elif name in self.arm:
                self.arm[name] = value
            elif name in ("left_gripper", "right_gripper"):
                self.gripper[name.split("_")[0]] = value
            else:
                unknown.append(name)
        return unknown

    def base(self, now):
        """(forward, turn) as fractions of top speed; zero once cmd_vel is stale."""
        if now - self.cmd_vel_time > CMD_VEL_TIMEOUT:
            return 0.0, 0.0
        v, w = self.cmd_vel
        v = v + np.sign(v) * SPEED_TRIM
        w = w + np.sign(w) * TURN_TRIM
        return float(np.clip(v / MAX_SPEED, -1, 1)), float(np.clip(w / MAX_TURN, -1, 1))

    def action(self, env, now):
        return S.make_action(
            env, base=self.base(now), lift=self.lift,
            right_arm=[self.arm[f"right_{j}"] for j in S.ARM_JOINTS],
            left_arm=[self.arm[f"left_{j}"] for j in S.ARM_JOINTS],
            right_gripper=S.gripper_action(self.gripper["right"]),
            left_gripper=S.gripper_action(self.gripper["left"]))


class StateReader:
    """Reads joint and base state out of the simulation."""

    def __init__(self, env):
        self.env = env
        self.names = list(joint_map(env))
        model = env.sim.model._model
        ids = [model.joint(n).id for n in joint_map(env).values()]
        self.qpos = [int(model.jnt_qposadr[i]) for i in ids]
        self.qvel = [int(model.jnt_dofadr[i]) for i in ids]
        prefix = env.robots[0].robot_model.naming_prefix
        self.base_body = model.body(f"{prefix}base_link").id
        self.origin = None  # odom frame = where the robot stood at start
        self.last = None  # previous (x, y, yaw, time), for the twist

    def base_pose(self):
        data = self.env.sim.data._data
        pos = data.xpos[self.base_body]
        mat = data.xmat[self.base_body].reshape(3, 3)
        return float(pos[0]), float(pos[1]), float(np.arctan2(mat[1, 0], mat[0, 0]))

    def read(self, seq, peek=False):
        model, data = self.env.sim.model._model, self.env.sim.data._data
        x, y, yaw = self.base_pose()
        if self.origin is None:
            self.origin = (x, y, yaw)
        ox, oy, oyaw = self.origin
        c, s = np.cos(oyaw), np.sin(oyaw)
        dx, dy = x - ox, y - oy
        # twist in the base frame, averaged over the last control step so that it
        # integrates to exactly the pose change reported in "odom"
        t = float(data.time)
        if self.last is not None and t > self.last[3]:
            px, py, pyaw, pt = self.last
            dt = t - pt
            wx, wy = (x - px) / dt, (y - py) / dt
            mid = pyaw + ((yaw - pyaw + np.pi) % (2 * np.pi) - np.pi) / 2
            vx = np.cos(mid) * wx + np.sin(mid) * wy
            vy = -np.sin(mid) * wx + np.cos(mid) * wy
            wz = ((yaw - pyaw + np.pi) % (2 * np.pi) - np.pi) / dt
        else:
            vel = np.zeros(6)  # [angular, linear] in the base frame
            mujoco.mj_objectVelocity(model, data, mujoco.mjtObj.mjOBJ_BODY, self.base_body, vel, 1)
            vx, vy, wz = vel[3], vel[4], vel[2]
        if not peek:
            self.last = (x, y, yaw, t)
        return {
            "t": float(data.time), "seq": seq, "names": self.names,
            "pos": [float(data.qpos[i]) for i in self.qpos],
            "vel": [float(data.qvel[i]) for i in self.qvel],
            "odom": {"x": float(c * dx + s * dy), "y": float(-s * dx + c * dy),
                     "yaw": float((yaw - oyaw + np.pi) % (2 * np.pi) - np.pi),
                     "vx": float(vx), "vy": float(vy), "wz": float(wz)},
            "world": {"x": x, "y": y, "yaw": yaw},
        }


def open_sockets():
    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    rx.bind(COMMAND_ADDR)
    rx.setblocking(False)
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    return rx, tx


def drain(rx, commands, now, log=print):
    """Apply every command packet that has arrived."""
    count = 0
    while True:
        try:
            packet, _ = rx.recvfrom(65535)
        except BlockingIOError:
            return count
        try:
            unknown = commands.apply(json.loads(packet.decode()), now)
        except (ValueError, TypeError, KeyError) as exc:
            log(f"ignored a bad command packet: {exc}")
            continue
        if unknown:
            log(f"ignored unknown joints: {', '.join(unknown)}")
        count += 1


def serve(env, render=True, realtime=True, max_steps=None, log=print):
    rx, tx = open_sockets()
    commands, reader = Commands(), StateReader(env)
    period = 1.0 / env.control_freq
    seq, last_report, behind = 0, time.time(), 0.0
    try:
        while max_steps is None or seq < max_steps:
            start = time.time()
            drain(rx, commands, start, log)
            if render and S.viewer_closed(env):
                break
            env.step(commands.action(env, start))
            if render:
                env.render()
            tx.sendto(json.dumps(reader.read(seq)).encode(), STATE_ADDR)
            seq += 1
            spent = time.time() - start
            behind = 0.95 * behind + 0.05 * (period / max(spent, period) if realtime else 1.0)
            if realtime and spent < period:
                time.sleep(period - spent)
            if time.time() - last_report > 5.0:
                last_report = time.time()
                x, y, yaw = (reader.read(seq, peek=True)["odom"][k] for k in ("x", "y", "yaw"))
                log(f"sim time {env.sim.data._data.time:7.1f} s | odom x {x:+.2f} y {y:+.2f} "
                    f"yaw {yaw:+.2f} | speed {behind:.2f}x real time")
    except KeyboardInterrupt:
        pass
    finally:
        rx.close()
        tx.close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--task", default="Kitchen")
    ap.add_argument("--layout", type=int, default=1)
    ap.add_argument("--style", type=int, default=1)
    ap.add_argument("--no-viewer", action="store_true")
    ap.add_argument("--viewer", choices=["true", "false"], default="true",
                    help="same switch in a form launch files can pass")
    args = ap.parse_args()

    import robosuite

    extra = {}
    if args.task not in robosuite.ALL_ENVIRONMENTS or args.task == "Kitchen":
        extra = dict(layout_ids=args.layout, style_ids=args.style)
    view = not args.no_viewer and args.viewer == "true"
    env = S.make_env(args.task, has_renderer=view,
                     **(dict(renderer="mjviewer", render_camera=None) if view else {}), **extra)
    env.reset()
    print(f"SYSMIC simulation server running. State -> udp {STATE_ADDR[1]}, "
          f"commands <- udp {COMMAND_ADDR[1]}. Ctrl+C or close the viewer to stop.")
    serve(env, render=view)
    env.close()
    print("Stopped.")


if __name__ == "__main__":
    main()
