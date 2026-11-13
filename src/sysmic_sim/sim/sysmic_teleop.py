#!/usr/bin/env python3
"""Drive SYSMIC V2 in a RoboCasa kitchen from a control panel.

    conda activate robocasa
    python sysmic_teleop.py                      # kitchen layout 1, style 1
    python sysmic_teleop.py --layout 5 --style 12

Two windows open: the MuJoCo viewer and a small "SYSMIC control" panel.

Panel
  Base      hold W/S (or Up/Down) to drive, A/D (or Left/Right) to turn.
            Click on the panel first so it receives the keys.
            Releasing the key stops the robot. Space or the red STOP button
            stops it at once.
            The two speed sliders set how fast.
  Lift      height in metres
  Arms      one slider per joint (radians)
  Grippers  tick to close, untick to open
  Buttons   Ready pose / Reset scene / Quit
Closing either window also quits.
"""
import argparse
import time

import numpy as np

import sysmic_robosuite as S

VERSION = "v3"
DRIVE_KEYS = {"w": (1, 0), "Up": (1, 0), "s": (-1, 0), "Down": (-1, 0),
              "a": (0, 1), "Left": (0, 1), "d": (0, -1), "Right": (0, -1)}


class DriveKeys:
    """Which drive keys are held right now.

    Keyboard auto-repeat sends release+press pairs while a key is held. After a
    release the main loop waits GRACE seconds for the matching press; if none
    comes the key really was let go and the robot stops on that same step.

    Safety net: if the release event never reaches the panel (it happens on some
    desktops), a held key normally keeps sending repeat presses. Once those
    repeats stop for STALE seconds the key is dropped anyway.
    """

    GRACE = 0.02
    STALE = 0.7

    def __init__(self):
        self.down = {}  # key -> time of the latest press / repeat
        self.since = {}  # key -> time of the first press
        self.released = {}
        self.repeating = set()  # keys whose current hold has already auto-repeated
        self.repeat_delay = None  # longest wait seen before auto-repeat starts

    def press(self, key, now=None):
        if key not in DRIVE_KEYS:
            return
        now = time.time() if now is None else now
        if key in self.down or key in self.released:  # auto-repeat
            if key not in self.repeating:  # first repeat of this hold: how long did it take?
                self.repeating.add(key)
                self.repeat_delay = max(self.repeat_delay or 0.0, now - self.since.get(key, now))
        else:
            self.since[key] = now
            self.repeating.discard(key)
        self.down[key] = now
        self.released.pop(key, None)

    def release(self, key, now=None):
        if key in self.down:
            del self.down[key]
            self.released[key] = time.time() if now is None else now

    def clear(self):
        self.down.clear()
        self.since.clear()
        self.released.clear()
        self.repeating.clear()

    def command(self, now=None):
        """(drive, turn), each in [-1, 1]."""
        now = time.time() if now is None else now
        self.released = {k: t for k, t in self.released.items() if now - t < self.GRACE}
        if self.repeat_delay is not None:  # auto-repeat is on: silence means the key is up
            limit = max(self.STALE, 1.5 * self.repeat_delay)
            for key in [k for k, t in self.down.items() if now - t > limit]:
                del self.down[key]
        for key in list(self.since):
            if key not in self.down and key not in self.released:
                del self.since[key]
        drive = turn = 0
        for key in set(self.down) | set(self.released):
            drive += DRIVE_KEYS[key][0]
            turn += DRIVE_KEYS[key][1]
        return float(np.clip(drive, -1, 1)), float(np.clip(turn, -1, 1))


class Teleop:
    """Holds the commanded targets and turns them into robosuite actions."""

    def __init__(self, env):
        self.env = env
        self.lift = 0.2
        self.arm = {"right": np.array(S.ARM_READY, dtype=float),
                    "left": np.array(S.ARM_READY, dtype=float)}
        self.grip_closed = {"right": False, "left": False}
        self.speed = {"drive": 0.6, "turn": 0.6}  # fraction of the base's top speed
        self.keys = DriveKeys()

    def joint_range(self, name):
        sim = self.env.sim
        prefix = self.env.robots[0].robot_model.naming_prefix
        return tuple(sim.model.jnt_range[sim.model.joint_name2id(prefix + name)])

    def base_command(self, now=None):
        drive, turn = self.keys.command(now)
        return drive * self.speed["drive"], turn * self.speed["turn"]

    def action(self, now=None):
        return S.make_action(
            self.env, base=self.base_command(now), lift=self.lift,
            right_arm=self.arm["right"], left_arm=self.arm["left"],
            right_gripper=1.0 if self.grip_closed["right"] else -1.0,
            left_gripper=1.0 if self.grip_closed["left"] else -1.0)

    def step(self, now=None):
        self.env.step(self.action(now))
        self.env.render()


def build_panel(teleop, on_reset, on_quit):
    import tkinter as tk

    root = tk.Tk()
    root.title(f"SYSMIC control {VERSION}")
    widgets = {}

    def slider(parent, label, lo, hi, value, setter, resolution=0.01):
        row = tk.Frame(parent)
        row.pack(fill="x", padx=6, pady=1)
        tk.Label(row, text=label, width=16, anchor="w").pack(side="left")
        var = tk.DoubleVar(value=value)
        tk.Scale(row, from_=lo, to=hi, resolution=resolution, orient="horizontal", length=260,
                 variable=var, takefocus=0, command=lambda v: setter(float(v))).pack(side="left")
        return var

    base = tk.LabelFrame(root, text="Base  (click here, hold W/S/A/D or arrows; Space = stop)")
    base.pack(fill="x", padx=8, pady=4)
    slider(base, "drive speed", 0.1, 1.0, teleop.speed["drive"],
           lambda v: teleop.speed.__setitem__("drive", v), 0.05)
    slider(base, "turn speed", 0.1, 1.0, teleop.speed["turn"],
           lambda v: teleop.speed.__setitem__("turn", v), 0.05)
    tk.Button(base, text="STOP", command=teleop.keys.clear, takefocus=0, bg="#c62828",
              fg="white", activebackground="#8e0000", activeforeground="white",
              font=("TkDefaultFont", 12, "bold")).pack(fill="x", padx=6, pady=4)
    status = tk.StringVar(value="base: stopped")
    tk.Label(base, textvariable=status, anchor="w").pack(fill="x", padx=6)
    widgets["status"] = status

    body = tk.LabelFrame(root, text="Lift")
    body.pack(fill="x", padx=8, pady=4)
    widgets["lift"] = slider(body, "height (m)", *S.LIFT_RANGE, teleop.lift,
                             lambda v: setattr(teleop, "lift", v), 0.005)

    for side in ("left", "right"):
        frame = tk.LabelFrame(root, text=f"{side.capitalize()} arm (rad)")
        frame.pack(fill="x", padx=8, pady=4)
        for i, joint in enumerate(S.ARM_JOINTS):
            lo, hi = teleop.joint_range(f"{side}_{joint}")

            def setter(v, side=side, i=i):
                teleop.arm[side][i] = v

            widgets[side, i] = slider(frame, joint, lo, hi, teleop.arm[side][i], setter)
        var = tk.BooleanVar(value=False)
        tk.Checkbutton(frame, text="close gripper", variable=var, takefocus=0,
                       command=lambda side=side, var=var: teleop.grip_closed.__setitem__(
                           side, bool(var.get()))).pack(anchor="w", padx=6)
        widgets[side, "grip"] = var

    def ready():
        teleop.lift = 0.2
        widgets["lift"].set(0.2)
        for side in ("left", "right"):
            teleop.arm[side][:] = S.ARM_READY
            for i, v in enumerate(S.ARM_READY):
                widgets[side, i].set(v)
            teleop.grip_closed[side] = False
            widgets[side, "grip"].set(False)

    def reset():
        teleop.keys.clear()
        ready()
        on_reset()

    buttons = tk.Frame(root)
    buttons.pack(fill="x", padx=8, pady=6)
    tk.Button(buttons, text="Ready pose", command=ready, takefocus=0).pack(side="left", padx=4)
    tk.Button(buttons, text="Reset scene", command=reset, takefocus=0).pack(side="left", padx=4)
    tk.Button(buttons, text="Quit", command=on_quit, takefocus=0).pack(side="right", padx=4)
    root.protocol("WM_DELETE_WINDOW", on_quit)

    # Keys belong to driving only: stop sliders / tick boxes from reacting to them,
    # and give the keyboard back to the panel after every mouse click.
    for cls in ("Scale", "Checkbutton", "Button"):
        for seq in ("<Left>", "<Right>", "<Up>", "<Down>", "<space>",
                    "<Key-Left>", "<Key-Right>", "<Key-Up>", "<Key-Down>", "<Key-space>"):
            root.unbind_class(cls, seq)
    root.bind_all("<ButtonRelease-1>", lambda e: root.focus_set(), add="+")

    def key_down(event):
        if event.keysym == "space":
            teleop.keys.clear()
        else:
            teleop.keys.press(event.keysym.lower() if len(event.keysym) == 1 else event.keysym)

    def key_up(event):
        teleop.keys.release(event.keysym.lower() if len(event.keysym) == 1 else event.keysym)

    root.bind("<KeyPress>", key_down)
    root.bind("<KeyRelease>", key_up)
    # if the panel loses the keyboard while a key is held, the release never arrives
    root.bind("<FocusOut>", lambda e: teleop.keys.clear())
    root.focus_set()
    return root, widgets


def run(env, teleop, root, widgets, state, max_ticks=None):
    """Main loop: drain all pending panel events, then advance the simulation one step."""
    import tkinter as tk

    period = 1.0 / env.control_freq
    ticks, slow = 0, 0.0
    while not state["quit"] and (max_ticks is None or ticks < max_ticks):
        start = time.time()
        try:
            root.update()  # handles every queued key / slider event, so releases are seen at once
        except tk.TclError:
            break
        if teleop.keys.released:  # real release, or just auto-repeat? wait a moment and look again
            time.sleep(DriveKeys.GRACE)
            try:
                root.update()
            except tk.TclError:
                break
        if state["quit"] or S.viewer_closed(env):
            break
        teleop.step(now=time.time())
        ticks += 1
        while time.time() - start < period:  # sim is faster than real time: wait, stay responsive
            try:
                root.update()
            except tk.TclError:
                return
            time.sleep(0.002)
        slow = 0.9 * slow + 0.1 * (period / max(time.time() - start, 1e-6))
        if ticks % 10 == 0:
            drive, turn = teleop.base_command()
            held = " ".join(sorted(teleop.keys.down)) or "-"
            moving = ("stopped" if drive == 0 and turn == 0
                      else f"drive {drive:+.2f}  turn {turn:+.2f}") + f"   keys: {held}"
            widgets["status"].set(f"base: {moving}    sim speed: {slow:.2f}x real time")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--task", default="Kitchen")
    ap.add_argument("--layout", type=int, default=1)
    ap.add_argument("--style", type=int, default=1)
    args = ap.parse_args()

    import robosuite

    extra = {}
    if args.task not in robosuite.ALL_ENVIRONMENTS or args.task == "Kitchen":
        extra = dict(layout_ids=args.layout, style_ids=args.style)
    env = S.make_env(args.task, has_renderer=True, renderer="mjviewer",
                     render_camera=None, **extra)
    env.reset()
    teleop = Teleop(env)
    state = {"quit": False}
    root, widgets = build_panel(teleop, on_reset=env.reset,
                                on_quit=lambda: state.__setitem__("quit", True))
    print(f"SYSMIC teleop {VERSION}. Click the panel, then hold W/S/A/D (or arrow keys) to "
          "drive. Let go to stop; Space or the red STOP button also stops.")
    try:
        run(env, teleop, root, widgets, state)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            root.destroy()
        except Exception:
            pass
        env.close()
    print("Stopped.")


if __name__ == "__main__":
    main()
