#!/usr/bin/env python3

import gymnasium as gym
import mujoco
import numpy as np

class SysmicV2Env(gym.Env):
    metadata = {'render_modes': ['human'], 'render_fps': 50}

    def __init__(self, mjcf_file, render_mode=None, max_episode_steps=500):
        super().__init__()

        self.model = mujoco.MjModel.from_xml_path(mjcf_file)
        self.data = mujoco.MjData(self.model)

        self.frame_skip = 10
        self.render_mode = render_mode
        self.max_episode_steps = max_episode_steps
        self.step_count = 0

        # Action Space: 13 dimensions
        # - Lift control mechanism (6 dims): effort
        # - Left arm joints (6 dims): position
        # - Right arm joints (6 dims): position
        self.action_space = gym.spaces.Box(low=-1.0, high=1.0, shape=(13,), dtype=np.float32)

        # Observation Space: 69 dimensions
        # - Current State for lift control mechanism (1 dim)
        # - Current State for left arm (13 dims): 6 joints, 3 EEF pos, 4 EE quat
        # - Current State for right arm (13 dims): 6 joints, 3 EEF pos, 4 EE quat
        # - Current Velocity for lift control mechanism (1 dim)
        # - Current Velocity for left arm (6 dims)
        # - Current Velocity for right arm (6 dims)
        # - Previous Action (13 dims)
        # - Goal State for left arm (8 dims): 3 EE pos, 4 EE quat, 1 gripper state
        # - Goal State for right arm (8 dims): 3 EE pos, 4 EE quat, 1 gripper state
        self.observation_space = gym.spaces.Box(low=-np.inf, high=np.inf, shape=(69,), dtype=np.float32)

        arm_links = [
            'left_link0',
            'left_link1',
            'left_link2',
            'left_link3',
            'left_link4',
            'left_link5',
            'left_jaw',
            'right_link0',
            'right_link1',
            'right_link2',
            'right_link3',
            'right_link4',
            'right_link5',
            'right_jaw',
        ]
        self.arm_body_ids = {
            self.model.body(name).id
            for name in arm_links
            if self.model.body(name).id >= 0
        }

        self.lift_joint_id = self.model.joint('lift_joint').id
        self.lift_qpos_id = int(self.model.joint('lift_joint').qposadr[0])
        self.lift_dof_id = int(self.model.joint('lift_joint').dofadr[0])
        self.left_arm_joints = [
            'left_shoulder_pan',
            'left_shoulder_lift',
            'left_elbow_flex',
            'left_wrist_flex',
            'left_wrist_roll',
            'left_gripper',
        ]
        self.right_arm_joints = [
            'right_shoulder_pan',
            'right_shoulder_lift',
            'right_elbow_flex',
            'right_wrist_flex',
            'right_wrist_roll',
            'right_gripper',
        ]
        self.arm_joints = self.left_arm_joints + self.right_arm_joints
        self.arm_qpos_ids = np.array(
            [self.model.joint(j).qposadr[0] for j in self.arm_joints],
            dtype=int
        )
        self.arm_dof_ids = np.array(
            [self.model.joint(j).dofadr[0] for j in self.arm_joints],
            dtype=int
        )

        self.base_link_id = self.model.body('base_link').id
        self.lift_carriage_id = self.model.body('lift_carriage_link').id
        self.left_jaw_id = self.model.body('left_jaw').id
        self.right_jaw_id = self.model.body('right_jaw').id

        joint_to_actuator = {}
        for actuator_id in range(self.model.nu):
            joint_id = self.model.actuator_trnid[actuator_id, 0]
            if joint_id >= 0:
                joint_to_actuator[joint_id] = actuator_id

        self.lift_actuator_id = int(joint_to_actuator[self.lift_joint_id])
        self.arm_actuator_ids = np.array(
            [joint_to_actuator[self.model.joint(j).id] for j in self.arm_joints],
            dtype=int,
        )

        self.lift_min = float(self.model.jnt_range[self.lift_joint_id, 0])
        self.lift_max = float(self.model.jnt_range[self.lift_joint_id, 1])
        self.lift_ctrl_min, self.lift_ctrl_max = self.model.actuator_ctrlrange[
            self.lift_actuator_id
        ]
        self.arm_min_limits = np.array(
            [self.model.jnt_range[self.model.joint(j).id, 0] for j in self.arm_joints],
            dtype=np.float32,
        )
        self.arm_max_limits = np.array(
            [self.model.jnt_range[self.model.joint(j).id, 1] for j in self.arm_joints],
            dtype=np.float32,
        )

        # Control & smoothness parameters (50 Hz policy at default dt=0.002s)
        self.arm_action_scale = 0.02    # rad per step (~1.0 rad/s max)
        self.lift_action_scale = 0.005  # m per step (~0.25 m/s max)
        self.action_ema_alpha = 0.25    # Lower = smoother, higher = snappier

        # Software PD gains for motor-driven lift (~17.1 kg total, passive damping=50)
        self.lift_kp = 1500.0
        self.lift_kd = 270.0

        # Anti-windup bounds between commanded setpoint and actual joint state
        self.lift_max_lag = 0.03  # m
        self.arm_max_lag = 0.15   # rad

        self.lift_target_qpos = 0.0
        self.arm_target_qpos = np.zeros(12, dtype=np.float32)
        self.smoothed_action = np.zeros(13, dtype=np.float32)

        self.left_target_pos = np.zeros(3, dtype=np.float32)
        self.left_target_quat = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)
        self.left_target_gripper = 0.0
        self.right_target_pos = np.zeros(3, dtype=np.float32)
        self.right_target_quat = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)
        self.right_target_gripper = 0.0

    @staticmethod
    def _random_quat_wxyz():
        u1, u2, u3 = np.random.random(3)
        w = np.sqrt(1.0 - u1) * np.sin(2.0 * np.pi * u2)
        x = np.sqrt(1.0 - u1) * np.cos(2.0 * np.pi * u2)
        y = np.sqrt(u1) * np.sin(2.0 * np.pi * u3)
        z = np.sqrt(u1) * np.cos(2.0 * np.pi * u3)
        q = np.array([w, x, y, z], dtype=np.float32)
        return q / np.linalg.norm(q)

    def _get_arm_collisions(self):
        collisions = []
        for i in range(self.data.ncon):
            con = self.data.contact[i]
            b1 = self.model.geom_bodyid[con.geom1]
            b2 = self.model.geom_bodyid[con.geom2]
            if (b1 in self.arm_body_ids or b2 in self.arm_body_ids) and b1 != b2:
                collisions.append((b1, b2, float(con.dist)))
        return collisions

    def _get_obs(self):
        lift_qpos = np.array([self.data.qpos[self.lift_qpos_id]], dtype=np.float32)
        left_arm_qpos = self.data.qpos[self.arm_qpos_ids[:6]]
        left_ee_pos = self.data.xpos[self.left_jaw_id]
        left_ee_quat = self.data.xquat[self.left_jaw_id]
        right_arm_qpos = self.data.qpos[self.arm_qpos_ids[6:]]
        right_ee_pos = self.data.xpos[self.right_jaw_id]
        right_ee_quat = self.data.xquat[self.right_jaw_id]

        lift_qvel = np.array([self.data.qvel[self.lift_dof_id]], dtype=np.float32)
        arm_qvel = self.data.qvel[self.arm_dof_ids]

        current_state = np.concatenate([
            lift_qpos,
            left_arm_qpos,
            left_ee_pos,
            left_ee_quat,
            right_arm_qpos,
            right_ee_pos,
            right_ee_quat,
            lift_qvel,
            arm_qvel,
            self.smoothed_action,
        ])

        target_state = np.concatenate([
            self.left_target_pos,
            self.left_target_quat,
            np.array([self.left_target_gripper], dtype=np.float32),
            self.right_target_pos,
            self.right_target_quat,
            np.array([self.right_target_gripper], dtype=np.float32),
        ])

        return np.concatenate([current_state, target_state]).astype(np.float32)

    def _sample_target(self):
        base_pos = self.data.xpos[self.base_link_id]
        carriage_x = base_pos[0] - 0.117
        carriage_z_zero = base_pos[2] + 0.64
        virtual_lift_z = carriage_z_zero + np.random.uniform(self.lift_min, self.lift_max)

        self.left_target_pos = np.array(
            [
                carriage_x + np.random.uniform(0.15, 0.35),
                base_pos[1] + np.random.uniform(0.02, 0.25),
                virtual_lift_z + np.random.uniform(-0.12, 0.15),
            ],
            dtype=np.float32,
        )
        self.left_target_quat = self._random_quat_wxyz()
        self.left_target_gripper = float(
            np.random.uniform(self.arm_min_limits[5], self.arm_max_limits[5])
        )

        self.right_target_pos = np.array(
            [
                carriage_x + np.random.uniform(0.15, 0.35),
                base_pos[1] + np.random.uniform(-0.25, -0.02),
                virtual_lift_z + np.random.uniform(-0.12, 0.15),
            ],
            dtype=np.float32,
        )
        self.right_target_quat = self._random_quat_wxyz()
        self.right_target_gripper = float(
            np.random.uniform(self.arm_min_limits[11], self.arm_max_limits[11])
        )

    def _set_target(
        self,
        left_target_pos,
        left_target_quat,
        left_target_gripper,
        right_target_pos,
        right_target_quat,
        right_target_gripper,
    ):
        if left_target_pos is not None:
            self.left_target_pos = np.asarray(left_target_pos, dtype=np.float32)
        if left_target_quat is not None:
            self.left_target_quat = np.asarray(left_target_quat, dtype=np.float32)
        if left_target_gripper is not None:
            self.left_target_gripper = float(left_target_gripper)
        if right_target_pos is not None:
            self.right_target_pos = np.asarray(right_target_pos, dtype=np.float32)
        if right_target_quat is not None:
            self.right_target_quat = np.asarray(right_target_quat, dtype=np.float32)
        if right_target_gripper is not None:
            self.right_target_gripper = float(right_target_gripper)

    def reset(self, seed=None, options=None):
        super().reset(seed=seed, options=options)
        options = options or {}
        reposition = options.get('reposition', True)
        left_target_pos = options.get('left_target_pos', None)
        left_target_quat = options.get('left_target_quat', None)
        left_target_gripper = options.get('left_target_gripper', None)
        right_target_pos = options.get('right_target_pos', None)
        right_target_quat = options.get('right_target_quat', None)
        right_target_gripper = options.get('right_target_gripper', None)

        if reposition:
            mujoco.mj_resetData(self.model, self.data)
            init_arm_qpos = np.random.uniform(-0.2, 0.2, size=len(self.arm_qpos_ids))
            self.data.qpos[self.lift_qpos_id] = np.random.uniform(
                self.lift_min, self.lift_max
            )
            self.data.qpos[self.arm_qpos_ids] = np.clip(
                init_arm_qpos, self.arm_min_limits, self.arm_max_limits
            )

        mujoco.mj_forward(self.model, self.data)

        self.step_count = 0
        self.lift_target_qpos = float(self.data.qpos[self.lift_qpos_id])
        self.arm_target_qpos = self.data.qpos[self.arm_qpos_ids].astype(np.float32).copy()
        self.smoothed_action = np.zeros(13, dtype=np.float32)

        self.data.ctrl[self.lift_actuator_id] = np.clip(
            self.data.qfrc_bias[self.lift_dof_id],
            self.lift_ctrl_min,
            self.lift_ctrl_max,
        )
        self.data.ctrl[self.arm_actuator_ids] = self.arm_target_qpos

        if any(
            x is not None
            for x in (
                left_target_pos, left_target_quat, left_target_gripper,
                right_target_pos, right_target_quat, right_target_gripper,
            )
        ):
            self._set_target(
                left_target_pos, left_target_quat, left_target_gripper,
                right_target_pos, right_target_quat, right_target_gripper,
            )
        else:
            self._sample_target()

        try:
            left_mocap_id = self.model.body('left_target').mocapid[0]
            self.data.mocap_pos[left_mocap_id] = self.left_target_pos
            self.data.mocap_quat[left_mocap_id] = self.left_target_quat
            right_mocap_id = self.model.body('right_target').mocapid[0]
            self.data.mocap_pos[right_mocap_id] = self.right_target_pos
            self.data.mocap_quat[right_mocap_id] = self.right_target_quat
        except Exception:
            pass

        mujoco.mj_forward(self.model, self.data)
        return self._get_obs(), {}

    def step(self, action):
        self.step_count += 1
        raw_action = np.clip(np.asarray(action, dtype=np.float32), -1.0, 1.0)

        # 1. Low-pass filter (EMA) to eliminate high-frequency jitter
        prev_smoothed = self.smoothed_action.copy()
        self.smoothed_action = (
            self.action_ema_alpha * raw_action
            + (1.0 - self.action_ema_alpha) * prev_smoothed
        )

        # 2. Update setpoints with anti-windup bounds around current joint state
        prev_lift_target = self.lift_target_qpos
        prev_arm_target = self.arm_target_qpos.copy()

        current_lift_qpos = float(self.data.qpos[self.lift_qpos_id])
        current_arm_qpos = self.data.qpos[self.arm_qpos_ids]

        next_lift = prev_lift_target + float(self.smoothed_action[0]) * self.lift_action_scale
        next_lift = np.clip(
            next_lift,
            current_lift_qpos - self.lift_max_lag,
            current_lift_qpos + self.lift_max_lag,
        )
        self.lift_target_qpos = float(np.clip(next_lift, self.lift_min, self.lift_max))

        next_arm = prev_arm_target + self.smoothed_action[1:] * self.arm_action_scale
        next_arm = np.clip(
            next_arm,
            current_arm_qpos - self.arm_max_lag,
            current_arm_qpos + self.arm_max_lag,
        )
        self.arm_target_qpos = np.clip(
            next_arm, self.arm_min_limits, self.arm_max_limits
        ).astype(np.float32)

        # 3. Substep interpolation + software PD/gravity-comp lift control
        for sub in range(self.frame_skip):
            alpha = (sub + 1) / self.frame_skip
            interp_lift = (1.0 - alpha) * prev_lift_target + alpha * self.lift_target_qpos
            interp_arm = (1.0 - alpha) * prev_arm_target + alpha * self.arm_target_qpos

            lift_err = interp_lift - self.data.qpos[self.lift_qpos_id]
            lift_vel = self.data.qvel[self.lift_dof_id]
            lift_bias = self.data.qfrc_bias[self.lift_dof_id]
            lift_force = lift_bias + self.lift_kp * lift_err - self.lift_kd * lift_vel
            self.data.ctrl[self.lift_actuator_id] = np.clip(
                lift_force, self.lift_ctrl_min, self.lift_ctrl_max
            )
            self.data.ctrl[self.arm_actuator_ids] = interp_arm

            mujoco.mj_step(self.model, self.data)

        obs = self._get_obs()

        # ----------------------------------------------------
        # REWARD FUNCTION
        # ----------------------------------------------------
        left_ee_pos = self.data.xpos[self.left_jaw_id]
        left_ee_quat = self.data.xquat[self.left_jaw_id]
        left_current_gripper = self.data.qpos[self.arm_qpos_ids[5]]

        right_ee_pos = self.data.xpos[self.right_jaw_id]
        right_ee_quat = self.data.xquat[self.right_jaw_id]
        right_current_gripper = self.data.qpos[self.arm_qpos_ids[11]]

        left_error_pos = float(np.linalg.norm(left_ee_pos - self.left_target_pos))
        right_error_pos = float(np.linalg.norm(right_ee_pos - self.right_target_pos))

        left_quat_dot = np.abs(np.dot(left_ee_quat, self.left_target_quat))
        left_error_quat = float(1.0 - np.clip(left_quat_dot, 0.0, 1.0))
        right_quat_dot = np.abs(np.dot(right_ee_quat, self.right_target_quat))
        right_error_quat = float(1.0 - np.clip(right_quat_dot, 0.0, 1.0))

        left_error_gripper = float(np.abs(left_current_gripper - self.left_target_gripper))
        right_error_gripper = float(np.abs(right_current_gripper - self.right_target_gripper))

        collisions = self._get_arm_collisions()
        penetration = sum(max(0.0, -dist) for _, _, dist in collisions)

        action_delta = raw_action - prev_smoothed
        reward_action_rate = -0.05 * float(np.sum(action_delta ** 2))
        all_qvel = np.append(
            self.data.qvel[self.arm_dof_ids], self.data.qvel[self.lift_dof_id]
        )
        reward_vel = -0.005 * float(np.sum(all_qvel ** 2))

        reward_pos = -2.0 * (left_error_pos + right_error_pos)
        reward_quat = -4.0 * (left_error_quat + right_error_quat)
        reward_gripper = -1.0 * (left_error_gripper + right_error_gripper)
        reward_collision = -10.0 * len(collisions) - 100.0 * penetration

        success = (
            (left_error_pos < 0.01)
            and (left_error_quat < 0.01)
            and (left_error_gripper < 0.01)
            and (right_error_pos < 0.01)
            and (right_error_quat < 0.01)
            and (right_error_gripper < 0.01)
        )
        reward_success = 100.0 if success else 0.0

        reward = (
            reward_pos +
            reward_quat +
            reward_gripper +
            reward_collision +
            reward_action_rate +
            reward_vel +
            reward_success
        )

        # ----------------------------------------------------
        # TERMINATION CONDITIONS
        # ----------------------------------------------------
        terminated = bool(success)
        truncated = self.step_count >= self.max_episode_steps

        return obs, reward, terminated, truncated, {
            'success': success,
            'left_error_position': left_error_pos,
            'left_error_orientation': left_error_quat,
            'left_error_gripper': left_error_gripper,
            'right_error_position': right_error_pos,
            'right_error_orientation': right_error_quat,
            'right_error_gripper': right_error_gripper,
            'collisions': len(collisions),
            'penetration': penetration
        }