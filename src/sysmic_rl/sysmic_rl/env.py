#!/usr/bin/env python3

import gymnasium as gym
import mujoco
import numpy as np

class SysmicV2Env(gym.Env):
    metadata = {'render_modes': ['human'], 'render_fps': 50}

    def __init__(self, mjcf_file, render_mode=None):
        super().__init__()

        self.model = mujoco.MjModel.from_xml_path(mjcf_file)
        self.data = mujoco.MjData(self.model)
        self.render_mode = render_mode

        # Action Space: 12 dimensions
        # 6 continuous values for 5 left arm joints + left gripper
        # 6 continuous values for 5 right arm joints + right gripper
        self.action_space = gym.spaces.Box(low=-1.0, high=1.0, shape=(12,), dtype=np.float32)

        # Observation Space: 42 dimensions
        # - Current State for left arm (13 dims): 6 joint pos, 3 EE pos, 4 EE quat
        # - Current State for right arm (13 dims): 6 joint pos, 3 EE pos, 4 EE quat
        # - Goal State (8 dims for left arm): 3 EE pos, 4 EE quat, 1 gripper state
        # - Goal State (8 dims for right arm): 3 EE pos, 4 EE quat, 1 gripper state
        self.observation_space = gym.spaces.Box(low=-np.inf, high=np.inf, shape=(42,), dtype=np.float32)

        total_links = [
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
            for name in total_links
            if self.model.body(name).id >= 0
        }
        
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
        self.total_joints = self.left_arm_joints + self.right_arm_joints

        joint_to_actuator = {}
        for actuator_id in range(self.model.nu):
            joint_id = self.model.actuator_trnid[actuator_id, 0]
            if joint_id >= 0:
                joint_to_actuator[joint_id] = actuator_id
        self.actuator_ids = []
        for jnt in self.total_joints:
            joint_id = self.model.joint(jnt).id
            self.actuator_ids.append(joint_to_actuator[joint_id])
        self.actuator_ids = np.array(self.actuator_ids, dtype=int)

        self.min_limits = [self.model.jnt_range[self.model.joint(jnt).id, 0] for jnt in self.total_joints]
        self.max_limits = [self.model.jnt_range[self.model.joint(jnt).id, 1] for jnt in self.total_joints]

        self.frame_skip = 50
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
        left_arm_qpos = [self.data.qpos[self.model.joint(jname).qposadr[0]] for jname in self.left_arm_joints]
        left_ee_pos = self.data.body('left_jaw').xpos.copy()
        left_ee_quat = self.data.body('left_jaw').xquat.copy()
        right_arm_qpos = [self.data.qpos[self.model.joint(jname).qposadr[0]] for jname in self.right_arm_joints]
        right_ee_pos = self.data.body('right_jaw').xpos.copy()
        right_ee_quat = self.data.body('right_jaw').xquat.copy()
        current_state = np.concatenate([
            left_arm_qpos,
            left_ee_pos,
            left_ee_quat,
            right_arm_qpos,
            right_ee_pos,
            right_ee_quat
        ])

        target_state = np.concatenate([
            self.left_target_pos,
            self.left_target_quat,
            np.array([self.left_target_gripper], dtype=np.float32),
            self.right_target_pos,
            self.right_target_quat,
            np.array([self.right_target_gripper], dtype=np.float32)
        ])

        return np.concatenate([current_state, target_state]).astype(np.float32)

    def _sample_target(self):
        anchor = self.data.body('lift_carriage_link').xpos.copy()

        self.left_target_pos = np.array(
            [
                anchor[0] + np.random.uniform(0.15, 0.35),
                np.random.uniform(0.02, 0.25),
                anchor[2] + np.random.uniform(-0.15, 0.20),
            ],
            dtype=np.float32,
        )
        self.left_target_quat = self._random_quat_wxyz()
        self.left_target_gripper = float(
            np.random.uniform(
                self.model.jnt_range[self.model.joint(self.left_arm_joints[5]).id, 0],
                self.model.jnt_range[self.model.joint(self.left_arm_joints[5]).id, 1],
            )
        )

        self.right_target_pos = np.array(
            [
                anchor[0] + np.random.uniform(0.15, 0.35),
                np.random.uniform(-0.25, -0.02),
                anchor[2] + np.random.uniform(-0.15, 0.20),
            ],
            dtype=np.float32,
        )
        self.right_target_quat = self._random_quat_wxyz()
        self.right_target_gripper = float(
            np.random.uniform(
                self.model.jnt_range[self.model.joint(self.right_arm_joints[5]).id, 0],
                self.model.jnt_range[self.model.joint(self.right_arm_joints[5]).id, 1],
            )
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
            joint_indices = [self.model.joint(jname).qposaddr[0] for jname in self.total_joints]
            init_qpos = np.random.uniform(-0.2, 0.2, size=len(joint_indices))
            self.data.qpos[joint_indices] = np.clip(init_qpos, self.min_limits, self.max_limits)

        if left_target_pos is not None or left_target_quat is not None or left_target_gripper is not None or right_target_pos is not None or right_target_quat is not None or right_target_gripper is not None:
            self._set_target(
                left_target_pos,
                left_target_quat,
                left_target_gripper,
                right_target_pos,
                right_target_quat,
                right_target_gripper
            )
        else:
            self._sample_target()

        try:
            left_target_mocap_id = self.model.body('left_target').mocapid[0]
            self.data.mocap_pos[left_target_mocap_id] = self.left_target_pos
            self.data.mocap_quat[left_target_mocap_id] = self.left_target_quat
            right_target_mocap_id = self.model.body('right_target').mocapid[0]
            self.data.mocap_pos[right_target_mocap_id] = self.right_target_pos
            self.data.mocap_quat[right_target_mocap_id] = self.right_target_quat
        except Exception:
            pass

        mujoco.mj_forward(self.model, self.data)

        obs = self._get_obs()
        info = {}
        return obs, info

    def step(self, action):
        scaled_action = np.clip(action, -1.0, 1.0) * 0.05
        current_qpos = np.concatenate([
            [self.data.qpos[self.model.joint(jname).qposadr[0]] for jname in self.left_arm_joints],
            [self.data.qpos[self.model.joint(jname).qposadr[0]] for jname in self.right_arm_joints]
        ])
        target_qpos = current_qpos + scaled_action

        self.data.ctrl[self.actuator_ids] = np.clip(target_qpos, self.min_limits, self.max_limits)

        for _ in range(self.frame_skip):
            mujoco.mj_step(self.model, self.data)

        obs = self._get_obs()

        # ----------------------------------------------------
        # REWARD FUNCTION
        # ----------------------------------------------------
        left_ee_pos = self.data.body('left_jaw').xpos
        left_ee_quat = self.data.body('left_jaw').xquat
        left_current_gripper = self.data.qpos[self.model.joint('left_gripper').qposadr[0]]

        right_ee_pos = self.data.body('right_jaw').xpos
        right_ee_quat = self.data.body('right_jaw').xquat
        right_current_gripper = self.data.qpos[self.model.joint('right_gripper').qposadr[0]]

        left_error_pos = np.linalg.norm(left_ee_pos - self.left_target_pos)
        right_error_pos = np.linalg.norm(right_ee_pos - self.right_target_pos)

        left_quat_dot = np.abs(np.dot(left_ee_quat, self.left_target_quat))
        left_error_quat = 1.0 - np.clip(left_quat_dot, 0.0, 1.0)
        right_quat_dot = np.abs(np.dot(right_ee_quat, self.right_target_quat))
        right_error_quat = 1.0 - np.clip(right_quat_dot, 0.0, 1.0)

        left_error_gripper = np.abs(left_current_gripper - self.left_target_gripper)
        right_error_gripper = np.abs(right_current_gripper - self.right_target_gripper)

        collisions = self._get_arm_collisions()
        penetration = sum(max(0.0, -dist) for _, _, dist in collisions)

        reward_pos = -2.0 * left_error_pos + -2.0 * right_error_pos
        reward_quat = -4.0 * left_error_quat + -4.0 * right_error_quat
        reward_gripper = -1.0 * left_error_gripper + -1.0 * right_error_gripper
        reward_collision = -10.0 * len(collisions) - 100.0 * penetration

        success = (left_error_pos < 0.01) and (left_error_quat < 0.01) and (left_error_gripper < 0.01) and (right_error_pos < 0.01) and (right_error_quat < 0.01) and (right_error_gripper < 0.01)
        reward_success = 100.0 if success else 0.0
        reward = reward_pos + reward_quat + reward_gripper + reward_collision + reward_success

        # ----------------------------------------------------
        # TERMINATION CONDITIONS
        # ----------------------------------------------------
        terminated = bool(success)
        truncated = False

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