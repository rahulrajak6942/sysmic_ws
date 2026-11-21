#!/usr/bin/env python3

from ament_index_python import get_package_share_directory
from gymnasium.wrappers import TimeLimit
import os
from stable_baselines3 import PPO, SAC
from stable_baselines3.common.callbacks import CheckpointCallback
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import SubprocVecEnv
from sysmic_rl.env import SysmicV2Env
import torch

def make_env(mjcf_file: str):
    def initialize_env():
        env = SysmicV2Env(mjcf_file=mjcf_file)
        env = TimeLimit(env, max_episode_steps=300)
        env = Monitor(env)
        return env
    return initialize_env

def main():
    model_type = 'PPO'
    mjcf_file = os.path.join(get_package_share_directory('sysmic_sim'), 'mjcf', 'sysmic_v2_scene.xml')
    time_steps = 2_000_000

    env = SubprocVecEnv([make_env(mjcf_file=mjcf_file) for _ in range(8)])

    if model_type == 'PPO':
        # ---------------------------------------------------------
        # Option A: Train with PPO (Proximal Policy Optimization)
        # ---------------------------------------------------------
        if (torch.cuda.is_available()):
            model = PPO(
                'MlpPolicy',
                env,
                verbose=1,
                learning_rate=3e-4,
                n_steps=2048,
                batch_size=64,
                tensorboard_log=f'log/{model_type}',
                device='cuda'
            )
        else:
            model = PPO(
                'MlpPolicy',
                env,
                verbose=1,
                learning_rate=3e-4,
                n_steps=2048,
                batch_size=64,
                tensorboard_log=f'log/{model_type}',
                device='cpu'
            )

        checkpoint_callback = CheckpointCallback(
            save_freq=80_000,
            save_path=f'log/{model_type}/checkpoints'
        )

    elif model_type == 'SAC':
        # ---------------------------------------------------------
        # Option B: Train with SAC (Soft Actor-Critic)
        # ---------------------------------------------------------
        if (torch.cuda.is_available()):
            model = SAC(
                'MlpPolicy',
                env,
                verbose=1,
                learning_rate=3e-4,
                buffer_size=800_000,
                learning_starts=8000,
                batch_size=256,
                tensorboard_log=f'log/{model_type}',
                device='cuda'
            )
        else:
            model = SAC(
                'MlpPolicy',
                env,
                verbose=1,
                learning_rate=3e-4,
                buffer_size=800_000,
                learning_starts=8000,
                batch_size=256,
                tensorboard_log=f'log/{model_type}',
                device='cpu'
            )

        checkpoint_callback = CheckpointCallback(
            save_freq=10_000,
            save_path=f'log/{model_type}/checkpoints'
        )

    model.learn(
        total_timesteps=time_steps,
        callback=checkpoint_callback,
        progress_bar=True
    )
    model.save(f'sysmic_v2_{model_type}')
    print(f'sysmic_v2_{model_type} model saved.')

if __name__ == '__main__':
    main()