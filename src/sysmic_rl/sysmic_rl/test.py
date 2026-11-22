#!/usr/bin/env python3

from ament_index_python import get_package_share_directory
from gymnasium.wrappers import TimeLimit
import mujoco.viewer
import os
from stable_baselines3 import PPO, SAC
from sysmic_rl.env import SysmicV2Env
import time

def test(mjcf_file, model_type, episodes):
    env = SysmicV2Env(mjcf_file=mjcf_file)
    env = TimeLimit(env, max_episode_steps=300)

    model_path = os.path.join(
        get_package_share_directory('sysmic_rl'),
        'models',
        f'sysmic_v2_{model_type}.zip'
    )
    print(f'Loading {model_type} model from {model_path}...')
    if model_type == 'PPO':
        model = PPO.load(model_path)
    elif model_type == 'SAC':
        model = SAC.load(model_path)
    else:
        raise ValueError('Unsupported model type.')

    with mujoco.viewer.launch_passive(env.unwrapped.model, env.unwrapped.data) as viewer:
        for episode in range(episodes):
            obs, info = env.reset()
            viewer.sync()

            terminated = False
            truncated = False
            episode_reward = 0
            
            print(f'\n--- Episode {episode + 1} ---')
            
            while not (terminated or truncated):
                if not viewer.is_running():
                    print('Viewer closed by user. Exiting evaluation.')
                    return

                action, _states = model.predict(obs, deterministic=True)
                obs, reward, terminated, truncated, info = env.step(action)
                episode_reward += reward

                viewer.sync()
                time.sleep(0.02)

            print(f'Success: {info.get("success", False)}')
            print(f'Reward: {episode_reward:.2f}')
            print(f'Left Position Error: {info.get("left_error_position", 0.0)}')
            print(f'Left Orientation Error: {info.get("left_error_orientation", 0.0)}')
            print(f'Left Gripper Error: {info.get("left_error_gripper", 0.0)}')
            print(f'Right Position Error: {info.get("right_error_position", 0.0)}')
            print(f'Right Orientation Error: {info.get("right_error_orientation", 0.0)}')
            print(f'Right Gripper Error: {info.get("right_error_gripper", 0.0)}')
            print(f'Collisions: {info.get("collisions", 0.0)}')
            print(f'Penetration: {info.get("penetration", 0.0)}')

    env.close()

def main():
    model_type = 'PPO'
    mjcf_file = os.path.join(get_package_share_directory('sysmic_sim'), 'mjcf', 'sysmic_v2_scene.xml')
    episodes = 10

    test(mjcf_file=mjcf_file, model_type=model_type, episodes=episodes)

if __name__ == '__main__':
    main()