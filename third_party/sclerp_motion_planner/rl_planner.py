import torch
import kinlib_py
import torch.nn as nn
import torch.nn.functional as F
import numpy as np
import gymnasium as gym
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from stable_baselines3.common.env_checker import check_env
from stable_baselines3.common.torch_layers import BaseFeaturesExtractor
from stable_baselines3 import PPO
from gymnasium.spaces import Box, Discrete
from sclerp_py import make_rclcpp_node, ScLERPInterface

import os
import multiprocessing
os.environ["OMP_NUM_THREADS"] = "8"
print("OMP_NUM_THREADS =", os.environ.get("OMP_NUM_THREADS"))
print("CPU cores:", multiprocessing.cpu_count())


# -------------------------------------------------------------
# Motion mode setup
# -------------------------------------------------------------

MODE_NAMES = ['r3', 'so3', 'se3']
NUM_MODES = len(MODE_NAMES)
MODE_COLORS = {
    'r3': 'blue',                            
    'so3': 'orange',
    'se3': 'cyan'
}

# Global singletons
_shared_node = None
_shared_sclerp = None

def get_shared_node():
    global _shared_node
    if _shared_node is None:
        _shared_node = make_rclcpp_node("sclerp_rl")
    return _shared_node

def get_shared_sclerp():
    global _shared_sclerp
    if _shared_sclerp is None:
        _shared_sclerp = ScLERPInterface(
            "world", "wrist3_Link", get_shared_node(),
            "/home/byd/Videos/aubo_i10.urdf",
            [f"/home/byd/Videos/aubo/collision/link_{i}.stl" for i in range(7)]
        )
    return _shared_sclerp

# -------------------------------------------------------------
# Gym like environment
# -------------------------------------------------------------
class PourTaskEnv(gym.Env):
    metadata = {"render_modes": ["human"]}

    def __init__(self):
        super().__init__()
        obs_low = np.array([-2, -2, -2, -np.pi, -np.pi, -np.pi] * 2 + [-2, -2, -2] + [0, 0, 0], dtype=np.float32)
        obs_high = np.array([2, 2, 2, np.pi, np.pi, np.pi] * 2 + [2, 2, 2] + [1, 1, 1], dtype=np.float32)
        self.observation_space = Box(low=obs_low, high=obs_high, dtype=np.float32)
        self.action_space = Box(low=-1.0, high=1.0, shape=(1 + 6,), dtype=np.float32)
        self._dist_thresh = 0.05
        self._tip_angle = 0.7
        self.node = get_shared_node()
        self.sclerp = get_shared_sclerp()
        self.reset()

    def get_ee_pose_from_joint(self, joint_values):
        success, pose = self.sclerp.kinlib_solver_.getFK(joint_values)
        if success.value != 0:
            raise RuntimeError("FK failed")
        position = pose[:3, 3]
        rz = np.arctan2(pose[1, 0], pose[0, 0])
        ry = np.arcsin(-pose[2, 0])
        rx = np.arctan2(pose[2, 1], pose[2, 2])
        return np.concatenate([position, [rx, ry, rz]]).astype(np.float32)

    def _get_obs(self):
        stage_one_hot = np.eye(3)[self.stage].astype(np.float32)
        obs = np.concatenate([
            self.current_pose,
            self.grasp_pose,
            self.target_pos,
            stage_one_hot
        ])
        return obs.astype(np.float32)

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        self.grasp_joint = np.array([0.0, -1.57, 1.57, 0.0, 1.57, 0.0], dtype=np.float32)
        self.grasp_pose = self.get_ee_pose_from_joint(self.grasp_joint)
        self.target_pos = np.array([0.5, 0.3, 0.2], dtype=np.float32)
        self.current_joint = self.grasp_joint.copy()
        self.current_pose = self.get_ee_pose_from_joint(self.current_joint)
        self.stage = 0
        self.timestep = 0
        return self._get_obs(), {}

    def pose_to_matrix(self, pose):
        T = np.eye(4)
        T[:3, 3] = pose[:3]
        rx, ry, rz = pose[3:]
        cx, sx = np.cos(rx), np.sin(rx)
        cy, sy = np.cos(ry), np.sin(ry)
        cz, sz = np.cos(rz), np.sin(rz)

        Rz = np.array([
            [cz, -sz, 0],
            [sz,  cz, 0],
            [ 0,   0, 1]
        ])
        Ry = np.array([
            [ cy, 0, sy],
            [  0, 1,  0],
            [-sy, 0, cy]
        ])
        Rx = np.array([
            [1,  0,   0],
            [0, cx, -sx],
            [0, sx,  cx]
        ])

        T[:3, :3] = Rz @ Ry @ Rx
        return T

    def step(self, action):
        self.timestep += 1
        action = np.clip(action, -1.0, 1.0)
        mode_idx = int(np.clip(np.round(((action[0] + 1) / 2) * (NUM_MODES - 1)), 0, NUM_MODES - 1))
        mode_name = MODE_NAMES[mode_idx]
        delta = action[1:] * 0.1

        new_pose = self.current_pose.copy()
        if mode_name == 'r3':
            new_pose[:3] += delta[:3]
        elif mode_name == 'so3':
            new_pose[3:] += delta[:3]
        elif mode_name == 'se3':
            new_pose += delta[:6]

        goal_matrix = self.pose_to_matrix(new_pose)
        success, traj = self.sclerp.solve_with_collision(
            self.current_joint,
            goal_matrix,
            False,
            0,
            [],
            None
        )

        reward = -0.01
        done = False

        if success:
            if traj and len(traj) > 0:
                self.current_joint = traj[-1]
                self.current_pose = self.get_ee_pose_from_joint(self.current_joint)
                if self.stage == 0:
                    dist = np.linalg.norm(self.current_pose[:3] - self.target_pos)
                    reward -= dist
                    if dist < self._dist_thresh:
                        self.stage = 1
                elif self.stage == 1:
                    tip = abs(self.current_pose[4])
                    reward += tip
                    if tip >= self._tip_angle:
                        reward += 1.0
                        self.stage = 2
                elif self.stage == 2:
                    upright = abs(self.current_pose[4]) < 0.1
                    reward += 0.5 if upright else -abs(self.current_pose[4])
                    if upright:
                        done = True
                        print(f"✅ Goal reached at step {self.timestep}")
            else:
                reward -= 1.0
                print(f"Warning: Planning reported success but returned empty trajectory at step {self.timestep}")
        else:
            reward -= 1.0
            print(f"Warning: Planning failed at step {self.timestep}, action discarded.")

        if self.timestep >= 200:
            done = True

        print(f"Step {self.timestep} - Mode: {mode_name}, ∆: {delta}, Planner: {'OK' if success and traj and len(traj) > 0 else 'FAIL'}")
        print(f"Step {self.timestep} | Stage: {self.stage} | Distance: {np.linalg.norm(self.current_pose[:3] - self.target_pos):.4f}")

        obs = self._get_obs()
        info = {'stage': self.stage}
        return obs, reward, done, False, info

    def render(self):
        print(f"Pose: {self.current_pose}, stage: {self.stage}")

    def close(self):
        pass

# -------------------------------------------------------------
# Custom SB3 Feature Extractor
# -------------------------------------------------------------
class HybridExtractor(BaseFeaturesExtractor):
    def __init__(self, observation_space, features_dim=128):
        super().__init__(observation_space, features_dim)
        self.net = nn.Sequential(
            nn.Linear(observation_space.shape[0], 128),
            nn.ReLU(),
            nn.Linear(128, features_dim),
            nn.ReLU()
        )

    def forward(self, obs):
        return self.net(obs)

# -------------------------------------------------------------
# Training Loop
# -------------------------------------------------------------
if __name__ == "__main__":
    env = PourTaskEnv()
    # check_env(env, warn=True)

    from stable_baselines3.common.env_util import make_vec_env
    from stable_baselines3.common.policies import ActorCriticPolicy

    class CustomPolicy(ActorCriticPolicy):
        def __init__(self, *args, **kwargs):
            super().__init__(
                *args,
                features_extractor_class=HybridExtractor,
                features_extractor_kwargs=dict(features_dim=128),
                **kwargs
            )

    vec_env = make_vec_env(PourTaskEnv, n_envs=1)
    # model = PPO(CustomPolicy, vec_env, verbose=1, n_steps=2048)
    # model.learn(total_timesteps=10000)
    # model.save("ppo_pour_task")

    from stable_baselines3 import PPO

    eval_env = PourTaskEnv()
    model = PPO.load("ppo_pour_task", env=eval_env)

    obs, _ = eval_env.reset()
    done = False
    trajectory = [eval_env.current_pose.copy()]

    while not done:
        action, _ = model.predict(obs, deterministic=True)
        obs, reward, done, truncated, info = eval_env.step(action)
        eval_env.render()
        trajectory.append(eval_env.current_pose.copy())

    trajectory = np.array(trajectory)



    # fig = plt.figure(figsize=(6, 5))
    # ax = fig.add_subplot(111, projection='3d')
    # ax.plot(trajectory[:, 0], trajectory[:, 1], trajectory[:, 2], marker='o')
    # ax.set_title('Evaluated Trajectory')
    # ax.set_xlabel('X')
    # ax.set_ylabel('Y')
    # ax.set_zlabel('Z')

    from scipy.spatial.transform import Rotation as R
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D
    import numpy as np

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Plot trajectory path
    ax.plot(trajectory[:, 0], trajectory[:, 1], trajectory[:, 2], marker='o')

    # Compute dynamic arrow length
    extent = np.ptp(trajectory[:, :3], axis=0)  # range along each axis
    arrow_length = 0.05 * np.linalg.norm(extent)  # 5% of trajectory size

    # Plot orientation arrows
    for pose in trajectory[::1]:
        pos = pose[:3]
        rot = R.from_euler('xyz', pose[3:]).as_matrix()
        for i, color in zip(range(3), ['r', 'g', 'b']):
            direction = rot[:, i] * arrow_length
            ax.quiver(
                pos[0], pos[1], pos[2],
                direction[0], direction[1], direction[2],
                color=color, arrow_length_ratio=0.3, linewidth=0.8, normalize=False
            )

    # Auto-scale view
    ax.set_box_aspect([1, 1, 1])  # Equal aspect ratio
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    plt.tight_layout()
    plt.show()
