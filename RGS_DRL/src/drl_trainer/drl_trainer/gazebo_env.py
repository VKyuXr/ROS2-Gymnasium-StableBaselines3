from drl_trainer.trainer_bridge import TrainerBridge
from gymnasium import spaces
from typing import Optional
import numpy as np
import gymnasium
import rclpy
import time

class GazeboEnv(gymnasium.Env):
    metadata = {"render_modes": ["human"]}

    def __init__(self, render_mode: Optional[str] = None, max_episode_steps: int = 2048):
        super().__init__()

        self.max_episode_steps = max_episode_steps

        # 延迟初始化 ROS，避免进程冲突
        self.bridge_initialized = False
        self._init_bridge()

        # 定义 observation space
        self.observation_space = spaces.Dict({
            "depth_image": spaces.Box(
                low=0,
                high=1,
                shape=(1, 16, 180),
                dtype=np.float64
            ),
            "goal": spaces.Box(
                low=-np.inf,
                high=np.inf,
                shape=(2,),
                dtype=np.float64
            ),
        })
        # 定义 action space
        self.action_space = spaces.Box(
            low=np.array([0.0, -10], dtype=np.float64),
            high=np.array([10.0, 10], dtype=np.float64),
            shape=(2,),
            dtype=np.float64
        )

    def _init_bridge(self):
        if not self.bridge_initialized:
            if not rclpy.ok():
                rclpy.init()
            self.agent = TrainerBridge()
            self.bridge_initialized = True

    def reset(self, seed: Optional[int] = None, options: Optional[dict] = None):
        super().reset(seed=seed)
        obs = self.agent.reset()
        self.steps_in_episode = 0  # 重置计数器
        return obs, {}

    def step(self, action):
        time.sleep(0.01)

        obs, reward, done, info = self.agent.step(action)
        self.steps_in_episode += 1

        # 如果达到最大步数，则 done=True 并 reset()
        if self.steps_in_episode >= self.max_episode_steps:
            done = True
            info["TimeLimit.truncated"] = True

        truncated = False
        return obs, reward, done, truncated, info

    def close(self):
        if self.bridge_initialized:
            self.agent.destroy_node()
            rclpy.shutdown()
            self.bridge_initialized = False