== 将ROS2封装为Gymnasium环境
#line(length: 80%) 

将ROS2封装为Gymnasium环境的核心是在ROS2的Node中实现`step()`和`reset()`方法，然后在继承自`gymnasium.Env`的类中编写对应的处理。

```python
import gymnasium
from gymnasium import spaces
import numpy as np
from trainer.trainer_bridge import TrainerBridge
import rclpy
from typing import Optional, Dict, Any
import torch
import time

class GazeboEnv(gymnasium.Env):
    metadata = {'render_modes': ['human']}

    def __init__(self, render_mode: Optional[str] = None, max_episode_steps: int = 2048):
        super().__init__()

        self.max_episode_steps = max_episode_steps

        # 延迟初始化 ROS，避免多进程冲突
        self.bridge_initialized = False
        self._init_bridge()

        # 定义 observation_space 为 Dict 类型
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

        # 如果达到最大步数，则设置 done=True 并 reset
        if self.steps_in_episode >= self.max_episode_steps:
            done = True

        truncated = False
        return obs, reward, done, truncated, info

    def close(self):
        if self.bridge_initialized:
            self.agent.destroy_node()
            rclpy.shutdown()
            self.bridge_initialized = False
```

== 使用Stable Baselines3进行训练和评估
#line(length: 80%) 

```python
import rclpy
from rclpy.node import Node
import gymnasium as gym
from stable_baselines3 import PPO, SAC, TD3
from .gazebo_env import GazeboEnv
from stable_baselines3.common.callbacks import CheckpointCallback
from .extractor import Extractor

class TrainingNode(Node):
    def __init__(self):
        super().__init__('training_node')
        self.get_logger().info('Training Node Started')

        # 注册 Gym 环境
        gym.register("GazeboSimple-v0", entry_point="trainer.gazebo_env:GazeboEnv")
        
        self.env = GazeboEnv()

        # 添加回调：每 10,000 步保存一次模型
        checkpoint_callback = CheckpointCallback(
            save_freq=10000,
            save_path='./checkpoints/',
            name_prefix='model' # 文件名前缀，例如 model_10000_steps
        )

        self.get_logger().info('[TrainingNode] initialize trainer...')
        policy_kwargs = dict(
            features_extractor_class=Extractor,
            features_extractor_kwargs=dict(features_dim=256),
            normalize_images=False,
        )

        self.model = SAC(
            policy="MultiInputPolicy",
            env=self.env,
            policy_kwargs=policy_kwargs,
            verbose=1,
            learning_rate=3e-4,
            device="cuda",
            buffer_size=100000,
            train_freq=10,
            tensorboard_log="./tensorboard/"
        )

        self.get_logger().info('[TrainingNode] Starting training...')
        self.model.learn(
            total_timesteps=1000000,
            callback=checkpoint_callback
        )
        self.get_logger().info('[TrainingNode] training end...')
        self.model.save("model_end")
        self.get_logger().info('Training complete and model saved.')

def main(args=None):
    rclpy.init()
    node = TrainingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

== 使用RLlib进行训练和评估
#line(length: 80%) 