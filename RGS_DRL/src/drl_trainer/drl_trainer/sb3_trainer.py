from stable_baselines3.common.callbacks import CheckpointCallback
from stable_baselines3 import PPO, SAC, TD3
from .gazebo_env import GazeboEnv
from .extractor import Extractor
from rclpy.node import Node
import gymnasium as gym
import rclpy

class TrainingNode(Node):
    def __init__(self):
        super().__init__("training_node")
        self.get_logger().info("Training Node Started")

        # 注册 Gym 环境
        gym.register("GazeboSimple-v0", entry_point="trainer.gazebo_env:GazeboEnv")

        self.env = GazeboEnv()

        # 回调：每 10,000 步保存一次模型
        checkpoint_callback = CheckpointCallback(
            save_freq=10000,           # 每隔多少步保存一次
            save_path="./checkpoints/",  # 保存路径
            name_prefix="model"        # 文件名前缀，例如 model_10000_steps
        )

        self.get_logger().info("[TrainingNode] initialize trainer...")
        policy_kwargs = dict(
            features_extractor_class=Extractor,
            features_extractor_kwargs=dict(features_dim=256),
            normalize_images=False,
        )

        # self.model = PPO(
        #     policy="MultiInputPolicy",
        #     env=self.env,
        #     policy_kwargs=policy_kwargs,
        #     verbose=1,
        #     learning_rate=3e-4,
        #     n_steps=3000,
        #     batch_size=128,
        #     n_epochs=20,
        #     device="cuda",
        #     tensorboard_log="./tensorboard/"
        # )
        
        self.model = SAC(
            policy="MultiInputPolicy",
            env=self.env,
            policy_kwargs=policy_kwargs,
            verbose=1,
            learning_rate=3e-4,
            device="cuda",
            buffer_size=100000,
            batch_size=128,
            train_freq=2,
            tensorboard_log="./tensorboard/"
        )

        # action_space = self.env.action_space
        # print("动作空间:", action_space)

        # self.model = PPO.load(
        #     path="./checkpoints/model_310000_steps",
        #     env=self.env,
        #     device="cuda",
        # )

        # self.model = SAC.load(
        #     path="./checkpoints/SAC-490kSteps",
        #     env=self.env,
        #     device="cuda",
        # )

        self.get_logger().info("[TrainingNode] Starting training...")
        self.model.learn(
            total_timesteps=1000000,
            callback=checkpoint_callback
        )
        self.get_logger().info("[TrainingNode] Training end...")
        self.model.save("simple")
        self.get_logger().info("Training complete and model saved.")

def main(args=None):
    rclpy.init()
    node = TrainingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()