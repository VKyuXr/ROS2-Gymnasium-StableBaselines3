from stable_baselines3.common.torch_layers import BaseFeaturesExtractor
from gymnasium import spaces
import torch.nn as nn
import torch

class CustomCNN(BaseFeaturesExtractor):
    def __init__(self, observation_space: spaces.Box, features_dim: int = 64):
        super().__init__(observation_space, features_dim)

        self.cnn = nn.Sequential(
            nn.Conv2d(1, 8, kernel_size=3, stride=1, padding=1),
            nn.ReLU(),
            nn.MaxPool2d(2, 2),

            nn.Conv2d(8, 16, kernel_size=3, stride=1, padding=1),
            nn.ReLU(),
            nn.MaxPool2d(3, 3),

            nn.Flatten(),
        )

        # 自动计算 flatten 后的维度
        with torch.no_grad():
            dummy = torch.as_tensor(observation_space.sample()[None]).float()
            n_flatten = self.cnn(dummy).shape[1]

        self.linear = nn.Sequential(
            nn.Linear(n_flatten, features_dim),
            nn.ReLU()
        )

    def forward(self, obs: torch.Tensor) -> torch.Tensor:
        return self.linear(self.cnn(obs))


class Extractor(BaseFeaturesExtractor):
    def __init__(self, observation_space: spaces.Dict, features_dim: int):
        super().__init__(observation_space, features_dim=features_dim)

        self.cnn = CustomCNN(observation_space["depth_image"])

        # for param in self.cnn.parameters():
        #     param.requires_grad = False

        self.goal_net = nn.Sequential(
            nn.Linear(2, 16),
            nn.ReLU(),
            nn.Linear(16, 16),
            nn.ReLU()
        )

        # for param in self.goal_net.parameters():
        #     param.requires_grad = False

        combined_size = self.cnn._features_dim + 16
        self.combined_mlp = nn.Sequential(
            nn.Linear(combined_size, 256),
            nn.ReLU(),
            nn.Linear(256, features_dim),
            nn.ReLU()
        )

        # for param in self.combined_mlp.parameters():
        #     param.requires_grad = False

    def forward(self, obs) -> torch.Tensor:
        depth_out = self.cnn(obs["depth_image"])
        goal_out = self.goal_net(obs["goal"])
        combined = torch.cat([depth_out, goal_out], dim=1)

        return self.combined_mlp(combined)
