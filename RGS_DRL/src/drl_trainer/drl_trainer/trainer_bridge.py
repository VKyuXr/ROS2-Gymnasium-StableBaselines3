from sensor_msgs.msg import Image
from gazebo_msgs.srv import DeleteEntity, SpawnEntity, SetEntityState
from gazebo_msgs.msg import ContactsState
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from dataclasses import dataclass
from rclpy.clock import Clock
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from cv_bridge import CvBridge
from threading import Thread
from typing import Tuple, Optional
import numpy as np
import threading
import random
import rclpy
import copy
import time
import cv2

def xy2rtheta(x, y):
    r = np.sqrt(x**2 + y**2)
    theta = np.arcsin(y/r)
    return r, theta

def rtheta2xy(r, theta):
    return r * np.cos(theta), r * np.sin(theta)

class Target_Manager:
    def __init__(self, trainer):
        self.spawn_entity_client = trainer.spawn_entity_client
        self.delete_entity_client = trainer.delete_entity_client

    def create_target(self):
        r = random.uniform(9, 10)
        theta = random.uniform(-np.pi, np.pi)
        z = 1
        pose = [r * np.cos(theta), r * np.sin(theta), z, 0, 0, 0]

        sdf = f"""
<?xml version="1.0"?>
<sdf version="1.6">
    <model name="target">
        <static>true</static>
        <pose>{pose[0]} {pose[1]} {pose[2]} {pose[3]} {pose[4]} {pose[5]}</pose>
        <link name="sphere_link">
            <visual name="visual">
                <geometry><cylinder><radius>0.2</radius><length>2</length></cylinder></geometry>
                <material><diffuse>1.0 1.0 0.0 0.5</diffuse></material>
            </visual>
        </link>
    </model>
</sdf>
"""
        req = SpawnEntity.Request()
        req.name = "target"
        req.xml = sdf.replace('\n', '').replace('  ', ' ')
        self.spawn_entity_client.call_async(req)

        # 经测试，服务不会有 response，所以不需要等待

        return np.array([r * np.cos(theta), r * np.sin(theta)])

    def delete_target(self):
        req = DeleteEntity.Request()
        req.name = "target"
        self.delete_entity_client.call_async(req)

        # 经测试，服务不会有 response，所以不需要等待

    def get_target_position(self):
        return np.array([self.x, self.y, self.z])

class Obstacle_Manager:
    obstacles = []

    def __init__(self, trainer, num_obstacles: int = 10):
        self.spawn_entity_client = trainer.spawn_entity_client
        self.delete_entity_client = trainer.delete_entity_client
        self.num_obstacles = num_obstacles

    def create_obstacle(self):
        def sdf_box(num: int, size: list = None, pose: list = None):
            r = random.uniform(3, 7)
            theta = random.uniform(-np.pi, np.pi)

            size = [random.uniform(0.1, 2.0), random.uniform(0.1, 2.0), 1.0]
            pose = [r * np.cos(theta), r * np.sin(theta), size[2] / 2, 0, 0, random.uniform(-np.pi, np.pi)]

            sdf = f"""
<?xml version="1.0"?>
<sdf version="1.6">
    <model name="obstacle_{num}">
        <static>true</static>
        <pose>{pose[0]} {pose[1]} {pose[2]} {pose[3]} {pose[4]} {pose[5]}</pose>
        <link name="box_link">
            <visual name="visual"><geometry><box><size>{size[0]} {size[1]} {size[2]}</size></box></geometry></visual>
            <collision name="collision"><geometry><box><size>{size[0]} {size[1]} {size[2]}</size></box></geometry></collision>
        </link>
    </model>
</sdf>
"""
            return sdf

        def sdf_sphere(num: int, radius: float = None, pose: list = None):
            r = random.uniform(3, 7)
            theta = random.uniform(-np.pi, np.pi)

            radius = random.uniform(0.1, 1)
            pose = [r * np.cos(theta), r * np.sin(theta), radius, 0, 0, 0]

            sdf = f"""
<?xml version="1.0"?>
<sdf version="1.6">
    <model name="obstacle_{num}">
        <static>true</static>
        <pose>{pose[0]} {pose[1]} {pose[2]} {pose[3]} {pose[4]} {pose[5]}</pose>
        <link name="sphere_link">
            <visual name="visual"><geometry><sphere><radius>{radius}</radius></sphere></geometry></visual>
            <collision name="collision"><geometry><sphere><radius>{radius}</radius></sphere></geometry></collision>
        </link>
    </model>
</sdf>
"""
            return sdf

        def sdf_cylinder(num: int, radius: float = None, height: float = None, pose: list = None):
            r = random.uniform(3, 7)
            theta = random.uniform(-np.pi, np.pi)

            radius = random.uniform(0.1, 1)
            height = random.uniform(0.1, 2.0)
            pose = [r * np.cos(theta), r * np.sin(theta), height / 2, 0, 0, random.uniform(-np.pi, np.pi)]

            sdf = f"""
<?xml version="1.0"?>
<sdf version="1.6">
    <model name="obstacle_{num}">
        <static>true</static>
        <pose>{pose[0]} {pose[1]} {pose[2]} {pose[3]} {pose[4]} {pose[5]}</pose>
        <link name="cylinder_link">
            <visual name="visual"><geometry><cylinder><radius>{radius}</radius><length>{height}</length></cylinder></geometry></visual>
            <collision name="collision"><geometry><cylinder><radius>{radius}</radius><length>{height}</length></cylinder></geometry></collision>
        </link>
    </model>
</sdf>
"""
            return sdf
        
        create_obstacles = []
        for i in range(0, self.num_obstacles):
            # self.get_logger().info(f"正在创建 obstacle_{i}")
            obstacle = random.choice(["box", "sphere", "cylinder"])
            if obstacle == "box":
                sdf = sdf_box(i)
            elif obstacle == "sphere":
                sdf = sdf_sphere(i)
            else:
                sdf = sdf_cylinder(i)
            
            # 获取当前时间戳
            timestamp = str(time.time_ns())

            req = SpawnEntity.Request()
            req.name = timestamp
            create_obstacles.append(timestamp)
            req.xml = sdf.replace('\n', '').replace('  ', ' ')
            self.spawn_entity_client.call_async(req)
            time.sleep(0.01)

            # 经测试，服务不会有 response，所以不需要等待
        self.obstacles = create_obstacles

    def delete_obstacle(self):
        delete_obstacles = self.obstacles

        for obstacle in delete_obstacles:
            # self.get_logger().info(f"正在删除 obstacle_{i}")
            # self.call_delete_entity(f"obstacle_{i}")
            req = DeleteEntity.Request()
            req.name = obstacle
            self.delete_entity_client.call_async(req)
            time.sleep(0.01)

        # 经测试，服务不会有 response，所以不需要等待

@dataclass
class StepData:
    state:          Optional[Tuple[np.ndarray, ...]]    = None          # 环境状态
    action:         Optional[Tuple[np.ndarray, ...]]    = None          # 输出动作
    goal:           Optional[Tuple[np.ndarray, ...]]    = None          # 目标

    position:       np.ndarray                = np.array([0, 0, 0], dtype=np.float32)          # 位置
    orientation:    np.ndarray                = np.array([0, 0, 0, 0], dtype=np.float32)          # 朝向
    linear:         np.ndarray                = np.array([0, 0, 0], dtype=np.float32)          # 线速度
    angular:        np.ndarray                = np.array([0, 0, 0], dtype=np.float32)          # 角速度

    reward:         float                       = 0.0

class Storage:
    done:           bool                        = None          # 完成目标

    goal_position:  np.ndarray                  = None          # 目标位置：(B, N)
    collision:      bool                        = None          # 碰撞标志位
    
    last:           StepData                    = StepData()    # 上一 step 状态
    current:        StepData                    = StepData()    # 当前 step 状态
    step:           int                         = 0             # step 计数器
    step_episode:   int                         = 0             # step 在当前 episode 中的计数器
    episode:        int                         = 0             # episode 计数器
    episode_reward: float                       = 0.0           # 当前 episode 的 reward

    def __init__(self, logger):
        self.logger = logger

    def new_step(self):
        self.last = copy.deepcopy(self.current)
        self.step += 1
        self.step_episode += 1

    def new_episode(self):
        self.done = False
        self.collision = False
        self.last = StepData()
        self.current = StepData()
        self.step_episode = 0
        self.episode += 1
        self.episode_reward = 0.0

class TrainerBridge(Node):
    def __init__(self):
        super().__init__("trainer_bridge")

        qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._init_input(qos_profile)
        self._init_output(qos_profile)

        self.storage = Storage(self.get_logger())
        # 图像处理工具
        self.bridge = CvBridge()

        self.target_manager = Target_Manager(self)
        self.obstacle_manager = Obstacle_Manager(self)

        self.node = rclpy.create_node("gazebo_env_client")

        # 启动后台线程执行 spin
        self.spin_thread = Thread(target=rclpy.spin, args=(self,), daemon=True)
        self.spin_thread.start()


        self.first_state_event = threading.Event()

        self.last_step_time = None

    def update_obs_for_gym(self, obs):
        if hasattr(self, "gym_env"):
            self.gym_env.update_observation(obs)

    def _init_input(self, qos_profile: QoSProfile):
        state_topic = "/agent/state"
        odometry_topic = "/odom"
        collision_topic = "/bumper_states"

        # 订阅状态输入
        self.state_subscription = self.create_subscription(
            Image,
            state_topic,
            self.image_handler,
            qos_profile=qos_profile
        )

        # 订阅机器人位姿
        self.odometry_subscription = self.create_subscription(
            Odometry,
            odometry_topic,
            self.odometry_handler,
            qos_profile=qos_profile
        )

        # 订阅碰撞检测
        self.collision_sub = self.create_subscription(
            ContactsState,
            collision_topic,
            self.collision_handler,
            qos_profile=qos_profile
        )

    def _init_output(self, qos_profile: QoSProfile):
        # 发布控制动作
        action_topic = "/agent/action"
        reward_topic = "/agent/reward"
        spawn_entity_service = "/spawn_entity"
        delete_entity_service = "/delete_entity"
        set_entity_state_service = "/set_entity_state"

        self.action_pub = self.create_publisher(
            Float32MultiArray,
            action_topic,
            qos_profile=qos_profile
        )

        self.reward_pub = self.create_publisher(
            Float32MultiArray,
            reward_topic,
            qos_profile=qos_profile
        )
        
        self.spawn_entity_client = self.create_client(SpawnEntity, spawn_entity_service)
        self.delete_entity_client = self.create_client(DeleteEntity, delete_entity_service)
        self.set_entity_state_client = self.create_client(SetEntityState, set_entity_state_service)

    def image_handler(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="8UC1")
        except Exception as e:
            self.get_logger().error(f"转换图像数据失败：{e}")
            return

        # 归一化为 [0, 1] 范围的 float32 numpy array，形状为 (H, W)
        normalized = cv2.normalize(cv_image, None, 0, 1, cv2.NORM_MINMAX, dtype=cv2.CV_32F)

        normalized = normalized[np.newaxis, :, :]

        # 现在直接使用 np.ndarray
        if self.storage.step_episode == 0 or self.storage.current.state is None:
            self.storage.last.state = (normalized.copy(),)   # 存储为 ndarray
            self.storage.current.state = (normalized.copy(),)
        else:
            self.storage.last.state = (self.storage.current.state[0].copy(),)
            self.storage.current.state = (normalized.copy(),)


    def odometry_handler(self, msg):
        if self.storage.goal_position is None:
            return
        position = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ])
        orientation = np.array([
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        ])
        linear = np.array([
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z
        ])
        angular = np.array([
            msg.twist.twist.angular.x,
            msg.twist.twist.angular.y,
            msg.twist.twist.angular.z
        ])

        # 将 quaternion 转换为 yaw（弧度）
        sin_yaw = 2.0 * (orientation[3] * orientation[2] + orientation[0] * orientation[1])
        cos_yaw = 1.0 - 2.0 * (orientation[1] * orientation[1] + orientation[2] * orientation[2])
        current_yaw = np.arctan2(sin_yaw, cos_yaw)

        # 计算到目标的距离（仅平面 xy）
        distance_to_goal = np.linalg.norm(self.storage.goal_position[:2] - position[:2])

        # 计算期望的方向角（相对于正东方向）
        delta_x = self.storage.goal_position[0] - position[0]
        delta_y = self.storage.goal_position[1] - position[1]
        desired_angle = np.arctan2(delta_y, delta_x)

        # 计算 heading error（机器人朝向与目标方向之间的角度差）
        radian_to_goal = desired_angle - current_yaw
        radian_to_goal = np.arctan2(np.sin(radian_to_goal), np.cos(radian_to_goal))  # 规范化到 [-π, π]

        goal = (np.array([distance_to_goal, radian_to_goal]), )

        if self.storage.step_episode == 0:
            self.storage.last.position = copy.deepcopy(position)
            self.storage.last.orientation = copy.deepcopy(orientation)
            self.storage.last.linear = copy.deepcopy(linear)
            self.storage.last.angular = copy.deepcopy(angular)
            self.storage.last.goal = copy.deepcopy(goal)

            self.storage.current.position = copy.deepcopy(position)
            self.storage.current.orientation = copy.deepcopy(orientation)
            self.storage.current.linear = copy.deepcopy(linear)
            self.storage.current.angular = copy.deepcopy(angular)
            self.storage.current.goal = copy.deepcopy(goal)
        else:
            self.storage.last.position = copy.deepcopy(self.storage.current.position)
            self.storage.last.orientation = copy.deepcopy(self.storage.current.orientation)
            self.storage.last.linear = copy.deepcopy(self.storage.current.linear)
            self.storage.last.angular = copy.deepcopy(self.storage.current.angular)
            self.storage.last.goal = copy.deepcopy(self.storage.current.goal)

            self.storage.current.position = copy.deepcopy(position)
            self.storage.current.orientation = copy.deepcopy(orientation)
            self.storage.current.linear = copy.deepcopy(linear)
            self.storage.current.angular = copy.deepcopy(angular)
            self.storage.current.goal = copy.deepcopy(goal)


    def collision_handler(self, msg):
        if msg.states:
            self.get_logger().info("检测到碰撞")
            self.storage.collision = True
        else:
            self.storage.collision = False

    def publish_action(self, action):
        # self.get_logger().info("[TrainerBridge.publish_action]")
        action_msg = Float32MultiArray()
        action = np.array(action)

        action_msg.data = action.flatten().tolist()
        self.action_pub.publish(action_msg)

    def publish_reward(self, reward):
        # self.get_logger().info("[TrainerBridge.publish_action]")
        reward_msg = Float32MultiArray()
        reward = np.array(reward)

        # self.get_logger().info(f"{type(reward)}, {type(reward[0])}, {type(reward[1])}")
        reward_msg.data = [float(x) for x in reward.flatten().tolist()]
        self.reward_pub.publish(reward_msg)

    def step(self, action):

        self.publish_action(np.array([action[0], action[1]]))

        # self.get_logger().info(f"{self.storage.current.state}")
        if self.storage.current.state is  None:
            self.get_logger().info(f"empty state when {self.storage.step_episode} step in {self.storage.episode} episode")
            self.storage.current.state = (np.zeros((16, 180)),)

        # self.get_logger().info(f"step: {self.storage.current.goal}")
        if self.storage.current.goal is None:
            self.storage.current.goal = ((0, 0),)
        observation = {
            "depth_image": copy.deepcopy(self.storage.current.state[0]),
            "goal": copy.deepcopy(self.storage.current.goal[0]),
        }
        # self.get_logger().info(f"{self.storage.current.goal}, {self.storage.last.goal}")
        reward = self._calculate_reward()
        # self.get_logger().info(f"{reward}")
        done = self.storage.done or self.storage.collision
        info = {}

        self.storage.new_step()

        return observation, reward, done, info

    def _calculate_reward(self) -> float:
        goal_threshold = 0.2
        collision_penalty = -250
        distance_coefficient = -1

        # 碰撞惩罚
        collision_reward = 0
        if self.storage.collision:
            collision_reward = collision_penalty

        # 到达目标
        goal_reward = 0
        if self.storage.current.goal[0][0] < goal_threshold and self.storage.step_episode >= 10:
            self.get_logger().info(f"arrive goal: {self.storage.current.goal}")
            self.storage.done = True
            goal_reward = 1000

        # 距离变化奖励
        distance_reward = 0
        time_interval = 0
        distance_reward_per_secend = 0
        if self.storage.last.goal:
            current_step_time = Clock().now().nanoseconds * 1e-9
            if self.last_step_time is not None:
                time_interval = current_step_time - self.last_step_time
                distance_reward = self.storage.current.goal[0][0] * distance_coefficient
            self.last_step_time = current_step_time

        # 时间惩罚
        total_reward = (
            + distance_reward
            + goal_reward
            + collision_reward
        )
        self.publish_reward([time_interval, total_reward, distance_reward, distance_reward_per_secend])

        return float(total_reward)

    def reset(self):
        def reset_robot():
            req = SetEntityState.Request()
            req.state.name = "differential_robot"
            # req.state.name = "ackerman_robot"
            req.state.pose.position.x = float(0)
            req.state.pose.position.y = float(0)
            req.state.pose.position.z = float(0.1)

            yaw = 0
            req.state.pose.orientation.x = 0.0
            req.state.pose.orientation.y = 0.0
            req.state.pose.orientation.z = np.sin(yaw * 0.5)
            req.state.pose.orientation.w = np.cos(yaw * 0.5)

            self.get_logger().info("[TrainerBridge.reset] reset robot") 
            self.set_entity_state_client.call_async(req)

        # 1. 暂停仿真
        self.get_logger().info("[TrainerBridge.reset] pause simulation") 
        self.publish_action([0.0, 0.0])

        self.get_logger().info("[TrainerBridge.reset] delete") 
        self.target_manager.delete_target()
        self.obstacle_manager.delete_obstacle()

        time.sleep(10)

        self.get_logger().info("[TrainerBridge.reset] create") 
        self.storage.goal_position = self.target_manager.create_target()
        self.obstacle_manager.create_obstacle()

        reset_robot()

        time.sleep(10)

        # if self.storage.step % 100000 == 0:
        #     self.get_logger().info("等待电脑冷却")
        #     time.sleep(60)

        # 经测试，服务不会有 response，所以不需要等待

        if self.storage.current.state is None:
            self.storage.current.state = (np.zeros((16, 180)),)
        if self.storage.current.goal is None:
            r, theta = xy2rtheta(self.storage.goal_position[0], self.storage.goal_position[0])
            self.storage.last.goal = (np.array([r, theta]),)
            self.storage.current.goal = (np.array([r, theta]),)
        self.get_logger().info(f"goal: {self.storage.current.goal}")

        observation = {
            "depth_image": copy.deepcopy(self.storage.current.state[0]),
            "goal": copy.deepcopy(self.storage.current.goal[0]),
        }
        self.get_logger().info("[TrainerBridge.reset] unpause simulation") 

        self.storage.new_episode()

        return observation

