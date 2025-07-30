from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import PointCloud2, Image
from rclpy.node import Node
from cv_bridge import CvBridge
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import rclpy
import yaml
import cv2
import os


class LaserProcessor(Node):
    """
    一个用于处理激光扫描数据并将其转换为深度图像 ROS2 Node
    该 Node 订阅 /laser/scan topic 的 PointCloud2 数据，处理点云数据
    生成 depth image，并将 Image 消息发布到 /depth_image topic
    """
    def __init__(self):
        super().__init__("laser_processor")

        self.get_logger().info("[laser_processor:init] 正在初始化")
        self.get_logger().info("[laser_processor:init] 正在加载配置文件")

        # 获取配置文件的目录
        config_dir = os.path.join(get_package_share_directory("drl_trainer"), 'config')
        config_file_path = os.path.join(config_dir, "laser_processor.yaml")

        # 检查配置文件是否存
        if not os.path.exists(config_file_path):
            self.get_logger().error(f"[laser_processor:init] 配置文件没有找到：{config_file_path}")
            raise FileNotFoundError(f"[laser_processor:init] 配置文件没有找到：{config_file_path}")
        
        # 读取配置文件
        with open(config_file_path, 'r') as file:
            config: yaml = yaml.safe_load(file)
            if not config:
                self.get_logger().error("[laser_processor:init] 配置文件为空或者非法定义")
                raise ValueError("[laser_processor:init] 配置文件为空或者非法定义")
        self.get_logger().info("[laser_processor:init] 配置文件加载成功")

        # 从配置中获取参数
        self.radian_horizontal = np.pi * config.get("laser").get("degree_horizontal", 180) / 180
        self.radian_vertical = np.pi * config.get("laser").get("degree_vertical", 30) / 180
        self.depthImage_width = config.get("laser").get("depthImage_width", 180)
        self.depthImage_height = config.get("laser").get("depthImage_height", 16)
        self.distance_min = config.get("laser").get("distance_min", 0.15)
        self.distance_max = config.get("laser").get("distance_max", 10.0)

        self.visualization = config.get("visualization").get("enable", True)
        self.window_name = config.get("visualization").get("window_name", "Depth Image")
        self.scale_x = config.get("visualization").get("scale_x", 5)
        self.scale_y = config.get("visualization").get("scale_y", 10)

        # 设置节点参数
        self.declare_parameter('visualization', self.visualization)
        self.visualization = self.get_parameter('visualization').get_parameter_value().bool_value

        # 初始化可视化窗口
        self.bridge = CvBridge()
        cv2.namedWindow(self.window_name, cv2.WINDOW_AUTOSIZE)

        # 创建订阅者
        self.laserScan_sub_ = self.create_subscription(
            PointCloud2,
            "/gazebo_ros_lidar_controller/out",
            self.laserScan_processor,
            10,
        )

        # 创建发布者
        self.depthImage_pub_ = self.create_publisher(
            Image,
            "/agent/state",
            10,
        )

        self.get_logger().info("[laser_processor:init] 初始化成功")

    def laserScan_processor(self, msg):
        """
        处理 PointCloud2 消息，将其转换为深度图像并发布
        """
        # 初始化深度图像（使用最大值填充）
        depth_array = np.full((self.depthImage_height, self.depthImage_width), 
                            self.distance_max, 
                            dtype=np.float32)
        
        # 批量处理点云
        points = np.array(list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)))
        if len(points) == 0:
            return

        # 提取坐标并计算距离
        x = points["x"]
        y = points["y"]
        z = points["z"]
        distances = np.sqrt(x**2 + y**2 + z**2)
        
        # 距离过滤
        valid_mask = (distances >= self.distance_min) & (distances <= self.distance_max)
        x, y, z, distances = x[valid_mask], y[valid_mask], z[valid_mask], distances[valid_mask]
        
        # 计算角度坐标
        azimuth = -np.arctan2(y, x)
        elevation = -np.arctan2(z, np.sqrt(x**2 + y**2))
        
        # 修改坐标计算部分
        u = (azimuth + self.radian_horizontal/2) * (self.depthImage_width / self.radian_horizontal)
        v = (elevation + self.radian_vertical/2) * (self.depthImage_height / self.radian_vertical)

        # 箝位到有效范围
        u = np.clip(u, 0, self.depthImage_width - 1).astype(np.int32)
        v = np.clip(v, 0, self.depthImage_height - 1).astype(np.int32)

        # 创建临时数组存储最小距离
        temp_array = np.full_like(depth_array, np.inf)
        np.minimum.at(temp_array, (v, u), distances)  # 原子最小化操作
        
        # 仅更新比当前值小的距离
        update_mask = temp_array < depth_array
        depth_array[update_mask] = temp_array[update_mask]
        
        # 创建8UC1图像
        depth_image = np.zeros((self.depthImage_height, self.depthImage_width), dtype=np.uint8)
        
        # 有效区域掩码
        valid_mask = (depth_array >= self.distance_min) & (depth_array <= self.distance_max)
        
        if np.any(valid_mask):
            # 归一化并转换为0-255
            normalized = (depth_array[valid_mask] - self.distance_min) 
            normalized /= (self.distance_max - self.distance_min)
            depth_image[valid_mask] = (normalized * 255).astype(np.uint8)

        # 构建 Image 消息
        image_msg = Image()
        image_msg.header = msg.header
        image_msg.height = self.depthImage_height
        image_msg.width = self.depthImage_width
        image_msg.encoding = "8UC1"
        image_msg.step = self.depthImage_width
        image_msg.data = depth_image.flatten().tolist()

        # 发布图像
        self.depthImage_pub_.publish(image_msg)

        # 显示图像
        try:
            # 放大图像以便查看
            resized_image = cv2.resize(
                depth_image, 
                (self.depthImage_width * self.scale_x, self.depthImage_height * self.scale_y), 
                interpolation=cv2.INTER_NEAREST
            )
            
            # 应用伪彩色映射
            colored_image = cv2.applyColorMap(resized_image, cv2.COLORMAP_WINTER)
            cv2.imshow(self.window_name, colored_image)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"[laser_processor:init] 可视化图像失败：{e}")

def main(args=None):
    rclpy.init(args=args)
    node = LaserProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()