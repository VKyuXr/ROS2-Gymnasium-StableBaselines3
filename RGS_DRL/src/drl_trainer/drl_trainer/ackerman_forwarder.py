from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import Float32MultiArray, Float64MultiArray
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from array import array
import rclpy
import yaml
import os

class AckermanForwarder(Node):
    def __init__(self):
        super().__init__("ackerman_forwarder")

        self.get_logger().info("[ackerman_forwarder.init] 正在初始化")
        self.get_logger().info("[ackerman_forwarder.init] 正在加载配置文件")

        # 获取配置文件的目录
        config_dir = os.path.join(get_package_share_directory("drl_trainer"), 'config')
        config_file_path = os.path.join(config_dir, "forwarder.yaml")

        # 检查配置文件是否存
        if not os.path.exists(config_file_path):
            self.get_logger().error(f"[ackerman_forwarder.init] 配置文件没有找到：{config_file_path}")
            raise FileNotFoundError(f"[ackerman_forwarder.init] 配置文件没有找到：{config_file_path}")
        
        # 读取配置文件
        with open(config_file_path, 'r') as file:
            config: yaml = yaml.safe_load(file)
            if not config:
                self.get_logger().error("[ackerman_forwarder.init] 配置文件为空或者非法定义")
                raise ValueError("[ackerman_forwarder.init] 配置文件为空或者非法定义")
        self.get_logger().info("[ackerman_forwarder.init] 配置文件加载成功")

        # self.controller_name = config.get("controller", {}).get("name", "wheel_controller")
        self.freedom_degree = config.get("controller", {}).get("freedom_degree", 2)

        qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        # 订阅 /agent/action 获取模型输出
        self.control_sub = self.create_subscription(
            Float32MultiArray,
            "/agent/action",
            self.controlMatrix_callback,
            qos_profile=qos_profile,
        )
        
        # 发布到 diff drive controller 的 command 主题
        self.wheel_pub = self.create_publisher(
            Float64MultiArray, 
            "/wheel_controller/commands", 
            10
        )

        self.steering_pub = self.create_publisher(
            Float64MultiArray, 
            "/steering_controller/commands", 
            10
        )
        
        self.get_logger().info("[ackerman_forwarder.init] 初始化成功")

    def controlMatrix_callback(self, msg):
        if len(msg.data) != self.freedom_degree:
            self.get_logger().error(f"[ackerman_forwarder.controlMatrix_callback] 数据非法定义：{msg.data}")
            return
        
        v = msg.data[0]
        omega = msg.data[1]
        epsilon = 1e-6

        velocity_l = v - 0.075 * omega
        velocity_r = v + 0.075 * omega

        # alpha = np.arctan((0.15 * omega) / (v + epsilon))
        alpha = omega / 10
        # alpha_l = np.sign(omega) * abs(alpha * (velocity_l / (v + epsilon)))
        # alpha_r = np.sign(omega) * abs(alpha * (velocity_r / (v + epsilon)))

        wheel_msg = Float64MultiArray()
        wheel_msg.data = array('d', [velocity_l, velocity_r])
        self.wheel_pub.publish(wheel_msg)

        steering_msg = Float64MultiArray()
        # steering_msg.data = array('d', [alpha_l, alpha_r])
        steering_msg.data = array('d', [alpha])
        self.steering_pub.publish(steering_msg)

def main(args=None):
    rclpy.init(args=args)
    node = AckermanForwarder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()