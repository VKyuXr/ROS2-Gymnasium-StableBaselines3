from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import Float32MultiArray, Float64MultiArray
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from array import array
import rclpy
import yaml
import os

class DifferentialForwarder(Node):
    def __init__(self):
        super().__init__("differential_forwarder")

        self.get_logger().info("[differential_forwarder.init] 正在初始化")
        self.get_logger().info("[differential_forwarder.init] 正在加载配置文件")

        # 获取配置文件的目录
        config_dir = os.path.join(get_package_share_directory("drl_trainer"), 'config')
        config_file_path = os.path.join(config_dir, "forwarder.yaml")

        # 检查配置文件是否存
        if not os.path.exists(config_file_path):
            self.get_logger().error(f"[differential_forwarder.init] 配置文件没有找到：{config_file_path}")
            raise FileNotFoundError(f"[differential_forwarder.init] 配置文件没有找到：{config_file_path}")
        
        # 读取配置文件
        with open(config_file_path, 'r') as file:
            config: yaml = yaml.safe_load(file)
            if not config:
                self.get_logger().error("[differential_forwarder.init] 配置文件为空或者非法定义")
                raise ValueError("[differential_forwarder.init] 配置文件为空或者非法定义")
        self.get_logger().info("[differential_forwarder.init] 配置文件加载成功")

        self.controller_name = config.get("controller", {}).get("name", "wheel_controller")
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
        self.cmd_pub = self.create_publisher(
            Float64MultiArray, 
            "/" + self.controller_name + "/commands", 
            10
        )
        
        self.get_logger().info("[differential_forwarder.init] 初始化成功")

    def controlMatrix_callback(self, msg):
        if len(msg.data) != self.freedom_degree:
            self.get_logger().error("[differential_forwarder.controlMatrix_callback] 数据非法定义")
            return
        
        v = msg.data[0]
        omega = msg.data[1]
        wheel_countinous = [v + omega/2, v - omega/2]

        control_msg = Float64MultiArray()
        control_msg.data = array('d', [float(d) for d in wheel_countinous])
        self.cmd_pub.publish(control_msg)

def main(args=None):
    rclpy.init(args=args)
    node = DifferentialForwarder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()