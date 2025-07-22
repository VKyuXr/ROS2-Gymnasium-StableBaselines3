sensor是实现反馈控制的关键组件。它们提供了关于机器人状态和环境的信息，使得控制器能够做出相应的调整。
在ROS2中，传感器通常通过发布消息到特定的主题来工作。控制器可以订阅这些主题，以获取传感器数据并进行处理。
传感器可以是各种类型的，例如激光雷达、摄像头、IMU等。每种传感器都有其特定的消息类型和数据格式。

== 机器人姿态的获取
#line(length: 80%) 

首先需要在URDF文件中添加一个Gazebo插件，用于输出机器人的位姿信息。

```xml
  <!-- Gazebo 通用插件 -->
  <gazebo>
      <!-- 位姿输出 -->
      <plugin name="gazebo_ros_p3d" filename="libgazebo_ros_p3d.so">
          <!-- /odom -->
          <alwaysOn>true</alwaysOn>
          <updateRate>50.0</updateRate>
          <body_name>wheelFC_steering_link</body_name>
          <gaussianNoise>0.0</gaussianNoise>
      </plugin>
  </gazebo>
```

然后在Node中订阅对应的主题来获取位姿信息，并对其进行处理。

```python
from nav_msgs.msg import Odometry

# 订阅机器人位姿
self.odometry_subscription = self.create_subscription(
    Odometry,
    "/odom",
    self.odometry_handler,
    10
)

def odometry_handler(self, msg):
    # 数据获取
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
    # 其他处理
```

== 激光雷达的使用
#line(length: 80%) 

首先需要在URDF文件中添加一个Gazebo插件，用于模拟雷达。

```xml
<!-- 激光雷达插件 -->
<gazebo reference="laser_link">
    <sensor type="ray" name="lidar_sensor">
        <pose>0 0 0 0 0 0</pose>
        <visualize>true</visualize>
        <update_rate>40</update_rate>               <!-- 更新率 40Hz -->
        <ray>
            <scan>
                <horizontal>                        <!-- 水平方向 -->
                    <samples>180</samples>          <!-- 180 线 -->
                    <min_angle>-1.5708</min_angle>  <!-- -180 度 -->
                    <max_angle>1.5708</max_angle>   <!-- 180 度 -->
                </horizontal>
                <vertical>                          <!-- 垂直方向 -->
                    <samples>16</samples>           <!-- 16 线 -->
                    <min_angle>-0.2618</min_angle>  <!-- -15 度 -->
                    <max_angle>0.2618</max_angle>   <!-- 15 度 -->
                </vertical>
            </scan>
            <range>
                <min>0.15</min>                     <!-- 最小探测距离 0.14m -->
                <max>10.0</max>                     <!-- 最大探测距离 10m -->
                <resolution>0.01</resolution>       <!-- 分辨率 0.01m -->
            </range>
        </ray>
        <plugin name="gazebo_ros_lidar_controller" filename="libgazebo_ros_ray_sensor.so">
            <!-- /gazebo_ros_lidar_controller/out -->
            <frame_name>laser_link</frame_name>
        </plugin>
    </sensor>
</gazebo>
```

雷达的输出是点云数据，我们可以直接对其处理，或者转换为深度图。

```python
from sensor_msgs.msg import PointCloud2

# 创建订阅者
self.laserScan_subscription = self.create_subscription(
    PointCloud2,
    "/gazebo_ros_lidar_controller/out",
    self.laserScan_processor,
    10,
)
```

```python
from sensor_msgs.msg import Image

# 创建发布者
self.depthImage_publisher = self.create_publisher(
    Image,
    "/depth_image",
    10,
)

def laserScan_processor(self, msg):
    # 初始化深度图像（使用最大值填充）
    depth_array = np.full((self.depthImage_height, self.depthImage_width), 
                        self.distance_max, 
                        dtype=np.float32)
    
    # 批量处理点云
    points = np.array(list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)))
    if len(points) == 0:
        return

    # 提取坐标并计算距离
    x, y, z = points.T
    distances = np.sqrt(x**2 + y**2 + z**2)
    
    # 距离过滤
    valid_mask = (distances >= self.distance_min) & (distances <= self.distance_max)
    x, y, z, distances = x[valid_mask], y[valid_mask], z[valid_mask], distances[valid_mask]
    
    # 计算角度坐标
    azimuth = -np.arctan2(y, x)
    elevation = -np.arctan2(z, np.sqrt(x**2 + y**2))
    
    # 修改坐标计算
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
    
    # === 图像归一化 ===
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
    self.depthImage_publisher.publish(image_msg)
```

== 对碰撞进行检测
#line(length: 80%) 

首先需要在URDF文件中添加一个Gazebo插件，用于检测接触。

```xml
<!-- 碰撞检测插件 -->
<gazebo reference="body_link">
    <selfCollide>false</selfCollide>
    <sensor name="base_contact_sensor" type="contact">
        <always_on>true</always_on>
        <update_rate>100.0</update_rate>
        <contact>
            <collision>body_link_fixed_joint_lump__body_collision_collision</collision> <!-- 这个名字需要将xacro/urdf 转换为 sdf，使用 sdf 中的 collision 名字 -->
        </contact>
        <plugin filename="libgazebo_ros_bumper.so" name="base_gazebo_ros_bumper_controller">
            <!-- /bumper_states -->
        </plugin>
    </sensor>
</gazebo>
```

碰撞信息的内容较为复杂，这里只对是否发生碰撞进行检测。

```python
from gazebo_msgs.msg import ContactsState

# 订阅碰撞检测
self.collision_sub = self.create_subscription(
    ContactsState,
    "/bumper_states",
    self.collision_handler,
    10,
)

def collision_handler(self, msg):
    if msg.states:
        self.get_logger().info("检测到碰撞")
        self.collision = True
    else:
        self.collision = False
```