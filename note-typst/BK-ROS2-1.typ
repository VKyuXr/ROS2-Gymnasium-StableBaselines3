== ROS2主要构成
#line(length: 80%)

ROS2本质上是一个分布式通信框架，主要由Node（节点）与通信机制组成。
Node是ROS2的基本执行单元，负责处理特定任务。
通信机制则包括Topic（话题）、Service（服务）和Action（动作），用于实现节点间的数据交换。

// #table(
//   columns: 2,
//   table.header([通信方式], [使用场景]),
//   [Topic], [用于发布和订阅数据流，如传感器数据、状态信息等],
//   [Service], [用于请求-响应式通信，如获取传感器状态、执行特定操作等],
//   [Action], [用于处理长时间运行的任务，如路径规划、复杂计算等]
// )

ROS2的工作是通过使用通信机制在不同的Node之间传递消息来实现的。
每个Node可以发布消息到一个或多个消息，也可以订阅其他Node发布的消息。
此外，Node还可以提供Service供其他Node调用，或者执行Action以处理复杂任务。

为了便于数据的交换，ROS2提供了一些常用的基础数据类型：

#table(
  columns: 2,
  table.header([基础数据类型], [描述]),
  [`std_msgs/msg/Bool`], [布尔类型],
  [`std_msgs/msg/Byte`], [字节类型],
  [`std_msgs/msg/Char`], [字符类型],
  [`std_msgs/msg/Float32`, `Float64`], [浮点数类型],
  [`std_msgs/msg/Int8`, `UInt8`], [有符号和无符号8位整数类型],
  [`std_msgs/msg/Int16`, `UInt16`], [有符号和无符号16位整数类型],
  [`std_msgs/msg/Int32`, `UInt32`], [有符号和无符号32位整数类型],
  [`std_msgs/msg/Int64`, `UInt64`], [有符号和无符号64位整数类型],
  [`std_msgs/msg/String`], [字符串类型]
)

进一步的，有如下复杂数据类型（这里仅列举了一部分）：

#table(
  columns: 2,
  table.header([复杂数据类型], [描述]),
  [`sensor_msgs/msg/Image`], [图像数据类型，用于表示图像信息],
  [`sensor_msgs/msg/PointCloud2`], [点云数据类型，用于表示三维点云信息],
  [`nav_msgs/msg/Odometry`], [里程计数据类型，用于表示机器人位置和速度],
)

对于静态的数据（如URDF模型），ROS2提供了参数服务器机制对其进行管理。
可以通过命令行的方方式对参数进行发布和提取：

```bash
ros2 param set /node_name param_name value

ros2 param get /node_name param_name
```

在Python脚本中，可以在Node中通过如下方式进行参数的声明和获取：

```python
self.declare_parameter("my_int_param", 42)

int_val = self.get_parameter("my_int_param").get_parameter_value().integer_value
```

如果要在Python中进行跨节点的参数服务器访问，需要实现对应的服务端和客户端，较为复杂，不推荐。推荐使用Python创建命令行子进程进行访问。

== Topic的使用
#line(length: 80%)

Topic是一种异步通信机制，使用发布订阅模型（Publish-Subscribe Model）来实现节点间的数据交换。
节点可以发布消息到一个Topic，也可以订阅其他节点发布的消息。

可以在终端中使用`ros2 topic`命令查看当前系统中的Topic信息。

在Python中可以使用如下方法创建一个Node并发布消息到Topic：

```python
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class WheelVeloctyPublisher(Node):
    def __init__(self):
        super().__init__('wheel_velocty_publisher')
        self.wheel_velocity_publisher = self.create_publisher(
            Float64MultiArray,
            "/wheel/commands", 
            10
        )

    def publish_wheel_velocity(self, velocity):
        msg = Float64MultiArray()
        msg.data = array('d', velocity)
        self.wheel_velocity_publisher.publish(msg)
```

上面这个发布者例子中，定义了一个继承自`Node`的类`WheelVeloctyPublisher`，在其构造函数中创建了一个发布者`wheel_velocity_publisher`，向名为`/wheel/commands`的Topic发送`Float64MultiArray`类型的数据。
在类函数`publish_wheel_velocity`中，创建了一个消息对象`msg`，并将传入的速度数据赋值给`msg.data`，最后调用`self.wheel_velocity_publisher.publish(msg)`将消息发布到Topic。

可以使用如下方式创建一个订阅者：

```python
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class WheelVeloctySubscriber(Node):
    def __init__(self):
        super().__init__('wheel_velocity_subscriber')
        self.wheel_velocity_subscriber = self.create_subscription(
            Float64MultiArray,
            "/wheel/commands",
            self.wheel_velocity_handler,
            10
        )

    def wheel_velocity_handler(self, msg):
        // 处理接收到的消息
        pass
```

上面这个订阅者例子通过`self.create_subscription()`创建了一个订阅者`wheel_velocity_subscriber`，订阅名为`/wheel/commands`的Topic，并指定消息类型为`Float64MultiArray`。当接收到消息时，会调用`wheel_velocity_handler`方法，在该方法中处理接收到的消息。

以上便是ROS2中Topic的基本使用方法。

== Service的使用
#line(length: 80%)

// Service是一种同步通信机制，使用请求-响应模型（Request-Response Model）来实现节点间的通信。
// Service允许一个节点向另一个节点发送请求，并等待响应。
// 可以在终端中使用`ros2 service`命令查看当前系统中的Service信息。

// 下面以实现一个将两个整数相加的Service为例，展示如何在Python中使用Service。

// === 定义Service接口`.srv`文件并添加依赖
// #line(length: 60%)

// ROS2的Service的接口定义依赖于IDL（Interface Definition Language），通常使用`.srv`文件来定义。
// 在`my_package/srv/`目录下创建一个名为`AddTwoInts.srv`的文件，内容如下：

// ```
// int64 a
// int64 b
// ---
// int64 sum
// ```

// 其中`a`和`b`是请求参数，`sum`是响应结果。

// 这些接口依赖于`rosidl_default_generators`包，需要在`package.xml`中添加依赖：

// ```xml
// <build_depend>rosidl_default_generators</build_depend>
// <exec_depend>rosidl_default_runtime</exec_depend>
// <member_of_group>rosidl_interface_packages</member_of_group>
// ```

// 同时还需要修改`CMakeLists.txt`文件：

// ```cmake
// find_package(rosidl_default_generators REQUIRED)

// rosidl_generate_interfaces(${PROJECT_NAME}
//   "srv/AddTwoInts.srv"
// )
// ```

// === 创建Service服务器
// #line(length: 60%)

// 在Python中创建Service服务器节点可以使用如下代码：

// ```python
// import rclpy
// from rclpy.node import Node
// from my_package.srv import AddTwoInts

// class AddTwoIntsServer(Node):
//     def __init__(self):
//         super().__init__('add_two_ints_server')
//         self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

//     def add_two_ints_callback(self, request, response):
//         response.sum = request.a + request.b
//         self.get_logger().info(f'Request: a={request.a}, b={request.b}, sum={response.sum}')
//         return response

// def main(args=None):
//     rclpy.init(args=args)
//     node = AddTwoIntsServer()
//     rclpy.spin(node)
//     rclpy.shutdown()
// ```

// 在上面的代码中，定义了一个继承自`Node`的类`AddTwoIntsServer`，在其构造函数中创建了一个Service服务器，监听名为`add_two_ints`的Service，并指定请求和响应的类型为`AddTwoInts`。当接收到请求时，会调用`add_two_ints_callback`方法，在该方法中计算两个整数的和，并将结果返回。

// === 创建Service客户端
// #line(length: 60%)

// 在Python中创建Service客户端节点可以使用如下代码：

// ```python
// import rclpy
// from rclpy.node import Node
// from my_package.srv import AddTwoInts

// class AddTwoIntsClient(Node):
//     def __init__(self):
//         super().__init__('add_two_ints_client')
//         self.client = self.create_client(AddTwoInts, 'add_two_ints')

//     def send_request(self, a, b):
//         request = AddTwoInts.Request()
//         request.a = a
//         request.b = b
//         future = self.client.call_async(request)
//         rclpy.spin_until_future_complete(self, future)
//         if future.result() is not None:
//             self.get_logger().info(f'Response: sum={future.result().sum}')
//         else:
//             self.get_logger().error('Service call failed')
//         return future.result()

// def main(args=None):
//     rclpy.init(args=args)
//     client = AddTwoIntsClient()
//     response = client.send_request(3, 5)
//     rclpy.shutdown()
// ```

// 在上面的代码中，定义了一个继承自`Node`的类`AddTwoIntsClient`，在其构造函数中创建了一个Service客户端，连接到名为`add_two_ints`的Service。
// 通过调用`send_request`方法，可以向Service发送请求，并等待响应。

// 以上便是ROS2中Service的基本使用方法。
// Service通常用于请求-响应式通信，如获取传感器状态、执行特定操作等。

// === 配置`setup.py`文件
// #line(length: 60%)

// 在`setup.py`文件中，需要在`entry_points`中添加Service的相关配置：

// ```python
// entry_points={
//     'console_scripts': [
//         'add_two_ints_server = my_package.add_two_ints_server:main',
//         'add_two_ints_client = my_package.add_two_ints_client:main',
//     ],
// },
// ```

== Action的使用
#line(length: 80%)

// Action是一种用于处理长时间运行任务的通信机制，使用目标-反馈-结果模型（Goal-Feedback-Result Model）来实现节点间的通信。
// Action允许一个节点向另一个节点发送目标，并在任务执行过程中接收反馈，最终获取结果。
// 可以在终端中使用`ros2 action`命令查看当前系统中的Action信息。

// 下面以实现一个Fibonacci数列计算的Action为例，展示如何在Python中使用Action。

// === 定义Action接口`.action`文件并添加依赖
// #line(length: 60%)

// ROS2的Action的接口定义依赖于IDL（Interface Definition Language），通常使用`.action`文件来定义。
// 在`my_package/action/`目录下创建一个名为`Fibonacci.action`的文件，内容如下：

// ```
// int32 order
// ---
// int32[] sequence
// ---
// int32[] partial_sequence
// ```

// - `goal`：请求的参数是`order`，表示要生成多少个斐波那契数。
// - `result`：最终结果是一个整数数组。
// - `feedback`：每次计算后发送部分结果。

// 这些接口依赖于`rosidl_default_generators`包，需要在`package.xml`中添加依赖：

// ```xml
// <build_depend>rosidl_default_generators</build_depend>
// <exec_depend>rosidl_default_runtime</exec_depend>
// <member_of_group>rosidl_interface_packages</member_of_group>
// ```

// 同时还需要修改`CMakeLists.txt`文件：

// ```cmake
// find_package(rosidl_default_generators REQUIRED)
// rosidl_generate_interfaces(${PROJECT_NAME}
//   "action/Fibonacci.action"
// )
// ```

// === 创建Action服务器
// #line(length: 60%)

// 在Python中创建Action服务器节点可以使用如下代码：

// ```python
// import rclpy
// from rclpy.node import Node
// from my_package.action import Fibonacci
// from rclpy.action import ActionServer

// class FibonacciActionServer(Node):
//     def __init__(self):
//         super().__init__('fibonacci_action_server')
//         self._action_server = ActionServer(
//             self,
//             Fibonacci,
//             'fibonacci',
//             self.execute_callback
//         )

//     def execute_callback(self, goal_handle):
//         order = goal_handle.request.order
//         feedback_msg = Fibonacci.Feedback()
//         feedback_msg.partial_sequence = []

//         a, b = 0, 1
//         for i in range(order):
//             feedback_msg.partial_sequence.append(a)
//             goal_handle.publish_feedback(feedback_msg)
//             a, b = b, a + b

//         result = Fibonacci.Result()
//         result.sequence = feedback_msg.partial_sequence
//         goal_handle.succeed()
//         return result

// def main(args=None):
//     rclpy.init(args=args)
//     node = FibonacciActionServer()
//     rclpy.spin(node)
//     rclpy.shutdown()
// ```

// 在上面的代码中，定义了一个继承自`Node`的类`FibonacciActionServer`，在其构造函数中创建了一个Action服务器，监听名为`fibonacci`的Action，并指定请求和响应的类型为`Fibonacci`。当接收到目标时，会调用`execute_callback`方法，在该方法中计算斐波那契数列，并通过`goal_handle.publish_feedback()`发送反馈，最终返回结果。

// === 创建Action客户端
// #line(length: 60%)

// 在Python中创建Action客户端节点可以使用如下代码：

// ```python
// import rclpy
// from rclpy.node import Node
// from my_package.action import Fibonacci
// from rclpy.action import ActionClient

// class FibonacciActionClient(Node):
//     def __init__(self):
//         super().__init__('fibonacci_action_client')
//         self._action_client = ActionClient(self, Fibonacci, 'fibonacci')

//     def send_goal(self, order):
//         goal_msg = Fibonacci.Goal()
//         goal_msg.order = order

//         self._action_client.wait_for_server()
//         future = self._action_client.send_goal_async(goal_msg)
//         rclpy.spin_until_future_complete(self, future)

//         if future.result() is not None:
//             goal_handle = future.result()
//             feedback_future = goal_handle.get_result_async()
//             rclpy.spin_until_future_complete(self, feedback_future)
//             result = feedback_future.result().result
//             self.get_logger().info(f'Result: {result.sequence}')
//         else:
//             self.get_logger().error('Action call failed')
//         return future.result()

// def main(args=None):
//     rclpy.init(args=args)
//     client = FibonacciActionClient()
//     client.send_goal(5)  # 计算前5个斐波那契数
//     rclpy.shutdown()
// ```

// 在上面的代码中，定义了一个继承自`Node`的类`FibonacciActionClient`，在其构造函数中创建了一个Action客户端，连接到名为`fibonacci`的Action。
// 通过调用`send_goal`方法，可以向Action发送目标，并等待结果。

// === 配置`setup.py`文件
// #line(length: 60%)

// 在`setup.py`文件中，需要在`entry_points`中添加Action的相关配置：

// ```python
// entry_points={
//     'console_scripts': [
//         'fibonacci_action_server = my_package.fibonacci_action_server:main',
//         'fibonacci_action_client = my_package.fibonacci_action_client:main',
//     ],
// },
// ```
