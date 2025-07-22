由于ROS2并没有集成开发环境，所以理清楚ROS2项目的结构对ROS2的使用非常重要的。
为了记录完整的ROS2学习流程，我们将从ROS2项目的创建开始。

ROS2的安装过程在 https://docs.ros.org/ 中有详细的说明。
这里跳过了ROS2的安装过程，假设你已经安装好了ROS2，并且可以使用`ros2`命令。

== 创建ROS2项目的完整流程
#line(length: 80%) 

ROS2项目包含多个包（Package），每个包可以包含多个节点（Node）。
每个节点可以实现特定的功能，如传感器数据处理、控制算法等。

=== 创建并初始化工作空间
#line(length: 60%) 

在ROS2中，工作空间是一个包含多个包的目录结构。
我们可以使用以下命令创建一个新的工作空间：

```bash
cd <path-of-workspace>
mkdir -p <name-of-workspace>
```

然后需要创建一个文件夹用于放置功能包的源代码：

```bash
mkdir -p <name-of-workspace>/src
```

接下来，使用`colcon`命令初始化工作空间：

```bash
cd <name-of-workspace>
colcon build
source install/setup.bash
```

这将编译所有的包，并将它们安装到工作空间的`install`目录中。
编译完成后，我们需要使用`source`命令来设置环境变量，以便ROS2能够找到我们编译的包。
第一次时可能会报错，因为还没有编译内容，这一步主要是初始化环境。

=== 创建描述性资源
#line(length: 60%) 

为了保持工作空间的整洁和有序，我们通常会将描述性资源（如机器人模型、环境等）放在一个单独的包中。
在这个包中，我们只需要实现描述性资源的存放和加载功能，而不需要实现具体的算法或控制逻辑。

在工作空间的`src`目录下创建一个新的包来存放这些资源。可以使用以下命令：

```bash
cd src
ros2 pkg create <name-of-package> --build-type ament_cmake
```

+ `<name-of-package>`是你要创建的包的名称，建议将其命名为`<name>_description`，表示这是一个描述性包。
+ `--build-type`参数指定了包的构建类型，`ament_cmake`是ROS2中常用的构建类型，表示使用CMake进行构建。`--build-type`参数的值可以是`ament_cmake`或`ament_python`，具体取决于你要实现的功能。`ament_cmake`适用于C++包，而`ament_python`适用于Python包。通常描述性资源包使用`ament_cmake`。

在使用ROS2进行仿真时，有两种资源必不可少：一是控制对象，即机器人的模型。在ROS2中，通常使用URDF（Unified Robot Description Format）来描述机器人的模型。二是环境，即机器人的工作场景，通常使用Gazebo等仿真工具来创建。

在`<name-of-package>`中添加`urdf`目录和`worlds`目录，并放入对应的资源文件。`urdf`目录用于存放机器人的URDF文件，而`worlds`目录用于存放Gazebo的世界文件。

然后，在 `<name-of-package>/launch` 中创建 `.launch.py` 文件用于加载资源。

上述资源只是我们在创建源代码时的内容.
在ROS2的实际工作过程中，ROS2并不会在我们存放文件的目录中直接使用这些资源，而是在使用`colcon build`命令的时候将其安装到工作空间的`install`目录中。
因此我们还需要在包的根目录下创建一个 `CMakeLists.txt` 文件来描述如何安装这些资源。

在我们使用`ros2 pkg create`命令时，ROS2会自动生成一个基本的 `CMakeLists.txt` 文件。
在这个文件中，我们需要添加如下指令来安装URDF和launch文件：

```Cmake
install(
  DIRECTORY urdf launch
  DESTINATION share/${PROJECT_NAME}/
)
```

=== 创建算法功能包
#line(length: 60%) 

回到 `src` 目录，并创建算法功能包：

```bash
ros2 pkg create <name-of-package> --build-type <ament_type>
```

其中`<ament_type>`可以是`ament_cmake`或`ament_python`，具体取决于你要实现的功能。

在这里仅创建一个空的包，而忽略这个包中具体的算法实现。

=== 编译整个工作空间
#line(length: 60%) 

为了运行ROS2项目，我们需要编译整个工作空间。
在工作空间的根目录下，使用以下命令编译整个工作空间：

```bash
cd <name-of-workspace>
colcon build
source install/setup.bash
```

=== 启动目标节点
#line(length: 60%) 

ROS2的具体运行方式通常是通过`ros2 launch`命令来启动一个或多个节点。
在运行之前，我们需要确保工作空间已经编译成功，并且环境变量已经设置好。
然后使用以下命令来运行一个launch文件：

```bash
ros2 launch <name-of-package> <name-of-launch>
```

== `.urdf`文件与`.xacro`的编写
#line(length: 80%)

URDF（Unified Robot Description Format）是ROS2中用于描述机器人的模型文件格式，通常包含机器人的几何形状、关节、传感器等信息。
其通常位于包的`urdf`目录下，文件名通常以`.urdf`或`.xacro`结尾。
URDF文件可以使用XML格式编写，也可以使用Xacro（XML Macros）格式编写。
Xacro是一种ROS2提供的宏语言，可以简化URDF文件的编写。

=== `.urdf`文件的基本结构
#line(length: 60%)

`.urdf`文件可以理解为是由多个模块组成的，每个模块描述了机器人的一个部分。
下面是一些URDF文件的基本元素：

#table(
  columns: 2,
  table.header(
    [元素], [作用]
  ),
  [link], [描述机器人的一个部分，如手臂、腿等],
  [joint], [描述机器人的关节，如旋转关节、滑动关节等],
)

link的参数使用一对`<link>`标签来定义，通常包含以下部分：

```xml
<link name="link_name">

  <visual>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <box size="1 1 1"/>
    </geometry>
    <material name="material_name"/>
  </visual>

  <collision>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <box size="1 1 1"/>
    </geometry>
  </collision>

  <inertial>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <mass value="1.0"/>
    <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
  </inertial>

</link>
```

- `<visual>`：用于描述机器人的外观。`name`属性指定了链接的名称。
  - `<origin>`：指定了几何形状的原点位置`xyz`和朝向`rpy`。
  - `<geometry>`：指定几何形状，可以是`box`、`cylinder`、`sphere`等。
  - `<material>`：指定几何形状的材质，可以指定颜色、纹理等。
- `<collision>`：指定机器人的碰撞体积，用于物理仿真。
  - `<origin>`：指定了碰撞体积的原点位置`xyz`和朝向`rpy`。
  - `<geometry>`：指定了碰撞体积的几何形状，通常与`<visual>`元素相同。
- `<inertial>`：描述了机器人的惯性属性。
  - `<origin>`：指定了惯性属性的原点位置`xyz`和朝向`rpy`。
  - `<mass>`：指定了机器人的质量。
  - `<inertia>`：指定了机器人的惯性矩阵，包括`ixx`、`iyy`、`izz`等元素。

joint的参数使用一对`<joint>`标签来定义，通常包含以下部分：
```xml
<joint name="joint_name" type="revolute">
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <parent link="parent_link_name"/>
  <child link="child_link_name"/>
  <axis xyz="1 0 0"/>
  <limit effort="10.0" velocity="1.0" lower="-1.57" upper="1.57"/>
  <dynamics damping="0.1" friction="0.1"/>
</joint>
```

`name`用于定义关节的名称，`type`用于定义关节的类型。`type`可以是以下几种类型：

#table(
  columns: 3,
  table.header(
    [类型], [自由度], [描述]
  ),
  [`fixed`], [0], [固定关节，不允许移动],
  [`prismatic`], [1], [滑动关节，允许沿一个轴滑动],
  [`revolute`], [1], [旋转关节，允许绕一个轴旋转],
  [`continuous`], [1], [连续旋转关节，允许绕一个轴无限旋转],
  [`planar`], [3], [平面关节，允许在一个平面内移动],
  [`floating`], [6], [浮动关节，允许在三维空间内自由移动],
)

- `<origin>`：指定了关节的原点位置`xyz`和朝向`rpy`。
- `<parent>`：指定了关节的父链接`link`。
- `<child>`：指定了关节的子链接`link`。
- `<axis>`：指定了关节的旋转轴或滑动轴`xyz`。
- `<limit>`：指定了关节的限制条件，如最大力矩`effort`（N或N·m）、最大速度`velocity`（m/s或rad/s）、最小位移`lower`（m或rad）和最大位移`upper`（m或rad）。
- `<dynamics>`：指定了关节的动力学属性，如阻尼`damping`和摩擦`friction`。

URDF可以看作是一个将link作为边，joint作为节点的图结构。

可以使用`urdf_check`命令来检查URDF文件的语法是否正确：

```bash
urdf_check <urdf-file>
```

这里只介绍最简单的URDF文件结构，更多的URDF元素和属性会在之后的具体实例中介绍。

=== `.xacro`文件的基本结构
#line(length: 60%)

`.xacro`文件是ROS2中用于生成URDF文件的宏语言文件，其基本结构与URDF文件类似，但它支持宏定义和参数化，可以更方便地生成复杂的URDF文件。`.xacro`文件可以使用以下标签对象来定义：

#table(
  columns: 2,
  table.header(
    [元素], [作用]
  ),
  [`<xacro:property>`], [定义一个属性，可以在URDF中使用],
  [`<xacro:macro>`], [定义一个宏，可以在URDF中调用],
  [`<xacro:include>`], [包含其他Xacro文件],
)

`.xacro`的使用方法用下面的例子来说明：

```xml
<?xml version="1.0"?>
<xacro:property name="PI" value="3.141592653589793"/>
<xacro:property name="wheel_radius" value="0.05"/>
<xacro:property name="wheel_width" value="0.03"/>
<xacro:property name="wheel_mass" value="0.5"/>

<xacro:macro name="wheel" params="name prefix parent xyz rpy">
  <link name="${name}">
    <visual>
      <geometry>
        <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
      </geometry>
      <origin xyz="0 0 0" rpy="${PI/2} 0 0"/>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>

    <collision>
      <geometry>
        <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
      </geometry>
      <origin xyz="0 0 0" rpy="${PI/2} 0 0"/>
    </collision>
    
    <inertial>
      <mass value="${wheel_mass}"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0"
               iyy="0.001" iyz="0.0"
               izz="0.001"/>
    </inertial>
  </link>

  <joint name="${prefix}_wheel_joint" type="continuous">
    <parent link="${parent}"/>
    <child link="${name}"/>
    <origin xyz="${xyz}" rpy="${rpy}"/>
    <axis xyz="0 1 0"/>
  </joint>
</xacro:macro>

<xacro:wheel name="left_wheel" prefix="left" parent="base_link"
  xyz="${base_length/2} ${-base_width/2} 0" rpy="0 0 0"/>
```

在这个例子中，我们定义了一个轮子的宏`<xacro:macro>`，它接受四个参数：`name`、`prefix`、`parent`、`xyz`和`rpy`。
这个宏生成了一个轮子的链接和一个关节。我们可以在URDF中多次调用这个宏来生成多个轮子。
`.xacro`文件的语法与URDF文件类似，但它支持更多的功能，如宏定义、属性定义和条件判断等。

=== `.xacro`文件的使用
#line(length: 60%)

可以使用`xacro`命令将`.xacro`文件转换为`.urdf`文件：

```bash
xacro <xacro-file> -o <urdf-file>
```

也可以在ROS2的launch文件中直接使用`.xacro`文件，而不需要手动转换为`.urdf`文件：

```python
import os
import xacro

xacro_file = os.path.join(path, 'urdf', 'my_xacro.xacro')
robot_model = xacro.parse(open(xacro_file)).toxml()

node_robot_state_publisher = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    output='screen',
    parameters=[{'robot_description': robot_model}]
)
```

== `.launch.py`文件的编写
#line(length: 80%) 

在ROS2中，`.launch.py`文件用于定义如何启动一个或多个节点，以及它们之间的关系。
`.launch.py`文件通常位于包的`launch`目录下，用于描述如何启动ROS2节点、加载参数、设置环境变量等。
通常一个`.launch.py`文件都包含有下面三个Python包：

```python
import os # 用于操作系统路径操作。
from launch import LaunchDescription # 启动描述类，用来定义要启动的节点。
from launch_ros.actions import Node # 定义一个 ROS 节点。

# 其他常用
from ament_index_python.packages import get_package_share_directory # 获取指定 ROS 包的共享目录路径（通常存放配置、URDF、meshes 等资源文件）。
```

每个`.launch.py`文件都需要实现`generate_launch_description()`函数。
这个函数返回一个`LaunchDescription`对象，描述了要启动的节点和其他相关配置。

使用如下方式实例化 ROS2 节点描述：

```python
NODE_INSTENCE_NAME = Node(
    package=PACKAGE_NAME_STR,
    executable=PROGRAM_IN_PACKAGE_STR,
    name=NODE_NAME_STR,
    output='both', # 输出日志信息到控制台和日志文件。
    parameters=[{PARAMETER_NAME_STR: PARAMETER_VAR}],
)
```

最后需要将实例化的节点描述传给ROS2：

```python
return LaunchDescription([
    NODE_INSTENCE_NAME
])
```

== 算法功能包的编写
#line(length: 80%)

功能包是ROS2中实现具体功能的核心部分。
功能包通常包含一个或多个节点，每个节点实现特定的功能。

=== 功能包的基本结构
#line(length: 60%)

功能包可以使用C++或Python编写。
在这里以Python为例，介绍功能包的基本结构。
功能包目录通常如下：

```bash
package_name/
├── package.xml
├── setup.py
└── package_name/
    ├── __init__.py
    └── node_1_name.py
    └── node_2_name.py
```

在`setup.py`中注册对应的功能：

```python
entry_points={
    'console_scripts': [
        'my_node_1_exe = package_name.node_1_name:main',
        'my_node_2_exe = package_name.node_2_name:main',
    ],
},
```

`my_node_1_exe`和`my_node_2_exe`的标识符是任意定义的，用于调用，比如在`launch.py`的`Node()`中，将其作为`executable`参数传递给`Node()`：

```python
executable='my_node_1_exe'
```

=== 节点文件的具体实现

节点文件中，都需要实现一个继承自`rclpy.node.Node`的类，用于获取节点的基本能力。如：

```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('node_name')

        # 初始化逻辑
        self.declare_parameter('a', 0)
        a = self.get_parameter('a').get_parameter_value().integer_value

        self.get_logger().info(f'a = {a}')

def main():
    rclpy.init()
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

其中，`self.declare_parameter('<name-of-parameter>', <value-of-default>)`用于声明一个节点可以接受的参数。这个参数可以从`launch.py`文件或命令行传入：

```bash
ros2 run my_package my_node --ros-args -p a:=5
```

或者在`launch.py`文件里，使用下面代码将参数传递给`Node()`：

```python
parameters=[{'a': 5}]
```

#table(
	columns: 2,
	table.header(
		[函数], [作用]
  ),
	[`self.get_parameter('a')`], [获取参数对象],
    [`.get_parameter_value()`], [取参数值部分（可能是 int、double、str 等类型）],
    [`.integer_value`], [获取其整数类型的值]
)
