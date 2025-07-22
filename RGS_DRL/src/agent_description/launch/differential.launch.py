from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit
from launch.substitutions import PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch import LaunchDescription

def generate_launch_description():
    # 获取资源
    agent_decription_path = FindPackageShare("agent_description")
    gazebo_ros_path = FindPackageShare("gazebo_ros")
    differential_path = PathJoinSubstitution([agent_decription_path, "urdf", "differential.urdf"])
    world_path = PathJoinSubstitution([agent_decription_path, "worlds", "empty.world"])

    # 使用 xacro 命令读取 xacro/urdf
    differential_model = Command(["xacro ", differential_path])

    # 启动状态发布节点
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": differential_model}]
    )

    # 启动 gazebo 软件
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([gazebo_ros_path, "launch", "gazebo.launch.py"])
                ]),
                launch_arguments={
                    'world': world_path,
                    'verbose': 'true',
                    }.items()
             )

    # 启动 gazebo_ros.spawn_entity 节点
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'differential_robot'
        ],
        output='screen'
    )

    # 加载反馈器
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start', 'joint_state_broadcaster'],
        output='screen'
    )

    # 加载驱动轮控制器
    load_wheel_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start', 'wheel_controller'],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher_node,
        spawn_entity_node,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity_node,
                on_exit=[load_joint_state_broadcaster],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_wheel_controller],
            )
        )
    ])
