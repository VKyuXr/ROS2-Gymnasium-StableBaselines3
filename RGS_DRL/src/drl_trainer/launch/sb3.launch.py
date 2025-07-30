from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():
    sb3_trainer_node = Node(
        package="drl_trainer",
        executable="sb3_trainer",
        output="screen",
    )

    return LaunchDescription([
        sb3_trainer_node,
    ])
