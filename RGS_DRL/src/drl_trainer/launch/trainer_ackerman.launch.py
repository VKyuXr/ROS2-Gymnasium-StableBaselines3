from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():
    laser_processer_node = Node(
        package="drl_trainer",
        executable="laser_processor",
        output="screen",
    )

    ackerman_forwarder_node = Node(
        package="drl_trainer",
        executable="ackerman_forwarder",
        output="screen",
    )

    return LaunchDescription([
        laser_processer_node,
        ackerman_forwarder_node,
    ])
