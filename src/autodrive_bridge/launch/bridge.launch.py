from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ackermann_to_autodrive_node = Node(
        package="autodrive_bridge",
        executable="ackermann_to_autodrive_node",
        name="ackermann_to_autodrive_node",
        output="screen",
    )

    wheel_odometry_node = Node(
        package="autodrive_bridge",
        executable="wheel_odometry_node",
        name="wheel_odometry_node",
        output="screen",
    )

    return LaunchDescription([ackermann_to_autodrive_node, wheel_odometry_node])
