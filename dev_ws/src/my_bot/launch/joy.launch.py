import os
from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    path_joy_config = os.path.join(
        get_package_share_directory('my_bot'), 'config', 'joy.yaml')
    
    # run joy_node from joy package
    joy_node = Node(
        package='joy',
        executable='joy_node',
        parameters=[path_joy_config]
    )

    # teleop reads from /joy and publishes to /cmd_vel as Twist
    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_node',
        parameters=[path_joy_config]
    )

    return LaunchDescription([
        joy_node,
        teleop_node
    ])