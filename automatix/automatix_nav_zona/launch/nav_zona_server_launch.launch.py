from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='automatix_nav_zona',
            executable='nav_zona_server',
            output='screen'
        ),
    ])