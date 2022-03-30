from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='automatix_escaneo_autonomo',
            executable='escaneo_autonomo_server',
            output='screen'
        ),
    ])