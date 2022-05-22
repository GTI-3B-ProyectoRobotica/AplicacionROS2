from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='automatix_guardar_zona_service',
            executable='automatix_guardar_zona_server',
            output='screen'
        ),
    ])