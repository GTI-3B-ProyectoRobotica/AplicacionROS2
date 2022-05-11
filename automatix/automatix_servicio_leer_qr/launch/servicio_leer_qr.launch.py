from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    

    

    return LaunchDescription([
        
        Node(
            package='automatix_servicio_leer_qr',
            executable='servicio_leer_qr',
            output='screen'
        ),
    ])