from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='automatix_my_nav2_system',
            executable='navigate_to_pose_client',
            output='screen'
        ),
    ])