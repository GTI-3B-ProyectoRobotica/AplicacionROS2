from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    # posicion
    pose_x = LaunchConfiguration('pose-x', default='0')
    pose_y = LaunchConfiguration('pose-y', default='0')

    #orientacion
    orien_x = LaunchConfiguration('orien-x', default='0')
    orien_y = LaunchConfiguration('orien-y', default='0')
    orien_z = LaunchConfiguration('orien-z', default='0')
    orien_w = LaunchConfiguration('orien-w', default='0')
    
    return LaunchDescription([

        DeclareLaunchArgument(
            'pose-x',
            default_value="0",
            description='Posicion X del robot'),

        DeclareLaunchArgument(
            'pose-y',
            default_value= "0",
            description='Posicion Y del robot'),


        DeclareLaunchArgument(
            'orien-x',
            default_value= "0",
            description='Posicion Y del robot'),

        DeclareLaunchArgument(
            'orien-y',
            default_value= "0",
            description='Posicion Y del robot'),

        DeclareLaunchArgument(
            'orien-z',
            default_value= "0",
            description='Posicion Y del robot'),

        DeclareLaunchArgument(
            'orien-w',
            default_value= "1",
            description='Posicion Y del robot'),

        Node(
            package='automatix_my_nav2_system',
            executable='navigate_to_pose_client',
            output='screen',
            parameters=[
                {'pose-x':pose_x},
                {'pose-y':pose_y},

                {'orien-x':orien_x},
                {'orien-y':orien_y},
                {'orien-z':orien_z},
                {'orien-w':orien_w}
                #{'pose':{'x':pose_x,'y':pose_y}}, 
                #{'orientacion':{'x':orien_x,'y':orien_y,'z':orien_z,'w':orien_w}}, 
            ]
        ),
    ])