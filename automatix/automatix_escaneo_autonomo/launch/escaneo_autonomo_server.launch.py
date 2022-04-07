from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            LogInfo, RegisterEventHandler, TimerAction)
from launch.conditions import IfCondition
from launch.event_handlers import (OnExecutionComplete, OnProcessExit,
                                OnProcessIO, OnProcessStart, OnShutdown)
from launch.events import Shutdown
from launch.substitutions import (EnvironmentVariable, FindExecutable,
                                LaunchConfiguration, LocalSubstitution,
                                PythonExpression)

def generate_launch_description():
    
    #escaneo_proceso = ExecuteProcess(
    #    cmd=[[
    #        FindExecutable(name='ros2'),
    #        ' launch ',
    #        'automatix_escaneo_autonomo ',
    #        ' escaneo_autonomo_server.launch.py',
    #    ]],
    #    shell=True
    #)
    

    return LaunchDescription([
        
        Node(
            package='automatix_escaneo_autonomo',
            executable='escaneo_autonomo_server',
            output='screen'
        ),
        #RegisterEventHandler(
        #    OnProcessIO(
        #        target_action=escaneo_proceso,
        #        on_stdout=lambda event: 
        #            #ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=False
        #            print('Spawn request says "{}"'.format(
        #                    event.text.decode().strip())
        #            )
        #    )
        #)
    ])