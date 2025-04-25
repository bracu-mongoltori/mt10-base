from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_path = get_package_share_directory('my_bot')
    
    
    
    # The robot_description parameter is created using a Command substitution.
    # We wrap it in a ParameterValue and set value_type to str.
    robot_description = ParameterValue(
        Command([
            'xacro ', os.path.join(pkg_path, 'description', 'rocker_bogie.urdf.xacro')
        ]),
        value_type=str
    )
    
    return LaunchDescription([

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        ),

            ]
        )

