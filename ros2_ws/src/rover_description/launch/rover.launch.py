from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': open(os.path.join(
                    get_package_share_directory('rover_description'),
                    'urdf/rocker_bogie.urdf.xacro'), 'r').read()
            }]
        ),

        # Spawn in Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'rover',
                '-topic', 'robot_description',
                '-x', '0.0',
                '-y', '0.0',
                '-z', '0.1'
            ],
            output='screen'
        ),

        # Controller Manager (Load controllers.yaml)
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[os.path.join(
                get_package_share_directory('rover_description'),
                'config/controllers.yaml'
            )],
            output='screen'
        ),

        # Spawn Controllers
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster"],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["velocity_controllers"],
            output="screen",
        )
    ])