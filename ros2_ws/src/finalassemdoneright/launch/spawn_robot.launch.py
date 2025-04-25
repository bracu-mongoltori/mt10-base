import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'finalassemdoneright'
    pkg_share = get_package_share_directory(pkg_name)

    # Absolute path to the URDF file
    urdf_file = os.path.join(pkg_share, 'urdf', 'finalassemdoneright.urdf')

    # Read the URDF file contents
    with open(urdf_file, 'r') as infp:
        robot_description = infp.read()

    # Robot State Publisher Node publishes the TF frames and robot description
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # Optionally, launch RViz2 with a preconfigured view
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'display.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    return LaunchDescription([
        robot_state_publisher_node,
        rviz_node  # Remove this node if you want to launch RViz2 manually
    ])

