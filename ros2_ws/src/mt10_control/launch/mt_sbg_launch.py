import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler, ExecuteProcess, LogInfo
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.events import Shutdown

def generate_launch_description():
    config = '/home/mt/ros2_ws/src/mt10_control/config/mt_sbg_config.yaml'

    # Port finder node
    port_finder_node = Node(
        package='mt10_control',
        executable='port_finder',
        name='port_finder',
        output='screen'
    )

    # SBG driver node
    sbg_node = Node(
        package='sbg_driver',
        executable='sbg_device',
        output='screen',
        parameters=[config]
    )

    # Event handler to start SBG node after port_finder exits
    start_sbg_on_port_finder_exit = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=port_finder_node,
            on_exit=[sbg_node],
        )
    )

    # Event handler to log a message when SBG node starts
    log_sbg_start = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=sbg_node,
            on_start=[LogInfo(msg="SBG driver node started successfully.")],
        )
    )

    return LaunchDescription([
        port_finder_node,
        start_sbg_on_port_finder_exit,
        log_sbg_start,
    ])