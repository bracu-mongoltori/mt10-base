import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    # print(get_package_share_directory('miniark_ik'))
    # print("-----")
    # config = os.path.join(
    #     get_package_share_directory('miniarm_ik'),
    #     'control_config.yaml'
    # )

    # Debugging: Check if the file exists
    # if not os.path.isfile(config):
    #     print(f"Config file does not exist: {config}")
    # else:
    #     print(f"Config file found: {config}")


    keyb_ik = Node(
        package='miniarm_ik',
        executable='keyb_ik',
        name='keyb_ik',
        
    )
    ik_feed_tuning = Node(
        package='miniarm_ik',
        executable='ik_feed_tuning',
        name='ik_feed_tuning',
    )
    miniarm_ser = Node(
        package='miniarm_ik',
        executable='miniarm_ser',
        name='miniarm_ser',
    )

    ld.add_action(keyb_ik)
    ld.add_action(ik_feed_tuning)
    ld.add_action(miniarm_ser)

    return ld