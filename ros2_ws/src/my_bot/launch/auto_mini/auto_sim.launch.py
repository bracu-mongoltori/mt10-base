import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node



def generate_launch_description():
    
    package_name='my_bot'
    pkg_path = os.path.join(get_package_share_directory('my_bot'))
    world_path= os.path.join(pkg_path, "worlds", "obstacle.world")


    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch/auto_mini','auto_rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                    launch_arguments={'world': world_path}.items()
             )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'my_bot'],
                        output='screen')
    
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gazebo_velocity_controller"],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )



    # Launch them all!
    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity,
        diff_drive_spawner,
        joint_broad_spawner,
    ])