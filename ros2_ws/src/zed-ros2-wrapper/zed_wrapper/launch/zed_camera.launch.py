# Copyright 2024 Stereolabs
#
# Licensed under the Apache License, Version 2.0 (the 'License');
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an 'AS IS' BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import sys

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    SetEnvironmentVariable,
    LogInfo
)
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    Command,
    TextSubstitution
)
from launch_ros.actions import (
    Node,
    ComposableNodeContainer,
    LoadComposableNodes
)
from launch_ros.descriptions import ComposableNode

# ZED Configurations to be loaded by ZED Node
default_config_common = os.path.join(
    get_package_share_directory('zed_wrapper'),
    'config',
    'common'
)
    
# FFMPEG Configuration to be loaded by ZED Node
default_config_ffmpeg = os.path.join(
    get_package_share_directory('zed_wrapper'),
    'config',
    'ffmpeg.yaml'
)

# URDF/xacro file to be loaded by the Robot State Publisher node
default_xacro_path = os.path.join(
    get_package_share_directory('zed_wrapper'),
    'urdf',
    'zed_descr.urdf.xacro'
)


def parse_array_param(param):
    str = param.replace('[', '')
    str = str.replace(']', '')
    arr = str.split(',')

    return arr


def launch_setup(context, *args, **kwargs):
    return_array = []

    wrapper_dir = get_package_share_directory('zed_wrapper')    

    # Launch configuration variables
    svo_path = LaunchConfiguration('svo_path')

    use_sim_time = LaunchConfiguration('use_sim_time')
    sim_mode = LaunchConfiguration('sim_mode')
    sim_address = LaunchConfiguration('sim_address')
    sim_port = LaunchConfiguration('sim_port')

    stream_address = LaunchConfiguration('stream_address')
    stream_port = LaunchConfiguration('stream_port')

    container_name = LaunchConfiguration('container_name')
    namespace = LaunchConfiguration('namespace')
    camera_name = LaunchConfiguration('camera_name')
    camera_model = LaunchConfiguration('camera_model')

    node_name = LaunchConfiguration('node_name')

    config_common_path = LaunchConfiguration('config_path')
    config_ffmpeg = LaunchConfiguration('ffmpeg_config_path')

    serial_number = LaunchConfiguration('serial_number')

    publish_urdf = LaunchConfiguration('publish_urdf')
    publish_tf = LaunchConfiguration('publish_tf')
    publish_map_tf = LaunchConfiguration('publish_map_tf')
    publish_imu_tf = LaunchConfiguration('publish_imu_tf')
    xacro_path = LaunchConfiguration('xacro_path')

    custom_baseline = LaunchConfiguration('custom_baseline')

    ros_params_override_path = LaunchConfiguration('ros_params_override_path')

    enable_gnss = LaunchConfiguration('enable_gnss')
    gnss_antenna_offset = LaunchConfiguration('gnss_antenna_offset')

    container_name_val = container_name.perform(context)
    namespace_val = namespace.perform(context)
    camera_name_val = camera_name.perform(context)
    camera_model_val = camera_model.perform(context)
    node_name_val = node_name.perform(context)
    enable_gnss_val = enable_gnss.perform(context)
    gnss_coords = parse_array_param(gnss_antenna_offset.perform(context))
    custom_baseline_val = custom_baseline.perform(context)

    if (camera_name_val == ''):
        camera_name_val = 'zed'

    if (camera_model_val == 'virtual' and float(custom_baseline_val) <= 0):
        return [
            LogInfo(msg="Please set a positive value for the 'custom_baseline' argument when using a 'virtual' Stereo Camera with two ZED X One devices."),
        ]
    
    if(namespace_val == ''):
        namespace_val = camera_name_val
    else:
        node_name_val = camera_name_val
    
    config_common_path_val = config_common_path.perform(context)
    if (config_common_path_val == ''):
        if (camera_model_val == 'zed' or 
            camera_model_val == 'zedm' or 
            camera_model_val == 'zed2' or 
            camera_model_val == 'zed2i' or 
            camera_model_val == 'zedx' or 
            camera_model_val == 'zedxm' or
            camera_model_val == 'virtual'):
            config_common_path_val = default_config_common + '_stereo.yaml'
        else:
            config_common_path_val = default_config_common + '_mono.yaml'

    print('Using common configuration file: ' + config_common_path_val)

    config_camera_path = os.path.join(
        get_package_share_directory('zed_wrapper'),
        'config',
        camera_model_val + '.yaml'
    )

    # Xacro command with options
    xacro_command = []
    xacro_command.append('xacro')
    xacro_command.append(' ')
    xacro_command.append(xacro_path.perform(context))
    xacro_command.append(' ')
    xacro_command.append('camera_name:=')
    xacro_command.append(camera_name_val)
    xacro_command.append(' ')
    xacro_command.append('camera_model:=')
    xacro_command.append(camera_model_val)
    xacro_command.append(' ')
    xacro_command.append('custom_baseline:=')
    xacro_command.append(custom_baseline_val)   
    if(enable_gnss_val=='true'):
        xacro_command.append(' ')
        xacro_command.append('enable_gnss:=true')
        xacro_command.append(' ')
        if(len(gnss_coords)==3):
            xacro_command.append('gnss_x:=')
            xacro_command.append(gnss_coords[0])
            xacro_command.append(' ')
            xacro_command.append('gnss_y:=')
            xacro_command.append(gnss_coords[1])
            xacro_command.append(' ')
            xacro_command.append('gnss_z:=')
            xacro_command.append(gnss_coords[2])
            xacro_command.append(' ')

    # Robot State Publisher node
    rsp_name = camera_name_val + '_state_publisher'
    rsp_node = Node(
        condition=IfCondition(publish_urdf),
        package='robot_state_publisher',
        namespace=namespace_val,
        executable='robot_state_publisher',
        name=rsp_name,
        output='screen',
        parameters=[{
            'robot_description': Command(xacro_command)
        }]
    )
    return_array.append(rsp_node)

    # ROS 2 Component Container
    if(container_name_val == ''):
        container_name_val='zed_container'
        distro = os.environ['ROS_DISTRO']
        if distro == 'foxy':
            # Foxy does not support the isolated mode
            container_exec='component_container'
        else:
            container_exec='component_container_isolated'
        
        zed_container = ComposableNodeContainer(
                name=container_name_val,
                namespace=namespace_val,
                package='rclcpp_components',
                executable=container_exec,
                arguments=['--ros-args', '--log-level', 'info'],
                output='screen',
        )
        return_array.append(zed_container)

    # ZED Node parameters
    node_parameters = [
            # YAML files
            config_common_path_val,  # Common parameters
            config_camera_path,  # Camera related parameters
            config_ffmpeg, # FFMPEG parameters
            # Overriding
            {
                'use_sim_time': use_sim_time,
                'simulation.sim_enabled': sim_mode,
                'simulation.sim_address': sim_address,
                'simulation.sim_port': sim_port,
                'stream.stream_address': stream_address,
                'stream.stream_port': stream_port,
                'general.camera_name': camera_name_val,
                'general.camera_model': camera_model_val,
                'svo.svo_path': svo_path,
                'general.serial_number': serial_number,
                'pos_tracking.publish_tf': publish_tf,
                'pos_tracking.publish_map_tf': publish_map_tf,
                'sensors.publish_imu_tf': publish_imu_tf,
                'gnss_fusion.gnss_fusion_enabled': enable_gnss
            }
        ]
    
    if( ros_params_override_path.perform(context) != ''):
        node_parameters.append(ros_params_override_path)

    # ZED Wrapper component
    if( camera_model_val=='zed' or
        camera_model_val=='zedm' or
        camera_model_val=='zed2' or
        camera_model_val=='zed2i' or
        camera_model_val=='zedx' or
        camera_model_val=='zedxm' or
        camera_model_val=='virtual'):
        zed_wrapper_component = ComposableNode(
            package='zed_components',
            namespace=namespace_val,
            plugin='stereolabs::ZedCamera',
            name=node_name_val,
            parameters=node_parameters,
            extra_arguments=[{'use_intra_process_comms': True}]
        )
    else: # 'zedxonegs' or 'zedxone4k')
        zed_wrapper_component = ComposableNode(
            package='zed_components',
            namespace=namespace_val,
            plugin='stereolabs::ZedCameraOne',
            name=node_name_val,
            parameters=node_parameters,
            extra_arguments=[{'use_intra_process_comms': True}]
        )
    
    full_container_name = '/' + namespace_val + '/' + container_name_val
    info = '* Loading ZED node: ' + node_name_val + ' in container: ' + full_container_name
    return_array.append(LogInfo(msg=TextSubstitution(text=info)))
    
    load_composable_node = LoadComposableNodes(
        target_container=full_container_name,
        composable_node_descriptions=[zed_wrapper_component]
    )
    return_array.append(load_composable_node)

    return return_array

def generate_launch_description():
    return LaunchDescription(
        [
            SetEnvironmentVariable(name='RCUTILS_COLORIZED_OUTPUT', value='1'),
            DeclareLaunchArgument(
                'camera_name',
                default_value=TextSubstitution(text='zed'),
                description='The name of the camera. It can be different from the camera model and it will be used as node `namespace`.'),
            DeclareLaunchArgument(
                'camera_model',
                description='[REQUIRED] The model of the camera. Using a wrong camera model can disable camera features.',
                choices=['zed', 'zedm', 'zed2', 'zed2i', 'zedx', 'zedxm', 'virtual', 'zedxonegs', 'zedxone4k']),
            DeclareLaunchArgument(
                'container_name',
                default_value='',
                description='The name of the container to be used to load the ZED component. If empty (default) a new container will be created.'),
            DeclareLaunchArgument(
                'namespace',
                default_value='',
                description='The namespace of the node. If empty (default) the camera name is used.'),
            DeclareLaunchArgument(
                'node_name',
                default_value='zed_node',
                description='The name of the zed_wrapper node. All the topic will have the same prefix: `/<camera_name>/<node_name>/`. If a namespace is specified, the node name is replaced by the camera name.'),
            DeclareLaunchArgument(
                'config_path',
                default_value='',
                description='Path to the YAML configuration file for the camera. common_stereo.yaml is used by default for stereo cameras, common_mono.yaml for mono cameras.'),
            DeclareLaunchArgument(
                'ffmpeg_config_path',
                default_value=TextSubstitution(text=default_config_ffmpeg),
                description='Path to the YAML configuration file for the FFMPEG parameters when using FFMPEG image transport plugin.'),                
            DeclareLaunchArgument(
                'serial_number',
                default_value='0',
                description='The serial number of the camera to be opened. It is mandatory to use this parameter in multi-camera rigs to distinguish between different cameras.'),
            DeclareLaunchArgument(
                'publish_urdf',
                default_value='true',
                description='Enable URDF processing and starts Robot State Published to propagate static TF.',
                choices=['true', 'false']),
            DeclareLaunchArgument(
                'publish_tf',
                default_value='true',
                description='Enable publication of the `odom -> camera_link` TF.',
                choices=['true', 'false']),
            DeclareLaunchArgument(
                'publish_map_tf',
                default_value='true',
                description='Enable publication of the `map -> odom` TF. Note: Ignored if `publish_tf` is False.',
                choices=['true', 'false']),
            DeclareLaunchArgument(
                'publish_imu_tf',
                default_value='true',
                description='Enable publication of the IMU TF. Note: Ignored if `publish_tf` is False.',
                choices=['true', 'false']),
            DeclareLaunchArgument(
                'xacro_path',
                default_value=TextSubstitution(text=default_xacro_path),
                description='Path to the camera URDF file as a xacro file.'),
            DeclareLaunchArgument(
                'ros_params_override_path',
                default_value='',
                description='The path to an additional parameters file to override the defaults'),
            DeclareLaunchArgument(
                'svo_path',
                default_value=TextSubstitution(text='live'),
                description='Path to an input SVO file.'),
            DeclareLaunchArgument(
                'enable_gnss',
                default_value='false',
                description='Enable GNSS fusion to fix positional tracking pose with GNSS data from messages of type `sensor_msgs::msg::NavSatFix`. The fix topic can be customized in `common_stereo.yaml`.',
                choices=['true', 'false']),
            DeclareLaunchArgument(
                'gnss_antenna_offset',
                default_value='[]',
                description='Position of the GNSS antenna with respect to the mounting point of the ZED camera. Format: [x,y,z]'),
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='false',
                description='If set to `true` the node will wait for messages on the `/clock` topic to start and will use this information as the timestamp reference',
                choices=['true', 'false']),
            DeclareLaunchArgument(
                'sim_mode',
                default_value='false',
                description='Enable simulation mode. Set `sim_address` and `sim_port` to configure the simulator input.',
                choices=['true', 'false']),
            DeclareLaunchArgument(
                'sim_address',
                default_value='127.0.0.1',
                description='The connection address of the simulation server. See the documentation of the supported simulation plugins for more information.'),
            DeclareLaunchArgument(
                'sim_port',
                default_value='30000',
                description='The connection port of the simulation server. See the documentation of the supported simulation plugins for more information.'),
            DeclareLaunchArgument(
                'stream_address',
                default_value='',
                description='The connection address of the input streaming server.'),
            DeclareLaunchArgument(
                'stream_port',
                default_value='30000',
                description='The connection port of the input streaming server.'),
            DeclareLaunchArgument(
                'custom_baseline',
                default_value='0.0',
                description='Distance between the center of ZED X One cameras in a custom stereo rig.'),
            OpaqueFunction(function=launch_setup)
        ]
    )
