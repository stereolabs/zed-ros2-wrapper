# Copyright 2025 Stereolabs
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

# Enable colored output
os.environ["RCUTILS_COLORIZED_OUTPUT"] = "1"

# ZED Configurations to be loaded by ZED Node
default_config_common = os.path.join(
    get_package_share_directory('zed_wrapper'),
    'config',
    'common'
)

# Object Detection Configuration to be loaded by ZED Node
default_object_detection_config_path = os.path.join(
    get_package_share_directory('zed_wrapper'),
    'config',
    'object_detection.yaml'
)
# Custom Object Detection Configuration to be loaded by ZED Node
default_custom_object_detection_config_path = os.path.join(
    get_package_share_directory('zed_wrapper'),
    'config',
    'custom_object_detection.yaml'
)

# URDF/xacro file to be loaded by the Robot State Publisher node
default_xacro_path = os.path.join(
    get_package_share_directory('zed_wrapper'),
    'urdf',
    'zed_descr.urdf.xacro'
)

# Function to parse array-like launch arguments
def parse_array_param(param):
    cleaned = param.replace('[', '').replace(']', '').replace(' ', '')
    if not cleaned:
        return []
    return cleaned.split(',')

def launch_setup(context, *args, **kwargs):
    return_array = []

    # Launch configuration variables
    node_log_type = LaunchConfiguration('node_log_type')

    svo_path = LaunchConfiguration('svo_path')
    publish_svo_clock = LaunchConfiguration('publish_svo_clock')

    enable_ipc = LaunchConfiguration('enable_ipc')
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

    ros_params_override_path = LaunchConfiguration('ros_params_override_path')
    object_detection_config_path = LaunchConfiguration('object_detection_config_path')
    custom_object_detection_config_path = LaunchConfiguration('custom_object_detection_config_path')

    serial_number = LaunchConfiguration('serial_number')
    camera_id = LaunchConfiguration('camera_id')

    serial_numbers = LaunchConfiguration('serial_numbers')
    camera_ids = LaunchConfiguration('camera_ids')

    publish_urdf = LaunchConfiguration('publish_urdf')
    publish_tf = LaunchConfiguration('publish_tf')
    publish_map_tf = LaunchConfiguration('publish_map_tf')
    publish_imu_tf = LaunchConfiguration('publish_imu_tf')
    xacro_path = LaunchConfiguration('xacro_path')

    enable_gnss = LaunchConfiguration('enable_gnss')
    gnss_antenna_offset = LaunchConfiguration('gnss_antenna_offset')

    node_log_type_val = node_log_type.perform(context)
    container_name_val = container_name.perform(context)
    namespace_val = namespace.perform(context)
    camera_name_val = camera_name.perform(context)
    camera_model_val = camera_model.perform(context)
    node_name_val = node_name.perform(context)
    enable_gnss_val = enable_gnss.perform(context)
    gnss_coords = parse_array_param(gnss_antenna_offset.perform(context))
    serial_numbers_val = serial_numbers.perform(context)
    camera_ids_val = camera_ids.perform(context)

    if(node_log_type_val == 'both'):
        node_log_effective = 'both'
    else:  # 'screen' or 'log'
        node_log_effective = {
            'stdout': node_log_type_val,
            'stderr': node_log_type_val
            }

    if (camera_name_val == ''):
        camera_name_val = 'zed'

    if (camera_model_val == 'virtual'):
        # Virtual Stereo Camera setup
        serials = parse_array_param(serial_numbers_val)
        ids = parse_array_param(camera_ids_val)

        # If not in live mode, at least one of serials or ids must be a valid 2-values array
        if(len(serials) != 2 and len(ids) != 2 and svo_path.perform(context) == 'live'):
            return [
                LogInfo(msg=TextSubstitution(
                    text='With a Virtual Stereo Camera setup, one of `serial_numbers` or `camera_ids` launch arguments must contain two valid values (Left and Right camera identification).'))
            ]
    
    if(namespace_val == ''):
        namespace_val = camera_name_val
    else:
        node_name_val = camera_name_val
    
    # Common configuration file
    if (camera_model_val == 'zed' or 
        camera_model_val == 'zedm' or 
        camera_model_val == 'zed2' or 
        camera_model_val == 'zed2i' or 
        camera_model_val == 'zedx' or 
        camera_model_val == 'zedxm' or
        camera_model_val == 'zedxhdr' or
        camera_model_val == 'zedxhdrmini' or
        camera_model_val == 'zedxhdrmax' or
        camera_model_val == 'virtual'):
        config_common_path_val = default_config_common + '_stereo.yaml'
    else:
        config_common_path_val = default_config_common + '_mono.yaml'

    info = 'Using common configuration file: ' + config_common_path_val
    return_array.append(LogInfo(msg=TextSubstitution(text=info)))

    # Camera configuration file
    config_camera_path = os.path.join(
        get_package_share_directory('zed_wrapper'),
        'config',
        camera_model_val + '.yaml'
    )

    info = 'Using camera configuration file: ' + config_camera_path
    return_array.append(LogInfo(msg=TextSubstitution(text=info)))

    # Object Detection configuration file
    info = 'Using Object Detection configuration file: ' + object_detection_config_path.perform(context)
    return_array.append(LogInfo(msg=TextSubstitution(text=info)))
    
    # Custom Object Detection configuration file
    info = 'Using Custom Object Detection configuration file: ' + custom_object_detection_config_path.perform(context)
    return_array.append(LogInfo(msg=TextSubstitution(text=info)))

    # ROS parameters override file
    ros_params_override_path_val = ros_params_override_path.perform(context)
    if(ros_params_override_path_val != ''):
        info = 'Using ROS parameters override file: ' + ros_params_override_path_val
        return_array.append(LogInfo(msg=TextSubstitution(text=info)))

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
        output=node_log_effective,
        parameters=[{
            'use_sim_time': publish_svo_clock,
            'robot_description': Command(xacro_command)
        }],
        remappings=[('robot_description', camera_name_val+'_description')]
    )
    return_array.append(rsp_node)

    # ROS 2 Component Container
    if(container_name_val == ''):
        container_name_val='zed_container'
        distro = os.environ['ROS_DISTRO']
        if distro == 'foxy':
            # Foxy does not support the isolated mode
            container_exec='component_container'
            arguments_val=['--ros-args', '--log-level', 'info']
        else:
            container_exec='component_container_isolated'
            arguments_val=['--use_multi_threaded_executor','--ros-args', '--log-level', 'info']
        
        zed_container = ComposableNodeContainer(
                name=container_name_val,
                namespace=namespace_val,
                package='rclcpp_components',
                executable=container_exec,
                arguments=arguments_val,
                output=node_log_effective,
                composable_node_descriptions=[]
        )
        return_array.append(zed_container)

    # ZED Node parameters
    node_parameters = [
            # YAML files
            config_common_path_val,  # Common parameters
            config_camera_path,  # Camera related parameters
            object_detection_config_path, # Object detection parameters
            custom_object_detection_config_path # Custom object detection parameters
    ]

    if( ros_params_override_path_val != ''):
        node_parameters.append(ros_params_override_path)

    node_parameters.append( 
            # Launch arguments must override the YAML files values
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
                'svo.publish_svo_clock': publish_svo_clock,
                'general.serial_number': serial_number,
                'general.camera_id': camera_id,
                'pos_tracking.publish_tf': publish_tf,
                'pos_tracking.publish_map_tf': publish_map_tf,
                'sensors.publish_imu_tf': publish_imu_tf,
                'gnss_fusion.gnss_fusion_enabled': enable_gnss,
                'general.virtual_serial_numbers': serial_numbers_val,
                'general.virtual_camera_ids': camera_ids_val
            }
    )

    # ZED Wrapper component
    if( camera_model_val=='zed' or
        camera_model_val=='zedm' or
        camera_model_val=='zed2' or
        camera_model_val=='zed2i' or
        camera_model_val=='zedx' or
        camera_model_val=='zedxm' or
        camera_model_val == 'zedxhdr' or
        camera_model_val == 'zedxhdrmini' or
        camera_model_val == 'zedxhdrmax' or
        camera_model_val=='virtual'):
        zed_wrapper_component = ComposableNode(
            package='zed_components',
            namespace=namespace_val,
            plugin='stereolabs::ZedCamera',
            name=node_name_val,
            parameters=node_parameters,
            extra_arguments=[{'use_intra_process_comms': enable_ipc}]
        )
    else: # camera_model_val == 'zedxonegs' or camera_model_val == 'zedxone4k' or camera_model_val == 'zedxonehdr'
        zed_wrapper_component = ComposableNode(
            package='zed_components',
            namespace=namespace_val,
            plugin='stereolabs::ZedCameraOne',
            name=node_name_val,
            parameters=node_parameters,
            extra_arguments=[{'use_intra_process_comms': enable_ipc}]
        )
    
    full_container_name = '/' + namespace_val + '/' + container_name_val
    info = 'Loading ZED node `' + node_name_val + '` in container `' + full_container_name + '`'
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
            # Declare launch arguments
            DeclareLaunchArgument(
                'node_log_type',
                default_value=TextSubstitution(text='both'),
                description='The log type of the node. It can be `screen`, `log` or `both`. The `log` type will save the log in a file in the `~/.ros/log/` folder. The `screen` type will print the log on the terminal. The `both` type will do both.',
                choices=['screen', 'log', 'both']),
            DeclareLaunchArgument(
                'camera_name',
                default_value=TextSubstitution(text='zed'),
                description='The name of the camera. It can be different from the camera model and it will be used as node `namespace`.'),
            DeclareLaunchArgument(
                'camera_model',
                description='[REQUIRED] The model of the camera. Using a wrong camera model can disable camera features.',
                choices=['zed', 'zedm', 'zed2', 'zed2i', 'zedx', 'zedxm', 'zedxhdr', 'zedxhdrmini', 'zedxhdrmax', 'virtual', 'zedxonegs', 'zedxone4k', 'zedxonehdr']),
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
                'ros_params_override_path',
                default_value='',
                description='The path to an additional parameters file to override the default values.'),
            DeclareLaunchArgument(
                'object_detection_config_path',
                default_value=TextSubstitution(text=default_object_detection_config_path),
                description='Path to the YAML configuration file for the Object Detection parameters.'),
            DeclareLaunchArgument(
                'custom_object_detection_config_path',
                default_value=TextSubstitution(text=default_custom_object_detection_config_path),
                description='Path to the YAML configuration file for the Custom Object Detection parameters.'),
            DeclareLaunchArgument(
                'serial_number',
                default_value='0',
                description='The serial number of the camera to be opened. It is mandatory to use this parameter or camera ID in multi-camera rigs to distinguish between different cameras. Use `ZED_Explorer -a` to retrieve the serial number of all the connected cameras.'),
            DeclareLaunchArgument(
                'serial_numbers',
                default_value='[]',
                description='The serial numbers of the two cameras to be opened to compose a Virtual Stereo Camera, [left_sn,right_sn]. Use `ZED_Explorer -a` to retrieve the serial number of all the connected cameras.'),
            DeclareLaunchArgument(
                'camera_id',
                default_value='-1',
                description='The ID of the camera to be opened. It is mandatory to use this parameter or serial number in multi-camera rigs to distinguish between different cameras.  Use `ZED_Explorer -a` to retrieve the ID of all the connected cameras.'),
            DeclareLaunchArgument(
                'camera_ids',
                default_value='[]',
                description='The IDs of the two cameras to be opened to compose a Virtual Stereo Camera, [left_id,right_id]. Use `ZED_Explorer -a` to retrieve the ID of all the connected cameras.'),
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
                default_value='false',
                description='Enable publication of the IMU TF. Note: Ignored if `publish_tf` is False.',
                choices=['true', 'false']),
            DeclareLaunchArgument(
                'xacro_path',
                default_value=TextSubstitution(text=default_xacro_path),
                description='Path to the camera URDF file as a xacro file.'),
            DeclareLaunchArgument(
                'svo_path',
                default_value=TextSubstitution(text='live'),
                description='Path to an input SVO file.'),
            DeclareLaunchArgument(
                'publish_svo_clock',
                default_value='false',
                description='If set to `true` the node will act as a clock server publishing the SVO timestamp. This is useful for node synchronization'),
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
                'enable_ipc',
                default_value='true',
                description='Enable intra-process communication (IPC) with ROS 2 Composition',
                choices=['true', 'false']),
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
            OpaqueFunction(function=launch_setup)
        ]
    )
