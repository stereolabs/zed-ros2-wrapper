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

from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    IncludeLaunchDescription,
    LogInfo
)
from launch.substitutions import (
    LaunchConfiguration,
    Command,
    TextSubstitution
)
from launch_ros.actions import (
    Node,
    ComposableNodeContainer
)

def parse_array_param(param):
    str = param.replace('[', '')
    str = str.replace(']', '')
    str = str.replace(' ', '')
    arr = str.split(',')

    return arr

def launch_setup(context, *args, **kwargs):

    # List of actions to be launched
    actions = []

    namespace_val = 'zed_multi'
    
    # URDF/xacro file to be loaded by the Robot State Publisher node
    multi_zed_xacro_path = os.path.join(
    get_package_share_directory('zed_wrapper'),
    'urdf',
    'zed_quad.urdf.xacro')

    names = LaunchConfiguration('cam_names')
    models = LaunchConfiguration('cam_models')
    serials = LaunchConfiguration('cam_serials')
    ids = LaunchConfiguration('cam_ids')
    side_camera_names = LaunchConfiguration('side_camera_names')

    disable_tf = LaunchConfiguration('disable_tf')

    names_arr = parse_array_param(names.perform(context))
    models_arr = parse_array_param(models.perform(context))
    serials_arr = parse_array_param(serials.perform(context))
    ids_arr = parse_array_param(ids.perform(context))
    disable_tf_val = disable_tf.perform(context)
    side_camera_names_val = side_camera_names.perform(context)
    
    num_cams = len(names_arr)

    if (num_cams != len(models_arr)):
        return [
            LogInfo(msg=TextSubstitution(
                text='The `cam_models` array argument must match the size of the `cam_names` array argument.'))
        ]

    if ((num_cams != len(serials_arr)) and (num_cams != len(ids_arr))):
        return [
            LogInfo(msg=TextSubstitution(
                text='The `cam_serials` or `cam_ids` array argument must match the size of the `cam_names` array argument.'))
        ]
    
    info = 'Using side_camera_names: ' + side_camera_names_val
    actions.append(LogInfo(msg=TextSubstitution(text=info)))

    # ROS 2 Component Container
    container_name = 'zed_multi_container'
    distro = os.environ['ROS_DISTRO']
    if distro == 'foxy':
        # Foxy does not support the isolated mode
        container_exec='component_container'
    else:
        container_exec='component_container_isolated'
    
    info = '* Starting Composable node container: /' + namespace_val + '/' + container_name
    actions.append(LogInfo(msg=TextSubstitution(text=info)))

    zed_container = ComposableNodeContainer(
        name=container_name,
        namespace=namespace_val,
        package='rclcpp_components',
        executable=container_exec,
        arguments=['--ros-args', '--log-level', 'info'],
        output='screen',
    )
    actions.append(zed_container)

    # Set the first camera idx
    cam_idx = 0

    for name in names_arr:
        model = models_arr[cam_idx]
        if len(serials_arr) == num_cams:
            serial = serials_arr[cam_idx]
        else:
            serial = '0'

        if len(ids_arr) == num_cams:
            id = ids_arr[cam_idx]
        else:
            id = '-1'
        
        pose = '['

        info = '* Starting a ZED ROS2 node for camera ' + name + \
            ' (' + model        
        if(serial != '0'):
            info += ', serial: ' + serial
        elif( id!= '-1'):
            info += ', id: ' + id
        info += ')'

        actions.append(LogInfo(msg=TextSubstitution(text=info)))

        # Only the first camera send odom and map TF
        publish_tf = 'false'
        if (cam_idx == 0):
            if (disable_tf_val == 'False' or disable_tf_val == 'false'):
                publish_tf = 'true'

        # A different node name is required by the Diagnostic Updated
        node_name = 'zed_node_' + str(cam_idx)

        if(name in side_camera_names_val):
            use_overide = 'true'
        else:
            use_overide = 'false'

        # Add the node
        # ZED Wrapper launch file
        zed_wrapper_launch = IncludeLaunchDescription(
            launch_description_source=PythonLaunchDescriptionSource([
                get_package_share_directory('zed_wrapper'),
                '/launch/zed_camera.launch.py'
            ]),
            launch_arguments={
                'container_name': container_name,
                'camera_name': name,
                'camera_model': model,
                'serial_number': serial,
                'camera_id': id,
                'publish_tf': publish_tf,
                'publish_map_tf': publish_tf,
                'namespace': namespace_val,
                'use_overide': use_overide
            }.items()
        )
        actions.append(zed_wrapper_launch)

        cam_idx += 1

    # Create the Xacro command with correct camera names
    xacro_command = []
    xacro_command.append('xacro')
    xacro_command.append(' ')
    xacro_command.append(multi_zed_xacro_path)
    xacro_command.append(' ')
    cam_idx = 0
    for name in names_arr:
        xacro_command.append('camera_name_'+str(cam_idx)+':=')
        xacro_command.append(name)
        xacro_command.append(' ')
        cam_idx+=1

    # Robot State Publisher node
    # this will publish the static reference link for a multi-camera configuration
    # and all the joints. See 'urdf/zed_dual.urdf.xacro' as an example    
    rsp_name = 'state_publisher'
    info = '* Starting robot_state_publisher node to link all the frames: ' + rsp_name
    actions.append(LogInfo(msg=TextSubstitution(text=info)))
    multi_rsp_node = Node(
        package='robot_state_publisher',
        namespace=namespace_val,
        executable='robot_state_publisher',
        name=rsp_name,
        output='screen',
        parameters=[{
            'robot_description': Command(xacro_command).perform(context)
        }]
    )

    # Add the robot_state_publisher node to the list of nodes to be started
    actions.append(multi_rsp_node)

    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'cam_names',
                description='An array containing the name of the cameras, e.g. [zed_front,zed_back]'),
            DeclareLaunchArgument(
                'cam_models',
                description='An array containing the model of the cameras, e.g. [zed2i,zed2]'),
            DeclareLaunchArgument(
                'cam_serials',
                default_value=[],
                description='An array containing the serial number of the cameras, e.g. [35199186,23154724]'),
            DeclareLaunchArgument(
                'cam_ids',
                default_value=[],
                description='An array containing the ID number of the cameras, e.g. [0,1]'),
            DeclareLaunchArgument(
                'disable_tf',
                default_value='False',
                description='If `True` disable TF broadcasting for all the cameras in order to fuse visual odometry information externally.'),
            DeclareLaunchArgument(
                'side_camera_names',
                default_value='[]',
                description='An array containing names of side cameras that need special override config, e.g. [zed2,zed3]'),
            OpaqueFunction(function=launch_setup)
        ]
    )