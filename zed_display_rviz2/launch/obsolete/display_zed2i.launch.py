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
    IncludeLaunchDescription
)
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    TextSubstitution
)
from launch_ros.actions import Node

# Set LOG format
os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = '{time} [{name}] [{severity}] {message}'

# Camera model (force value)
camera_model = 'zed2i'


def launch_setup(context, *args, **kwargs):

    start_zed_node = LaunchConfiguration('start_zed_node')
    camera_name = LaunchConfiguration('camera_name')

    camera_name_val = camera_name.perform(context)

    if (camera_name_val == ''):
        camera_name_val = camera_model

    # Rviz2 Configurations to be loaded by ZED Node
    config_rviz2 = os.path.join(
        get_package_share_directory('zed_display_rviz2'),
        'rviz2',
        camera_name_val + '.rviz'
    )

    # Rviz2 node
    rviz2_node = Node(
        package='rviz2',
        namespace=camera_name,
        executable='rviz2',
        name=camera_name_val+'_rviz2',
        output='screen',
        arguments=[['-d'], [config_rviz2]],
    )

    # ZED Wrapper launch file
    zed_wrapper_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource([
            get_package_share_directory('zed_wrapper'),
            '/launch/' + camera_model + '.launch.py'
        ]),
        launch_arguments={
            'camera_name': camera_name_val
        }.items(),
        condition=IfCondition(start_zed_node)
    )

    return [
        rviz2_node,
        zed_wrapper_launch
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'start_zed_node',
                default_value='True',
                description='Set to `False` to start only Rviz2 if a ZED node is already running.'),
            DeclareLaunchArgument(
                'camera_name',
                default_value=TextSubstitution(text=''),
                description='The name of the camera. It can be different from the camera model and it will be used as node `namespace`. Leave empty to use the camera model as camera name.'),
            OpaqueFunction(function=launch_setup)
        ]
    )
