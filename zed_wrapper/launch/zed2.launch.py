#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    # Launch configuration variables
    svo_path = LaunchConfiguration('svo_path')

    # use:
    #  - 'zed' for "ZED" camera
    #  - 'zedm' for "ZED mini" camera
    #  - 'zed2' for "ZED2" camera
    #  - 'zed2i' for "ZED2i" camera
    camera_model = 'zed2'

    # Camera name
    camera_name = 'zed2'

    # Zed Node name
    node_name = 'zed_node'

    # Publish URDF
    publish_urdf = 'true'

    # ZED Configurations to be loaded by ZED Node
    config_common_path = os.path.join(
        get_package_share_directory('zed_wrapper'),
        'config',
        'common.yaml'
    )

    if(camera_model != 'zed'):
        config_camera_path = os.path.join(
            get_package_share_directory('zed_wrapper'),
            'config',
            camera_model + '.yaml'
        )
    else:
        config_camera_path = ''

    # URDF/xacro file to be loaded by the Robot State Publisher node
    xacro_path = os.path.join(
        get_package_share_directory('zed_wrapper'),
        'urdf', 'zed_descr.urdf.xacro'
    )

    # ZED Wrapper node
    zed_wrapper_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource([
            get_package_share_directory('zed_wrapper'),
            '/launch/include/zed_camera.launch.py'
        ]),
        launch_arguments={
            'camera_model': camera_model,
            'camera_name': camera_name,
            'node_name': node_name,
            'config_common_path': config_common_path,
            'config_camera_path': config_camera_path,
            'publish_urdf': publish_urdf,
            'xacro_path': xacro_path,
            'svo_path': svo_path
        }.items()
    )

    declare_svo_path_cmd = DeclareLaunchArgument(
        'svo_path',
        default_value='',
        description='Path to an input SVO file. Note: overrides the parameter `general.svo_file` in `common.yaml`.')

    # Define LaunchDescription variable
    ld = LaunchDescription()

    # Launch parameters
    ld.add_action(declare_svo_path_cmd)

    # Add nodes to LaunchDescription
    ld.add_action(zed_wrapper_launch)

    return ld
