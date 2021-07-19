#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    # use:
    #  - 'zed' for "ZED" camera
    #  - 'zedm' for "ZED mini" camera
    #  - 'zed2' for "ZED2" camera
    #  - 'zed2i' for "ZED2i" camera
    camera_model = 'zed'

    # Camera name
    camera_name = 'zed'

    # Publish URDF
    publish_urdf = 'true'

    # ZED Configurations to be loaded by ZED Node
    config_common_path = os.path.join(
        get_package_share_directory('zed_wrapper'),
        'config',
        'common.yaml'
    )

    config_camera_path = os.path.join(
        get_package_share_directory('zed_wrapper'),
        'config',
        camera_model + '.yaml'
    )

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
            'config_common_path': config_common_path,
            'config_camera_path': config_camera_path,
            'publish_urdf': publish_urdf,
            'xacro_path': xacro_path

        }.items()
    )

    # Define LaunchDescription variable
    ld = LaunchDescription()

    # Add nodes to LaunchDescription
    ld.add_action(zed_wrapper_launch)

    return ld
