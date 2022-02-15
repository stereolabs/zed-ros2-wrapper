#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():

    wrapper_dir = get_package_share_directory('zed_wrapper')

    # Launch configuration variables
    svo_path = LaunchConfiguration('svo_path')

    camera_name = LaunchConfiguration('camera_name')
    camera_model = LaunchConfiguration('camera_model')

    node_name = LaunchConfiguration('node_name')

    config_common_path = LaunchConfiguration('config_common_path')
    config_camera_path = LaunchConfiguration('config_camera_path')

    base_frame = LaunchConfiguration('base_frame')
    cam_pos_x = LaunchConfiguration('cam_pos_x')
    cam_pos_y = LaunchConfiguration('cam_pos_y')
    cam_pos_z = LaunchConfiguration('cam_pos_z')
    cam_roll = LaunchConfiguration('cam_roll')
    cam_pitch = LaunchConfiguration('cam_pitch')
    cam_yaw = LaunchConfiguration('cam_yaw')

    publish_urdf = LaunchConfiguration('publish_urdf')
    xacro_path = LaunchConfiguration('xacro_path')

    # ZED Configurations to be loaded by ZED Node
    default_config_common = os.path.join(
        get_package_share_directory('zed_wrapper'),
        'config',
        'common.yaml'
    )

    # URDF/xacro file to be loaded by the Robot State Publisher node
    default_xacro_path = os.path.join(
        get_package_share_directory('zed_wrapper'),
        'urdf',
        'zed_descr.urdf.xacro'
    )

    # Declare the launch arguments
    declare_camera_name_cmd = DeclareLaunchArgument(
        'camera_name',
        description='The name of the camera. It can be different from the camera model and it will be used as node `namespace`.')

    declare_camera_model_cmd = DeclareLaunchArgument(
        'camera_model',
        description='The model of the camera. Using a wrong camera model can disable camera features. Valid models: `zed`, `zedm`, `zed2`, `zed2i`.')

    declare_node_name_cmd = DeclareLaunchArgument(
        'node_name',
        default_value='zed_node',
        description='The name of the zed_wrapper node. All the topic will have the same prefix: `/<camera_name>/<node_name>/`')

    declare_config_common_path_cmd = DeclareLaunchArgument(
        'config_common_path',
        default_value=default_config_common,
        description='Path to the `common.yaml` file.')

    declare_config_camera_path_cmd = DeclareLaunchArgument(
        'config_camera_path',
        description='Path to the `<camera_model>.yaml` file.')

    declare_publish_urdf_cmd = DeclareLaunchArgument(
        'publish_urdf',
        default_value='true',
        description='Enable URDF processing and starts Robot State Published to propagate static TF.')

    declare_xacro_path_cmd = DeclareLaunchArgument(
        'xacro_path',
        default_value=default_xacro_path,
        description='Path to the camera URDF file as a xacro file.')

    declare_svo_path_cmd = DeclareLaunchArgument(
        'svo_path',
        default_value='live', # 'live' used as patch for launch files not allowing empty strings as default parameters
        description='Path to an input SVO file. Note: overrides the parameter `general.svo_file` in `common.yaml`.')

    declare_base_frame_cmd = DeclareLaunchArgument(
        'base_frame',
        default_value='base_link',
        description='Name of the base link.')

    declare_pos_x_cmd = DeclareLaunchArgument(
        'cam_pos_x',
        default_value='0.0',
        description='Position X of the camera with respect to the base frame.')

    declare_pos_y_cmd = DeclareLaunchArgument(
        'cam_pos_y',
        default_value='0.0',
        description='Position Y of the camera with respect to the base frame.')

    declare_pos_z_cmd = DeclareLaunchArgument(
        'cam_pos_z',
        default_value='0.0',
        description='Position Z of the camera with respect to the base frame.')

    declare_roll_cmd = DeclareLaunchArgument(
        'cam_roll',
        default_value='0.0',
        description='Roll orientation of the camera with respect to the base frame.')

    declare_pitch_cmd = DeclareLaunchArgument(
        'cam_pitch',
        default_value='0.0',
        description='Pitch orientation of the camera with respect to the base frame.')

    declare_yaw_cmd = DeclareLaunchArgument(
        'cam_yaw',
        default_value='0.0',
        description='Yaw orientation of the camera with respect to the base frame.')

    # Robot State Publisher node
    rsp_node = Node(
        condition=IfCondition(publish_urdf),
        package='robot_state_publisher',
        namespace=camera_name,
        executable='robot_state_publisher',
        name='zed_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(
                [
                    'xacro', ' ', xacro_path, ' ',
                    'camera_name:=', camera_name, ' ',
                    'camera_model:=', camera_model, ' ',
                    'base_frame:=', base_frame, ' ',
                    'cam_pos_x:=', cam_pos_x, ' ',
                    'cam_pos_y:=', cam_pos_y, ' ',
                    'cam_pos_z:=', cam_pos_z, ' ',
                    'cam_roll:=', cam_roll, ' ',
                    'cam_pitch:=', cam_pitch, ' ',
                    'cam_yaw:=', cam_yaw
                ])
        }]
    )

    # Set LOG format
    os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = '{time} [{name}] [{severity}] {message}'

    # ZED Wrapper node
    zed_wrapper_node = Node(
        package='zed_wrapper',        
        namespace=camera_name,
        executable='zed_wrapper',
        name=node_name,
        output='screen',
        parameters=[
            # YAML files
            config_common_path,  # Common parameters
            config_camera_path,  # Camera related parameters
            # Overriding
            {
                 'general.camera_name': camera_name,
                 'general.camera_model': camera_model,
                 'general.svo_file': svo_path,
                 'pos_tracking.base_frame': base_frame
            }
        ]
    )

    # Define LaunchDescription variable and return it
    ld = LaunchDescription()

    ld.add_action(declare_camera_name_cmd)
    ld.add_action(declare_camera_model_cmd)
    ld.add_action(declare_node_name_cmd)
    ld.add_action(declare_publish_urdf_cmd)
    ld.add_action(declare_config_common_path_cmd)
    ld.add_action(declare_config_camera_path_cmd)
    ld.add_action(declare_xacro_path_cmd)
    ld.add_action(declare_svo_path_cmd)
    ld.add_action(declare_base_frame_cmd)
    ld.add_action(declare_pos_x_cmd)
    ld.add_action(declare_pos_y_cmd)
    ld.add_action(declare_pos_z_cmd)
    ld.add_action(declare_roll_cmd)
    ld.add_action(declare_pitch_cmd)
    ld.add_action(declare_yaw_cmd)

    ld.add_action(rsp_node)
    ld.add_action(zed_wrapper_node)

    return ld
