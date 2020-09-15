import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro


def generate_launch_description():
    # use:
    #  - 'zed' for "ZED" camera
    #  - 'zedm' for "ZED mini" camera
    #  - 'zed2' for "ZED2" camera
    camera_model = 'zed2'

    # URDF file to be loaded by Robot State Publisher
    urdf = os.path.join(
        get_package_share_directory('zed_wrapper'),
        'urdf', camera_model + '.urdf'
    )

    # ZED Configurations to be loaded by ZED Node
    config_common = os.path.join(
        get_package_share_directory('zed_wrapper'),
        'params', 'common.yaml'
    )

    config_camera = os.path.join(
        get_package_share_directory('zed_wrapper'),
        'params', camera_model + '.yaml'
    )

    # Set LOG format
    os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = '{time}: [{name}] [{severity}] - {message}'

    return LaunchDescription([
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            node_namespace=camera_model,
            node_executable='robot_state_publisher',
            node_name=camera_model+'_state_publisher',
            output='screen',
            arguments=[urdf],
        ),
        Node(
            package='zed_wrapper',
            node_namespace=camera_model,
            node_executable='zed_wrapper',
            node_name='zed_node',
            output='screen',
            parameters=[
                config_common,  # Common parameters
                config_camera,  # Camera related parameters
            ],
        )
    ])
