import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import LifecycleNode

def generate_launch_description():

    # use: 'zed' for "ZED" camera - 'zedm' for "ZED mini" camera
    camera_model = 'zed' 

    # URDF file to be loaded by Robot State Publisher
    urdf = os.path.join(
        get_package_share_directory('stereolabs_zed'), 
            'urdf', camera_model + '.urdf'
    )
    
    # ZED Configurations to be loaded by ZED Node
    config_common = os.path.join(
        get_package_share_directory( 'stereolabs_zed' ), 
        'config', 'common.yaml'
    )

    config_camera = os.path.join(
        get_package_share_directory('stereolabs_zed'), 
        'config', camera_model + '.yaml'
    )

    # Set LOG format
    os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = '{time}: [{name}] [{severity}] - {message}'

    return LaunchDescription( [
        # Robot State Publisher
        Node(
            node_namespace='zed',
            node_name='zed_state_publisher',
            package='robot_state_publisher',
            node_executable='robot_state_publisher',
            output='screen',
            arguments=[urdf],
        ),

        # ZED
        LifecycleNode(
            node_namespace='zed',        # must match the namespace in config -> YAML
            node_name='zed_node',        # must match the node name in config -> YAML
            package='stereolabs_zed',
            node_executable='zed_wrapper_node',
            output='screen',
            parameters=[
                config_common,  # Common parameters
                config_camera,  # Camera related parameters
            ],
        ),
    ])
