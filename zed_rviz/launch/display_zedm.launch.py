"""Launch a lifecycle ZED node and the Robot State Publisher"""

import os


import launch
from launch import LaunchIntrospector

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import EmitEvent
from launch.actions import LogInfo
from launch.actions import RegisterEventHandler
from launch_ros.actions import Node
from launch_ros.actions import LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition

import lifecycle_msgs.msg

def generate_launch_description():

    camera_model = 'zedm' 

    # URDF file to be loaded by Robot State Publisher
    urdf = os.path.join(get_package_share_directory('stereolabs_zed'), 'urdf', camera_model + '.urdf')
    
    # ZED Configurations to be loaded by ZED Node
    config_common = os.path.join(get_package_share_directory('stereolabs_zed'), 'config', 'common.yaml')

    config_camera = os.path.join(get_package_share_directory('stereolabs_zed'), 'config', camera_model + '.yaml')

    # RVIZ2 configuration
    rviz_config = os.path.join(get_package_share_directory('stereolabs_zed_rviz'), 'rviz', camera_model + '.rviz')
    sl_logo = os.path.join(get_package_share_directory('stereolabs_zed_rviz'), 'rviz', 'Logo_STEREOLABS.png')

    # Set LOG format
    os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = '{time}: [{name}] [{severity}]\t{message}'

    # Launch Description
    ld = launch.LaunchDescription()

    # Prepare the ZED node
    zed_node = LifecycleNode(
        node_namespace = 'zed',        # must match the namespace in config -> YAML
        node_name = 'zed_node',        # must match the node name in config -> YAML
        package = 'stereolabs_zed',
        node_executable = 'zed_wrapper_node',
        output = 'screen',
        parameters = [
            config_common,  # Common parameters
            config_camera,  # Camera related parameters
        ]
    )

    # Prepare the Robot State Publisher node
    rsp_node = Node(
        node_name = 'zed_state_publisher',
        package = 'robot_state_publisher',
        node_executable = 'robot_state_publisher',
        output = 'screen',
        arguments = [urdf, 'robot_description:=zed_description']
    )

    # Prepare the RVIZ2 node
    rviz2_node = Node(
        node_name = 'rviz2',
        package = 'rviz2',
        node_executable = 'rviz2',
        output = 'screen',
        arguments = ['-d', rviz_config,
                     '-s', sl_logo]
    )

    # Make the ZED node take the 'configure' transition
    zed_configure_trans_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher = launch.events.process.matches_action(zed_node),
            transition_id = lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    # Make the ZED node take the 'activate' transition
    zed_activate_trans_event = EmitEvent(
        event = ChangeState(
            lifecycle_node_matcher = launch.events.process.matches_action(zed_node),
            transition_id = lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
         )
    )

    # Shutdown event
    shutdown_event = EmitEvent( event = launch.events.Shutdown() )

    # When the ZED node reaches the 'inactive' state from 'unconfigured', make it take the 'activate' transition and start the Robot State Publisher
    zed_inactive_from_unconfigured_state_handler = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node = zed_node,
            start_state = 'configuring',
            goal_state = 'inactive',
            entities = [
                # Log
                LogInfo( msg = "'ZED' reached the 'INACTIVE' state, start the 'Robot State Publisher' node and 'activating'." ),
                # Robot State Publisher
                rsp_node,
                # Change State event ( inactive -> active )
                zed_activate_trans_event,
            ],
        )
    )

    # When the ZED node reaches the 'inactive' state from 'active', it has been deactivated and it will wait for a manual activation
    zed_inactive_from_active_state_handler = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node = zed_node,
            start_state = 'deactivating',
            goal_state = 'inactive',
            entities = [
                # Log
                LogInfo( msg = "'ZED' reached the 'INACTIVE' state from 'ACTIVE' state. Waiting for manual activation..." )
            ],
        )
    )

    # When the ZED node reaches the 'active' state, log a message and start RVIZ2
    zed_active_state_handler = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node = zed_node,
            goal_state = 'active',
            entities = [
                # Log
                LogInfo( msg = "'ZED' reached the 'ACTIVE' state." ),
                # Log
                LogInfo( msg = "Starting RVIZ2 with the configuration " + rviz_config ),
                # RVIZ2
                rviz2_node,
            ],
        )
    )

    # When the ZED node reaches the 'finalized' state, log a message and exit.
    zed_finalized_state_handler = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node = zed_node,
            goal_state = 'finalized',
            entities = [
                # Log
                LogInfo( msg = "'ZED' reached the 'FINALIZED' state. Killing the node..." ),
                shutdown_event,
            ],
        )
    )

    # Add the actions to the launch description.
    # The order they are added reflects the order in which they will be executed.
    ld.add_action( zed_inactive_from_unconfigured_state_handler )
    ld.add_action( zed_inactive_from_active_state_handler )
    ld.add_action( zed_active_state_handler )
    ld.add_action( zed_finalized_state_handler )
    ld.add_action( zed_node )
    ld.add_action( zed_configure_trans_event)

    return ld
