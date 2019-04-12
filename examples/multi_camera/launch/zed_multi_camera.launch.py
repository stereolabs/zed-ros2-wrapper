"""Launch a lifecycle ZED node and the Robot State Publisher"""

import os
import sys

import launch

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import EmitEvent
from launch.actions import LogInfo
from launch.actions import RegisterEventHandler
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
from launch_ros.actions import LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition

import lifecycle_msgs.msg

def generate_launch_description():

    # Launch Description
    ld = launch.LaunchDescription()

    # Set LOG format
    os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = '{time}: [{name}] [{severity}]\t{message}'

    ################ ZED ################

    # ZED URDF file to be loaded by Robot State Publisher
    zed_urdf = os.path.join(
        get_package_share_directory('stereolabs_example_multi_camera'),
            'urdf', 'zed.urdf'
    )

    # ZED Configurations to be loaded by ZED Node
    zed_cfg_camera = os.path.join(
        get_package_share_directory('stereolabs_example_multi_camera'),
        'config', 'zed.yaml'
    )

    # Prepare the ZED node
    zed_node = LifecycleNode(
        node_namespace = 'zed',        # must match the namespace in config -> YAML
        node_name = 'zed_node',        # must match the node name in config -> YAML
        package = 'stereolabs_zed',
        node_executable = 'zed_wrapper_node',
        output = 'screen',
        parameters = [
            zed_cfg_camera,  # Camera related parameters
        ]
    )

    # Prepare the Robot State Publisher node for the ZED URDF
    zed_rsp_node = Node(
        node_name = 'zed_state_publisher',
        package = 'robot_state_publisher',
        node_executable = 'robot_state_publisher',
        output = 'screen',
        arguments = [zed_urdf, 'robot_description:=zed_description']
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
                zed_rsp_node,
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

    # When the ZED node reaches the 'active' state, log a message.
    zed_active_state_handler = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node = zed_node,
            goal_state = 'active',
            entities = [
                # Log
                LogInfo( msg = "'ZED' reached the 'ACTIVE' state" ),
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

    ################ ZED-M ################

    # ZED-M URDF file to be loaded by Robot State Publisher
    zedm_urdf = os.path.join(
        get_package_share_directory('stereolabs_example_multi_camera'),
            'urdf', 'zedm.urdf'
    )

    # ZED-M Configurations to be loaded by ZED Node
    zedm_cfg_camera = os.path.join(
        get_package_share_directory('stereolabs_example_multi_camera'),
        'config', 'zedm.yaml'
    )

    # Prepare the ZED-M node
    zedm_node = LifecycleNode(
        node_namespace = 'zed',        # must match the namespace in config -> YAML
        node_name = 'zedm_node',       # must match the node name in config -> YAML
        package = 'stereolabs_zed',
        node_executable = 'zed_wrapper_node',
        output = 'screen',
        parameters = [
            zedm_cfg_camera,  # Camera related parameters
        ]
    )

    # Prepare the Robot State Publisher node for the ZED URDF
    zedm_rsp_node = Node(
        node_name = 'zedm_state_publisher',
        package = 'robot_state_publisher',
        node_executable = 'robot_state_publisher',
        output = 'screen',
        arguments = [zedm_urdf, 'robot_description:=zedm_description']
    )

    # Make the ZED node take the 'configure' transition
    zedm_configure_trans_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher = launch.events.process.matches_action(zedm_node),
            transition_id = lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    # Make the ZED node take the 'activate' transition
    zedm_activate_trans_event = EmitEvent(
        event = ChangeState(
            lifecycle_node_matcher = launch.events.process.matches_action(zedm_node),
            transition_id = lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
         )
    )

    # Shutdown event
    shutdown_event = EmitEvent( event = launch.events.Shutdown() )

    # When the ZED-M node reaches the 'inactive' state from 'unconfigured', make it take the 'activate' transition and start the Robot State Publisher
    zedm_inactive_from_unconfigured_state_handler = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node = zedm_node,
            start_state = 'configuring',
            goal_state = 'inactive',
            entities = [
                # Log
                LogInfo( msg = "'ZED-M' reached the 'INACTIVE' state, start the 'Robot State Publisher' node and 'activating'." ),
                # Robot State Publisher
                zedm_rsp_node,
                # Change State event ( inactive -> active )
                zedm_activate_trans_event,
            ],
        )
    )

    # When the ZED node reaches the 'inactive' state from 'active', it has been deactivated and it will wait for a manual activation
    zedm_inactive_from_active_state_handler = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node = zedm_node,
            start_state = 'deactivating',
            goal_state = 'inactive',
            entities = [
                # Log
                LogInfo( msg = "'ZED-M' reached the 'INACTIVE' state from 'ACTIVE' state. Waiting for manual activation..." )
            ],
        )
    )

    # When the ZED-M node reaches the 'active' state, log a message.
    zedm_active_state_handler = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node = zed_node,
            goal_state = 'active',
            entities = [
                # Log
                LogInfo( msg = "'ZED-M' reached the 'ACTIVE' state" ),
            ],
        )
    )

    # When the ZED-M node reaches the 'finalized' state, log a message and exit.
    zedm_finalized_state_handler = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node = zedm_node,
            goal_state = 'finalized',
            entities = [
                # Log
                LogInfo( msg = "'ZED-M' reached the 'FINALIZED' state. Killing the node..." ),
                shutdown_event,
            ],
        )
    )

    ######################################################

    # Add the actions to the launch description.
    # The order they are added reflects the order in which they will be executed.
    ld.add_action( zed_inactive_from_unconfigured_state_handler )
    ld.add_action( zed_inactive_from_active_state_handler )
    ld.add_action( zed_active_state_handler )
    ld.add_action( zed_finalized_state_handler )
    ld.add_action( zed_node )
    ld.add_action( zed_configure_trans_event)

    ld.add_action( zedm_inactive_from_unconfigured_state_handler )
    ld.add_action( zedm_inactive_from_active_state_handler )
    ld.add_action( zedm_active_state_handler )
    ld.add_action( zedm_finalized_state_handler )
    ld.add_action( zedm_node )
    ld.add_action( zedm_configure_trans_event)

    return ld
