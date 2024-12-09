# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2022-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# SPDX-License-Identifier: Apache-2.0

import launch
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, EnvironmentVariable
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch import LaunchContext

import yaml


def launch_setup(context, *args, **kwargs):
    encoder_config = LaunchConfiguration("encoder_config")
    ros_params_override_path = LaunchConfiguration("ros_params_override_path")

    with open(ros_params_override_path.perform(context), "r") as file:
        ros_params_override_config = yaml.safe_load(file)
        datahub_name = ros_params_override_config["/**"]["ros__parameters"]["general"]["datahub_name"]
        camera_name = ros_params_override_config["/**"]["ros__parameters"]["general"]["camera_name"]
        resolution_ID = ros_params_override_config["/**"]["ros__parameters"]["general"][
            "grab_resolution"
        ]  # Make sure pub_resolution=NATIVE
        frame_rate = ros_params_override_config["/**"]["ros__parameters"]["general"]["pub_frame_rate"]

    if resolution_ID == "HD2K":
        input_height = 1440
        input_width = 2560
    elif resolution_ID == "HD1080":
        input_height = 1080
        input_width = 1920
    elif resolution_ID == "HD720":
        input_height = 720
        input_width = 1280
    elif resolution_ID == "VGA":
        input_height = 480
        input_width = 640
    else:
        raise ValueError(f"Invalid resolution_ID {resolution_ID}")

    zed_left_raw_topic = PathJoinSubstitution([datahub_name, camera_name, "left", "image_rect_color"])
    zed_left_compressed_topic = PathJoinSubstitution([datahub_name, camera_name, "left", "image_rect_color", "h264"])
    zed_right_raw_topic = PathJoinSubstitution([datahub_name, camera_name, "right", "image_rect_color"])
    zed_right_compressed_topic = PathJoinSubstitution([datahub_name, camera_name, "right", "image_rect_color", "h264"])

    encoder_node_left = ComposableNode(
        name="encoder_zed_rgb_left",
        package="isaac_ros_h264_encoder",
        plugin="nvidia::isaac_ros::h264_encoder::EncoderNode",
        parameters=[
            {
                "input_height": input_height,
                "input_width": input_width,
                "config": encoder_config,
                "iframe_interval": int(frame_rate),
                "use_intra_process_comms": True,
            }
        ],
        remappings=[("image_raw", zed_left_raw_topic), ("image_compressed", zed_left_compressed_topic)],
    )

    encoder_node_right = ComposableNode(
        name="encoder_zed_rgb_right",
        package="isaac_ros_h264_encoder",
        plugin="nvidia::isaac_ros::h264_encoder::EncoderNode",
        parameters=[
            {
                "input_height": input_height,
                "input_width": input_width,
                "config": encoder_config,
                "iframe_interval": int(frame_rate),
                "use_intra_process_comms": True,
            }
        ],
        remappings=[("image_raw", zed_right_raw_topic), ("image_compressed", zed_right_compressed_topic)],
    )

    container = ComposableNodeContainer(
        namespace="",
        name="nitros_container",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[encoder_node_left, encoder_node_right],
        output="screen",
        arguments=["--ros-args", "--log-level", "info"],
    )

    return [container]


def generate_launch_description():
    """Launch the H.264 Encoder Node."""
    return launch.LaunchDescription(
        [
            DeclareLaunchArgument("encoder_config", default_value="custom", description="Config of encoder"),
            DeclareLaunchArgument(
                "ros_params_override_path",
                default_value="/zed_mini_ros_config.yaml",
                description="The path to an additional parameters file to override the defaults",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
