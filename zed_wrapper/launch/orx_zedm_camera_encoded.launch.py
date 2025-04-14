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
import sys

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command, TextSubstitution, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

import yaml

# ZED Configurations to be loaded by ZED Node
default_config_common = os.path.join(get_package_share_directory("zed_wrapper"), "config", "common.yaml")

# FFMPEG Configuration to be loaded by ZED Node
default_config_ffmpeg = os.path.join(get_package_share_directory("zed_wrapper"), "config", "ffmpeg.yaml")

# URDF/xacro file to be loaded by the Robot State Publisher node
default_xacro_path = os.path.join(get_package_share_directory("zed_wrapper"), "urdf", "zed_descr.urdf.xacro")


def parse_array_param(param):
    str = param.replace("[", "")
    str = str.replace("]", "")
    arr = str.split(",")

    return arr


def launch_setup(context, *args, **kwargs):
    wrapper_dir = get_package_share_directory("zed_wrapper")

    # Launch configuration variables
    svo_path = LaunchConfiguration("svo_path")

    use_sim_time = LaunchConfiguration("use_sim_time")
    sim_mode = LaunchConfiguration("sim_mode")
    sim_address = LaunchConfiguration("sim_address")
    sim_port = LaunchConfiguration("sim_port")

    stream_address = LaunchConfiguration("stream_address")
    stream_port = LaunchConfiguration("stream_port")

    camera_name = LaunchConfiguration("camera_name")
    camera_model = LaunchConfiguration("camera_model")

    node_name = LaunchConfiguration("node_name")

    config_common_path = LaunchConfiguration("config_path")
    config_ffmpeg = LaunchConfiguration("ffmpeg_config_path")

    serial_number = LaunchConfiguration("serial_number")

    publish_urdf = LaunchConfiguration("publish_urdf")
    publish_tf = LaunchConfiguration("publish_tf")
    publish_map_tf = LaunchConfiguration("publish_map_tf")
    publish_imu_tf = LaunchConfiguration("publish_imu_tf")
    xacro_path = LaunchConfiguration("xacro_path")

    custom_baseline = LaunchConfiguration("custom_baseline")

    encoder_config = LaunchConfiguration("encoder_config")
    ros_params_override_path = LaunchConfiguration("ros_params_override_path")

    enable_gnss = LaunchConfiguration("enable_gnss")
    gnss_antenna_offset = LaunchConfiguration("gnss_antenna_offset")

    camera_name_val = camera_name.perform(context)
    camera_model_val = camera_model.perform(context)
    enable_gnss_val = enable_gnss.perform(context)
    gnss_coords = parse_array_param(gnss_antenna_offset.perform(context))
    custom_baseline_val = custom_baseline.perform(context)

    if camera_name_val == "":
        camera_name_val = "zed"

    if camera_model_val == "virtual" and float(custom_baseline_val) <= 0:
        return [
            LogInfo(
                msg="Please set a positive value for the 'custom_baseline' argument when using a 'virtual' Stereo Camera with two ZED X One devices."
            ),
        ]

    config_camera_path = os.path.join(get_package_share_directory("zed_wrapper"), "config", camera_model_val + ".yaml")

    with open(ros_params_override_path.perform(context), "r") as file:
        ros_params_override_config = yaml.safe_load(file)
        datahub_name = ros_params_override_config["/**"]["ros__parameters"]["general"]["datahub_name"]
        camera_name = ros_params_override_config["/**"]["ros__parameters"]["general"]["camera_name"]
        resolution_ID = ros_params_override_config["/**"]["ros__parameters"]["general"][
            "grab_resolution"
        ]  # Make sure pub_resolution=NATIVE
        frame_rate = ros_params_override_config["/**"]["ros__parameters"]["general"]["pub_frame_rate"]

    # Xacro command with options
    xacro_command = []
    xacro_command.append("xacro")
    xacro_command.append(" ")
    xacro_command.append(xacro_path.perform(context))
    xacro_command.append(" ")
    xacro_command.append("camera_name:=")
    xacro_command.append(camera_name)
    xacro_command.append(" ")
    xacro_command.append("camera_model:=")
    xacro_command.append(camera_model_val)
    xacro_command.append(" ")
    xacro_command.append("custom_baseline:=")
    xacro_command.append(custom_baseline_val)
    if enable_gnss_val == "true":
        xacro_command.append(" ")
        xacro_command.append("enable_gnss:=true")
        xacro_command.append(" ")
        if len(gnss_coords) == 3:
            xacro_command.append("gnss_x:=")
            xacro_command.append(gnss_coords[0])
            xacro_command.append(" ")
            xacro_command.append("gnss_y:=")
            xacro_command.append(gnss_coords[1])
            xacro_command.append(" ")
            xacro_command.append("gnss_z:=")
            xacro_command.append(gnss_coords[2])
            xacro_command.append(" ")

    # Robot State Publisher node
    rsp_node = ComposableNode(
        condition=IfCondition(publish_urdf),
        package="robot_state_publisher",
        namespace=datahub_name,
        plugin="robot_state_publisher::RobotStatePublisher",
        name="zed_state_publisher",
        parameters=[{"robot_description": Command(xacro_command)}],
    )

    node_parameters = [ros_params_override_path.perform(context)]
    # if ros_params_override_path.perform(context) != "":
    #     node_parameters.append(ros_params_override_path)

    # ZED Wrapper node
    zed_wrapper_node = ComposableNode(
        package="zed_components",
        namespace=datahub_name,
        plugin="stereolabs::ZedCamera",
        name=camera_name,
        # prefix=['valgrind'],
        # prefix=['xterm -e valgrind --tools=callgrind'],
        # prefix=['xterm -e gdb -ex run --args'],
        # prefix=['gdbserver localhost:3000'],
        parameters=node_parameters,
    )

    # Encoder node
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
        name=f"encoder_{camera_name}_rgb_left",
        namespace=datahub_name,
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
        name=f"encoder_{camera_name}_rgb_right",
        namespace=datahub_name,
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
        namespace=datahub_name,
        name=f"nitros_container_{camera_name}",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[rsp_node, zed_wrapper_node, encoder_node_left, encoder_node_right],
        output="screen",
        arguments=["--ros-args", "--log-level", "info"],
    )

    return [container]


def generate_launch_description():
    return LaunchDescription(
        [
            SetEnvironmentVariable(name="RCUTILS_COLORIZED_OUTPUT", value="1"),
            DeclareLaunchArgument(
                "camera_name",
                default_value=TextSubstitution(text="zed"),
                description="The name of the camera. It can be different from the camera model and it will be used as node `namespace`.",
            ),
            DeclareLaunchArgument(
                "camera_model",
                default_value=TextSubstitution(text="zedm"),
                description="The model of the camera. Using a wrong camera model can disable camera features.",
                choices=["zed", "zedm", "zed2", "zed2i", "zedx", "zedxm", "virtual"],
            ),
            DeclareLaunchArgument(
                "node_name",
                default_value="zed_node",
                description="The name of the zed_wrapper node. All the topic will have the same prefix: `/<camera_name>/<node_name>/`",
            ),
            DeclareLaunchArgument(
                "config_path",
                default_value=TextSubstitution(text=default_config_common),
                description="Path to the YAML configuration file for the camera.",
            ),
            DeclareLaunchArgument(
                "ffmpeg_config_path",
                default_value=TextSubstitution(text=default_config_ffmpeg),
                description="Path to the YAML configuration file for the FFMPEG parameters when using FFMPEG image transport plugin.",
            ),
            DeclareLaunchArgument(
                "serial_number",
                default_value="0",
                description="The serial number of the camera to be opened. It is mandatory to use this parameter in multi-camera rigs to distinguish between different cameras.",
            ),
            DeclareLaunchArgument(
                "publish_urdf",
                default_value="true",
                description="Enable URDF processing and starts Robot State Published to propagate static TF.",
                choices=["true", "false"],
            ),
            DeclareLaunchArgument(
                "publish_tf",
                default_value="true",
                description="Enable publication of the `odom -> camera_link` TF.",
                choices=["true", "false"],
            ),
            DeclareLaunchArgument(
                "publish_map_tf",
                default_value="true",
                description="Enable publication of the `map -> odom` TF. Note: Ignored if `publish_tf` is False.",
                choices=["true", "false"],
            ),
            DeclareLaunchArgument(
                "publish_imu_tf",
                default_value="true",
                description="Enable publication of the IMU TF. Note: Ignored if `publish_tf` is False.",
                choices=["true", "false"],
            ),
            DeclareLaunchArgument(
                "xacro_path",
                default_value=TextSubstitution(text=default_xacro_path),
                description="Path to the camera URDF file as a xacro file.",
            ),
            DeclareLaunchArgument("encoder_config", default_value="custom", description="Config of encoder"),
            DeclareLaunchArgument(
                "ros_params_override_path",
                default_value="/zed_mini_ros_config.yaml",
                description="The path to an additional parameters file to override the defaults",
            ),
            DeclareLaunchArgument(
                "svo_path", default_value=TextSubstitution(text="live"), description="Path to an input SVO file."
            ),
            DeclareLaunchArgument(
                "enable_gnss",
                default_value="false",
                description="Enable GNSS fusion to fix positional tracking pose with GNSS data from messages of type `sensor_msgs::msg::NavSatFix`. The fix topic can be customized in `common.yaml`.",
                choices=["true", "false"],
            ),
            DeclareLaunchArgument(
                "gnss_antenna_offset",
                default_value="[]",
                description="Position of the GNSS antenna with respect to the mounting point of the ZED camera. Format: [x,y,z]",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="If set to `true` the node will wait for messages on the `/clock` topic to start and will use this information as the timestamp reference",
                choices=["true", "false"],
            ),
            DeclareLaunchArgument(
                "sim_mode",
                default_value="false",
                description="Enable simulation mode. Set `sim_address` and `sim_port` to configure the simulator input.",
                choices=["true", "false"],
            ),
            DeclareLaunchArgument(
                "sim_address",
                default_value="127.0.0.1",
                description="The connection address of the simulation server. See the documentation of the supported simulation plugins for more information.",
            ),
            DeclareLaunchArgument(
                "sim_port",
                default_value="30000",
                description="The connection port of the simulation server. See the documentation of the supported simulation plugins for more information.",
            ),
            DeclareLaunchArgument(
                "stream_address", default_value="", description="The connection address of the input streaming server."
            ),
            DeclareLaunchArgument(
                "stream_port", default_value="30000", description="The connection port of the input streaming server."
            ),
            DeclareLaunchArgument(
                "custom_baseline",
                default_value="0.0",
                description="Distance between the center of ZED X One cameras in a custom stereo rig.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
