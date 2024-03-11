cmake_minimum_required(VERSION 3.5)
project(zed_interfaces)

## Generate symbols for IDE indexer
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

################################################
# Check the ROS2 version

set(ROS2_FOUND FALSE)
if(DEFINED ENV{ROS_DISTRO})
    set(FOUND_ROS2_DISTRO $ENV{ROS_DISTRO})
    set(ROS2_FOUND TRUE)
    #message("* Found ROS2 ${FOUND_ROS2_DISTRO}")
else()
    message("* ROS2 distro variable not set. Trying to figure it out...")
    set(ROS2_DISTROS "ardent;crystal;dashing;eloquent;foxy;galactic;humble;rolling")
    set(ROS2_FOUND FALSE)
    foreach(distro ${ROS2_DISTROS})
        if(NOT ROS2_FOUND)
            find_path(RCLCPP_H rclcpp.hpp PATHS /opt/ros/${distro}/include/rclcpp)
            if(RCLCPP_H)
                #message("* Found ROS2 ${distro}")
                set(FOUND_ROS2_DISTRO ${distro})
                set(ROS2_FOUND TRUE)
            endif()
        endif()
    endforeach()
endif()

if(ROS2_FOUND)
    if(${FOUND_ROS2_DISTRO} STREQUAL "foxy")
        #message("* ROS2 ${FOUND_ROS2_DISTRO} is officially supported by this package.")
        add_definitions(-DFOUND_FOXY)
    elseif(${FOUND_ROS2_DISTRO} STREQUAL "humble")
        #message("* ROS2 ${FOUND_ROS2_DISTRO} is officially supported by this package.")
        add_definitions(-DFOUND_HUMBLE)
    else()
        message("*** WARNING *** ROS2 ${FOUND_ROS2_DISTRO} is not officially supported by this package. Correct working is not guarantee.")
    endif()
else()
    message("*** WARNING *** ROS2 distro is unknown. This package could not work correctly.")
endif()
################################################

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(shape_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

###############################################################################
# Add all files in subdirectories of the project in
# a dummy_target so qtcreator have access to all files
file(GLOB_RECURSE extra_files ${CMAKE_SOURCE_DIR}/*)
add_custom_target(all_${PROJECT_NAME}_files SOURCES ${extra_files})
###############################################################################

set(MSG_FILES
    "msg/Object.msg"
    "msg/ObjectsStamped.msg"
    "msg/Keypoint2Di.msg"
    "msg/Keypoint2Df.msg"
    "msg/Keypoint3D.msg"
    "msg/BoundingBox2Di.msg"
    "msg/BoundingBox2Df.msg"
    "msg/BoundingBox3D.msg"
    "msg/Skeleton2D.msg"
    "msg/Skeleton3D.msg"
    "msg/DepthInfoStamped.msg"
    "msg/PlaneStamped.msg"
    "msg/PosTrackStatus.msg"
    "msg/Heartbeat.msg"
)

set(SRV_FILES
    "srv/SetPose.srv"
    "srv/StartSvoRec.srv"
    "srv/SetROI.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
    ${MSG_FILES}
    ${SRV_FILES}
    DEPENDENCIES builtin_interfaces std_msgs geometry_msgs shape_msgs
)

# Install URDF files
install(DIRECTORY
  meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
