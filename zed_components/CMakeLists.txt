cmake_minimum_required(VERSION 3.5)
project(zed_components)

################################################
## Generate symbols for IDE indexer (VSCode)
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
    elseif(${FOUND_ROS2_DISTRO} STREQUAL "iron")
        #message("* ROS2 ${FOUND_ROS2_DISTRO} is officially supported by this package.")
	add_definitions(-DFOUND_IRON)
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

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# if CMAKE_BUILD_TYPE is not specified, take 'Release' as default
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

if(CMAKE_BUILD_TYPE MATCHES Release)
    #message(" * Release Mode")
    add_compile_options(-Wno-deprecated-declarations)
endif()

if(CMAKE_BUILD_TYPE MATCHES RelWithDebInfo)
    #message(" * Release with Debug Info Mode")
    add_compile_options(-Wno-deprecated-declarations)
endif()

if(CMAKE_BUILD_TYPE MATCHES Debug)
    message(" * Debug Mode")
endif()

#############################################
# Dependencies
find_package(ZED 4 REQUIRED)

exec_program(uname ARGS -p OUTPUT_VARIABLE CMAKE_SYSTEM_NAME2)
if(CMAKE_SYSTEM_NAME2 MATCHES "aarch64") # Jetson TX
    set(CUDA_USE_STATIC_CUDA_RUNTIME OFF)
endif()

find_package(CUDA REQUIRED)

set(DEPENDENCIES
    rclcpp
    rclcpp_components
    image_transport
    builtin_interfaces
    std_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    geometry_msgs
    nav_msgs
    nmea_msgs
    geographic_msgs
    sensor_msgs
    stereo_msgs
    zed_interfaces
    std_srvs
    diagnostic_msgs
    diagnostic_updater
    visualization_msgs
    shape_msgs
    robot_localization
    ${TM_DEP}
)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(rcutils REQUIRED)
find_package(zed_interfaces)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nmea_msgs REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(stereo_msgs REQUIRED)

find_package(image_transport REQUIRED)
find_package(std_srvs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(robot_localization REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

###############################################################################
#Add all files in subdirectories of the project in
# a dummy_target so qtcreator have access to all files
file(GLOB_RECURSE all_files ${CMAKE_SOURCE_DIR}/*)
add_custom_target(all_${PROJECT_NAME}_files SOURCES ${all_files})

###############################################################################
# INCLUDES and LIBS
include_directories(
    ${CUDA_INCLUDE_DIRS}
    ${ZED_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/src/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src/zed_camera/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src/tools/include
)

# create ament index resource which references the libraries in the binary dir
set(node_plugins "")

link_directories(${ZED_LIBRARY_DIR})
link_directories(${CUDA_LIBRARY_DIRS})

set(ZED_LIBS
  ${ZED_LIBRARIES}
  ${CUDA_LIBRARIES}
)

###############################################################################
# SOURCES
set(SL_TOOLS_INC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/tools/include/sl_tools.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/tools/include/sl_win_avg.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/tools/include/sl_logging.hpp
)

set(SL_TOOLS_SRC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/tools/src/sl_tools.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/tools/src/sl_win_avg.cpp
)

set(ZED_CAMERA_INC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/include/visibility_control.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/zed_camera/include/sl_types.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/zed_camera/include/zed_camera_component.hpp    
)

set(ZED_CAMERA_SRC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/zed_camera/src/zed_camera_component.cpp
)

###############################################################################
# Bin and Install

add_library(zed_camera_component SHARED
    ${SL_TOOLS_INC}
    ${SL_TOOLS_SRC}
    ${ZED_CAMERA_INC}
    ${ZED_CAMERA_SRC}
)
target_compile_definitions(zed_camera_component
    PRIVATE "COMPOSITION_BUILDING_DLL"
)
target_link_libraries(zed_camera_component
    ${ZED_LIBS}
)
ament_target_dependencies(zed_camera_component
    ${DEPENDENCIES}
)

rclcpp_components_register_nodes(zed_camera_component "stereolabs::ZedCamera")
set(node_plugins "${node_plugins}stereolabs::ZedCamera;$<TARGET_FILE:zed_camera_component>\n")

# Install components
install(TARGETS zed_camera_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY
    ${CMAKE_CURRENT_SOURCE_DIR}/src/zed_camera/include/
    ${CMAKE_CURRENT_SOURCE_DIR}/src/tools/include/
    ${CMAKE_CURRENT_SOURCE_DIR}/src/include/
    DESTINATION include/${PROJECT_NAME}/
)

ament_export_include_directories(include)
ament_export_libraries(zed_camera_component)
ament_export_dependencies(${DEPENDENCIES})
ament_package()
