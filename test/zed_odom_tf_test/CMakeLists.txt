cmake_minimum_required(VERSION 3.5)
project(stereolabs_zed_odom_tf_test)

# if CMAKE_BUILD_TYPE is not specified, take 'Release' as default
IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Release)
ENDIF(NOT CMAKE_BUILD_TYPE)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

###############################################################################
# Add all files in subdirectories of the project in
# a dummy_target so qtcreator have access to all files
FILE(GLOB_RECURSE extra_files ${CMAKE_SOURCE_DIR}/*)
add_custom_target(${PROJECT_NAME}_files SOURCES ${extra_files})
###############################################################################

# includes
include_directories(
    ${rclcpp_INCLUDE_DIRS}
    ${sensor_msgs_INCLUDE_DIRS}
    ${sensor_msgs_INCLUDE_DIRS}
    ${nav_msgs_INCLUDE_DIRS}
    ${geometry_msgs_INCLUDE_DIRS}
    ${tf2_INCLUDE_DIRS}
    ${tf2_ros_INCLUDE_DIRS}
    ${tf2_geometry_msgs_INCLUDE_DIRS}
)

## Build 
add_executable(zed_odom_tf_test src/zed_odom_tf_test.cpp)
target_link_libraries(zed_odom_tf_test
    ${rclcpp_LIBRARIES}
    ${nav_msgs_LIBRARIES}
)
ament_target_dependencies(zed_odom_tf_test
  rclcpp
  sensor_msgs
  nav_msg
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msg
)

# Install executable
install(
    TARGETS zed_odom_tf_test
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
