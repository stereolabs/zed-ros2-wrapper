cmake_minimum_required(VERSION 3.5)

project(stereolabs_zed_ros2 NONE)

find_package(ament_cmake REQUIRED)

###############################################################################
# Add all files in subdirectories of the project in
# a dummy_target so qtcreator have access to all files
FILE(GLOB_RECURSE extra_files ${CMAKE_SOURCE_DIR}/*)
add_custom_target(${PROJECT_NAME}_files SOURCES ${extra_files})
###############################################################################

ament_package()
