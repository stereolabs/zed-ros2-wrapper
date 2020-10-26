cmake_minimum_required(VERSION 3.5)

project(zed_ros2 NONE)

find_package(ament_cmake REQUIRED)

###############################################################################
# Add all files in subdirectories of the project in
# a dummy_target so qtcreator have access to all files
FILE(GLOB_RECURSE all_files ${CMAKE_SOURCE_DIR}/*)
add_custom_target(all_${PROJECT_NAME}_files SOURCES ${all_files})
###############################################################################

ament_package()
