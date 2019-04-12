cmake_minimum_required(VERSION 3.5)
project(stereolabs_example_multi_camera)

# if CMAKE_BUILD_TYPE is not specified, take 'Release' as default
IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Release)
ENDIF(NOT CMAKE_BUILD_TYPE)

# dependencies
find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(stereolabs_zed REQUIRED)

###############################################################################
# Add all files in subdirectories of the project in
# a dummy_target so qtcreator have access to all files
FILE(GLOB_RECURSE extra_files ${CMAKE_SOURCE_DIR}/*)
add_custom_target(${PROJECT_NAME}_files SOURCES ${extra_files})
###############################################################################

# Install launch files
install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME})

# Install URDF files
install(
    DIRECTORY urdf
    DESTINATION share/${PROJECT_NAME})

# Install YAML files
install(
    DIRECTORY config
    DESTINATION share/${PROJECT_NAME})

ament_package()
