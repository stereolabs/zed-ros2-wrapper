cmake_minimum_required(VERSION 3.5)

project(stereolabs_zed_interfaces)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

###############################################################################
# Add all files in subdirectories of the project in
# a dummy_target so qtcreator have access to all files
FILE(GLOB_RECURSE extra_files ${CMAKE_SOURCE_DIR}/*)
add_custom_target(${PROJECT_NAME}_files SOURCES ${extra_files})
###############################################################################

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ResetOdometry.srv"
  "srv/RestartTracking.srv"
  "srv/SetPose.srv"
  "srv/StartSvoRecording.srv"
  "srv/StopSvoRecording.srv"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
