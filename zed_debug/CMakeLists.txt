cmake_minimum_required(VERSION 3.8)
project(zed_debug)

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
    set(ROS2_DISTROS "ardent;bouncy;crystal;dashing;eloquent;foxy;galactic;humble;iron;jazzy;kilted;lyrical;rolling")
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
    elseif(${FOUND_ROS2_DISTRO} STREQUAL "jazzy")
        #message("* ROS2 ${FOUND_ROS2_DISTRO} is officially supported by this package.")
        add_definitions(-DFOUND_JAZZY)
    elseif(${FOUND_ROS2_DISTRO} STREQUAL "lyrical")
        add_definitions(-DFOUND_LYRICAL)
    elseif(${FOUND_ROS2_DISTRO} STREQUAL "rolling")
        add_definitions(-DFOUND_ROLLING)
    else()
        message("*** WARNING *** Unsupported ROS2 ${FOUND_ROS2_DISTRO}. '${PROJECT_NAME}' may not work correctly.")
    endif()
else()
    message("*** WARNING *** ROS2 distro is unknown. This package could not work correctly.")
endif()
################################################

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
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
if(CMAKE_VERSION VERSION_GREATER_EQUAL "3.27")
  cmake_policy(SET CMP0146 OLD)
endif()
if(CMAKE_VERSION VERSION_GREATER_EQUAL "3.28")
  cmake_policy(SET CMP0153 OLD)
endif()
find_package(ZED REQUIRED)
if(CMAKE_VERSION VERSION_GREATER_EQUAL "3.27")
  cmake_policy(SET CMP0146 NEW)
endif()
if(CMAKE_VERSION VERSION_GREATER_EQUAL "3.28")
  cmake_policy(SET CMP0153 NEW)
endif()

execute_process(COMMAND uname -p OUTPUT_VARIABLE CMAKE_SYSTEM_NAME2 OUTPUT_STRIP_TRAILING_WHITESPACE)
if(CMAKE_SYSTEM_NAME2 MATCHES "aarch64") # Jetson TX
    set(CUDA_USE_STATIC_CUDA_RUNTIME OFF)
endif()

find_package(CUDAToolkit REQUIRED)
set(CUDA_INCLUDE_DIRS ${CUDAToolkit_INCLUDE_DIRS})
set(CUDA_LIBRARIES CUDA::cudart)
set(CUDA_LIBRARY_DIRS ${CUDAToolkit_LIBRARY_DIR})

set(DEPENDENCIES
    rclcpp
    rclcpp_components
    zed_components
    backward_ros
)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(rcutils REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(zed_components REQUIRED)

# Debugging with backward-cpp
find_package(backward_ros REQUIRED)

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
# ament_target_dependencies() replacement: ROS 2 Lyrical's ament_cmake removed
# that macro outright. For each dependency, link its exported modern CMake
# target(s) (${dep}_TARGETS), falling back to the classic
# ${dep}_INCLUDE_DIRS / ${dep}_LIBRARIES variables for dependencies that only
# export those (e.g. backward_ros, zed_components).
macro(zed_link_ros_dependencies target)
  foreach(_dep ${ARGN})
    if(${_dep}_TARGETS)
      target_link_libraries(${target} ${${_dep}_TARGETS})
    else()
      if(${_dep}_INCLUDE_DIRS)
        target_include_directories(${target} PUBLIC ${${_dep}_INCLUDE_DIRS})
      endif()
      if(${_dep}_LIBRARIES)
        target_link_libraries(${target} ${${_dep}_LIBRARIES})
      endif()
    endif()
  endforeach()
endmacro()

###############################################################################
# LIBS

link_directories(${ZED_LIBRARY_DIR})
link_directories(${CUDA_LIBRARY_DIRS})

set(ZED_LIBS
  ${ZED_LIBRARIES}
  ${CUDA_LIBRARIES}
)

###############################################################################
# SOURCES
set(ZED_DEBUG_PROC 
    ${CMAKE_CURRENT_SOURCE_DIR}/src/zed_debug_proc.cpp
)

###############################################################################
# Bin and Install

# ZED DEBUG PROC executable
add_executable(zed_debug_proc
    ${ZED_DEBUG_PROC}
)
target_include_directories(zed_debug_proc PUBLIC
    ${CUDA_INCLUDE_DIRS}
    ${ZED_INCLUDE_DIRS}
)
target_link_directories(zed_debug_proc PUBLIC
    ${CUDA_LIBRARY_DIRS}
)
target_link_libraries(zed_debug_proc
    ${CUDA_LIBRARIES}
)
zed_link_ros_dependencies(zed_debug_proc ${DEPENDENCIES})

# Install executable
install(TARGETS zed_debug_proc
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}  
)

# Install LAUNCH and CONFIG files
install(DIRECTORY
  launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
