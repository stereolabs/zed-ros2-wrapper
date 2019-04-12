cmake_minimum_required(VERSION 3.5)

project(stereolabs_zed)

# ROS2 Flags
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

#if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#  add_compile_options(-Wall -Wextra -Wpedantic)
#endif()

# if CMAKE_BUILD_TYPE is not specified, take 'Release' as default
IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Release)
ENDIF(NOT CMAKE_BUILD_TYPE)

function(checkPackage package customMessage)
    set(varName "${package}_FOUND")
    if (NOT "${${varName}}")
        string(REPLACE "_" "-" aptPackage ${package})
        if("${customMessage}" STREQUAL "")
            message(FATAL_ERROR "\n\n ${package} is missing, please try to install it with:\n sudo apt-get install ros-$(rosversion -d)-${aptPackage}\n\n")
        else()
            message(FATAL_ERROR "\n\n ${customMessage} \n\n")
        endif()
    endif()
endfunction(checkPackage)

find_package(ZED 2.6)
checkPackage("ZED" "ZED SDK not found, install it from:\n https://www.stereolabs.com/developers/")

exec_program(uname ARGS -p OUTPUT_VARIABLE CMAKE_SYSTEM_NAME2)
if ( CMAKE_SYSTEM_NAME2 MATCHES "aarch64" ) # Jetson TX
    SET(CUDA_USE_STATIC_CUDA_RUNTIME OFF)
endif()

find_package(CUDA)
checkPackage("CUDA" "CUDA not found, install it from:\n https://developer.nvidia.com/cuda-downloads")

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(class_loader REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw_implementation REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(stereo_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(stereolabs_zed_interfaces REQUIRED)

###############################################################################
#Add all files in subdirectories of the project in
# a dummy_target so qtcreator have access to all files
FILE(GLOB_RECURSE extra_files ${CMAKE_SOURCE_DIR}/*)
add_custom_target(${PROJECT_NAME}_files SOURCES ${extra_files})

###############################################################################
# INCLUDES and LIBS
include_directories(
    ${CUDA_INCLUDE_DIRS}
    ${ZED_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/src/tools/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src/component/include
    ${rclcpp_lifecycle_INCLUDE_DIRS}
    ${lifecycle_msgs_INCLUDE_DIRS}
    ${rclcpp_INCLUDE_DIRS}
    ${class_loader_INCLUDE_DIRS}
    ${rmw_implementation_INCLUDE_DIRS}
    ${std_msgs_INCLUDE_DIRS}
    ${sensor_msgs_INCLUDE_DIRS}
    ${stereo_msgs_INCLUDE_DIRS}
    ${tf2_INCLUDE_DIRS}
    ${tf2_ros_INCLUDE_DIRS}
    ${tf2_INCLUDE_DIRS}
    ${tf2_geometry_msgs_INCLUDE_DIRS}
    ${nav_msgs_INCLUDE_DIRS}
    ${image_transport_INCLUDE_DIRS}
    ${geometry_msgs_INCLUDE_DIRS}
    ${stereolabs_zed_interfaces_INCLUDE_DIRS}
)

link_directories(${ZED_LIBRARY_DIR})
link_directories(${CUDA_LIBRARY_DIRS})

if(NOT DEFINED CUDA_NPP_LIBRARIES_ZED)
    #To deal with cuda 9 nppi libs and previous versions of ZED SDK
    set(CUDA_NPP_LIBRARIES_ZED ${CUDA_npp_LIBRARY} ${CUDA_npps_LIBRARY} ${CUDA_nppi_LIBRARY})
endif()

set(LINK_LIBRARIES
  ${ZED_LIBRARIES}
  ${CUDA_LIBRARIES} ${CUDA_NPP_LIBRARIES_ZED}
  )

###############################################################################
# SOURCES
set(TOOLS_SRC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/tools/src/sl_tools.cpp )

set(NODE_SRC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/zed_wrapper_container.cpp
)

set(ZED_COMPONENT_SRC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/component/src/zed_component.cpp
)

set(ZED_IT_SRC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/component/src/zed_it_broadcaster.cpp
)

set(ZED_TF_SRC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/component/src/zed_tf_broadcaster.cpp
)
###############################################################################
# ZED COMPONENT PLUGIN and ZED EXECUTABLE

# create ament index resource which references the libraries in the binary dir
set(node_plugins "")

add_library(zed_component SHARED ${TOOLS_SRC} ${ZED_COMPONENT_SRC})
target_compile_definitions(zed_component PRIVATE "ZED_BUILDING_DLL")
target_link_libraries(zed_component
    ${rclcpp_lifecycle_LIBRARIES}
    ${LINK_LIBRARIES}
)
ament_target_dependencies(zed_component
  lifecycle_msgs
  rclcpp_lifecycle
  rclcpp
  std_msgs
  sensor_msgs
  stereo_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  nav_msgs
  geometry_msgs
  stereolabs_zed_interfaces
)
rclcpp_register_node_plugins(zed_component "stereolabs::ZedCameraComponent")
set(node_plugins "${node_plugins}stereolabs::ZedCameraComponent;$<TARGET_FILE:zed_component>\n")

add_library(zed_it_broadcaster SHARED ${TOOLS_SRC} ${ZED_IT_SRC})
target_compile_definitions(zed_it_broadcaster PRIVATE "ZED_BUILDING_DLL")
target_link_libraries(zed_it_broadcaster
    ${rclcpp_LIBRARIES}
)
ament_target_dependencies(zed_it_broadcaster
  rclcpp
  std_msgs
  sensor_msgs
  image_transport
)
rclcpp_register_node_plugins(zed_it_broadcaster "stereolabs::ZedCameraIT")
set(node_plugins "${node_plugins}stereolabs::ZedCameraIT;$<TARGET_FILE:zed_it_broadcaster>\n")

add_library(zed_tf_broadcaster SHARED ${TOOLS_SRC} ${ZED_TF_SRC})
target_compile_definitions(zed_tf_broadcaster PRIVATE "ZED_BUILDING_DLL")
target_link_libraries(zed_tf_broadcaster
    ${rclcpp_LIBRARIES}
)
ament_target_dependencies(zed_tf_broadcaster
  rclcpp
  std_msgs
  sensor_msgs
  navigation_msgs
  geometry_msgs
  tf2_geometry_msgs
  tf2
  tf2_ros
)
rclcpp_register_node_plugins(zed_tf_broadcaster "stereolabs::ZedCameraTF")
set(node_plugins "${node_plugins}stereolabs::ZedCameraTF;$<TARGET_FILE:zed_tf_broadcaster>\n")

add_executable(zed_wrapper_node ${NODE_SRC})
target_link_libraries(zed_wrapper_node
    zed_component
    zed_it_broadcaster
    zed_tf_broadcaster
)
ament_target_dependencies(zed_wrapper_node
  rclcpp
  class_loader
  zed_component
  zed_it_broadcaster
  zed_tf_broadcaster
)

# Install executable
install(
    TARGETS zed_wrapper_node
    DESTINATION lib/${PROJECT_NAME}
)

# Install plugins
install(
    TARGETS zed_component
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

# Install header files
install(
    DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/src/component/include/
    DESTINATION include/${PROJECT_NAME}/
)

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
