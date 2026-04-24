cmake_minimum_required(VERSION 3.5)
project(zed_opencv_calibration)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE "Release")

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_subdirectory(stereo_calibration)
add_subdirectory(stereo_reprojection_viewer)