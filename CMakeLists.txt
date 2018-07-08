cmake_minimum_required(VERSION 3.0)
project(ProjectLiDARToImage)

set(CMAKE_CXX_STANDARD 11)

add_executable(ProjectLiDARToImage ProjectLiDARToImage.cpp)

#PCL
find_package(PCL 1.5 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
target_link_libraries(ProjectLiDARToImage ${PCL_LIBRARIES})

#OpenCV
find_package(OpenCV)
target_link_libraries(ProjectLiDARToImage ${OpenCV_LIBRARIES})