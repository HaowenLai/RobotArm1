cmake_minimum_required(VERSION 3.2)

project(opencv_test)

find_package(OpenCV REQUIRED)

#use c++ 11
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
#debug mode, use gdb to debug
SET(CMAKE_BUILD_TYPE "Debug") 
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")

include_directories(
 include
 /opt/ros/kinetic/include/opencv-3.3.1
 /usr/local/include
)


#all source put here
set(mainSrcList 
    src/main.cpp
    src/RsVideoCapture.cpp
    src/ArucoMarker.cpp)

add_executable(main ${mainSrcList})
add_executable(twoJoints src/twoJoints.cpp)

target_link_libraries(main ${OpenCV_LIBS} realsense2)
target_link_libraries(twoJoints ${OpenCV_LIBS} m)
