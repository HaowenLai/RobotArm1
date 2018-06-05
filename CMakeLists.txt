cmake_minimum_required(VERSION 3.2)

project(RoboticArm)

find_package(OpenCV REQUIRED)

#use c++ 11
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

#debug mode, use gdb to debug
SET(CMAKE_BUILD_TYPE "Debug") 
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")

include_directories(
 include
 /opt/ros/kinetic/include/opencv-3.3.1-dev
 /usr/local/include
)


#all source put here
set(markerPosSrcList 
    exec/markerPos.cpp
    src/RsVideoCapture.cpp
    src/ArucoMarker.cpp)

set(commuSrcList 
    exec/communication.cpp
    src/BpNetwork.cpp
    src/UsbCAN.cpp)

set(dataCollectSrcList 
    exec/dataCollect.cpp
    src/ArucoMarker.cpp
    src/UsbCAN.cpp)


add_executable(markerPos ${markerPosSrcList})
add_executable(twoJoints exec/twoJointsSimu.cpp)
add_executable(commu ${commuSrcList})
add_executable(dataCollect ${dataCollectSrcList})

target_link_libraries(markerPos ${OpenCV_LIBS} realsense2)
target_link_libraries(twoJoints ${OpenCV_LIBS} m)
target_link_libraries(commu ${OpenCV_LIBS} usbcan)
target_link_libraries(dataCollect ${OpenCV_LIBS} usbcan)
