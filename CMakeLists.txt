cmake_minimum_required(VERSION 3.2)

project(RoboticArm)

find_package(OpenCV REQUIRED)
find_package(PythonLibs REQUIRED)

#use c++ 11
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

#debug mode, use gdb to debug
SET(CMAKE_BUILD_TYPE "Debug") 
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")

include_directories(
 include
 /opt/ros/kinetic/include/opencv-3.3.1-dev
 /usr/local/include
 ${PYTHON_INCLUDE_PATH}
)


#all source put here
set(markerPosSrcList 
    exec/markerPos.cpp
    src/RsVideoCapture.cpp
    src/ArucoMarker.cpp)

set(commuSrcList 
    exec/communication.cpp
    src/BpNetwork.cpp
    src/UsbCAN.cpp
    src/ArucoMarker.cpp
    src/RsVideoCapture.cpp
    src/control.cpp)

set(dataCollectSrcList 
    exec/dataCollect.cpp
    src/ArucoMarker.cpp
    src/RsVideoCapture.cpp
    src/BpNetwork.cpp
    src/control.cpp
    src/UsbCAN.cpp)

set(manipulateSrcList 
    exec/manipulate.cpp
    src/ArucoMarker.cpp
    src/RsVideoCapture.cpp
    src/BpNetwork.cpp
    src/control.cpp
    src/UsbCAN.cpp)

add_executable(markerPos ${markerPosSrcList})
add_executable(twoJoints exec/twoJointsSimu.cpp)
add_executable(commu ${commuSrcList})
add_executable(dataCollect ${dataCollectSrcList})
add_executable(manipulate ${manipulateSrcList})

target_link_libraries(markerPos ${OpenCV_LIBS} realsense2)
target_link_libraries(twoJoints ${OpenCV_LIBS} m)
target_link_libraries(commu ${OpenCV_LIBS} ${PYTHON_LIBRARIES} usbcan realsense2 pthread)
target_link_libraries(dataCollect ${OpenCV_LIBS} ${PYTHON_LIBRARIES} usbcan pthread realsense2)
target_link_libraries(manipulate ${OpenCV_LIBS} ${PYTHON_LIBRARIES} usbcan realsense2)
