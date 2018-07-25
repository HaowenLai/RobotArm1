cmake_minimum_required(VERSION 3.2)

project(RoboticArm)

find_package(OpenCV REQUIRED)
find_package(PythonLibs REQUIRED)

#use c++ 11
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

#debug mode, use gdb to debug
#SET(CMAKE_BUILD_TYPE "Debug") 
#SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")

include_directories(
 include
 /usr/local/include
 ${OpenCV_INCLUDE_DIRS}
 ${PYTHON_INCLUDE_PATH}
)


#all source put here
set(markerPosSrcList 
    exec/markerPos.cpp
    src/RsVideoCapture.cpp
    src/control.cpp
    src/position.cpp
    src/BpNetwork.cpp
    src/UsbCAN.cpp)

set(smallTestsSrcList 
    exec/smallTests.cpp
    src/BpNetwork.cpp
    src/LettersClassify.cpp
    src/UsbCAN.cpp
    src/position.cpp
    src/RsVideoCapture.cpp
    src/control.cpp
    src/Wifi.cpp)

set(dataCollectSrcList 
    exec/dataCollect.cpp
    src/position.cpp
    src/RsVideoCapture.cpp
    src/BpNetwork.cpp
    src/control.cpp
    src/UsbCAN.cpp)

set(manipulateSrcList 
    exec/manipulate.cpp
    src/position.cpp
    src/RsVideoCapture.cpp
    src/BpNetwork.cpp
    src/control.cpp
    src/UsbCAN.cpp)

set(mainTaskSrcList
    exec/mainTask.cpp
    src/BpNetwork.cpp
    src/UsbCAN.cpp
    src/position.cpp
    src/Wifi.cpp
    src/RsVideoCapture.cpp
    src/control.cpp)



add_executable(markerPos ${markerPosSrcList})
#add_executable(smallTests ${smallTestsSrcList})
#add_executable(dataCollect ${dataCollectSrcList})
add_executable(manipulate ${manipulateSrcList})
add_executable(mainTask ${mainTaskSrcList})
add_executable(startIt exec/startIt.cpp)

target_link_libraries(markerPos ${OpenCV_LIBS} ${PYTHON_LIBRARIES} usbcan realsense2)
#target_link_libraries(smallTests ${OpenCV_LIBS} ${PYTHON_LIBRARIES} usbcan realsense2)
#target_link_libraries(dataCollect ${OpenCV_LIBS} ${PYTHON_LIBRARIES} usbcan pthread realsense2)
target_link_libraries(manipulate ${OpenCV_LIBS} ${PYTHON_LIBRARIES} usbcan realsense2)
target_link_libraries(mainTask ${OpenCV_LIBS} ${PYTHON_LIBRARIES} usbcan realsense2 pthread)
target_link_libraries(startIt pthread usbcan)

