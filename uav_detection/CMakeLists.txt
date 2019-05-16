########### 版本要求
cmake_minimum_required(VERSION 2.8.3)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

########### 项目名称
project(uav_detection)

########### 添加系统库
find_package(OpenCV 3.4.0 REQUIRED COMPONENTS core highgui imgproc imgcodecs aruco)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        dji_sdk
        tf
        )

catkin_package()

########### 添加头文件
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

########### 编译库
add_library(coordinatetransform src/coordinatetransform.cpp)
target_link_libraries(coordinatetransform ${catkin_LIBRARIES})

add_library(camera src/camera.cpp)
target_link_libraries(camera dcam ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_library(detector src/detector.cpp)
target_link_libraries(detector coordinatetransform ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_library(serialtransmission src/serialtransmission.cpp)
target_link_libraries(serialtransmission serial ${catkin_LIBRARIES})

add_library(targetdetect src/targetdetect.cpp)
target_link_libraries(targetdetect coordinatetransform detector camera serialtransmission ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_library(flightcontrol src/flightcontrol.cpp)
target_link_libraries(flightcontrol coordinatetransform ${catkin_LIBRARIES})

########### 编译可执行文件
add_executable(targetdetect_node src/targetdetect_node.cpp)
target_link_libraries(targetdetect_node targetdetect ${catkin_LIBRARIES})

add_executable(flightcontrol_node src/flightcontrol_node.cpp)
target_link_libraries(flightcontrol_node flightcontrol ${catkin_LIBRARIES})

add_executable(drawpath_node src/drawpath_node.cpp)
target_link_libraries(drawpath_node ${catkin_LIBRARIES})



























