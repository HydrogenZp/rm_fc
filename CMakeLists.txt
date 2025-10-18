cmake_minimum_required(VERSION 3.10)
project(rm_fc)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_definitions(-Wall -Werror)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    nav_msgs          
    geometry_msgs     
    sensor_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
)

find_package(Eigen3 REQUIRED)

catkin_package(
    INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIR}
    CATKIN_DEPENDS roscpp nav_msgs geometry_msgs sensor_msgs
    roscpp
    std_msgs
    geometry_msgs
    nav_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

add_executable(vins_to_mavros src/vision_to_mavros.cpp) 
target_link_libraries(vins_to_mavros ${catkin_LIBRARIES})
