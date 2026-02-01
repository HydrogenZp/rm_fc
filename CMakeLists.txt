cmake_minimum_required(VERSION 3.10)
project(rm_fc)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    nav_msgs
    geometry_msgs
    tf2_ros
    tf2_eigen
    tf2_geometry_msgs
)

find_package(Eigen3 REQUIRED)

catkin_package()

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

add_executable(odom_to_acfly src/odom_to_acfly.cpp)
target_link_libraries(odom_to_acfly ${catkin_LIBRARIES})

add_executable(odom_debug src/odom_debug.cpp)
target_link_libraries(odom_debug ${catkin_LIBRARIES})
