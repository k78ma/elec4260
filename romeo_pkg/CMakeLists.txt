cmake_minimum_required(VERSION 3.0.2)
project(romeo_pkg)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
)

catkin_package()

include_directories(
include
    ${catkin_INCLUDE_DIRS}
)

add_executable(romeo_node src/romeo_node.cpp)
target_link_libraries(romeo_node ${catkin_LIBRARIES})
add_executable(juliet_node src/juliet_node.cpp)
target_link_libraries(juliet_node ${catkin_LIBRARIES})

