cmake_minimum_required(VERSION 3.0.2)
project(godfather_pkg)

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

add_executable(godfather_node src/godfather_node.cpp)
target_link_libraries(godfather_node ${catkin_LIBRARIES})