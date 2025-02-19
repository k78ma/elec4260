cmake_minimum_required(VERSION 3.0.2)
project(helloworld)

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

add_executable(helloworld_node src/helloworld.cpp)
target_link_libraries(helloworld_node ${catkin_LIBRARIES})