cmake_minimum_required(VERSION 3.0.2)
project(udp_client)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(udp_client src/udp_client.cpp)
target_link_libraries(udp_client ${catkin_LIBRARIES})