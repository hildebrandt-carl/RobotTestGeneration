cmake_minimum_required(VERSION 2.8.3)
project(tester)

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -o3 -std=c++11")

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs geometry_msgs)

## Declare a catkin package
catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(goal_sender src/goal_sender.cpp)
target_link_libraries(goal_sender ${catkin_LIBRARIES})

add_executable(collision_detector src/collision_detector.cpp)
target_link_libraries(collision_detector ${catkin_LIBRARIES})
