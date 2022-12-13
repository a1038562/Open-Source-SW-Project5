cmake_minimum_required(VERSION 3.0.2)
project(control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  message_generation
)

add_message_files(
  FILES
  control.msg
)

generate_messages(
  DEPENDENCIES
  sensor_msgs  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES control
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs
)

file(GLOB_RECURSE project_HEADERS
  "*.h"
  "*.hpp"
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_executable(
  ${PROJECT_NAME}
  src/LocalControl.cpp
  src/main.cpp
)

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

target_link_libraries(
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)