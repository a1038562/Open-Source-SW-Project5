cmake_minimum_required(VERSION 3.0.2)
project(lidar_detection)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  message_generation
)

add_message_files(
  FILES
  lidar.msg
)

generate_messages(
  DEPENDENCIES
  sensor_msgs  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES lidar_detection
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/lidar_detect.cpp)

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)
