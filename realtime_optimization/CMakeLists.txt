cmake_minimum_required(VERSION 3.0.2)
project(realtime_optimization)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  visualization_msgs
)

find_package(Boost REQUIRED COMPONENTS system)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES optimization
  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)



###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(op_realtime_node src/main.cpp)
target_link_libraries(op_realtime_node
  ${catkin_LIBRARIES}
)

