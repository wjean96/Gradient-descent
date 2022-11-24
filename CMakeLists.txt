cmake_minimum_required(VERSION 3.0.2)
project(capstone_WJJ)


find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
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
#  INCLUDE_DIRS include
 CATKIN_DEPENDS pcl_ros roscpp std_msgs
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

add_executable(GradientDescent src/laneFitting.cpp)
target_link_libraries(GradientDescent
  ${catkin_LIBRARIES}
)
