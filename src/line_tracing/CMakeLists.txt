cmake_minimum_required(VERSION 2.8.3)
project(line_tracing)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  core_msgs
  image_transport
  message_generation
  visualization_msgs
)

find_package(OpenCV REQUIRED )
find_package( cv_bridge REQUIRED )
catkin_package(
#  INCLUDE_DIRS include
#  CATKIN_DEPENDS roscpp
#  sensor_msgs
#  std_msgs
#  core_msgs
#  image_transport
#  visualization_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  include
)

add_executable(line_tracing_node src/line_tracing.cpp)
add_dependencies(line_tracing_node core_msgs_generate_messages_cpp)

target_link_libraries(line_tracing_node
  ${catkin_LIBRARIES} ${OpenCV_LIBS} ${cv_bridge_LIBRARIES}
)

add_executable(line_stop src/line_stop.cpp)
add_dependencies(line_stop core_msgs_generate_messages_cpp)

target_link_libraries(line_stop
  ${catkin_LIBRARIES} ${OpenCV_LIBS} ${cv_bridge_LIBRARIES}
)
