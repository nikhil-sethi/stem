cmake_minimum_required(VERSION 3.0.2)
project(stem_metrics)

find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  rospy
  stem_msgs
  plan_env
  pcl_conversions
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

catkin_package()

add_executable(wig_vsep_node src/wig_vsep.cpp)
## Specify libraries to link a library or executable target against
target_link_libraries(wig_vsep_node
  ${catkin_LIBRARIES}
)

catkin_install_python(PROGRAMS
  src/metrics_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)