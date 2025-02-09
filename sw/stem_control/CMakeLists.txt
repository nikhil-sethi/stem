cmake_minimum_required(VERSION 3.0.2)
project(stem_control)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  rospy
  roscpp
  std_msgs
  drone_toolbox_ext_control_template
  quadrotor_msgs
  tf2_ros
)
catkin_python_setup()

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/control_interface_node.cpp)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)

catkin_install_python(PROGRAMS
  src/teleop_if.py
  src/racer_if.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
