cmake_minimum_required(VERSION 3.0.2)
project(stem_planner)

find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED
    common
    geometry_msgs
    roscpp
    active_perception
    message_generation
    stem_msgs
)

catkin_python_setup()

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
)


catkin_package()

include_directories(
    # include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS} 
)

catkin_install_python(PROGRAMS
  src/mo_tsp_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
