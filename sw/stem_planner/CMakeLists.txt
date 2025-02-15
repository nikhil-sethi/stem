cmake_minimum_required(VERSION 3.0.2)
project(stem_planner)

find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED
    roscpp    
    geometry_msgs
    
    active_perception
    stem_msgs
    stem_utils
)

catkin_python_setup()

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
