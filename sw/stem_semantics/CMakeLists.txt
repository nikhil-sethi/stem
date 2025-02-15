cmake_minimum_required(VERSION 3.0.2)
project(stem_semantics)


find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
)

catkin_python_setup()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
catkin_package()
catkin_install_python(PROGRAMS
  src/object_detection.py
  src/priority_mask.py
  src/priority_mask_vsep.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
