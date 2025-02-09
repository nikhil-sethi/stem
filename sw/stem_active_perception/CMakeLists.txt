cmake_minimum_required(VERSION 3.0.2)
project(stem_active_perception)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE "Debug")
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  cv_bridge

  sensor_msgs
  visualization_msgs
  tf2_geometry_msgs
  nav_msgs

  active_perception
  traj_utils
  plan_env
  
  stem_planner
  stem_utils
  stem_msgs
  )
find_package(PCL 1.7 REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
 CATKIN_DEPENDS stem_utils stem_msgs plan_env traj_utils
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS} 
)

add_executable(target_att_gt_node src/target_att_gt.cpp)
add_executable(diff_map_gt_node src/diffusion_map_gt.cpp)
add_executable(attention_map_2d_node src/attention_map_2d.cpp)


target_link_libraries(target_att_gt_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
target_link_libraries(diff_map_gt_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
target_link_libraries(attention_map_2d_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

## in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS
  src/attention_map.py
  # src/att_gmm.py
  # src/att_gmm_dynamic.py
  # src/test.py
  # src/gmm_map.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
