cmake_minimum_required(VERSION 3.0.2)
project(stem_utils)

## System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED)
find_package(gazebo REQUIRED)
find_package(PCL REQUIRED)
find_package(catkin REQUIRED
  geometry_msgs
  roscpp
  sensor_msgs
  image_transport
  compressed_depth_image_transport
  cv_bridge
  tf2
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  geographic_msgs
  nav_msgs
  gazebo_ros
  gazebo_plugins
  pcl_ros
  pcl_conversions
  stem_msgs
  )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES sensor_model
)

catkin_install_python(PROGRAMS
  src/perception/depth_filler_node.py
  src/perception/rosimg_to_matplotlib.py
  src/perception/depth_compress.py
  src/simulation/gazebo_repub.py
  src/simulation/marker_to_pcl.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
    ${OpenCV_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIR}
    ${GAZEBO_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)


## perception utils
add_executable(depth_compress_node src/perception/depth_compress.cpp)
target_link_libraries(depth_compress_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)


add_executable(test_utils test/test_utils.cpp)
target_link_libraries(test_utils
  ${catkin_LIBRARIES}
)

add_library(sensor_model
  src/perception/camera.cpp
)
target_link_libraries(sensor_model
  ${catkin_LIBRARIES}
)


## simulation utils

add_executable(fake_localization_node src/simulation/fake_localization_node.cpp)
target_link_libraries(fake_localization_node
  ${catkin_LIBRARIES}
)

add_executable(fake_bag_odom_node src/simulation/fake_bag_odom.cpp)
target_link_libraries(fake_bag_odom_node
  ${catkin_LIBRARIES}
)

add_library(gazebo_gt
  src/simulation/gt_models.cpp
)
target_link_libraries(gazebo_gt
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)

add_executable(repub_node src/simulation/pcl_repub.cpp)
target_link_libraries(repub_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)