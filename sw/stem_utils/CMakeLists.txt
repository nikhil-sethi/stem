cmake_minimum_required(VERSION 3.0.2)
project(stem_utils)

## System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED)
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
)

catkin_package()

catkin_install_python(PROGRAMS
  src/img_utils/depth_filler_node.py
  src/img_utils/rosimg_to_matplotlib.py
  src/img_utils/depth_compress.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
    ${OpenCV_INCLUDE_DIRS}

)
add_executable(depth_compress_node src/img_utils/depth_compress.cpp)
target_link_libraries(depth_compress_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)


add_executable(test_utils test/test_utils.cpp)
target_link_libraries(test_utils
  ${catkin_LIBRARIES}
)

add_library(${PROJECT_NAME}
  src/camera.cpp
)
target_link_libraries(sensor_model
  ${catkin_LIBRARIES}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)