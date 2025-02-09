cmake_minimum_required(VERSION 3.0.2)
project(stem_utils)

find_package(catkin REQUIRED
  geometry_msgs
  roscpp
)

## System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED)

catkin_package()

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 

)

add_executable(test_utils test/test_utils.cpp)

target_link_libraries(test_utils
  ${catkin_LIBRARIES}
)
