cmake_minimum_required(VERSION 3.0.2)
project(dirac_description)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp geometry_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(dirac_control src/dirac_control.cpp)
target_link_libraries(dirac_control ${catkin_LIBRARIES})
