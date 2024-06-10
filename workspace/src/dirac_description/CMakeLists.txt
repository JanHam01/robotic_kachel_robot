cmake_minimum_required(VERSION 3.0.2)
project(dirac_description)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  sensor_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp geometry_msgs sensor_msgs std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Add the dirac_control executable
add_executable(dirac_control src/dirac_control.cpp)
target_link_libraries(dirac_control ${catkin_LIBRARIES})

# Add the lidar_listener executable
add_executable(lidar_listener src/lidar_listener.cpp)
target_link_libraries(lidar_listener ${catkin_LIBRARIES})
add_dependencies(lidar_listener ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})



