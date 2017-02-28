cmake_minimum_required(VERSION 2.8.3)
project(uvic_scanner)

find_package(catkin REQUIRED COMPONENTS
  tf
  roscpp
  pcl_ros
  std_msgs
  nav_msgs
  uvic_msgs
  sensor_msgs
  geometry_msgs
  pcl_conversions
  laser_geometry
  cmake_modules
  )

find_package(
  Eigen3 REQUIRED
  )

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  )

catkin_package()

add_executable(scan_conversion src/scan_conversion.cpp)
target_link_libraries(scan_conversion ${catkin_LIBRARIES})
add_dependencies(scan_conversion uvic_msgs_gencpp)

add_executable(scan_matching src/scan_matching.cpp)
target_link_libraries(scan_matching ${catkin_LIBRARIES} ${EIGEN_LIBRARIES})
add_dependencies(scan_matching uvic_msgs_gencpp)
