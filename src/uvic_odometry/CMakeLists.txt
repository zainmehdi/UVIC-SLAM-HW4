cmake_minimum_required(VERSION 2.8.3)
project(uvic_odometry)

find_package(catkin REQUIRED COMPONENTS
  tf
  roscpp
  nav_msgs
  std_msgs
  uvic_msgs
  geometry_msgs
  )

find_package(
  Eigen3 REQUIRED
  )

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(odometry_integrator src/odometry_integrator.cpp)
target_link_libraries(odometry_integrator ${catkin_LIBRARIES} ${Eigen_LIBRARIES})
add_dependencies(odometry_integrator uvic_msgs_gencpp)

add_executable(odometry_tf src/odometry_tf.cpp)
target_link_libraries(odometry_tf ${catkin_LIBRARIES})
add_dependencies(odometry_tf uvic_msgs_gencpp)

add_executable(odometry_buffer src/odometry_buffer.cpp)
target_link_libraries(odometry_buffer ${catkin_LIBRARIES})
add_dependencies(odometry_buffer uvic_msgs_gencpp)
