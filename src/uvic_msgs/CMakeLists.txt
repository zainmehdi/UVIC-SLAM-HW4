cmake_minimum_required(VERSION 2.8.3)
project(uvic_msgs)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
  message_generation
  )

add_message_files(
  FILES
  factor.msg
  keyframe.msg
  odometry.msg
  registration.msg
  covariance.msg
  )

add_service_files(
  FILES
  odometry_buffer.srv
  scan_to_pointcloud.srv
  kf_select.srv
  )

generate_messages(
  DEPENDENCIES
  std_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
  )

catkin_package(
  CATKIN_DEPENDS message_runtime
  )

