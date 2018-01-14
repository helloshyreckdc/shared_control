cmake_minimum_required(VERSION 2.8.3)
project(sh_ctrl)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  pcl_conversions
  pcl_ros
)

catkin_package(CATKIN_DEPENDS
  roscpp
  rospy
  std_msgs
  pcl_conversions
  pcl_ros
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

link_directories(
  ${PCL_LIBRARY_DIRS}
)

add_executable(target_pose_estimation src/target_pose_estimation.cpp)
target_link_libraries(target_pose_estimation ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(serial_receiver src/serial_receiver.cpp)
target_link_libraries(serial_receiver ${catkin_LIBRARIES})
