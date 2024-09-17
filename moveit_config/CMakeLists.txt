cmake_minimum_required(VERSION 3.1.3)
project(moveit_config)

find_package(catkin REQUIRED COMPONENTS
   roscpp
   rospy
   std_msgs
   sensor_msgs
   interactive_markers
   moveit_core
   moveit_ros_planning
   moveit_ros_planning_interface
   moveit_ros_perception
   rviz_visual_tools
   moveit_visual_tools
   pluginlib
   geometric_shapes
   pcl_ros
   pcl_conversions
   rosbag
   tf2_ros
   tf2_eigen
   tf2_geometry_msgs
   actionlib_msgs
   control_msgs
   opencv_services
   gazebo_msgs
   geometry_msgs
   std_srvs
   message_generation 
)

## Generate messages in the 'msg' folder
add_message_files(FILES blocks_poses.msg)
add_message_files(FILES Tracker.msg)

generate_messages(DEPENDENCIES std_msgs)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread) 


catkin_package(
LIBRARIES
  INCLUDE_DIRS
  CATKIN_DEPENDS
    moveit_core
    moveit_visual_tools
    moveit_ros_planning_interface
    interactive_markers
    tf2_geometry_msgs
    actionlib_msgs
    control_msgs
    opencv_services
    message_runtime
)

include_directories(SYSTEM ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS})

add_executable(arm_control scripts/arm_control.cpp)
target_link_libraries(arm_control ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_executable(picknplace scripts/picknplace.cpp)
target_link_libraries(picknplace ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_executable(const_vel_test scripts/const_vel_test.cpp)
target_link_libraries(const_vel_test ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_executable(pos_est scripts/pos_est.cpp)
target_link_libraries(pos_est ${catkin_LIBRARIES} ${Boost_LIBRARIES})
##add_executable(open_cv_control scripts/open_cv_control.cpp)
##target_link_libraries(open_cv_control ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_executable(depth_cam scripts/depth_cam.cpp)
target_link_libraries(depth_cam ${catkin_LIBRARIES} ${Boost_LIBRARIES})
##add_executable(blocks_poses_publisher msg/blocks_poses_publisher.cpp)
##target_link_libraries(blocks_poses_publisher ${catkin_LIBRARIES})
add_executable(collision_object scripts/collision_object.cpp)
target_link_libraries(collision_object ${catkin_LIBRARIES})
add_executable(object_pose scripts/object_pose.cpp)
target_link_libraries(object_pose ${catkin_LIBRARIES})
add_executable(object_pose_collision scripts/object_pose_collision.cpp)
target_link_libraries(object_pose_collision ${catkin_LIBRARIES})
add_executable(pick_n_place scripts/pick_n_place.cpp)
target_link_libraries(pick_n_place ${catkin_LIBRARIES})
add_executable(pick_n_place_main scripts/pick_n_place_main.cpp)
target_link_libraries(pick_n_place_main ${catkin_LIBRARIES})
add_executable(pose_est_base_link scripts/pose_est_base_link.cpp)
target_link_libraries(pose_est_base_link ${catkin_LIBRARIES})
add_executable(waypoints scripts/waypoints.cpp)
target_link_libraries(waypoints ${catkin_LIBRARIES})
add_executable(dynamic_planning scripts/dynamic_planning.cpp)
target_link_libraries(dynamic_planning ${catkin_LIBRARIES})


install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(TARGETS arm_control DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

