#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>


int main(int argc, char** argv)
{
  ros::init(argc, argv, "pose_est_base_link");
  ros::NodeHandle n;
  
  // ROS spinning must be running for the MoveGroupInterface to get information
  // about the robot's state. One way to do this is to start an AsyncSpinner
  // beforehand.

  ros::AsyncSpinner spinner(1);
  spinner.start();

    // MoveIt operates on sets of joints called "planning groups" and stores them in an object called
  // the `JointModelGroup`. Throughout MoveIt the terms "planning group" and "joint model group"
  // are used interchangably.
    static const std::string PLANNING_GROUP_ARM = "ur5_arm";
    static const std::string PLANNING_GROUP_GRIPPER = "grippers";
    
    
    // The :planning_interface:`MoveGroupInterface` class can be easily
    // setup using just the name of the planning group you would like to control and plan for.
    moveit::planning_interface::MoveGroupInterface move_group_interface_arm(PLANNING_GROUP_ARM);
    moveit::planning_interface::MoveGroupInterface move_group_interface_gripper(PLANNING_GROUP_GRIPPER);
    

    // We can get a list of all the groups in the robot:
    ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
    std::copy(move_group_interface_arm.getJointModelGroupNames().begin(),
            move_group_interface_arm.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));
    
    //create a tf2_ros::Buffer and a tf2_ros::TransformListener
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    // wait for the trasnform from world to shoulder_link
    std::string target_frame = "base_link";
    //std::string source_frame = "amr_link";
    
    // Get the current pose in world frame
    geometry_msgs::PoseStamped current_pose, current_pose1, pose_gripper, pose_gripper1, pose_gripper2, pose_gripper3;
    current_pose = move_group_interface_arm.getCurrentPose("tool0");
    current_pose1 = move_group_interface_arm.getCurrentPose("gripper_link");
    pose_gripper = move_group_interface_arm.getCurrentPose("vacuum_gripper");
    pose_gripper1 = move_group_interface_arm.getCurrentPose("vacuum_gripper1");
    pose_gripper2 = move_group_interface_arm.getCurrentPose("vacuum_gripper2");
    pose_gripper3 = move_group_interface_arm.getCurrentPose("vacuum_gripper3");



    geometry_msgs::Pose target_pose1, target_pose2, target_gripper, target_gripper1, target_gripper2, target_gripper3;
    target_pose1.orientation = current_pose.pose.orientation;
    target_pose1.position = current_pose.pose.position;
    std::cout<<target_pose1.position.x<<","<<target_pose1.position.y<<","<<target_pose1.position.z<<std::endl;
    std::cout<<target_pose1.orientation.w<<","<<target_pose1.orientation.x<<","<<target_pose1.orientation.y<<","<<target_pose1.orientation.z<<std::endl;

    target_pose2.orientation = current_pose1.pose.orientation;
    target_pose2.position = current_pose1.pose.position;
    // std::cout<<target_pose2.position.x<<","<<target_pose2.position.y<<","<<target_pose2.position.z<<std::endl;
    // std::cout<<target_pose2.orientation.w<<","<<target_pose2.orientation.x<<","<<target_pose2.orientation.y<<","<<target_pose2.orientation.z<<std::endl;

    target_gripper.orientation = pose_gripper.pose.orientation;
    target_gripper.position = pose_gripper.pose.position;
    // std::cout<<target_gripper.position.x<<","<<target_gripper.position.y<<","<<target_gripper.position.z<<std::endl;
    // std::cout<<target_gripper.orientation.w<<","<<target_gripper.orientation.x<<","<<target_gripper.orientation.y<<","<<target_gripper.orientation.z<<std::endl;

    target_gripper1.orientation = pose_gripper1.pose.orientation;
    target_gripper1.position = pose_gripper1.pose.position;
    // std::cout<<target_gripper1.position.x<<","<<target_gripper1.position.y<<","<<target_gripper1.position.z<<std::endl;
    // std::cout<<target_gripper1.orientation.w<<","<<target_gripper1.orientation.x<<","<<target_gripper1.orientation.y<<","<<target_gripper1.orientation.z<<std::endl;

    target_gripper2.orientation = pose_gripper2.pose.orientation; 
    target_gripper2.position = pose_gripper2.pose.position;
    // std::cout<<target_gripper2.position.x<<","<<target_gripper2.position.y<<","<<target_gripper2.position.z<<std::endl;
    // std::cout<<target_gripper2.orientation.w<<","<<target_gripper2.orientation.x<<","<<target_gripper2.orientation.y<<","<<target_gripper2.orientation.z<<std::endl;

    target_gripper3.orientation = pose_gripper3.pose.orientation;
    target_gripper3.position = pose_gripper3.pose.position;
    // std::cout<<target_gripper3.position.x<<","<<target_gripper3.position.y<<","<<target_gripper3.position.z<<std::endl;
    // std::cout<<target_gripper3.orientation.w<<","<<target_gripper3.orientation.x<<","<<target_gripper3.orientation.y<<","<<target_gripper3.orientation.z<<std::endl;
    
    geometry_msgs::TransformStamped transformStamped;
    try {
    transformStamped = tfBuffer.lookupTransform(target_frame, current_pose.header.frame_id, ros::Time(0), ros::Duration(3.0));
    tf2::doTransform(current_pose, current_pose, transformStamped);
    std::cout << "Transformed current pose in " << target_frame << " frame: " << std::endl;
    std::cout << "Position: " << current_pose.pose.position.x << ", "
              << current_pose.pose.position.y << ", "
              << current_pose.pose.position.z << std::endl;
    std::cout << "Orientation: " << current_pose.pose.orientation.w << ", "
              << current_pose.pose.orientation.x << ", "
              << current_pose.pose.orientation.y << ", "
              << current_pose.pose.orientation.z << std::endl;

    } catch (tf2::TransformException &ex) {
    ROS_WARN("%s",ex.what());
    }

    
    ros::shutdown();
    return 0;
}
