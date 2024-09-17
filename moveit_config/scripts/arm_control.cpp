#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>


int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_group_interface_tutorial");
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

    moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;
    
    // 1. Move to home position
    move_group_interface_arm.setJointValueTarget(move_group_interface_arm.getNamedTargetValues("grasp"));
    
    bool success = (move_group_interface_arm.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    move_group_interface_arm.move();

    ros::Duration(3.0).sleep();

    // 2. Move the arm to the target position
    geometry_msgs::PoseStamped current_pose, current_pose1;
    current_pose = move_group_interface_arm.getCurrentPose("tool0");
    current_pose1 = move_group_interface_arm.getCurrentPose("vacuum_gripper");

    geometry_msgs::Pose pose, pose1;
    pose.orientation = current_pose.pose.orientation;
    pose.position = current_pose.pose.position; 
    pose1.orientation = current_pose1.pose.orientation;
    pose1.position = current_pose1.pose.position;
    std::cout<<pose.position.x<<","<<pose.position.y<<","<<pose.position.z<<std::endl;
    std::cout<<pose.orientation.w<<","<<pose.orientation.x<<","<<pose.orientation.y<<","<<pose.orientation.z<<std::endl;  
    std::cout<<pose1.position.x<<","<<pose1.position.y<<","<<pose1.position.z<<std::endl;
    std::cout<<pose1.orientation.w<<","<<pose1.orientation.x<<","<<pose1.orientation.y<<","<<pose1.orientation.z<<std::endl;
    
    move_group_interface_arm.setEndEffectorLink("vacuum_gripper");
    std::string i = move_group_interface_arm.getEndEffectorLink();
    ROS_INFO("eef %s",i.c_str());
     
    geometry_msgs::Pose target_pose1;
  
    target_pose1.orientation = pose1.orientation;
    target_pose1.position.x = -0.45; // grasp position for  "tool0": x = -0.522345, y = 0.196404, z = 1.04315; for "gripper_link": x = -0.57083, y = 0.206368, z = 1.00305;
    target_pose1.position.y = 1.33;  // grasp position for  "vacuum_gripper": x = -0.528103, y = 0.553516, z = 1.02902; for "vacuum_gripper1": x = -0.506495, y = 0.553314, z = 1.05396; 
    target_pose1.position.z = 1.18;  //for "vacuum_gripper2": x = -0.534653, y = 0.553651, z = 1.04131; for "vacuum_gripper3": x = -0.499945, y = 0.553179, z = 1.04167;
    move_group_interface_arm.setPoseTarget(target_pose1);

    success = (move_group_interface_arm.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    move_group_interface_arm.move();

    // // 3. Move to bar code position
    // move_group_interface_arm.setJointValueTarget(move_group_interface_arm.getNamedTargetValues("bar_code"));
    
    // success = (move_group_interface_arm.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    // ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    // move_group_interface_arm.move();
    
    // // 4. Move to drop position
    // move_group_interface_arm.setJointValueTarget(move_group_interface_arm.getNamedTargetValues("drop"));
    
    // success = (move_group_interface_arm.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    // ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    // move_group_interface_arm.move();

    // 5. Move back to home position
    // move_group_interface_arm.setJointValueTarget(move_group_interface_arm.getNamedTargetValues("home"));
    
    // success = (move_group_interface_arm.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    // ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    // move_group_interface_arm.move();

  ros::shutdown();
  return 0;
}
