#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include "opencv_services/box_and_target_position.h"


int main(int argc, char** argv)
{
  ros::init(argc, argv, "open_cv_control");
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

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    robot_model_loader::RobotModelLoaderPtr robot_model_loader(new robot_model_loader::RobotModelLoader("robot_description"));
    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor(new planning_scene_monitor::PlanningSceneMonitor(robot_model_loader));

    // Allow collisions between the gripper and the box to be able to grasp it
    planning_scene_monitor::LockedPlanningSceneRW ls(planning_scene_monitor);
    collision_detection::AllowedCollisionMatrix& acm = ls->getAllowedCollisionMatrixNonConst();
    acm.setEntry("box_red", "gripper_link", true);
    acm.setEntry("box_red", "vacuum_gripper", true);
    acm.setEntry("box_red", "vacuum_gripper1", true);
    acm.setEntry("box_red","vacuum_gripper2", true);
    acm.setEntry("box_red","vacuum_gripper3", true);
    acm.setEntry("box_red", "gripper_link", true);
    acm.setEntry("box_red", "vacuum_gripper", true);
    acm.setEntry("box_red", "vacuum_gripper1", true);
    acm.setEntry("box_red", "vacuum_gripper2", true);
    acm.setEntry("box_red", "vacuum_gripper3", true);
    
    std::cout << "\nAllowedCollisionMatrix:\n";
    acm.print(std::cout);
    moveit_msgs::PlanningScene diff_scene;
    ls->getPlanningSceneDiffMsg(diff_scene);

    planning_scene_interface.applyPlanningScene(diff_scene); 

    ros::Duration(0.5).sleep();

    // We can get a list of all the groups in the robot:
    ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
    std::copy(move_group_interface_arm.getJointModelGroupNames().begin(),
            move_group_interface_arm.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    
    // 1. Move to home position
    move_group_interface_arm.setJointValueTarget(move_group_interface_arm.getNamedTargetValues("home"));
    
    bool success = (move_group_interface_arm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    move_group_interface_arm.move();

    // Get the box and the target position from the opencv node
    ros::ServiceClient box_and_target_position_srv_client = n.serviceClient<opencv_services::box_and_target_position>("box_and_target_position");

    opencv_services::box_and_target_position srv;
  
    if(box_and_target_position_srv_client.call(srv)) {
      //ROS_INFO_STREAM("3d target position camera frame: x " << srv.response.target_position.x << " y " << srv.response.target_position.y << " z " << srv.response.target_position.z);
      ROS_INFO_STREAM("3d box position camera frame: x " << srv.response.box_position.x << " y " << srv.response.box_position.y << " z " << srv.response.box_position.z);
    } else {
      ROS_INFO_STREAM("Failed to call box and target position service");
    }

        // Add the object to be grasped (the suqare box) to the planning scene
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = move_group_interface_arm.getPlanningFrame();

    collision_object.id = "box_red";

    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.04;
    primitive.dimensions[1] = 0.12;
    primitive.dimensions[2] = 0.18;

    geometry_msgs::Pose box_pose;
    box_pose.orientation.w = 0.707;
    box_pose.orientation.z = -0.707;
    box_pose.position.x = srv.response.box_position.x;
    box_pose.position.y = srv.response.box_position.y;
    box_pose.position.z = srv.response.box_position.z;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(collision_object);

    planning_scene_interface.applyCollisionObjects(collision_objects);

    ROS_INFO_NAMED("tutorial", "Add an object into the world");

    ros::Duration(0.1).sleep();

    // 2. Move to the box position
    geometry_msgs::PoseStamped current_pose;
    move_group_interface_arm.setEndEffectorLink("vacuum_gripper");
    std::string i = move_group_interface_arm.getEndEffectorLink();
    ROS_INFO("eef %s",i.c_str());
    
    current_pose = move_group_interface_arm.getCurrentPose("vacuum_gripper");

    geometry_msgs::Pose target_pose1;
  
    // target_pose1.orientation.w = 0.5;
    // target_pose1.orientation.y = 0.5;
    target_pose1.position.x = srv.response.box_position.x;
    target_pose1.position.y = srv.response.box_position.y-0.25;
    target_pose1.position.z = srv.response.box_position.z;
    move_group_interface_arm.setPoseTarget(target_pose1);

    success = (move_group_interface_arm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("tutorial", "Goal position x:%f y:%f z:%f", target_pose1.position.x, target_pose1.position.y, target_pose1.position.z);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    move_group_interface_arm.move();


    // 3. Move back to home position
    // move_group_interface_arm.setJointValueTarget(move_group_interface_arm.getNamedTargetValues("home"));
    
    // success = (move_group_interface_arm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    // ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    // move_group_interface_arm.move();

    // ROS_INFO_NAMED("tutorial", "Remove the object from the world");
    // std::vector<std::string> object_ids;
    // object_ids.push_back(collision_object.id);
    // planning_scene_interface.removeCollisionObjects(object_ids);

  ros::shutdown();
  return 0;
}