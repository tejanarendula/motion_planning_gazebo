#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Pose.h>
#include <iostream>
#include <sensor_msgs/PointCloud2.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>


// Global variable to store the pose of the object
geometry_msgs::Pose object_pose;

// Global variable to store the name of the object
std::string object_name;

// Define the planning scene interface
moveit::planning_interface::PlanningSceneInterface planning_scene_interface;


// Map to store dimensions for different objects
std::map<std::string, std::vector<double>> object_dimensions_map = {
    {"box_red", {0.04, 0.12, 0.18}},
    {"box_green", {0.04, 0.12, 0.12}},
    {"box_blue", {0.04, 0.2, 0.15}},
    {"box_red_clone", {0.04, 0.12, 0.18}},
    {"box_green_clone", {0.04, 0.12, 0.12}},
    {"box_blue_clone", {0.04, 0.2, 0.1}}
};

// Callback function for model states
void modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr& msg) {
    // Find the index of the object in the model states message
    int object_index = -1;
    for (size_t i = 0; i < msg->name.size(); ++i) {
        if (msg->name[i] == object_name) {
            object_index = i;
        break;
        }
    }

    // If the object is found, extract its pose
    if (object_index != -1) {
        object_pose = msg->pose[object_index];
        std::cout << "Object Pose:" << std::endl;
        std::cout << "Position (x, y, z): " << object_pose.position.x << ", " 
                  << object_pose.position.y << ", " << object_pose.position.z << std::endl;
        std::cout << "Orientation (x, y, z, w): " << object_pose.orientation.x << ", " 
                  << object_pose.orientation.y << ", " << object_pose.orientation.z << ", "
                  << object_pose.orientation.w << std::endl;
        
        // Check if object name exists in the dimensions map
        if (object_dimensions_map.find(object_name) != object_dimensions_map.end()) {
            // Get dimensions from map
            std::vector<double> dimensions = object_dimensions_map[object_name];

        
            // Add the box as a collision object
            
            moveit_msgs::CollisionObject collision_object;
            collision_object.header.frame_id = "world";
            collision_object.id = object_name;

            shape_msgs::SolidPrimitive primitive;
            primitive.type = primitive.BOX;
            primitive.dimensions.resize(3);
            primitive.dimensions[0] = dimensions[0];
            primitive.dimensions[1] = dimensions[1];
            primitive.dimensions[2] = dimensions[2];

            collision_object.primitives.push_back(primitive);
            collision_object.primitive_poses.push_back(object_pose);
            collision_object.operation = collision_object.ADD;
            planning_scene_interface.applyCollisionObject(collision_object);
            
        } else {
            std::cout << "Dimensions not found for object '" << object_name << "'" << std::endl;
            }
    }  else {
        std::cout << "Object '" << object_name << "' not found in model states!" << std::endl;
    }
}

void arm_control(){
    
    // MoveIt operates on sets of joints called "planning groups" 
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
    
    robot_model_loader::RobotModelLoaderPtr robot_model_loader(new robot_model_loader::RobotModelLoader("robot_description"));
    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor(new planning_scene_monitor::PlanningSceneMonitor(robot_model_loader));

    // Allow collisions between the gripper and the object to be able to grasp it
    planning_scene_monitor::LockedPlanningSceneRW ls(planning_scene_monitor);
    collision_detection::AllowedCollisionMatrix& acm = ls->getAllowedCollisionMatrixNonConst();
    acm.setEntry("object_name", "gripper_link", true);
    acm.setEntry("object_name", "vacuum_gripper", true);
    acm.setEntry("object_name", "vacuum_gripper1", true);
    acm.setEntry("object_name","vacuum_gripper2", true);
    acm.setEntry("object_name","vacuum_gripper3", true);
    std::cout << "\nAllowedCollisionMatrix:\n";
    acm.print(std::cout);
    moveit_msgs::PlanningScene diff_scene;
    ls->getPlanningSceneDiffMsg(diff_scene);

    planning_scene_interface.applyPlanningScene(diff_scene); 

    ros::Duration(1.0).sleep();
    
    // 1. Move to grasp position
    move_group_interface_arm.setJointValueTarget(move_group_interface_arm.getNamedTargetValues("grasp"));
    
    bool success = (move_group_interface_arm.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    move_group_interface_arm.move();

    ros::Duration(3.0).sleep();

    // 2. Move the arm to the target position
    move_group_interface_arm.setEndEffectorLink("vacuum_gripper");
    std::string i = move_group_interface_arm.getEndEffectorLink();
    ROS_INFO("eef %s",i.c_str());

    geometry_msgs::PoseStamped current_pose;
    current_pose = move_group_interface_arm.getCurrentPose("vacuum_gripper");
    

    geometry_msgs::Pose target_pose;
    target_pose.position.x = object_pose.position.x - 0.35;
    target_pose.position.y = object_pose.position.y;
    target_pose.position.z = object_pose.position.z;
    //target_pose.orientation = move_group_interface_arm.getCurrentPose("grasp").pose.orientation;
    target_pose.orientation = current_pose.pose.orientation;
}

int main(int argc, char** argv) {
    // Initialize ROS node
    ros::init(argc, argv, "pick_n_place");
    ros::NodeHandle nh;
    
    // Prompt user to input the name of the object
    std::cout << "Enter the name of the object: ";
    std::cin >> object_name;

    // Subscribe to model states topic
    ros::Subscriber model_states_sub = nh.subscribe("/gazebo/model_states", 1, modelStatesCallback);
    ros::Duration(5.0).sleep();

    // Call the arm control function
    arm_control();

    // Spin and wait for callbacks
    ros::spin();

    return 0;
}
