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
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap/OcTree.h>

// Global variables to store the pose and name of the object
geometry_msgs::Pose object_pose;
std::string object_name;

// Define the planning scene interface
moveit::planning_interface::PlanningSceneInterface* planning_scene_interface_ptr;

// Map to store dimensions for different objects
std::map<std::string, std::vector<double>> object_dimensions_map = {
    {"box_red", {0.04, 0.12, 0.12}},
    {"box_green", {0.04, 0.12, 0.18}},
    {"box_blue", {0.04, 0.2, 0.15}},
    {"box_red_clone", {0.04, 0.12, 0.12}},
    {"box_green_clone", {0.04, 0.12, 0.18}},
    {"box_blue_clone", {0.04, 0.2, 0.1}}
};

// Global variables for octomap
octomap::OcTree* initial_octomap = nullptr;
octomap::OcTree* current_octomap = nullptr;

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
            if (planning_scene_interface_ptr != nullptr) {
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
                planning_scene_interface_ptr->applyCollisionObject(collision_object);
            }
        } else {
            std::cout << "Dimensions not found for object '" << object_name << "'" << std::endl;
        }
    } else {
        std::cout << "Object '" << object_name << "' not found in model states!" << std::endl;
    }
}

// Callback function for octomap
void octomapCallback(const octomap_msgs::Octomap::ConstPtr& msg) {
    // Convert the octomap message to an OcTree
    octomap::AbstractOcTree* tree = octomap_msgs::fullMsgToMap(*msg);
    if (tree) {
        delete current_octomap;
        current_octomap = dynamic_cast<octomap::OcTree*>(tree);
        if (!initial_octomap) {
            initial_octomap = new octomap::OcTree(*current_octomap);
        }
    }
}

// Function to check for dynamic changes in the environment
bool checkForDynamicChanges() {
    if (initial_octomap && current_octomap) {
        // Check for new occupied nodes in current octomap that were not in initial octomap
        for (octomap::OcTree::leaf_iterator it = current_octomap->begin_leafs(),
             end = current_octomap->end_leafs(); it != end; ++it) {
            // Convert OcTreeKey to point3d
            octomap::point3d point = current_octomap->keyToCoord(it.getKey());

            if (!initial_octomap->search(point)) {
                // New leaf in the current map
                return true;
            }
        }
    }
    return false;
}

// Function to control the arm
void control_manipulator() {
    // Initialize MoveIt! node
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // MoveIt operates on sets of joints called "planning groups" 
    static const std::string PLANNING_GROUP_ARM = "ur5_arm";
    static const std::string PLANNING_GROUP_GRIPPER = "grippers";
    
    // The :planning_interface:`MoveGroupInterface` class can be easily
    // setup using just the name of the planning group you would like to control and plan for.
    moveit::planning_interface::MoveGroupInterface move_group_interface_arm(PLANNING_GROUP_ARM);
    moveit::planning_interface::MoveGroupInterface move_group_interface_gripper(PLANNING_GROUP_GRIPPER);

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    
    // 1. Move to grasp position
    move_group_interface_arm.setJointValueTarget(move_group_interface_arm.getNamedTargetValues("start"));
    move_group_interface_arm.move();

    ros::Duration(3.0).sleep();

    robot_model_loader::RobotModelLoaderPtr robot_model_loader(new robot_model_loader::RobotModelLoader("robot_description"));
    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor(new planning_scene_monitor::PlanningSceneMonitor(robot_model_loader));

    //Allow collisions between the gripper and the object to be able to grasp it
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
    
    // 2. Move the arm to the target position
    move_group_interface_arm.setEndEffectorLink("vacuum_gripper");
    geometry_msgs::PoseStamped current_pose = move_group_interface_arm.getCurrentPose("vacuum_gripper");

    geometry_msgs::Pose target_pose;
    target_pose.position.x = object_pose.position.x;
    target_pose.position.y = object_pose.position.y;
    target_pose.position.z = object_pose.position.z;
    target_pose.orientation = current_pose.pose.orientation;
    move_group_interface_arm.setPoseTarget(target_pose);

    std::cout << "Target Pose:"  << target_pose.position.x << ", " 
              << target_pose.position.y << ", " << target_pose.position.z << std::endl;

    // Plan and execute the motion
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group_interface_arm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (success) {
        ROS_INFO("Planning successful. Executing motion...");
        move_group_interface_arm.execute(my_plan);

        // Monitor for dynamic changes during execution
        while (ros::ok()) {
            if (checkForDynamicChanges()) {
                ROS_WARN("Dynamic change detected. Replanning...");

                // Stop the current motion
                move_group_interface_arm.stop();

                // Replan the motion
                success = (move_group_interface_arm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
                if (success) {
                    ROS_INFO("Replanning successful. Executing new motion...");
                    move_group_interface_arm.execute(my_plan);
                } else {
                    ROS_ERROR("Failed to replan motion!");
                    break;
                }
            }

            ros::Duration(1.0).sleep();  // Adjust the sleep duration as needed
        }
    } else {
        ROS_ERROR("Failed to plan motion!");
    }
}

int main(int argc, char** argv) {
    // Initialize ROS node
    ros::init(argc, argv, "pick_n_place");
    ros::NodeHandle nh;

    // Initialize planning scene interface pointer
    planning_scene_interface_ptr = new moveit::planning_interface::PlanningSceneInterface();
    
    // Prompt user to input the name of the object
    std::cout << "Enter the name of the object: ";
    std::cin >> object_name;

    // Subscribe to model states topic
    ros::Subscriber model_states_sub = nh.subscribe("/gazebo/model_states", 1, modelStatesCallback);

    // Subscribe to octomap topic
    ros::Subscriber octomap_sub = nh.subscribe("/octomap_full", 1, octomapCallback);

    ros::Duration(5.0).sleep();

    // Call the arm control function
    control_manipulator();

    // Delete the planning scene interface pointer
    delete planning_scene_interface_ptr;
    delete initial_octomap;
    delete current_octomap;

    return 0;
}
