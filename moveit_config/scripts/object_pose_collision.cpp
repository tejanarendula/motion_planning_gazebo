#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Pose.h>
#include <iostream>
#include <sensor_msgs/PointCloud2.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>



// Global variable to store the pose of the object
geometry_msgs::Pose object_pose;

// Global variable to store the name of the object
std::string object_name;

// Define the planning scene interface
moveit::planning_interface::PlanningSceneInterface* planning_scene_interface_ptr;

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
    }  else {
        std::cout << "Object '" << object_name << "' not found in model states!" << std::endl;
    }
}

int main(int argc, char** argv) {
    // Initialize ROS node
    ros::init(argc, argv, "object_pose_collision");
    ros::NodeHandle nh;
    
    // Initialize planning scene interface pointer
    planning_scene_interface_ptr = new moveit::planning_interface::PlanningSceneInterface();
    
    // Prompt user to input the name of the object
    std::cout << "Enter the name of the object: ";
    std::cin >> object_name;

    // Subscribe to model states topic
    ros::Subscriber model_states_sub = nh.subscribe("/gazebo/model_states", 1, modelStatesCallback);

    // Spin and wait for callbacks
    ros::spin();

    // Delete the planning scene interface pointer
    delete planning_scene_interface_ptr;

    return 0;
}
