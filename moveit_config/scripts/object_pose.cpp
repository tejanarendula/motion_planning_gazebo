#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Pose.h>
#include <iostream>

// Global variable to store the pose of the object
geometry_msgs::Pose object_pose;

// Global variable to store the name of the object
std::string object_name;

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
    } else {
        std::cout << "Object '" << object_name << "' not found in model states!" << std::endl;
    }
}

int main(int argc, char** argv) {
    // Initialize ROS node
    ros::init(argc, argv, "object_pose_listener");
    ros::NodeHandle nh;

    // Prompt user to input the name of the object
    std::cout << "Enter the name of the object: ";
    std::cin >> object_name;

    // Subscribe to model states topic
    ros::Subscriber model_states_sub = nh.subscribe("/gazebo/model_states", 1, modelStatesCallback);

    // Spin and wait for callbacks
    ros::spin();

    return 0;
}
