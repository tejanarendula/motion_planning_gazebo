#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <gazebo_msgs/ModelStates.h>

// for storing the box pose from gazebo model states
geometry_msgs::Pose box_pose;

// Callback function for handling the model states
void modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr& model_states) {
    // Find the index of the box in the model states message
    int box_index = -1;
    for (size_t i = 0; i < model_states->name.size(); ++i) {
        if (model_states->name[i] == "box_red") {
            box_index = i;
            break;
        }
    }

    // If the box is found, extract its pose and orientation
    if (box_index != -1) {
        box_pose.position = model_states->pose[box_index].position;
        box_pose.orientation = model_states->pose[box_index].orientation;
    }
}

void addbox()
{
    // Call the modelStatesCallback to fetch the latest pose and orientation
    ros::NodeHandle nh;
    //ros::Subscriber model_states_sub = nh.subscribe("/gazebo/model_states", 1, modelStatesCallback);
    ros::spinOnce();

    // Define the planning scene interface
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // Print the box pose
    ROS_INFO_STREAM("Box Pose: x=" << box_pose.position.x << ", y=" << box_pose.position.y << ", z=" << box_pose.position.z);
    ROS_INFO_STREAM("Box Orientation: w=" << box_pose.orientation.w << ", x=" << box_pose.orientation.x << ", y=" << box_pose.orientation.y << ", z=" << box_pose.orientation.z);
        
    // Define a collision object ROS message.
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = "world";
    collision_object.id = "box_red";

    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.04;
    primitive.dimensions[1] = 0.12;
    primitive.dimensions[2] = 0.18;

    geometry_msgs::Pose box_pose;
    box_pose.orientation.w = 0.707;
    box_pose.orientation.x = 0.0;
    box_pose.orientation.y = 0.0;
    box_pose.orientation.z = -0.707;
    box_pose.position.x = -0.2775;
    box_pose.position.y = 1.3562;
    box_pose.position.z = 1.4699;

    // Add box as collision object
    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;
    planning_scene_interface.applyCollisionObject(collision_object);
    
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "collision_object");
    ros::NodeHandle nh;
    
    //start the ROS spinner
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::Duration(2.0).sleep();

    // Add the box as a collision object
    addbox();

    return 0;
}