#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayTrajectory.h>

static const std::string PLANNING_GROUP_ARM = "ur5_arm";
moveit::planning_interface::MoveGroupInterface move_group_interface_arm(PLANNING_GROUP_ARM);


void trajectoryCallback(const moveit_msgs::DisplayTrajectory::ConstPtr& msg) {
    // Iterate through all trajectories in the message
    for (const auto& trajectory : msg->trajectory) {
        // Iterate through all joint trajectory points
        for (const auto& point : trajectory.joint_trajectory.points) {
            // Get joint positions
            std::vector<double> joint_positions = point.positions;
            
            // Use forward kinematics to compute gripper pose
            move_group_interface_arm.setJointValueTarget(joint_positions);
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            move_group_interface_arm.plan(plan);
            
            // Extract gripper pose from the end effector link
            geometry_msgs::PoseStamped gripper_pose;
            gripper_pose = move_group_interface_arm.getPoseTarget();
            
            // Extract (x, y, z) coordinates from gripper pose
            double x = gripper_pose.pose.position.x;
            double y = gripper_pose.pose.position.y;
            double z = gripper_pose.pose.position.z;
            
            // Print gripper coordinates
            ROS_INFO_STREAM("Gripper Pose: (" << x << ", " << y << ", " << z << ")");
        }
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "move_group_interface_tutorial");
    ros::NodeHandle n;
  
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // static const std::string PLANNING_GROUP_ARM = "ur5_arm";
    // moveit::planning_interface::MoveGroupInterface move_group_interface_arm(PLANNING_GROUP_ARM);

    // Subscribe to the planned path topic
    ros::Subscriber sub = n.subscribe("/move_group/display_planned_path", 1000, trajectoryCallback);

    ros::waitForShutdown();
    return 0;
}
