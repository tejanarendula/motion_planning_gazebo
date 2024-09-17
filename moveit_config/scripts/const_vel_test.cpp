#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ur5_arm_controller");
    ros::NodeHandle nh;

    // Joint names for your UR5 robot (replace these with your actual joint names)
    static const std::string PLANNING_GROUP_ARM = "ur5_arm";

    // Create a MoveGroupInterface for the specified planning group
    moveit::planning_interface::MoveGroupInterface move_group_interface_arm(PLANNING_GROUP_ARM);

    // Set the target joint angles
    std::vector<double> target_joint_angles = {0.47, -0.68, -0.14, 5.31, -5.96, 4.78};

    // Set the joint target
    move_group_interface_arm.setJointValueTarget(target_joint_angles);

    // Plan the trajectory
    moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;
    bool success = (move_group_interface_arm.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    if (success)
    {
        // Modify the trajectory to achieve constant velocity
        trajectory_msgs::JointTrajectory& trajectory = my_plan_arm.trajectory_.joint_trajectory;
        double duration = 1.0;  // 1 second
        int num_points = trajectory.points.size();
        double joint_velocity = 0.5;  // Constant joint velocity

        // Calculate time per point to achieve the desired duration
        double time_per_point = duration / num_points;

        // Set constant velocity for each trajectory point
        for (int i = 1; i < num_points; ++i)
        {
            for (size_t j = 0; j < trajectory.joint_names.size(); ++j)
            {
                trajectory.points[i].velocities[j] = joint_velocity;
            }
            trajectory.points[i].time_from_start = ros::Duration(time_per_point * i);
        }

        // Use MoveIt to execute the trajectory with existing controllers
        move_group_interface_arm.move();
        ROS_INFO("Movement completed.");
    }
    else
    {
        ROS_ERROR("Failed to plan the trajectory");
    }

    ros::shutdown();

    return 0;
}
