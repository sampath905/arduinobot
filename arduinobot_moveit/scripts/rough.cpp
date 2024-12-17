#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <memory>
#include <unordered_map>
#include <vector>
#include <thread>
#include <chrono>

void move_robot(const std::shared_ptr<rclcpp::Node> node)
{
    auto arm_move_group = moveit::planning_interface::MoveGroupInterface(node, "arm");
    auto gripper_move_group = moveit::planning_interface::MoveGroupInterface(node, "gripper");

    // Define the joint goals for different poses using a map
    std::unordered_map<std::string, std::vector<double>> arm_poses = {
        {"zero_pose", {0.0, 0.0, 0.0, 0.0, 0.0}},
        {"pick_pose", {0.0, 1.1868, -0.4537, -0.7504, 1.5707}},
        {"opposite_pose", {3.142, 0.026, -0.4166, -0.677, 1.5702}},
        {"place_pose", {3.142, 1.1868, -0.4537, -0.7504, 1.5707}},
        {"straight_up_pose", {0.0, 0.0, -1.572, -1.572, 0.0}},
    };

    std::unordered_map<std::string, std::vector<double>> gripper_poses = {
        {"gripper_close", {0.0, 0.0}},
        {"gripper_open", {0.055, -0.055}},
    };

    // Define the sequence of poses to move through
    std::vector<std::string> pose_sequence = {
        "pick_pose",
        "gripper_open", // Open the gripper before picking
        "gripper_close", // Close the gripper after picking
        "opposite_pose",
        "place_pose",
        "gripper_open",
        "opposite_pose"
  // Open the gripper after placing
    };

    while (rclcpp::ok())  // This will continue until the node is stopped (e.g., via Ctrl+C)
    {
        for (const auto& pose_name : pose_sequence)
        {
            if (arm_poses.find(pose_name) != arm_poses.end())
            {
                // Set arm pose
                bool arm_within_bounds = arm_move_group.setJointValueTarget(arm_poses[pose_name]);
                if (!arm_within_bounds)
                {
                    RCLCPP_WARN(rclcpp::get_logger("rclcpp"),
                                "Target arm joint position(s) were outside of limits, but we will plan and clamp to the limits.");
                    continue;
                }

                // Plan and execute arm movement
                moveit::planning_interface::MoveGroupInterface::Plan arm_plan;
                bool arm_plan_success = (arm_move_group.plan(arm_plan) == moveit::core::MoveItErrorCode::SUCCESS);
                if (arm_plan_success)
                {
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Moving arm to pose: %s", pose_name.c_str());
                    arm_move_group.move();
                }
                else
                {
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Arm planner failed for pose: %s", pose_name.c_str());
                    continue;
                }
            }
            else if (gripper_poses.find(pose_name) != gripper_poses.end())
            {
                // Set gripper pose
                gripper_move_group.setJointValueTarget(gripper_poses[pose_name]);

                // Plan and execute gripper movement
                moveit::planning_interface::MoveGroupInterface::Plan gripper_plan;
                bool gripper_plan_success = (gripper_move_group.plan(gripper_plan) == moveit::core::MoveItErrorCode::SUCCESS);
                if (gripper_plan_success)
                {
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Moving gripper to pose: %s", pose_name.c_str());
                    gripper_move_group.move();
                }
                else
                {
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Gripper planner failed for pose: %s", pose_name.c_str());
                }
            }

            // Wait for 1 second, but 3 seconds if it's the "opposite_pose"
            if (pose_name == "opposite_pose")
            {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Waiting for 3 seconds before next action (opposite_pose).");
                std::this_thread::sleep_for(std::chrono::seconds(3));
            }
            else
            {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Waiting for 1 second before next action.");
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
        }

        // Allow ROS 2 to handle events and callbacks, ensuring that the node remains responsive
        rclcpp::spin_some(node);
    }

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sequence canceled or node shutdown.");
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("simple_moveit_interface");
    
    // Call the move_robot function
    move_robot(node);
    
    // Keep the node alive
    rclcpp::spin(node);
    
    // Shutdown the ROS 2 system
    rclcpp::shutdown();
    return 0;
}