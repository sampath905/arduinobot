#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <memory>
#include <unordered_map>
#include <vector>
#include <thread>
#include <chrono>
#include <iostream>

void move_robot(const std::shared_ptr<rclcpp::Node> node)
{
    auto arm_move_group = moveit::planning_interface::MoveGroupInterface(node, "arm");
    auto gripper_move_group = moveit::planning_interface::MoveGroupInterface(node, "gripper");

    // Define the joint goals for different poses using a map
    std::unordered_map<int, std::vector<double>> arm_poses = {
        {5, {0.0, 0.0, 0.0, 0.0, 0.0}},        // zero_pose
        {2, {0.0, 1.1860, -0.4537, -0.7504, 1.5707}},  // pick_pose
        {4, {3.142, 0.026, -0.4166, -0.677, 1.5702}},  // opposite_pose
        {8, {3.142, 1.1860, -0.4537, -0.7504, 1.5707}}, // place_pose
        {6, {0.0, 0.026, -0.4166, -0.677, 1.5702}}, // retract pose
    };

    std::unordered_map<int, std::vector<double>> gripper_poses = {
        {1, {0.055, -0.055}}, // gripper_open
        {3, {0.0, 0.0}},      // gripper_close
    };

    while (rclcpp::ok())  // This will continue until the node is stopped (e.g., via Ctrl+C)
    {
        // Prompt the user for a pose choice
        std::string input;
        std::cout << "Enter the pose or action number (1-7) or 'q' to cancel:\n"
                  << "5 - zero_pose\n"
                  << "2 - pick_pose\n"
                  << "4 - opposite_pose\n"
                  << "8 - place_pose\n"
                  << "6 - retract_pose\n"
                  << "1 - gripper_open\n"
                  << "3 - gripper_close\n"
                  << "q - Cancel\n"
                  << "Choice: ";
        std::cin >> input;

        // Check if the user wants to cancel
        if (input == "q" || input == "Q")
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sequence canceled by user.");
            break;
        }

        // Convert input to integer
        int choice;
        try
        {
            choice = std::stoi(input);
        }
        catch (const std::exception &)
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Invalid input! Please enter a valid number or 'q' to cancel.");
            continue;
        }

        // Check if input is valid
        if (arm_poses.find(choice) != arm_poses.end())  // For arm poses
        {
            // Set arm pose
            arm_move_group.setJointValueTarget(arm_poses[choice]);

            moveit::planning_interface::MoveGroupInterface::Plan arm_plan;
            if (arm_move_group.plan(arm_plan) == moveit::core::MoveItErrorCode::SUCCESS)
            {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Moving arm to pose %d", choice);
                arm_move_group.move();
            }
            else
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Arm planner failed for pose %d", choice);
            }
        }
        else if (gripper_poses.find(choice) != gripper_poses.end())  // For gripper poses
        {
            // Set gripper pose
            gripper_move_group.setJointValueTarget(gripper_poses[choice]);

            moveit::planning_interface::MoveGroupInterface::Plan gripper_plan;
            if (gripper_move_group.plan(gripper_plan) == moveit::core::MoveItErrorCode::SUCCESS)
            {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Moving gripper to pose %d", choice);
                gripper_move_group.move();
            }
            else
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Gripper planner failed for pose %d", choice);
            }
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Invalid choice! Please enter a number between 1 and 7 or 'q' to cancel.");
            continue;
        }

        // Sleep for a short time to allow for user input
        std::this_thread::sleep_for(std::chrono::seconds(1));
        // Allow ROS 2 to handle events and callbacks, ensuring that the node remains responsive
        rclcpp::spin_some(node);
    }

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Shutting down...");
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("simple_moveit_interface");

    // Call the move_robot function
    move_robot(node);

    // Shutdown the ROS 2 system
    rclcpp::shutdown();
    return 0;
}
