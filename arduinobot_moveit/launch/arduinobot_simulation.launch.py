import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    gazebo = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("arduinobot_description"),
            "launch",
            "gazebo.launch.py"
        ),
    )
    
    controllers = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("arduinobot_controller"),
            "launch",
            "controller.launch.py"
        ),
    )

    moveit = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("arduinobot_moveit"),
            "launch",
            "moveit.launch.py"
        ),
    )

    return LaunchDescription([
        gazebo,
        controllers,
        moveit,
        
    ])