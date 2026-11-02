from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():

    side_length = LaunchConfiguration("side_length")

    diffdrive_launch = os.path.join(
        get_package_share_directory("diffdrive_bot"),
        "launch",
        "control.launch.py"
    )

    return LaunchDescription([

        # Declare square side length argument
        DeclareLaunchArgument(
            "side_length",
            default_value="0.5",
            description="Length of each square side in metres"
        ),

        # Launch Gazebo and robot control
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(diffdrive_launch)
        ),

        # Start rotation action server
        Node(
            package="my_robot_nodes",
            executable="rotate_server",
            name="rotate_server",
            output="screen"
        ),

        # Start square driver
        Node(
            package="my_robot_nodes",
            executable="square_driver",
            name="square_driver",
            parameters=[{
                "side_length": side_length
            }],
            output="screen"
        ),

    ])
