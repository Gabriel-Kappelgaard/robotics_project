
"""Run simple_move using the existing UR MoveIt configuration."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
import os


def generate_launch_description():

    ur_type = LaunchConfiguration("ur_type")
    robot_ip = LaunchConfiguration("robot_ip")

    ur_moveit_launch = os.path.join(
        get_package_share_directory("ur_moveit_config"),
        "launch",
        "ur_moveit.launch.py",
    )

    # Start the standard UR MoveIt configuration.
    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ur_moveit_launch),
        launch_arguments={
            "ur_type": ur_type,
            "robot_ip": robot_ip,
            "launch_rviz": "false",
        }.items(),
    )

    # Python node using pymoveit2.
    simple_move = Node(
        package="ur_app_demo",
        executable="simple_move",
        name="simple_move",
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "ur_type",
            default_value="ur5e",
        ),

        DeclareLaunchArgument(
            "robot_ip",
            default_value="192.168.0.10",
        ),

        moveit,

        # Start simple_move after MoveIt has been launched.
        simple_move,
    ])
