"""Cartographer 2D: /scan + EKF 오돔(/odometry/filtered) → map→odom, /map."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_dir = PathJoinSubstitution([FindPackageShare("mecanum_slam"), "config"])
    return LaunchDescription([
        DeclareLaunchArgument("config_basename", default_value="mecanum_2d.lua"),
        DeclareLaunchArgument("resolution", default_value="0.05"),
        Node(
            package="cartographer_ros",
            executable="cartographer_node",
            name="cartographer_node",
            output="screen",
            arguments=[
                "-configuration_directory", config_dir,
                "-configuration_basename", LaunchConfiguration("config_basename"),
            ],
            remappings=[("odom", "/odometry/filtered")],
        ),
        Node(
            package="cartographer_ros",
            executable="cartographer_occupancy_grid_node",
            name="cartographer_occupancy_grid_node",
            output="screen",
            arguments=["-resolution", LaunchConfiguration("resolution"), "-publish_period_sec", "1.0"],
        ),
    ])
