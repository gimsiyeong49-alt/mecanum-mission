"""지도 만들기 한 번에: robot.launch.py + cartographer.launch.py"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    launch_dir = PathJoinSubstitution([FindPackageShare("mecanum_slam"), "launch"])
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            PathJoinSubstitution([launch_dir, "robot.launch.py"]))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            PathJoinSubstitution([launch_dir, "cartographer.launch.py"]))),
    ])
