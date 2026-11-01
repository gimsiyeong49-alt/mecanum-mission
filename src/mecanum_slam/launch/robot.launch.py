"""메카넘 하위 구동 + 라이다 + base_link→laser TF.

mecanum_bringup 의 URDF·controllers.yaml·ekf.yaml 을 그대로 쓰고,
외부 카메라용 Global EKF(map→odom)만 뺀다. map→odom 은 SLAM/위치추정이 낸다.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

ARDUINO_PORT = "/dev/serial/by-path/platform-3610000.usb-usb-0:2.4:1.0-port0"
IMU_PORT = "/dev/serial/by-path/platform-3610000.usb-usb-0:2.3:1.0-port0"
LIDAR_PORT = "/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_c4efdaad6cdf46439fd83d2dd53ef425-if00-port0"


def generate_launch_description():
    bringup = FindPackageShare("mecanum_bringup")
    serial_port = LaunchConfiguration("serial_port")

    robot_description = ParameterValue(
        Command([
            FindExecutable(name="xacro"), " ",
            PathJoinSubstitution([bringup, "urdf", "mecanum_robot.urdf.xacro"]),
            " serial_port:=", serial_port,
        ]),
        value_type=str,
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[
            {"robot_description": robot_description},
            PathJoinSubstitution([bringup, "config", "controllers.yaml"]),
        ],
    )

    ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[PathJoinSubstitution([bringup, "config", "ekf.yaml"])],
        remappings=[("odometry/filtered", "/odometry/filtered")],
    )

    imu = Node(
        package="mecanum_bringup",
        executable="witmotion_ble_imu",
        name="witmotion_ble_imu",
        output="screen",
        parameters=[{"port": IMU_PORT, "baud": 115200, "frame_id": "imu_link"}],
    )

    cmd_vel_bridge = Node(
        package="mecanum_bringup",
        executable="cmd_vel_bridge",
        name="cmd_vel_bridge",
        output="screen",
    )

    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["mecanum_drive_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    lidar = Node(
        package="sllidar_ros2",
        executable="sllidar_node",
        name="sllidar_node",
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_lidar")),
        parameters=[{
            "channel_type": "serial",
            "serial_port": LIDAR_PORT,
            "serial_baudrate": 256000,
            "frame_id": "laser",
            "inverted": False,
            "angle_compensate": True,
            # Sensitivity 는 가까운(약 30 cm 안) 검은 벽을 버림 → 옆 칸 벽이 안 보였음. Boost 는 18~20 cm 부터 잡음 (10-21 같은 자리 비교)
            "scan_mode": LaunchConfiguration("lidar_mode"),
        }],
    )

    # 라이다 쪽이 로봇 정면(+x). yaw 는 라이다 0° 와 차체 정면의 차이(실측으로 맞춤).
    laser_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_laser_tf",
        arguments=[
            "--x", LaunchConfiguration("lidar_x"),
            "--y", LaunchConfiguration("lidar_y"),
            "--z", LaunchConfiguration("lidar_z"),
            "--yaw", LaunchConfiguration("lidar_yaw"),
            "--frame-id", "base_link",
            "--child-frame-id", "laser",
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("serial_port", default_value=ARDUINO_PORT),
        DeclareLaunchArgument("use_lidar", default_value="true"),
        DeclareLaunchArgument("lidar_x", default_value="0.095"),
        DeclareLaunchArgument("lidar_y", default_value="0.0"),
        DeclareLaunchArgument("lidar_z", default_value="0.16"),
        DeclareLaunchArgument("lidar_yaw", default_value="1.641"),
        DeclareLaunchArgument("lidar_mode", default_value="Boost"),
        robot_state_publisher,
        controller_manager,
        cmd_vel_bridge,
        imu,
        ekf,
        lidar,
        laser_tf,
        jsb_spawner,
        RegisterEventHandler(OnProcessExit(target_action=jsb_spawner, on_exit=[drive_spawner])),
    ])
