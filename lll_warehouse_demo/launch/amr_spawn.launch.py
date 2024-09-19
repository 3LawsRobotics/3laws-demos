#!/usr/bin/python3

from os.path import join

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node


def generate_launch_description():
    this_package_path = get_package_share_directory("lll_warehouse_demo")
    camera_enabled = LaunchConfiguration("camera_enabled")
    stereo_camera_enabled = LaunchConfiguration("stereo_camera_enabled", default=False)
    two_d_lidar_enabled = LaunchConfiguration("two_d_lidar_enabled", default=True)
    conveyor_enabled = LaunchConfiguration("conveyor_enabled")

    desc = LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("camera_enabled", default_value="True"),
            DeclareLaunchArgument("stereo_camera_enabled", default_value="False"),
            DeclareLaunchArgument("two_d_lidar_enabled", default_value="True"),
            DeclareLaunchArgument("conveyor_enabled", default_value="False"),
            DeclareLaunchArgument("x0", default_value="-2.0"),
            DeclareLaunchArgument("y0", default_value="0.0"),
            DeclareLaunchArgument("yaw0", default_value="-1.57079632679"),
            DeclareLaunchArgument("supervisor_config", default_value="amr_supervisor_config.yaml"),
            DeclareLaunchArgument("robot_name", default_value="amr"),
            DeclareLaunchArgument("with_box", default_value="True"),
        ]
    )

    desc.add_action(
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[
                {
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "robot_description": Command(
                        [
                            "xacro ",
                            join(this_package_path, "urdf", "amr.xacro"),
                            " camera_enabled:=",
                            camera_enabled,
                            " stereo_camera_enabled:=",
                            stereo_camera_enabled,
                            " two_d_lidar_enabled:=",
                            two_d_lidar_enabled,
                            " conveyor_enabled:=",
                            conveyor_enabled,
                            " robot_namespace:=",
                            LaunchConfiguration("robot_name"),
                        ]
                    ),
                }
            ],
        )
    )

    desc.add_action(
        Node(
            package="ros_gz_sim",
            executable="create",
            arguments=[
                "-topic",
                "robot_description",
                "-name",
                LaunchConfiguration("robot_name"),
                "-allow_renaming",
                "true",
                "-x",
                LaunchConfiguration("x0"),
                "-y",
                LaunchConfiguration("y0"),
                "-z",
                "0.05",
                "-Y",
                LaunchConfiguration("yaw0"),
            ],
        )
    )

    desc.add_action(
        Node(
            package="ros_gz_sim",
            executable="create",
            arguments=[
                "-file",
                join(this_package_path, "models", "cardboard_box", "model.sdf"),
                "-name",
                [LaunchConfiguration("robot_name"), "_box"],
                "-allow_renaming",
                "true",
                "-x",
                LaunchConfiguration("x0"),
                "-y",
                LaunchConfiguration("y0"),
                "-z",
                "0.5",
                "-Y",
                LaunchConfiguration("yaw0"),
            ],
            condition=IfCondition(LaunchConfiguration("with_box")),
        ),
    )

    desc.add_action(
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=[
                "--x",
                "0.0",
                "--y",
                "0.0",
                "--z",
                "0.0",
                "--yaw",
                "0.0",
                "--pitch",
                "0.0",
                "--roll",
                "0.0",
                "--frame-id",
                "kinect_camera",
                "--child-frame-id",
                [LaunchConfiguration("robot_name"), "/base_footprint/kinect_camera"],
            ],
        )
    )

    desc.add_action(
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=[
                [
                    LaunchConfiguration("robot_name"),
                    "/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist",
                ],
                [
                    LaunchConfiguration("robot_name"),
                    "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
                ],
                [
                    LaunchConfiguration("robot_name"),
                    "/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry",
                ],
                [
                    LaunchConfiguration("robot_name"),
                    "/wheel_odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry",
                ],
                "/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V",
                [
                    LaunchConfiguration("robot_name"),
                    "/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
                ],
                [
                    LaunchConfiguration("robot_name"),
                    "/kinect_camera@sensor_msgs/msg/Image[ignition.msgs.Image",
                ],
                [
                    LaunchConfiguration("robot_name"),
                    "/stereo_camera/left/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image",
                ],
                [
                    LaunchConfiguration("robot_name"),
                    "/stereo_camera/right/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image",
                ],
                [
                    LaunchConfiguration("robot_name"),
                    "/kinect_camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo",
                ],
                [
                    LaunchConfiguration("robot_name"),
                    "/stereo_camera/left/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo",
                ],
                [
                    LaunchConfiguration("robot_name"),
                    "/stereo_camera/right/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo",
                ],
                [
                    LaunchConfiguration("robot_name"),
                    "/kinect_camera/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked",
                ],
                [LaunchConfiguration("robot_name"), "/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU"],
                [
                    "/world/default/model/",
                    LaunchConfiguration("robot_name"),
                    "/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model",
                ],
            ],
            remappings=[
                (
                    ["/world/default/model/", LaunchConfiguration("robot_name"), "/joint_state"],
                    "joint_states",
                ),
                ([LaunchConfiguration("robot_name"), "/odom"], "odom"),
                ([LaunchConfiguration("robot_name"), "/wheel_odom"], "wheel_odom"),
                ([LaunchConfiguration("robot_name"), "/scan"], "scan"),
                ([LaunchConfiguration("robot_name"), "/kinect_camera"], "kinect_camera"),
                (
                    [LaunchConfiguration("robot_name"), "/stereo_camera/left/image_raw"],
                    "stereo_camera/left/image_raw",
                ),
                (
                    [LaunchConfiguration("robot_name"), "/stereo_camera/right/image_raw"],
                    "stereo_camera/right/image_raw",
                ),
                ([LaunchConfiguration("robot_name"), "/imu"], "imu"),
                ([LaunchConfiguration("robot_name"), "/cmd_vel"], "cmd_vel"),
                (
                    [LaunchConfiguration("robot_name"), "/kinect_camera/camera_info"],
                    "kinect_camera/camera_info",
                ),
                (
                    [LaunchConfiguration("robot_name"), "/stereo_camera/left/camera_info"],
                    "stereo_camera/left/camera_info",
                ),
                (
                    [LaunchConfiguration("robot_name"), "/stereo_camera/right/camera_info"],
                    "stereo_camera/right/camera_info",
                ),
                (
                    [LaunchConfiguration("robot_name"), "/kinect_camera/points"],
                    "kinect_camera/points",
                ),
            ],
        )
    )

    desc.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                join(
                    get_package_share_directory("lll_supervisor"),
                    "launch",
                    "supervisor.launch.py",
                )
            ),
            launch_arguments={
                "config_filepath": PathJoinSubstitution(
                    [
                        this_package_path,
                        "config",
                        LaunchConfiguration("supervisor_config"),
                    ]
                ),
                "log_filepath": "",
            }.items(),
        )
    )

    desc.add_action(
        Node(
            package="lll_teleop_twist_keyboard",
            executable="lll_teleop_twist_keyboard",
            output="screen",
            prefix="xterm -e",
            emulate_tty=True,
            parameters=[
                {
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "keyboard_publish_rate_ms": 50,
                    "cmd_output_topic_name": "cmd_vel_des",
                    "vx_min": -1.5,
                    "vx_max": 1.5,
                    "vy_min": -0.5,
                    "vy_max": 0.5,
                    "wz_min": -2.0,
                    "wz_max": 2.0,
                }
            ],
        )
    )

    return desc
