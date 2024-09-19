#!/usr/bin/python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import PushRosNamespace


def generate_launch_description():
    this_package_path = get_package_share_directory("lll_warehouse_demo")
    gz_sim_share = get_package_share_directory("ros_gz_sim")

    desc = LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "world_file", default_value=os.path.join(this_package_path, "worlds", "depot.sdf")
            ),
            AppendEnvironmentVariable(
                name="IGN_GAZEBO_RESOURCE_PATH", value=os.path.join(this_package_path, "worlds")
            ),
            AppendEnvironmentVariable(
                name="IGN_GAZEBO_RESOURCE_PATH", value=os.path.join(this_package_path, "models")
            ),
        ]
    )

    desc.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gz_sim_share, "launch", "gz_sim.launch.py")),
            launch_arguments={
                "gz_args": PythonExpression(["'", LaunchConfiguration("world_file"), " -r'"])
            }.items(),
        )
    )

    desc.add_action(
        GroupAction(
            [
                PushRosNamespace("amr"),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(this_package_path, "launch", "amr_spawn.launch.py")
                    ),
                    launch_arguments={
                        "x0": "-2.0",
                        "y0": "2.0",
                        "yaw0": "-1.57",
                        "robot_name": "amr",
                        "supervisor_config": "amr_supervisor_config_teleop.yaml",
                    }.items(),
                ),
            ]
        )
    )

    return desc
