#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import ReplaceString


def generate_launch_description():
    # Get the urdf file
    sdf_path = os.path.join(
        get_package_share_directory("ugv_arm_sim"),
        "models",
        "ugv_arm",
        "ugv_arm.sdf",
    )

    # Launch configuration variables specific to simulation
    namespace = LaunchConfiguration("namespace", default="ugv_arm")
    x_pose = LaunchConfiguration("x_pose", default="0.0")
    y_pose = LaunchConfiguration("y_pose", default="0.0")
    yaw_pose = LaunchConfiguration("yaw_pose", default="0.0")

    # Declare the launch arguments
    declare_namespace_arg = DeclareLaunchArgument(
        "namespace", default_value=namespace, description="Specify robot namespace"
    )

    declare_x_pose_arg = DeclareLaunchArgument(
        "x_pose", default_value=x_pose, description="Specify robot x position"
    )

    declare_y_pose_arg = DeclareLaunchArgument(
        "y_pose", default_value=y_pose, description="Specify robot y position"
    )

    declare_yaw_pose_arg = DeclareLaunchArgument(
        "yaw_pose", default_value=yaw_pose, description="Specify robot yaw angle"
    )

    # Set GAZEBO environment variables
    install_dir = get_package_prefix("scout_description")
    gazebo_models_path = os.path.join("scout_description", "meshes")

    if "GZ_SIM_RESOURCE_PATH" in os.environ:
        os.environ["GZ_SIM_RESOURCE_PATH"] = (
            os.environ["GZ_SIM_RESOURCE_PATH"] + ":" + install_dir + "/share/"
        )
    else:
        os.environ["GZ_SIM_RESOURCE_PATH"] = install_dir + "/share/"

    os.environ["GZ_SIM_SYSTEM_PLUGIN_PATH"] = ":".join(
        [
            os.environ.get("GZ_SIM_SYSTEM_PLUGIN_PATH", default=""),
            os.environ.get("LD_LIBRARY_PATH", default=""),
        ]
    )

    # To support pre-garden. Deprecated.
    os.environ["IGN_GAZEBO_SYSTEM_PLUGIN_PATH"] = ":".join(
        [
            os.environ.get("IGN_GAZEBO_SYSTEM_PLUGIN_PATH", default=""),
            os.environ.get("LD_LIBRARY_PATH", default=""),
        ]
    )

    # Add namespace to gazebo model file
    namespaced_sdf_file = ReplaceString(
        source_file=os.path.join(
            get_package_share_directory("ugv_arm_sim"),
            "models",
            "ugv_arm",
            "ugv_arm.sdf",
        ),
        replacements={"/robot_namespace": ("/", namespace)},
    )

    # Nodes
    start_gazebo_ros_spawner_cmd = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", "ugv_arm/robot_description",
            "-name",  "ugv_arm",
            "-x", x_pose,
            "-y", y_pose,
            "-z", "0.245",
            "-Y", yaw_pose,
        ],
        output="screen",
    )

    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_namespace_arg)
    ld.add_action(declare_x_pose_arg)
    ld.add_action(declare_y_pose_arg)
    ld.add_action(declare_yaw_pose_arg)

    # Add any conditioned actions
    ld.add_action(start_gazebo_ros_spawner_cmd)

    return ld
