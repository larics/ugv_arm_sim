#!/usr/bin/env python3

import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
# Imports
import launch_ros.descriptions
import launch.substitutions

def generate_launch_description():
    # Arguments and parameters
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    namespace = LaunchConfiguration("namespace", default="ugv_arm")

    pkg_name = "ugv_arm_sim"


    declare_use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value=use_sim_time,
        description="Use simulation (Gazebo) clock if true",
    )

    declare_namespace_arg = DeclareLaunchArgument(
        "namespace", default_value=namespace, description="Specify robot namespace"
    )

    # Load xacro file
    xacro_path = os.path.join(
        get_package_share_directory("ugv_arm_sim"), "urdf", "ugv_arm.xacro"
    )

    robot_description_raw = xacro.process_file(xacro_path).toxml()

    print(robot_description_raw)

    #print(robot_description)

    # Nodes
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=namespace,
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time, "robot_description": robot_description_raw}
        ],
        arguments=[robot_description_raw],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
    )

    return LaunchDescription(
        [
            declare_use_sim_time_arg,
            declare_namespace_arg,
            robot_state_publisher_node,
        ]
    )
