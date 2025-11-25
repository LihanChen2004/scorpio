# Copyright 2025 Lihan Chen
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml
from sdformat_tools.urdf_generator import UrdfGenerator
from xmacro.xmacro4sdf import XMLMacro4sdf


def launch_setup(context: LaunchContext) -> list:
    # Using OpaqueFunction in order to get the context in string format...
    # But it is too hacky and not recommended.

    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    visualize_robot_desc = LaunchConfiguration("visualize_robot_desc")
    use_respawn = LaunchConfiguration("use_respawn")
    log_level = LaunchConfiguration("log_level")

    # Load the robot xmacro file from the launch configuration
    xmacro = XMLMacro4sdf()
    xmacro.set_xml_file(context.launch_configurations["robot_xmacro_file"])

    # Generate SDF from xmacro
    xmacro.generate()
    robot_xml = xmacro.to_string()

    # Generate URDF from SDF
    urdf_generator = UrdfGenerator()
    urdf_generator.parse_from_sdf_string(robot_xml)
    robot_urdf_xml = urdf_generator.to_string()

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {"use_sim_time": use_sim_time}

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True,
        ),
        allow_substs=True,
    )

    stdout_linebuf_envvar = SetEnvironmentVariable(
        "RCUTILS_LOGGING_BUFFERED_STREAM", "1"
    )

    colorized_output_envvar = SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1")

    bringup_cmd_group = GroupAction(
        [
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                name="joint_state_publisher",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[
                    configured_params,
                    {"robot_description": robot_urdf_xml},
                ],
                arguments=["--ros-args", "--log-level", log_level],
            ),
            Node(
                condition=IfCondition(visualize_robot_desc),
                package="rviz2",
                executable="rviz2",
                arguments=[
                    "-d",
                    os.path.join(
                        get_package_share_directory("scorpio_description"),
                        "rviz",
                        "visualize_robot.rviz",
                    ),
                ],
                output="screen",
            ),
        ]
    )

    return [
        stdout_linebuf_envvar,
        colorized_output_envvar,
        bringup_cmd_group,
    ]


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory("scorpio_description")

    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Top-level namespace",
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="False",
        description="Use simulation (Gazebo) clock if true",
    )

    declare_robot_name_cmd = DeclareLaunchArgument(
        "robot_name",
        default_value="scorpio",
        description="The file name of the robot xmacro to be used",
    )

    declare_robot_xmacro_file_cmd = DeclareLaunchArgument(
        "robot_xmacro_file",
        default_value=[
            # Use TextSubstitution to concatenate strings
            TextSubstitution(text=os.path.join(bringup_dir, "resource", "xmacro", "")),
            LaunchConfiguration("robot_name"),
            TextSubstitution(text=".sdf.xmacro"),
        ],
        description="The file path of the robot xmacro to be used",
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(bringup_dir, "params", "robot_description.yaml"),
        description="Full path to the ROS2 parameters file to use for all launched nodes",
    )

    declare_visualize_robot_desc_cmd = DeclareLaunchArgument(
        "visualize_robot_desc",
        default_value="True",
        description="Whether to start RViz",
    )

    declare_use_respawn_cmd = DeclareLaunchArgument(
        "use_respawn",
        default_value="False",
        description="Whether to respawn if a node crashes. Applied when composition is disabled.",
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        "log_level", default_value="info", description="log level"
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_robot_xmacro_file_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_visualize_robot_desc_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)

    # Add the actions to launch all of the nodes
    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld
