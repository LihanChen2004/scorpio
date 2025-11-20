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

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from sdformat_tools.urdf_generator import UrdfGenerator
from xmacro.xmacro4sdf import XMLMacro4sdf


def generate_launch_description():
    # Get package directories
    scorpio_description_dir = get_package_share_directory("scorpio_description")
    scorpio_simulator_dir = get_package_share_directory("scorpio_simulator")

    # Define file paths
    robot_xmacro_file = os.path.join(
        scorpio_description_dir,
        "resource",
        "xmacro",
        "scorpio.sdf.xmacro",
    )

    ros2_control_urdf_file = os.path.join(
        scorpio_description_dir,
        "resource",
        "scorpio_ros2_control_config",
        "ros2_control.urdf",
    )

    ros2_controllers_config = os.path.join(
        scorpio_description_dir,
        "resource",
        "scorpio_ros2_control_config",
        "ackermann_drive_controller.yaml",
    )

    bridge_config_file = os.path.join(
        scorpio_simulator_dir, "config", "ros_gz_bridge.yaml"
    )

    # Load robot spawn configuration
    gz_world_config_file = os.path.join(
        scorpio_simulator_dir, "config", "gz_world.yaml"
    )

    with open(gz_world_config_file) as file:
        world_config = yaml.safe_load(file)
        selected_world = world_config.get("world")
        robots_config = world_config["robots"].get(selected_world)

    # Initialize xmacro processor
    xmacro_processor = XMLMacro4sdf()
    xmacro_processor.set_xml_file(robot_xmacro_file)

    # Create launch description
    ld = LaunchDescription()

    # Process each robot configuration
    for robot_config in robots_config:
        # Generate SDF from xmacro
        xmacro_processor.generate(
            {"ros2_control_parameters_file_path": ros2_controllers_config}
        )
        robot_sdf_xml = xmacro_processor.to_string()

        # Generate URDF from SDF
        urdf_generator = UrdfGenerator()
        urdf_generator.parse_from_sdf_string(robot_sdf_xml)
        urdf_generator.merge_urdf_file(ros2_control_urdf_file)
        robot_urdf_xml = urdf_generator.to_string()

        # Create robot spawn node
        spawn_robot_node = Node(
            package="ros_gz_sim",
            executable="create",
            arguments=[
                "-string",
                robot_sdf_xml,
                "-name",
                robot_config["name"],
                "-allow_renaming",
                "true",
                "-x",
                robot_config["x_pose"],
                "-y",
                robot_config["y_pose"],
                "-z",
                robot_config["z_pose"],
                "-Y",
                robot_config["yaw"],
            ],
        )

        # Create controller nodes
        joint_state_broadcaster_node = Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster"],
        )

        ackermann_steering_controller_node = Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "ackermann_steering_controller",
                "--param-file",
                ros2_controllers_config,
            ],
        )

        # Create robot state publisher node
        robot_state_publisher_node = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[
                {
                    "use_sim_time": True,
                    "robot_description": robot_urdf_xml,
                }
            ],
        )

        # Create bridge node
        ros_gz_bridge_node = Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            parameters=[{"config_file": bridge_config_file}],
        )

        # Create event handlers for sequential startup
        spawn_robot_event_handler = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_robot_node,
                on_exit=[joint_state_broadcaster_node],
            )
        )

        joint_state_broadcaster_event_handler = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_node,
                on_exit=[ackermann_steering_controller_node],
            )
        )

        # Add all actions to launch description
        ld.add_action(spawn_robot_node)
        ld.add_action(spawn_robot_event_handler)
        ld.add_action(joint_state_broadcaster_event_handler)
        ld.add_action(robot_state_publisher_node)
        ld.add_action(ros_gz_bridge_node)

    return ld
