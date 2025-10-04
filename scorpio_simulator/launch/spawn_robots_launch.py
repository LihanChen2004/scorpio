import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from sdformat_tools.urdf_generator import UrdfGenerator
from xmacro.xmacro4sdf import XMLMacro4sdf


def generate_launch_description():
    pkg_simulator = get_package_share_directory("scorpio_simulator")
    pkg_robot_description = get_package_share_directory("scorpio_description")

    robot_xmacro_path = os.path.join(
        pkg_robot_description,
        "resource",
        "models",
        "scorpio",
        "model.sdf.xmacro",
    )

    bridge_config = os.path.join(pkg_simulator, "config", "ros_gz_bridge.yaml")

    # Get spawn robot init pose
    gz_world_path = os.path.join(pkg_simulator, "config", "gz_world.yaml")
    with open(gz_world_path) as file:
        config = yaml.safe_load(file)
        selected_world = config.get("world")
        robots = config["robots"].get(selected_world)

    xmacro = XMLMacro4sdf()
    xmacro.set_xml_file(robot_xmacro_path)

    ld = LaunchDescription()

    for robot in robots:
        # Generate SDF from xmacro
        xmacro.generate()
        robot_xml = xmacro.to_string()

        # Generate URDF from SDF
        urdf_generator = UrdfGenerator()
        urdf_generator.parse_from_sdf_string(robot_xml)
        robot_urdf_xml = urdf_generator.to_string()

        spawn_robot = Node(
            package="ros_gz_sim",
            executable="create",
            arguments=[
                "-string",
                robot_xml,
                "-name",
                robot["name"],
                "-allow_renaming",
                "true",
                "-x",
                robot["x_pose"],
                "-y",
                robot["y_pose"],
                "-z",
                robot["z_pose"],
                "-Y",
                robot["yaw"],
            ],
        )

        robot_state_publisher = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[
                {
                    "use_sim_time": True,
                    "robot_description": robot_urdf_xml,
                }
            ],
        )

        robot_ign_bridge = Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            parameters=[{"config_file": bridge_config}],
        )

        ld.add_action(spawn_robot)
        ld.add_action(robot_state_publisher)
        ld.add_action(robot_ign_bridge)

    return ld
