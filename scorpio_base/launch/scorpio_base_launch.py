#!/usr/bin/env python3
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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Launch Scorpio base driver."""
    # Create the launch configuration variables
    base_frame_id = LaunchConfiguration("base_frame_id")
    odom_frame_id = LaunchConfiguration("odom_frame_id")
    stm32_port = LaunchConfiguration("stm32_port")
    motor_port = LaunchConfiguration("motor_port")
    stm32_baud = LaunchConfiguration("stm32_baud")
    motor_baud = LaunchConfiguration("motor_baud")
    hall_encoder = LaunchConfiguration("hall_encoder")
    limited_speed = LaunchConfiguration("limited_speed")
    wheelbase = LaunchConfiguration("wheelbase")

    # Set environment variables for better logging
    stdout_linebuf_envvar = SetEnvironmentVariable(
        "RCUTILS_LOGGING_BUFFERED_STREAM", "1"
    )

    colorized_output_envvar = SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1")

    # Declare launch arguments
    declare_base_frame_id_cmd = DeclareLaunchArgument(
        "base_frame_id",
        default_value="base_footprint",
        description="Base frame ID",
    )

    declare_odom_frame_id_cmd = DeclareLaunchArgument(
        "odom_frame_id",
        default_value="odom",
        description="Odometry frame ID",
    )

    declare_stm32_port_cmd = DeclareLaunchArgument(
        "stm32_port",
        default_value="/dev/ttyS0",
        description="STM32 serial port",
    )

    declare_motor_port_cmd = DeclareLaunchArgument(
        "motor_port",
        default_value="/dev/ttyS3",
        description="Motor controller serial port",
    )

    declare_stm32_baud_cmd = DeclareLaunchArgument(
        "stm32_baud",
        default_value="115200",
        description="STM32 baud rate",
    )

    declare_motor_baud_cmd = DeclareLaunchArgument(
        "motor_baud",
        default_value="57600",
        description="Motor controller baud rate",
    )

    declare_hall_encoder_cmd = DeclareLaunchArgument(
        "hall_encoder",
        default_value="true",
        description="Enable hall encoder odometry",
    )

    declare_limited_speed_cmd = DeclareLaunchArgument(
        "limited_speed",
        default_value="1.0",
        description="Maximum speed in m/s",
    )

    declare_wheelbase_cmd = DeclareLaunchArgument(
        "wheelbase",
        default_value="0.315",
        description="Wheelbase in meters",
    )

    # CmdVel to Ackermann converter node
    cmd_vel_to_ackermann_node = Node(
        package="scorpio_base",
        executable="cmd_vel_to_ackermann_node",
        name="cmd_vel_to_ackermann",
        output="screen",
        parameters=[
            {
                "frame_id": base_frame_id,
                "wheelbase": wheelbase,
            }
        ],
    )

    # Main Scorpio base driver node
    scorpio_base_node = Node(
        package="scorpio_base",
        executable="scorpio_base_node",
        name="scorpio_base",
        output="screen",
        parameters=[
            {
                "base_frame_id": base_frame_id,
                "odom_frame_id": odom_frame_id,
                "stm32_port": stm32_port,
                "motor_port": motor_port,
                "stm32_baud": stm32_baud,
                "motor_baud": motor_baud,
                "hall_encoder": hall_encoder,
                "limited_speed": limited_speed,
                "wheelbase": wheelbase,
            }
        ],
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(colorized_output_envvar)

    # Declare the launch options
    ld.add_action(declare_base_frame_id_cmd)
    ld.add_action(declare_odom_frame_id_cmd)
    ld.add_action(declare_stm32_port_cmd)
    ld.add_action(declare_motor_port_cmd)
    ld.add_action(declare_stm32_baud_cmd)
    ld.add_action(declare_motor_baud_cmd)
    ld.add_action(declare_hall_encoder_cmd)
    ld.add_action(declare_limited_speed_cmd)
    ld.add_action(declare_wheelbase_cmd)

    # Add the nodes
    ld.add_action(cmd_vel_to_ackermann_node)
    ld.add_action(scorpio_base_node)

    return ld
