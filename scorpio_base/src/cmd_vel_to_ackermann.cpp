// Copyright 2025 Lihan Chen
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "scorpio_base/cmd_vel_to_ackermann.hpp"

namespace scorpio_base
{

CmdVelToAckermannNode::CmdVelToAckermannNode(const rclcpp::NodeOptions & options)
: Node("cmd_vel_to_ackermann", options)
{
  // Declare parameters
  this->declare_parameter("frame_id", "base_footprint");
  this->declare_parameter("wheelbase", 0.315);

  // Get parameters
  frame_id_ = this->get_parameter("frame_id").as_string();
  wheelbase_ = this->get_parameter("wheelbase").as_double();

  RCLCPP_INFO(this->get_logger(), "CmdVelToAckermann node initialized");

  // Create publisher and subscriber
  ackermann_pub_ =
    this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("ackermann_cmd", 10);

  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 10, [this](const geometry_msgs::msg::Twist::SharedPtr msg) { cmdVelCallback(msg); });
}

float CmdVelToAckermannNode::convertToSteeringAngle(float linear_vel, float angular_vel) const
{
  // If linear velocity is near zero, cannot determine steering angle
  if (std::abs(linear_vel) < 1e-6) {
    return 0.0f;
  }

  // If angular velocity is near zero, going straight
  if (std::abs(angular_vel) < 1e-6) {
    return 0.0f;
  }

  // Calculate turning radius: r = v / omega
  // The sign of radius is preserved from angular_vel
  float radius = linear_vel / angular_vel;

  // Calculate steering angle: delta = atan(wheelbase / radius)
  float steering_angle = std::atan(wheelbase_ / radius);

  const float max_steering_angle = static_cast<float>(M_PI / 4.0);  // π/4 radians
  if (steering_angle > max_steering_angle) {
    steering_angle = max_steering_angle;
  } else if (steering_angle < -max_steering_angle) {
    steering_angle = -max_steering_angle;
  }

  // I don't know why but the steering angle needs to be scaled by 1.5 to match the real steering angle
  return steering_angle * 1.5f;
}

void CmdVelToAckermannNode::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  float steering_angle = convertToSteeringAngle(msg->linear.x, msg->angular.z);

  auto ackermann_msg = ackermann_msgs::msg::AckermannDriveStamped();
  ackermann_msg.header.stamp = this->now();
  ackermann_msg.header.frame_id = frame_id_;
  ackermann_msg.drive.steering_angle = steering_angle;
  ackermann_msg.drive.speed = msg->linear.x;

  ackermann_pub_->publish(ackermann_msg);
}

}  // namespace scorpio_base

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(scorpio_base::CmdVelToAckermannNode)
