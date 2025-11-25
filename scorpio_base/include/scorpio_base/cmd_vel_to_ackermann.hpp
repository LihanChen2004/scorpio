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

#ifndef SCORPIO_BASE__CMD_VEL_TO_ACKERMANN_HPP_
#define SCORPIO_BASE__CMD_VEL_TO_ACKERMANN_HPP_

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

namespace scorpio_base
{

class CmdVelToAckermannNode : public rclcpp::Node
{
public:
  explicit CmdVelToAckermannNode(const rclcpp::NodeOptions & options);

private:
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  float convertToSteeringAngle(float linear_vel, float angular_vel) const;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackermann_pub_;

  std::string frame_id_;
  double wheelbase_;
};

}  // namespace scorpio_base

#endif  // SCORPIO_BASE__CMD_VEL_TO_ACKERMANN_HPP_
