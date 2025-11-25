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

#ifndef SCORPIO_BASE__SCORPIO_BASE_NODE_HPP_
#define SCORPIO_BASE__SCORPIO_BASE_NODE_HPP_

#include <atomic>
#include <memory>
#include <mutex>

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "scorpio_base/cereal_port.hpp"
#include "scorpio_base/kalman_filter.hpp"
#include "scorpio_base/scorpio_base_parameters.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2_ros/transform_broadcaster.hpp"

namespace scorpio_base
{

class ScorpioBaseNode : public rclcpp::Node
{
public:
  explicit ScorpioBaseNode(const rclcpp::NodeOptions & options);
  ~ScorpioBaseNode() override;

private:
  // Serial port callbacks
  void stm32DataCallback(char * data, int len);
  void motorDataCallback(char * data, int len);

  // ROS callbacks
  void ackermannCmdCallback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg);

  // Timer callbacks
  void checkSerialTimeout(const rclcpp::Time & now);
  void motorSendTimer();

  // Serial communication
  void sendPwmCommand(float linear_vel, float steering_angle);
  void sendMotorSpeed(float speed);
  void sendControlCommand(uint8_t type, uint8_t onoff);
  void writeConfigData(uint16_t write_addr, uint16_t write_len, int16_t data);
  void readWriteData(
    uint16_t read_addr, uint16_t read_len, uint16_t write_addr, uint16_t write_len, int16_t vel);

  // Protocol parsing
  void parseStm32Packet(const uint8_t * buf, int len);
  void parseMotorPacket(const uint8_t * buf, int len);
  uint8_t calculateChecksum(const uint8_t * buf) const;
  uint16_t calculateCRC16(const uint8_t * msg_ptr, unsigned int msg_len) const;

  // Odometry
  void resetOdometry();
  void publishOdometry(const rclcpp::Time & stamp);

  // Parameter management
  std::shared_ptr<scorpio_base::ParamListener> param_listener_;
  scorpio_base::Params params_;

  // Serial ports
  std::unique_ptr<CerealPort> stm32_port_ptr_;
  std::unique_ptr<CerealPort> motor_port_ptr_;

  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr feedback_vel_pub_;

  // Subscribers
  rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackermann_sub_;

  // Timer
  rclcpp::TimerBase::SharedPtr check_timer_;
  rclcpp::TimerBase::SharedPtr motor_timer_;

  // TF
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Odometry state
  double odom_x_;
  double odom_y_;
  double odom_yaw_;
  double wheel_dist_;

  // Kalman filters
  KalmanFilter odom_x_filter_;
  KalmanFilter odom_y_filter_;

  // Velocity tracking
  std::mutex vel_mutex_;
  float current_speed_;
  std::atomic<bool> new_vel_flag_;
  rclcpp::Time last_cmd_time_;

  // Motor state
  std::mutex motor_mutex_;
  int current_pwm_;
  int last_pwm_;
  std::atomic<bool> over_current_;

  // Serial packet parsing state (for STM32)
  std::mutex parse_mutex_;
  std::vector<uint8_t> recv_buffer_;
  rclcpp::Time last_packet_time_;

  // Circular buffer for velocity smoothing
  static constexpr int COUNT_TIMES = 20;
  int vel_idx_;
  std::array<double, COUNT_TIMES> fb_time_;
  std::array<double, COUNT_TIMES> fb_dist_;
  std::array<double, COUNT_TIMES> odom_x_arr_;
  std::array<double, COUNT_TIMES> odom_yaw_arr_;
  std::array<double, COUNT_TIMES> vel_x_list_;

  // Constants
  static constexpr int ANGLE_MIDDLE_POINT = 1118;
  static constexpr double WHEEL_DIAMETER = 0.105;  // meters
  static constexpr double MAX_SPEED = 1.0;         // m/s
  static constexpr int MOTOR_VELOCITY_SCALE = 11800;
  static constexpr int64_t PACKET_TIMEOUT_NS = 3000000000LL;  // 3 seconds

  // CRC lookup tables
  static const uint8_t CRC_HIGH[256];
  static const uint8_t CRC_LOW[256];
};

}  // namespace scorpio_base

#endif  // SCORPIO_BASE__SCORPIO_BASE_NODE_HPP_
