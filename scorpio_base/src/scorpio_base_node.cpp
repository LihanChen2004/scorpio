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

#include "scorpio_base/scorpio_base_node.hpp"

#include <sys/time.h>

#include "tf2/LinearMath/Quaternion.hpp"

namespace scorpio_base
{

// CRC16-MODBUS lookup tables
const uint8_t ScorpioBaseNode::CRC_HIGH[256] = {
  0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
  0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
  0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
  0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
  0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
  0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
  0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
  0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
  0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
  0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
  0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
  0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
  0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
  0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
  0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
  0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40};

const uint8_t ScorpioBaseNode::CRC_LOW[256] = {
  0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04,
  0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09, 0x08, 0xC8,
  0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC,
  0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3, 0x11, 0xD1, 0xD0, 0x10,
  0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4,
  0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A, 0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38,
  0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C,
  0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26, 0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0,
  0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4,
  0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F, 0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68,
  0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C,
  0xB4, 0x74, 0x75, 0xB5, 0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0,
  0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54,
  0x9C, 0x5C, 0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98,
  0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
  0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80, 0x40};

ScorpioBaseNode::ScorpioBaseNode(const rclcpp::NodeOptions & options)
: Node("scorpio_base", options),
  odom_x_(0.0),
  odom_y_(0.0),
  odom_yaw_(0.0),
  wheel_dist_(0.0),
  current_speed_(0.0f),
  new_vel_flag_(false),
  current_pwm_(0),
  last_pwm_(0),
  over_current_(false),
  vel_idx_(0)
{
  // Initialize parameter listener and get parameters
  param_listener_ =
    std::make_shared<scorpio_base::ParamListener>(this->get_node_parameters_interface());
  params_ = param_listener_->get_params();

  RCLCPP_INFO(this->get_logger(), "Scorpio Base Node Initializing...");
  RCLCPP_INFO(
    this->get_logger(), "  STM32 Port: %s @ %ld", params_.stm32_port.c_str(), params_.stm32_baud);
  RCLCPP_INFO(
    this->get_logger(), "  Motor Port: %s @ %ld", params_.motor_port.c_str(), params_.motor_baud);
  RCLCPP_INFO(this->get_logger(), "  Speed Limit: %.2f m/s", params_.limited_speed);
  RCLCPP_INFO(
    this->get_logger(), "  Hall Encoder: %s", params_.hall_encoder ? "enabled" : "disabled");

  // Initialize arrays
  fb_time_.fill(0.0);
  fb_dist_.fill(0.0);
  odom_x_arr_.fill(0.0);
  odom_yaw_arr_.fill(0.0);
  vel_x_list_.fill(0.0);

  // Create serial ports
  stm32_port_ptr_ = std::make_unique<CerealPort>();
  motor_port_ptr_ = std::make_unique<CerealPort>();

  // Initialize odometry
  resetOdometry();
  last_packet_time_ = this->now();
  last_cmd_time_ = this->now();

  // Create TF broadcaster
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

  // Open STM32 port with retry until success or shutdown
  RCLCPP_INFO(
    this->get_logger(), "Opening STM32 port: %s @ %ld...", params_.stm32_port.c_str(),
    params_.stm32_baud);
  while (rclcpp::ok()) {
    try {
      stm32_port_ptr_->open(params_.stm32_port.c_str(), params_.stm32_baud);
      RCLCPP_INFO(this->get_logger(), "STM32 port opened successfully");
      break;
    } catch (const CerealException & e) {
      RCLCPP_WARN(this->get_logger(), "Failed to open STM32 port: %s. Retrying in 2s...", e.what());
      rclcpp::sleep_for(std::chrono::seconds(2));
    }
  }

  // Open motor port with retry until success or shutdown
  RCLCPP_INFO(
    this->get_logger(), "Opening motor port: %s @ %ld...", params_.motor_port.c_str(),
    params_.motor_baud);
  while (rclcpp::ok()) {
    try {
      motor_port_ptr_->open(params_.motor_port.c_str(), params_.motor_baud);
      RCLCPP_INFO(this->get_logger(), "Motor port opened successfully");
      break;
    } catch (const CerealException & e) {
      RCLCPP_WARN(this->get_logger(), "Failed to open motor port: %s. Retrying in 2s...", e.what());
      rclcpp::sleep_for(std::chrono::seconds(2));
    }
  }

  // Create publishers
  imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu_data", 10);

  if (params_.hall_encoder) {
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    feedback_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("feedback_velocity", 10);
  }

  // Create timers
  check_timer_ =
    this->create_wall_timer(std::chrono::seconds(1), [this]() { checkSerialTimeout(this->now()); });

  motor_timer_ =
    this->create_wall_timer(std::chrono::milliseconds(100), [this]() { motorSendTimer(); });

  // Send initial config after a short delay
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  writeConfigData(0x0006, 0x0001, 0x0001);  // Enable motor power

  // Create subscriber (after initialization)
  ackermann_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
    "ackermann_cmd", 10, [this](const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg) {
      ackermannCmdCallback(msg);
    });

  stm32_port_ptr_->startReadStream(
    [this](char * data, int len) { this->stm32DataCallback(data, len); });
  motor_port_ptr_->startReadStream(
    [this](char * data, int len) { this->motorDataCallback(data, len); });

  RCLCPP_INFO(this->get_logger(), "Scorpio Base Node Initialized");
}

ScorpioBaseNode::~ScorpioBaseNode()
{
  RCLCPP_INFO(this->get_logger(), "Shutting down Scorpio Base Node");

  // Stop motors
  sendControlCommand(0x01, 0x00);

  // Stop serial streams
  if (stm32_port_ptr_) {
    stm32_port_ptr_->stopStream();
  }
  if (motor_port_ptr_) {
    motor_port_ptr_->stopStream();
  }
}

void ScorpioBaseNode::resetOdometry()
{
  odom_x_ = 0.0;
  odom_y_ = 0.0;
  odom_yaw_ = 0.0;
  wheel_dist_ = 0.0;
  odom_x_filter_.reset();
  odom_y_filter_.reset();
}

uint16_t ScorpioBaseNode::calculateCRC16(const uint8_t * msg_ptr, unsigned int msg_len) const
{
  uint8_t crc_high = 0xFF;
  uint8_t crc_low = 0xFF;

  while (msg_len--) {
    uint8_t index = crc_low ^ (*(msg_ptr++));
    crc_low = crc_high ^ CRC_HIGH[index];
    crc_high = CRC_LOW[index];
  }

  return static_cast<uint16_t>((crc_high << 8) | crc_low);
}

uint8_t ScorpioBaseNode::calculateChecksum(const uint8_t * buf) const
{
  uint8_t sum = 0;
  int len = (buf[2] << 8) + buf[3];
  for (int i = 0; i < len - 1; i++) {
    sum += buf[i];
  }
  return sum;
}

void ScorpioBaseNode::sendControlCommand(uint8_t type, uint8_t onoff)
{
  uint8_t buf[2] = {type, onoff};
  uint8_t buffer[16];

  buffer[0] = 'N';
  buffer[1] = 'X';
  buffer[2] = 0x00;
  buffer[3] = 0x08;  // length = 6 + 2
  buffer[4] = 0x06;  // command
  buffer[5] = buf[0];
  buffer[6] = buf[1];

  uint8_t sum = 0;
  for (int i = 0; i < 7; i++) {
    sum += buffer[i];
  }
  buffer[7] = sum;

  try {
    stm32_port_ptr_->write(reinterpret_cast<char *>(buffer), 8);
  } catch (const CerealException & e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to send control command: %s", e.what());
  }
}

void ScorpioBaseNode::writeConfigData(uint16_t write_addr, uint16_t write_len, int16_t data)
{
  uint8_t buffer[16];

  buffer[0] = 0x01;
  buffer[1] = 0x10;
  buffer[2] = write_addr >> 8;
  buffer[3] = write_addr & 0xFF;
  buffer[4] = write_len >> 8;
  buffer[5] = write_len & 0xFF;
  buffer[6] = write_len * 2;
  buffer[7] = data >> 8;
  buffer[8] = data & 0xFF;

  uint16_t crc = calculateCRC16(buffer, 9);
  buffer[9] = crc & 0xFF;
  buffer[10] = crc >> 8;

  std::lock_guard<std::mutex> lock(motor_mutex_);
  try {
    motor_port_ptr_->write(reinterpret_cast<char *>(buffer), 11);
  } catch (const CerealException & e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to write config data: %s", e.what());
  }
}

void ScorpioBaseNode::readWriteData(
  uint16_t read_addr, uint16_t read_len, uint16_t write_addr, uint16_t write_len, int16_t vel)
{
  uint8_t buffer[20];

  vel = -vel;  // Invert for hardware

  buffer[0] = 0x01;
  buffer[1] = 0x17;
  buffer[2] = read_addr >> 8;
  buffer[3] = read_addr & 0xFF;
  buffer[4] = read_len >> 8;
  buffer[5] = read_len & 0xFF;
  buffer[6] = write_addr >> 8;
  buffer[7] = write_addr & 0xFF;
  buffer[8] = write_len >> 8;
  buffer[9] = write_len & 0xFF;
  buffer[10] = 0x02;
  buffer[11] = vel >> 8;
  buffer[12] = vel & 0xFF;

  uint16_t crc = calculateCRC16(buffer, 13);
  buffer[13] = crc & 0xFF;
  buffer[14] = crc >> 8;

  std::lock_guard<std::mutex> lock(motor_mutex_);
  try {
    motor_port_ptr_->write(reinterpret_cast<char *>(buffer), 15);
  } catch (const CerealException & e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to send motor command: %s", e.what());
  }
}

void ScorpioBaseNode::sendMotorSpeed(float speed)
{
  // Update parameters if changed
  if (param_listener_->is_old(params_)) {
    params_ = param_listener_->get_params();
    RCLCPP_INFO(this->get_logger(), "Parameters updated");
  }

  // Clamp speed
  speed = std::clamp(
    speed, static_cast<float>(-params_.limited_speed), static_cast<float>(params_.limited_speed));

  int16_t motor_vel = static_cast<int16_t>(speed * MOTOR_VELOCITY_SCALE);
  readWriteData(0x002a, 0x0001, 0x002B, 0x0001, motor_vel);
}

void ScorpioBaseNode::sendPwmCommand(float /* linear_vel */, float steering_angle)
{
  // Convert steering angle to PWM
  int pwm_left = 1080;  // Default throttle (unused in this implementation)
  int pwm_angle = ANGLE_MIDDLE_POINT;

  // Clamp steering angle
  steering_angle = std::clamp(steering_angle, -1.0f, 1.0f);

  // Convert angle: -180 * angle / M_PI * 6
  float dz = -180.0f * steering_angle / M_PI * 6.0f;

  if (dz > 0.0f) {
    // Right turn
    pwm_angle = ANGLE_MIDDLE_POINT - static_cast<int>(dz);
    pwm_angle = std::max(pwm_angle, 800);
  } else if (dz < 0.0f) {
    // Left turn
    pwm_angle = ANGLE_MIDDLE_POINT - static_cast<int>(dz);
    pwm_angle = std::min(pwm_angle, 1460);
  }

  // Send command to STM32
  uint8_t buf[4];
  buf[0] = pwm_left >> 8;
  buf[1] = pwm_left & 0xFF;
  buf[2] = pwm_angle >> 8;
  buf[3] = pwm_angle & 0xFF;

  uint8_t buffer[16];
  buffer[0] = 'N';
  buffer[1] = 'X';
  buffer[2] = 0x00;
  buffer[3] = 0x0A;  // length = 6 + 4
  buffer[4] = 0x01;  // command
  std::memcpy(&buffer[5], buf, 4);

  uint8_t sum = 0;
  for (int i = 0; i < 9; i++) {
    sum += buffer[i];
  }
  buffer[9] = sum;

  try {
    stm32_port_ptr_->write(reinterpret_cast<char *>(buffer), 10);
  } catch (const CerealException & e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to send PWM command: %s", e.what());
  }
}

void ScorpioBaseNode::ackermannCmdCallback(
  const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg)
{
  {
    std::lock_guard<std::mutex> lock(vel_mutex_);
    current_speed_ = msg->drive.speed;
    new_vel_flag_ = true;
    last_cmd_time_ = this->now();
  }

  sendPwmCommand(msg->drive.speed, msg->drive.steering_angle);
}

void ScorpioBaseNode::motorSendTimer()
{
  bool should_send = false;
  float vel = 0.0f;

  {
    std::lock_guard<std::mutex> lock(vel_mutex_);
    if (new_vel_flag_.load()) {
      vel = current_speed_;
      should_send = true;
      new_vel_flag_ = false;
    }
  }

  if (should_send) {
    sendMotorSpeed(vel);
  }
}

void ScorpioBaseNode::checkSerialTimeout(const rclcpp::Time & now)
{
  // Check for command timeout
  auto cmd_elapsed = (now - last_cmd_time_).nanoseconds();
  if (cmd_elapsed > PACKET_TIMEOUT_NS) {
    // Send stop commands
    sendPwmCommand(0.0f, 0.0f);
    sendMotorSpeed(0.0f);
  }

  // Check for overcurrent
  if (over_current_.load()) {
    static int over_current_count = 0;
    over_current_count++;

    if (over_current_count > 4) {
      RCLCPP_ERROR(this->get_logger(), "Motor overcurrent detected! Shutting down motor power.");
      sendControlCommand(0x01, 0x01);  // Close motor power
    } else if (over_current_count > 2) {
      sendControlCommand(0x01, 0x00);  // Open motor power
    }
  }
}

void ScorpioBaseNode::motorDataCallback(char * data, int len)
{
  if (len < 3) {
    return;
  }

  const uint8_t * buf = reinterpret_cast<uint8_t *>(data);

  if (buf[0] == 0x01 && buf[1] == 0x17 && len > 3) {
    if ((buf[2] + 5) == len) {
      uint16_t crc_word = calculateCRC16(buf, 5);
      uint16_t received_crc = buf[len - 2] + (buf[len - 1] << 8);

      if (received_crc == crc_word) {
        if (buf[4] & 0x01) {
          RCLCPP_ERROR_THROTTLE(
            this->get_logger(), *this->get_clock(), 1000, "Motor overcurrent detected!");
          over_current_ = true;
        } else {
          over_current_ = false;
        }
      }
    }
  }
}

void ScorpioBaseNode::stm32DataCallback(char * data, int len)
{
  std::lock_guard<std::mutex> lock(parse_mutex_);

  rclcpp::Time current_time = this->now();

  // Check timeout
  auto time_diff = (current_time - last_packet_time_).nanoseconds();
  if (time_diff > PACKET_TIMEOUT_NS) {
    recv_buffer_.clear();
    RCLCPP_WARN(this->get_logger(), "STM32 packet timeout, buffer cleared");
  }

  // Check buffer overflow
  if (recv_buffer_.size() + len > 512) {
    recv_buffer_.clear();
    RCLCPP_ERROR(this->get_logger(), "Receive buffer overflow! Dropping data.");
    return;
  }

  // Append new data
  recv_buffer_.insert(recv_buffer_.end(), data, data + len);

  // Parse packets
  while (recv_buffer_.size() > 4) {
    // Find header "NX"
    auto it = std::search(
      recv_buffer_.begin(), recv_buffer_.end(), reinterpret_cast<const uint8_t *>("NX"),
      reinterpret_cast<const uint8_t *>("NX") + 2);

    if (it == recv_buffer_.end()) {
      // No header found, check if last byte is 'N'
      if (recv_buffer_.back() == 'N') {
        recv_buffer_ = {recv_buffer_.back()};
      } else {
        recv_buffer_.clear();
      }
      break;
    }

    // Remove garbage before header
    if (it != recv_buffer_.begin()) {
      recv_buffer_.erase(recv_buffer_.begin(), it);
    }

    // Check if we have length field
    if (recv_buffer_.size() < 4) {
      break;
    }

    // Parse frame length
    int frame_len = (recv_buffer_[2] << 8) + recv_buffer_[3];

    if (frame_len < 6 || frame_len > 256) {
      // Invalid frame length, skip this header
      recv_buffer_.erase(recv_buffer_.begin(), recv_buffer_.begin() + 2);
      continue;
    }

    // Wait for complete packet
    if (static_cast<int>(recv_buffer_.size()) < frame_len) {
      break;
    }

    // Verify checksum
    uint8_t expected_sum = calculateChecksum(recv_buffer_.data());
    if (expected_sum == recv_buffer_[frame_len - 1]) {
      // Parse packet
      parseStm32Packet(recv_buffer_.data() + 5, frame_len - 6);
      last_packet_time_ = current_time;
    } else {
      RCLCPP_ERROR(this->get_logger(), "STM32 packet checksum error");
    }

    // Remove processed packet
    recv_buffer_.erase(recv_buffer_.begin(), recv_buffer_.begin() + frame_len);
  }
}

void ScorpioBaseNode::parseStm32Packet(const uint8_t * buf, int len)
{
  if (len < 34) {
    return;  // Not enough data for IMU + encoder
  }

  // Parse IMU data
  auto imu_msg = sensor_msgs::msg::Imu();
  imu_msg.header.stamp = this->now();
  imu_msg.header.frame_id = "base_imu";

  // Accelerometer (m/s^2)
  float accel_x = static_cast<int16_t>((buf[1] << 8) | buf[0]) / 32768.0f * 16.0f * 9.8f;
  float accel_y = static_cast<int16_t>((buf[3] << 8) | buf[2]) / 32768.0f * 16.0f * 9.8f;
  float accel_z = static_cast<int16_t>((buf[5] << 8) | buf[4]) / 32768.0f * 16.0f * 9.8f;

  // Gyroscope (deg/s -> rad/s)
  float gyro_x = static_cast<int16_t>((buf[7] << 8) | buf[6]) / 32768.0f * 2000.0f * M_PI / 180.0f;
  float gyro_y = static_cast<int16_t>((buf[9] << 8) | buf[8]) / 32768.0f * 2000.0f * M_PI / 180.0f;
  float gyro_z =
    static_cast<int16_t>((buf[11] << 8) | buf[10]) / 32768.0f * 2000.0f * M_PI / 180.0f;

  // Orientation (radians)
  float roll = static_cast<int16_t>((buf[13] << 8) | buf[12]) / 32768.0f * M_PI;
  float pitch = static_cast<int16_t>((buf[15] << 8) | buf[14]) / 32768.0f * M_PI;
  float yaw = static_cast<int16_t>((buf[17] << 8) | buf[16]) / 32768.0f * M_PI;

  // Convert RPY to quaternion
  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw);

  imu_msg.orientation.x = q.x();
  imu_msg.orientation.y = q.y();
  imu_msg.orientation.z = q.z();
  imu_msg.orientation.w = q.w();

  imu_msg.angular_velocity.x = gyro_x;
  imu_msg.angular_velocity.y = gyro_y;
  imu_msg.angular_velocity.z = gyro_z;

  imu_msg.linear_acceleration.x = accel_x;
  imu_msg.linear_acceleration.y = accel_y;
  imu_msg.linear_acceleration.z = accel_z;

  // Set covariances
  imu_msg.orientation_covariance[0] = 0.0017 * 0.0017;
  imu_msg.orientation_covariance[4] = 0.0017 * 0.0017;
  imu_msg.orientation_covariance[8] = 0.0017 * 0.0017;

  imu_msg.angular_velocity_covariance[0] = 0.1 * 0.1;
  imu_msg.angular_velocity_covariance[4] = 0.1 * 0.1;
  imu_msg.angular_velocity_covariance[8] = 0.1 * 0.1;

  imu_msg.linear_acceleration_covariance[0] = 0.1 * 0.1;
  imu_msg.linear_acceleration_covariance[4] = 0.1 * 0.1;
  imu_msg.linear_acceleration_covariance[8] = 0.1 * 0.1;

  imu_pub_->publish(imu_msg);

  // Parse encoder data if hall encoder is enabled
  if (params_.hall_encoder) {
    int current_pwm = (buf[26] << 24) | (buf[27] << 16) | (buf[28] << 8) | buf[29];

    // First time initialization
    static bool first_time = true;
    if (first_time) {
      last_pwm_ = current_pwm;
      first_time = false;
    }

    // Calculate distance traveled
    double distance = M_PI * WHEEL_DIAMETER * (current_pwm - last_pwm_) * 5.0 / 574.0;
    last_pwm_ = current_pwm;

    // Update odometry
    odom_x_ += distance * std::cos(odom_yaw_);
    odom_y_ += distance * std::sin(odom_yaw_);
    odom_yaw_ = yaw;
    wheel_dist_ += distance;

    // Apply Kalman filter
    double est_x = odom_x_filter_.predict(wheel_dist_);

    // Update circular buffer
    int curr_idx = (vel_idx_ + COUNT_TIMES - 1) % COUNT_TIMES;
    struct timeval tv;
    gettimeofday(&tv, nullptr);
    double ts = static_cast<double>(tv.tv_sec) * 1000.0 + tv.tv_usec / 1000.0;

    fb_time_[curr_idx] = ts;
    fb_dist_[curr_idx] = wheel_dist_;
    odom_x_arr_[curr_idx] = est_x;
    odom_yaw_arr_[curr_idx] = odom_yaw_;

    // Calculate velocity
    double dt = (fb_time_[curr_idx] - fb_time_[vel_idx_]) * 0.001;
    if (dt > 0.001) {
      vel_x_list_[curr_idx] = (odom_x_arr_[curr_idx] - odom_x_arr_[vel_idx_]) / dt;

      // Average velocity
      double vel_x = 0.0;
      for (int i = 0; i < COUNT_TIMES; i++) {
        vel_x += vel_x_list_[i];
      }
      vel_x /= COUNT_TIMES;

      // Angular velocity
      double delta_yaw = odom_yaw_arr_[curr_idx] - odom_yaw_arr_[vel_idx_];
      // Normalize angle
      while (delta_yaw > M_PI) delta_yaw -= 2.0 * M_PI;
      while (delta_yaw < -M_PI) delta_yaw += 2.0 * M_PI;
      double vel_yaw = delta_yaw / dt;

      vel_idx_ = (vel_idx_ + 1) % COUNT_TIMES;

      // Publish odometry
      publishOdometry(imu_msg.header.stamp);

      // Publish feedback velocity
      auto twist_msg = geometry_msgs::msg::Twist();
      twist_msg.linear.x = vel_x;
      twist_msg.linear.y = 0.0;
      twist_msg.angular.z = vel_yaw;
      feedback_vel_pub_->publish(twist_msg);
    }
  }
}

void ScorpioBaseNode::publishOdometry(const rclcpp::Time & stamp)
{
  // Publish TF
  geometry_msgs::msg::TransformStamped odom_trans;
  odom_trans.header.stamp = stamp;
  odom_trans.header.frame_id = params_.odom_frame_id;
  odom_trans.child_frame_id = params_.base_frame_id;

  odom_trans.transform.translation.x = odom_x_;
  odom_trans.transform.translation.y = odom_y_;
  odom_trans.transform.translation.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, odom_yaw_);
  odom_trans.transform.rotation.x = q.x();
  odom_trans.transform.rotation.y = q.y();
  odom_trans.transform.rotation.z = q.z();
  odom_trans.transform.rotation.w = q.w();

  tf_broadcaster_->sendTransform(odom_trans);

  // Publish Odometry message
  auto odom_msg = nav_msgs::msg::Odometry();
  odom_msg.header.stamp = stamp;
  odom_msg.header.frame_id = params_.odom_frame_id;
  odom_msg.child_frame_id = params_.base_frame_id;

  odom_msg.pose.pose.position.x = odom_x_;
  odom_msg.pose.pose.position.y = odom_y_;
  odom_msg.pose.pose.position.z = 0.0;
  odom_msg.pose.pose.orientation.x = q.x();
  odom_msg.pose.pose.orientation.y = q.y();
  odom_msg.pose.pose.orientation.z = q.z();
  odom_msg.pose.pose.orientation.w = q.w();

  // Set covariances
  odom_msg.pose.covariance[0] = 0.01 * 0.01;
  odom_msg.pose.covariance[7] = 0.05 * 0.05;
  odom_msg.pose.covariance[35] = 0.1 * 0.1;

  odom_pub_->publish(odom_msg);
}

}  // namespace scorpio_base

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(scorpio_base::ScorpioBaseNode)
