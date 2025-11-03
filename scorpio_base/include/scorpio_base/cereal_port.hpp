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

#ifndef SCORPIO_BASE__CEREAL_PORT_HPP_
#define SCORPIO_BASE__CEREAL_PORT_HPP_

#include <fcntl.h>
#include <termios.h>

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace scorpio_base
{

// Exception types
class CerealException : public std::runtime_error
{
public:
  explicit CerealException(const char * msg) : std::runtime_error(msg) {}
};

class TimeoutException : public CerealException
{
public:
  explicit TimeoutException(const char * msg) : CerealException(msg) {}
};

// Serial port wrapper
class CerealPort
{
public:
  CerealPort();
  ~CerealPort();

  // Delete copy/move
  CerealPort(const CerealPort &) = delete;
  CerealPort & operator=(const CerealPort &) = delete;

  void open(const char * port_name, int baud_rate = 115200);
  void close();

  bool portOpen() const { return fd_ != -1; }
  int baudRate() const { return baud_; }

  int write(const char * data, int length = -1);
  int read(char * data, int max_length, int timeout = -1);
  int readBytes(char * data, int length, int timeout = -1);

  int flush();

  // Stream functions with callbacks
  bool startReadStream(std::function<void(char *, int)> callback);
  void stopStream();
  void pauseStream();
  void resumeStream();

private:
  void readThread();

  int fd_;
  int baud_;

  std::unique_ptr<std::thread> stream_thread_;
  std::function<void(char *, int)> read_callback_;

  std::atomic<bool> stream_paused_;
  std::atomic<bool> stream_stopped_;
};

}  // namespace scorpio_base

#endif  // SCORPIO_BASE__CEREAL_PORT_HPP_
