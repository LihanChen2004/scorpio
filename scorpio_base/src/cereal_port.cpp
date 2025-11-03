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

#include "scorpio_base/cereal_port.hpp"

#include <errno.h>
#include <poll.h>
#include <string.h>
#include <unistd.h>

#include <cstdio>

namespace scorpio_base
{

#define CEREAL_EXCEPT(except, msg, ...)                               \
  {                                                                   \
    char buf[1000];                                                   \
    snprintf(buf, 1000, msg " (in %s)", ##__VA_ARGS__, __FUNCTION__); \
    throw except(buf);                                                \
  }

CerealPort::CerealPort() : fd_(-1), baud_(0), stream_paused_(false), stream_stopped_(false) {}

CerealPort::~CerealPort()
{
  if (portOpen()) {
    close();
  }
}

void CerealPort::open(const char * port_name, int baud_rate)
{
  if (portOpen()) {
    close();
  }

  fd_ = ::open(port_name, O_RDWR | O_NONBLOCK | O_NOCTTY);

  if (fd_ == -1) {
    const char * extra_msg = "";
    switch (errno) {
      case EACCES:
        extra_msg = "Permission denied. Check udev rules or user groups.";
        break;
      case ENOENT:
        extra_msg = "Port does not exist.";
        break;
    }
    CEREAL_EXCEPT(
      CerealException, "Failed to open port: %s. %s (errno = %d). %s", port_name, strerror(errno),
      errno, extra_msg);
  }

  try {
    struct flock fl;
    fl.l_type = F_WRLCK;
    fl.l_whence = SEEK_SET;
    fl.l_start = 0;
    fl.l_len = 0;
    fl.l_pid = getpid();

    if (fcntl(fd_, F_SETLK, &fl) != 0) {
      CEREAL_EXCEPT(
        CerealException, "Device %s is already locked. Try 'lsof | grep %s'.", port_name,
        port_name);
    }

    struct termios newtio;
    tcgetattr(fd_, &newtio);
    memset(&newtio.c_cc, 0, sizeof(newtio.c_cc));
    newtio.c_cflag = CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;
    newtio.c_lflag = 0;
    cfsetspeed(&newtio, baud_rate);
    baud_ = baud_rate;

    tcflush(fd_, TCIFLUSH);
    if (tcsetattr(fd_, TCSANOW, &newtio) < 0) {
      CEREAL_EXCEPT(CerealException, "Unable to set serial port attributes for %s", port_name);
    }
    usleep(200000);
  } catch (CerealException & e) {
    if (fd_ != -1) {
      ::close(fd_);
    }
    fd_ = -1;
    throw;
  }
}

void CerealPort::close()
{
  int retval = ::close(fd_);
  fd_ = -1;

  if (retval != 0) {
    CEREAL_EXCEPT(
      CerealException, "Failed to close port -- error = %d: %s\n", errno, strerror(errno));
  }
}

int CerealPort::write(const char * data, int length)
{
  int len = length == -1 ? strlen(data) : length;

  int origflags = fcntl(fd_, F_GETFL, 0);
  fcntl(fd_, F_SETFL, origflags & ~O_NONBLOCK);
  int retval = ::write(fd_, data, len);
  fcntl(fd_, F_SETFL, origflags | O_NONBLOCK);

  if (retval == len) {
    return retval;
  } else {
    CEREAL_EXCEPT(CerealException, "write failed");
  }
}

int CerealPort::read(char * buffer, int max_length, int timeout)
{
  struct pollfd ufd[1];
  ufd[0].fd = fd_;
  ufd[0].events = POLLIN;

  if (timeout == 0) {
    timeout = -1;
  }

  int retval = poll(ufd, 1, timeout);
  if (retval < 0) {
    CEREAL_EXCEPT(CerealException, "poll failed -- error = %d: %s", errno, strerror(errno));
  }

  if (retval == 0) {
    CEREAL_EXCEPT(TimeoutException, "timeout reached");
  }

  if (ufd[0].revents & POLLERR) {
    CEREAL_EXCEPT(CerealException, "error on socket, possibly unplugged");
  }

  int ret = ::read(fd_, buffer, max_length);

  if (ret == -1 && errno != EAGAIN && errno != EWOULDBLOCK) {
    CEREAL_EXCEPT(CerealException, "read failed");
  }

  return ret;
}

int CerealPort::readBytes(char * buffer, int length, int timeout)
{
  int current = 0;

  struct pollfd ufd[1];
  ufd[0].fd = fd_;
  ufd[0].events = POLLIN;

  if (timeout == 0) {
    timeout = -1;
  }

  while (current < length) {
    int retval = poll(ufd, 1, timeout);
    if (retval < 0) {
      CEREAL_EXCEPT(CerealException, "poll failed -- error = %d: %s", errno, strerror(errno));
    }

    if (retval == 0) {
      CEREAL_EXCEPT(TimeoutException, "timeout reached");
    }

    if (ufd[0].revents & POLLERR) {
      CEREAL_EXCEPT(CerealException, "error on socket, possibly unplugged");
    }

    int ret = ::read(fd_, &buffer[current], length - current);

    if (ret == -1 && errno != EAGAIN && errno != EWOULDBLOCK) {
      CEREAL_EXCEPT(CerealException, "read failed");
    }

    current += ret;
  }
  return current;
}

int CerealPort::flush()
{
  int retval = tcflush(fd_, TCIOFLUSH);
  if (retval != 0) {
    CEREAL_EXCEPT(CerealException, "tcflush failed");
  }

  return retval;
}

bool CerealPort::startReadStream(std::function<void(char *, int)> callback)
{
  if (stream_thread_ != nullptr) {
    return false;
  }

  stream_stopped_ = false;
  stream_paused_ = false;

  read_callback_ = callback;
  stream_thread_ = std::make_unique<std::thread>(&CerealPort::readThread, this);
  return true;
}

void CerealPort::readThread()
{
  constexpr int k_max_length = 128;
  char data[k_max_length];

  struct pollfd ufd[1];
  ufd[0].fd = fd_;
  ufd[0].events = POLLIN;

  while (!stream_stopped_) {
    if (!stream_paused_) {
      if (poll(ufd, 1, 100) > 0) {
        if (!(ufd[0].revents & POLLERR)) {
          int ret = ::read(fd_, data, k_max_length);
          if (ret > 0) {
            read_callback_(data, ret);
          }
        }
      }
    }
  }
}

void CerealPort::stopStream()
{
  stream_stopped_ = true;
  if (stream_thread_ && stream_thread_->joinable()) {
    stream_thread_->join();
  }
  stream_thread_.reset();
}

void CerealPort::pauseStream() { stream_paused_ = true; }

void CerealPort::resumeStream() { stream_paused_ = false; }

}  // namespace scorpio_base
