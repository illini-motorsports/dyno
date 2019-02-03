#include "Log.hpp"

#include <cstring>
#include <errno.h>
#include <fcntl.h>
#include <sys/epoll.h>
#include <sys/timerfd.h>
#include <termios.h>
#include <unistd.h>
#include <vector>

#define TEMP_POLL_SEC 2

static const char* ttyName = "/dev/ttyUSB0";

int main(int argc, char* argv[])
{
  /**
   * Open serial port
   */

  LOG(INFO) << "opening serial port name=(" << ttyName << ")";
  int fd = ::open(ttyName, O_RDWR | O_NOCTTY | O_SYNC | O_NONBLOCK);
  if (fd < 0) {
    LOG(ERROR) << "failed to open serial port error=(" << std::strerror(errno) << ")";
    return EXIT_FAILURE;
  }
  LOG(INFO) << "opened serial port name=(" << ttyName << ") fd=" << fd;

  struct termios tty;
  ::memset(&tty, 0, sizeof(tty));
  if (::tcgetattr(fd, &tty) != 0) {
    LOG(ERROR) << "failed to get serial port config error=(" << std::strerror(errno) << ")";
    ::close(fd);
    return EXIT_FAILURE;
  }

  ::cfsetspeed(&tty, B38400); // Baud rate
  ::cfmakeraw(&tty);
  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8-bit chars
  tty.c_cflag &= ~(PARENB | PARODD); // Disable parity
  tty.c_cflag &= ~CSTOPB; // 1 stop bit
  tty.c_cflag &= ~CRTSCTS; // No hardware flow control
  ::tcflush(fd, TCIOFLUSH);
  if (::tcsetattr(fd, TCSANOW, &tty) != 0) {
    LOG(ERROR) << "failed to set serial port config error=(" << std::strerror(errno) << ")";
    ::close(fd);
    return EXIT_FAILURE;
  }

  /**
   * Create timer
   */

  int tfd = ::timerfd_create(CLOCK_MONOTONIC, TFD_NONBLOCK);
  if (tfd < 0) {
    LOG(ERROR) << "failed to create timer file descriptor error=(" << std::strerror(errno) << ")";
    ::close(fd);
    return EXIT_FAILURE;
  }

  struct itimerspec timerspec;
  memset(&timerspec, 0, sizeof(timerspec));
  timerspec.it_value.tv_sec = 0;
  timerspec.it_value.tv_nsec = 1;
  timerspec.it_interval.tv_sec = TEMP_POLL_SEC;
  if (::timerfd_settime(tfd, 0, &timerspec, nullptr) != 0) {
    LOG(ERROR) << "failed to set timer error=(" << std::strerror(errno) << ")";
    ::close(tfd);
    ::close(fd);
    return EXIT_FAILURE;
  }

  /**
   * Initialize epoll
   */

  int efd = ::epoll_create(1);
  if (efd < 0) {
    LOG(ERROR) << "failed to create epoll file descriptor error=(" << std::strerror(errno) << ")";
    ::close(tfd);
    ::close(fd);
    return EXIT_FAILURE;
  }

  struct epoll_event event, events[5];
  event.events = EPOLLIN;
  event.data.fd = fd;
  if (::epoll_ctl(efd, EPOLL_CTL_ADD, fd, &event)) {
    LOG(ERROR) << "failed to add serial port to epoll error=(" << std::strerror(errno) << ")";
    ::close(efd);
    ::close(tfd);
    ::close(fd);
    return EXIT_FAILURE;
  }
  event.data.fd = tfd;
  if (::epoll_ctl(efd, EPOLL_CTL_ADD, tfd, &event)) {
    LOG(ERROR) << "failed to add timer to epoll error=(" << std::strerror(errno) << ")";
    ::close(efd);
    ::close(tfd);
    ::close(fd);
    return EXIT_FAILURE;
  }

  /**
   * Event loop
   */

  std::vector<uint8_t> _recvBuf;

  for (;;) {
    int n = ::epoll_wait(efd, events, 5, 10000);
    if (n == 0)
      LOG(INFO) << "timeout";

    for (int i = 0; i < n; ++i) {
      // Serial port
      if (events[i].data.fd == fd) {
        uint8_t buf[1024];
        memset(&buf, 0, sizeof(buf));
        int r = ::read(fd, buf, sizeof(buf));
        if (r <= 0) {
          if (errno = EAGAIN)
            continue;
          LOG(ERROR) << "read error=(" << std::strerror(errno) << ")";
          break;
        }

        for (int j = 0; j < r; ++j)
          _recvBuf.push_back(buf[j]);

        if (_recvBuf.size() >= 28) {
          for (int j = 0; j < 14; ++j) {
            uint16_t val = ((uint16_t)_recvBuf[2*j] << 8) | _recvBuf[2*j+1];
            LOG(INFO) << "received temperature"
                      << " channel=" << j+1
                      << " temp=" << (val == 32767 ? "NaN" : std::to_string(val));
          }
          _recvBuf.erase(_recvBuf.begin(), _recvBuf.begin() + 28);
        }
      }

      // Timer
      else if (events[i].data.fd == tfd) {
        uint64_t expirations;
        if (::read(tfd, &expirations, sizeof(expirations)) < 0) {
          LOG(ERROR) << "read error=(" << std::strerror(errno) << ")";
          break;
        }
        LOG(INFO) << "timer fired";
        // Write query all command to thermo-scan device
        uint8_t send[] = {0x80, 0x8f};
        if (::write(fd, &send, 2) < 0) {
          LOG(ERROR) << "write error=(" << std::strerror(errno) << ")";
          break;
        }
      }

      // Invalid
      else {
        LOG(ERROR) << "invalid fd=" << events[i].data.fd;
        break;
      }
    }
  }

  /**
   * Shut down
   */

  if(::close(efd)) {
    LOG(ERROR) << "failed to close epoll file descriptor error=(" << std::strerror(errno) << ")";
    return EXIT_FAILURE;
  }
  if(::close(tfd)) {
    LOG(ERROR) << "failed to close timer file descriptor error=(" << std::strerror(errno) << ")";
    return EXIT_FAILURE;
  }
  if(::close(fd)) {
    LOG(ERROR) << "failed to close serial port file descriptor error=(" << std::strerror(errno) << ")";
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
