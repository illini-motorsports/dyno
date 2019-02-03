#include "Log.hpp"

#include <cstring>
#include <errno.h>
#include <fcntl.h>
#include <sys/epoll.h>
#include <termios.h>
#include <unistd.h>

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
   * Initialize epoll
   */

  int efd = ::epoll_create(1);
  if (efd == -1) {
    LOG(ERROR) << "failed to create epoll file descriptor error=(" << std::strerror(errno) << ")";
    ::close(fd);
    return EXIT_FAILURE;
  }

  struct epoll_event event, events[5];
  event.events = EPOLLIN;
  event.data.fd = fd;
  if (::epoll_ctl(efd, EPOLL_CTL_ADD, fd, &event)) {
    LOG(ERROR) << "failed to add serial port to epoll error=(" << std::strerror(errno) << ")";
    ::close(fd);
    ::close(efd);
    return EXIT_FAILURE;
  }

  /**
   * Event loop
   */

  for (;;) {
    int n = ::epoll_wait(efd, events, 5, 10000);
    if (n == 0)
      LOG(INFO) << "timeout";

    for (int i = 0; i < n; ++i) {
      if (events[i].data.fd != fd) {
        LOG(ERROR) << "invalid fd=" << events[i].data.fd;
        break;
      }

      uint8_t buf[1024];
      memset(&buf, 0, sizeof(buf));
      int r = ::read(fd, buf, sizeof(buf));
      if (r < 0) {
        if (errno = EAGAIN)
          continue;
        LOG(ERROR) << "read error=(" << std::strerror(errno) << ")";
        break;
      }
      if (r == 0) {
        LOG(ERROR) << "zero bytes read";
        break;
      }

      LOG(INFO) << "read bytes len=" << r << " data=(";
      for (int j = 0; j < r; ++j)
        printf("%2X", buf[j]);
      printf("\n)\n");
    }
  }

  /**
   * Shut down
   */

  if(::close(efd)) {
    LOG(ERROR) << "failed to close epoll file descriptor error=(" << std::strerror(errno) << ")";
    return EXIT_FAILURE;
  }
  if(::close(fd)) {
    LOG(ERROR) << "failed to close serial port file descriptor error=(" << std::strerror(errno) << ")";
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
