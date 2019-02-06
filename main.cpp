#include "CsvLogger.hpp"
#include "EventLoop.hpp"
#include "Log.hpp"
#include "Timer.hpp"

#include <cassert>
#include <vector>

#include <fcntl.h>
#include <termios.h>

#define TEMP_TTY "/dev/ttyUSB0"
#define TEMP_POLL_NSEC 500000000L // 0.5s

// Class to read/write from a serial port
class SerialPort : public IEventReader {
  public:
    SerialPort(const char* ttyName, speed_t baud,
               std::function<void(std::vector<uint8_t>&)> callback) {
      _callback = callback;

      LOG(INFO) << "opening serial port name=(" << ttyName << ")";

      _fd = ::open(ttyName, O_RDWR | O_NOCTTY | O_SYNC | O_NONBLOCK);
      if (_fd < 0) {
        LOG(ERROR) << "failed to open serial port error=(" << std::strerror(errno) << ")";
        throw std::runtime_error("");
      }
      LOG(INFO) << "opened serial port name=(" << ttyName << ") fd=" << _fd;

      struct termios tty;
      ::memset(&tty, 0, sizeof(tty));
      if (::tcgetattr(_fd, &tty) != 0) {
        LOG(ERROR) << "failed to get serial port config error=(" << std::strerror(errno) << ")";
        ::close(_fd);
        throw std::runtime_error("");
      }

      ::cfsetspeed(&tty, baud); // Baud rate
      ::cfmakeraw(&tty);
      tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8-bit chars
      tty.c_cflag &= ~(PARENB | PARODD); // Disable parity
      tty.c_cflag &= ~CSTOPB; // 1 stop bit
      tty.c_cflag &= ~CRTSCTS; // No hardware flow control
      ::tcflush(_fd, TCIOFLUSH);
      if (::tcsetattr(_fd, TCSANOW, &tty) != 0) {
        LOG(ERROR) << "failed to set serial port config error=(" << std::strerror(errno) << ")";
        ::close(_fd);
        throw std::runtime_error("");
      }
    }
    ~SerialPort() {
      LOG(INFO) << "closing serial port fd=" << _fd;
      if (_fd != -1 && ::close(_fd))
        LOG(ERROR) << "failed to close serial port error=(" << std::strerror(errno) << ")";
    }

    void read() override {
      //TODO: Make this better
      uint8_t buf[1024];
      memset(&buf, 0, sizeof(buf));
      int r = ::read(_fd, buf, sizeof(buf));
      if (r <= 0) {
        if (errno != EAGAIN)
          LOG(ERROR) << "read error=(" << std::strerror(errno) << ")";
        return;
      }
      for (int j = 0; j < r; ++j)
        _recvBuf.push_back(buf[j]);
      _callback(_recvBuf);
    }

    int getFd() override {
      return _fd;
    }

  private:
    int _fd = -1;
    std::vector<uint8_t> _recvBuf;
    std::function<void(std::vector<uint8_t>&)> _callback;
};

int main(int argc, char* argv[])
{
  unsigned _firstTempIdx = 0;

  std::unique_ptr<EventLoop> _eventLoop;
  std::unique_ptr<SerialPort> _tempPort;
  std::unique_ptr<Timer> _tempTimer;
  std::unique_ptr<CsvLogger> _csv;

  try {
    _eventLoop.reset(new EventLoop());

    _tempPort.reset(new SerialPort(TEMP_TTY, B38400, [&] (std::vector<uint8_t>& buf) {
      if (buf.size() >= 28) {
        assert(_firstTempIdx != 0);
        for (int j = 0; j < 14; ++j) {
          uint16_t val = ((uint16_t)buf[2*j] << 8) | buf[2*j+1];
          double temp = (val == 32767 ? NAN : val);
          _csv->updateVal(_firstTempIdx + j, temp);
        }
        buf.erase(buf.begin(), buf.begin() + 28);
        _csv->writeLine();
      }
    }));

    _tempTimer.reset(new Timer(TEMP_POLL_NSEC, [&_tempPort] () {
      // Write query all command to thermo-scan device
      uint8_t send[] = {0x80, 0x8f};
      if (::write(_tempPort->getFd(), &send, sizeof(send)) < 0)
        LOG(ERROR) << "write error=(" << std::strerror(errno) << ")";
    }));

    _csv.reset(new CsvLogger());
  } catch (std::runtime_error) {
    return EXIT_FAILURE;
  }

  if (!_eventLoop->addReader(_tempPort.get()) || !_eventLoop->addReader(_tempTimer.get()))
    return EXIT_FAILURE;

  for (int j = 0; j < 14; ++j) {
    std::string name = "Temp" + std::to_string(j+1);
    static std::string units = "C";
    if (j == 0)
      _firstTempIdx = _csv->addDataChannel(name, units, 0);
    else
      _csv->addDataChannel(name, units, 0);
  }

  _csv->writeHeader();

  int ret = _eventLoop->run();

  _csv.reset(nullptr);
  _tempTimer.reset(nullptr);
  _tempPort.reset(nullptr);
  _eventLoop.reset(nullptr);

  return ret;
}
