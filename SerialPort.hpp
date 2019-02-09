#include "Log.hpp"

#include <assert.h>
#include <fcntl.h>
#include <termios.h>

#define READ_SIZE 256

struct SerialConfig {
  const char* _ttyName = nullptr;
  speed_t     _baud    = B0;
  bool        _ctsrts  = false;
  SerialConfig(const char* ttyName, speed_t baud, bool ctsrts)
    : _ttyName(ttyName), _baud(baud), _ctsrts(ctsrts) {}
};

struct ISerialPort {
  virtual void consume(std::vector<uint8_t>& data) = 0;
};

// Class which handles reading from and writing to a serial port
class SerialPort : public IEventReader {

  public:

    SerialPort(const SerialConfig& cfg, ISerialPort* handler);
    virtual ~SerialPort();

    SerialPort(const SerialPort& o) = delete;
    SerialPort(SerialPort&& o) = delete;
    SerialPort& operator=(const SerialPort& o) = delete;
    SerialPort& operator=(SerialPort&& o) = delete;

    void read() override;
    int getFd() override { return _fd; }

  private:

    int _fd = -1;
    ISerialPort* _handler = nullptr;
    std::vector<uint8_t> _recvBuf;
};

inline SerialPort::SerialPort(const SerialConfig& cfg, ISerialPort* handler)
  : _handler(handler)
{
  LOG(INFO) << "opening serial port name=(" << cfg._ttyName << ")";

  _fd = ::open(cfg._ttyName, O_RDWR | O_NOCTTY | O_SYNC | O_NONBLOCK);
  if (_fd < 0) {
    LOG(ERROR) << "failed to open serial port error=(" << std::strerror(errno) << ")";
    throw std::runtime_error("");
  }
  LOG(INFO) << "opened serial port name=(" << cfg._ttyName << ") fd=" << _fd;

  struct termios tty;
  ::memset(&tty, 0, sizeof(tty));
  if (::tcgetattr(_fd, &tty) != 0) {
    LOG(ERROR) << "failed to get current serial port config error=(" << std::strerror(errno) << ")";
    ::close(_fd);
    throw std::runtime_error("");
  }

  ::cfsetspeed(&tty, cfg._baud); // Baud rate
  ::cfmakeraw(&tty); // Raw mode, 8-bit chars, no parity
  tty.c_cflag &= ~CSTOPB; // 1 stop bit

  // Hardware flow control
  if (cfg._ctsrts)
    tty.c_cflag |= CRTSCTS;
  else
    tty.c_cflag &= ~CRTSCTS;

  ::tcflush(_fd, TCIOFLUSH);
  if (::tcsetattr(_fd, TCSANOW, &tty) != 0) {
    LOG(ERROR) << "failed to set serial port config error=(" << std::strerror(errno) << ")";
    ::close(_fd);
    throw std::runtime_error("");
  }
}

inline SerialPort::~SerialPort()
{
  if (_fd != -1) {
    LOG(INFO) << "closing serial port fd=" << _fd;
    ::tcflush(_fd, TCIOFLUSH);
    if (::close(_fd))
      LOG(ERROR) << "failed to close serial port error=(" << std::strerror(errno) << ")";
  }
}

inline void SerialPort::read() {
  const size_t origSize = _recvBuf.size();

  for (;;) {
    const size_t preReadSize = _recvBuf.size();
    _recvBuf.resize(preReadSize + READ_SIZE);
    uint8_t* writePtr = _recvBuf.data() + preReadSize;

    int r = ::read(_fd, writePtr, READ_SIZE);
    if (r <= 0) {
      if (r < 0) {
        if (errno != EAGAIN)
          LOG(ERROR) << "read error=(" << std::strerror(errno) << ")";
      }
      _recvBuf.resize(preReadSize);
      break;
    }
    _recvBuf.resize(preReadSize + r);
  }

  if (_handler && _recvBuf.size() != origSize)
    _handler->consume(_recvBuf);
}
