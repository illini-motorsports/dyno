#include "Log.hpp"

#include <cassert>
#include <cmath>
#include <cstring>
#include <functional>
#include <fstream>
#include <vector>

#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <sys/epoll.h>
#include <sys/signalfd.h>
#include <sys/timerfd.h>
#include <termios.h>
#include <unistd.h>

#define TEMP_TTY "/dev/ttyUSB0"
#define TEMP_POLL_SEC 1

struct IEventReader {
  virtual void read() = 0;
  virtual int getFd() = 0;
};

// Class which wraps epoll and manages readers
class EventLoop {
  public:
    EventLoop() {
      LOG(INFO) << "creating event loop";
      _fd = ::epoll_create(1);
      if (_fd < 0) {
        LOG(ERROR) << "failed to create epoll file descriptor error=(" << std::strerror(errno) << ")";
        throw std::runtime_error("");
      }

      // Add exit signal handler
      {
        sigset_t mask;
        ::sigemptyset(&mask);
        ::sigaddset(&mask, SIGTERM);
        ::sigaddset(&mask, SIGINT);
        if (::sigprocmask(SIG_BLOCK, &mask, nullptr) < 0) {
          LOG(ERROR) << "failed to set signal mask error=(" << std::strerror(errno) << ")";
          throw std::runtime_error("");
        }
        _sfd = ::signalfd(-1, &mask, SFD_NONBLOCK);
        if (_sfd < 0) {
          LOG(ERROR) << "failed to signal file descriptor error=(" << std::strerror(errno) << ")";
          throw std::runtime_error("");
        }
        struct epoll_event event;
        event.events = EPOLLIN;
        event.data.fd = _sfd;
        if (::epoll_ctl(_fd, EPOLL_CTL_ADD, _sfd, &event)) {
          LOG(ERROR) << "failed to add signal reader to epoll error=(" << std::strerror(errno) << ")";
          throw std::runtime_error("");
        }
      }
    }
    ~EventLoop() {
      LOG(INFO) << "destroying event loop";
      if (_sfd != -1 && ::close(_sfd))
        LOG(ERROR) << "failed to close signal file descriptor error=(" << std::strerror(errno) << ")";
      if (_fd != -1 && ::close(_fd))
        LOG(ERROR) << "failed to close epoll file descriptor error=(" << std::strerror(errno) << ")";
    }
    EventLoop(const EventLoop& o) = delete;
    EventLoop(EventLoop&& o) = delete;
    EventLoop& operator=(const EventLoop& o) = delete;
    EventLoop& operator=(EventLoop&& o) = delete;

    bool addReader(IEventReader* reader) {
      struct epoll_event event;
      event.events = EPOLLIN;
      event.data.fd = reader->getFd();
      event.data.ptr = reader;
      if (::epoll_ctl(_fd, EPOLL_CTL_ADD, reader->getFd(), &event)) {
        LOG(ERROR) << "failed to add reader to epoll error=(" << std::strerror(errno) << ")";
        return false;
      }
      return true;
    }

    int run() {
      bool _running = true;
      struct epoll_event events[5];
      while (_running) {
        int n = ::epoll_wait(_fd, events, 5, -1);
        for (int i = 0; i < n; ++i) {
          const auto& event(events[i]);
          if (event.events & EPOLLERR || event.events & EPOLLHUP) {
            LOG(ERROR) << "epoll encountered unknown error";
            break;
          }
          if (!(event.events & EPOLLIN)) {
            LOG(ERROR) << "unhandled epoll event";
            break;
          }
          if (event.data.fd == _sfd) {
            LOG(INFO) << "caught signal, exiting";
            _running = false;
            break;
          }
          ((IEventReader*) event.data.ptr)->read();
        }
      }
      return EXIT_SUCCESS;
    }

  private:
    int _fd = -1;
    int _sfd = -1;
};

// Periodic timer with callback
class Timer : public IEventReader {
  public:
    Timer(uint32_t secInterval, std::function<void(void)> callback) {
      _callback = callback;

      LOG(INFO) << "creating timer with interval=" << secInterval;

      _fd = ::timerfd_create(CLOCK_MONOTONIC, TFD_NONBLOCK);
      if (_fd < 0) {
        LOG(ERROR) << "failed to create timer error=(" << std::strerror(errno) << ")";
        throw std::runtime_error("");
      }

      struct itimerspec timerspec;
      bzero(&timerspec, sizeof(timerspec));
      timerspec.it_value.tv_sec = 0;
      timerspec.it_value.tv_nsec = 1;
      timerspec.it_interval.tv_sec = secInterval;

      if (::timerfd_settime(_fd, 0, &timerspec, nullptr) != 0) {
        LOG(ERROR) << "failed to set timer error=(" << std::strerror(errno) << ")";
        ::close(_fd);
        throw std::runtime_error("");
      }
    }
    ~Timer() {
      LOG(INFO) << "destroying timer fd=" << _fd;
      if(_fd != -1 && ::close(_fd))
        LOG(ERROR) << "failed to close timer file descriptor error=(" << std::strerror(errno) << ")";
    }
    Timer(const Timer& o) = delete;
    Timer(Timer&& o) = delete;
    Timer& operator=(const Timer& o) = delete;
    Timer& operator=(Timer&& o) = delete;

    void read() override {
      uint64_t expirations;
      if (::read(_fd, &expirations, sizeof(expirations)) < 0) {
        LOG(ERROR) << "timer read error=(" << std::strerror(errno) << ")";
      }
      //LOG(INFO) << "timer fired fd=" << _fd;
      _callback();
    }

    int getFd() override {
      return _fd;
    }

  private:
    int _fd = -1;
    std::function<void(void)> _callback;
};

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

class CsvLogger {
  public:
    CsvLogger() {
      std::ostringstream str;
      std::time_t t = std::time(0);
      str << "dynologger-" << std::put_time(std::localtime(&t), "%Y-%m-%d::%H:%M:%S") << ".csv";
      const std::string outfileName = str.str();

      LOG(INFO) << "opening output csv file name=" << outfileName;
      _outstream.open(outfileName);
      if (!_outstream.is_open()) {
        LOG(ERROR) << "failed to open output file error=(" << std::strerror(errno) << ")";
        throw std::runtime_error("");
      }
      _outstream << std::fixed << std::setprecision(6);

      _timeIdx = addDataVal("xtime", "s");
    };
    ~CsvLogger() {
      if (_outstream.is_open()) {
        LOG(INFO) << "closing output file";
        _outstream.close();
      }
    }

    unsigned addDataVal(const std::string& name, const std::string& units) {
      _data.push_back({});
      _data.back()._name = name;
      _data.back()._units = units;
      _data.back()._latestVal = NAN;
      return _data.size() - 1;
    }

    void updateVal(const unsigned idx, const double newVal) {
      if (idx >= _data.size())
        return;
      _data[idx]._latestVal = newVal;
    }

    void writeHeader() {
      bool isFirst = true;
      for (const auto& val : _data) {
        _outstream << (isFirst ? "" : " ") << val._name << " [" << val._units << "]";
        isFirst =  false;
      }
      _outstream << "\n";
      _outstream.flush();
    }

    void writeLine() {
      struct timeval tv;
      ::gettimeofday(&tv, nullptr);
      const double now = tv.tv_sec + (double(tv.tv_usec) * 1e-6);
      updateVal(_timeIdx, now);

      bool isFirst = true;
      for (const auto& val : _data) {
        _outstream << (isFirst ? "" : ",") << val._latestVal;
        isFirst =  false;
      }
      _outstream << "\n";
      _outstream.flush();
    }

  private:
    struct DataVal {
      std::string _name;
      std::string _units;
      double _latestVal;
    };

    std::ofstream _outstream;
    std::vector<DataVal> _data;
    unsigned _timeIdx;
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

    _tempTimer.reset(new Timer(TEMP_POLL_SEC, [&_tempPort] () {
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
      _firstTempIdx = _csv->addDataVal(name, units);
    else
      _csv->addDataVal(name, units);
  }

  _csv->writeHeader();

  int ret = _eventLoop->run();

  _csv.reset(nullptr);
  _tempTimer.reset(nullptr);
  _tempPort.reset(nullptr);
  _eventLoop.reset(nullptr);

  return ret;
}
