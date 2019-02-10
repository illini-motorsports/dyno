#include "Log.hpp"

#include <functional>

#include <sys/timerfd.h>

// Periodic timer with callback
class Timer : public IEventReader {

  public:

    Timer(uint64_t nsecInterval, std::function<void(void)> callback);
    ~Timer();

    Timer(const Timer& o) = delete;
    Timer(Timer&& o) = delete;
    Timer& operator=(const Timer& o) = delete;
    Timer& operator=(Timer&& o) = delete;

    void read() override;
    int getFd() override { return _fd; }

    static int64_t getTime();

  private:

    int _fd = -1;
    std::function<void(void)> _callback;
};

inline Timer::Timer(uint64_t nsecInterval, std::function<void(void)> callback)
  : _callback(callback)
{
  LOG(INFO) << "creating timer with nano interval=" << nsecInterval;

  _fd = ::timerfd_create(CLOCK_MONOTONIC, TFD_NONBLOCK);
  if (_fd < 0) {
    LOG(ERROR) << "failed to create timer error=(" << std::strerror(errno) << ")";
    throw std::runtime_error("");
  }

  struct itimerspec timerspec;
  bzero(&timerspec, sizeof(timerspec));
  timerspec.it_value.tv_sec = 0;
  timerspec.it_value.tv_nsec = 1;
  timerspec.it_interval.tv_sec = nsecInterval / 1000000000L;
  timerspec.it_interval.tv_nsec = nsecInterval % 1000000000L;

  if (::timerfd_settime(_fd, 0, &timerspec, nullptr) != 0) {
    LOG(ERROR) << "failed to set timer error=(" << std::strerror(errno) << ")";
    ::close(_fd);
    throw std::runtime_error("");
  }
}

inline Timer::~Timer()
{
  LOG(INFO) << "destroying timer fd=" << _fd;

  if (_fd != -1 && ::close(_fd))
    LOG(ERROR) << "failed to close timer file descriptor error=(" << std::strerror(errno) << ")";
}

inline void Timer::read()
{
  uint64_t expirations;
  if (::read(_fd, &expirations, sizeof(expirations)) < 0 || expirations == 0) {
    LOG(ERROR) << "timer read error=(" << std::strerror(errno) << ")";
  }

  _callback();
}

inline int64_t Timer::getTime()
{
  static struct timeval val;
  ::gettimeofday(&val, nullptr);
  return (val.tv_sec * 1000000000L) + (val.tv_usec * 1000L);
}
