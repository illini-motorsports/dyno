#include "Log.hpp"

#include <cstring>

#include <errno.h>
#include <signal.h>
#include <sys/epoll.h>
#include <sys/signalfd.h>
#include <unistd.h>

struct IEventReader {
  virtual void read() = 0;
  virtual int getFd() = 0;
};

// Class which wraps epoll and manages reading file descriptors
class EventLoop {

  public:

    EventLoop();
    ~EventLoop();

    EventLoop(const EventLoop& o) = delete;
    EventLoop(EventLoop&& o) = delete;
    EventLoop& operator=(const EventLoop& o) = delete;
    EventLoop& operator=(EventLoop&& o) = delete;

    // Add file descriptor to epoll loop and listen for EPOLLIN events
    bool addReader(IEventReader* reader);

    // Start running the event loop
    int run();

  private:

    int _fd = -1;
    int _sfd = -1;
};

inline EventLoop::EventLoop()
{
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

inline EventLoop::~EventLoop()
{
  LOG(INFO) << "destroying event loop";

  if (_sfd != -1 && ::close(_sfd))
    LOG(ERROR) << "failed to close signal file descriptor error=(" << std::strerror(errno) << ")";

  if (_fd != -1 && ::close(_fd))
    LOG(ERROR) << "failed to close epoll file descriptor error=(" << std::strerror(errno) << ")";
}

inline bool EventLoop::addReader(IEventReader* reader)
{
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

inline int EventLoop::run()
{
  bool _running = true;
  struct epoll_event events[1024];

  int ret = EXIT_SUCCESS;
  while (_running) {
    int n = ::epoll_wait(_fd, events, sizeof(events), -1);
    for (int i = 0; i < n; ++i) {
      const auto& event(events[i]);
      if (event.events & EPOLLERR || event.events & EPOLLHUP) {
        LOG(ERROR) << "epoll encountered unknown error";
        ret = EXIT_FAILURE;
        _running = false;
        break;
      }
      if (!(event.events & EPOLLIN)) {
        LOG(ERROR) << "unhandled epoll event";
        ret = EXIT_FAILURE;
        _running = false;
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

  return ret;
}
