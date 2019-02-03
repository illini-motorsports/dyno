#pragma once

#include <ctime>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <sys/time.h>

enum Severity {INFO, WARN, ERROR};

static std::map<Severity, const char*> SeverityString = {
  {INFO, "I"},
  {WARN, "W"},
  {ERROR, "E"}
};

static struct timeval tv;

#define LOGB()  (Log::newline())
#define LOG(s) (Log(s))

/**
 * Simple class for logging messages
 */
class Log
{
  public:

    inline static void newline()
    {
      std::cout << std::endl;
    };

    // Print log header in constructor
    Log(Severity s)
    {
      gettimeofday(&tv, nullptr);

      _stream << "[" << SeverityString.at(s) << "] ";
      _stream << std::put_time(std::localtime(&(tv.tv_sec)), "<%m/%d %H:%M:%S.")
              << std::setfill('0') << std::setw(6) << tv.tv_usec << "> ";
      _stream << "| ";
    }

    // Print newline and flush in destructor
    ~Log()
    {
      _stream << std::endl;

      // Flush stream to cout
      std::cout << _stream.str();
    }

    Log(Log&&) = delete;
    Log(const Log&) = delete;
    void operator=(const Log&) = delete;
    void operator=(Log&&) = delete;

    // Log mesg, and return self for chaining
    template<typename T>
    inline Log& operator<<(T mesg)
    {
      _stream << mesg;
      return *this;
    }

    // Log operator specification for name-value pairs
    template<typename T>
    inline Log& operator<<(const std::pair<T,T>& p)
    {
      _stream << " " << p.first << "=(" << p.second << ")";
      return *this;
    }

  private:

    std::stringstream _stream;
};
