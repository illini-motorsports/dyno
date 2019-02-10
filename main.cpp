#include "CsvLogger.hpp"
#include "EventLoop.hpp"
#include "Log.hpp"
#include "Timer.hpp"
#include "SerialPort.hpp"

#include <algorithm>

#include <assert.h>
#include <getopt.h>

void usage() {
  using namespace std;
  cout << "USAGE: dyno-logger" << endl;
  cout << "Data acquisition tool for logging all dyno data in a single CSV file" << endl;
  cout << endl;
  cout << "ARGUMENTS" << endl;
  cout << "  -d, --dynloc-port [TTY]     Name of the serial port for the DYN-LOC IV" << endl;
  cout << "  -t, --thermo-port [TTY]     Name of the serial port for the ETAS Thermo-Scan" << endl;
  cout << "  -T, --thermo-poll [msec]    Polling interval for the Thermo-Scan (in milliseconds)" << endl;
  cout << "                                - Defaults to 500ms" << endl;
  cout << "  -h, --help                  Output this help text" << endl;
  cout << endl;
  cout << "NOTES" << endl;
  cout << "  - The logger will only connect to the devices for which you specify" << endl;
  cout << "    tty / serial port names. You can connect to all of them or only" << endl;
  cout << "    some of them, but you must connect to at least one." << endl;
}

// Handles reading temps from the ETAS Thermo-Scan thermocouple reader
class ThermoScan : public ISerialPort {
  public:
    ThermoScan(const char* ttyName, CsvLogger* csv)
      : _port(SerialConfig(ttyName, B38400, false), this)
      , _csv(csv)
    {
      for (int j = 0; j < 14; ++j) {
        std::string name = "Temp" + std::to_string(j+1);
        static std::string units = "C";
        if (j == 0)
          _firstTempIdx = _csv->addDataChannel(name, units, 0);
        else
          _csv->addDataChannel(name, units, 0);
      }
    }
    ~ThermoScan() {}

    // Process responses from the thermo-scan
    void consume(std::vector<uint8_t>& data) override {
      if (data.size() >= 28) {
        assert(_firstTempIdx != 0);
        for (int j = 0; j < 14; ++j) {
          uint16_t val = ((uint16_t)data[2*j] << 8) | data[2*j+1];
          double temp = (val == 32767 ? NAN : val);
          _csv->updateVal(_firstTempIdx + j, temp);
        }
        data.erase(data.begin(), data.begin() + 28);
        _csv->writeLine();
      }
    }

    // Send query-all command to the themo-scan device
    void queryTemp() {
      uint8_t send[] = {0x80, 0x8f};
      if (::write(_port.getFd(), &send, sizeof(send)) < 0)
        LOG(ERROR) << "write error=(" << std::strerror(errno) << ")";
    }

    IEventReader* getReader() { return &_port; }

  private:
    SerialPort _port;
    CsvLogger* _csv;
    unsigned _firstTempIdx = 0;
};

// Handles reading speed & torque from the DYN-LOK IV dyno controller
class DynLoc : public ISerialPort {

  /*
   * DYNE-LOC IV Data Acquisition Sequence
   * -------------------------------------
   *
   * *INIT*
   * SYS OPEN
   * SYS ECHO OFF
   * SYS CLOSE
   *
   * *LOOP*
   * DLC SR
   * DLC DR TQ
   * DLC DR RPM
   */

  #define BREAK "\x03"
  #define SYS_OPEN "SYS OPEN\x0d"
  #define SYS_ECHO_OFF "SYS ECHO OFF\x0d"
  #define SYS_CLOSE "SYS CLOSE\x0d"
  #define REQ_TQ "DLC DR TQ\x0d"
  #define REQ_SP "DLC DR RPM\x0d"
  #define CRLF "\x0d\x0a"
  #define PROMPT "\\"
  #define BREAK_ECHO "\x5e\x43"

  public:
    DynLoc(const char* ttyName, CsvLogger* csv)
      : _port(SerialConfig(ttyName,  B19200, true), this)
      , _csv(csv)
    {
      _speedIdx = _csv->addDataChannel("Dyno Speed", "rpm", 0);
      _torqueIdx = _csv->addDataChannel("Torque", "ft-lb", 1);

      LOG(INFO) << "writing BREAK to dynloc";
      write(BREAK);
    }
    ~DynLoc() {}

    IEventReader* getReader() { return &_port; }

    // Check the beginning of the buffer for a string sequence
    template<size_t strLen>
    bool startsWith(const std::vector<uint8_t>& data, const char (&tag) [strLen]) {
      return data.size() >= strLen - 1 && ::memcmp(data.data(), tag, strLen - 1) == 0;
    }

    // Check the beginning of the buffer for a string sequence and consume it
    template<size_t strLen>
    bool checkAndErase(std::vector<uint8_t>& data, const char (&tag) [strLen]) {
      if (data.size() >= strLen - 1 && ::memcmp(data.data(), tag, strLen - 1) == 0) {
        data.erase(data.begin(), data.begin() + strLen - 1);
        return true;
      }
      return false;
    }

    // Write the string sequence to the serial port
    template<size_t len>
    void write(const char (&data) [len]) {
      int n;
      if ((n = ::write(_port.getFd(), data, len - 1)) < len - 1)
        LOG(ERROR) << "write error n=" << n << " error=(" << std::strerror(errno) << ")";
    }

    // Periodically check whether we need to re-request data
    void checkTimeout() {
      if (_reqState == RequestState::NONE)
        return;
      const auto now = Timer::getTime();
      if (now - _lastRecv >= 500000000L) { // 0.5s
        LOG(WARN) << "response timeout, writing BREAK to dynloc";
        write(BREAK);
      }
    }

    // Parse responses from the DYN-LOC IV
    void consume(std::vector<uint8_t>& data) override {
      // Strip off any unnecessary sequences
      while (checkAndErase(data, BREAK_ECHO) || checkAndErase(data, CRLF));

      while (!data.empty()) {
        // See if we have a complete line yet (contains the prompt)
        auto eol = std::find(data.begin(), data.end(), *PROMPT);
        if (eol == data.end())
          return;

        // Empty prompt
        if (data.front() == *PROMPT) {
          if (!_sentOpen) {
            LOG(INFO) << "received empty prompt from dynloc, writing SYS OPEN";
            write(SYS_OPEN);
            _sentOpen = true;
          } else if (!_sentClose) {
            LOG(INFO) << "received empty prompt from dynloc, writing SYS CLOSE";
            write(SYS_CLOSE);
            _sentClose = true;
          } else {
            LOG(INFO) << "received empty prompt from dynloc, restarting data loop";
            write(REQ_TQ);
            _reqState = RequestState::TORQUE;
          }
        }

        // Open response
        else if (startsWith(data, "sys open" CRLF PROMPT)) {
          LOG(INFO) << "received sys opened from dynloc, writing ECHO OFF";
          write(SYS_ECHO_OFF);
        }

        // Close response
        else if (startsWith(data, "sys closed" CRLF PROMPT)) {
          LOG(INFO) << "received sys closed from dynloc, starting data loop";
          write(REQ_TQ);
          _reqState = RequestState::TORQUE;
        }

        // Data loop responses
        else {
          char ascii[64];
          bzero(ascii, sizeof(ascii));

          // Torque response
          if (_reqState == RequestState::TORQUE &&
              (data.size() > 3 && ::memcmp(&(*eol) - 2, CRLF PROMPT, 3) == 0)) {
            for (int i = 0; i < data.size() - 3; ++i)
              ::sprintf(ascii + i, "%c", data[i]);

            try {
              _csv->updateVal(_torqueIdx, std::stod(ascii));
              _csv->writeLine();
              _lastRecv = Timer::getTime();
            } catch (std::invalid_argument e) {
              LOG(ERROR) << "invalid torque string error=(" << e.what() << ") string=(" << ascii << ")";
            }

            write(REQ_SP);
            _reqState = RequestState::SPEED;
          }

          // Speed response
          else if (_reqState == RequestState::SPEED &&
                   (data.size() > 3 && ::memcmp(&(*eol) - 2, CRLF PROMPT, 3) == 0)) {
            for (int i = 0; i < data.size() - 3; ++i)
              ::sprintf(ascii + i, "%c", data[i]);

            try {
              _csv->updateVal(_speedIdx, std::stod(ascii));
              _csv->writeLine();
              _lastRecv = Timer::getTime();
            } catch (std::invalid_argument e) {
              LOG(ERROR) << "invalid speed string error=(" << e.what() << ") string=(" << ascii << ")";
            }

            write(REQ_TQ);
            _reqState = RequestState::TORQUE;
          }

          // Error
          else {
            LOG(ERROR) << "unhandled message";
          }
        }

        data.erase(data.begin(), eol + 1); // Clear the consumed line
      }
    }

  private:

    enum class RequestState {
      NONE = 0,
      TORQUE = 1,
      SPEED = 2,
    };

    SerialPort _port;
    CsvLogger* _csv;
    unsigned _speedIdx = 0;
    unsigned _torqueIdx = 0;
    bool _sentOpen = false;
    bool _sentClose = false;
    RequestState _reqState = RequestState::NONE;
    int64_t _lastRecv = 0;
};

int main(int argc, char* argv[])
{
  std::string dynlocPort;
  std::string thermoPort;
  unsigned thermoPollMs = 500;

  const char* optstring = "d:t:T:h";
  struct option options[] = {
    {"dynloc-port", required_argument, 0, 'd'},
    {"thermo-port", required_argument, 0, 't'},
    {"thermo-poll", required_argument, 0, 'T'},
    {"help",        no_argument,       0, 'h'}
  };
  char opt;
  while ((opt = ::getopt_long(argc, argv, optstring, options, nullptr)) != -1) {
    switch (opt) {
      case 'd':
        dynlocPort = optarg;
        LOG(INFO) << "using dynloc port=(" << dynlocPort << ")";
        break;
      case 't':
        thermoPort = optarg;
        LOG(INFO) << "using thermo port=(" << thermoPort << ")";
        break;
      case 'T':
        thermoPollMs = ::atoi(optarg);
        LOG(INFO) << "using thermo poll=(" << thermoPollMs << ")";
        break;
      case 'h':
        usage();
        return EXIT_SUCCESS;;
      default:
        usage();
        return EXIT_FAILURE;
    }
  }

  if (thermoPort.empty() && dynlocPort.empty()) {
    usage();
    return EXIT_FAILURE;
  }

  std::unique_ptr<EventLoop> _eventLoop;
  std::unique_ptr<CsvLogger> _csv;

  std::unique_ptr<DynLoc> _dynloc;
  std::unique_ptr<Timer> _dynlocTimer;

  std::unique_ptr<ThermoScan> _thermoScan;
  std::unique_ptr<Timer> _thermoTimer;

  try {
    _eventLoop.reset(new EventLoop());
    _csv.reset(new CsvLogger());

    if (!dynlocPort.empty()) {
      _dynloc.reset(new DynLoc(dynlocPort.c_str(), _csv.get()));
      _dynlocTimer.reset(new Timer(250000000L, [&_dynloc] () { _dynloc->checkTimeout(); }));
      if (!_eventLoop->addReader(_dynloc->getReader()) ||
          !_eventLoop->addReader(_dynlocTimer.get()))
        return EXIT_FAILURE;
    }

    if (!thermoPort.empty()) {
      _thermoScan.reset(new ThermoScan(thermoPort.c_str(), _csv.get()));
      _thermoTimer.reset(new Timer(thermoPollMs * 1000000L, [&_thermoScan] () { _thermoScan->queryTemp(); }));
      if (!_eventLoop->addReader(_thermoScan->getReader()) ||
          !_eventLoop->addReader(_thermoTimer.get()))
        return EXIT_FAILURE;
    }
  }
  catch (std::runtime_error) {
    return EXIT_FAILURE;
  }

  _csv->writeHeader();

  // "This is where the fun begins" - Anakin Skywalker
  int ret = _eventLoop->run();

  _thermoTimer.reset(nullptr);
  _thermoScan.reset(nullptr);

  _dynloc.reset(nullptr);

  _csv.reset(nullptr);
  _eventLoop.reset(nullptr);

  return ret;
}
