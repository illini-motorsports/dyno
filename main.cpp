#include "CsvLogger.hpp"
#include "EventLoop.hpp"
#include "Log.hpp"
#include "Timer.hpp"
#include "SerialPort.hpp"

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

  public:
    DynLoc(const char* ttyName, CsvLogger* csv)
      : _port(SerialConfig(ttyName,  B19200, true), this)
      , _csv(csv)
    {
      _speedIdx = _csv->addDataChannel("Dyno Speed", "rpm", 0);
      _torqueIdx = _csv->addDataChannel("Torque", "ft-lb", 2);
    }
    ~DynLoc() {}

    void consume(std::vector<uint8_t>& data) override {
      char ascii[4096];
      bzero(ascii, sizeof(ascii));
      char hexString[4096];
      bzero(hexString, sizeof(hexString));
      for (int i = 0; i < data.size(); ++i) {
        ::sprintf(&(hexString[2*i]), "%02x", data[i]);
        ::sprintf(&(ascii[i]), "%c", data[i]);
      }
      data.clear();
      LOG(INFO) << "received DYN-LOC data=(" << hexString << ") ascii=(" <<  ascii << ")";
    }

    IEventReader* getReader() { return &_port; }

  private:
    SerialPort _port;
    CsvLogger* _csv;
    unsigned _speedIdx = 0;
    unsigned _torqueIdx = 0;
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

  std::unique_ptr<ThermoScan> _thermoScan;
  std::unique_ptr<Timer> _thermoTimer;

  try {
    _eventLoop.reset(new EventLoop());
    _csv.reset(new CsvLogger());

    if (!dynlocPort.empty()) {
      _dynloc.reset(new DynLoc(dynlocPort.c_str(), _csv.get()));
      if (!_eventLoop->addReader(_dynloc->getReader()))
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
