#include "CsvLogger.hpp"
#include "EventLoop.hpp"
#include "Log.hpp"
#include "Timer.hpp"
#include "SerialPort.hpp"

#include <cassert>
#include <vector>

#define TEMP_TTY "/dev/ttyUSB1"
#define TEMP_POLL_NSEC 500000000L // 0.5s
#define DYN_TTY "/dev/ttyUSB0"

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
  std::unique_ptr<EventLoop> _eventLoop;
  std::unique_ptr<CsvLogger> _csv;

  std::unique_ptr<DynLoc> _dynLoc;

  std::unique_ptr<ThermoScan> _thermoScan;
  std::unique_ptr<Timer> _thermoTimer;

  try {
    _eventLoop.reset(new EventLoop());
    _csv.reset(new CsvLogger());

    _dynLoc.reset(new DynLoc(DYN_TTY, _csv.get()));

    _thermoScan.reset(new ThermoScan(TEMP_TTY, _csv.get()));
    _thermoTimer.reset(new Timer(TEMP_POLL_NSEC, [&_thermoScan] () { _thermoScan->queryTemp(); }));
  }
  catch (std::runtime_error) {
    return EXIT_FAILURE;
  }

  if (! _eventLoop->addReader(_dynLoc->getReader()) ||
      ! _eventLoop->addReader(_thermoScan->getReader()) ||
      ! _eventLoop->addReader(_thermoTimer.get()))
    return EXIT_FAILURE;

  _csv->writeHeader();

  // "This is where the fun begins" - Anakin Skywalker
  int ret = _eventLoop->run();

  _thermoTimer.reset(nullptr);
  _thermoScan.reset(nullptr);

  _dynLoc.reset(nullptr);

  _csv.reset(nullptr);
  _eventLoop.reset(nullptr);

  return ret;
}
