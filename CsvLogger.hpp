#include "Log.hpp"

#include <cmath>
#include <cstring>
#include <fstream>
#include <vector>

// Class which handles writing data channels to a CSV file on disk
class CsvLogger {

  public:

    CsvLogger();
    ~CsvLogger();

    CsvLogger(const CsvLogger& o) = delete;
    CsvLogger(CsvLogger&& o) = delete;
    CsvLogger& operator=(const CsvLogger& o) = delete;
    CsvLogger& operator=(CsvLogger&& o) = delete;

    // Register a new data channel to be added as a column in the CSV file
    unsigned addDataChannel(const std::string& name,
                            const std::string& units,
                            const unsigned precision);

    // Save a new data value for the given data channel
    void updateVal(const unsigned idx, const double newVal);

    // Write the header line with channel names and units
    void writeHeader();

    // Write a line of most recently updated data to the output file
    void writeLine();

  private:

    struct DataChannel {
      std::string _name;
      std::string _units;
      unsigned    _precision;
      double      _latestVal;
    };

    std::ofstream            _outstream;
    std::vector<DataChannel> _data;
    unsigned                 _timeIdx;
};

inline CsvLogger::CsvLogger()
{
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

  _outstream << std::fixed;

  _timeIdx = addDataChannel("xtime", "s", 6);
}

inline CsvLogger::~CsvLogger()
{
  if (_outstream.is_open()) {
    LOG(INFO) << "closing output file";
    _outstream.close();
  }
}

inline unsigned CsvLogger::addDataChannel(const std::string& name,
                                          const std::string& units,
                                          const unsigned precision)
{
  _data.push_back({});
  _data.back()._name = name;
  _data.back()._units = units;
  _data.back()._precision = precision;
  _data.back()._latestVal = NAN;
  return _data.size() - 1;
}

inline void CsvLogger::updateVal(const unsigned idx, const double newVal)
{
  if (idx >= _data.size())
    return;

  _data[idx]._latestVal = newVal;
}

inline void CsvLogger::writeHeader()
{
  bool isFirst = true;
  for (const auto& val : _data) {
    _outstream << (isFirst ? "" : " ") << val._name << " [" << val._units << "]";
    isFirst =  false;
  }

  _outstream << "\n";
  _outstream.flush();
}

inline void CsvLogger::writeLine()
{
  struct timeval tv;
  ::gettimeofday(&tv, nullptr);
  const double now = tv.tv_sec + (double(tv.tv_usec) * 1e-6);
  updateVal(_timeIdx, now);

  bool isFirst = true;
  for (const auto& val : _data) {
    _outstream << std::setprecision(val._precision)
               << (isFirst ? "" : ",") << val._latestVal;
    isFirst =  false;
  }

  _outstream << "\n";
  _outstream.flush();
}
