#include <sstream>
#include <iostream>

#include "datatamer_parser.h"
#include "PlotJuggler/fmt/format.h"

using namespace PJ;

namespace DataTamer
{
enum class ValueType
{
  UINT8,
  UINT16,
  UINT32,
  UINT64,

  INT8,
  INT16,
  INT32,
  INT64,

  FLOAT,
  DOUBLE,

  OTHER
};

ValueType FromStr(std::string const& str) {

  static std::unordered_map<std::string, ValueType> names = {
      { "UINT8", ValueType::UINT8 },
      { "UINT16", ValueType::UINT16 },
      { "UINT32", ValueType::UINT32 },
      { "UINT64", ValueType::UINT64 },

      { "INT8", ValueType::INT8 },
      { "INT16", ValueType::INT16 },
      { "INT32", ValueType::INT32 },
      { "INT64", ValueType::INT64 },

      { "FLOAT", ValueType::FLOAT },
      { "DOUBLE", ValueType::DOUBLE }
  };
  auto const it = names.find(str);
  return it == names.end() ? ValueType::OTHER : it->second;
}

}

template <typename T>
T Deserialize(const uint8_t* ptr, int& offset)
{
  T out = {};
  std::memcpy(&out, ptr + offset, sizeof(T));
  offset += sizeof(T);
  return out;
}

class DataTamerParser: public MessageParser
{
  public:
  DataTamerParser(const std::string &topic_name,
                  const std::string &type_name,
                  const std::string &schema,
                  PJ::PlotDataMapRef &data):
            MessageParser(topic_name, data),
            topic_name_(topic_name)
  {
    // the expected schema contains a series per line
    std::istringstream ss(schema);
    std::string line;

    while (std::getline(ss, line))
    {
      TimeSeries ts;
      auto pos = line.find(' ');
      ts.name = line.substr(0, pos);
      ts.type = DataTamer::FromStr(line.substr(pos+1));
      timeseries_.push_back(std::move(ts));
    }
  }

  bool parseMessage(const MessageRef serialized_msg, double& timestamp) override
  {
    int offset = 0;

    const auto* msg_ptr = serialized_msg.data();
    uint32_t flags_size = Deserialize<uint32_t>(msg_ptr, offset);

    thread_local std::vector<uint8_t> enable_vector;
    enable_vector.resize(flags_size);
    std::memcpy(enable_vector.data(), msg_ptr + offset, flags_size);
    offset += flags_size;

    const uint32_t remaining_bytes = Deserialize<uint32_t>(msg_ptr, offset);

    if(remaining_bytes + offset != serialized_msg.size())
    {
      throw std::runtime_error("DataTamerParser: corrupted size");
    }

    for(size_t i=0; i<timeseries_.size(); i++)
    {
      const uint8_t flag_byte = enable_vector[i/8];
      const uint8_t mask =  uint8_t(1 << (i%8));
      if(flag_byte & mask)
      {
        double val = 0;
        switch(timeseries_[i].type)
        {
        case DataTamer::ValueType::UINT8:
          val = static_cast<double>(Deserialize<uint8_t>(msg_ptr, offset));
          break;
        case DataTamer::ValueType::UINT16:
          val = static_cast<double>(Deserialize<uint16_t>(msg_ptr, offset));
          break;
        case DataTamer::ValueType::UINT32:
          val = static_cast<double>(Deserialize<uint32_t>(msg_ptr, offset));
          break;
        case DataTamer::ValueType::UINT64:
          val = static_cast<double>(Deserialize<uint64_t>(msg_ptr, offset));
          break;

        case DataTamer::ValueType::INT8:
          val = static_cast<double>(Deserialize<int8_t>(msg_ptr, offset));
          break;
        case DataTamer::ValueType::INT16:
          val = static_cast<double>(Deserialize<int16_t>(msg_ptr, offset));
          break;
        case DataTamer::ValueType::INT32:
          val = static_cast<double>(Deserialize<int32_t>(msg_ptr, offset));
          break;
        case DataTamer::ValueType::INT64:
          val = static_cast<double>(Deserialize<int64_t>(msg_ptr, offset));
          break;

        case DataTamer::ValueType::FLOAT:
          val = static_cast<double>(Deserialize<float>(msg_ptr, offset));
          break;
        case DataTamer::ValueType::DOUBLE:
          val = Deserialize<double>(msg_ptr, offset);
          break;
        default:
          break;
        }
        auto& ts = timeseries_[i];
        if(!ts.plot_data)
        {
          ts.plot_data = &(_plot_data.addNumeric(topic_name_ + "/" + ts.name)->second);
        }
        ts.plot_data->pushBack({timestamp, val});
      }
    }
    return true;
  }

  private:

  struct TimeSeries {
    std::string name;
    DataTamer::ValueType type;
    PlotData* plot_data = nullptr;
  };

  std::string topic_name_;
  std::vector<TimeSeries> timeseries_;
};


MessageParserPtr ParserDataTamer::createParser(const std::string &topic_name,
                                               const std::string &type_name,
                                               const std::string &schema,
                                               PJ::PlotDataMapRef &data)
{
  return std::make_shared<DataTamerParser>(topic_name, type_name, schema, data);
}
