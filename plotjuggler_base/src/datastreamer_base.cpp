#include "PlotJuggler/datastreamer_base.h"

namespace PJ
{

void DataStreamer::setAvailableParsers(std::shared_ptr<MessageParserFactory> parsers_factory) {
  _available_parsers = parsers_factory;
}

std::shared_ptr<MessageParserFactory> DataStreamer::availableParsers()
{
  if( _available_parsers && _available_parsers->empty() ) {
    return {};
  }
  return _available_parsers;
}

void PJ::DataStreamer::setMaximumRangeX(double range)
{
  std::lock_guard<std::mutex> lock(mutex());
  for (auto& it : dataMap().numeric)
  {
    it.second.setMaximumRangeX(range);
  }
  for (auto& it : dataMap().strings)
  {
    it.second.setMaximumRangeX(range);
  }
  for (auto& it : dataMap().user_defined)
  {
    it.second.setMaximumRangeX(range);
  }
}


}
