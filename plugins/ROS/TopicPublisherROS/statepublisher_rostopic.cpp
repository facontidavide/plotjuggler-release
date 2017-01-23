#include "statepublisher_rostopic.h"
#include <PlotJuggler/any.hpp>
#include "../shape_shifter_factory.hpp"
#include "../qnodedialog.h"

TopicPublisherROS::TopicPublisherROS():
  enabled_(false )
{

}

TopicPublisherROS::~TopicPublisherROS()
{

}

void TopicPublisherROS::setEnabled(bool to_enable)
{
  enabled_ = to_enable;

  if( enabled_ )
  {
    node_ = getGlobalRosNode();

    if( node_ ) enabled_ = true;
    else        enabled_ = false;
  }
}


void TopicPublisherROS::updateState(PlotDataMap *datamap, double current_time)
{
  static long count = 0;
  if(!enabled_) return;

  for(const auto& data_it:  datamap->user_defined )
  {
    const std::string&    topic_name = data_it.first;

    nonstd::optional<RosIntrospection::ShapeShifter*> registered_shapeshifted_msg = ShapeShifterFactory::getInstance().getMessage( topic_name );
    if( ! registered_shapeshifted_msg )
    {
      // will not be able to use this anyway, just skip
      continue;
    }

    RosIntrospection::ShapeShifter* shapeshifted_msg = *registered_shapeshifted_msg;
    const PlotDataAnyPtr& plot_any = data_it.second;

    nonstd::optional<nonstd::any> any_value = plot_any->getYfromX( current_time );

    if(!any_value || any_value->type() != typeid( std::vector<uint8_t> ))
    {
      // can't cast to expected type
      continue;
    }

    std::vector<uint8_t> raw_buffer =  nonstd::any_cast<std::vector<uint8_t>>( any_value );

    ros::serialization::IStream stream( raw_buffer.data(), raw_buffer.size() );
    shapeshifted_msg->read( stream );

    auto publisher_it = publishers_.find( topic_name );
    if( publisher_it == publishers_.end())
    {
       auto res = publishers_.insert( std::make_pair(topic_name, shapeshifted_msg->advertise( *node_, topic_name, 10, true) ) );
        publisher_it = res.first;
    }

    const ros::Publisher& publisher = publisher_it->second;
    publisher.publish( *shapeshifted_msg );

    qDebug() << "publishing " << count++; ;

  }

}
