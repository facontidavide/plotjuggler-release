#include "statepublisher_rostopic.h"
#include "PlotJuggler/any.hpp"
#include "../shape_shifter_factory.hpp"
#include "../qnodedialog.h"
#include "ros_type_introspection/ros_introspection.hpp"
#include <rosbag/bag.h>
#include <std_msgs/Header.h>

TopicPublisherROS::TopicPublisherROS():
  enabled_(false ),
  _node(nullptr)
{
}

TopicPublisherROS::~TopicPublisherROS()
{
  enabled_ = false;
}

void TopicPublisherROS::setParentMenu(QMenu *menu)
{
  _menu = menu;
  _current_time = new QAction(QString("use current time in std_msg/Header "), _menu);
  _current_time->setCheckable(true);
  _current_time->setChecked(true);
  _menu->addAction( _current_time );
}

void TopicPublisherROS::setEnabled(bool to_enable)
{  
  if( !_node )
  {
    _node = RosManager::getNode();
  }
  enabled_ = (to_enable && _node);
}


void TopicPublisherROS::updateState(PlotDataMap *datamap, double current_time)
{
  if(!enabled_ || !_node) return;

  const ros::Time ros_time = ros::Time::now();

  for(const auto& data_it:  datamap->user_defined )
  {
    const std::string&    topic_name = data_it.first;

    const RosIntrospection::ShapeShifter* registered_shapeshifted_msg = RosIntrospectionFactory::get().getShapeShifter( topic_name );
    if( ! registered_shapeshifted_msg )
    {
      // Not registered, just skip
      continue;
    }

    RosIntrospection::ShapeShifter shapeshifted_msg = *registered_shapeshifted_msg;
    const PlotDataAnyPtr& plot_any = data_it.second;

    nonstd::optional<nonstd::any> any_value = plot_any->getYfromX( current_time );

    if(!any_value)
    {
      // can't cast "any" to expected type
      continue;
    }
    const bool isRawBuffer     = any_value->type() == typeid( std::vector<uint8_t>);
    const bool isRosbagMessage = any_value->type() == typeid(rosbag::MessageInstance);

    std::vector<uint8_t> raw_buffer;

    if( isRawBuffer){
      raw_buffer = nonstd::any_cast<std::vector<uint8_t>>( any_value.value() );
    }
    else if( isRosbagMessage ){
      const rosbag::MessageInstance& msg_instance = nonstd::any_cast<rosbag::MessageInstance>( any_value.value() );
      raw_buffer.resize( msg_instance.size() );
      ros::serialization::OStream stream(raw_buffer.data(), raw_buffer.size());
      msg_instance.write(stream);
    }
    else{
      continue;
    }

    if( _current_time->isChecked())
    {
      const RosIntrospection::Parser::VisitingCallback modifyTimestamp = [&ros_time](const RosIntrospection::ROSType&, absl::Span<uint8_t>& buffer)
      {
        std_msgs::Header msg;
        ros::serialization::IStream is( buffer.data(), buffer.size() );
        ros::serialization::deserialize(is, msg);
        msg.stamp = ros_time;
        ros::serialization::OStream os( buffer.data(), buffer.size() );
        ros::serialization::serialize(os, msg);
      };

      auto msg_info = RosIntrospectionFactory::parser().getMessageInfo( topic_name );
      if(msg_info)
      {
        const RosIntrospection::ROSType header_type( ros::message_traits::DataType<std_msgs::Header>::value() ) ;
        absl::Span<uint8_t> buffer_span(raw_buffer);
        RosIntrospectionFactory::parser().applyVisitorToBuffer(topic_name, header_type,
                                      buffer_span,  modifyTimestamp );
      }
    }

    ros::serialization::IStream stream( raw_buffer.data(), raw_buffer.size() );
    shapeshifted_msg.read( stream );

    auto publisher_it = publishers_.find( topic_name );
    if( publisher_it == publishers_.end())
    {
      auto res = publishers_.insert( std::make_pair(topic_name,
                                                    shapeshifted_msg.advertise( *_node, topic_name, 10, true) ) );
      publisher_it = res.first;
    }

    const ros::Publisher& publisher = publisher_it->second;
    publisher.publish( shapeshifted_msg );
  }

}
