#include "statepublisher_rostopic.h"
#include <PlotJuggler/any.hpp>
#include "../shape_shifter_factory.hpp"
#include "../qnodedialog.h"
#include "timestamp_injector.h"


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

    const ros::Time ros_time =ros::Time::now();

    for(const auto& data_it:  datamap->user_defined )
    {
        const std::string&    topic_name = data_it.first;

        const RosIntrospection::ShapeShifter* registered_shapeshifted_msg = RosIntrospectionFactory::getInstance().getShapeShifter( topic_name );
    if( ! registered_shapeshifted_msg )
    {
      // Not registered, just skip
      continue;
    }

    RosIntrospection::ShapeShifter shapeshifted_msg = *registered_shapeshifted_msg;
    const PlotDataAnyPtr& plot_any = data_it.second;

    nonstd::optional<nonstd::any> any_value = plot_any->getYfromX( current_time );

    if(!any_value || any_value->type() != typeid( std::vector<uint8_t> ))
    {
      // can't cast "any" to expected type
      continue;
    }

    std::vector<uint8_t> raw_buffer =  nonstd::any_cast<std::vector<uint8_t>>( any_value.value() );

    if( _current_time->isChecked())
    {
        auto type_map = RosIntrospectionFactory::getInstance().getRosTypeList( shapeshifted_msg.getMD5Sum());
        if(type_map)
        {
            injectTime(*type_map, shapeshifted_msg.getDataType(), raw_buffer.data(), ros_time);
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
