#include "statepublisher_rostopic.h"
#include "PlotJuggler/any.hpp"
#include "../qnodedialog.h"
#include "ros_type_introspection/ros_introspection.hpp"
#include <QDialog>
#include <QFormLayout>
#include <QCheckBox>
#include <QLabel>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QDialogButtonBox>
#include <QScrollArea>
#include <QPushButton>
#include <QSettings>
#include <QRadioButton>
#include <rosbag/bag.h>
#include <std_msgs/Header.h>
#include <unordered_map>
#include <rosgraph_msgs/Clock.h>

TopicPublisherROS::TopicPublisherROS():
    enabled_(false ),
    _node(nullptr),
    _publish_clock(true)
{
    QSettings settings;
    _publish_clock = settings.value( "TopicPublisherROS/publish_clock", true ).toBool();

}

TopicPublisherROS::~TopicPublisherROS()
{
    enabled_ = false;
}

void TopicPublisherROS::setParentMenu(QMenu *menu)
{
    _menu = menu;

    _select_topics_to_publish = new QAction(QString("Select topics to be published"), _menu);
    _menu->addAction( _select_topics_to_publish );
    connect(_select_topics_to_publish, &QAction::triggered,
            this, &TopicPublisherROS::filterDialog);
}

void TopicPublisherROS::setEnabled(bool to_enable)
{  
    if( !_node && to_enable)
    {
        _node = RosManager::getNode();
    }
    enabled_ = (to_enable && _node);

    if(enabled_)
    {
        filterDialog();
        if( !_tf_publisher)
        {
            _tf_publisher = std::unique_ptr<tf::TransformBroadcaster>( new tf::TransformBroadcaster );
        }
        _previous_published_msg.clear();
        _previous_play_index = std::numeric_limits<int>::max();
    }
}

void TopicPublisherROS::filterDialog(bool)
{   
    auto all_topics = RosIntrospectionFactory::get().getTopicList();

    if( all_topics.empty() ) return;

    QDialog* dialog = new QDialog();
    dialog->setWindowTitle("Select topics to be published");
    dialog->setMinimumWidth(350);
    QVBoxLayout* vertical_layout = new QVBoxLayout();
    QFormLayout* grid_layout = new QFormLayout();

    std::map<std::string, QCheckBox*> checkbox;

    QFrame* frame = new QFrame;

    auto publish_sim_time  = new QRadioButton("Keep original timestamp and publish [/clock]");
    auto publish_real_time = new QRadioButton("Overwrite timestamp [std_msgs/Header/stamp]");
    QPushButton* select_button = new QPushButton("Select all");
    QPushButton* deselect_button = new QPushButton("Deselect all");

    publish_sim_time->setChecked( _publish_clock );
    publish_sim_time->setFocusPolicy(Qt::NoFocus);
    publish_sim_time->setToolTip("Publish the topic [/clock].\n"
                                 "You might want to set rosparam use_sim_time = true" );

    publish_real_time->setChecked( !_publish_clock );
    publish_real_time->setFocusPolicy(Qt::NoFocus);
    publish_real_time->setToolTip("Pretend it is a new message.\n"
                                 "The timestamp of the original message will be overwritten"
                                 "with ros::Time::Now()");

    select_button->setFocusPolicy(Qt::NoFocus);
    deselect_button->setFocusPolicy(Qt::NoFocus);

    for (const auto& topic: all_topics)
    {
        auto cb = new QCheckBox(dialog);
        auto filter_it = _topics_to_publish.find( *topic );
        if( filter_it == _topics_to_publish.end() )
        {
            cb->setChecked( true );
        }
        else{
            cb->setChecked( filter_it->second );
        }
        cb->setFocusPolicy(Qt::NoFocus);
        grid_layout->addRow( new QLabel( QString::fromStdString(*topic)), cb);
        checkbox.insert( std::make_pair(*topic, cb) );
        connect( select_button,   &QPushButton::pressed, [cb](){ cb->setChecked(true);} );
        connect( deselect_button, &QPushButton::pressed, [cb](){ cb->setChecked(false);} );
    }

    frame->setLayout(grid_layout);

    QScrollArea* scrollArea = new QScrollArea;
    scrollArea->setWidget(frame);

    QDialogButtonBox* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);

    QHBoxLayout* select_buttons_layout = new QHBoxLayout;
    select_buttons_layout->addWidget( select_button );
    select_buttons_layout->addWidget( deselect_button );

    vertical_layout->addWidget( publish_sim_time );
    vertical_layout->addWidget( publish_real_time );
    vertical_layout->addWidget(scrollArea);
    vertical_layout->addLayout(select_buttons_layout);
    vertical_layout->addWidget( buttons );

    connect(buttons, SIGNAL(accepted()), dialog, SLOT(accept()));
    connect(buttons, SIGNAL(rejected()), dialog, SLOT(reject()));

    dialog->setLayout(vertical_layout);
    auto result = dialog->exec();

    if(result == QDialog::Accepted)
    {
        _topics_to_publish.clear();
        for(const auto& it: checkbox )
        {
            _topics_to_publish.insert( {it.first, it.second->isChecked() } );
        }

        //remove already created publisher if not needed anymore
        for (auto it = _publishers.begin(); it != _publishers.end(); /* no increment */)
        {
            const std::string& topic_name = it->first;
            if( !toPublish(topic_name) )
            {
                it = _publishers.erase(it);
            }
            else{
                it++;
            }
        }

        _publish_clock = publish_sim_time->isChecked();

        if( _publish_clock )
        {
            _clock_publisher = _node->advertise<rosgraph_msgs::Clock>( "/clock", 10, true);
        }
        else{
            _clock_publisher.shutdown();
        }

        QSettings settings;
        settings.setValue( "TopicPublisherROS/publish_clock", _publish_clock );
    }
}

void TopicPublisherROS::broadcastTF(double current_time)
{
    std::unordered_map<std::string, geometry_msgs::TransformStamped> transforms;

    for(const auto& data_it:  _datamap->user_defined )
    {
        const std::string& topic_name = data_it.first;
        const PlotDataAny& plot_any = data_it.second;

        if( !toPublish(topic_name) )
        {
            continue;// Not selected
        }
        const RosIntrospection::ShapeShifter* shapeshifter =
                RosIntrospectionFactory::get().getShapeShifter( topic_name );
        if( shapeshifter->getDataType() != "tf/tfMessage" &&
            shapeshifter->getDataType() != "tf2_msgs/TFMessage"   )
        {
            continue;
        }

         const PlotDataAny* tf_data = &plot_any;
         int last_index = tf_data->getIndexFromX( current_time );
         if( last_index < 0)
         {
             continue;
         }

         std::vector<uint8_t> raw_buffer;
         // 1 second in the past (to be configurable in the future
         int initial_index = tf_data->getIndexFromX( current_time - 2.0 );

         if( _previous_play_index < last_index &&
             _previous_play_index > initial_index )
         {
             initial_index = _previous_play_index;
         }

         for(size_t index = std::max(0, initial_index); index <= last_index; index++ )
         {
             const nonstd::any& any_value = tf_data->at(index).y;

             const bool isRosbagMessage = any_value.type() == typeid(rosbag::MessageInstance);

             if( isRosbagMessage )
             {
                 const auto& msg_instance = nonstd::any_cast<rosbag::MessageInstance>( any_value );

                 raw_buffer.resize( msg_instance.size() );
                 ros::serialization::OStream ostream(raw_buffer.data(), raw_buffer.size());
                 msg_instance.write(ostream);

                 tf::tfMessage tf_msg;
                 ros::serialization::IStream istream( raw_buffer.data(), raw_buffer.size() );
                 ros::serialization::deserialize(istream, tf_msg);

                 for(const auto& stamped_transform: tf_msg.transforms)
                 {
                     const auto& child_id = stamped_transform.child_frame_id;
                     auto it = transforms.find(child_id);
                     if( it == transforms.end())
                     {
                         transforms.insert( {stamped_transform.child_frame_id, stamped_transform} );
                     }
                     else if( it->second.header.stamp <= stamped_transform.header.stamp)
                     {
                         it->second = stamped_transform;
                     }
                 }
             }
         }
    }

    std::vector<geometry_msgs::TransformStamped> transforms_vector;
    transforms_vector.reserve(transforms.size());

    const auto now = ros::Time::now();
    for(auto& trans: transforms)
    {
        if( !_publish_clock )
        {
            trans.second.header.stamp = now;
        }
        transforms_vector.emplace_back( std::move(trans.second) );
    }

    _tf_publisher->sendTransform(transforms_vector);
}

bool TopicPublisherROS::toPublish(const std::string &topic_name)
{
    auto it = _topics_to_publish.find( topic_name );
    if( it == _topics_to_publish.end() ){

        return false;
    }
    else {
        return it->second;
    }
}


void TopicPublisherROS::publishAnyMsg(const rosbag::MessageInstance& msg_instance)
{
    const auto& topic_name = msg_instance.getTopic();
    RosIntrospection::ShapeShifter* shapeshifted =
            RosIntrospectionFactory::get().getShapeShifter( topic_name );

    if( ! shapeshifted )
    {
        return;// Not registered, just skip
    }

    std::vector<uint8_t> raw_buffer;
    raw_buffer.resize( msg_instance.size() );
    ros::serialization::OStream ostream(raw_buffer.data(), raw_buffer.size());
    msg_instance.write(ostream);

    if( !_publish_clock )
    {
        const RosIntrospection::Parser::VisitingCallback modifyTimestamp =
                [](const RosIntrospection::ROSType&, absl::Span<uint8_t>& buffer)
        {
            std_msgs::Header msg;
            ros::serialization::IStream is( buffer.data(), buffer.size() );
            ros::serialization::deserialize(is, msg);
            msg.stamp = ros::Time::now();
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

    ros::serialization::IStream istream( raw_buffer.data(), raw_buffer.size() );
    shapeshifted->read( istream );

    auto publisher_it = _publishers.find( topic_name );
    if( publisher_it == _publishers.end())
    {
        auto res = _publishers.insert( {topic_name, shapeshifted->advertise( *_node, topic_name, 10, true)} );
        publisher_it = res.first;
    }


    const ros::Publisher& publisher = publisher_it->second;
    publisher.publish( *shapeshifted );
}



void TopicPublisherROS::updateState(double current_time)
{
    if(!enabled_ || !_node) return;

    //-----------------------------------------------
    broadcastTF(current_time);
    //-----------------------------------------------

    auto data_it = _datamap->user_defined.find( "__consecutive_message_instances__" );
    if( data_it != _datamap->user_defined.end() )
    {
        const PlotDataAny& continuous_msgs = data_it->second;
        _previous_play_index = continuous_msgs.getIndexFromX(current_time);
        //qDebug() << QString("u: %1").arg( current_index ).arg(current_time, 0, 'f', 4 );
    }


    for(const auto& data_it:  _datamap->user_defined )
    {
        const std::string& topic_name = data_it.first;
        const PlotDataAny& plot_any = data_it.second;
        if( !toPublish(topic_name) )
        {
            continue;// Not selected
        }

        const RosIntrospection::ShapeShifter* shapeshifter =
                RosIntrospectionFactory::get().getShapeShifter( topic_name );

        if( shapeshifter->getDataType() == "tf/tfMessage" ||
            shapeshifter->getDataType() == "tf2_msgs/TFMessage"   )
        {
            continue;
        }

        int last_index = plot_any.getIndexFromX( current_time );
        if( last_index < 0)
        {
            continue;
        }

        const auto& any_value = plot_any.at(last_index).y;

        if( any_value.type() == typeid(rosbag::MessageInstance) )
        {
            const auto& msg_instance = nonstd::any_cast<rosbag::MessageInstance>( any_value );
            publishAnyMsg( msg_instance );
        }
    }

    if( _publish_clock )
    {
        rosgraph_msgs::Clock clock;
        try{
            clock.clock.fromSec( current_time );
           _clock_publisher.publish( clock );
        }
        catch(...)
        {
            qDebug() << "error: " << current_time;
        }
    }
}


void TopicPublisherROS::play(double current_time)
{
    if(!enabled_ || !_node) return;

    auto data_it = _datamap->user_defined.find( "__consecutive_message_instances__" );
    if( data_it == _datamap->user_defined.end() )
    {
        return;
    }
    const PlotDataAny& continuous_msgs = data_it->second;
    int current_index = continuous_msgs.getIndexFromX(current_time);

    if( _previous_play_index > current_index)
    {
        _previous_play_index = current_index;
        updateState(current_time);
        return;
    }
    else
    {
        const PlotDataAny& consecutive_msg = data_it->second;
        for(int index = _previous_play_index+1; index <= current_index; index++)
        {

            const auto& any_value = consecutive_msg.at(index).y;
            if( any_value.type() == typeid(rosbag::MessageInstance) )
            {
                const auto& msg_instance = nonstd::any_cast<rosbag::MessageInstance>( any_value );

                if( !toPublish( msg_instance.getTopic() ) )
                {
                    continue;// Not selected
                }

                //qDebug() << QString("p: %1").arg( index );
                publishAnyMsg( msg_instance );

                if( _publish_clock )
                {
                    rosgraph_msgs::Clock clock;
                    clock.clock = msg_instance.getTime();
                   _clock_publisher.publish( clock );
                }
            }
        }
        _previous_play_index = current_index;
    }
}
