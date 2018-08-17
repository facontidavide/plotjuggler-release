#include "statepublisher_rostopic.h"
#include "PlotJuggler/any.hpp"
#include "../shape_shifter_factory.hpp"
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
#include <rosbag/bag.h>
#include <std_msgs/Header.h>


TopicPublisherROS::TopicPublisherROS():
    enabled_(false ),
    _node(nullptr),
    _filter_topics(false)
{
}

TopicPublisherROS::~TopicPublisherROS()
{
    enabled_ = false;
}

void TopicPublisherROS::setParentMenu(QMenu *menu)
{
    _menu = menu;
    _current_time = new QAction(QString("Overwrite std_msg/Header/stamp"), _menu);
    _current_time->setCheckable(true);
    _current_time->setChecked(true);
    _menu->addAction( _current_time );

    _select_topics_to_publish = new QAction(QString("Select topics to be published"), _menu);
    _menu->addAction( _select_topics_to_publish );
    connect(_select_topics_to_publish, SIGNAL(triggered(bool)), this, SLOT(ChangeFilter(bool)));
}

void TopicPublisherROS::setEnabled(bool to_enable)
{  
    if( !_node )
    {
        _node = RosManager::getNode();
    }
    enabled_ = (to_enable && _node);

    if(enabled_)
    {
        ChangeFilter();
    }
}

void TopicPublisherROS::ChangeFilter(bool)
{   
    const std::set<std::string> all_topics = RosIntrospectionFactory::get().getTopicList();

    if( all_topics.empty() ) return;

    QDialog* dialog = new QDialog();
    dialog->setWindowTitle("Select topics to be published");
    dialog->setMinimumWidth(350);
    QVBoxLayout* vertical_layout = new QVBoxLayout();
    QFormLayout* grid_layout = new QFormLayout();

    std::map<std::string, QCheckBox*> checkbox;

    QFrame* frame = new QFrame;
    QPushButton* select_button = new QPushButton("Select all");
    QPushButton* deselect_button = new QPushButton("Deselect all");

    for (const auto& topic: all_topics)
    {
        auto cb = new QCheckBox(dialog);
        if( _filter_topics == false )
        {
            cb->setChecked( true );
        }
        else{
            cb->setChecked( _topics_to_publish.count(topic) != 0 );
        }
        grid_layout->addRow( new QLabel( QString::fromStdString(topic)), cb);
        checkbox.insert( std::make_pair(topic, cb));
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
            if( it.second->isChecked() )
            {
                _topics_to_publish.insert(it.first);
            }
        }
        _filter_topics = true;

        //remove already created publisher if not needed anymore
        for (auto it= _publishers.begin(); it != _publishers.end(); /* no increment */)
        {
            const std::string& topic_name = it->first;
            if( _topics_to_publish.count(topic_name) == 0)
            {
                it = _publishers.erase(it);
            }
            else{
                it++;
            }
        }
    }
}


void TopicPublisherROS::updateState(double current_time)
{
    if(!enabled_ || !_node) return;

    const ros::Time ros_time = ros::Time::now();

    for(const auto& data_it:  _datamap->user_defined )
    {
        const std::string& topic_name = data_it.first;

        if( _filter_topics && _topics_to_publish.count(topic_name) == 0)
        {
            continue;// Not selected
        }
        const RosIntrospection::ShapeShifter* registered_shapeshifted_msg = RosIntrospectionFactory::get().getShapeShifter( topic_name );
        if( ! registered_shapeshifted_msg )
        {
            continue;// Not registered, just skip
        }

        RosIntrospection::ShapeShifter shapeshifted_msg = *registered_shapeshifted_msg;
        const PlotDataAny& plot_any = data_it.second;

        nonstd::optional<nonstd::any> any_value = plot_any.getYfromX( current_time );

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

        auto publisher_it = _publishers.find( topic_name );
        if( publisher_it == _publishers.end())
        {
            auto res = _publishers.insert( std::make_pair(topic_name,
                                                          shapeshifted_msg.advertise( *_node, topic_name, 10, true) ) );
            publisher_it = res.first;
        }

        const ros::Publisher& publisher = publisher_it->second;
        publisher.publish( shapeshifted_msg );
    }

}
