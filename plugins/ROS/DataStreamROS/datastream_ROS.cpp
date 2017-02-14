#include "datastream_ROS.h"
#include <QTextStream>
#include <QFile>
#include <QMessageBox>
#include <QDebug>
#include <thread>
#include <mutex>
#include <chrono>
#include <thread>
#include <QProgressDialog>
#include <QtGlobal>
#include <ros/callback_queue.h>
#include <QApplication>

#include "../dialog_select_ros_topics.h"
#include "../rule_editing.h"
#include "../qnodedialog.h"

DataStreamROS::DataStreamROS()
{
    _enabled = false;
    _running = false;
    _initial_time = std::numeric_limits<double>::max();

    _use_header_timestamp = true;
    _normalize_time = true;
}

PlotDataMap& DataStreamROS::getDataMap()
{
    return _plot_data;
}

void DataStreamROS::topicCallback(const topic_tools::ShapeShifter::ConstPtr& msg, const std::string &topic_name)
{
    std::lock_guard<std::mutex> lock(_mutex);

    if( !_running ||  !_enabled){
        return;
    }

    //    static ros::Time prev_time = ros::Time::now();
    //    ros::Duration elapsed_time = ros::Time::now() - prev_time;
    //    _received_msg_count++;
    //    if( elapsed_time > ros::Duration(1))
    //    {
    //        prev_time += elapsed_time;
    //      //  qDebug() << "count: " << ((double)_received_msg_count)/ elapsed_time.toSec();
    //        _received_msg_count = 0;
    //    }
    using namespace RosIntrospection;

    auto& datatype = msg->getDataType();

    // Decode this message time if it is the first time you receive it
    auto it = _ros_type_map.find(datatype);
    if( it == _ros_type_map.end() )
    {
        auto typemap = buildROSTypeMapFromDefinition(
                    datatype,
                    msg->getMessageDefinition() );
        auto ret = _ros_type_map.insert( std::make_pair(datatype, std::move(typemap)));
        it = ret.first;
    }
    const RosIntrospection::ROSTypeList& type_map = it->second;

    //------------------------------------
    uint8_t buffer[1024*64]; // "64 KB ought to be enough for anybody"

    // it is more efficient to recycle ROSTypeFlat
    static ROSTypeFlat flat_container;

    ros::serialization::OStream stream(buffer, sizeof(buffer));
    msg->write(stream);

    SString topicname( topic_name.data(), topic_name.length() );

    buildRosFlatType( type_map, datatype, topicname, buffer, &flat_container);
    applyNameTransform( _rules[datatype], &flat_container );

    SString header_stamp_field( topic_name );
    header_stamp_field.append(".header.stamp");

    double msg_time = 0;

    // detrmine the time offset
    if(_use_header_timestamp == false)
    {
        msg_time = ros::Time::now().toSec();
    }
    else{
        auto offset = FlatContainedContainHeaderStamp(flat_container);
        if(offset){
            msg_time = offset.value();
        }
        else{
            msg_time = ros::Time::now().toSec();
        }
    }
    if( _normalize_time )
    {
        _initial_time = std::min( _initial_time, msg_time );
        msg_time -= _initial_time;
    }


    for(auto& it: flat_container.renamed_value )
    {
        std::string field_name ( it.first.data(), it.first.size());
        auto value = it.second;

        auto plot = _plot_data.numeric.find( field_name );
        if( plot == _plot_data.numeric.end() )
        {
            PlotDataPtr temp(new PlotData());
            temp->setMaximumRangeX( 4.0 );
            auto res = _plot_data.numeric.insert( std::make_pair(field_name, temp ) );
            plot = res.first;
        }

        // IMPORTANT: don't use pushBack(), it may cause a segfault
        plot->second->pushBackAsynchronously( PlotData::Point(msg_time, value));
    }
}

void DataStreamROS::extractInitialSamples()
{
    int wait_time = 2;

    QProgressDialog progress_dialog;
    progress_dialog.setLabelText( "Collecting ROS topic samples to understand data layout. ");
    progress_dialog.setRange(0, wait_time*1000);
    progress_dialog.setAutoClose(true);
    progress_dialog.setAutoReset(true);

    progress_dialog.show();

    using namespace std::chrono;
    auto start_time = system_clock::now();

    _enabled = true;
    while ( system_clock::now() - start_time < seconds(wait_time) )
    {
        ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.1));
        int i = duration_cast<milliseconds>(system_clock::now() - start_time).count() ;
        progress_dialog.setValue( i );
        QApplication::processEvents();
        if( progress_dialog.wasCanceled() )
        {
            break;
        }
    }
    _enabled = false;

    if( progress_dialog.wasCanceled() == false )
    {
        progress_dialog.cancel();
    }
}


bool DataStreamROS::start()
{
    _plot_data.numeric.clear();
    _plot_data.user_defined.clear();
    _initial_time = std::numeric_limits<double>::max();

    _node = getGlobalRosNode();
    if( !_node )
    {
        return false;
    }

    using namespace RosIntrospection;

    std::vector<std::pair<QString,QString>> all_topics;
    ros::master::V_TopicInfo topic_infos;
    ros::master::getTopics(topic_infos);
    for (ros::master::TopicInfo topic_info: topic_infos)
    {
        all_topics.push_back(
                    std::make_pair(QString(topic_info.name.c_str()),
                                   QString(topic_info.datatype.c_str()) ) );
    }

    DialogSelectRosTopics dialog(all_topics, QStringList(), 0 );
    int res = dialog.exec();

    QStringList topic_selected = dialog.getSelectedItems();

    if( res != QDialog::Accepted || topic_selected.empty() )
    {
        return false;
    }

    // load the rules
    if( dialog.checkBoxUseRenamingRules()->isChecked() ){
        _rules = RuleEditing::getRenamingRules();
    }
    _normalize_time       = dialog.checkBoxNormalizeTime()->isChecked();
    _use_header_timestamp = dialog.checkBoxUseHeaderStamp()->isChecked();
    //-------------------------

    ros::start(); // needed because node will go out of scope

    _subscribers.clear();
    for (int i=0; i<topic_selected.size(); i++ )
    {
        auto topic_name = topic_selected.at(i).toStdString();
        boost::function<void(const topic_tools::ShapeShifter::ConstPtr&) > callback;
        callback = boost::bind( &DataStreamROS::topicCallback, this, _1, topic_name ) ;
        _subscribers.push_back( _node->subscribe( topic_name, 0,  callback)  );
    }

    _running = true;

    extractInitialSamples();

    _thread = std::thread([this](){ this->updateLoop();} );

    return true;
}

void DataStreamROS::enableStreaming(bool enable) { _enabled = enable; }

bool DataStreamROS::isStreamingEnabled() const { return _enabled; }


void DataStreamROS::shutdown()
{
    if( _running ){
        _running = false;
        _thread.join();
        _node.reset();
    }

    for(ros::Subscriber& sub: _subscribers)
    {
        sub.shutdown();
    }
    if(ros::isStarted() )
    {
        ros::shutdown(); // explicitly needed since we use ros::start();;
        ros::waitForShutdown();
    }

    _subscribers.clear();
}

DataStreamROS::~DataStreamROS()
{
    shutdown();
}


void DataStreamROS::updateLoop()
{
    while (ros::ok() && _running)
    {
        //  ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.2));
        // _node->spin();
        ros::spinOnce();
    }
    ros::shutdown();
}
