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

#include "rostopicselector.h"
#include "../ruleloaderwidget.h"
#include "../qnodedialog.h"

DataStreamROS::DataStreamROS()
{
  _enabled = false;
  _running = false;
}

PlotDataMap& DataStreamROS::getDataMap()
{
  return _plot_data;
}

void DataStreamROS::topicCallback(const topic_tools::ShapeShifter::ConstPtr& msg, const std::string &topic_name)
{
  if( !_running ||  !_enabled)
  {
    return;
  }

  using namespace RosIntrospection;

  static std::set<std::string> registered_type;

  auto& datatype = msg->getDataType();

  if( registered_type.find( datatype ) == registered_type.end() )
  {
    registered_type.insert( datatype );
    _ros_type_map = buildROSTypeMapFromDefinition(
          datatype,
          msg->getMessageDefinition() );
  }

  //------------------------------------
  uint8_t buffer[1024*64]; // "64 KB ought to be enough for anybody"
  double msg_time = (ros::Time::now() - _initial_time).toSec();

  ROSTypeFlat flat_container;

  ros::serialization::OStream stream(buffer, sizeof(buffer));
  msg->write(stream);

  SString topicname( topic_name.data(), topic_name.length() );

  buildRosFlatType( _ros_type_map, datatype, topicname, buffer, &flat_container);
  applyNameTransform( _rules[topic_name], &flat_container );

  // qDebug() << " pushing " << msg_time;

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
  _initial_time = ros::Time::now();

  int wait_time = 4;

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
  _node = getGlobalRosNode();

  using namespace RosIntrospection;

  RosTopicSelector dialog( 0 );
  int res = dialog.exec();

  QStringList topic_selected = dialog.getSelectedTopicsList();

  if( res != QDialog::Accepted || topic_selected.empty() )
  {
    return false;
  }

  // load the rules
  _rules = dialog.getLoadedRules();

  //-------------------------

  ros::start(); // needed because node will go out of scope

  _subscribers.clear();
  for (int i=0; i<topic_selected.size(); i++ )
  {
    auto topic_name = topic_selected.at(i).toStdString();
    boost::function<void(const topic_tools::ShapeShifter::ConstPtr&) > callback;
    callback = boost::bind( &DataStreamROS::topicCallback, this, _1, topic_name ) ;
    _subscribers.push_back( _node->subscribe( topic_name, 1000,  callback)  );
  }

  _running = true;

  extractInitialSamples();

  _thread = std::thread([this](){ this->update();} );

  return true;
}

void DataStreamROS::enableStreaming(bool enable) { _enabled = enable; }

bool DataStreamROS::isStreamingEnabled() const { return _enabled; }


void DataStreamROS::shutdown()
{
    _subscribers.clear();
    //_plot_data.numeric.clear();

    if(ros::isStarted() && _running)
    {
      _running = false;
      ros::shutdown(); // explicitly needed since we use ros::start();;
      ros::waitForShutdown();
      _thread.join();
    }
}

DataStreamROS::~DataStreamROS()
{
    shutdown();
}


void DataStreamROS::update()
{
  while (ros::ok() && _running)
  {
    ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.1));
  }
  ros::shutdown();
}
