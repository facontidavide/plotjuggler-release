#include "dataload_ros.h"
#include <QTextStream>
#include <QFile>
#include <QMessageBox>
#include <QDebug>
#include <QApplication>
#include <QProgressDialog>
#include <QElapsedTimer>

#include <rosbag/view.h>

#include "dialog_select_ros_topics.h"
#include "../ruleloaderwidget.h"
#include "../shape_shifter_factory.hpp"


DataLoadROS::DataLoadROS()
{
  _extensions.push_back( "bag");
}

const std::vector<const char*> &DataLoadROS::compatibleFileExtensions() const
{
  return _extensions;
}


PlotDataMap DataLoadROS::readDataFromFile(const std::string& file_name,
                                          std::string &load_configuration  )
{
  using namespace RosIntrospection;

  QStringList all_topic_names;
  PlotDataMap plot_map;

  rosbag::Bag bag;
  bag.open( file_name, rosbag::bagmode::Read );


  rosbag::View bag_view ( bag, ros::TIME_MIN, ros::TIME_MAX, true );
  auto first_time = bag_view.getBeginTime();
  std::vector<const rosbag::ConnectionInfo *> connections = bag_view.getConnections();

  // create a list and a type map for each topic
  std::map<std::string,ROSTypeList> type_map;

  for(unsigned i=0;  i<connections.size(); i++)
  {
    all_topic_names.push_back( QString( connections[i]->topic.c_str() ) );

    const auto&  md5sum     =  connections[i]->md5sum;
    const auto&  data_type  =  connections[i]->datatype;
    const auto&  definition =  connections[i]->msg_def;

    auto topic_map = buildROSTypeMapFromDefinition( data_type, definition);
    type_map.insert( std::make_pair(data_type,topic_map));

    ShapeShifterFactory::getInstance().registerMessage(connections[i]->topic, md5sum, data_type, definition);
  }

  all_topic_names.sort();

  int count = 0;

  DialogSelectRosTopics* dialog = new DialogSelectRosTopics( all_topic_names );

  std::set<std::string> topic_selected;

  if( dialog->exec() == QDialog::Accepted)
  {
    const auto& selected_items = dialog->getSelectedItems();
    for(auto item: selected_items)
    {
      topic_selected.insert( item.toStdString() );
    }
    // load the rules
    _rules = dialog->getLoadedRules();

  }

  rosbag::View bag_view_reduced ( true );
  bag_view_reduced.addQuery(bag, [topic_selected](rosbag::ConnectionInfo const* connection)
  {
    return topic_selected.find( connection->topic ) != topic_selected.end();
  } );

  QProgressDialog progress_dialog;
  progress_dialog.setLabelText("Loading... please wait");
  progress_dialog.setWindowModality( Qt::ApplicationModal );
  progress_dialog.setRange(0, bag_view_reduced.size() -1);
  progress_dialog.show();

  QElapsedTimer timer;
  timer.start();


  ROSTypeFlat flat_container;

  for(const rosbag::MessageInstance& msg: bag_view_reduced )
  {
    /* if( topic_selected.find( msg.getTopic() ) == topic_selected.end() )
    {
      continue;
    }*/

    const auto& md5sum    = msg.getMD5Sum();
    const auto& datatype  = msg.getDataType();
    auto msg_size = msg.size();

    std::vector<uint8_t> buffer ( msg_size );

    double msg_time = (msg.getTime() - first_time).toSec();

    if( count++ %1000 == 0)
    {
      qDebug() << count << " / " << bag_view_reduced.size();

      progress_dialog.setValue( count );
      QApplication::processEvents();

      if( progress_dialog.wasCanceled() ) {
        return PlotDataMap();
      }
    }

    ros::serialization::OStream stream(buffer.data(), buffer.size());

    // this single line takes almost the entire time of the loop
    msg.write(stream);

    SString topic_name( msg.getTopic().data(),  msg.getTopic().size() );

    buildRosFlatType(type_map[ datatype ], datatype, topic_name, buffer.data(), &flat_container);
    applyNameTransform( _rules[datatype], &flat_container );

    for(auto& it: flat_container.renamed_value )
    {
      std::string field_name ( it.first.data(), it.first.size());
      auto value = it.second;

      auto plot_pair = plot_map.numeric.find( field_name );
      if( plot_pair == plot_map.numeric.end() )
      {
        PlotDataPtr temp(new PlotData());
        auto res = plot_map.numeric.insert( std::make_pair(field_name, temp ) );
        plot_pair = res.first;
      }

      PlotDataPtr& plot_data = plot_pair->second;
      plot_data->pushBack( PlotData::Point(msg_time, value));

    } //end of for flat_container.renamed_value

    //-----------------------------------------
    // adding raw serialized topic for future uses.
    {

      auto plot_pair = plot_map.user_defined.find( md5sum );

      if( plot_pair == plot_map.user_defined.end() )
      {
        PlotDataAnyPtr temp(new PlotDataAny());
        auto res = plot_map.user_defined.insert( std::make_pair( msg.getTopic(), temp ) );
        plot_pair = res.first;
      }

      PlotDataAnyPtr& plot_raw = plot_pair->second;
      plot_raw->pushBack( PlotDataAny::Point(msg_time, nonstd::any(std::move(buffer)) ));
    }
    //   qDebug() << msg.getTopic().c_str();
  }
  qDebug() << "The loading operation took" << timer.elapsed() << "milliseconds";

  return plot_map;
}



DataLoadROS::~DataLoadROS()
{

}


