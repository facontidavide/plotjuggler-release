#include "dataload_ros.h"
#include <QTextStream>
#include <QFile>
#include <QMessageBox>
#include <QDebug>
#include <QApplication>
#include <QProgressDialog>
#include <QFileInfo>
#include <QProcess>
#include <rosbag/view.h>
#include <sys/sysinfo.h>
#include <QSettings>
#include <QElapsedTimer>

#include "../dialog_select_ros_topics.h"
#include "../shape_shifter_factory.hpp"
#include "../rule_editing.h"
#include "../dialog_with_itemlist.h"

DataLoadROS::DataLoadROS()
{
    _extensions.push_back( "bag");
}

const std::vector<const char*> &DataLoadROS::compatibleFileExtensions() const
{
    return _extensions;
}

size_t getAvailableRAM()
{
    struct sysinfo info;
    sysinfo(&info);
    return info.freeram;
}

std::vector<std::pair<QString,QString>> DataLoadROS::getAndRegisterAllTopics()
{
  std::vector<std::pair<QString,QString>> all_topics;
  rosbag::View bag_view ( *_bag, ros::TIME_MIN, ros::TIME_MAX, true );

  for(auto& conn: bag_view.getConnections() )
  {
      const auto&  topic      =  conn->topic;
      const auto&  md5sum     =  conn->md5sum;
      const auto&  datatype   =  conn->datatype;
      const auto&  definition =  conn->msg_def;

      all_topics.push_back( std::make_pair(QString( topic.c_str()), QString( datatype.c_str()) ) );
      _parser->registerMessageDefinition(topic, RosIntrospection::ROSType(datatype), definition);
      RosIntrospectionFactory::registerMessage(topic, md5sum, datatype, definition);
  }
  return all_topics;
}

void DataLoadROS::storeMessageInstancesAsUserDefined(PlotDataMapRef& plot_map,
                                                     const std::string& prefix,
                                                     bool use_header_stamp)
{
  using namespace RosIntrospection;

  rosbag::View bag_view ( *_bag, ros::TIME_MIN, ros::TIME_MAX, true );

  RenamedValues renamed_value;

  for(const rosbag::MessageInstance& msg_instance: bag_view )
  {
      const std::string& topic_name  = msg_instance.getTopic();
      const std::string key = prefix + topic_name;
      double msg_time = msg_instance.getTime().toSec();

      if(use_header_stamp)
      {
          const auto header_stamp = FlatContainerContainHeaderStamp(renamed_value);
          if(header_stamp)
          {
              const double time = header_stamp.value();
              if( time > 0 ) {
                msg_time = time;
              }
          }
      }

      auto plot_pair = plot_map.user_defined.find( key );

      if( plot_pair == plot_map.user_defined.end() )
      {
          plot_pair = plot_map.addUserDefined( key );
      }
      PlotDataAny& plot_raw = plot_pair->second;
      plot_raw.pushBack( PlotDataAny::Point(msg_time, nonstd::any(std::move(msg_instance)) ));
  }
}

PlotDataMapRef DataLoadROS::readDataFromFile(const QString &file_name, bool use_previous_configuration)
{
    if( _bag ) _bag->close();

    _bag = std::make_shared<rosbag::Bag>();
    _parser.reset( new RosIntrospection::Parser );

    using namespace RosIntrospection;

    try{
        _bag->open( file_name.toStdString(), rosbag::bagmode::Read );
    }
    catch( rosbag::BagException&  ex)
    {
        QMessageBox::warning(0, tr("Error"),
                             QString("rosbag::open thrown an exception:\n")+
                             QString(ex.what()) );
        return PlotDataMapRef();
    }

    auto all_topics = getAndRegisterAllTopics();

    //----------------------------------
    QSettings settings( "IcarusTechnology", "PlotJuggler");

    _use_renaming_rules = settings.value("DataLoadROS/use_renaming").toBool();

    if( _default_topic_names.empty() )
    {
        // if _default_topic_names is empty (xmlLoad didn't work) use QSettings.
        QVariant def = settings.value("DataLoadROS/default_topics");
        if( !def.isNull() && def.isValid())
        {
            _default_topic_names = def.toStringList();
        }
    }

    DialogSelectRosTopics* dialog = new DialogSelectRosTopics( all_topics, _default_topic_names );

    if( !use_previous_configuration )
    {
        if( dialog->exec() == static_cast<int>(QDialog::Accepted) )
        {
            _default_topic_names = dialog->getSelectedItems();
            settings.setValue("DataLoadROS/default_topics", _default_topic_names);
            settings.setValue("DataLoadROS/use_renaming", _use_renaming_rules);
        }
        else{
            return PlotDataMapRef();
        }
    }

    _use_renaming_rules = dialog->checkBoxUseRenamingRules()->isChecked();

    if( _use_renaming_rules )
    {
      _rules = RuleEditing::getRenamingRules();
      for(const auto& it: _rules) {
        _parser->registerRenamingRules( ROSType(it.first) , it.second );
      }
    }
    else{
      _rules.clear();
    }
    const int max_array_size    = dialog->maxArraySize();
    const std::string prefix    = dialog->prefix().toStdString();
    const bool use_header_stamp = dialog->checkBoxUseHeaderStamp()->isChecked();

    //-----------------------------------
    std::set<std::string> topic_selected;
    for(const auto& topic: _default_topic_names)
    {
        topic_selected.insert( topic.toStdString() );
    }

    QProgressDialog progress_dialog;
    progress_dialog.setLabelText("Loading... please wait");
    progress_dialog.setWindowModality( Qt::ApplicationModal );

    rosbag::View bag_view_selected ( true );
    bag_view_selected.addQuery( *_bag, [topic_selected](rosbag::ConnectionInfo const* connection)
    {
        return topic_selected.find( connection->topic ) != topic_selected.end();
    } );
    progress_dialog.setRange(0, bag_view_selected.size()-1);
    progress_dialog.show();

    PlotDataMapRef plot_map;

    FlatMessage flat_container;
    std::vector<uint8_t> buffer;
    RenamedValues renamed_values;

    std::unordered_set<std::string> warning_headerstamp;
    std::unordered_set<std::string> warning_monotonic;
    std::unordered_set<std::string> warning_cancellation;
    std::unordered_set<std::string> warning_max_arraysize;
    int msg_count = 0;

    QElapsedTimer timer;
    timer.start();

    for(const rosbag::MessageInstance& msg_instance: bag_view_selected )
    {
        const std::string& topic_name  = msg_instance.getTopic();
        const size_t msg_size  = msg_instance.size();

        buffer.resize(msg_size);

        if( msg_count++ %100 == 0)
        {
            progress_dialog.setValue( msg_count );
            QApplication::processEvents();

            if( progress_dialog.wasCanceled() ) {
                return PlotDataMapRef();
            }
        }

        ros::serialization::OStream stream(buffer.data(), buffer.size());
        msg_instance.write(stream);

        bool max_size_ok = _parser->deserializeIntoFlatContainer( topic_name,
                                                                  absl::Span<uint8_t>(buffer),
                                                                  &flat_container,
                                                                  max_array_size );
        if( !max_size_ok )
        {
          warning_max_arraysize.insert(topic_name);
        }
        _parser->applyNameTransform( topic_name, flat_container, &renamed_values );

        double msg_time = msg_instance.getTime().toSec();

        if(use_header_stamp)
        {
            const auto header_stamp = FlatContainerContainHeaderStamp(renamed_values);
            if(header_stamp)
            {
                const double time = header_stamp.value();
                if( time > 0 ) {
                  msg_time = time;
                }
                else{
                  warning_headerstamp.insert(topic_name);
                }
            }
        }

        for(const auto& it: renamed_values )
        {
            const std::string key = prefix + it.first;
            const RosIntrospection::Variant& value = it.second;

            auto plot_pair = plot_map.numeric.find( key );
            if( (plot_pair == plot_map.numeric.end()) )
            {
                plot_pair = plot_map.addNumeric( key );
            }

            PlotData& plot_data = plot_pair->second;
            size_t data_size = plot_data.size();
            if( data_size > 0 )
            {
              const double last_time = plot_data.back().x;
              if( msg_time < last_time)
              {
                warning_monotonic.insert(key);
              }
            }

            if( value.getTypeID() == RosIntrospection::UINT64)
            {
                uint64_t val_i = value.extract<uint64_t>();
                double val_d = static_cast<double>(val_i);
                bool error = (val_i != static_cast<uint64_t>(val_d));
                if(error)
                {
                    warning_cancellation.insert(key);
                }
                plot_data.pushBack( PlotData::Point(msg_time, val_d) );
            }
            else if( value.getTypeID() == RosIntrospection::INT64)
            {
                int64_t val_i = value.extract<int64_t>();
                double val_d = static_cast<double>(val_i);
                bool error = (val_i != static_cast<int64_t>(val_d));
                if(error)
                {
                    warning_cancellation.insert(key);
                }
                plot_data.pushBack( PlotData::Point(msg_time, val_d) );
            }
            else{
                plot_data.pushBack( PlotData::Point(msg_time, value.convert<double>() ));
            }

        } //end of for renamed_value
    }

    storeMessageInstancesAsUserDefined(plot_map, prefix, use_header_stamp);

    qDebug() << "The loading operation took" << timer.elapsed() << "milliseconds";

    if( !warning_max_arraysize.empty() )
    {
      QString message = QString("The following topics contain arrays with more than %1 elements.\n"
                                "They were trunkated to the maximum array size %1\n").arg(max_array_size);
      DialogWithItemList::warning( message, warning_max_arraysize );
    }

    if( !warning_monotonic.empty() )
    {
      QString message = "The time of one or more fields is not strictly monotonic.\n"
                         "Some plots will not be displayed correctly\n";

      if( use_header_stamp)
      {
        message += "\nNOTE: you should probably DISABLE this checkbox:\n\n"
                   "[If present, use the timestamp in the field header.stamp]\n";
      }
      DialogWithItemList::warning( message, warning_monotonic );
    }

    if( !warning_headerstamp.empty() )
    {
        QString message = "You checked the option:\n\n"
                          "[If present, use the timestamp in the field header.stamp]\n\n"
                          "But the [header.stamp] of one or more messages were NOT initialized correctly.\n";
        DialogWithItemList::warning( message, warning_headerstamp );
    }

    if( !warning_cancellation.empty() )
    {
        QString message = "During the parsing process, one or more conversions to double failed"
                          " because of numerical cancellation.\n"
                          "This happens when the absolute value of a long integer exceed 2^52.\n\n"
                          "You have been warned... don't trust the following timeseries\n";
        DialogWithItemList::warning( message, warning_cancellation );
    }
    return plot_map;
}


DataLoadROS::~DataLoadROS()
{

}

QDomElement DataLoadROS::xmlSaveState(QDomDocument &doc) const
{
    QString topics_list = _default_topic_names.join(";");
    QDomElement list_elem = doc.createElement("selected_topics");
    list_elem.setAttribute("list", topics_list );
    return list_elem;
}

bool DataLoadROS::xmlLoadState(QDomElement &parent_element)
{
    QDomElement list_elem = parent_element.firstChildElement( "selected_topics" );
    if( !list_elem.isNull()    )
    {
        if( list_elem.hasAttribute("list") )
        {
            QString topics_list = list_elem.attribute("list");
            _default_topic_names = topics_list.split(";", QString::SkipEmptyParts);
            return true;
        }
    }
    return false;
}


