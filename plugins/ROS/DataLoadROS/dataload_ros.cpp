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

void StrCat(const std::string& a, const std::string& b,  std::string& out)
{
   out.clear();
   out.reserve(a.size() + b.size());
   out.assign(a);
   out.append(b);
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

  RosIntrospectionFactory::reset();

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
                                                     const std::string& prefix)
{
  using namespace RosIntrospection;

  rosbag::View bag_view ( *_bag, ros::TIME_MIN, ros::TIME_MAX, true );

  RenamedValues renamed_value;

  std::string prefixed_name;

  PlotDataAny& plot_consecutive = plot_map.addUserDefined( "__consecutive_message_instances__" )->second;

  for(const rosbag::MessageInstance& msg_instance: bag_view )
  {
      const std::string& topic_name  = msg_instance.getTopic();
      double msg_time = msg_instance.getTime().toSec();
      auto data_point = PlotDataAny::Point(msg_time, nonstd::any(msg_instance) );
      plot_consecutive.pushBack( data_point );

      if( prefix.empty() == false)
      {
          StrCat(prefix, topic_name, prefixed_name);
      }
      const std::string* key_ptr = prefix.empty() ? &topic_name : &prefixed_name;

      auto plot_pair = plot_map.user_defined.find( *key_ptr );

      if( plot_pair == plot_map.user_defined.end() )
      {
          plot_pair = plot_map.addUserDefined( *key_ptr );
      }
      PlotDataAny& plot_raw = plot_pair->second;
      plot_raw.pushBack( data_point );
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
        QMessageBox::warning(nullptr, tr("Error"),
                             QString("rosbag::open thrown an exception:\n")+
                             QString(ex.what()) );
        return PlotDataMapRef();
    }

    auto all_topics = getAndRegisterAllTopics();

    //----------------------------------
    QSettings settings;

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

    bool use_header_stamp = dialog->checkBoxTimestamp()->isChecked();

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
    std::string prefixed_name;
    QElapsedTimer timer;
    timer.start();

    setMaxArrayPolicy( _parser.get(), dialog->discardEntireArrayIfTooLarge() );

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
            const auto header_stamp = FlatContainerContainHeaderStamp(flat_container);
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
            const auto& field_name = it.first;

            if( prefix.empty() == false)
            {
                StrCat(prefix, field_name, prefixed_name);
            }
            const std::string* key_ptr = prefix.empty() ? &field_name : &prefixed_name;

            const RosIntrospection::Variant& value = it.second;

            auto plot_pair = plot_map.numeric.find( *key_ptr );
            if( (plot_pair == plot_map.numeric.end()) )
            {
                plot_pair = plot_map.addNumeric( *key_ptr );
            }

            PlotData& plot_data = plot_pair->second;
            size_t data_size = plot_data.size();
            if( data_size > 0 )
            {
              const double last_time = plot_data.back().x;
              if( msg_time < last_time)
              {
                 warning_monotonic.insert(*key_ptr);
              }
            }

            if( value.getTypeID() == RosIntrospection::UINT64)
            {
                uint64_t val_i = value.extract<uint64_t>();
                double val_d = static_cast<double>(val_i);
                bool error = (val_i != static_cast<uint64_t>(val_d));
                if(error)
                {
                    warning_cancellation.insert(*key_ptr);
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
                    warning_cancellation.insert(*key_ptr);
                }
                plot_data.pushBack( PlotData::Point(msg_time, val_d) );
            }
            else{
                try{
                    double val_d = value.convert<double>();
                    plot_data.pushBack( PlotData::Point(msg_time, val_d ));
                }
                catch(std::exception&)
                {
                    continue;
                }
            }
        } //end of for renamed_value
    }

    storeMessageInstancesAsUserDefined(plot_map, prefix);

    qDebug() << "The loading operation took" << timer.elapsed() << "milliseconds";

    if( !warning_max_arraysize.empty() )
    {
      QString message = QString("The following topics contain arrays with more than %1 elements.\n").arg(max_array_size);
      if( dialog->discardEntireArrayIfTooLarge() )
      {
          message += tr("The fields containing thes extra large arrays have been discarded\n");
      }
      else{
          message += tr("These arrays were trunkated to the maximum size %1\n").arg(max_array_size);
      }
      DialogWithItemList::warning( message, warning_max_arraysize );
    }

    if( !warning_monotonic.empty() )
    {
      QString message = "The time of one or more fields is not strictly monotonic.\n"
                         "Some plots will not be displayed correctly\n";

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


