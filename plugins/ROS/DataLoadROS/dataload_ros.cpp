#include "dataload_ros.h"
#include <QTextStream>
#include <QFile>
#include <QMessageBox>
#include <QDebug>
#include <QApplication>
#include <QProgressDialog>
#include <QElapsedTimer>
#include <QFileInfo>
#include <QProcess>
#include <rosbag/view.h>
#include <sys/sysinfo.h>
#include <QSettings>

#include "../dialog_select_ros_topics.h"
#include "../shape_shifter_factory.hpp"
#include "../rule_editing.h"


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

PlotDataMap DataLoadROS::readDataFromFile(const QString &file_name, bool use_previous_configuration)
{
    if( _bag ) _bag->close();

    _bag = std::make_shared<rosbag::Bag>();
    _parser.reset( new RosIntrospection::Parser );

    using namespace RosIntrospection;

    std::vector<std::pair<QString,QString>> all_topics;
    PlotDataMap plot_map;

    try{
        _bag->open( file_name.toStdString(), rosbag::bagmode::Read );
    }
    catch( rosbag::BagException&  ex)
    {
        QMessageBox::warning(0, tr("Error"),
                             QString("rosbag::open thrown an exception:\n")+
                             QString(ex.what()) );
        return PlotDataMap{};
    }

    rosbag::View bag_view ( *_bag, ros::TIME_MIN, ros::TIME_MAX, true );
    std::vector<const rosbag::ConnectionInfo*> connections = bag_view.getConnections();

    for(unsigned i=0; i<connections.size(); i++)
    {
        const auto&  topic      =  connections[i]->topic;
        const auto&  md5sum     =  connections[i]->md5sum;
        const auto&  datatype   =  connections[i]->datatype;
        const auto&  definition =  connections[i]->msg_def;

        all_topics.push_back( std::make_pair(QString( topic.c_str()), QString( datatype.c_str()) ) );
        _parser->registerMessageDefinition(topic, ROSType(datatype), definition);
        RosIntrospectionFactory::registerMessage(topic, md5sum, datatype, definition);
    }

    int count = 0;

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

            // load the rules
            if( dialog->checkBoxUseRenamingRules()->isChecked())
            {
                _rules = RuleEditing::getRenamingRules();
                _use_renaming_rules = true;
            }
            else{
                _rules.clear();
                _use_renaming_rules = false;
            }
        }
        settings.setValue("DataLoadROS/default_topics", _default_topic_names);
        settings.setValue("DataLoadROS/use_renaming", _use_renaming_rules);
    }

    if( _use_renaming_rules )
    {
      for(const auto& it: _rules) {
        _parser->registerRenamingRules( ROSType(it.first) , it.second );
      }
    }
    const int max_array_size = dialog->maxArraySize();
    const std::string prefix = dialog->prefix().toStdString();

    //-----------------------------------
    std::set<std::string> topic_selected;
    for(const auto& topic: _default_topic_names)
    {
        topic_selected.insert( topic.toStdString() );
    }

    const bool use_header_stamp = dialog->checkBoxUseHeaderStamp()->isChecked();
    bool warning_use_header_stamp_ignored = false;

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

    QElapsedTimer timer;
    timer.start();

    FlatMessage flat_container;
    std::vector<uint8_t> buffer;
    RenamedValues renamed_value;
    
    bool parsed = true;
    bool monotonic_time = true;

    for(rosbag::MessageInstance msg_instance: bag_view_selected )
    {
        const std::string& topic_name  = msg_instance.getTopic();
        const size_t msg_size  = msg_instance.size();

        buffer.resize(msg_size);

        if( count++ %100 == 0)
        {
            progress_dialog.setValue( count );
            QApplication::processEvents();

            if( progress_dialog.wasCanceled() ) {
                return PlotDataMap();
            }
        }

        ros::serialization::OStream stream(buffer.data(), buffer.size());
        msg_instance.write(stream);

        parsed &= _parser->deserializeIntoFlatContainer( topic_name, absl::Span<uint8_t>(buffer), &flat_container, max_array_size );
        _parser->applyNameTransform( topic_name, flat_container, &renamed_value );

        double msg_time = msg_instance.getTime().toSec();

        if(use_header_stamp)
        {
            const auto header_stamp = FlatContainedContainHeaderStamp(renamed_value);
            if(header_stamp)
            {
                const double time = header_stamp.value();
                if( time > 0 ) {
                  msg_time = time;
                }
                else{
                  warning_use_header_stamp_ignored = true;
                }
            }
        }

        for(const auto& it: renamed_value )
        {
            const std::string key = prefix + it.first;

            auto plot_pair = plot_map.numeric.find( key );
            if( (plot_pair == plot_map.numeric.end()) )
            {
                PlotDataPtr temp(new PlotData( key.c_str() ));
                auto res = plot_map.numeric.insert( std::make_pair(key, temp ) );
                plot_pair = res.first;
            }

            PlotDataPtr& plot_data = plot_pair->second;
            size_t data_size = plot_data->size();
            if( monotonic_time  && data_size>0 )
            {
              const double last_time = plot_data->at(data_size-1).x;
              monotonic_time = (msg_time > last_time);
              if(!monotonic_time)
              {
                qDebug() << "Not monotonic time in [" << key.c_str()
                         << "]: " << msg_time << " / " << last_time;
              }
            }
            plot_data->pushBack( PlotData::Point(msg_time, it.second.convert<double>() ));
        } //end of for renamed_value
    }

    for(rosbag::MessageInstance msg_instance: bag_view )
    {
        const std::string& topic_name  = msg_instance.getTopic();
        const std::string key = prefix + topic_name;
        double msg_time = msg_instance.getTime().toSec();

        if(use_header_stamp)
        {
            const auto header_stamp = FlatContainedContainHeaderStamp(renamed_value);
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
            PlotDataAnyPtr temp(new PlotDataAny(key.c_str()));
            auto res = plot_map.user_defined.insert( std::make_pair( key, temp ) );
            plot_pair = res.first;
        }
        PlotDataAnyPtr& plot_raw = plot_pair->second;
        plot_raw->pushBack( PlotDataAny::Point(msg_time, nonstd::any(std::move(msg_instance)) ));
    }

    if( !parsed )
    {
      QMessageBox::warning(0, tr("Warning"),
                           tr("Some fields were not parsed, because they contain\n"
                              "one or more vectors having more than %1 elements.").arg(max_array_size) );
    }

    if( !monotonic_time )
    {
      QString message = "The time of one or more fields is not strictly monotonic.\n"
                         "Some plots will not be displayed correctly\n";

      if( use_header_stamp)
      {
        message += "\nNOTE: you should probably DISABLE this checkbox:\n\n"
                   "[If present, use the timestamp in the field header.stamp]";
      }

      QMessageBox::warning(0, tr("Warning"), message );
    }

    if( warning_use_header_stamp_ignored )
    {
      QString message = "You checked the option:\n\n"
          "[If present, use the timestamp in the field header.stamp]\n\n"
          "But the [header.stamp] of one or more messages was NOT initialized correctly.\n";
      QMessageBox::warning(0, tr("Warning"), message );
    }

    qDebug() << "The loading operation took" << timer.elapsed() << "milliseconds";
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


