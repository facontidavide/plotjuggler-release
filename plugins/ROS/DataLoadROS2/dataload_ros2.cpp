#include "dataload_ros2.h"
#include <QTextStream>
#include <QFile>
#include <QMessageBox>
#include <QPushButton>
#include <QDebug>
#include <QApplication>
#include <QProgressDialog>
#include <QFileInfo>
#include <QDir>
#include <QProcess>
#include <QSettings>
#include <QElapsedTimer>
#include <set>
#include <QDebug>
#include <rosidl_typesupport_introspection_cpp/identifier.hpp>
#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>
#include <rosidl_typesupport_introspection_cpp/field_types.hpp>
#include <rosidl_typesupport_cpp/identifier.hpp>
#include <rosbag2/typesupport_helpers.hpp>
#include <rosbag2/types/introspection_message.hpp>
#include <unordered_map>
#include <rclcpp/rclcpp.hpp>
#include <rmw/rmw.h>

#include "../dialog_select_ros_topics.h"

DataLoadROS2::DataLoadROS2()
{
    _extensions.push_back( "yaml");
    loadDefaultSettings();
}

const std::vector<const char*> &DataLoadROS2::compatibleFileExtensions() const
{
    return _extensions;
}

bool DataLoadROS2::readDataFromFile(FileLoadInfo* info, PlotDataMapRef& plot_map)
{
    if(!_bagReader){
      _bagReader = std::make_shared<rosbag2::readers::SequentialReader>();
    }

    QString bagDir;
    {
        QFileInfo finfo(info->filename);
        bagDir = finfo.dir().path();
    }

    rosbag2::StorageOptions storageOptions;
    storageOptions.uri = bagDir.toStdString();
    storageOptions.storage_id = "sqlite3";
    rosbag2::ConverterOptions converterOptions;
    converterOptions.input_serialization_format = "cdr";
    converterOptions.output_serialization_format = rmw_get_serialization_format();

    // Temporarily change the current directory as a workaround for rosbag2 relative directories not working properly
    QString oldPath = QDir::currentPath();
    QDir::setCurrent(QDir::cleanPath(bagDir + QDir::separator() + ".."));
    _bagReader->open(storageOptions, converterOptions);
    QDir::setCurrent(oldPath);

    std::vector<rosbag2::TopicMetadata> metadata = _bagReader->get_all_topics_and_types();
    std::unordered_map<std::string, std::string> topicTypesByName;

    std::vector<std::pair<QString, QString>> all_topics;
    for(const rosbag2::TopicMetadata& topic : metadata)
    {
        all_topics.push_back(std::make_pair(QString::fromStdString(topic.name), QString::fromStdString(topic.type)));
        topicTypesByName.emplace(topic.name, topic.type);
        //qDebug() << QString::fromStdString(topic.name) << " : " << QString::fromStdString(topic.type);
    }

    if( info->plugin_config.hasChildNodes() )
    {
        xmlLoadState( info->plugin_config.firstChildElement() );
    }

    if( ! info->selected_datasources.empty() )
    {
        _config.selected_topics = info->selected_datasources;
    }
    else{
        DialogSelectRosTopics* dialog = new DialogSelectRosTopics( all_topics, _config );

        if( dialog->exec() != static_cast<int>(QDialog::Accepted) )
        {
            delete dialog;
            return false;
        }
        _config = dialog->getResult();
        delete dialog;
    }

    saveDefaultSettings();

    std::set<std::string> topic_selected;
    for(const auto& topic_qt: _config.selected_topics)
    {
        const std::string topic_name = topic_qt.toStdString();
        const std::string& topic_type = topicTypesByName.at(topic_name);
        topic_selected.insert( topic_name );

        _parser.registerMessageType(topic_name, topic_type);

        if( _topic_info.count(topic_name) == 0 )
        {
            _topic_info.insert( {topic_name, TopicInfo(topic_type)} );
        }
    }

    if( _config.discard_large_arrays ){
        _parser.setMaxArrayPolicy( Ros2Introspection::Parser::DISCARD_LARGE_ARRAYS );
    }
    else{
        _parser.setMaxArrayPolicy( Ros2Introspection::Parser::KEEP_LARGE_ARRAYS );
    }

    auto time_prev = std::chrono::high_resolution_clock::now();
    while(_bagReader->has_next())
    {
        std::shared_ptr<rosbag2::SerializedBagMessage> msg = _bagReader->read_next();

        auto topic_info_it = _topic_info.find(msg->topic_name);
        if( topic_info_it == _topic_info.end() )
        {
            continue;
        }

        auto& tp = topic_info_it->second;

        _parser.deserializeIntoFlatMessage(msg->topic_name, msg->serialized_data.get(), &tp.flat_msg, _config.max_array_size);

        double timestamp = 1e-9 * double( msg->time_stamp ); // nanoseconds to seconds

        if(tp.has_header_stamp && _config.use_header_stamp)
        {
            double sec  = tp.flat_msg.values[0].second;
            double nsec = tp.flat_msg.values[1].second;
            timestamp = sec + (nsec*1e-9);
        }

        Ros2Introspection::ConvertFlatMessageToRenamedValues(tp.flat_msg, tp.renamed);

        for(const auto& it: tp.renamed)
        {
            const auto& key = it.first;
            double value = it.second;

            auto plot_pair = plot_map.numeric.find( key );
            if( plot_pair == plot_map.numeric.end() )
            {
                plot_pair = plot_map.addNumeric( key );
            }
            auto& series = plot_pair->second;
            series.pushBack( {timestamp, value} );
        }
    } // end while

    auto now = std::chrono::high_resolution_clock::now();
    double diff = std::chrono::duration_cast< std::chrono::milliseconds >( now - time_prev).count();

    qDebug() << "The rosbag loaded the data in " << diff <<" milliseconds";

    info->selected_datasources = _config.selected_topics;
    return true;
}

bool DataLoadROS2::xmlSaveState(QDomDocument &doc, QDomElement &plugin_elem) const
{
    QDomElement stamp_elem = doc.createElement("use_header_stamp");
    stamp_elem.setAttribute("value", _config.use_header_stamp ? "true" : "false");
    plugin_elem.appendChild( stamp_elem );

    QDomElement discard_elem = doc.createElement("discard_large_arrays");
    discard_elem.setAttribute("value", _config.discard_large_arrays ? "true" : "false");
    plugin_elem.appendChild( discard_elem );

    QDomElement max_elem = doc.createElement("max_array_size");
    max_elem.setAttribute("value", QString::number(_config.max_array_size));
    plugin_elem.appendChild( max_elem );

    return true;
}

bool DataLoadROS2::xmlLoadState(const QDomElement &parent_element)
{
    QDomElement stamp_elem = parent_element.firstChildElement( "use_header_stamp" );
    _config.use_header_stamp = ( stamp_elem.attribute("value") == "true");

    QDomElement discard_elem = parent_element.firstChildElement( "discard_large_arrays" );
    _config.discard_large_arrays = ( discard_elem.attribute("value") == "true");

    QDomElement max_elem = parent_element.firstChildElement( "max_array_size" );
    _config.max_array_size = static_cast<size_t>(max_elem.attribute("value").toInt());

    return true;
}

void DataLoadROS2::saveDefaultSettings()
{
    QSettings settings;
    settings.setValue("DataLoadROS2/default_topics", _config.selected_topics);
    settings.setValue("DataLoadROS2/use_header_stamp", _config.use_header_stamp);
    settings.setValue("DataLoadROS2/max_array_size", (int)_config.max_array_size);
    settings.setValue("DataLoadROS2/discard_large_arrays", _config.discard_large_arrays);
}

void DataLoadROS2::loadDefaultSettings()
{
    QSettings settings;
    _config.selected_topics      = settings.value("DataLoadROS2/default_topics", false ).toStringList();
    _config.use_header_stamp     = settings.value("DataLoadROS2/use_header_stamp", false ).toBool();
    _config.max_array_size       = settings.value("DataLoadROS2/max_array_size", 100 ).toInt();
    _config.discard_large_arrays = settings.value("DataLoadROS2/discard_large_arrays", true ).toBool();
}

