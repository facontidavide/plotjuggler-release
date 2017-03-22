#include "dataload_ros.h"
#include <QTextStream>
#include <QFile>
#include <QMessageBox>
#include <QDebug>
#include <QApplication>
#include <QProgressDialog>
#include <QElapsedTimer>

#include <rosbag/view.h>

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

PlotDataMap DataLoadROS::readDataFromFile(const std::string& file_name,
                                          std::string &load_configuration  )
{
    using namespace RosIntrospection;

    std::vector<std::pair<QString,QString>> all_topics;
    PlotDataMap plot_map;

    rosbag::Bag bag;
    try{
        bag.open( file_name, rosbag::bagmode::Read );
    }
    catch( rosbag::BagException&  ex)
    {
        QMessageBox::warning(0, tr("Error"),
                             QString("rosbag::open thrown an exception:\n")+
                             QString(ex.what()) );
        return plot_map;
    }

    rosbag::View bag_view ( bag, ros::TIME_MIN, ros::TIME_MAX, true );
    auto first_time = bag_view.getBeginTime();
    std::vector<const rosbag::ConnectionInfo *> connections = bag_view.getConnections();

    // create a list and a type map for each topic
    std::map<std::string,ROSTypeList> type_map;

    for(unsigned i=0;  i<connections.size(); i++)
    {
        const auto&  topic      =  connections[i]->topic;
        const auto&  md5sum     =  connections[i]->md5sum;
        const auto&  data_type  =  connections[i]->datatype;
        const auto&  definition =  connections[i]->msg_def;

        all_topics.push_back( std::make_pair(QString( topic.c_str()), QString( data_type.c_str()) ) );

        auto topic_map = buildROSTypeMapFromDefinition( data_type, definition);
        type_map.insert( std::make_pair(data_type,topic_map));

        ShapeShifterFactory::getInstance().registerMessage(connections[i]->topic, md5sum, data_type, definition);
    }

    int count = 0;

    QStringList default_topic_names = QString(load_configuration.c_str()).split(' ');

    DialogSelectRosTopics* dialog = new DialogSelectRosTopics( all_topics, default_topic_names );

    std::set<std::string> topic_selected;

    if( dialog->exec() == QDialog::Accepted)
    {
        const auto& selected_items = dialog->getSelectedItems();
        for(auto item: selected_items)
        {
            topic_selected.insert( item.toStdString() );
        }
        // load the rules
        if( dialog->checkBoxUseRenamingRules()->isChecked())
        {
            _rules = RuleEditing::getRenamingRules();
        }
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
        const auto& topic      = msg.getTopic();
        const auto& md5sum     = msg.getMD5Sum();
        const auto& datatype   = msg.getDataType();
        auto msg_size = msg.size();

        std::vector<uint8_t> buffer ( msg_size );

        if( count++ %1000 == 0)
        {
            // qDebug() << count << " / " << bag_view_reduced.size();

            progress_dialog.setValue( count );
            QApplication::processEvents();

            if( progress_dialog.wasCanceled() ) {
                return PlotDataMap();
            }
        }

        ros::serialization::OStream stream(buffer.data(), buffer.size());

        // this single line takes almost the entire time of the loop
        msg.write(stream);

        SString topic_name( topic.data(),  topic.size() );

        buildRosFlatType(type_map[ datatype ], datatype, topic_name, buffer.data(), &flat_container);
        applyNameTransform( _rules[datatype], &flat_container );

        // apply time offsets
        double msg_time;

        if(dialog->checkBoxUseHeaderStamp()->isChecked() == false)
        {
            msg_time = msg.getTime().toSec();
        }
        else{
            auto offset = FlatContainedContainHeaderStamp(flat_container);
            if(offset){
                msg_time = offset.value();
            }
            else{
                msg_time = msg.getTime().toSec();
            }
        }

        for(const auto& it: flat_container.renamed_value )
        {
            std::string field_name( it.first.data(), it.first.size());

            auto plot_pair = plot_map.numeric.find( field_name );
            if( plot_pair == plot_map.numeric.end() )
            {
                PlotDataPtr temp(new PlotData(field_name.c_str()));
                auto res = plot_map.numeric.insert( std::make_pair(field_name, temp ) );
                plot_pair = res.first;
            }

            PlotDataPtr& plot_data = plot_pair->second;
            plot_data->pushBack( PlotData::Point(msg_time, it.second));
        } //end of for flat_container.renamed_value

        //-----------------------------------------
        // adding raw serialized topic for future uses.
        {
            auto plot_pair = plot_map.user_defined.find( md5sum );

            if( plot_pair == plot_map.user_defined.end() )
            {
                PlotDataAnyPtr temp(new PlotDataAny(topic.c_str()));
                auto res = plot_map.user_defined.insert( std::make_pair( topic, temp ) );
                plot_pair = res.first;
            }
            PlotDataAnyPtr& plot_raw = plot_pair->second;
            plot_raw->pushBack( PlotDataAny::Point(msg_time, nonstd::any(std::move(buffer)) ));
        }
    }
    qDebug() << "The loading operation took" << timer.elapsed() << "milliseconds";
    return plot_map;
}


DataLoadROS::~DataLoadROS()
{

}


