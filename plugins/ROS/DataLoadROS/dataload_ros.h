#ifndef DATALOAD_ROS_H
#define DATALOAD_ROS_H

#include <ros/ros.h>
#include <rosbag/bag.h>

#include <QObject>
#include <QtPlugin>
#include "PlotJuggler/dataloader_base.h"
#include <ros_type_introspection/ros_introspection.hpp>

class  DataLoadROS: public QObject, DataLoader
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "com.icarustechnology.Superplotter.DataLoader" "../dataloader.json")
    Q_INTERFACES(DataLoader)

public:
    DataLoadROS();
    virtual const std::vector<const char*>& compatibleFileExtensions() const override;

    virtual PlotDataMap readDataFromFile(const std::string& file_name,
                                          std::string &load_configuration  ) override;

    virtual const char* name() const override { return "DataLoad ROS bags"; }

    virtual ~DataLoadROS();

protected:
    void loadSubstitutionRule(QStringList all_topic_names);

private:
    RosIntrospection::SubstitutionRuleMap  _rules;

    std::vector<const char*> _extensions;


};

#endif // DATALOAD_CSV_H
