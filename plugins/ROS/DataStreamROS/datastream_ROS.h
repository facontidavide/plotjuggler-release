#ifndef DATASTREAM_ROS_TOPIC_H
#define DATASTREAM_ROS_TOPIC_H

#include <QtPlugin>
#include <thread>
#include <topic_tools/shape_shifter.h>
#include "PlotJuggler/datastreamer_base.h"
#include <ros_type_introspection/ros_introspection.hpp>


class  DataStreamROS: public QObject, DataStreamer
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "com.icarustechnology.PlotJuggler.DataStreamer" "../datastreamer.json")
    Q_INTERFACES(DataStreamer)

public:

    DataStreamROS();

    virtual PlotDataMap &getDataMap() override;

    virtual bool start() override;

    virtual void shutdown() override;

    virtual void enableStreaming(bool enable) override;

    virtual bool isStreamingEnabled() const override;

    virtual ~DataStreamROS() override;

    virtual const char* name() const override { return "ROS Topic Streamer";  }

    virtual QObject* getObject() override { return this; }

private:

    void topicCallback(const topic_tools::ShapeShifter::ConstPtr& msg, const std::string &topic_name);

    void update();

    PlotDataMap _plot_data;

    bool _enabled;

    bool _running;

    std::thread _thread;

    RosIntrospection::ROSTypeList _ros_type_map;

    void extractInitialSamples();

    ros::Time _initial_time;

    ros::NodeHandlePtr _node;
    std::vector<ros::Subscriber> _subscribers;

    RosIntrospection::SubstitutionRuleMap _rules;
};

#endif // DATALOAD_CSV_H
