#ifndef DATASTREAM_ROS_TOPIC_H
#define DATASTREAM_ROS_TOPIC_H

#include <QtPlugin>
#include <QAction>
#include <QTimer>
#include <thread>
#include <topic_tools/shape_shifter.h>
#include "PlotJuggler/datastreamer_base.h"
#include <ros_type_introspection/ros_introspection.hpp>
#include <rosgraph_msgs/Clock.h>

class  DataStreamROS: public DataStreamer
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "com.icarustechnology.PlotJuggler.DataStreamer" "../datastreamer.json")
    Q_INTERFACES(DataStreamer)

public:

    DataStreamROS();

    virtual bool start() override;

    virtual void shutdown() override;

    virtual bool isRunning() const override;

    virtual ~DataStreamROS() override;

    virtual const char* name() const override { return "ROS Topic Streamer";  }

    virtual void setParentMenu(QMenu* menu) override;

    virtual QDomElement xmlSaveState(QDomDocument &doc) const override;

    virtual bool xmlLoadState(QDomElement &parent_element ) override;

private:

    void topicCallback(const topic_tools::ShapeShifter::ConstPtr& msg, const std::string &topic_name);

    void clockCallback(const rosgraph_msgs::Clock::ConstPtr& msg);

    bool _running;

    std::shared_ptr<ros::AsyncSpinner> _spinner;

    void extractInitialSamples();

    double _initial_time;
    int _max_array_size;
    std::string _prefix;

    ros::NodeHandlePtr _node;
    ros::Subscriber _clock_subscriber;
    std::map<std::string, ros::Subscriber> _subscribers;
    RosIntrospection::SubstitutionRuleMap _rules;

    int _received_msg_count;

    QAction* _action_saveIntoRosbag;
    QAction* _action_clearBuffer;

    std::map<std::string, int> _msg_index;

    QStringList _default_topic_names;

    std::unique_ptr<RosIntrospection::Parser> _parser;
    bool _using_renaming_rules;
    bool _use_header_stamp;

    QTimer* _periodic_timer;

    bool _roscore_disconnection_already_notified;

    ros::Time _clock_time;

    void timerCallback();

    void subscribe();

private slots:

    void saveIntoRosbag();
};

#endif // DATALOAD_CSV_H
