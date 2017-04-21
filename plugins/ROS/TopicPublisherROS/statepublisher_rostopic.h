#ifndef STATE_PUBLISHER_ROSTOPIC_H
#define STATE_PUBLISHER_ROSTOPIC_H

#include <QObject>
#include <QtPlugin>
#include <map>
#include <ros/ros.h>
#include "PlotJuggler/statepublisher_base.h"


class  TopicPublisherROS: public QObject, StatePublisher
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "com.icarustechnology.PlotJuggler.StatePublisher" "../statepublisher.json")
    Q_INTERFACES(StatePublisher)

public:
    TopicPublisherROS();

    virtual void updateState(PlotDataMap* datamap, double current_time) override;
    virtual const char* name() const override { return "TopicPublisherROS"; }
    virtual ~TopicPublisherROS();

    virtual bool enabled() const override { return enabled_; }

public slots:
    virtual void setEnabled(bool enabled) override;

private:
    std::map<std::string, ros::Publisher> publishers_;
    bool enabled_;
    ros::NodeHandlePtr node_;
};

#endif // DATALOAD_CSV_H
