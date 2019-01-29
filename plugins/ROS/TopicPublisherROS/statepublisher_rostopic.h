#ifndef STATE_PUBLISHER_ROSTOPIC_H
#define STATE_PUBLISHER_ROSTOPIC_H

#include <QObject>
#include <QtPlugin>
#include <map>
#include <ros/ros.h>
#include <ros_type_introspection/ros_introspection.hpp>
#include <tf/transform_broadcaster.h>
#include "PlotJuggler/statepublisher_base.h"


class  TopicPublisherROS: public QObject, StatePublisher
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "com.icarustechnology.PlotJuggler.StatePublisher" "../statepublisher.json")
    Q_INTERFACES(StatePublisher)

public:
    TopicPublisherROS();
    virtual ~TopicPublisherROS() override;

    virtual void updateState(double current_time) override;

    virtual const char* name() const override { return "TopicPublisherROS"; }

    virtual bool enabled() const override { return enabled_; }

    void setParentMenu(QMenu *menu) override;

public slots:
    virtual void setEnabled(bool enabled) override;
    void ChangeFilter(bool toggled = true);

private:

    void broadcastTF(double current_time);

    std::map<std::string, ros::Publisher> _publishers;
    bool enabled_;
    ros::NodeHandlePtr _node;
    std::unique_ptr<tf::TransformBroadcaster> _tf_publisher;

    QAction* _current_time;
    QAction* _select_topics_to_publish;

    std::unordered_map<std::string,bool> _topics_to_publish;
    std::unordered_map<const PlotDataAny*, int> _previous_published_msg;

    bool toPublish(const std::string& topic_name);

};

#endif // DATALOAD_CSV_H
