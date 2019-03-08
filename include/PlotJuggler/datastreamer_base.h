#ifndef DATA_STREAMER_TEMPLATE_H
#define DATA_STREAMER_TEMPLATE_H

#include <QtPlugin>
#include <QMenu>
#include <QDomDocument>
#include <mutex>
#include <unordered_set>
#include "PlotJuggler/plotdata.h"

/**
 * @brief The DataStreamer base class to create your own plugin.
 *
 * Important. To avoid problems with thread safety, it is important that ANY update to
 * dataMap(), which share its elements with the main application, is protected by the mutex()
 *
 * This includes in particular the periodic updates.
 */
class DataStreamer: public QObject{

    Q_OBJECT
public:
    DataStreamer(): _menu(NULL){}

    virtual bool start() = 0;

    virtual void shutdown() = 0;

    virtual bool isRunning() const = 0;

    virtual ~DataStreamer() {}

    virtual const char* name() const = 0;

    virtual bool isDebugPlugin() { return false; }

    virtual void setParentMenu(QMenu* menu) { _menu = menu; }

    virtual QWidget* embeddedWidget() { return nullptr; }

    virtual QDomElement xmlSaveState(QDomDocument &doc) const { return QDomElement(); }

    virtual bool xmlLoadState(QDomElement &parent_element ) { return false; }

    std::mutex& mutex(){
        return _mutex;
    }

    PlotDataMapRef& dataMap()
    {
        return _data_map;
    }

    const PlotDataMapRef& dataMap() const
    {
        return _data_map;
    }

signals:

    void clearBuffers();

    void dataUpdated();

    void connectionClosed();

protected:
    QMenu* _menu;
private:
    std::mutex _mutex;
    PlotDataMapRef _data_map;
};

QT_BEGIN_NAMESPACE

#define DataStream_iid "com.icarustechnology.PlotJuggler.DataStreamer"

Q_DECLARE_INTERFACE(DataStreamer, DataStream_iid)

QT_END_NAMESPACE


#endif

