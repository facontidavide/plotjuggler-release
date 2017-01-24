#ifndef DATA_STREAMER_TEMPLATE_H
#define DATA_STREAMER_TEMPLATE_H

#include <QtPlugin>
#include <mutex>
#include "PlotJuggler/plotdata.h"


class DataStreamer{

public:

    virtual PlotDataMap& getDataMap() = 0;

    virtual bool start() = 0;

    virtual void shutdown() = 0;

    virtual void enableStreaming(bool enable) = 0;

    virtual bool isStreamingEnabled() const = 0;

    virtual ~DataStreamer() {}

    virtual const char* name() const = 0;

};

QT_BEGIN_NAMESPACE

#define DataStream_iid "com.icarustechnology.PlotJuggler.DataStreamer"

Q_DECLARE_INTERFACE(DataStreamer, DataStream_iid)

QT_END_NAMESPACE


#endif

