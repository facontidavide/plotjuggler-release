#ifndef STATEPUBLISHER_TEMPLATE_H
#define STATEPUBLISHER_TEMPLATE_H

#include <QObject>
#include <QtPlugin>
#include <QMenu>
#include <QDomElement>
#include <functional>
#include "PlotJuggler/plotdata.h"


class StatePublisher{

public:

    virtual bool enabled() const = 0;

    virtual const char* name() const = 0;

    virtual void updateState(double current_time) = 0;

    virtual ~StatePublisher() {}

    virtual void setEnabled(bool enabled) = 0;

    virtual bool isDebugPlugin() { return false; }

    virtual void setParentMenu(QMenu* menu) { _menu = menu; }

    virtual QWidget* embeddedWidget() { return nullptr; }

    virtual QDomElement xmlSaveState(QDomDocument &doc) const { return QDomElement(); }

    virtual bool xmlLoadState(QDomElement &parent_element ) { return false; }

    void setDataMap(const PlotDataMapRef* datamap) { _datamap = datamap; }

protected:
    QMenu* _menu;
    const PlotDataMapRef *_datamap;
};

QT_BEGIN_NAMESPACE

#define StatePublisher_iid "com.icarustechnology.PlotJuggler.StatePublisher"

Q_DECLARE_INTERFACE(StatePublisher, StatePublisher_iid)

QT_END_NAMESPACE


#endif

