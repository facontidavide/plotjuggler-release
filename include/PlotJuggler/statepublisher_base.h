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

    virtual void play(double interval) = 0;

    virtual ~StatePublisher() {}

    virtual void setEnabled(bool enabled)
    {
        auto prev = _action->blockSignals(true);
        _action->setChecked(enabled);
        _action->blockSignals(prev);
    }

    virtual bool isDebugPlugin() { return false; }

    virtual void setParentMenu(QMenu* parent_menu, QAction* parent_action)
    {
        _menu   = parent_menu;
        _action = parent_action;
    }

    virtual QWidget* embeddedWidget() { return nullptr; }

    virtual bool xmlSaveState(QDomDocument &doc, QDomElement &parent_element) const { return false; }

    virtual bool xmlLoadState(const QDomElement &parent_element ) { return false; }

    void setDataMap(const PlotDataMapRef* datamap) { _datamap = datamap; }

    QDomElement xmlSaveState(QDomDocument &doc) const
    {
        QDomElement plugin_elem = doc.createElement("plugin");
        plugin_elem.setAttribute("ID", QString(this->name()).replace(" ", "_") );
        xmlSaveState(doc, plugin_elem);
        return plugin_elem;
    }


protected:
    QMenu* _menu;
    QAction* _action;
    const PlotDataMapRef *_datamap;
};

QT_BEGIN_NAMESPACE

#define StatePublisher_iid "com.icarustechnology.PlotJuggler.StatePublisher"

Q_DECLARE_INTERFACE(StatePublisher, StatePublisher_iid)

QT_END_NAMESPACE


#endif

