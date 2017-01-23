#ifndef SUBWINDOW_H
#define SUBWINDOW_H

#include <QMainWindow>
#include "tabbedplotwidget.h"

class SubWindow : public QMainWindow
{
    Q_OBJECT
public:
    explicit SubWindow(PlotDataMap *mapped_data, QMainWindow *parent_window);

    virtual ~SubWindow();

    TabbedPlotWidget* tabbedWidget() { return tabbed_widget_;}

signals:
    void closeRequestedByUser();

protected:
    virtual void closeEvent(QCloseEvent *event) override;
    TabbedPlotWidget* tabbed_widget_;

};

#endif // SUBWINDOW_H
