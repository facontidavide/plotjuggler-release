#include "subwindow.h"
#include <QDebug>

SubWindow::SubWindow(PlotDataMap *mapped_data, QMainWindow *parent_window) :
  QMainWindow(parent_window)
{
    tabbed_widget_ = new TabbedPlotWidget(parent_window, mapped_data, 0 );
    this->setCentralWidget( tabbed_widget_ );
}

SubWindow::~SubWindow(){

}

void SubWindow::closeEvent(QCloseEvent *event)
{
    qDebug() << "SubWindow::closeEvent";
    emit closeRequestedByUser();
    QMainWindow::closeEvent(event);
}
