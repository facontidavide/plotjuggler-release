#include "subwindow.h"
#include <QDebug>

SubWindow::SubWindow(PlotMatrix *first_tab, PlotDataMap &mapped_data, QMainWindow *parent_window) :
  QMainWindow(parent_window)
{
    tabbed_widget_ = new TabbedPlotWidget(parent_window, first_tab, mapped_data, 0 );
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
