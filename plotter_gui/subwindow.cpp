#include "subwindow.h"
#include <QDebug>
#include <QSettings>

SubWindow::SubWindow(QString name, PlotMatrix *first_tab, PlotDataMap &mapped_data, QMainWindow *parent_window) :
  QMainWindow(parent_window)
{
    tabbed_widget_ = new TabbedPlotWidget( name, parent_window, first_tab, mapped_data, this );
    this->setCentralWidget( tabbed_widget_ );

    Qt::WindowFlags flags = this->windowFlags();
    this->setWindowFlags( flags | Qt::SubWindow );
    this->setWindowTitle( tabbed_widget_->name() );

    QSettings settings;
    restoreGeometry(settings.value( QString("SubWindow.%1.geometry").arg(name) ).toByteArray());

    this->setAttribute( Qt::WA_DeleteOnClose );
}

SubWindow::~SubWindow()
{
    QSettings settings;
    settings.setValue(QString("SubWindow.%1.geometry").arg( tabbedWidget()->name() ), saveGeometry());
    tabbed_widget_->close();
}

void SubWindow::closeEvent(QCloseEvent *event)
{
    QMainWindow::closeEvent(event);
}
