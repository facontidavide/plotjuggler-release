#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QElapsedTimer>
#include <QShortcut>
#include <QCommandLineParser>
#include <QSignalMapper>
#include <set>
#include <deque>
#include <functional>
#include "plotwidget.h"
#include "plotmatrix.h"
#include "filterablelistwidget.h"
#include "tabbedplotwidget.h"
#include "subwindow.h"
#include "realslider.h"
#include "utils.h"
#include "PlotJuggler/dataloader_base.h"
#include "PlotJuggler/statepublisher_base.h"
#include "PlotJuggler/datastreamer_base.h"

namespace Ui {
class MainWindow;
}


class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(const QCommandLineParser& commandline_parser, QWidget *parent = 0);
    ~MainWindow();

public slots:
    void onUndoableChange();

private slots:

    void onTrackerTimeUpdated(double absolute_time );

    void onTrackerMovedFromWidget(QPointF pos );

    void onSplitterMoved(int, int);

    void resizeEvent(QResizeEvent *) ;

    void onPlotAdded(PlotWidget* plot);

    void onPlotMatrixAdded(PlotMatrix* matrix);

    void onActionSaveLayout();

    void onActionLoadLayout(bool reload_previous = false);

    void onActionLoadLayoutFromFile(QString filename, bool load_data);

    void onActionLoadDataFile();

    void onReloadDatafile();

    void onActionLoadDataFileImpl(QString filename, bool reuse_last_configuration = false );

    void onActionReloadRecentDataFile();

    void onActionReloadRecentLayout();

    void onActionLoadStreamer(QString streamer_name);

    void onUndoInvoked();

    void onRedoInvoked();

    void on_tabbedAreaDestroyed(QObject*object);

    void onFloatingWindowDestroyed(QObject*object);

    void onCreateFloatingWindow(PlotMatrix* first_tab = NULL);

    void onSwapPlots(PlotWidget* source, PlotWidget* destination);

    void on_pushButtonStreaming_toggled(bool streaming);

    void on_ToggleStreaming();

    void updateDataAndReplot();

    void on_streamingSpinBox_valueChanged(int value);

    void onDeleteLoadedData();

    void on_actionAbout_triggered();

    void on_actionStopStreaming_triggered();

    void on_actionExit_triggered();

    void on_actionQuick_Help_triggered();

    void onTimeSlider_valueChanged(double value);

    void updateLeftTableValues();

    void deleteDataOfSingleCurve(const QString &curve_name);

    void on_pushButtonRemoveTimeOffset_toggled(bool checked);

    void on_pushButtonOptions_toggled(bool checked);

    void on_pushButtonActivateGrid_toggled(bool checked);

    void on_actionClearBuffer_triggered();

    void on_pushButtonUseDateTime_toggled(bool checked);

    void on_pushButtonTimeTracker_pressed();

    void on_minimizeView();

private:

    Ui::MainWindow *ui;

    TabbedPlotWidget *      _main_tabbed_widget;

    QShortcut _undo_shortcut;
    QShortcut _redo_shortcut;
    QShortcut _minimize_view;
    QShortcut _toggle_streaming;

    bool _minimized;

    void createActions();

    FilterableListWidget* _curvelist_widget;

    void updatedDisplayTime();

    void forEachWidget(std::function<void(PlotWidget*, PlotMatrix*, int, int)> op);

    void forEachWidget(std::function<void(PlotWidget*)> op);

    void updateTimeSlider();

    void buildDummyData();

    PlotDataMap    _mapped_plot_data;

    void rearrangeGridLayout();

    void loadPlugins(QString subdir_name);

    std::map<QString,DataLoader*>      _data_loader;

    std::map<QString,StatePublisher*>  _state_publisher;

    std::map<QString,DataStreamer*>    _data_streamer;

    DataLoader*   _last_dataloader;
    DataStreamer* _current_streamer;

    QDomDocument xmlSaveState() const;

    bool xmlLoadState(QDomDocument state_document);

    std::deque<QDomDocument> _undo_states;

    std::deque<QDomDocument> _redo_states;

    QElapsedTimer _undo_timer;

    bool _disable_undo_logging;

    bool _test_option;

    double _tracker_time;

    QString _loaded_datafile;

    QSignalMapper *_streamer_signal_mapper;

    void createTabbedDialog(QString suggest_win_name, PlotMatrix *first_tab);

    void importPlotDataMap(const PlotDataMap &new_data, bool delete_older);

    bool isStreamingActive() const ;

    CurveTracker::Parameter _tracker_param;

    std::map<CurveTracker::Parameter, QIcon> _tracker_button_icons;

    void closeEvent(QCloseEvent *event);

    void loadPluginState(const QDomElement &root);
    
    void savePluginState(QDomDocument &doc);
    
protected:

    MonitoredValue _time_offset;

    QTimer *_replot_timer;
signals:
    void requestRemoveCurveByName(const QString& name);

    void activateStreamingMode( bool active);

    void activateTracker(bool active);

};



#endif // MAINWINDOW_H
