#ifndef DragableWidget_H
#define DragableWidget_H

#include <map>
#include <QObject>
#include <QTextEdit>
#include <qwt_plot.h>
#include <qwt_plot_curve.h>
#include <qwt_plot_grid.h>
#include <qwt_symbol.h>
#include <qwt_legend.h>
#include <deque>
#include <QMessageBox>
#include <QTime>
#include "plotmagnifier.h"
#include "plotzoomer.h"
#include <qwt_plot_panner.h>
#include <QDomDocument>
#include "timeseries_qwt.h"
#include "customtracker.h"
#include "axis_limits_dialog.h"
#include <qwt_plot_legenditem.h>

class PlotWidget : public QwtPlot
{
    Q_OBJECT

public:

    PlotWidget(PlotDataMap& datamap, QWidget *parent=0);
    virtual ~PlotWidget();

    bool addCurve(const QString&  name, bool do_replot );

    bool isEmpty() const;

    const std::map<QString, std::shared_ptr<QwtPlotCurve> > &curveList() const;

    QDomElement xmlSaveState(QDomDocument &doc) const;

    bool xmlLoadState(QDomElement &element, QMessageBox::StandardButton* answer);

    QRectF currentBoundingRect() const;

    PlotData::RangeTime getMaximumRangeX() const;

    PlotData::RangeValue getMaximumRangeY( PlotData::RangeTime range_X, bool absolute_time ) const;

    void setScale( QRectF rect, bool emit_signal );

    void reloadPlotData( );

    void changeAxisX(QString curve_name);

    bool isXYPlot() const;

protected:
    virtual void dragEnterEvent(QDragEnterEvent *event) ;
    virtual void dragMoveEvent(QDragMoveEvent *event) ;
    virtual void dropEvent(QDropEvent *event) ;
    virtual bool eventFilter(QObject *obj, QEvent *event);


signals:
    void swapWidgetsRequested(PlotWidget* source, PlotWidget* destination);
    void rectChanged(PlotWidget* self, QRectF rect );
    void undoableChange();
    void trackerMoved(QPointF pos);

public slots:

    void updateCurves(bool force);

    void replot() ;

    void detachAllCurves();

    void zoomOut(bool emit_signal);

    void on_zoomOutHorizontal_triggered(bool emit_signal = true);

    void on_zoomOutVertical_triggered(bool emit_signal = true);

    void removeCurve(const QString& name);

    void activateLegent(bool activate);

    void activateGrid(bool activate);

    void configureTracker(CurveTracker::Parameter val);

    void enableTracker(bool enable);

    void setTrackerPosition(double abs_time);

    void on_changeTimeOffset(double offset);

private slots:

    void on_noTransform_triggered(bool checked );

    void on_1stDerivativeTransform_triggered(bool checked);

    void on_2ndDerivativeTransform_triggered(bool checked);

    void on_convertToXY_triggered(bool checked);

    void on_savePlotToFile();

    void on_editAxisLimits_triggered();

private slots:
    void launchRemoveCurveDialog();
    void canvasContextMenuTriggered(const QPoint &pos);
    void on_changeColorsDialog_triggered();
    void on_changeColor(QString curve_name, QColor new_color);
    void on_showPoints_triggered(bool checked);
    void on_externallyResized(const QRectF &new_rect);


private:
    std::map<QString, std::shared_ptr<QwtPlotCurve> > _curve_list;
    std::map<QString, QwtPlotMarker*> _point_marker;

    QAction *_action_removeCurve;
    QAction *_action_removeAllCurves;
    QAction *_action_changeColorsDialog;
    QAction *_action_showPoints;
    QAction *_action_zoomOutHorizontally;
    QAction *_action_zoomOutVertically;
    QAction *_action_noTransform;
    QAction *_action_1stDerivativeTransform;
    QAction *_action_2ndDerivativeTransform;
    QAction *_action_phaseXY;
    QAction *_action_saveToFile;
    QAction *_action_editLimits;

    PlotZoomer* _zoomer;
    PlotMagnifier* _magnifier;
    QwtPlotPanner* _panner;

    CurveTracker* _tracker;
    QwtPlotLegendItem* _legend;
    QwtPlotGrid* _grid;

    PlotDataMap& _mapped_data;
    TimeseriesQwt::Transform _current_transform;

    void buildActions();
    void buildLegend();

    int   _fps_counter;
    QTime _fps_timeStamp;
    bool _show_line_and_points;

    void setDefaultRangeX();

    PlotDataPtr _axisX;

    double _time_offset;

    PlotData::RangeValue _custom_Y_limits;

    AxisLimitsDialog* _axis_limits_dialog;

};

#endif
