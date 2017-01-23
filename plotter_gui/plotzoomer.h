#ifndef PLOTZOOMER_H
#define PLOTZOOMER_H

#include <QObject>
#include <qwt_plot_zoomer.h>

class PlotZoomer : public QwtPlotZoomer
{
public:
    PlotZoomer();

    explicit PlotZoomer( QWidget *, bool doReplot = true );

    virtual ~PlotZoomer() = default;
protected:
     virtual void widgetMouseReleaseEvent( QMouseEvent * ) override;
};

#endif // PLOTZOOMER_H
