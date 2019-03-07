#ifndef PLOTZOOMER_H
#define PLOTZOOMER_H

#include <QObject>
#include <qwt_plot_zoomer.h>

class PlotZoomer : public QwtPlotZoomer
{
public:
    PlotZoomer();

    explicit PlotZoomer( QWidget *, bool doReplot = true );

    virtual ~PlotZoomer() override = default;
protected:
    virtual void widgetMousePressEvent( QMouseEvent * event)  override;
    virtual void widgetMouseReleaseEvent( QMouseEvent * event) override;
    virtual void widgetMouseMoveEvent( QMouseEvent * event) override;
    virtual bool accept( QPolygon & ) const override;

    virtual QSizeF minZoomSize() const override;
private:
    bool _mouse_pressed;
    bool _zoom_enabled;
    QPoint _initial_pos;
};

#endif // PLOTZOOMER_H
