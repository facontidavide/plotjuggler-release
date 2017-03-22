#ifndef PLOTMAGNIFIER_H
#define PLOTMAGNIFIER_H

#include <QTimer>
#include <qwt_plot_magnifier.h>
#include <qwt_plot.h>

class PlotMagnifier : public QwtPlotMagnifier
{
    Q_OBJECT


public:
    explicit PlotMagnifier( QWidget *canvas);
    virtual ~PlotMagnifier();

    void setAxisLimits(int axis,double lower, double upper);

protected:
    virtual void rescale( double factor ) override;

    virtual void widgetWheelEvent( QWheelEvent *event ) override;
    virtual void widgetMousePressEvent( QMouseEvent* event ) override;

    double _lower_bounds[QwtPlot::axisCnt];
    double _upper_bounds[QwtPlot::axisCnt];

    QPointF _mouse_position;

signals:
    void rescaled(QRectF new_size);

private:
    QPointF invTransform(QPoint pos);
    QTimer _future_emit;

};

#endif // PLOTMAGNIFIER_H
