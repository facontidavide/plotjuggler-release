#ifndef PLOTMAGNIFIER_H
#define PLOTMAGNIFIER_H

#include <QTimer>
#include <qwt_plot_magnifier.h>
#include <qwt_plot.h>
#include <QEvent>

class PlotMagnifier : public QwtPlotMagnifier
{
    Q_OBJECT


public:
    explicit PlotMagnifier( QWidget *canvas);
    virtual ~PlotMagnifier() override;

    void setAxisLimits(int axis,double lower, double upper);
    virtual void rescale( double factor ) override;
    virtual void widgetWheelEvent( QWheelEvent *event ) override;

protected:

    virtual void widgetMousePressEvent( QMouseEvent* event ) override;
//    virtual void widgetKeyPressEvent( QKeyEvent * event) override;
//    virtual void widgetKeyReleaseEvent( QKeyEvent *event)  override;

    bool eventFilter( QObject *obj, QEvent *event) override;

    double _lower_bounds[QwtPlot::axisCnt];
    double _upper_bounds[QwtPlot::axisCnt];

    QPointF _mouse_position;

signals:
    void rescaled(QRectF new_size);

private:
    QPointF invTransform(QPoint pos);
    QTimer _future_emit;
    bool _x_pressed;
    bool _y_pressed;
};

#endif // PLOTMAGNIFIER_H
