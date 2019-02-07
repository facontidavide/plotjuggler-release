#include "plotmagnifier.h"
#include <qwt_plot.h>
#include <QDebug>
#include <limits>
#include <QWheelEvent>
#include <QApplication>

PlotMagnifier::PlotMagnifier( QWidget *canvas) : QwtPlotMagnifier(canvas)
{
    _x_pressed = false;
    _y_pressed = false;
    for ( int axisId = 0; axisId < QwtPlot::axisCnt; axisId++ )
    {
        _lower_bounds[axisId] = -std::numeric_limits<double>::max();
        _upper_bounds[axisId] =  std::numeric_limits<double>::max();
    }
}

PlotMagnifier::~PlotMagnifier() {}

void PlotMagnifier::setAxisLimits(int axis, double lower, double upper)
{
    if ( axis >= 0 && axis < QwtPlot::axisCnt )
    {
        _lower_bounds[axis] = lower;
        _upper_bounds[axis] = upper;
    }
}

void PlotMagnifier::rescale( double factor )
{
    factor = qAbs( factor );

    QwtPlot* plt = plot();
    if ( plt == NULL || factor == 1.0 ){
        return;
    }

    bool doReplot = false;

    const bool autoReplot = plt->autoReplot();
    plt->setAutoReplot( false );

    const int axis_list[2] = {QwtPlot::xBottom, QwtPlot::yLeft};
    QRectF new_rect;

    for ( int i = 0; i <2; i++ )
    {
        double temp_factor = factor;
        if( i==1 && _x_pressed)
        {
            temp_factor = 1.0;
        }
        if( i==0 && _y_pressed)
        {
            temp_factor = 1.0;
        }

        int axisId = axis_list[i];

        if ( isAxisEnabled( axisId ) )
        {
            const QwtScaleMap scaleMap = plt->canvasMap( axisId );

            double v1 = scaleMap.s1();
            double v2 = scaleMap.s2();
            double center = _mouse_position.x();

            if( axisId == QwtPlot::yLeft){
                center = _mouse_position.y();
            }

            if ( scaleMap.transformation() )
            {
                // the coordinate system of the paint device is always linear
                v1 = scaleMap.transform( v1 ); // scaleMap.p1()
                v2 = scaleMap.transform( v2 ); // scaleMap.p2()
            }

            const double width = ( v2 - v1 );
            const double ratio = (v2-center)/ (width);

            v1 = center - width*temp_factor*(1-ratio);
            v2 = center + width*temp_factor*(ratio);

            if( v1 > v2 ) std::swap( v1, v2 );

            if ( scaleMap.transformation() )
            {
                v1 = scaleMap.invTransform( v1 );
                v2 = scaleMap.invTransform( v2 );
            }

            if( v1 < _lower_bounds[axisId]) v1 = _lower_bounds[axisId];
            if( v2 > _upper_bounds[axisId]) v2 = _upper_bounds[axisId];

            plt->setAxisScale( axisId, v1, v2 );

            if( axisId == QwtPlot::xBottom)
            {
                new_rect.setLeft(  v1 );
                new_rect.setRight( v2 );
            }
            else{
                new_rect.setBottom( v1 );
                new_rect.setTop( v2 );
            }

            doReplot = true;
        }
    }

    plt->setAutoReplot( autoReplot );

    if ( doReplot ){
        emit rescaled( new_rect );
    }
}

QPointF PlotMagnifier::invTransform(QPoint pos)
{
    QwtScaleMap xMap = plot()->canvasMap( QwtPlot::xBottom );
    QwtScaleMap yMap = plot()->canvasMap( QwtPlot::yLeft );
    return QPointF ( xMap.invTransform( pos.x() ), yMap.invTransform( pos.y() ) );
}

void PlotMagnifier::widgetWheelEvent(QWheelEvent *event)
{
    _mouse_position = invTransform(event->pos());
    QwtPlotMagnifier::widgetWheelEvent(event);
}

void PlotMagnifier::widgetMousePressEvent(QMouseEvent *event)
{
    _mouse_position = invTransform(event->pos());
    QwtPlotMagnifier::widgetMousePressEvent(event);
}

bool PlotMagnifier::eventFilter(QObject *obj, QEvent *event)
{
    if( event->type() == QEvent::Enter || event->type() == QEvent::Leave)
    {
        _x_pressed = false;
        _y_pressed = false;
    }
    else if( event->type() == QEvent::KeyPress)
    {
        auto key_event = static_cast<QKeyEvent*>(event);
        if( key_event->key() == Qt::Key_X )
        {
            _x_pressed = true;
        }
        else if( key_event->key() == Qt::Key_Y )
        {
            _y_pressed = true;
        }
        //qDebug() << "p "<< _x_pressed << " " << _y_pressed;
    }
    else if( event->type() == QEvent::KeyRelease)
    {
        auto key_event = static_cast<QKeyEvent*>(event);
        if( key_event->key() == Qt::Key_X )
        {
            _x_pressed = false;
        }
        else if( key_event->key() == Qt::Key_Y )
        {
            _y_pressed = false;
        }
      //  qDebug() << "R "<< _x_pressed << " " << _y_pressed;
    }

    return QwtPlotMagnifier::eventFilter(obj, event);
}
