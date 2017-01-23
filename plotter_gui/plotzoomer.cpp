#include "plotzoomer.h"
#include <QMouseEvent>


PlotZoomer::PlotZoomer(QWidget *canvas, bool doReplot):
    QwtPlotZoomer(canvas,doReplot)
{

}

void PlotZoomer::widgetMouseReleaseEvent(QMouseEvent *me)
{
    if ( me->button() == Qt::MouseButton::LeftButton )
    {
        QwtPlotPicker::widgetMouseReleaseEvent( me );
    }
}
