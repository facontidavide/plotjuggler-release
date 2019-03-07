#include "plotzoomer.h"
#include <QMouseEvent>
#include <QApplication>
#include "qwt_scale_map.h"
#include "qwt_plot.h"

PlotZoomer::PlotZoomer(QWidget *canvas, bool doReplot):
    QwtPlotZoomer(canvas,doReplot),
    _mouse_pressed(false),
    _zoom_enabled(false)
{
    this->setTrackerMode(AlwaysOff);
}

void PlotZoomer::widgetMousePressEvent(QMouseEvent *me)
{
    _mouse_pressed = false;
    auto patterns = this->mousePattern();
    for (QwtEventPattern::MousePattern& pattern: patterns)
    {
        if( this->mouseMatch(pattern, me) ){
            _mouse_pressed = true;
           // this->setTrackerMode(AlwaysOn);
            _initial_pos = me->pos();
        }
        break;
    }
    QwtPlotPicker::widgetMousePressEvent( me );
}

void PlotZoomer::widgetMouseMoveEvent(QMouseEvent *me)
{
    static QCursor zoom_cursor(QPixmap(":/icons/resources/light/zoom_in.png"));

    if( _mouse_pressed )
    {
        auto patterns = this->mousePattern();
        for (QwtEventPattern::MousePattern& pattern: patterns)
        {
            QRect rect( me->pos(), _initial_pos );
            QRectF zoomRect = invTransform( rect.normalized() );

            if( zoomRect.width()  > minZoomSize().width() &&
                zoomRect.height() > minZoomSize().height()    )
            {
                if( !_zoom_enabled)
                {
                    _zoom_enabled = true;
                    this->setRubberBand( RectRubberBand );
                    this->setTrackerMode(AlwaysOff);
                    QApplication::setOverrideCursor(zoom_cursor);
                }
            }
            else if( _zoom_enabled)
            {
               _zoom_enabled = false;
               this->setRubberBand( NoRubberBand );
               QApplication::restoreOverrideCursor();
            }
            break;
        }
    }
    QwtPlotPicker::widgetMouseMoveEvent( me );
}

void PlotZoomer::widgetMouseReleaseEvent(QMouseEvent *me)
{
    _mouse_pressed = false;
    _zoom_enabled = false;
    QwtPlotPicker::widgetMouseReleaseEvent( me );
    this->setTrackerMode(AlwaysOff);
}

bool PlotZoomer::accept(QPolygon &pa) const
{
    QApplication::restoreOverrideCursor();

    if ( pa.count() < 2 )
        return false;

    QRect rect = QRect( pa[0], pa[int( pa.count() ) - 1] );
    QRectF zoomRect = invTransform( rect.normalized() );

    if ( zoomRect.width()  < minZoomSize().width() &&
         zoomRect.height() < minZoomSize().height() ){
        return false;
    }
    return true;
}

QSizeF PlotZoomer::minZoomSize() const
{
    return QSizeF(scaleRect().width() * 0.02,
                  scaleRect().height() * 0.02);
}
