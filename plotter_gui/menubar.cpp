#include "menubar.h"
#include <QDebug>
#include <QFontDatabase>
#include <QPainter>
#include <QFont>
#include <QColor>

void MenuBar::paintEvent(QPaintEvent *event)
{
    QMenuBar::paintEvent(event);

    int font_id = QFontDatabase::addApplicationFont("://resources/DejaVuSans-ExtraLight.ttf");
    QString family = QFontDatabase::applicationFontFamilies(font_id).at(0);
    QFont font(family);
    font.setStyleStrategy(QFont::PreferAntialias);

    QPainter painter(this);
    painter.setFont( QFont(family, 12) );

    int width_plot = painter.fontMetrics().width("Plot");
    int width_juggler = painter.fontMetrics().width("Juggler");
    int text_width = width_plot + width_juggler;
    {
        QPoint topleft( this->rect().width() - text_width - 12 , 0);
        QSize rect_size( width_plot, this->rect().height() );
        painter.setPen( QColor("#ce0e73"));
        painter.drawText( QRect(topleft, rect_size),
                          Qt::AlignHCenter | Qt::AlignVCenter, "Plot" );
    }
    {
        QPoint topleft( this->rect().width() - width_juggler - 10 , 0);
        QSize rect_size( width_juggler, this->rect().height() );
        painter.setPen( QColor("#1b72cf"));
        painter.drawText( QRect(topleft, rect_size),
                          Qt::AlignHCenter | Qt::AlignVCenter, "Juggler" );
    }
}

