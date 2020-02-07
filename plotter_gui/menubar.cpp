#include "menubar.h"
#include <QDebug>
#include <QFontDatabase>
#include <QFont>
#include <QColor>

MenuBar::MenuBar(QWidget *parent): QMenuBar(parent),
                                     _painter(this)
{

  int font_id = QFontDatabase::addApplicationFont("://resources/DejaVuSans-ExtraLight.ttf");
  QString family = QFontDatabase::applicationFontFamilies(font_id).at(0);
  QFont font(family);
  font.setStyleStrategy(QFont::PreferAntialias);

  _painter.setFont( QFont(family, 12) );
  _width_plot = _painter.fontMetrics().width("Plot");
  _width_juggler = _painter.fontMetrics().width("Juggler");
}

void MenuBar::paintEvent(QPaintEvent *event)
{
  QMenuBar::paintEvent(event);

  int text_width = _width_plot + _width_juggler;
  {
    QPoint topleft(this->rect().width() - text_width - 12, 0);
    QSize rect_size(_width_plot, this->rect().height());
    _painter.setPen(QColor("#ce0e73"));
    _painter.drawText(QRect(topleft, rect_size), Qt::AlignHCenter | Qt::AlignVCenter, "Plot");
  }
  {
    QPoint topleft(this->rect().width() - _width_juggler - 10, 0);
    QSize rect_size(_width_juggler, this->rect().height());
    _painter.setPen(QColor("#1b72cf"));
    _painter.drawText(QRect(topleft, rect_size), Qt::AlignHCenter | Qt::AlignVCenter, "Juggler");
  }
}

