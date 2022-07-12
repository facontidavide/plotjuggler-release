/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#ifndef MENUBAR_H
#define MENUBAR_H

#include <QMenuBar>
#include <QPaintEvent>
#include <QPainter>

class MenuBar : public QMenuBar
{
public:
  MenuBar(QWidget* parent);
  // void paintEvent(QPaintEvent* event);

private:
  QFont _font;
  int _width_plot;
  int _width_juggler;
};

#endif  // MENUBAR_H
