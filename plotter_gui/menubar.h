#ifndef MENUBAR_H
#define MENUBAR_H

#include <QMenuBar>
#include <QPaintEvent>

class MenuBar : public QMenuBar
{
public:
    MenuBar(QWidget* parent): QMenuBar(parent){}
    void paintEvent(QPaintEvent *event);
};

#endif // MENUBAR_H
