#ifndef RANDOM_COLOR_H
#define RANDOM_COLOR_H

#include <QColor>

inline QColor randomColorHint()
{
    static int index = 0;
    QColor color;
    switch( index%9 )
    {
    case 0:  color =  QColor("#1464a0");   break; //blue
    case 1:  color =  QColor("#05740d");   break; //green
    case 2:  color =  QColor("#ff1318");   break; //red
    case 3:  color =  QColor("#f700f7");   break; //magenta
    case 4:  color =  QColor("#0033ee");   break; //dark blue
    case 5:  color =  QColor("#007777");   break; //dark cyan
    case 6:  color =  QColor("#50b47f");   break; // green-ish
    case 7:  color =  QColor("#f4531d");   break; //orange
    case 8:  color =  QColor("#8e3488");   break; //volet
    }
    index++;
    return color;
}

#endif // RANDOM_COLOR_H
