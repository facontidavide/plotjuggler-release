#ifndef RANDOM_COLOR_H
#define RANDOM_COLOR_H

#include <QColor>

inline QColor randomColorHint()
{
    static int index = 0;
    QColor color;
   // https://matplotlib.org/3.1.1/users/dflt_style_changes.html
    switch( index%8 )
    {
    case 0:  color =  QColor("#1f77b4");   break;
    case 1:  color =  QColor("#d62728");   break;
    case 2:  color =  QColor("#1ac938");   break;
    case 3:  color =  QColor("#ff7f0e");   break;

    case 4:  color =  QColor("#f14cc1");   break;
    case 5:  color =  QColor("#9467bd");   break;
    case 6:  color =  QColor("#17becf");   break;
    case 7:  color =  QColor("#bcbd22");   break;
    }
    index++;
    return color;
}

#endif // RANDOM_COLOR_H
