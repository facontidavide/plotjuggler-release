#include "busytaskdialog.h"
#include <QApplication>

BusyTaskDialog::BusyTaskDialog(QString text, QWidget *parent) :
    QProgressDialog(text, "Cancel", 0, 100, parent)
{
   setWindowFlags(Qt::FramelessWindowHint | Qt::WindowStaysOnTopHint);
   setAttribute( Qt::WA_DeleteOnClose, true );
}





