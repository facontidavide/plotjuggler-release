#ifndef BUSYDIALOG_H
#define BUSYDIALOG_H

#include <QDialog>

namespace Ui {
class BusyDialog;
}

class BusyDialog : public QDialog
{
    Q_OBJECT

public:
    explicit BusyDialog(QWidget *parent );
    ~BusyDialog();

private:
    Ui::BusyDialog *ui;
};

#endif // BUSYDIALOG_H
