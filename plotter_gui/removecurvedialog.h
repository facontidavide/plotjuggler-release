#ifndef REMOVECURVEDIALOG_H
#define REMOVECURVEDIALOG_H

#include <QDialog>
#include <QListWidgetItem>
#include <qwt_plot_curve.h>

namespace Ui {
class RemoveCurveDialog;
}

class RemoveCurveDialog : public QDialog
{
    Q_OBJECT

public:
    explicit RemoveCurveDialog(QWidget *parent);
    ~RemoveCurveDialog();

    void addCurveName(const QString& name);

private slots:
    void on_listCurveWidget_itemClicked(QListWidgetItem *item);

    void on_pushButtonRemove_pressed();

    void on_pushButtonSelectAll_pressed();

private:
    Ui::RemoveCurveDialog *ui;

    void closeIfEmpty();
};

#endif // REMOVECURVEDIALOG_H
