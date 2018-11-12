#ifndef REMOVECURVEDIALOG_H
#define REMOVECURVEDIALOG_H

#include <QDialog>
#include <QListWidgetItem>
#include <qwt_plot_curve.h>

namespace Ui {
class RemoveCurveDialog;
}

class PlotWidget;

class RemoveCurveDialog : public QDialog
{
    Q_OBJECT

public:
    explicit RemoveCurveDialog(PlotWidget *parent);
    ~RemoveCurveDialog();

    void addCurveName(const QString& name, const QColor &color);

private slots:
    void on_listCurveWidget_itemClicked(QListWidgetItem *item);

    void on_pushButtonRemove_pressed();

    void on_pushButtonSelectAll_pressed();

private:
    Ui::RemoveCurveDialog *ui;

    void closeIfEmpty();

    PlotWidget *_parent;
};

#endif // REMOVECURVEDIALOG_H
