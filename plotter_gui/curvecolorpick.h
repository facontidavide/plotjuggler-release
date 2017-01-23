#ifndef CURVECOLORPICK_H
#define CURVECOLORPICK_H

#include <QDialog>
#include <QListWidgetItem>
#include "color_wheel.hpp"
#include "color_preview.hpp"

namespace Ui {
class CurveColorPick;
}

class CurveColorPick : public QDialog
{
    Q_OBJECT

public:
    explicit CurveColorPick(std::map<QString, QColor>* mapped_colors, QWidget *parent = 0);
    ~CurveColorPick();

private slots:
    void on_pushButtonClose_clicked();

    void on_pushButtonApply_clicked();

    void on_pushButtonReset_clicked();

    void on_listWidget_itemClicked(QListWidgetItem *item);

    void on_pushButtonResetAll_clicked();

private:
    Ui::CurveColorPick *ui;
    std::map<QString, QColor>  _prev_colors;
    std::map<QString, QColor>* _colors_ptr;

    color_widgets::ColorWheel   *color_wheel;
    color_widgets::ColorPreview *color_preview;
};

#endif // CURVECOLORPICK_H
