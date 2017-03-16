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
    explicit CurveColorPick(const std::map<QString, QColor>& mapped_colors, QWidget *parent = 0);
    ~CurveColorPick();

    bool anyColorModified() const ;

private slots:
    void on_pushButtonClose_clicked();

    void on_pushButtonApply_clicked();

    void on_listWidget_itemClicked(QListWidgetItem *item);


signals:
    void changeColor(QString, QColor);

private:
    Ui::CurveColorPick *ui;
    color_widgets::ColorWheel   *color_wheel;
    color_widgets::ColorPreview *color_preview;

    const std::map<QString, QColor>& _mapped_colors;
    bool _any_modified;
};

#endif // CURVECOLORPICK_H
