#include "curvecolorpick.h"
#include "ui_curvecolorpick.h"
#include <QColorDialog>


CurveColorPick::CurveColorPick(std::map<QString, QColor>* mapped_colors, QWidget *parent) :
    QDialog(parent),
    ui(new Ui::CurveColorPick)
{
    _colors_ptr = mapped_colors;
    _prev_colors = *mapped_colors;

    ui->setupUi(this);

    std::map<QString, QColor>::iterator it;
    for(it = mapped_colors->begin(); it != mapped_colors->end(); ++it)
    {
        QListWidgetItem* item = new QListWidgetItem( it->first );
        item->setForeground( it->second );
        ui->listWidget->addItem( item );
    }

    color_wheel = new  color_widgets::ColorWheel(this);
    ui->verticalLayoutRight->insertWidget(0, color_wheel );
    color_wheel->setMinimumWidth(150);
    color_wheel->setMinimumHeight(150);

    color_preview = new  color_widgets::ColorPreview(this);
    ui->verticalLayoutRight->insertWidget(1, color_preview );
    color_preview->setMinimumWidth(150);
    color_preview->setMinimumHeight(100);

    connect(color_wheel, SIGNAL(colorChanged(QColor)), color_preview, SLOT(setColor(QColor)) );

}

CurveColorPick::~CurveColorPick()
{
    delete ui;
}

void CurveColorPick::on_pushButtonClose_clicked()
{
    this->accept();
}

void CurveColorPick::on_pushButtonApply_clicked()
{
    QListWidgetItem *item = ui->listWidget->currentItem();
    item->setForeground( color_preview->color() );

    (*_colors_ptr)[ item->text() ] = color_preview->color();
}

void CurveColorPick::on_pushButtonReset_clicked()
{
    QListWidgetItem *item = ui->listWidget->currentItem();
    QString name = item->text();

    QColor previous = _prev_colors[ name ];

    (*_colors_ptr)[ name ] = previous;
    item->setForeground( previous );
    color_wheel->setColor( previous );
}

void CurveColorPick::on_listWidget_itemClicked(QListWidgetItem *item)
{
    color_wheel->setColor( item->foreground().color() );
}

void CurveColorPick::on_pushButtonResetAll_clicked()
{
    for(int i = 0; i < ui->listWidget->count(); ++i)
    {
        QListWidgetItem* item = ui->listWidget->item(i);
        QString name = item->text();

        QColor previous = _prev_colors[ name ];

        (*_colors_ptr)[ name ] = previous;
        item->setForeground( previous );
    }
}
