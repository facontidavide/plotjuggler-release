#include "curvecolorpick.h"
#include "ui_curvecolorpick.h"
#include <QColorDialog>


CurveColorPick::CurveColorPick(const std::map<QString, QColor> &mapped_colors, QWidget *parent) :
    QDialog(parent),
    ui(new Ui::CurveColorPick),
    _any_modified(false),
    _mapped_colors(mapped_colors)
{
    ui->setupUi(this);

    for(auto it : _mapped_colors)
    {
        QListWidgetItem* item = new QListWidgetItem( it.first );
        item->setForeground( it.second );
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

bool CurveColorPick::anyColorModified() const
{
    return _any_modified;
}

void CurveColorPick::on_pushButtonClose_clicked()
{
    this->accept();
}

void CurveColorPick::on_pushButtonApply_clicked()
{
    QListWidgetItem *item = ui->listWidget->currentItem();
    if( color_preview->color() != item->foreground().color())
    {
        _any_modified = true;
        item->setForeground( color_preview->color() );
        emit changeColor( item->text(), color_preview->color() );
    }
}

void CurveColorPick::on_listWidget_itemClicked(QListWidgetItem *item)
{
    color_wheel->setColor( item->foreground().color() );
}

