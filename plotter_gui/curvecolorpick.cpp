#include "curvecolorpick.h"
#include "ui_curvecolorpick.h"
#include <QColorDialog>


CurveColorPick::CurveColorPick(const std::map<std::string, QColor> &mapped_colors, QWidget *parent) :
    QDialog(parent),
    ui(new Ui::CurveColorPick),
    _any_modified(false),
    _mapped_colors(mapped_colors)
{
    ui->setupUi(this);

    for(auto& it : _mapped_colors)
    {
        QListWidgetItem* item = new QListWidgetItem( QString::fromStdString( it.first) );
        item->setForeground( it.second );
        ui->listWidget->addItem( item );
    }

    _color_wheel = new  color_widgets::ColorWheel(this);
    ui->verticalLayoutRight->insertWidget(0, _color_wheel );
    _color_wheel->setMinimumWidth(150);
    _color_wheel->setMinimumHeight(150);

    _color_preview = new  color_widgets::ColorPreview(this);
    ui->verticalLayoutRight->insertWidget(1, _color_preview );
    _color_preview->setMinimumWidth(150);
    _color_preview->setMinimumHeight(100);

    connect(_color_wheel,   &color_widgets::ColorWheel::colorChanged,
            _color_preview, &color_widgets::ColorPreview::setColor );

    connect(_color_wheel,   &color_widgets::ColorWheel::colorChanged,
            this, &CurveColorPick::on_colorChanged );
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

void CurveColorPick::on_pushButtonUndo_clicked()
{
    for(int row = 0; row < ui->listWidget->count(); row++)
    {
        QListWidgetItem *item = ui->listWidget->item(row);
        const std::string name = item->text().toStdString();
        const QColor& color = _mapped_colors.find(name)->second;

        item->setForeground( color );
        emit changeColor( item->text(), color );
    }
    QListWidgetItem *item = ui->listWidget->currentItem();
    QColor current_color = item->foreground().color();
    _color_wheel->setColor(current_color);

}

void CurveColorPick::on_listWidget_itemClicked(QListWidgetItem *item)
{
    _color_wheel->setColor( item->foreground().color() );
}

void CurveColorPick::on_colorChanged(QColor color)
{
    QListWidgetItem *item = ui->listWidget->currentItem();
    if( color != item->foreground().color())
    {
        _any_modified = true;
        item->setForeground( color );
        emit changeColor( item->text(), color );
    }
}

