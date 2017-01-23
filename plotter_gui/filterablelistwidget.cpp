#include "filterablelistwidget.h"
#include "ui_filterablelistwidget.h"

#include <QLayoutItem>
#include <QMenu>
#include <QSettings>


FilterableListWidget::FilterableListWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::FilterableListWidget)
{
    ui->setupUi(this);

    for( int i=0; i< ui->gridLayoutSettings->count(); i++)
    {
        QLayoutItem* item = ui->gridLayoutSettings->itemAt(i);
        if(item)
        {
            QWidget* widget = item->widget();
            if(widget) {
                widget->setMaximumHeight( 0 );
                widget->setVisible( false );
            }
        }
    }
}

FilterableListWidget::~FilterableListWidget()
{
    delete ui;
}

int FilterableListWidget::count() const
{
    return ui->listWidget->count();
}

void FilterableListWidget::clear()
{
    list()->clear();
    ui->labelNumberDisplayed->setText( "0 of 0");
}

void FilterableListWidget::addItem(QListWidgetItem *item)
{
    list()->addItem(item);
    on_lineEdit_textChanged( ui->lineEdit->text() );
}

void FilterableListWidget::addItems(const QStringList &index_list)
{
    list()->addItems(index_list);
    on_lineEdit_textChanged( ui->lineEdit->text() );
}

QList<QListWidgetItem *> FilterableListWidget::findItems(const QString &text) const
{
    return list()->findItems( text, Qt::MatchExactly);
}

const QListWidget *FilterableListWidget::list() const
{
    return ui->listWidget;
}

QListWidget *FilterableListWidget::list()
{
    return ui->listWidget;
}


void FilterableListWidget::on_radioContains_toggled(bool checked)
{
    if(checked)
    {
        ui->radioRegExp->setChecked( false);
        on_lineEdit_textChanged( ui->lineEdit->text() );
    }
}

void FilterableListWidget::on_radioRegExp_toggled(bool checked)
{
    if(checked)
    {
        ui->radioContains->setChecked( false);
        on_lineEdit_textChanged( ui->lineEdit->text() );
    }
}

void FilterableListWidget::on_checkBoxCaseSensitive_toggled(bool checked)
{
    on_lineEdit_textChanged( ui->lineEdit->text() );
}


void FilterableListWidget::on_lineEdit_textChanged(const QString &search_string)
{
    int item_count = ui->listWidget->count();
    int visible_count = 0;

    Qt::CaseSensitivity cs = Qt::CaseInsensitive;
    if( ui->checkBoxCaseSensitive->isChecked())
    {
        cs = Qt::CaseSensitive;
    }
    QRegExp regexp( search_string,  cs, QRegExp::Wildcard );
    QRegExpValidator v(regexp, 0);

    for (int i=0; i< ui->listWidget->count(); i++)
    {
        QListWidgetItem* item = ui->listWidget->item(i);
        QString name = item->text();
        int pos = 0;
        bool toHide = false;

        if( ui->radioRegExp->isChecked())
            toHide = v.validate( name, pos ) != QValidator::Acceptable;
        else{
            QStringList items = search_string.split(' ');
            for (int i=0; i< items.size(); i++)
            {
                if( name.contains(items[i], cs) == false )
                {
                    toHide = true;
                }
            }
        }
        if( !toHide ) visible_count++;

        item->setHidden( toHide );
    }
    ui->labelNumberDisplayed->setText( QString::number( visible_count ) + QString(" of ") + QString::number( item_count ) )   ;
}

void FilterableListWidget::on_pushButtonSettings_toggled(bool checked)
{
    for( int i=0; i< ui->gridLayoutSettings->count(); i++)
    {
        QLayoutItem* item = ui->gridLayoutSettings->itemAt(i);
        if(item)
        {
            QWidget* widget = item->widget();
            if(widget)
            {
                widget->setMaximumHeight( checked ? 25:0 );
                widget->setVisible( checked );

            }
        }
    }
}
