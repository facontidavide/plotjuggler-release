#include "filterablelistwidget.h"
#include "ui_filterablelistwidget.h"
#include <QDebug>
#include <QLayoutItem>
#include <QMenu>
#include <QSettings>
#include <QDrag>
#include <QMimeData>
#include <QHeaderView>

FilterableListWidget::FilterableListWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::FilterableListWidget)
{
    ui->setupUi(this);
    ui->tableWidget->viewport()->installEventFilter( this );

   table()->horizontalHeader()->setSectionResizeMode(0, QHeaderView::Stretch);
   table()->horizontalHeader()->setSectionResizeMode(1, QHeaderView::ResizeToContents);
   table()->horizontalHeader()->resizeSection(1, 120);

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
    return table()->rowCount();
}

void FilterableListWidget::clear()
{
    table()->clear();
    ui->labelNumberDisplayed->setText( "0 of 0");
}

void FilterableListWidget::addItem(QTableWidgetItem *item)
{
    int row = count();
    table()->setRowCount(row+1);
    table()->setItem(row, 0, item);
    table()->setItem(row, 1, new QTableWidgetItem( QString::number(row)) );
    on_lineEdit_textChanged( ui->lineEdit->text() );
    table()->resizeColumnToContents(0);
    table()->resizeColumnToContents(1);
}


QList<int>
FilterableListWidget::findRowsByName(const QString &text) const
{
    QList<int> output;
    QList<QTableWidgetItem*> item_list = table()->findItems( text, Qt::MatchExactly);
    for(QTableWidgetItem* item : item_list)
    {
        if(item->column() == 0) {
            output.push_back( item->row() );
        }
    }
    return output;
}

const QTableWidget *FilterableListWidget::table() const
{
    return ui->tableWidget;
}

QTableWidget *FilterableListWidget::table()
{
    return ui->tableWidget;
}

bool FilterableListWidget::eventFilter(QObject *object, QEvent *event)
{
   // qDebug() <<event->type();
    if(event->type() == QEvent::MouseButtonPress)
    {
        QMouseEvent *mouse_event = static_cast<QMouseEvent*>(event);
        if(mouse_event->button() == Qt::LeftButton )
        {
            if(mouse_event->modifiers() == Qt::ControlModifier)
            {
                //TODO
            }
            else{
                _drag_start_pos = mouse_event->pos();
            }
        }
    }
    else if(event->type() == QEvent::MouseMove)
    {
        QMouseEvent *mouse_event = static_cast<QMouseEvent*>(event);
        double distance_from_click = (mouse_event->pos() - _drag_start_pos).manhattanLength();

        if ((mouse_event->buttons() == Qt::LeftButton) &&
             distance_from_click >= QApplication::startDragDistance())
        {
            QDrag *drag = new QDrag(this);
            QMimeData *mimeData = new QMimeData;
            const QString mimeType("curveslist/copy");
            QByteArray mdata;
            QDataStream stream(&mdata, QIODevice::WriteOnly);

            for(QTableWidgetItem* item: table()->selectedItems()) {
                stream << item->text();
            }

            mimeData->setData(mimeType, mdata);
            drag->setMimeData(mimeData);
            drag->exec(Qt::CopyAction | Qt::MoveAction);
        }
    }
    return QWidget::eventFilter(object,event);
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
    int item_count = count();
    int visible_count = 0;

    Qt::CaseSensitivity cs = Qt::CaseInsensitive;
    if( ui->checkBoxCaseSensitive->isChecked())
    {
        cs = Qt::CaseSensitive;
    }
    QRegExp regexp( search_string,  cs, QRegExp::Wildcard );
    QRegExpValidator v(regexp, 0);

    for (int row=0; row< count(); row++)
    {
        QTableWidgetItem* item = table()->item(row,0);
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

    //    table()->setRowHidden(row, toHide );
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

void FilterableListWidget::on_checkBoxHideSecondColumn_toggled(bool checked)
{
    if(checked)
        table()->hideColumn(1);
    else
        table()->showColumn(1);
}
