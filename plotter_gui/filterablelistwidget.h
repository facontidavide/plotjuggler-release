#ifndef CURVE_SELECTOR_H
#define CURVE_SELECTOR_H

#include <QWidget>
#include <QAction>
#include <QListWidget>
#include <QMouseEvent>

namespace Ui {
class FilterableListWidget;
}
/*
class CustomListWidget: public QListWidget
{
    Q_OBJECT
public:
    CustomListWidget(QWidget *parent = 0): QListWidget(parent) {}
private:

    virtual void mouseMoveEvent(QMouseEvent *event) override
    {
        if (event->buttons() & Qt::LeftButton) {
            int distance = (event->pos() - event).manhattanLength();
            if (distance >= QApplication::startDragDistance())
                performDrag();
        }
        QListWidget::mouseMoveEvent(event);
    }

    void performDrag()
    {
        //const QMimeData *mimeData = event->mimeData();
        //Add(mimeData->text()); // probably the slot you defined for your Add button

        QString resourceName (":/icons/resources/office_chart_lines.png");
        QPixmap pixmap (resourceName);
        QCursor cursor (pixmap);
        setCursor(cursor);
        QListWidget::dragEnterEvent(event);
    }

};*/

class FilterableListWidget : public QWidget
{
    Q_OBJECT

public:
    explicit FilterableListWidget(QWidget *parent = 0);
    ~FilterableListWidget();

    //
    int count() const;

    void clear();

    void addItem(QListWidgetItem* item);

    void addItems(const QStringList& index_list);

    QList<QListWidgetItem*> findItems(const QString& text) const;

private slots:

    void on_radioContains_toggled(bool checked);

    void on_radioRegExp_toggled(bool checked);

    void on_checkBoxCaseSensitive_toggled(bool checked);

    void on_lineEdit_textChanged(const QString &search_string);

    void on_pushButtonSettings_toggled(bool checked);

private:
    Ui::FilterableListWidget *ui;
    QPoint _drag_start_pos;

    const QListWidget* list() const;
    QListWidget* list();

    bool eventFilter(QObject *object, QEvent *event);
};

#endif // CURVE_SELECTOR_H
