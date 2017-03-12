#ifndef CURVE_SELECTOR_H
#define CURVE_SELECTOR_H

#include <QWidget>
#include <QAction>
#include <QListWidget>
#include <QTableWidget>
#include <QMouseEvent>

namespace Ui {
class FilterableListWidget;
}


class FilterableListWidget : public QWidget
{
    Q_OBJECT

public:
    explicit FilterableListWidget(QWidget *parent = 0);
    ~FilterableListWidget();

    int rowCount() const;

    void clear();

    void addItem(QTableWidgetItem *item);

    void addItems(const QStringList& index_list);

    QList<int> findRowsByName(const QString& text) const;

    const QTableWidget *table() const;

    QTableWidget *table();

    void updateFilter();

private slots:


    void on_radioContains_toggled(bool checked);

    void on_radioRegExp_toggled(bool checked);

    void on_checkBoxCaseSensitive_toggled(bool checked);

    void on_lineEdit_textChanged(const QString &search_string);

    void on_pushButtonSettings_toggled(bool checked);

    void on_checkBoxHideSecondColumn_toggled(bool checked);

private:

    Ui::FilterableListWidget *ui;

    QPoint _drag_start_pos;

    bool eventFilter(QObject *object, QEvent *event);

signals:

    void hiddenItemsChanged();

};

#endif // CURVE_SELECTOR_H
