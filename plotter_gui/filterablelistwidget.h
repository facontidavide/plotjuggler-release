#ifndef CURVE_SELECTOR_H
#define CURVE_SELECTOR_H

#include <QWidget>
#include <QAction>
#include <QListWidget>
#include <QTableWidget>
#include <QMouseEvent>
#include <QStandardItemModel>


namespace Ui {
class FilterableListWidget;
}

class TreeModelCompleter;

class FilterableListWidget : public QWidget
{
    Q_OBJECT

public:
    explicit FilterableListWidget(QWidget *parent = 0);
    ~FilterableListWidget();

    int rowCount() const;

    void clear();

    void addItem(const QString& item_name, bool sort_columns);

    void sortColumns();

    QList<int> findRowsByName(const QString& text) const;

    void removeRow(int row);

    void updateFilter();

    const QTableWidget * getTable() const;

    virtual void keyPressEvent(QKeyEvent * event) override;

private slots:

    void on_radioContains_toggled(bool checked);

    void on_radioRegExp_toggled(bool checked);

    void on_checkBoxCaseSensitive_toggled(bool checked);

    void on_lineEdit_textChanged(const QString &search_string);

    void on_pushButtonSettings_toggled(bool checked);

    void on_checkBoxHideSecondColumn_toggled(bool checked);

    void removeSelectedCurves();

    void on_radioPrefix_toggled(bool checked);

private:

    Ui::FilterableListWidget *ui;

    QPoint _drag_start_pos;

    bool _newX_modifier;

    QStandardItemModel* _tree_model;

    TreeModelCompleter* _completer;

    bool eventFilter(QObject *object, QEvent *event);

    void updateTreeModel();
    
    void addToCompletionTree(QTableWidgetItem *item);

signals:

    void hiddenItemsChanged();

    void deleteCurve(QString curve_name);

};

#endif // CURVE_SELECTOR_H
