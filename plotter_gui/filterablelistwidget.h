#ifndef CURVE_SELECTOR_H
#define CURVE_SELECTOR_H

#include <QWidget>
#include <QAction>
#include <QListWidget>
#include <QMouseEvent>
#include <QStandardItemModel>
#include <QTableView>

#include "tree_completer.h"

class CustomSortedTableItem;

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

    void addItem(const QString& item_name);

    void refreshColumns();

    int findRowByName(const std::string &text) const;

    void removeRow(int row);

    void rebuildEntireList(const std::vector<std::string> &names);

    void updateFilter();

    QStandardItemModel *getTable() const
    {
        return _model;
    }

    QTableView* getView() const;

    bool is2ndColumnHidden() const
    {
        return getView()->isColumnHidden(1);
    }

    virtual void keyPressEvent(QKeyEvent * event) override;

private slots:

    void on_radioContains_toggled(bool checked);

    void on_radioRegExp_toggled(bool checked);

    void on_radioPrefix_toggled(bool checked);

    void on_checkBoxCaseSensitive_toggled(bool checked);

    void on_lineEdit_textChanged(const QString &search_string);

    void on_pushButtonSettings_toggled(bool checked);

    void on_checkBoxHideSecondColumn_toggled(bool checked);

    void removeSelectedCurves();

private:

    Ui::FilterableListWidget *ui;

    QPoint _drag_start_pos;

    bool _newX_modifier, _dragging;

    TreeModelCompleter* _completer;

    bool eventFilter(QObject *object, QEvent *event);

    void updateTreeModel();
    
    std::vector<std::string> getNonHiddenSelectedRows();

    bool _completer_need_update;

    QStandardItemModel* _model;


signals:

    void hiddenItemsChanged();

    void deleteCurves(const std::vector<std::string>& curve_names);

};

#endif // CURVE_SELECTOR_H
