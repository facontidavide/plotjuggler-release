#ifndef CURVE_SELECTOR_H
#define CURVE_SELECTOR_H

#include <QWidget>
#include <QAction>
#include <QListWidget>

namespace Ui {
class FilterableListWidget;
}

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

    const QListWidget* list() const;
    QListWidget* list();
};

#endif // CURVE_SELECTOR_H
