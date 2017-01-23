#ifndef DIALOG_SELECT_ROS_TOPICS_H
#define DIALOG_SELECT_ROS_TOPICS_H

#include <QDialog>
#include <QString>
#include <QFile>
#include <QStringList>
#include "../ruleloaderwidget.h"

namespace Ui {
class dialogSelectRosTopics;
}

class DialogSelectRosTopics : public QDialog
{
    Q_OBJECT

public:
    explicit DialogSelectRosTopics(QStringList topic_list, QWidget *parent = 0);
    ~DialogSelectRosTopics();

    QStringList getSelectedItems();

    const RosIntrospection::SubstitutionRuleMap& getLoadedRules() const;

private slots:

    void on_buttonBox_accepted();

    void on_listRosTopics_itemSelectionChanged();


private:

    QStringList _topic_list;

    Ui::dialogSelectRosTopics *ui;

    RuleLoaderWidget* _rule_widget;
};

#endif // DIALOG_SELECT_ROS_TOPICS_H
