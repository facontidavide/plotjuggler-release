#ifndef ROSTOPICSELECTOR_H
#define ROSTOPICSELECTOR_H

#include <QDialog>
#include <QStringList>
#include "../ruleloaderwidget.h"

namespace Ui {
class RosTopicSelector;
}

class RosTopicSelector : public QDialog
{
    Q_OBJECT

public:
    explicit RosTopicSelector(QWidget *parent = 0);
    ~RosTopicSelector();

    bool connectionSuccesfull();

    QStringList getSelectedTopicsList();

    const RosIntrospection::SubstitutionRuleMap& getLoadedRules() const;

private slots:
    void on_pushButtonRefresh_pressed();

    void on_listTopics_itemSelectionChanged();

    void on_buttonBox_accepted();

    void on_buttonBox_rejected();

private:

    Ui::RosTopicSelector *ui;

    QStringList _selected_topics;

    RuleLoaderWidget* _rule_widget;
};

#endif // ROSTOPICSELECTOR_H
