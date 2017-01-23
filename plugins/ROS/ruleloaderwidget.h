#ifndef RULELOADERWIDGET_H
#define RULELOADERWIDGET_H

#include <QWidget>
#include <QString>
#include <QFile>
#include <ros_type_introspection/renamer.hpp>

namespace Ui {
class RuleLoaderWidget;
}


class RuleLoaderWidget : public QWidget
{
  Q_OBJECT

public:


  explicit RuleLoaderWidget(QWidget *parent = 0);
  ~RuleLoaderWidget();

  const RosIntrospection::SubstitutionRuleMap& getLoadedRules() const;

private slots:

  void on_pushButtonLoadOther_pressed();

  void on_checkBoxEnableSubstitution_toggled(bool checked);

private:
  Ui::RuleLoaderWidget *ui;

  void readRuleFile(QFile& file);

  RosIntrospection::SubstitutionRuleMap _loaded_rules;
};

#endif // RULELOADERWIDGET_H
