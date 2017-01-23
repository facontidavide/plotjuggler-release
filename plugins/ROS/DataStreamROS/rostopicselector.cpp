#include <QMessageBox>
#include <QModelIndexList>
#include <QListWidget>
#include <QSettings>
#include <ros/ros.h>
#include <ros/master.h>
#include "rostopicselector.h"
#include "ui_rostopicselector.h"

RosTopicSelector::RosTopicSelector(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::RosTopicSelector)
{
    ui->setupUi(this);

    QSettings settings( "IcarusTechnology", "PlotJuggler");

    _rule_widget = new RuleLoaderWidget(this);
    ui->frame->layout()->addWidget( _rule_widget );
}

RosTopicSelector::~RosTopicSelector()
{
    delete ui;
}

QStringList RosTopicSelector::getSelectedTopicsList()
{
    return _selected_topics;
}


void RosTopicSelector::on_listTopics_itemSelectionChanged()
{
    QModelIndexList indexes = ui->listTopics->selectionModel()->selectedIndexes();
    ui->buttonBox->setEnabled( indexes.count() > 0);
}

void RosTopicSelector::on_buttonBox_accepted()
{
    QListWidget* list = ui->listTopics;
    QModelIndexList indexes = list->selectionModel()->selectedIndexes();

    _selected_topics.clear();
    foreach(QModelIndex index, indexes)
    {
        _selected_topics .append(  index.data(Qt::DisplayRole ).toString() );
    }
    this->accept();
}

void RosTopicSelector::on_buttonBox_rejected()
{
    this->reject();
}



const RosIntrospection::SubstitutionRuleMap &RosTopicSelector::getLoadedRules() const
{
    return _rule_widget->getLoadedRules();
}

void RosTopicSelector::on_pushButtonRefresh_pressed()
{
  ui->listTopics->clear();
  QStringList topic_advertised;

  ros::master::V_TopicInfo topic_infos;
  ros::master::getTopics(topic_infos);
  for (ros::master::TopicInfo topic_info: topic_infos)
  {
      topic_advertised.append( QString( topic_info.name.c_str() )  );
  }
  topic_advertised.sort();
  ui->listTopics->addItems( topic_advertised );
}
