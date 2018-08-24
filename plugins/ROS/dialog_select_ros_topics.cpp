#include <QFile>
#include <QTextStream>
#include <QSettings>
#include <QFileInfo>
#include <QDir>
#include <QFileDialog>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QSettings>
#include <QHeaderView>
#include <QDebug>
#include <QMessageBox>
#include <QAbstractItemView>

#include "dialog_select_ros_topics.h"
#include "rule_editing.h"
#include "ui_dialog_select_ros_topics.h"


DialogSelectRosTopics::DialogSelectRosTopics(const std::vector<std::pair<QString, QString>>& topic_list,
                                             QStringList default_selected_topics,
                                             QWidget *parent) :
    QDialog(parent),
    ui(new Ui::dialogSelectRosTopics),
    _default_selected_topics(default_selected_topics),
    _select_all(QKeySequence(Qt::CTRL + Qt::Key_A), this),
    _deselect_all(QKeySequence(Qt::CTRL + Qt::SHIFT + Qt::Key_A), this)
{

    auto flags = this->windowFlags();
    this->setWindowFlags( flags | Qt::WindowStaysOnTopHint);

    ui->setupUi(this);

    QSettings settings( "IcarusTechnology", "PlotJuggler");
    ui->checkBoxEnableRules->setChecked(     settings.value("DialogSelectRosTopics.enableRules", true ).toBool());
    ui->checkBoxUseHeaderStamp->setChecked(  settings.value("DialogSelectRosTopics.useHeaderStamp", true ).toBool());
    ui->spinBoxArraySize->setValue( settings.value( "DialogSelectRosTopics.maxArraySize", 100).toInt() );
    restoreGeometry(settings.value("DialogSelectRosTopics.geometry").toByteArray());

    if( _default_selected_topics.isEmpty())
    {
        QString default_topics = settings.value("DialogSelectRosTopics.selectedItems", "" ).toString();
        _default_selected_topics = default_topics.split(' ', QString::SkipEmptyParts);
    }

    QStringList labels;
    labels.push_back("Topic name");
    labels.push_back("Datatype");

    ui->listRosTopics->setHorizontalHeaderLabels(labels);
    ui->listRosTopics->verticalHeader()->setVisible(false);

    updateTopicList(topic_list);

    if( ui->listRosTopics->rowCount() == 1)
    {
        ui->listRosTopics->selectRow(0);
    }
    ui->listRosTopics->setFocus();

    _select_all.setContext(Qt::WindowShortcut);
    _deselect_all.setContext(Qt::WindowShortcut);

    connect( &_select_all, &QShortcut::activated,
             ui->listRosTopics, &QAbstractItemView::selectAll );

    connect( &_deselect_all, &QShortcut::activated,
             ui->listRosTopics, &QAbstractItemView::clearSelection );

}

void DialogSelectRosTopics::updateTopicList(std::vector<std::pair<QString, QString> > topic_list )
{
    std::set<QString> newly_added;

    // add if not present
    for (const auto& it: topic_list)
    {
        const QString& topic_name = it.first ;
        const QString& type_name  = it.second ;

        bool found = false;
        for(int r=0; r < ui->listRosTopics->rowCount(); r++ )
        {
            const QTableWidgetItem *item = ui->listRosTopics->item(r,0);
            if( item->text() == topic_name){
                found = true;
                break;
            }
        }

        if( !found )
        {
            int new_row = ui->listRosTopics->rowCount();
            ui->listRosTopics->setRowCount( new_row+1 );

            // order IS important, don't change it
            ui->listRosTopics->setItem(new_row, 1, new QTableWidgetItem( type_name ));
            ui->listRosTopics->setItem(new_row, 0, new QTableWidgetItem( topic_name ));
            newly_added.insert(topic_name);
        }
    }

    ui->listRosTopics->horizontalHeader()->setSectionResizeMode(0, QHeaderView::Stretch);
    ui->listRosTopics->horizontalHeader()->setSectionResizeMode(1, QHeaderView::ResizeToContents);
    ui->listRosTopics->sortByColumn(0, Qt::AscendingOrder);
    //----------------------------------------------

    QModelIndexList selection = ui->listRosTopics->selectionModel()->selectedRows();

    for(int row=0; row < ui->listRosTopics->rowCount(); row++ )
    {
        const QTableWidgetItem *item = ui->listRosTopics->item(row,0);
        QString topic_name = item->text();

        if(newly_added.count(topic_name) &&  _default_selected_topics.contains( topic_name ) )
        {
            bool selected = false;
            for (const auto& selected_item: selection)
            {
                if( selected_item.row() == row)
                {
                    selected = true;
                    break;
                }
            }
            if( !selected ){
                ui->listRosTopics->selectRow(row);
            }
        }
    }

}


DialogSelectRosTopics::~DialogSelectRosTopics()
{
    delete ui;
}

QStringList DialogSelectRosTopics::getSelectedItems()
{
  return _topic_list;
}

int DialogSelectRosTopics::maxArraySize() const
{
  return ui->spinBoxArraySize->value();
}


const QCheckBox *DialogSelectRosTopics::checkBoxUseHeaderStamp()
{
    return ui->checkBoxUseHeaderStamp;
}

const QCheckBox* DialogSelectRosTopics::checkBoxUseRenamingRules()
{
    return ui->checkBoxEnableRules;
}

QString DialogSelectRosTopics::prefix()
{
    return (ui->checkBoxPrefix->isChecked()) ? ui->linePrefix->text() : QString();
}


void DialogSelectRosTopics::on_buttonBox_accepted()
{
    QModelIndexList selected_indexes = ui->listRosTopics->selectionModel()->selectedIndexes();
    QString selected_topics;

    foreach(QModelIndex index, selected_indexes)
    {
        if(index.column() == 0){
            _topic_list.push_back( index.data(Qt::DisplayRole).toString() );
            selected_topics.append( _topic_list.back() ).append(" ");
        }
    }
    QSettings settings( "IcarusTechnology", "PlotJuggler");
    settings.setValue("DialogSelectRosTopics.enableRules",    ui->checkBoxEnableRules->isChecked() );
    settings.setValue("DialogSelectRosTopics.useHeaderStamp", ui->checkBoxUseHeaderStamp->isChecked() );
    settings.setValue("DialogSelectRosTopics.geometry", saveGeometry());
    settings.setValue("DialogSelectRosTopics.selectedItems", selected_topics );
    settings.setValue("DialogSelectRosTopics.maxArraySize", ui->spinBoxArraySize->value());
}

void DialogSelectRosTopics::on_listRosTopics_itemSelectionChanged()
{
    QModelIndexList indexes = ui->listRosTopics->selectionModel()->selectedIndexes();

    ui->buttonBox->setEnabled( indexes.size() > 0) ;
}


void DialogSelectRosTopics::on_checkBoxEnableRules_toggled(bool checked)
{
    ui->pushButtonEditRules->setEnabled( checked );
}

void DialogSelectRosTopics::on_pushButtonEditRules_pressed()
{
    RuleEditing* rule_editing = new RuleEditing(this);
    rule_editing->exec();
}

void DialogSelectRosTopics::closeEvent(QCloseEvent *event)
{
    QSettings settings( "IcarusTechnology", "PlotJuggler");
    settings.setValue("DialogSelectRosTopics.geometry", saveGeometry());
}

nonstd::optional<double> FlatContainerContainHeaderStamp(const RosIntrospection::RenamedValues& renamed_value)
{
    const char* ID = "/header/stamp";
    const int renamed_count = renamed_value.size();
    const int OFF = strlen(ID);

    // cache the previous result
    static std::map<const RosIntrospection::RenamedValues*,int> first_indexes;

    int first_index = first_indexes[&renamed_value];

    if( first_index >= 0 && first_index < renamed_count)
    {
        const auto& field_name = renamed_value[first_index].first;
        if( field_name.size() > OFF &&
            strcmp( &field_name.data()[ field_name.size() -OFF], ID) == 0)
        {
            return renamed_value[first_index].second.convert<double>();
        }
    }

    for(int i=0; i< renamed_count; i++ )
    {
        if( i == first_index ) continue;

        const auto& field_name = renamed_value[i].first;
        if( field_name.size() > OFF &&
            strcmp( &field_name.data()[ field_name.size() -OFF], ID) == 0)
        {
            first_indexes[&renamed_value] = i;
            return renamed_value[i].second.convert<double>();
        }
    }
    return nonstd::optional<double>();
}

void DialogSelectRosTopics::on_checkBoxPrefix_toggled(bool checked)
{
    ui->linePrefix->setEnabled(checked);
}

void DialogSelectRosTopics::on_maximumSizeHelp_pressed()
{
    QMessageBox msgBox;
    msgBox.setWindowTitle("Help");
    msgBox.setText("Maximum Size of Arrays:\n\n"
                   "Arrays which size is larger than the maximum, are simple skipped. "
                   "Nevertheless, they are still stored in memory and they can be republished!\n\n"
                   "This parameter is used to prevent the user from loading huge arrays, "
                   "such as images, pointclouds, maps, etc.\n"
                   "The term 'array' refers to the array in a message field,\n\n"
                   " See http://wiki.ros.org/msg.\n\n"
                   "This is NOT about the duration of a time series!\n\n"
                   "MOTIVATION: pretend that a user tries to load a RGB image, which probably contains "
                   "a few millions pixels.\n"
                   "Plotjuggler would naively create a single time series for each pixel of the image! "
                   "That makes no sense, of course, and it would probably freeze your system.\n"
                   );
    msgBox.exec();
}


