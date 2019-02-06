#ifndef DIALOG_SELECT_ROS_TOPICS_H
#define DIALOG_SELECT_ROS_TOPICS_H

#include <QDialog>
#include <QString>
#include <QFile>
#include <QStringList>
#include <QCheckBox>
#include <QShortcut>
#include "PlotJuggler/optional.hpp"
#include <ros_type_introspection/ros_introspection.hpp>

namespace Ui {
class dialogSelectRosTopics;
}

class DialogSelectRosTopics : public QDialog
{
    Q_OBJECT

public:

    explicit DialogSelectRosTopics(const std::vector<std::pair<QString,QString>>& topic_list,
                                   QStringList default_selected_topics,
                                   QWidget *parent = nullptr);

    ~DialogSelectRosTopics() override;

    QStringList getSelectedItems();

    int maxArraySize() const;

    const QCheckBox *checkBoxUseRenamingRules();

    bool discardEntireArrayIfTooLarge();

    QString prefix();

public slots:

    void updateTopicList(std::vector<std::pair<QString,QString>> topic_list);

private slots:

    void on_buttonBox_accepted();

    void on_listRosTopics_itemSelectionChanged();

    void on_checkBoxEnableRules_toggled(bool checked);

    void on_pushButtonEditRules_pressed();

    void on_checkBoxPrefix_toggled(bool checked);

    void on_maximumSizeHelp_pressed();

    void on_lineEditFilter_textChanged(const QString &search_string);

    void on_spinBoxArraySize_valueChanged(int value);

private:

    void closeEvent(QCloseEvent *event) override;

    QStringList _topic_list;
    QStringList _default_selected_topics;

    QShortcut _select_all;
    QShortcut _deselect_all;

    Ui::dialogSelectRosTopics *ui;

};

// SOME SFINAE magic....

template <typename T>
class has_setMaxArrayPolicy
{
    typedef char one;
    typedef long two;

    template <typename C> static one test( decltype(&C::setMaxArrayPolicy) ) ;
    template <typename C> static two test(...);

public:
    enum { value = sizeof(test<T>(nullptr)) == sizeof(char) };
};



nonstd::optional<double> FlatContainerContainHeaderStamp(const RosIntrospection::RenamedValues& flat_container);

template<class T, typename std::enable_if< has_setMaxArrayPolicy<T>::value, int>::type = 0>
inline bool setMaxArrayPolicy(T* parser, bool policy)
{
    parser->setMaxArrayPolicy( policy );
    return true;
}

template<class T, typename std::enable_if< !has_setMaxArrayPolicy<T>::value, int>::type = 0>
inline bool setMaxArrayPolicy(T* , bool )
{
    return false;
}


#endif // DIALOG_SELECT_ROS_TOPICS_H
