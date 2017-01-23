#ifndef QNODEDIALOG_H
#define QNODEDIALOG_H

#include <QDialog>
#include <ros/ros.h>

namespace Ui {
class QNodeDialog;
}

class QNodeDialog : public QDialog
{
  Q_OBJECT

public:
  ~QNodeDialog();
  explicit QNodeDialog(QWidget *parent = 0);

private slots:
  void on_checkBoxUseEnvironment_toggled(bool checked);

  void on_pushButtonDisconnect_pressed();

  void on_pushButtonConnect_pressed();

private:
  Ui::QNodeDialog *ui;

  void showNoMasterMessage();

};

ros::NodeHandlePtr getGlobalRosNode();

#endif // QNODEDIALOG_H
