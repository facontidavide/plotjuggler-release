#include "qnodedialog.h"
#include "ui_qnodedialog.h"
#include <QSettings>
#include <QMessageBox>

QNodeDialog::QNodeDialog( QWidget *parent) :
  QDialog(parent),
  ui(new Ui::QNodeDialog)
{
  ui->setupUi(this);

  QSettings settings( "IcarusTechnology", "PlotJuggler");

  auto master_ui = settings.value("QNode.master_uri", tr("http://localhost:11311")).toString();
  auto host_ip   = settings.value("QNode.host_ip", tr("localhost")).toString();

  ui->lineEditMaster->setText( master_ui );
  ui->lineEditHost->setText( host_ip );
}


QNodeDialog::~QNodeDialog()
{
  QSettings settings( "IcarusTechnology", "PlotJuggler");
  settings.setValue ("QNode.master_uri",  ui->lineEditMaster->text() );
  settings.setValue("QNode.host_ip",      ui->lineEditHost->text() );
  delete ui;
}

void QNodeDialog::on_pushButtonConnect_pressed()
{
  int init_argc = 0;
  char** init_argv = NULL;

  if(ui->checkBoxUseEnvironment->isChecked() &&  qgetenv("ROS_MASTER_URI").isEmpty() )
  {
    QMessageBox msgBox;
    msgBox.setText("ROS_MASTER_URI is not defined in the environment.\n"
                   "Either type the following or (preferrably) add this to your ~/.bashrc \n"
                   "file in order set up your local machine as a ROS master:\n\n"
                   "    export ROS_MASTER_URI=http://localhost:11311\n\n"
                   "Then, type 'roscore' in another shell to actually launch the master program.");
    msgBox.exec();
    return;
  }

  if( ui->checkBoxUseEnvironment->isChecked() )
  {
    ros::init(init_argc,init_argv,"PlotJugglerStreamingListener");
    if ( ! ros::master::check() ) {
      showNoMasterMessage();
      return;
    }
  }
  else{
    std::map<std::string,std::string> remappings;
    remappings["__master"]   = ui->lineEditMaster->text().toStdString();
    remappings["__hostname"] = ui->lineEditHost->text().toStdString();

    ros::init(remappings, "PlotJugglerStreamingListener");
    if ( ! ros::master::check() ) {
      showNoMasterMessage();
      return;
    }
  }
  ui->pushButtonConnect->setEnabled(false);
  ui->pushButtonDisconnect->setEnabled(true);


}

void QNodeDialog::showNoMasterMessage()
{
  QMessageBox msgBox;
  msgBox.setText("Couldn't find the ros master.");
  msgBox.exec();
}

void QNodeDialog::on_pushButtonDisconnect_pressed()
{
  ui->pushButtonConnect->setEnabled(true);
  ui->pushButtonDisconnect->setEnabled(false);
}

void QNodeDialog::on_checkBoxUseEnvironment_toggled(bool checked)
{
  ui->lineEditMaster->setEnabled( !checked );
  ui->lineEditHost->setEnabled( !checked );
}

ros::NodeHandlePtr getGlobalRosNode()
{
  static ros::NodeHandlePtr node_ptr;

  if( !node_ptr )
  {
    if( !ros::isInitialized() )
    {
      QNodeDialog dialog;
      dialog.exec();
    }

    if( ros::isInitialized()  )
    {
      node_ptr.reset( new ros::NodeHandle );
    }
  }
  return node_ptr;
}

