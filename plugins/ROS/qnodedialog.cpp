#include "qnodedialog.h"
#include "ui_qnodedialog.h"
#include <QSettings>
#include <QMessageBox>

void showNoMasterMessage()
{
  QMessageBox msgBox;
  msgBox.setText("Could not connect to the ros master.");
  msgBox.exec();
}

std::string getDefaultMasterURI()
{
    if(  qgetenv("ROS_MASTER_URI").isEmpty() )
    {
      QMessageBox msgBox;
      msgBox.setText("WARNINGS: the ROS_MASTER_URI is not defined in your environment\n"
                     "Using the default value [http://localhost:11311]\n");
      msgBox.exec();
      return "http://localhost:11311";
    }
    else{
      auto master_uri = ( qgetenv("ROS_MASTER_URI"));
      return std::string( master_uri.data() );
    }
}

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

bool QNodeDialog::Connect(const std::string& ros_master_uri,
                          const std::string& hostname)
{
  std::map<std::string,std::string> remappings;
  remappings["__master"] = ros_master_uri;
  remappings["__hostname"] = hostname;

  ros::init(remappings, "PlotJugglerStreamingListener");
  bool connected = ros::master::check();
  if ( ! connected ) {
    showNoMasterMessage();
  }
  return connected;
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

  if( ui->checkBoxUseEnvironment->isChecked() )
  {
    ros::init(init_argc,init_argv,"PlotJugglerStreamingListener");
    if ( ! ros::master::check() ) {
      showNoMasterMessage();
      return;
    }
  }
  else{
    std::string ros_master_uri = ui->lineEditMaster->text().toStdString();
    std::string hostname       = ui->lineEditHost->text().toStdString();
    QNodeDialog::Connect(ros_master_uri, hostname);
  }
  ui->pushButtonConnect->setEnabled(false);
  ui->pushButtonDisconnect->setEnabled(true);

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
      // TODO I need to decide when to enable this
      //QNodeDialog dialog;
      //dialog.exec();

      std::string master_uri = getDefaultMasterURI();
      QNodeDialog::Connect(master_uri, "localhost" );
    }

    if( ros::isInitialized()  )
    {
      node_ptr.reset( new ros::NodeHandle );
    }
    else{
      showNoMasterMessage();
      node_ptr.reset();
    }
  }
  return node_ptr;
}

