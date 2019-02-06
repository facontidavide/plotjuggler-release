#include "aboutdialog.h"
#include "ui_aboutdialog.h"
#include <QApplication>

AboutDialog::AboutDialog(QWidget *parent) :
  QDialog(parent),
  ui(new Ui::AboutDialog)
{
  ui->setupUi(this);
  ui->label_version->setText( QApplication::applicationVersion() );
  this->setWindowTitle("About...");
}

AboutDialog::~AboutDialog()
{
  delete ui;
}
