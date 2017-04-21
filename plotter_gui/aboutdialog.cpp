#include "aboutdialog.h"
#include "ui_aboutdialog.h"

AboutDialog::AboutDialog(QWidget *parent) :
  QDialog(parent),
  ui(new Ui::AboutDialog)
{
  ui->setupUi(this);

  QString version = QString("Version: %1.%2.%3")
          .arg(PJ_MAJOR_VERSION)
          .arg(PJ_MINOR_VERSION)
          .arg(PJ_PATCH_VERSION);

  ui->label_version->setText(version);

}

AboutDialog::~AboutDialog()
{
  delete ui;
}
