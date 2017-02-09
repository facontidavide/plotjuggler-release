#include "aboutdialog.h"
#include "ui_aboutdialog.h"

AboutDialog::AboutDialog(QWidget *parent) :
  QDialog(parent),
  ui(new Ui::AboutDialog)
{
  ui->setupUi(this);

  QString version = QString("Version: ") +
      QString::number(PJ_MAJOR_VERSION) + QString(".") +
      QString::number(PJ_MINOR_VERSION) + QString(".") +
      QString::number(PJ_PATCH_VERSION);

  ui->label_version->setText(version);

}

AboutDialog::~AboutDialog()
{
  delete ui;
}
