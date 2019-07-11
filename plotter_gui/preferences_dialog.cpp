#include "preferences_dialog.h"
#include "ui_preferences_dialog.h"
#include <QSettings>

PreferencesDialog::PreferencesDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::PreferencesDialog)
{
    ui->setupUi(this);
    QSettings settings;
    QString theme = settings.value("Preferences::theme").toString();
    if( theme == "style_dark")
    {
        ui->comboBoxTheme->setCurrentIndex(1);
    }
    else{
        ui->comboBoxTheme->setCurrentIndex(0);
    }
}

PreferencesDialog::~PreferencesDialog()
{
    delete ui;
}

void PreferencesDialog::on_buttonBox_accepted()
{
    QSettings settings;
    settings.setValue("Preferences::theme",
                      ui->comboBoxTheme->currentIndex() == 1 ? "style_dark" : "style_light");

}
