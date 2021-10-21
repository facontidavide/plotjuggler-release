#include "preferences_dialog.h"
#include "ui_preferences_dialog.h"
#include <QSettings>
#include <QDir>
#include <QFileDialog>
#include "PlotJuggler/svg_util.h"

PreferencesDialog::PreferencesDialog(QWidget* parent)
  : QDialog(parent), ui(new Ui::PreferencesDialog)
{
  ui->setupUi(this);
  QSettings settings;
  QString theme = settings.value("Preferences::theme", "light").toString();
  if (theme == "dark")
  {
    ui->comboBoxTheme->setCurrentIndex(1);
  }
  else
  {
    ui->comboBoxTheme->setCurrentIndex(0);
  }

  bool use_plot_color_index =
      settings.value("Preferences::use_plot_color_index", false).toBool();
  bool remember_color = settings.value("Preferences::remember_color", true).toBool();

  ui->checkBoxRememberColor->setChecked(remember_color);
  ui->radioLocalColorIndex->setChecked(use_plot_color_index);
  ui->radioGlobalColorIndex->setChecked(!use_plot_color_index);

  ui->pushButtonAdd->setIcon(LoadSvg(":/resources/svg/add_tab.svg", theme));
  ui->pushButtonRemove->setIcon(LoadSvg(":/resources/svg/trash.svg", theme));

  bool use_separator = settings.value("Preferences::use_separator", true).toBool();
  ui->checkBoxSeparator->setChecked(use_separator);

  bool use_opengl = settings.value("Preferences::use_opengl", true).toBool();
  ui->checkBoxOpenGL->setChecked(use_opengl);

  //---------------
  auto custom_plugin_folders =
      settings.value("Preferences::plugin_folders", true).toStringList();
  for (const auto& folder : custom_plugin_folders)
  {
    QDir dir(folder);
    auto item = new QListWidgetItem(folder);
    if (!dir.exists())
    {
      item->setForeground(Qt::red);
    }
    ui->listWidgetCustom->addItem(item);
  };
  ui->pushButtonRemove->setEnabled(!ui->listWidgetCustom->selectedItems().isEmpty());
  //---------------
  auto builtin_plugin_folders =
      settings.value("Preferences::builtin_plugin_folders", true).toStringList();
  for (const auto& folder : builtin_plugin_folders)
  {
    ui->listWidgetBuiltin->addItem(new QListWidgetItem(folder));
  };
}

PreferencesDialog::~PreferencesDialog()
{
  delete ui;
}

void PreferencesDialog::on_buttonBox_accepted()
{
  QSettings settings;
  settings.setValue("Preferences::theme",
                    ui->comboBoxTheme->currentIndex() == 1 ? "dark" : "light");
  settings.setValue("Preferences::remember_color",
                    ui->checkBoxRememberColor->isChecked());
  settings.setValue("Preferences::use_plot_color_index",
                    ui->radioLocalColorIndex->isChecked());
  settings.setValue("Preferences::use_separator", ui->checkBoxSeparator->isChecked());
  settings.setValue("Preferences::use_opengl", ui->checkBoxOpenGL->isChecked());

  QStringList plugin_folders;
  for (int row = 0; row < ui->listWidgetCustom->count(); row++)
  {
    plugin_folders.push_back(ui->listWidgetCustom->item(row)->text());
  }
  settings.setValue("Preferences::plugin_folders", plugin_folders);

  settings.sync();
}

void PreferencesDialog::on_pushButtonAdd_clicked()
{
  QString dir = QFileDialog::getExistingDirectory(
      this, tr("Select a Directory"), QDir::homePath(),
      QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
  auto item = new QListWidgetItem(dir);
  ui->listWidgetCustom->addItem(item);
}

void PreferencesDialog::on_pushButtonRemove_clicked()
{
  auto selected = ui->listWidgetCustom->selectedItems();

  for (QListWidgetItem* item : selected)
  {
    ui->listWidgetCustom->removeItemWidget(item);
    delete item;
  }
}

void PreferencesDialog::on_listWidgetCustom_itemSelectionChanged()
{
  ui->pushButtonRemove->setEnabled(!ui->listWidgetCustom->selectedItems().isEmpty());
}
