#ifndef MULTIFILE_PREFIX_H
#define MULTIFILE_PREFIX_H

#include <QDialog>
#include <QLineEdit>

namespace Ui {
class DialogMultifilePrefix;
}

class DialogMultifilePrefix : public QDialog
{
  Q_OBJECT

  public:
  explicit DialogMultifilePrefix(QStringList filenames, QWidget *parent = nullptr);

  std::map<QString, QString> getPrefixes() const;

  ~DialogMultifilePrefix();

  virtual void accept() override;

  private:
  Ui::DialogMultifilePrefix *ui;
  std::map<QString, QLineEdit*> _line_edits;
  std::map<QString, QString> _previous_prefixes;
  std::map<QString, QString> _prefixes;
};

#endif // MULTIFILE_PREFIX_H
