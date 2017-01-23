#include <QFile>
#include <QSettings>
#include <QDir>
#include <QFileInfo>
#include <QFileDialog>
#include <QTextStream>
#include <QMessageBox>
#include "ruleloaderwidget.h"
#include "ui_ruleloaderwidget.h"
#include <QDomDocument>

RuleLoaderWidget::RuleLoaderWidget(QWidget *parent) :
  QWidget(parent),
  ui(new Ui::RuleLoaderWidget)
{
  ui->setupUi(this);

  QSettings settings( "IcarusTechnology", "PlotJuggler");

  if( settings.contains( "RuleLoaderWidget.previouslyLoadedRules" ) )
  {
    QString fileName = settings.value( "RuleLoaderWidget.previouslyLoadedRules" ).toString();
    // check if it exist
    QFile file(fileName);

    if( file.exists() )
    {
      readRuleFile( file );
    }
    else{
      ui->labelLoadedRules->setText("none");
    }
  }

}

RuleLoaderWidget::~RuleLoaderWidget()
{
  QSettings settings( "IcarusTechnology", "PlotJuggler");
  settings.setValue(  "RuleLoaderWidget.enableRules" , ui->checkBoxEnableSubstitution->isChecked() );

  delete ui;
}

const RosIntrospection::SubstitutionRuleMap& RuleLoaderWidget::getLoadedRules() const
{
  return _loaded_rules;
}

void RuleLoaderWidget::on_pushButtonLoadOther_pressed()
{
  QSettings settings( "IcarusTechnology", "PlotJuggler");

  QString directory_path = QDir::currentPath();
  QString fileName;

  if( settings.contains( "RuleLoaderWidget.previouslyLoadedRules" ) )
  {
    fileName = settings.value( "RuleLoaderWidget.previouslyLoadedRules" ).toString();
    QFileInfo info( fileName );
    directory_path = info.absoluteFilePath();
  }

  fileName = QFileDialog::getOpenFileName(this, "Open Layout", directory_path, "*.txt *.xml");

  if (fileName.isEmpty())
    return;

  QFile file(fileName);
  readRuleFile( file );
}

void RuleLoaderWidget::on_checkBoxEnableSubstitution_toggled(bool checked)
{
  ui->pushButtonLoadOther->setEnabled( checked );
}

void RuleLoaderWidget::readRuleFile(QFile &file)
{
  using namespace RosIntrospection;

  if (! file.open(QFile::ReadOnly | QFile::Text))
  {
    QMessageBox::warning(0, tr("Warning"), tr("Can't open file\n") );
    ui->labelLoadedRules->setText("none");
    _loaded_rules.clear();
    return;
  }

  QSettings settings( "IcarusTechnology", "PlotJuggler");
  settings.setValue( "RuleLoaderWidget.previouslyLoadedRules", file.fileName() );

  ui->labelLoadedRules->setText( file.fileName() );
  //-----------------------------------------------
  QString errorStr;
  int errorLine, errorColumn;

  QDomDocument domDocument;

  if (!domDocument.setContent(&file, true, &errorStr, &errorLine, &errorColumn)) {
    QMessageBox::information(window(), tr("XML Layout"),
                             tr("Parse error at line %1:\n%2")
                             .arg(errorLine)
                             .arg(errorStr));
    return;
  }

  QDomElement root = domDocument.namedItem("SubstitutionRules").toElement();

  errorStr.clear();

  for ( auto type_el = root.firstChildElement(  "RosType" )  ;
        type_el.isNull() == false;
        type_el = type_el.nextSiblingElement( "RosType" ) )
  {
    if( type_el.hasAttribute("name") == false)
    {
      QMessageBox::warning(0, tr("Error"), tr("Missing attribute \"name\" in RosType") );
      ui->labelLoadedRules->setText("none");
      _loaded_rules.clear();
      return;
    }
    else{
      QString ros_type_name = type_el.attribute("name");
      std::vector< SubstitutionRule > rules;

      for ( auto rule_el = type_el.firstChildElement(  "rule" )  ;
            rule_el.isNull() == false;
            rule_el = rule_el.nextSiblingElement( "rule" ) )

      {
        auto pattern_el      = rule_el.firstChildElement("pattern");
        auto alias_el        = rule_el.firstChildElement("alias");
        auto substitution_el = rule_el.firstChildElement("substitution");

        if(pattern_el.isNull() ){
          errorStr = tr("<rule> needs a child called <pattern>");
        }
        else if( alias_el.isNull() ){
          errorStr = tr("<rule> needs a child called <alias>");
        }
        else if( substitution_el.isNull() ){
          errorStr = tr("<rule> needs a child called <substitution>");
        }
        else{
          SubstitutionRule rule(
                pattern_el.text().toStdString().c_str(),
                alias_el.text().toStdString().c_str(),
                substitution_el.text().toStdString().c_str() );

          rules.push_back( std::move( rule ) );
        }

        if( errorStr.size() != 0)
        {
          QMessageBox::warning(0, tr("Error"), errorStr );
          ui->labelLoadedRules->setText("none");
          _loaded_rules.clear();
          return;
        }
      }
      _loaded_rules[ros_type_name.toStdString() ] = rules;
    }
  }

  file.close();

}
