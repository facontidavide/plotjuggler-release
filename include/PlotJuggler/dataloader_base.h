#ifndef DATALOAD_TEMPLATE_H
#define DATALOAD_TEMPLATE_H

#include <QFile>

#include <functional>
#include "PlotJuggler/plotdata.h"
#include "PlotJuggler/pj_plugin.h"
#include "PlotJuggler/messageparser_base.h"

namespace PJ {

struct FileLoadInfo
{
  QString filename;
  QString prefix;
  QStringList selected_datasources;
  QDomDocument plugin_config;
};

class DataLoader : public PlotJugglerPlugin
{
public:
  DataLoader()
  {
  }

  virtual const std::vector<const char*>& compatibleFileExtensions() const = 0;

  virtual bool readDataFromFile(FileLoadInfo* fileload_info, PlotDataMapRef& destination) = 0;

  virtual ~DataLoader()
  {
  }

protected:
};

using DataLoaderPtr = std::shared_ptr<DataLoader>;

}

QT_BEGIN_NAMESPACE
#define DataRead_iid "facontidavide.PlotJuggler3.DataLoader"
Q_DECLARE_INTERFACE(PJ::DataLoader, DataRead_iid)
QT_END_NAMESPACE

#endif
