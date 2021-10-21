#ifndef DATALOAD_TEMPLATE_H
#define DATALOAD_TEMPLATE_H

#include <QFile>

#include <functional>
#include "PlotJuggler/plotdata.h"
#include "PlotJuggler/pj_plugin.h"
#include "PlotJuggler/messageparser_base.h"

namespace PJ
{
struct FileLoadInfo
{
  /// name of the file to open
  QString filename;
  /// prefix to be added to the name of the series (optional)
  QString prefix;
  /// Optional list of pre-selected datasource
  QStringList selected_datasources;
  /// Saved configuration from a previous run or a Layout file
  QDomDocument plugin_config;
};

/**
 * @brief The DataLoader plugin type is used to load files.
 *
 * The main application selects the correct plugin using the
 * compatibleFileExtensions()
 */
class DataLoader : public PlotJugglerPlugin
{
public:
  DataLoader() = default;

  virtual ~DataLoader() = default;

  /// Provide a list of file extensions that this plugin can open
  virtual const std::vector<const char*>& compatibleFileExtensions() const = 0;

  virtual bool readDataFromFile(FileLoadInfo* fileload_info,
                                PlotDataMapRef& destination) = 0;
};

using DataLoaderPtr = std::shared_ptr<DataLoader>;

}  // namespace PJ

QT_BEGIN_NAMESPACE
#define DataRead_iid "facontidavide.PlotJuggler3.DataLoader"
Q_DECLARE_INTERFACE(PJ::DataLoader, DataRead_iid)
QT_END_NAMESPACE

#endif
