#ifndef DATALOAD_TEMPLATE_H
#define DATALOAD_TEMPLATE_H

#include <QtPlugin>
#include <functional>
#include "PlotJuggler/plotdata.h"

QT_BEGIN_NAMESPACE
class QFile;
QT_END_NAMESPACE

enum { TIME_INDEX_NOT_DEFINED = -2 };

class DataLoader{


public:

    virtual const std::vector<const char*>& compatibleFileExtensions() const = 0;
    virtual PlotDataMap readDataFromFile(const std::string& file_name,
                                      std::string& time_index ) = 0;

    virtual const char* name() const = 0;

    virtual ~DataLoader() {}
};

QT_BEGIN_NAMESPACE

#define DataRead_iid "com.icarustechnology.PlotJuggler.DataLoader"

Q_DECLARE_INTERFACE(DataLoader, DataRead_iid)

QT_END_NAMESPACE


#endif

