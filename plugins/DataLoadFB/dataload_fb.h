#ifndef DATALOAD_FLATBUFFER_H
#define DATALOAD_FLATBUFFER_H

#include <QObject>
#include <QtPlugin>
#include "../dataloader_base.h"


class  DataLoadFlatbuffer: public QObject, DataLoader
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "com.icarustechnology.Superplotter.DataLoader" "../dataloader.json")
    Q_INTERFACES(DataLoader)

public:
    DataLoadFlatbuffer();
    virtual const std::vector<const char*>& compatibleFileExtensions() const ;
    virtual PlotDataMap readDataFromFile(QFile* file,
                                         std::function<void(int)> updateCompletion,
                                         std::function<bool()> checkInterruption,
                                         int time_index = TIME_INDEX_NOT_DEFINED );

    virtual ~DataLoadFlatbuffer();

private:
    std::vector<const char*> _extensions;


};

#endif // DATALOAD_FLATBUFFER_H
