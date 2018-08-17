#ifndef DATALOAD_CSV_H
#define DATALOAD_CSV_H

#include <QObject>
#include <QtPlugin>
#include "PlotJuggler/dataloader_base.h"


class  DataLoadCSV: public QObject, DataLoader
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "com.icarustechnology.PlotJuggler.DataLoader" "../dataloader.json")
    Q_INTERFACES(DataLoader)

public:
    DataLoadCSV();
    virtual const std::vector<const char*>& compatibleFileExtensions() const override;

    virtual PlotDataMapRef readDataFromFile(const QString& file_name, bool use_previous_configuration  ) override;

    virtual ~DataLoadCSV();

    virtual const char* name() const override { return "DataLoad CSV"; }

    virtual QDomElement xmlSaveState(QDomDocument &doc) const override;

    virtual bool xmlLoadState(QDomElement &parent_element ) override;

protected:
    QSize parseHeader(QFile *file, std::vector<std::string> &ordered_names);

private:
    std::vector<const char*> _extensions;

    std::string _default_time_axis;


};

#endif // DATALOAD_CSV_H
