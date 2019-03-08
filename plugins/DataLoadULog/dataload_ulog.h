#ifndef DATALOAD_CSV_H
#define DATALOAD_CSV_H

#include <QObject>
#include <QtPlugin>
#include <QWidget>
#include "PlotJuggler/dataloader_base.h"


class  DataLoadULog: public QObject, DataLoader
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "com.icarustechnology.PlotJuggler.DataLoader" "../dataloader.json")
    Q_INTERFACES(DataLoader)

public:
    DataLoadULog();

    const std::vector<const char*>& compatibleFileExtensions() const override;

    PlotDataMapRef readDataFromFile(const QString& file_name, bool) override;

    ~DataLoadULog() override;

    const char* name() const override { return "DataLoad ULog"; }

    QDomElement xmlSaveState(QDomDocument &doc) const override;

    bool xmlLoadState(QDomElement &parent_element ) override;

private:

    std::string _default_time_axis;
    QWidget* _main_win;
};

#endif // DATALOAD_CSV_H
