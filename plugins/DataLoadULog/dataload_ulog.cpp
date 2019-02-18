#include "dataload_ulog.h"
#include <QTextStream>
#include <QFile>
#include <QMessageBox>
#include <QDebug>
#include <QSettings>
#include <QProgressDialog>
#include "selectlistdialog.h"
#include "ulog_parser.h"

DataLoadULog::DataLoadULog()
{

}

const std::vector<const char*> &DataLoadULog::compatibleFileExtensions() const
{
    static  std::vector<const char*> extensions = { "ulg"};
    return extensions;
}


PlotDataMapRef DataLoadULog::readDataFromFile(const QString &file_name, bool)
{
    PlotDataMapRef plot_data;
    ULogParser parser( file_name.toStdString() );

    const auto& data = parser.getData();

    for( const auto& it: data)
    {
         const auto& format = it.first;
         const auto& timeseries = it.second;

         size_t index = 0;
         for (const auto& field: format->fields )
         {
             for(int array_index=0; array_index<field.array_size; array_index++ )
             {
                 char series_name[1000];
                 if( field.array_size == 1)
                 {
                     sprintf(series_name,"%s/%s", format->name.c_str(), field.field_name.c_str());
                 }
                 else{
                     sprintf(series_name,"%s/%s/%02d", format->name.c_str(), field.field_name.c_str(), array_index);
                 }

                 const auto& data = timeseries.data[index++];

                 auto series = plot_data.addNumeric( series_name );

                 for( size_t i=0; i < data.size(); i++ )
                 {
                     double msg_time = static_cast<double>(timeseries.timestamps[i]) * 0.000001;
                     PlotData::Point point( msg_time, data[i] );
                     series->second.pushBack( point );
                 }
             }
         }
    }

    return plot_data;
}

DataLoadULog::~DataLoadULog()
{

}

QDomElement DataLoadULog::xmlSaveState(QDomDocument &doc) const
{
    QDomElement elem = doc.createElement("no_params");
    return elem;
}

bool DataLoadULog::xmlLoadState(QDomElement&)
{
    return true;
}
