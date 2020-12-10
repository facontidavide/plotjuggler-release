#ifndef TIMESERIES_QWT_H
#define TIMESERIES_QWT_H

#include "PlotJuggler/plotdata.h"
#include "qwt_series_data.h"
#include "PlotJuggler/transform_function.h"

using namespace PJ;


// wrapper to PlotData inclduing a time offset
class QwtSeriesWrapper: public QwtSeriesData<QPointF>
{
private:
  const PlotDataBase<double>* _data;
  double _time_offset;

public:
  QwtSeriesWrapper(const PlotDataBase<double>* data): _data(data), _time_offset(0.0) {}

  QPointF sample(size_t i) const override;

  void setTimeOffset(double offset);

  virtual bool updateCache(bool reset_old_data) = 0;

  size_t size() const override;

  QRectF boundingRect() const override;

  const PlotDataBase<double>* plotData() const;

  virtual RangeOpt getVisualizationRangeX();

  virtual RangeOpt getVisualizationRangeY(Range range_X) = 0;

  virtual nonstd::optional<QPointF> sampleFromTime(double t) = 0;

};


class QwtTimeseries : public QwtSeriesWrapper
{
public:
  QwtTimeseries(const PlotData* data):
    QwtSeriesWrapper(data),
    _ts_data(data)
  {
  }

  virtual RangeOpt getVisualizationRangeY(Range range_X) override;

  virtual nonstd::optional<QPointF> sampleFromTime(double t) override;

protected:
  const PlotData* _ts_data;
};

//------------------------------------

class TransformedTimeseries : public QwtTimeseries
{
public:
  TransformedTimeseries(const PlotData* source_data);

  TimeSeriesTransformPtr transform();

  void setTransform(QString transform_ID);

  virtual bool updateCache(bool reset_old_data) override;

  QString transformName();

protected:
  const PlotData* _source_data;
  PlotData _dst_data;
  TimeSeriesTransformPtr _transform;
};

//---------------------------------------------------------



#endif  // PLOTDATA_H
