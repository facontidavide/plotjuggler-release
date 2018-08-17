#ifndef PLOTDATA_QWT_H
#define PLOTDATA_QWT_H

#include <QColor>
#include <qwt_series_data.h>
#include <qwt_plot_marker.h>
#include "PlotJuggler/plotdata.h"

class TimeseriesQwt: public QwtSeriesData<QPointF>
{
public:

    TimeseriesQwt(const PlotData* base, double time_offset);

    virtual ~TimeseriesQwt() {}

    virtual QPointF sample( size_t i ) const override;

    virtual QRectF boundingRect() const override;

    virtual size_t size() const override;

    const PlotData* data() const { return _plot_data; }

    void setSubsampleFactor();

    void updateData();

    PlotData::RangeTimeOpt getVisualizationRangeX();

    PlotData::RangeValueOpt getVisualizationRangeY(int first_index, int last_index );

    void setAlternativeAxisX(const PlotData* new_x_data);

    nonstd::optional<QPointF> sampleFromTime(double t);

    typedef enum{
      noTransform,
      firstDerivative,
      secondDerivative,
      XYPlot
    } Transform;

    void setTransform(Transform trans);

    Transform transform() const { return _transform; }

    double timeOffset() const { return _time_offset; }

public slots:

    void setTimeOffset(double offset);

private:
    const PlotData* _plot_data;
    const PlotData* _alternative_X_axis;

    std::vector<QPointF> _cached_transformed_curve;

    unsigned _subsample;

    Transform _transform;

    QRectF _bounding_box;

    double _time_offset;
};



#endif // PLOTDATA_H
