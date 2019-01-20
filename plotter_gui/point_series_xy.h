#ifndef POINT_SERIES_H
#define POINT_SERIES_H

#include "series_data.h"

class PointSeriesXY: public DataSeriesBase
{
public:
    PointSeriesXY(const PlotData* y_axis, const PlotData* x_axis);

    nonstd::optional<QPointF> sampleFromTime(double t) override;

    PlotData::RangeValueOpt getVisualizationRangeY(PlotData::RangeTime range_X) override;

    bool updateCache() override;

protected:

    const PlotData *_x_axis;
    const PlotData *_y_axis;
    PlotData _cached_curve;
};

#endif // POINT_SERIES_H
