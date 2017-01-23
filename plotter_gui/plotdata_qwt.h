#ifndef PLOTDATA_QWT_H
#define PLOTDATA_QWT_H

#include <QColor>
#include <qwt_series_data.h>
#include "PlotJuggler/plotdata.h"



class PlotDataQwt: public QwtSeriesData<QPointF>
{
public:

    PlotDataQwt(PlotDataPtr base);

    virtual ~PlotDataQwt() {}

    virtual QPointF sample( size_t i ) const override;

    virtual QRectF boundingRect() const override;

    virtual size_t size() const override;

    QRectF maximumBoundingRect(double min_X, double max_X);

    PlotDataPtr data() { return _plot_data; }

    QColor randomColorHint() const;
    void setColorHint(QColor color);

    void setSubsampleFactor();

    void updateData(bool force_transform);

    PlotData::RangeTime getRangeX();

    PlotData::RangeValue getRangeY(int first_index, int last_index );

    typedef enum{
      noTransform,
      firstDerivative,
      secondDerivative
    } Transform;

    void setTransform(Transform trans) { _transform = trans; }
    Transform transform() const { return _transform; }

private:
    PlotDataPtr _plot_data;
    std::vector<QPointF> _cached_transformed_curve;
    int      _preferedColor;
    unsigned _subsample;
    Transform _transform;

};



#endif // PLOTDATA_H
