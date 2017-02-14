#include "plotdata_qwt.h"
#include <limits>
#include <stdexcept>


PlotDataQwt::PlotDataQwt(PlotDataPtr base):
    _plot_data(base),
    _subsample(1),
    _transform( noTransform )
{

}

QPointF PlotDataQwt::sample(size_t i) const
{
  if(_transform == noTransform) {
    const auto& p = _plot_data->at( i );
    return QPointF( p.x, p.y) ;
  }
  else {
    return _cached_transformed_curve[i];
  }
}

QRectF PlotDataQwt::boundingRect() const
{
    qDebug() << "boundingRect not implemented";
    return QRectF(0,0,1,1);
}


size_t PlotDataQwt::size() const
{
  if(_transform == noTransform) {
    return _plot_data->size();
  }
  else {
    return _cached_transformed_curve.size();
  }
}


QRectF PlotDataQwt::maximumBoundingRect(double min_X, double max_X)
{
    int x1 = _plot_data->getIndexFromX( min_X );
    int x2 = _plot_data->getIndexFromX( max_X );

    if( x1 <0 || x2 <0){
      return QRectF();
    }

    auto range_X = getRangeX();

    if( !range_X ){
      return QRectF();
    }

    auto range_Y = getRangeY( x1, x2  );
    if( !range_Y){
      return QRectF();
    }

    QRectF rect ( range_X->min,
                  range_Y->min,
                  range_X->max - range_X->min,
                  range_Y->max - range_Y->min );
    return rect;
}

void PlotDataQwt::setSubsampleFactor()
{
  //  _subsample = (_plot_data->size() / 2000) + 1;
}

void PlotDataQwt::updateData(bool force_transform)
{
  bool updated = _plot_data->flushAsyncBuffer();

  if(updated || force_transform)
  {
    if(_transform == firstDerivative)
    {
      if( _plot_data->size() < 1){
        _cached_transformed_curve.clear();
      }
      else{
        _cached_transformed_curve.resize( _plot_data->size() - 1 );
      }

      for (size_t i=0; i< _plot_data->size() -1; i++ )
      {
        const auto& p0 = _plot_data->at( i );
        const auto& p1 = _plot_data->at( i+1 );
        const auto delta = p1.x - p0.x;
        const auto vel = (p1.y - p0.y) /delta;
        _cached_transformed_curve[i] = QPointF( (p1.x + p0.x)*0.5, vel) ;
      }
    }
    else if(_transform == secondDerivative)
    {
      if( _plot_data->size() < 2){
        _cached_transformed_curve.clear();
      }
      else{
        _cached_transformed_curve.resize( _plot_data->size() - 2 );
      }

      for (size_t i=0; i< _cached_transformed_curve.size(); i++ )
      {
        const auto& p0 = _plot_data->at( i );
        const auto& p1 = _plot_data->at( i+1 );
        const auto& p2 = _plot_data->at( i+2 );
        const auto delta = (p2.x - p0.x) *0.5;
        const auto acc = ( p2.y - 2.0* p1.y + p0.y)/(delta*delta);
        _cached_transformed_curve[i] =  QPointF( (p2.x + p0.x)*0.5, acc ) ;
      }
    }
  }
}

PlotData::RangeTimeOpt PlotDataQwt::getRangeX()
{
  // std::lock_guard<std::mutex> lock(_mutex);
  if( this->size() < 2 )
    return  PlotData::RangeTimeOpt() ;
  else
    return  PlotData::RangeTimeOpt( { sample(0).x(), sample( this->size() -1).x() } );
}




PlotData::RangeValueOpt PlotDataQwt::getRangeY(int first_index, int last_index)
{
  if( first_index < 0 || last_index < 0 || first_index > last_index)
  {
    return PlotData::RangeValueOpt();
  }
  //std::lock_guard<std::mutex> lock(_mutex);

  const double first_Y = sample(first_index).y();
  double y_min = first_Y;
  double y_max = first_Y;

  for (int i = first_index+1; i < last_index; i++)
  {
    const double Y = sample(i).y();

    if( Y < y_min )      y_min = Y;
    else if( Y > y_max ) y_max = Y;
  }
  return PlotData::RangeValueOpt( { y_min, y_max } );
}
