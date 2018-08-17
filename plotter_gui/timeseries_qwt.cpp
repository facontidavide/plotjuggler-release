#include "timeseries_qwt.h"
#include <limits>
#include <stdexcept>
#include <QMessageBox>
#include <QPushButton>
#include <QString>

bool if_xy_plot_failed_show_dialog = true;

TimeseriesQwt::TimeseriesQwt(const PlotData *base, double time_offset):
    _plot_data(base),
    _subsample(1),
    _transform( noTransform ),
    _time_offset(time_offset)
{
    updateData();
}

QPointF TimeseriesQwt::sample(size_t i) const
{
    if(_transform == noTransform)
    {
        auto p = _plot_data->at(i);
        return QPointF(p.x - _time_offset, p.y);
    }
    return _cached_transformed_curve[i];
}

QRectF TimeseriesQwt::boundingRect() const
{
    return _bounding_box;
}

size_t TimeseriesQwt::size() const
{
    if(_transform == noTransform){
        return _plot_data->size();
    }
    return _cached_transformed_curve.size();
}

void TimeseriesQwt::setSubsampleFactor()
{
    //  _subsample = (_plot_data->size() / 2000) + 1;
}


void TimeseriesQwt::updateData()
{
    if(_plot_data->size() == 0) return;
    const size_t data_size = _plot_data->size();

    double min_y =( std::numeric_limits<double>::max() );
    double max_y =(-std::numeric_limits<double>::max() );
    double min_x =( std::numeric_limits<double>::max() );
    double max_x =(-std::numeric_limits<double>::max() );

    auto updateMinMax = [&min_x, &min_y, &max_x, &max_y](double x, double y)
    {
        min_x = std::min( min_x, x );
        max_x = std::max( max_x, x );

        min_y = std::min( min_y, y );
        max_y = std::max( max_y, y );
    };

    if(_transform == noTransform)
    {
        _cached_transformed_curve.resize( 0 );
        _cached_transformed_curve.shrink_to_fit();

        for (size_t i=0; i < data_size; i++ )
        {
            auto p = _plot_data->at( i );
            updateMinMax( p.x, p.y );
        }
        min_x -= _time_offset;
        max_x -= _time_offset;
    }
    else if(_transform == firstDerivative)
    {
        if( data_size < 1){
            _cached_transformed_curve.clear();
        }
        else{
            _cached_transformed_curve.resize( data_size - 1 );
        }

        for (size_t i=0; i < data_size -1; i++ )
        {
            const auto& p0 = _plot_data->at( i );
            const auto& p1 = _plot_data->at( i+1 );
            const auto delta = p1.x - p0.x;
            const auto vel = (p1.y - p0.y) /delta;
            QPointF p( (p1.x + p0.x)*0.5, vel);
            p.setX( p.x() - _time_offset);
            _cached_transformed_curve[i] = p;

            updateMinMax( p.x(), p.y() );
        }
    }
    else if(_transform == secondDerivative)
    {
        if( data_size < 2){
            _cached_transformed_curve.clear();
        }
        else{
            _cached_transformed_curve.resize( data_size - 2 );
        }

        for (size_t i=0; i < data_size - 2; i++ )
        {
            const auto& p0 = _plot_data->at( i );
            const auto& p1 = _plot_data->at( i+1 );
            const auto& p2 = _plot_data->at( i+2 );
            const auto delta = (p2.x - p0.x) *0.5;
            const auto acc = ( p2.y - 2.0* p1.y + p0.y)/(delta*delta);
            QPointF p( (p2.x + p0.x)*0.5, acc );
            p.setX( p.x() - _time_offset);
            _cached_transformed_curve[i] = p;

            updateMinMax( p.x(), p.y() );
        }
    }
    else if( _transform == XYPlot && _alternative_X_axis)
    {
        bool failed = false;
        const size_t N = _alternative_X_axis->size();

        if( data_size != N ){
            failed = true ;
        }

        for (size_t i=0; i<N && !failed; i++ )
        {
            if( _alternative_X_axis->at(i).x != _plot_data->at(i).x ){
                failed = true ;
                break;
            }
        }

        if( failed)
        {
            if( if_xy_plot_failed_show_dialog )
            {
                QMessageBox msgBox;
                msgBox.setWindowTitle("Warnings");
                msgBox.setText("The creation of the XY plot failed because at least two "
                               "timeseries don't share the same time axis.");

                QAbstractButton* buttonDontRepear = msgBox.addButton("Don't show again",
                                                                     QMessageBox::ActionRole);
                msgBox.addButton("Continue", QMessageBox::AcceptRole);
                msgBox.exec();

                if (msgBox.clickedButton() == buttonDontRepear) {
                    if_xy_plot_failed_show_dialog = false;
                }
            }
            return;
        }
        _cached_transformed_curve.resize(N);
        for (size_t i=0; i<N; i++ )
        {
            const QPointF p(_alternative_X_axis->at(i).y,
                            _plot_data->at(i).y );
            _cached_transformed_curve[i] = p;

            updateMinMax( p.x(), p.y() );
        }
    }

    _bounding_box.setBottom(min_y);
    _bounding_box.setTop(max_y);

    _bounding_box.setLeft(min_x);
    _bounding_box.setRight(max_x);
}

PlotData::RangeTimeOpt TimeseriesQwt::getVisualizationRangeX()
{   
    // std::lock_guard<std::mutex> lock(_mutex);
    if( this->size() < 2 )
        return  PlotData::RangeTimeOpt();
    else{
        return PlotData::RangeTimeOpt( { _bounding_box.left(), _bounding_box.right() } );
    }
}


PlotData::RangeValueOpt TimeseriesQwt::getVisualizationRangeY(int first_index, int last_index)
{
    if( first_index < 0 || last_index < 0 || first_index > last_index)
    {
        return PlotData::RangeValueOpt();
    }

    if( (_transform == XYPlot && _alternative_X_axis) ||
            ( first_index==0 && last_index == size() -1) )
    {
        return PlotData::RangeValueOpt( { _bounding_box.bottom(), _bounding_box.top() } );
    }

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

void TimeseriesQwt::setAlternativeAxisX(const PlotData *new_x_data)
{
    _alternative_X_axis = new_x_data;
}

nonstd::optional<QPointF> TimeseriesQwt::sampleFromTime(double t)
{
    if( _transform == XYPlot && _alternative_X_axis)
    {
        auto res1 = _alternative_X_axis->getYfromX( t );
        if( res1)
        {
            auto res2 = _plot_data->getYfromX( t );
            if( res2 ){
                return nonstd::optional<QPointF>(
                            QPointF(res1.value(), res2.value() ) ) ;
            }
        }
    }
    else{
        auto res = _plot_data->getYfromX( t );
        if(res){
            return nonstd::optional<QPointF>( QPointF(t, res.value() ) ) ;
        }
    }
    return nonstd::optional<QPointF>();
}

void TimeseriesQwt::setTransform(TimeseriesQwt::Transform trans)
{
    if(trans != _transform)
    {
        _transform = trans;
        updateData();
    }
}

void TimeseriesQwt::setTimeOffset(double new_offset)
{
    double delta = new_offset - _time_offset;
    if( std::abs( delta ) > std::numeric_limits<double>::epsilon())
    {
        double min_x =( std::numeric_limits<double>::max() );
        double max_x =(-std::numeric_limits<double>::max() );
        _time_offset = new_offset;

        if( _transform == noTransform)
        {
            const size_t data_size = _plot_data->size();
            for (size_t i=0; i<data_size; i++  )
            {
                auto p = _plot_data->at(i);
                min_x = std::min( min_x, p.x );
                max_x = std::max( max_x, p.x );
            }
            min_x -= _time_offset;
            max_x -= _time_offset;
        }
        else if( _transform == firstDerivative || _transform == secondDerivative)
        {
            for (auto& p: _cached_transformed_curve )
            {
                p.setX( p.x() - delta );
                min_x = std::min( min_x, p.x() );
                max_x = std::max( max_x, p.x() );
            }
        }

        if( _transform != XYPlot && _plot_data->size() >=2 )
        {
           _bounding_box.setLeft( min_x );
           _bounding_box.setRight( max_x );
        }
    }
}
