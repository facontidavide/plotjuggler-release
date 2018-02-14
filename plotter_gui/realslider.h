#ifndef REALSLIDER_H
#define REALSLIDER_H

#include <QSlider>



inline static int Round(double x)
{
    return (x < 0.0) ? static_cast<int>(x - 0.5) : static_cast<int>(x + 0.5);
}




class RealSlider: public QSlider
{
    Q_OBJECT
public:
    RealSlider(QWidget* parent = nullptr);

    void setLimits(double min, double max, int steps);
    double getValue() const;
    void setRealValue(double val);

    double getMaximum() const { return _max_value; }

    double getMinimum() const { return _min_value; }

private slots:
    void onValueChanged(int value);

signals:
    void realValueChanged(double);

private:
    double _min_value;
    double _max_value;
};
//-------------------------------------------------------------

inline RealSlider::RealSlider(QWidget *parent):
    QSlider(parent)
{
    setLimits(0.0, 1.0, 1);
    connect(this, &QSlider::valueChanged,
            this, &RealSlider::onValueChanged);
}

inline void RealSlider::setLimits(double min, double max,int steps)
{
    _min_value = min;
    _max_value = max;
    QSlider::setRange(0,steps);
}

inline void RealSlider::setRealValue(double val)
{
    val = std::max( val, _min_value) ;
    val = std::min( val, _max_value) ;
    const double ratio = (val -_min_value)  / (_max_value - _min_value );
    int pos = Round( (double)(maximum() - minimum()) * ratio  + minimum()) ;
    QSlider::setValue(pos);
}

inline void RealSlider::onValueChanged(int value)
{
    int min =  minimum() ;
    int max =  maximum() ;
    const double ratio = (double)value / (double)(max -  min );
    double posX = (_max_value - _min_value) * ratio + _min_value;
    emit realValueChanged(posX);
}

#endif // REALSLIDER_H
