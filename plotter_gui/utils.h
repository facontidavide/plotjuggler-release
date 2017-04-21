#ifndef UTILS_H
#define UTILS_H

#include <QObject>

class MonitoredValue: public QObject{
    Q_OBJECT
public:
    MonitoredValue(QObject* parent = nullptr): QObject(parent), _value(0) {}

    void set(double newValue){
        _value = newValue;
        emit valueChanged(_value);
    }

    double get() const { return _value; }
signals:
    void valueChanged(double);
private:
    double _value;
};

#endif // UTILS_H
