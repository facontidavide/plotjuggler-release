#ifndef PLOTDATA_RAW_H
#define PLOTDATA_RAW_H

#include <vector>
#include <memory>
#include <string>
#include <map>
#include <mutex>
#include <deque>
#include "PlotJuggler/optional.hpp"
#include "PlotJuggler/any.hpp"
#include <QDebug>
#include <QColor>
#include <type_traits>
#include <cmath>
#include <cstdlib>
#include <unordered_map>

namespace PJ {

struct Range
{
  double min;
  double max;
};

typedef nonstd::optional<Range> RangeOpt;


template <typename Value>
class PlotDataBase
{
public:

  class Point
  {
  public:
    double x;
    Value y;
    Point(double _x, Value _y) : x(_x), y(_y)
    {
    }
    Point() = default;
  };

  enum
  {
    MAX_CAPACITY = 1024 * 1024,
    ASYNC_BUFFER_CAPACITY = 1024
  };

  typedef double TypeX;

  typedef Value TypeY;

  typedef typename std::deque<Point>::iterator Iterator;

  typedef typename std::deque<Point>::const_iterator ConstIterator;

  PlotDataBase(const std::string& name):
    _color_hint(Qt::black),
    _name(name),
    _range_x_dirty(true),
    _range_y_dirty(true)
  {}

  PlotDataBase(const PlotDataBase& other) = delete;

  PlotDataBase(PlotDataBase&& other)
  {
    (*this) = std::move(other);
  }

  void swapData(PlotDataBase& other)
  {
    std::swap(_points, other._points);
    std::swap(_range_x, other._range_x);
    std::swap(_range_y, other._range_y);
    std::swap(_range_x_dirty, other._range_x_dirty);
    std::swap(_range_y_dirty, other._range_y_dirty);
  }

  PlotDataBase& operator=(const PlotDataBase& other) = delete;

  virtual ~PlotDataBase() = default;

  const std::string& name() const
  {
    return _name;
  }

  virtual size_t size() const
  {
    return _points.size();
  }

  const Point& at(size_t index) const
  {
    return _points[index];
  }

  Point& at(size_t index)
  {
    return _points[index];
  }

  const Point& operator[](size_t index) const
  {
    return at(index);
  }

  Point& operator[](size_t index)
  {
    return at(index);
  }

  void clear()
  {
    _points.clear();
    _range_x_dirty = true;
    _range_y_dirty = true;
  }

  QColor getColorHint() const
  {
    return _color_hint;
  }

  void setColorHint(QColor color)
  {
    _color_hint = color;
  }

  const Point& front() const
  {
    return _points.front();
  }

  const Point& back() const
  {
    return _points.back();
  }

  ConstIterator begin() const
  {
    return _points.begin();
  }

  ConstIterator end() const
  {
    return _points.end();
  }

  Iterator begin()
  {
    return _points.begin();
  }

  Iterator end()
  {
    return _points.end();
  }

  virtual RangeOpt rangeX() const
  {
    if( _points.empty() ){
      return nonstd::nullopt;
    }
    if( _range_x_dirty )
    {
      _range_x.min = front().x;
      _range_x.max = _range_x.min;
      for(const auto& p: _points)
      {
        _range_x.min = std::min(_range_x.min, p.x);
        _range_x.max = std::max(_range_x.max, p.x);
      }
      _range_x_dirty = false;
    }
    return _range_x;
  }

  RangeOpt rangeY() const
  {
    if( _points.empty() ){
      return nonstd::nullopt;
    }
    if( _range_y_dirty )
    {
      _range_y.min = front().y;
      _range_y.max = _range_y.min;
      for(const auto& p: _points)
      {
        _range_y.min = std::min(_range_y.min, p.y);
        _range_y.max = std::max(_range_y.max, p.y);
      }
      _range_y_dirty = false;
    }
    return _range_y;
  }

  void pushBack(const Point &p)
  {
    auto temp = p;
    pushBack(std::move(temp));
  }

  virtual void pushBack(Point&& p)
  {
    if( _points.empty() )
    {
      _range_x_dirty = false;
      _range_x.min = p.x;
      _range_x.max = p.x;

      _range_y.min = p.y;
      _range_y.max = p.y;
    }
    if( !_range_x_dirty )
    {
      if( p.x > _range_x.max ){
        _range_x.max = p.x;
      }
      else if( p.x < _range_x.min ){
        _range_x.min = p.x;
      }
      else{
        _range_x_dirty = true;
      }
    }

    if( !_range_y_dirty )
    {
      if( p.y > _range_y.max ){
        _range_y.max = p.y;
      }
      else if( p.y < _range_y.min ){
        _range_y.min = p.y;
      }
      else{
        _range_y_dirty = true;
      }
    }
    _points.emplace_back(p);
  }

  virtual void popFront()
  {
    const auto& p = _points.front();

    if( !_range_x_dirty && (p.x == _range_x.max || p.x == _range_x.min) )
    {
      _range_x_dirty = true;
    }
    if( !_range_y_dirty && (p.y == _range_y.max || p.y == _range_y.min) )
    {
      _range_y_dirty = true;
    }
    _points.pop_front();
  }

protected:
  std::string _name;
  QColor _color_hint;

  // when everything is mutable, nothing is const :(
  std::deque<Point> _points;
  mutable Range _range_x;
  mutable Range _range_y;
  mutable bool _range_x_dirty;
  mutable bool _range_y_dirty;
};


//-----------------------------------

template <typename Value>
class TimeseriesBase: public PlotDataBase<Value>
{
protected:
  double _max_range_x;

  using PlotDataBase<Value>::_points;
  using PlotDataBase<Value>::_range_x_dirty;
  using PlotDataBase<Value>::_range_y_dirty;
  using PlotDataBase<Value>::_range_x;
  using PlotDataBase<Value>::_range_y;

public:
  using Point = typename PlotDataBase<Value>::Point;

  TimeseriesBase(const std::string& name):
    PlotDataBase<Value>(name),
    _max_range_x( std::numeric_limits<double>::max() )
  {
    _range_x_dirty = false;
  }

  void setMaximumRangeX(double max_range)
  {
    _max_range_x = max_range;
    trimRange();
  }

  double maximumRangeX() const
  {
    return _max_range_x;
  }

  int getIndexFromX(double x) const;

  nonstd::optional<Value> getYfromX(double x) const;

  void popFront() override
  {
    const auto& p = _points.front();

    if( !_range_y_dirty && (p.y == _range_y.max || p.y == _range_y.min) )
    {
      _range_y_dirty = true;
    }
    _points.pop_front();

    if( !_points.empty() )
    {
      _range_x.min = _points.front().x;
    }
  }

  void pushBack(const Point &p)
  {
    auto temp = p;
    pushBack(std::move(temp));
  }

  void pushBack(Point&& p) override
  {
    if (std::isinf(p.y) || std::isnan(p.y))
    {
      return;  // skip
    }
    bool need_sorting = false;

    if( _points.empty() )
    {
      _range_x_dirty = false;
      _range_x.min = p.x;
      _range_x.max = p.x;

      _range_y.min = p.y;
      _range_y.max = p.y;
    }
    else{
      if( p.x < this->back().x ){
        need_sorting = true;
      }
    }

    if( need_sorting ) {
      auto it = std::upper_bound(_points.begin(), _points.end(), p,
                                 [](const Point& a, const Point& b) { return a.x < b.x; });

      _points.insert(it, p);
    }
    else{
      _points.emplace_back(p);
    }

    _range_x.max = p.x;

    if( !_range_y_dirty )
    {
      if( p.y > _range_y.max ){
        _range_y.max = p.y;
      }
      else if( p.y < _range_y.min ){
        _range_y.min = p.y;
      }
      else{
        _range_y_dirty = true;
      }
    }

    trimRange();
  }

  RangeOpt rangeX() const override
  {
    if( _points.empty() ){
      return nonstd::nullopt;
    }
    if( _range_x_dirty )
    {
      _range_x.min = _points.front().x;
      _range_x.max = _points.back().x;
      _range_x_dirty = false;
    }
    return _range_x;
  }

private:
  void trimRange()
  {
    while (_points.size() > 2 && (_points.back().x - _points.front().x) > _max_range_x)
    {
      this->popFront();
    }
  }

};

// -----------  template specializations ----------
template <>
inline void PlotDataBase<nonstd::any>::popFront()
{
  _points.pop_front();

  if( !_points.empty() )
  {
    _range_x.min = _points.front().x;
  }
}

template <>
inline void TimeseriesBase<nonstd::any>::popFront()
{
  const auto& p = _points.front();
  if( !_range_x_dirty && (p.x == _range_x.max || p.x == _range_x.min) )
  {
    _range_x_dirty = true;
  }
  _points.pop_front();
}

template <>
inline RangeOpt PlotDataBase<nonstd::any>::rangeY() const
{
  return nonstd::nullopt;
}

template <>
inline void PlotDataBase<nonstd::any>::pushBack(Point&& p)
{
  if( _points.empty() )
  {
    _range_x_dirty = false;
    _range_x.min = p.x;
    _range_x.max = p.x;
  }
  if( !_range_x_dirty )
  {
    if( p.x > _range_x.max ){
      _range_x.max = p.x;
    }
    else if( p.x < _range_x.min ){
      _range_x.min = p.x;
    }
    else{
      _range_x_dirty = true;
    }
  }
  _points.emplace_back(p);
}

template <>
inline void TimeseriesBase<nonstd::any>::pushBack(Point&& p)
{
  if( _points.empty() )
  {
    _range_x_dirty = false;
    _range_x.min = p.x;
    _range_x.max = p.x;
  }
  else {
    if( p.x < this->back().x ){
      _range_x_dirty = true;
    }
    if( !_range_x_dirty ){
      _range_x.max = p.x;
    }
  }

  _points.emplace_back(p);
  trimRange();
}



//-----------------------------------
using PlotData = TimeseriesBase<double>;
using PlotDataAny = TimeseriesBase<nonstd::any>;

struct PlotDataMapRef
{
  std::unordered_map<std::string, PlotData> numeric;
  std::unordered_map<std::string, PlotDataAny> user_defined;

  std::unordered_map<std::string, PlotData>::iterator addNumeric(const std::string& name)
  {
    return numeric.emplace(std::piecewise_construct, std::forward_as_tuple(name), std::forward_as_tuple(name)).first;
  }

  std::unordered_map<std::string, PlotDataAny>::iterator addUserDefined(const std::string& name)
  {
    return user_defined.emplace(std::piecewise_construct, std::forward_as_tuple(name), std::forward_as_tuple(name))
        .first;
  }

};

//-----------------------------------
template <typename Value>
inline void AddPrefixToPlotData(const std::string& prefix,
                                std::unordered_map<std::string, Value>& data)
{
  if (prefix.empty()){
    return;
  }

  std::unordered_map<std::string, Value> temp;

  for (auto& it : data)
  {
    std::string key;
    key.reserve(prefix.size() + 2 + it.first.size());
    if (it.first.front() == '/')
    {
      key = prefix + it.first;
    }
    else
    {
      key = prefix + "/" + it.first;
    }

    auto new_plot =
        temp.emplace(std::piecewise_construct, std::forward_as_tuple(key), std::forward_as_tuple(key)).first;

    new_plot->second.swapData(it.second);
  }
  std::swap(data, temp);
}


template <typename Value>
inline int TimeseriesBase<Value>::getIndexFromX(double x) const
{
  if (_points.size() == 0)
  {
    return -1;
  }
  auto lower = std::lower_bound(_points.begin(), _points.end(), Point(x, 0),
                                [](const Point& a, const Point& b) { return a.x < b.x; });
  auto index = std::distance(_points.begin(), lower);

  if (index >= _points.size())
  {
    return _points.size() - 1;
  }
  if (index < 0)
  {
    return 0;
  }

  if (index > 0)
  {
    if (std::abs(_points[index - 1].x - x) < std::abs(_points[index].x - x))
    {
      return index - 1;
    }
    else
    {
      return index;
    }
  }
  return index;
}

template <typename Value>
inline nonstd::optional<Value> TimeseriesBase<Value>::getYfromX(double x) const
{
  int index = getIndexFromX(x);
  if (index == -1)
  {
    return {};
  }
  return _points.at(index).y;
}

} // end namespace

#endif  // PLOTDATA_H
