#ifndef SHAPE_SHIFTER_FACTORY_HPP
#define SHAPE_SHIFTER_FACTORY_HPP

#include <ros_type_introspection/shape_shifter.hpp>
#include <PlotJuggler/any.hpp>

class RosIntrospectionFactory{
public:
  static RosIntrospectionFactory &get();

  bool registerMessage(const std::string& topic_name, const std::string &md5sum, const std::string& datatype, const std::string& definition );

  const RosIntrospection::ShapeShifter* getShapeShifter(const std::string& topic_name);

  const RosIntrospection::ROSTypeList*  getRosTypeList(const std::string& md5sum);

  const std::vector<std::string>& getTopicList() const;

private:
  RosIntrospectionFactory() = default;
  std::map<std::string, RosIntrospection::ShapeShifter> _ss_map;
  std::map<std::string, RosIntrospection::ROSTypeList> _tl_map;
  std::vector<std::string> topics_;

};
//---------------------------------------------

inline RosIntrospectionFactory& RosIntrospectionFactory::get()
{
  static RosIntrospectionFactory instance;
  return instance;
}

// return true if added
inline bool RosIntrospectionFactory::registerMessage(const std::string &topic_name,
                                                 const std::string &md5sum,
                                                 const std::string &datatype,
                                                 const std::string &definition)
{

  auto itA = _ss_map.find(topic_name);
  if( itA == _ss_map.end() )
  {
      RosIntrospection::ShapeShifter msg;
      msg.morph(md5sum, datatype,definition);
      _ss_map.insert( std::make_pair(topic_name, std::move(msg) ));
      topics_.push_back( topic_name );

      if( _tl_map.find(md5sum) == _tl_map.end())
      {
          auto topic_map = RosIntrospection::buildROSTypeMapFromDefinition( datatype, definition);
          _tl_map.insert( std::make_pair(md5sum,topic_map));
      }
      return true;
  }
  return false;
}

inline const RosIntrospection::ShapeShifter* RosIntrospectionFactory::getShapeShifter(const std::string &topic_name)
{
  auto it = _ss_map.find( topic_name );
  return ( it == _ss_map.end()) ? nullptr :  &(it->second);
}

inline const RosIntrospection::ROSTypeList* RosIntrospectionFactory::getRosTypeList(const std::string &md5sum)
{
  auto it = _tl_map.find( md5sum );
  return ( it == _tl_map.end()) ? nullptr :  &(it->second);
}

inline const std::vector<std::string> &RosIntrospectionFactory::getTopicList() const
{
  return topics_;
}

#endif // SHAPE_SHIFTER_FACTORY_HPP



