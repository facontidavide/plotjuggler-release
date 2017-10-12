#ifndef SHAPE_SHIFTER_FACTORY_HPP
#define SHAPE_SHIFTER_FACTORY_HPP

#include <ros_type_introspection/utils/shape_shifter.hpp>
#include "PlotJuggler/any.hpp"


class RosIntrospectionFactory{

public:
  static RosIntrospectionFactory &get();

  static void registerMessage(const std::string& topic_name, const std::string &md5sum, const std::string& datatype, const std::string& definition );

  static const RosIntrospection::ShapeShifter* getShapeShifter(const std::string& topic_name);

  static const std::set<std::string> &getTopicList();

  static RosIntrospection::Parser& parser()
  {
      return get()._parser;
  }

  static bool isRegistered(const std::string& topic_name) { return get()._topics.count(topic_name) != 0; }

private:
  RosIntrospectionFactory() = default;
  std::map<std::string, RosIntrospection::ShapeShifter> _ss_map;
  std::set<std::string> _topics;
  RosIntrospection::Parser _parser;

};
//---------------------------------------------

inline RosIntrospectionFactory& RosIntrospectionFactory::get()
{
  static RosIntrospectionFactory instance;
  return instance;
}

// return true if added
inline void RosIntrospectionFactory::registerMessage(const std::string &topic_name,
                                                 const std::string &md5sum,
                                                 const std::string &datatype,
                                                 const std::string &definition)
{
    auto& instance = get();
    if( instance._ss_map.find(topic_name) == instance._ss_map.end() )
    {
        RosIntrospection::ShapeShifter msg;
        msg.morph(md5sum, datatype,definition);
        instance._ss_map.insert( std::make_pair(topic_name, std::move(msg) ));
        instance._topics.insert( topic_name );
        parser().registerMessageDefinition( topic_name, RosIntrospection::ROSType(datatype), definition);
    }
}

inline const RosIntrospection::ShapeShifter* RosIntrospectionFactory::getShapeShifter(const std::string &topic_name)
{
    auto& instance = get();
    auto it = instance._ss_map.find( topic_name );
    return ( it == instance._ss_map.end()) ? nullptr :  &(it->second);
}

inline const std::set<std::string> &RosIntrospectionFactory::getTopicList()
{
  return get()._topics;
}

#endif // SHAPE_SHIFTER_FACTORY_HPP



