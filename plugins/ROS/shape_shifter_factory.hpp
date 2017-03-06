#ifndef SHAPE_SHIFTER_FACTORY_HPP
#define SHAPE_SHIFTER_FACTORY_HPP

#include <ros_type_introspection/shape_shifter.hpp>
#include <PlotJuggler/any.hpp>

class ShapeShifterFactory{
public:
  static ShapeShifterFactory &getInstance();

  void registerMessage(const std::string& topic_name, const std::string &md5sum, const std::string& datatype, const std::string& definition );

  nonstd::optional<RosIntrospection::ShapeShifter *> getMessage(const std::string& topic_name);

  const std::vector<std::string>& getTopicList() const;

private:
  ShapeShifterFactory() = default;
  std::map<std::string, RosIntrospection::ShapeShifter> map_;
  std::vector<std::string> topics_;

};

#endif // SHAPE_SHIFTER_FACTORY_HPP

inline ShapeShifterFactory& ShapeShifterFactory::getInstance()
{
  static ShapeShifterFactory instance;
  return instance;
}

inline void ShapeShifterFactory::registerMessage(const std::string &topic_name,
                                                 const std::string &md5sum,
                                                 const std::string &datatype,
                                                 const std::string &definition)
{

  auto it = map_.find(topic_name);
  if( it == map_.end() )
  {
      RosIntrospection::ShapeShifter msg;
      msg.morph(md5sum, datatype,definition);
      map_.insert( std::make_pair(topic_name, std::move(msg) ));
      topics_.push_back( topic_name );
  }
}

inline nonstd::optional<RosIntrospection::ShapeShifter*> ShapeShifterFactory::getMessage(const std::string &topic_name)
{
  auto it = map_.find( topic_name );
  if( it == map_.end())
    return nonstd::optional<RosIntrospection::ShapeShifter*>();
  else
    return nonstd::optional<RosIntrospection::ShapeShifter*>( &(it->second) );
}

inline const std::vector<std::string> &ShapeShifterFactory::getTopicList() const
{
  return topics_;
}


