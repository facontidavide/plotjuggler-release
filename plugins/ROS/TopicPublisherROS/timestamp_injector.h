#ifndef TIMESTAMP_INJECTOR_H
#define TIMESTAMP_INJECTOR_H

#include <ros_type_introspection/deserializer.hpp>
#include <ros/time.h>


void injectTime(const RosIntrospection::ROSTypeList& type_map,
                RosIntrospection::ROSType type,
                uint8_t *buffer_ptr,
                const ros::Time& new_timestamp);

#endif // TIMESTAMP_INJECTOR_H
