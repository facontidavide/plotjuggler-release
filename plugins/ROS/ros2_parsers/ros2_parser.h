#pragma once

#include <string>
#include <unordered_map>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rmw/rmw.h"
#include "rmw/types.h"
#include "rosbag2/typesupport_helpers.hpp"
#include "rosbag2/types/introspection_message.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "ros2_introspection/ros2_introspection.hpp"

struct TopicInfo{
    TopicInfo(const std::string& type)
    {
        topic_type = type;
        const auto typesupport_identifier   = rosidl_typesupport_cpp::typesupport_identifier;
        const auto introspection_identifier = rosidl_typesupport_introspection_cpp::typesupport_identifier;

        introspection_support = rosbag2::get_typesupport(type, introspection_identifier);
        type_support = rosbag2::get_typesupport(type, typesupport_identifier);
        has_header_stamp = Ros2Introspection::TypeHasHeader( introspection_support );
        buffer = rosbag2::allocate_introspection_message(introspection_support, &allocator);
    }

    std::string topic_type;
    bool has_header_stamp;
    std::shared_ptr<rosbag2_introspection_message_t> buffer;
    const rosidl_message_type_support_t *introspection_support;
    const rosidl_message_type_support_t *type_support;
    Ros2Introspection::FlatMessage flat_msg;
    Ros2Introspection::RenamedValues renamed;

    static rcutils_allocator_t allocator;
};



