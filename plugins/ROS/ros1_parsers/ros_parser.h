#ifndef INTROSPECTIONPARSER_H
#define INTROSPECTIONPARSER_H

#include "ros_parser_base.h"
#include <ros_type_introspection/ros_introspection.hpp>
#include "marl/ticket.h"

class RosMessageParser : public RosParserBase
{
public:
    RosMessageParser();

    void clear();

    void setMaxArrayPolicy(size_t max_array_size,
                           bool discard_entire_array);

    void addRules( const RosIntrospection::SubstitutionRuleMap& rules)
    {
        for(const auto& it: rules)
        {
            _introspection_parser->registerRenamingRules(
                        RosIntrospection::ROSType(it.first) ,
                        it.second );
        }
    }

    bool registerSchema(const std::string& topic_name,
                        const std::string& md5sum,
                        RosIntrospection::ROSType type,
                        const std::string& definition);

    virtual const std::unordered_set<std::string>& getCompatibleKeys() const override
    {
        return _registered_md5sum;
    }


    void pushMessageRef(const std::string& topic_name,
                        const MessageRef& msg,
                        double timestamp) override;

    void showWarnings();

    virtual void extractData(PlotDataMapRef& destination,
                             const std::string& prefix) override;

    typedef std::unordered_map<std::string, std::unique_ptr<RosParserBase> > ParsersMap;

    marl::Ticket::Queue ticket_queue;

private:
    std::unordered_set<std::string> _registered_md5sum;
    std::unique_ptr<RosIntrospection::Parser> _introspection_parser;
    PlotDataMapRef _plot_map;

    ParsersMap _builtin_parsers;

    RosIntrospection::FlatMessage _flat_container;
    RosIntrospection::RenamedValues _renamed_values;

    uint32_t _max_array_size;
    bool _warnings_enabled;
    bool _discard_large_array;

    std::unordered_set<std::string> _warn_cancellation;
    std::unordered_set<std::string> _warn_max_arraysize;

    double extractRealValue( const RosIntrospection::Variant& value,
                             const std::string& item_name);
};

#endif // INTROSPECTIONPARSER_H
