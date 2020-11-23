#include "nlohmann_parsers.h"

#define FMT_HEADER_ONLY
#include "fmt/format.h"

bool NlohmannParser::parseMessageImpl(double timestamp)
{
    if (_use_message_stamp){

        auto ts = _json.find("/timestamp");
        if( ts != _json.end() && ts.value().is_number())
        {
           timestamp = ts.value().get<double>();
        }
    }

    std::string key;
    for(const auto& el: _json.items())
    {
        double value = 0;
        if( el.value().is_boolean())
        {
            value = el.value().get<bool>();
        }
        else if( el.value().is_number())
        {
            value = el.value().get<double>();
        }
        else{
            continue;
        }
        key = fmt::format("{}/{}", _topic_name, el.key() );
        auto plot_data = &(getSeries(key));
        plot_data->pushBack( {timestamp, value} );
    }
    return true;
}

bool MessagePack_Parser::parseMessage(const MessageRef msg,
                                      double timestamp)
{
    _json = nlohmann::json::from_msgpack( msg.data(),
                                          msg.data() + msg.size() ).flatten();
    return parseMessageImpl(timestamp);
}

bool JSON_Parser::parseMessage(const MessageRef msg,
                               double timestamp)
{
    _json = nlohmann::json::parse( msg.data(),
                                   msg.data()+ msg.size()).flatten();
    return parseMessageImpl(timestamp);
}

bool CBOR_Parser::parseMessage(const MessageRef msg,
                               double timestamp)
{
    _json = nlohmann::json::from_cbor( msg.data(),
                                       msg.data()+ msg.size() ).flatten();
    return parseMessageImpl(timestamp);
}

bool BSON_Parser::parseMessage(const MessageRef msg,
                               double timestamp)
{
    _json = nlohmann::json::from_bson( msg.data(),
                                       msg.data()+ msg.size() ).flatten();
    return parseMessageImpl(timestamp);
}
