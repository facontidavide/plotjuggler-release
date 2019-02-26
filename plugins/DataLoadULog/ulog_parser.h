#ifndef ULOG_PARSER_H
#define ULOG_PARSER_H

#include <iostream>
#include <vector>
#include <map>
#include <set>

#include "string_view.hpp"

typedef  nonstd::string_view StringView;

class ULogParser
{


public:

    enum FormatType{
        UINT8, UINT16, UINT32, UINT64,
        INT8, INT16, INT32, INT64,
        FLOAT, DOUBLE,
        BOOL, CHAR
    };

    struct Field{
        Field(): array_size(1) {}
        FormatType type;
        std::string field_name;
        int array_size;
    };

    struct Format
    {
        Format(): padding(0) {}
        std::string name;
        std::vector<Field> fields;
        int padding;
        size_t fieldsCount() const;
    };

    struct Subscription
    {
        Subscription(): msg_id(0), multi_id(0), format(nullptr) {}

        uint16_t msg_id;
        uint8_t multi_id;
        std::string message_name;
        const Format* format;
    };

    struct Timeseries{
        std::vector<uint64_t> timestamps;
        std::vector<std::vector<double>> data;
    };

public:

    ULogParser(const std::string& filename);

    const std::map<const Subscription*, Timeseries> &getData();

private:
    bool readFileHeader(std::ifstream &file);

    bool readFileDefinitions(std::ifstream &file);

    bool readFormat(std::ifstream &file, uint16_t msg_size);

    bool readFlagBits(std::ifstream &file, uint16_t msg_size);

    bool readParameter(std::ifstream &file, uint16_t msg_size);

    bool readSubscription(std::ifstream &file, uint16_t msg_size);

    uint64_t _file_start_time;

    std::vector<uint8_t> _read_buffer;

    std::streampos _data_section_start; ///< first ADD_LOGGED_MSG message

    int64_t _read_until_file_position = 1ULL << 60; ///< read limit if log contains appended data

    std::set<std::string> _overridden_params;  

    std::map<std::string, Format> _formats;

    std::map<uint16_t,Subscription> _subscriptions;

    std::map<const Subscription*, Timeseries> _timeseries;

    std::vector<StringView> splitString(const StringView& strToSplit, char delimeter);

};

#endif // ULOG_PARSER_H
