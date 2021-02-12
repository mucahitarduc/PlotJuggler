#include "nlohmann_parsers.h"
#include <QDebug>

#define FMT_HEADER_ONLY
#include "fmt/format.h"
#include <iostream>

mavlink_status_t m_mavlink_status[MAVLINK_COMM_NUM_BUFFERS];
mavlink_system_t mavlink_system = {81,50};

bool NlohmannParser::parseMessageImpl(double timestamp)
{
    if (_use_message_stamp){

        auto ts = _json.find("/timestamp");
        if( ts != _json.end() && ts.value().is_number())
        {
           timestamp = ts.value().get<double>();
        }
    }

    std::function<void(const std::string&,  const nlohmann::json&)> flatten;

    flatten =[&](const std::string& prefix,
            const nlohmann::json& value)
    {
        if (value.empty()){
            return;
        }

        switch (value.type())
        {
        case nlohmann::detail::value_t::array:{
            // iterate array and use index as reference string
            for (std::size_t i = 0; i < value.size(); ++i) {
                flatten( fmt::format("{}[{}]", prefix, i), value[i]);
            }
            break;
        }

        case nlohmann::detail::value_t::object:{
            // iterate object and use keys as reference string
            for(const auto& element: value.items()) {
                flatten( fmt::format("{}/{}", prefix, element.key()), element.value());
            }
            break;
        }

        default:{
            double numeric_value = 0;
            if( value.is_boolean()) {
                numeric_value = value.get<bool>();
            }
            else if( value.is_number()) {
                numeric_value = value.get<double>();
            }
            else{
                return;
            }

            auto plot_data = &(getSeries(prefix));
            plot_data->pushBack( {timestamp, numeric_value} );

            break;
        }
        } // end switch
    };

    flatten(_topic_name, _json);
    return true;
}

bool MessagePack_Parser::parseMessage(const MessageRef msg,
                                      double timestamp)
{
    _json = nlohmann::json::from_msgpack( msg.data(),
                                          msg.data() + msg.size() );
    return parseMessageImpl(timestamp);
}

bool Mavlink_Parser::parseMessage(const MessageRef msg,
                                      double timestamp)
{
    data_.append(msg.data(), msg.data() + msg.size());
    bool handled = false;
    for (auto byte_pos = data_.begin(); byte_pos != data_.end(); ++byte_pos) {
        mavlink_status_t status_;
        mavlink_message_t msg_;
        uint8_t decodeState = mavlink_parse_char(mavlink_channel_, *byte_pos, &msg_, &status_);

        if (decodeState == 1) {
            handleMavlinkMessage(&msg_, timestamp);
            handled = true;
            data_ = "";
        }
    }
    if (handled == false) {
        return true;
    }
    _json = nlohmann::json::from_msgpack( msg.data(),
                                          msg.data() + msg.size() );
    std::cout <<"Deneme\n";
    std::cout << data_.c_str() << "\n";
    return parseMessageImpl(timestamp);
}

void Mavlink_Parser::handleMavlinkMessage(mavlink_message_t* msg, double timestamp) {
    switch (msg->msgid) {

    case MAVLINK_MSG_ID_DISTANCE_SENSOR:
        //processDistanceSensorMessage(msg);
        break;
    case MAVLINK_MSG_ID_OPTICAL_FLOW_RAD:
        //processOpticalFlowMessage(msg);
        break;
    case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
        // Local position message.
            
        mavlink_local_position_ned_t localPosition;
        mavlink_msg_local_position_ned_decode(msg, &localPosition);

        // _json = nlohmann::json{{"time_boot_ms", timestamp}, 
        //                        {"x", localPosition.x}, 
        //                        {"y", localPosition.y}, 
        //                        {"z", localPosition.z}, 
        //                        {"vx", localPosition.vx}, 
        //                        {"vy", localPosition.vy}, 
        //                        {"vz", localPosition.vz}};
        //processLocalPositionMessage(msg);

    default:
        const mavlink_message_info_t* msgInfo = mavlink_get_message_info(msg);

        std::string messageName = msgInfo->name;
        nlohmann::json js_{{"timestamp",timestamp}};
        uint8_t* m = reinterpret_cast<uint8_t*>(&msg->payload64[0]);

        for (unsigned int i = 0; i < msgInfo->num_fields; ++i)
        {
            std::string fieldName = msgInfo->fields[i].name;
            switch (msgInfo->fields[i].type) {
            case MAVLINK_TYPE_CHAR:
                // ignore char fields

                /*if (msgInfo->fields[i].array_length > 0) {
                    char* str = reinterpret_cast<char*>(m+ msgInfo->fields[i].wire_offset);
                    // Enforce null termination
                    str[msgInfo->fields[i].array_length - 1] = '\0';
                    std::string v(str);
                    f->updateValue(v);
                } else {
                    // Single char
                    char b = *(reinterpret_cast<char*>(m + msgInfo->fields[i].wire_offset));
                    std::string v(b);
                    f->updateValue(v);
                }*/
                break;
            case MAVLINK_TYPE_UINT8_T:
                if (msgInfo->fields[i].array_length > 0) {
                    uint8_t* nums = m + msgInfo->fields[i].wire_offset;

                    for (unsigned int j = 0; j < msgInfo->fields[i].array_length; ++j) {
                        std::string const& arrayFieldName = generateArrayFieldName(fieldName, j);
                        setFieldAndValue<uint8_t>(generateFieldName(messageName, arrayFieldName), nums[j], js_);

                        
                    }

                }
                else {
                    // Single value
                    uint8_t u = *(m + msgInfo->fields[i].wire_offset);
                    setFieldAndValue<uint8_t>(generateFieldName(messageName, fieldName), u, js_);
                }
                break;
            case MAVLINK_TYPE_INT8_T:
                if (msgInfo->fields[i].array_length > 0) {
                    int8_t* nums = reinterpret_cast<int8_t*>(m + msgInfo->fields[i].wire_offset);

                    for (unsigned int j = 0; j < msgInfo->fields[i].array_length; ++j) {
                        std::string const& arrayFieldName = generateArrayFieldName(fieldName, j);
                        setFieldAndValue<int8_t>(generateFieldName(messageName, arrayFieldName), nums[j],js_);
                    }
                }
                else {
                    // Single value
                    int8_t n = *(reinterpret_cast<int8_t*>(m + msgInfo->fields[i].wire_offset));
                    setFieldAndValue<int8_t>(generateFieldName(messageName, fieldName), n, js_);
                }
                break;
            case MAVLINK_TYPE_UINT16_T:
                if (msgInfo->fields[i].array_length > 0) {
                    uint16_t* nums = reinterpret_cast<uint16_t*>(m + msgInfo->fields[i].wire_offset);

                    for (unsigned int j = 0; j < msgInfo->fields[i].array_length; ++j) {
                        std::string const& arrayFieldName = generateArrayFieldName(fieldName, j);
                        setFieldAndValue<uint16_t>(generateFieldName(messageName, arrayFieldName), nums[j], js_);
                    }
                }
                else {
                    // Single value
                    uint16_t n = *(reinterpret_cast<uint16_t*>(m + msgInfo->fields[i].wire_offset));
                    setFieldAndValue<uint16_t>(generateFieldName(messageName, fieldName), n, js_);
                }
                break;
            case MAVLINK_TYPE_INT16_T:
                if (msgInfo->fields[i].array_length > 0) {
                    int16_t* nums = reinterpret_cast<int16_t*>(m + msgInfo->fields[i].wire_offset);

                    for (unsigned int j = 0; j < msgInfo->fields[i].array_length; ++j) {
                        std::string const& arrayFieldName = generateArrayFieldName(fieldName, j);
                        setFieldAndValue<int16_t>(generateFieldName(messageName, arrayFieldName), nums[j], js_);
                    }
                }
                else {
                    // Single value
                    int16_t n = *(reinterpret_cast<int16_t*>(m + msgInfo->fields[i].wire_offset));
                    setFieldAndValue<int16_t>(generateFieldName(messageName, fieldName), n, js_);
                }
                break;
            case MAVLINK_TYPE_UINT32_T:
                if (msgInfo->fields[i].array_length > 0) {
                    uint32_t* nums = reinterpret_cast<uint32_t*>(m + msgInfo->fields[i].wire_offset);

                    for (unsigned int j = 0; j < msgInfo->fields[i].array_length; ++j) {
                        std::string const& arrayFieldName = generateArrayFieldName(fieldName, j);
                        setFieldAndValue<uint32_t>(generateFieldName(messageName, arrayFieldName), nums[j], js_);
                    }
                }
                else {
                    // Single value
                    uint32_t n = *(reinterpret_cast<uint32_t*>(m + msgInfo->fields[i].wire_offset));
                    setFieldAndValue<uint32_t>(generateFieldName(messageName, fieldName), n, js_);
                }
                break;
            case MAVLINK_TYPE_INT32_T:
                if (msgInfo->fields[i].array_length > 0) {
                    int32_t* nums = reinterpret_cast<int32_t*>(m + msgInfo->fields[i].wire_offset);

                    for (unsigned int j = 0; j < msgInfo->fields[i].array_length; ++j) {
                        std::string const& arrayFieldName = generateArrayFieldName(fieldName, j);
                        setFieldAndValue<int32_t>(generateFieldName(messageName, arrayFieldName), nums[j], js_);
                    }
                }
                else {
                    // Single value
                    int32_t n = *(reinterpret_cast<int32_t*>(m + msgInfo->fields[i].wire_offset));
                    setFieldAndValue<int32_t>(generateFieldName(messageName, fieldName), n, js_);
                }
                break;
            case MAVLINK_TYPE_FLOAT:
                if (msgInfo->fields[i].array_length > 0) {
                    float* nums = reinterpret_cast<float*>(m + msgInfo->fields[i].wire_offset);

                    for (unsigned int j = 0; j < msgInfo->fields[i].array_length; ++j) {
                        std::string const& arrayFieldName = generateArrayFieldName(fieldName, j);
                        setFieldAndValue<float>(generateFieldName(messageName, arrayFieldName), nums[j], js_);
                    }
                }
                else {
                    // Single value
                    float fv = *(reinterpret_cast<float*>(m + msgInfo->fields[i].wire_offset));
                    setFieldAndValue<float>(generateFieldName(messageName, fieldName), fv, js_);
                }
                break;
            case MAVLINK_TYPE_DOUBLE:
                if (msgInfo->fields[i].array_length > 0) {
                    double* nums = reinterpret_cast<double*>(m + msgInfo->fields[i].wire_offset);

                    for (unsigned int j = 0; j < msgInfo->fields[i].array_length; ++j) {
                        std::string const& arrayFieldName = generateArrayFieldName(fieldName, j);
                        setFieldAndValue<double>(generateFieldName(messageName, arrayFieldName), nums[j], js_);
                    }
                }
                else {
                    // Single value
                    double d = *(reinterpret_cast<double*>(m + msgInfo->fields[i].wire_offset));
                    setFieldAndValue<double>(generateFieldName(messageName, fieldName), d, js_);
                }
                break;
            case MAVLINK_TYPE_UINT64_T:
                if (msgInfo->fields[i].array_length > 0) {
                    uint64_t* nums = reinterpret_cast<uint64_t*>(m + msgInfo->fields[i].wire_offset);

                    for (unsigned int j = 0; j < msgInfo->fields[i].array_length; ++j) {
                        std::string const& arrayFieldName = generateArrayFieldName(fieldName, j);
                        setFieldAndValue<uint64_t>(generateFieldName(messageName, arrayFieldName), nums[j], js_);
                    }
                }
                else {
                    // Single value
                    uint64_t n = *(reinterpret_cast<uint64_t*>(m + msgInfo->fields[i].wire_offset));
                    setFieldAndValue<uint64_t>(generateFieldName(messageName, fieldName), n, js_);
                }
                break;
            case MAVLINK_TYPE_INT64_T:
                if (msgInfo->fields[i].array_length > 0) {
                    int64_t* nums = reinterpret_cast<int64_t*>(m + msgInfo->fields[i].wire_offset);

                    for (unsigned int j = 0; j < msgInfo->fields[i].array_length; ++j) {
                        std::string const& arrayFieldName = generateArrayFieldName(fieldName, j);
                        setFieldAndValue<int64_t>(generateFieldName(messageName, arrayFieldName), nums[j], js_);
                    }
                }
                else {
                    // Single value
                    int64_t n = *(reinterpret_cast<int64_t*>(m + msgInfo->fields[i].wire_offset));
                    setFieldAndValue<int64_t>(generateFieldName(messageName, fieldName), n, js_);
                }
                break;

            default:
                break;
            }
        }

        _json = nlohmann::json{{messageName.c_str(),js_}};
        break;
    }
}

bool JSON_Parser::parseMessage(const MessageRef msg,
                               double timestamp)
{
    _json = nlohmann::json::parse( msg.data(),
                                   msg.data()+ msg.size());
    return parseMessageImpl(timestamp);
}

bool CBOR_Parser::parseMessage(const MessageRef msg,
                               double timestamp)
{
    _json = nlohmann::json::from_cbor( msg.data(),
                                       msg.data()+ msg.size() );
    return parseMessageImpl(timestamp);
}

bool BSON_Parser::parseMessage(const MessageRef msg,
                               double timestamp)
{
    _json = nlohmann::json::from_bson( msg.data(),
                                       msg.data()+ msg.size() );
    return parseMessageImpl(timestamp);
}
