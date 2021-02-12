#pragma once
// MESSAGE SET_VISION_FUNCTION_ENABLED PACKING

#define MAVLINK_MSG_ID_SET_VISION_FUNCTION_ENABLED 206

MAVPACKED(
typedef struct __mavlink_set_vision_function_enabled_t {
 uint32_t vision_function; /*<  enum VISION_FUNCTION_TYPE */
 uint32_t function_param; /*<  vision function parameter */
 uint8_t enable; /*<  boolean field. 1 to enable, 0 to disable */
}) mavlink_set_vision_function_enabled_t;

#define MAVLINK_MSG_ID_SET_VISION_FUNCTION_ENABLED_LEN 9
#define MAVLINK_MSG_ID_SET_VISION_FUNCTION_ENABLED_MIN_LEN 9
#define MAVLINK_MSG_ID_206_LEN 9
#define MAVLINK_MSG_ID_206_MIN_LEN 9

#define MAVLINK_MSG_ID_SET_VISION_FUNCTION_ENABLED_CRC 93
#define MAVLINK_MSG_ID_206_CRC 93



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_SET_VISION_FUNCTION_ENABLED { \
    206, \
    "SET_VISION_FUNCTION_ENABLED", \
    3, \
    {  { "vision_function", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_set_vision_function_enabled_t, vision_function) }, \
         { "function_param", NULL, MAVLINK_TYPE_UINT32_T, 0, 4, offsetof(mavlink_set_vision_function_enabled_t, function_param) }, \
         { "enable", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_set_vision_function_enabled_t, enable) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_SET_VISION_FUNCTION_ENABLED { \
    "SET_VISION_FUNCTION_ENABLED", \
    3, \
    {  { "vision_function", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_set_vision_function_enabled_t, vision_function) }, \
         { "function_param", NULL, MAVLINK_TYPE_UINT32_T, 0, 4, offsetof(mavlink_set_vision_function_enabled_t, function_param) }, \
         { "enable", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_set_vision_function_enabled_t, enable) }, \
         } \
}
#endif

/**
 * @brief Pack a set_vision_function_enabled message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param enable  boolean field. 1 to enable, 0 to disable 
 * @param vision_function  enum VISION_FUNCTION_TYPE 
 * @param function_param  vision function parameter 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_set_vision_function_enabled_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t enable, uint32_t vision_function, uint32_t function_param)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SET_VISION_FUNCTION_ENABLED_LEN];
    _mav_put_uint32_t(buf, 0, vision_function);
    _mav_put_uint32_t(buf, 4, function_param);
    _mav_put_uint8_t(buf, 8, enable);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SET_VISION_FUNCTION_ENABLED_LEN);
#else
    mavlink_set_vision_function_enabled_t packet;
    packet.vision_function = vision_function;
    packet.function_param = function_param;
    packet.enable = enable;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SET_VISION_FUNCTION_ENABLED_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SET_VISION_FUNCTION_ENABLED;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SET_VISION_FUNCTION_ENABLED_MIN_LEN, MAVLINK_MSG_ID_SET_VISION_FUNCTION_ENABLED_LEN, MAVLINK_MSG_ID_SET_VISION_FUNCTION_ENABLED_CRC);
}

/**
 * @brief Pack a set_vision_function_enabled message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param enable  boolean field. 1 to enable, 0 to disable 
 * @param vision_function  enum VISION_FUNCTION_TYPE 
 * @param function_param  vision function parameter 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_set_vision_function_enabled_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t enable,uint32_t vision_function,uint32_t function_param)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SET_VISION_FUNCTION_ENABLED_LEN];
    _mav_put_uint32_t(buf, 0, vision_function);
    _mav_put_uint32_t(buf, 4, function_param);
    _mav_put_uint8_t(buf, 8, enable);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SET_VISION_FUNCTION_ENABLED_LEN);
#else
    mavlink_set_vision_function_enabled_t packet;
    packet.vision_function = vision_function;
    packet.function_param = function_param;
    packet.enable = enable;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SET_VISION_FUNCTION_ENABLED_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SET_VISION_FUNCTION_ENABLED;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SET_VISION_FUNCTION_ENABLED_MIN_LEN, MAVLINK_MSG_ID_SET_VISION_FUNCTION_ENABLED_LEN, MAVLINK_MSG_ID_SET_VISION_FUNCTION_ENABLED_CRC);
}

/**
 * @brief Encode a set_vision_function_enabled struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param set_vision_function_enabled C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_set_vision_function_enabled_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_set_vision_function_enabled_t* set_vision_function_enabled)
{
    return mavlink_msg_set_vision_function_enabled_pack(system_id, component_id, msg, set_vision_function_enabled->enable, set_vision_function_enabled->vision_function, set_vision_function_enabled->function_param);
}

/**
 * @brief Encode a set_vision_function_enabled struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param set_vision_function_enabled C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_set_vision_function_enabled_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_set_vision_function_enabled_t* set_vision_function_enabled)
{
    return mavlink_msg_set_vision_function_enabled_pack_chan(system_id, component_id, chan, msg, set_vision_function_enabled->enable, set_vision_function_enabled->vision_function, set_vision_function_enabled->function_param);
}

/**
 * @brief Send a set_vision_function_enabled message
 * @param chan MAVLink channel to send the message
 *
 * @param enable  boolean field. 1 to enable, 0 to disable 
 * @param vision_function  enum VISION_FUNCTION_TYPE 
 * @param function_param  vision function parameter 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_set_vision_function_enabled_send(mavlink_channel_t chan, uint8_t enable, uint32_t vision_function, uint32_t function_param)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SET_VISION_FUNCTION_ENABLED_LEN];
    _mav_put_uint32_t(buf, 0, vision_function);
    _mav_put_uint32_t(buf, 4, function_param);
    _mav_put_uint8_t(buf, 8, enable);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_VISION_FUNCTION_ENABLED, buf, MAVLINK_MSG_ID_SET_VISION_FUNCTION_ENABLED_MIN_LEN, MAVLINK_MSG_ID_SET_VISION_FUNCTION_ENABLED_LEN, MAVLINK_MSG_ID_SET_VISION_FUNCTION_ENABLED_CRC);
#else
    mavlink_set_vision_function_enabled_t packet;
    packet.vision_function = vision_function;
    packet.function_param = function_param;
    packet.enable = enable;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_VISION_FUNCTION_ENABLED, (const char *)&packet, MAVLINK_MSG_ID_SET_VISION_FUNCTION_ENABLED_MIN_LEN, MAVLINK_MSG_ID_SET_VISION_FUNCTION_ENABLED_LEN, MAVLINK_MSG_ID_SET_VISION_FUNCTION_ENABLED_CRC);
#endif
}

/**
 * @brief Send a set_vision_function_enabled message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_set_vision_function_enabled_send_struct(mavlink_channel_t chan, const mavlink_set_vision_function_enabled_t* set_vision_function_enabled)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_set_vision_function_enabled_send(chan, set_vision_function_enabled->enable, set_vision_function_enabled->vision_function, set_vision_function_enabled->function_param);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_VISION_FUNCTION_ENABLED, (const char *)set_vision_function_enabled, MAVLINK_MSG_ID_SET_VISION_FUNCTION_ENABLED_MIN_LEN, MAVLINK_MSG_ID_SET_VISION_FUNCTION_ENABLED_LEN, MAVLINK_MSG_ID_SET_VISION_FUNCTION_ENABLED_CRC);
#endif
}

#if MAVLINK_MSG_ID_SET_VISION_FUNCTION_ENABLED_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_set_vision_function_enabled_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t enable, uint32_t vision_function, uint32_t function_param)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, vision_function);
    _mav_put_uint32_t(buf, 4, function_param);
    _mav_put_uint8_t(buf, 8, enable);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_VISION_FUNCTION_ENABLED, buf, MAVLINK_MSG_ID_SET_VISION_FUNCTION_ENABLED_MIN_LEN, MAVLINK_MSG_ID_SET_VISION_FUNCTION_ENABLED_LEN, MAVLINK_MSG_ID_SET_VISION_FUNCTION_ENABLED_CRC);
#else
    mavlink_set_vision_function_enabled_t *packet = (mavlink_set_vision_function_enabled_t *)msgbuf;
    packet->vision_function = vision_function;
    packet->function_param = function_param;
    packet->enable = enable;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_VISION_FUNCTION_ENABLED, (const char *)packet, MAVLINK_MSG_ID_SET_VISION_FUNCTION_ENABLED_MIN_LEN, MAVLINK_MSG_ID_SET_VISION_FUNCTION_ENABLED_LEN, MAVLINK_MSG_ID_SET_VISION_FUNCTION_ENABLED_CRC);
#endif
}
#endif

#endif

// MESSAGE SET_VISION_FUNCTION_ENABLED UNPACKING


/**
 * @brief Get field enable from set_vision_function_enabled message
 *
 * @return  boolean field. 1 to enable, 0 to disable 
 */
static inline uint8_t mavlink_msg_set_vision_function_enabled_get_enable(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  8);
}

/**
 * @brief Get field vision_function from set_vision_function_enabled message
 *
 * @return  enum VISION_FUNCTION_TYPE 
 */
static inline uint32_t mavlink_msg_set_vision_function_enabled_get_vision_function(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field function_param from set_vision_function_enabled message
 *
 * @return  vision function parameter 
 */
static inline uint32_t mavlink_msg_set_vision_function_enabled_get_function_param(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  4);
}

/**
 * @brief Decode a set_vision_function_enabled message into a struct
 *
 * @param msg The message to decode
 * @param set_vision_function_enabled C-struct to decode the message contents into
 */
static inline void mavlink_msg_set_vision_function_enabled_decode(const mavlink_message_t* msg, mavlink_set_vision_function_enabled_t* set_vision_function_enabled)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    set_vision_function_enabled->vision_function = mavlink_msg_set_vision_function_enabled_get_vision_function(msg);
    set_vision_function_enabled->function_param = mavlink_msg_set_vision_function_enabled_get_function_param(msg);
    set_vision_function_enabled->enable = mavlink_msg_set_vision_function_enabled_get_enable(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_SET_VISION_FUNCTION_ENABLED_LEN? msg->len : MAVLINK_MSG_ID_SET_VISION_FUNCTION_ENABLED_LEN;
        memset(set_vision_function_enabled, 0, MAVLINK_MSG_ID_SET_VISION_FUNCTION_ENABLED_LEN);
    memcpy(set_vision_function_enabled, _MAV_PAYLOAD(msg), len);
#endif
}
