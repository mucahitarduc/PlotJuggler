#pragma once
// MESSAGE ADD_FLIGHT_POINT PACKING

#define MAVLINK_MSG_ID_ADD_FLIGHT_POINT 213

MAVPACKED(
typedef struct __mavlink_add_flight_point_t {
 float x; /*<  local position x of point */
 float y; /*<  local position y of point */
 uint32_t id; /*<  id of point */
 uint32_t type; /*<  type of point */
}) mavlink_add_flight_point_t;

#define MAVLINK_MSG_ID_ADD_FLIGHT_POINT_LEN 16
#define MAVLINK_MSG_ID_ADD_FLIGHT_POINT_MIN_LEN 16
#define MAVLINK_MSG_ID_213_LEN 16
#define MAVLINK_MSG_ID_213_MIN_LEN 16

#define MAVLINK_MSG_ID_ADD_FLIGHT_POINT_CRC 163
#define MAVLINK_MSG_ID_213_CRC 163



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ADD_FLIGHT_POINT { \
    213, \
    "ADD_FLIGHT_POINT", \
    4, \
    {  { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_add_flight_point_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_add_flight_point_t, y) }, \
         { "id", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_add_flight_point_t, id) }, \
         { "type", NULL, MAVLINK_TYPE_UINT32_T, 0, 12, offsetof(mavlink_add_flight_point_t, type) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ADD_FLIGHT_POINT { \
    "ADD_FLIGHT_POINT", \
    4, \
    {  { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_add_flight_point_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_add_flight_point_t, y) }, \
         { "id", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_add_flight_point_t, id) }, \
         { "type", NULL, MAVLINK_TYPE_UINT32_T, 0, 12, offsetof(mavlink_add_flight_point_t, type) }, \
         } \
}
#endif

/**
 * @brief Pack a add_flight_point message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param x  local position x of point 
 * @param y  local position y of point 
 * @param id  id of point 
 * @param type  type of point 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_add_flight_point_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float x, float y, uint32_t id, uint32_t type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ADD_FLIGHT_POINT_LEN];
    _mav_put_float(buf, 0, x);
    _mav_put_float(buf, 4, y);
    _mav_put_uint32_t(buf, 8, id);
    _mav_put_uint32_t(buf, 12, type);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ADD_FLIGHT_POINT_LEN);
#else
    mavlink_add_flight_point_t packet;
    packet.x = x;
    packet.y = y;
    packet.id = id;
    packet.type = type;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ADD_FLIGHT_POINT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ADD_FLIGHT_POINT;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ADD_FLIGHT_POINT_MIN_LEN, MAVLINK_MSG_ID_ADD_FLIGHT_POINT_LEN, MAVLINK_MSG_ID_ADD_FLIGHT_POINT_CRC);
}

/**
 * @brief Pack a add_flight_point message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param x  local position x of point 
 * @param y  local position y of point 
 * @param id  id of point 
 * @param type  type of point 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_add_flight_point_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float x,float y,uint32_t id,uint32_t type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ADD_FLIGHT_POINT_LEN];
    _mav_put_float(buf, 0, x);
    _mav_put_float(buf, 4, y);
    _mav_put_uint32_t(buf, 8, id);
    _mav_put_uint32_t(buf, 12, type);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ADD_FLIGHT_POINT_LEN);
#else
    mavlink_add_flight_point_t packet;
    packet.x = x;
    packet.y = y;
    packet.id = id;
    packet.type = type;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ADD_FLIGHT_POINT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ADD_FLIGHT_POINT;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ADD_FLIGHT_POINT_MIN_LEN, MAVLINK_MSG_ID_ADD_FLIGHT_POINT_LEN, MAVLINK_MSG_ID_ADD_FLIGHT_POINT_CRC);
}

/**
 * @brief Encode a add_flight_point struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param add_flight_point C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_add_flight_point_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_add_flight_point_t* add_flight_point)
{
    return mavlink_msg_add_flight_point_pack(system_id, component_id, msg, add_flight_point->x, add_flight_point->y, add_flight_point->id, add_flight_point->type);
}

/**
 * @brief Encode a add_flight_point struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param add_flight_point C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_add_flight_point_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_add_flight_point_t* add_flight_point)
{
    return mavlink_msg_add_flight_point_pack_chan(system_id, component_id, chan, msg, add_flight_point->x, add_flight_point->y, add_flight_point->id, add_flight_point->type);
}

/**
 * @brief Send a add_flight_point message
 * @param chan MAVLink channel to send the message
 *
 * @param x  local position x of point 
 * @param y  local position y of point 
 * @param id  id of point 
 * @param type  type of point 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_add_flight_point_send(mavlink_channel_t chan, float x, float y, uint32_t id, uint32_t type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ADD_FLIGHT_POINT_LEN];
    _mav_put_float(buf, 0, x);
    _mav_put_float(buf, 4, y);
    _mav_put_uint32_t(buf, 8, id);
    _mav_put_uint32_t(buf, 12, type);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ADD_FLIGHT_POINT, buf, MAVLINK_MSG_ID_ADD_FLIGHT_POINT_MIN_LEN, MAVLINK_MSG_ID_ADD_FLIGHT_POINT_LEN, MAVLINK_MSG_ID_ADD_FLIGHT_POINT_CRC);
#else
    mavlink_add_flight_point_t packet;
    packet.x = x;
    packet.y = y;
    packet.id = id;
    packet.type = type;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ADD_FLIGHT_POINT, (const char *)&packet, MAVLINK_MSG_ID_ADD_FLIGHT_POINT_MIN_LEN, MAVLINK_MSG_ID_ADD_FLIGHT_POINT_LEN, MAVLINK_MSG_ID_ADD_FLIGHT_POINT_CRC);
#endif
}

/**
 * @brief Send a add_flight_point message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_add_flight_point_send_struct(mavlink_channel_t chan, const mavlink_add_flight_point_t* add_flight_point)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_add_flight_point_send(chan, add_flight_point->x, add_flight_point->y, add_flight_point->id, add_flight_point->type);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ADD_FLIGHT_POINT, (const char *)add_flight_point, MAVLINK_MSG_ID_ADD_FLIGHT_POINT_MIN_LEN, MAVLINK_MSG_ID_ADD_FLIGHT_POINT_LEN, MAVLINK_MSG_ID_ADD_FLIGHT_POINT_CRC);
#endif
}

#if MAVLINK_MSG_ID_ADD_FLIGHT_POINT_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_add_flight_point_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float x, float y, uint32_t id, uint32_t type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, x);
    _mav_put_float(buf, 4, y);
    _mav_put_uint32_t(buf, 8, id);
    _mav_put_uint32_t(buf, 12, type);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ADD_FLIGHT_POINT, buf, MAVLINK_MSG_ID_ADD_FLIGHT_POINT_MIN_LEN, MAVLINK_MSG_ID_ADD_FLIGHT_POINT_LEN, MAVLINK_MSG_ID_ADD_FLIGHT_POINT_CRC);
#else
    mavlink_add_flight_point_t *packet = (mavlink_add_flight_point_t *)msgbuf;
    packet->x = x;
    packet->y = y;
    packet->id = id;
    packet->type = type;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ADD_FLIGHT_POINT, (const char *)packet, MAVLINK_MSG_ID_ADD_FLIGHT_POINT_MIN_LEN, MAVLINK_MSG_ID_ADD_FLIGHT_POINT_LEN, MAVLINK_MSG_ID_ADD_FLIGHT_POINT_CRC);
#endif
}
#endif

#endif

// MESSAGE ADD_FLIGHT_POINT UNPACKING


/**
 * @brief Get field x from add_flight_point message
 *
 * @return  local position x of point 
 */
static inline float mavlink_msg_add_flight_point_get_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field y from add_flight_point message
 *
 * @return  local position y of point 
 */
static inline float mavlink_msg_add_flight_point_get_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field id from add_flight_point message
 *
 * @return  id of point 
 */
static inline uint32_t mavlink_msg_add_flight_point_get_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  8);
}

/**
 * @brief Get field type from add_flight_point message
 *
 * @return  type of point 
 */
static inline uint32_t mavlink_msg_add_flight_point_get_type(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  12);
}

/**
 * @brief Decode a add_flight_point message into a struct
 *
 * @param msg The message to decode
 * @param add_flight_point C-struct to decode the message contents into
 */
static inline void mavlink_msg_add_flight_point_decode(const mavlink_message_t* msg, mavlink_add_flight_point_t* add_flight_point)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    add_flight_point->x = mavlink_msg_add_flight_point_get_x(msg);
    add_flight_point->y = mavlink_msg_add_flight_point_get_y(msg);
    add_flight_point->id = mavlink_msg_add_flight_point_get_id(msg);
    add_flight_point->type = mavlink_msg_add_flight_point_get_type(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ADD_FLIGHT_POINT_LEN? msg->len : MAVLINK_MSG_ID_ADD_FLIGHT_POINT_LEN;
        memset(add_flight_point, 0, MAVLINK_MSG_ID_ADD_FLIGHT_POINT_LEN);
    memcpy(add_flight_point, _MAV_PAYLOAD(msg), len);
#endif
}
