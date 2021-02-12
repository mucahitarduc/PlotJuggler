#pragma once
// MESSAGE OTHER_PLATFORM_POSITION PACKING

#define MAVLINK_MSG_ID_OTHER_PLATFORM_POSITION 211

MAVPACKED(
typedef struct __mavlink_other_platform_position_t {
 int32_t latitude; /*<  latitude of the other platform in degrees * 1E7*/
 int32_t longitude; /*<  longitude of the other platform in degrees * 1E7*/
 int32_t altitude; /*<  altitude of the other platform in MSL meters */
}) mavlink_other_platform_position_t;

#define MAVLINK_MSG_ID_OTHER_PLATFORM_POSITION_LEN 12
#define MAVLINK_MSG_ID_OTHER_PLATFORM_POSITION_MIN_LEN 12
#define MAVLINK_MSG_ID_211_LEN 12
#define MAVLINK_MSG_ID_211_MIN_LEN 12

#define MAVLINK_MSG_ID_OTHER_PLATFORM_POSITION_CRC 220
#define MAVLINK_MSG_ID_211_CRC 220



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_OTHER_PLATFORM_POSITION { \
    211, \
    "OTHER_PLATFORM_POSITION", \
    3, \
    {  { "latitude", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_other_platform_position_t, latitude) }, \
         { "longitude", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_other_platform_position_t, longitude) }, \
         { "altitude", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_other_platform_position_t, altitude) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_OTHER_PLATFORM_POSITION { \
    "OTHER_PLATFORM_POSITION", \
    3, \
    {  { "latitude", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_other_platform_position_t, latitude) }, \
         { "longitude", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_other_platform_position_t, longitude) }, \
         { "altitude", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_other_platform_position_t, altitude) }, \
         } \
}
#endif

/**
 * @brief Pack a other_platform_position message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param latitude  latitude of the other platform in degrees * 1E7
 * @param longitude  longitude of the other platform in degrees * 1E7
 * @param altitude  altitude of the other platform in MSL meters 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_other_platform_position_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               int32_t latitude, int32_t longitude, int32_t altitude)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_OTHER_PLATFORM_POSITION_LEN];
    _mav_put_int32_t(buf, 0, latitude);
    _mav_put_int32_t(buf, 4, longitude);
    _mav_put_int32_t(buf, 8, altitude);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_OTHER_PLATFORM_POSITION_LEN);
#else
    mavlink_other_platform_position_t packet;
    packet.latitude = latitude;
    packet.longitude = longitude;
    packet.altitude = altitude;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_OTHER_PLATFORM_POSITION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_OTHER_PLATFORM_POSITION;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_OTHER_PLATFORM_POSITION_MIN_LEN, MAVLINK_MSG_ID_OTHER_PLATFORM_POSITION_LEN, MAVLINK_MSG_ID_OTHER_PLATFORM_POSITION_CRC);
}

/**
 * @brief Pack a other_platform_position message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param latitude  latitude of the other platform in degrees * 1E7
 * @param longitude  longitude of the other platform in degrees * 1E7
 * @param altitude  altitude of the other platform in MSL meters 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_other_platform_position_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   int32_t latitude,int32_t longitude,int32_t altitude)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_OTHER_PLATFORM_POSITION_LEN];
    _mav_put_int32_t(buf, 0, latitude);
    _mav_put_int32_t(buf, 4, longitude);
    _mav_put_int32_t(buf, 8, altitude);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_OTHER_PLATFORM_POSITION_LEN);
#else
    mavlink_other_platform_position_t packet;
    packet.latitude = latitude;
    packet.longitude = longitude;
    packet.altitude = altitude;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_OTHER_PLATFORM_POSITION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_OTHER_PLATFORM_POSITION;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_OTHER_PLATFORM_POSITION_MIN_LEN, MAVLINK_MSG_ID_OTHER_PLATFORM_POSITION_LEN, MAVLINK_MSG_ID_OTHER_PLATFORM_POSITION_CRC);
}

/**
 * @brief Encode a other_platform_position struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param other_platform_position C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_other_platform_position_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_other_platform_position_t* other_platform_position)
{
    return mavlink_msg_other_platform_position_pack(system_id, component_id, msg, other_platform_position->latitude, other_platform_position->longitude, other_platform_position->altitude);
}

/**
 * @brief Encode a other_platform_position struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param other_platform_position C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_other_platform_position_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_other_platform_position_t* other_platform_position)
{
    return mavlink_msg_other_platform_position_pack_chan(system_id, component_id, chan, msg, other_platform_position->latitude, other_platform_position->longitude, other_platform_position->altitude);
}

/**
 * @brief Send a other_platform_position message
 * @param chan MAVLink channel to send the message
 *
 * @param latitude  latitude of the other platform in degrees * 1E7
 * @param longitude  longitude of the other platform in degrees * 1E7
 * @param altitude  altitude of the other platform in MSL meters 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_other_platform_position_send(mavlink_channel_t chan, int32_t latitude, int32_t longitude, int32_t altitude)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_OTHER_PLATFORM_POSITION_LEN];
    _mav_put_int32_t(buf, 0, latitude);
    _mav_put_int32_t(buf, 4, longitude);
    _mav_put_int32_t(buf, 8, altitude);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OTHER_PLATFORM_POSITION, buf, MAVLINK_MSG_ID_OTHER_PLATFORM_POSITION_MIN_LEN, MAVLINK_MSG_ID_OTHER_PLATFORM_POSITION_LEN, MAVLINK_MSG_ID_OTHER_PLATFORM_POSITION_CRC);
#else
    mavlink_other_platform_position_t packet;
    packet.latitude = latitude;
    packet.longitude = longitude;
    packet.altitude = altitude;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OTHER_PLATFORM_POSITION, (const char *)&packet, MAVLINK_MSG_ID_OTHER_PLATFORM_POSITION_MIN_LEN, MAVLINK_MSG_ID_OTHER_PLATFORM_POSITION_LEN, MAVLINK_MSG_ID_OTHER_PLATFORM_POSITION_CRC);
#endif
}

/**
 * @brief Send a other_platform_position message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_other_platform_position_send_struct(mavlink_channel_t chan, const mavlink_other_platform_position_t* other_platform_position)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_other_platform_position_send(chan, other_platform_position->latitude, other_platform_position->longitude, other_platform_position->altitude);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OTHER_PLATFORM_POSITION, (const char *)other_platform_position, MAVLINK_MSG_ID_OTHER_PLATFORM_POSITION_MIN_LEN, MAVLINK_MSG_ID_OTHER_PLATFORM_POSITION_LEN, MAVLINK_MSG_ID_OTHER_PLATFORM_POSITION_CRC);
#endif
}

#if MAVLINK_MSG_ID_OTHER_PLATFORM_POSITION_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_other_platform_position_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  int32_t latitude, int32_t longitude, int32_t altitude)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_int32_t(buf, 0, latitude);
    _mav_put_int32_t(buf, 4, longitude);
    _mav_put_int32_t(buf, 8, altitude);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OTHER_PLATFORM_POSITION, buf, MAVLINK_MSG_ID_OTHER_PLATFORM_POSITION_MIN_LEN, MAVLINK_MSG_ID_OTHER_PLATFORM_POSITION_LEN, MAVLINK_MSG_ID_OTHER_PLATFORM_POSITION_CRC);
#else
    mavlink_other_platform_position_t *packet = (mavlink_other_platform_position_t *)msgbuf;
    packet->latitude = latitude;
    packet->longitude = longitude;
    packet->altitude = altitude;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OTHER_PLATFORM_POSITION, (const char *)packet, MAVLINK_MSG_ID_OTHER_PLATFORM_POSITION_MIN_LEN, MAVLINK_MSG_ID_OTHER_PLATFORM_POSITION_LEN, MAVLINK_MSG_ID_OTHER_PLATFORM_POSITION_CRC);
#endif
}
#endif

#endif

// MESSAGE OTHER_PLATFORM_POSITION UNPACKING


/**
 * @brief Get field latitude from other_platform_position message
 *
 * @return  latitude of the other platform in degrees * 1E7
 */
static inline int32_t mavlink_msg_other_platform_position_get_latitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  0);
}

/**
 * @brief Get field longitude from other_platform_position message
 *
 * @return  longitude of the other platform in degrees * 1E7
 */
static inline int32_t mavlink_msg_other_platform_position_get_longitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  4);
}

/**
 * @brief Get field altitude from other_platform_position message
 *
 * @return  altitude of the other platform in MSL meters 
 */
static inline int32_t mavlink_msg_other_platform_position_get_altitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  8);
}

/**
 * @brief Decode a other_platform_position message into a struct
 *
 * @param msg The message to decode
 * @param other_platform_position C-struct to decode the message contents into
 */
static inline void mavlink_msg_other_platform_position_decode(const mavlink_message_t* msg, mavlink_other_platform_position_t* other_platform_position)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    other_platform_position->latitude = mavlink_msg_other_platform_position_get_latitude(msg);
    other_platform_position->longitude = mavlink_msg_other_platform_position_get_longitude(msg);
    other_platform_position->altitude = mavlink_msg_other_platform_position_get_altitude(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_OTHER_PLATFORM_POSITION_LEN? msg->len : MAVLINK_MSG_ID_OTHER_PLATFORM_POSITION_LEN;
        memset(other_platform_position, 0, MAVLINK_MSG_ID_OTHER_PLATFORM_POSITION_LEN);
    memcpy(other_platform_position, _MAV_PAYLOAD(msg), len);
#endif
}
