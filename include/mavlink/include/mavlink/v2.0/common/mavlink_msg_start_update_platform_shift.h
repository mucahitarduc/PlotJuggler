#pragma once
// MESSAGE START_UPDATE_PLATFORM_SHIFT PACKING

#define MAVLINK_MSG_ID_START_UPDATE_PLATFORM_SHIFT 207

MAVPACKED(
typedef struct __mavlink_start_update_platform_shift_t {
 int32_t pri_latitude; /*<  latitude of the primary platform. degrees * 1E7 */
 int32_t pri_longitude; /*<  longitude of the primary platform. degrees * 1E7 */
 int32_t pri_altitude; /*<  altitude of the primary platform. meters MSL */
 int32_t pri_takeoffAltitude; /*<  takeoff altitude of the primary platform. meters MSL */
 float pri_yaw; /*<  yaw angle in degrees of the primary platform */
 float pri_zoom_level; /*<  zoom level of the camera of primary platform */
 float pri_gimbal_pitch; /*<  gimbal pitch angle in degrees of the primary platform. */
 float pri_gimbal_yaw; /*<  gimbal yaw angle in degrees of the primary platform. */
 int16_t dest_wp_id; /*<  destination waypoint index of primary platform when shift command comes. */
}) mavlink_start_update_platform_shift_t;

#define MAVLINK_MSG_ID_START_UPDATE_PLATFORM_SHIFT_LEN 34
#define MAVLINK_MSG_ID_START_UPDATE_PLATFORM_SHIFT_MIN_LEN 34
#define MAVLINK_MSG_ID_207_LEN 34
#define MAVLINK_MSG_ID_207_MIN_LEN 34

#define MAVLINK_MSG_ID_START_UPDATE_PLATFORM_SHIFT_CRC 138
#define MAVLINK_MSG_ID_207_CRC 138



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_START_UPDATE_PLATFORM_SHIFT { \
    207, \
    "START_UPDATE_PLATFORM_SHIFT", \
    9, \
    {  { "pri_latitude", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_start_update_platform_shift_t, pri_latitude) }, \
         { "pri_longitude", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_start_update_platform_shift_t, pri_longitude) }, \
         { "pri_altitude", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_start_update_platform_shift_t, pri_altitude) }, \
         { "pri_takeoffAltitude", NULL, MAVLINK_TYPE_INT32_T, 0, 12, offsetof(mavlink_start_update_platform_shift_t, pri_takeoffAltitude) }, \
         { "pri_yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_start_update_platform_shift_t, pri_yaw) }, \
         { "pri_zoom_level", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_start_update_platform_shift_t, pri_zoom_level) }, \
         { "pri_gimbal_pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_start_update_platform_shift_t, pri_gimbal_pitch) }, \
         { "pri_gimbal_yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_start_update_platform_shift_t, pri_gimbal_yaw) }, \
         { "dest_wp_id", NULL, MAVLINK_TYPE_INT16_T, 0, 32, offsetof(mavlink_start_update_platform_shift_t, dest_wp_id) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_START_UPDATE_PLATFORM_SHIFT { \
    "START_UPDATE_PLATFORM_SHIFT", \
    9, \
    {  { "pri_latitude", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_start_update_platform_shift_t, pri_latitude) }, \
         { "pri_longitude", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_start_update_platform_shift_t, pri_longitude) }, \
         { "pri_altitude", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_start_update_platform_shift_t, pri_altitude) }, \
         { "pri_takeoffAltitude", NULL, MAVLINK_TYPE_INT32_T, 0, 12, offsetof(mavlink_start_update_platform_shift_t, pri_takeoffAltitude) }, \
         { "pri_yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_start_update_platform_shift_t, pri_yaw) }, \
         { "pri_zoom_level", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_start_update_platform_shift_t, pri_zoom_level) }, \
         { "pri_gimbal_pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_start_update_platform_shift_t, pri_gimbal_pitch) }, \
         { "pri_gimbal_yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_start_update_platform_shift_t, pri_gimbal_yaw) }, \
         { "dest_wp_id", NULL, MAVLINK_TYPE_INT16_T, 0, 32, offsetof(mavlink_start_update_platform_shift_t, dest_wp_id) }, \
         } \
}
#endif

/**
 * @brief Pack a start_update_platform_shift message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param pri_latitude  latitude of the primary platform. degrees * 1E7 
 * @param pri_longitude  longitude of the primary platform. degrees * 1E7 
 * @param pri_altitude  altitude of the primary platform. meters MSL 
 * @param pri_takeoffAltitude  takeoff altitude of the primary platform. meters MSL 
 * @param pri_yaw  yaw angle in degrees of the primary platform 
 * @param pri_zoom_level  zoom level of the camera of primary platform 
 * @param pri_gimbal_pitch  gimbal pitch angle in degrees of the primary platform. 
 * @param pri_gimbal_yaw  gimbal yaw angle in degrees of the primary platform. 
 * @param dest_wp_id  destination waypoint index of primary platform when shift command comes. 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_start_update_platform_shift_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               int32_t pri_latitude, int32_t pri_longitude, int32_t pri_altitude, int32_t pri_takeoffAltitude, float pri_yaw, float pri_zoom_level, float pri_gimbal_pitch, float pri_gimbal_yaw, int16_t dest_wp_id)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_START_UPDATE_PLATFORM_SHIFT_LEN];
    _mav_put_int32_t(buf, 0, pri_latitude);
    _mav_put_int32_t(buf, 4, pri_longitude);
    _mav_put_int32_t(buf, 8, pri_altitude);
    _mav_put_int32_t(buf, 12, pri_takeoffAltitude);
    _mav_put_float(buf, 16, pri_yaw);
    _mav_put_float(buf, 20, pri_zoom_level);
    _mav_put_float(buf, 24, pri_gimbal_pitch);
    _mav_put_float(buf, 28, pri_gimbal_yaw);
    _mav_put_int16_t(buf, 32, dest_wp_id);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_START_UPDATE_PLATFORM_SHIFT_LEN);
#else
    mavlink_start_update_platform_shift_t packet;
    packet.pri_latitude = pri_latitude;
    packet.pri_longitude = pri_longitude;
    packet.pri_altitude = pri_altitude;
    packet.pri_takeoffAltitude = pri_takeoffAltitude;
    packet.pri_yaw = pri_yaw;
    packet.pri_zoom_level = pri_zoom_level;
    packet.pri_gimbal_pitch = pri_gimbal_pitch;
    packet.pri_gimbal_yaw = pri_gimbal_yaw;
    packet.dest_wp_id = dest_wp_id;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_START_UPDATE_PLATFORM_SHIFT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_START_UPDATE_PLATFORM_SHIFT;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_START_UPDATE_PLATFORM_SHIFT_MIN_LEN, MAVLINK_MSG_ID_START_UPDATE_PLATFORM_SHIFT_LEN, MAVLINK_MSG_ID_START_UPDATE_PLATFORM_SHIFT_CRC);
}

/**
 * @brief Pack a start_update_platform_shift message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param pri_latitude  latitude of the primary platform. degrees * 1E7 
 * @param pri_longitude  longitude of the primary platform. degrees * 1E7 
 * @param pri_altitude  altitude of the primary platform. meters MSL 
 * @param pri_takeoffAltitude  takeoff altitude of the primary platform. meters MSL 
 * @param pri_yaw  yaw angle in degrees of the primary platform 
 * @param pri_zoom_level  zoom level of the camera of primary platform 
 * @param pri_gimbal_pitch  gimbal pitch angle in degrees of the primary platform. 
 * @param pri_gimbal_yaw  gimbal yaw angle in degrees of the primary platform. 
 * @param dest_wp_id  destination waypoint index of primary platform when shift command comes. 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_start_update_platform_shift_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   int32_t pri_latitude,int32_t pri_longitude,int32_t pri_altitude,int32_t pri_takeoffAltitude,float pri_yaw,float pri_zoom_level,float pri_gimbal_pitch,float pri_gimbal_yaw,int16_t dest_wp_id)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_START_UPDATE_PLATFORM_SHIFT_LEN];
    _mav_put_int32_t(buf, 0, pri_latitude);
    _mav_put_int32_t(buf, 4, pri_longitude);
    _mav_put_int32_t(buf, 8, pri_altitude);
    _mav_put_int32_t(buf, 12, pri_takeoffAltitude);
    _mav_put_float(buf, 16, pri_yaw);
    _mav_put_float(buf, 20, pri_zoom_level);
    _mav_put_float(buf, 24, pri_gimbal_pitch);
    _mav_put_float(buf, 28, pri_gimbal_yaw);
    _mav_put_int16_t(buf, 32, dest_wp_id);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_START_UPDATE_PLATFORM_SHIFT_LEN);
#else
    mavlink_start_update_platform_shift_t packet;
    packet.pri_latitude = pri_latitude;
    packet.pri_longitude = pri_longitude;
    packet.pri_altitude = pri_altitude;
    packet.pri_takeoffAltitude = pri_takeoffAltitude;
    packet.pri_yaw = pri_yaw;
    packet.pri_zoom_level = pri_zoom_level;
    packet.pri_gimbal_pitch = pri_gimbal_pitch;
    packet.pri_gimbal_yaw = pri_gimbal_yaw;
    packet.dest_wp_id = dest_wp_id;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_START_UPDATE_PLATFORM_SHIFT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_START_UPDATE_PLATFORM_SHIFT;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_START_UPDATE_PLATFORM_SHIFT_MIN_LEN, MAVLINK_MSG_ID_START_UPDATE_PLATFORM_SHIFT_LEN, MAVLINK_MSG_ID_START_UPDATE_PLATFORM_SHIFT_CRC);
}

/**
 * @brief Encode a start_update_platform_shift struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param start_update_platform_shift C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_start_update_platform_shift_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_start_update_platform_shift_t* start_update_platform_shift)
{
    return mavlink_msg_start_update_platform_shift_pack(system_id, component_id, msg, start_update_platform_shift->pri_latitude, start_update_platform_shift->pri_longitude, start_update_platform_shift->pri_altitude, start_update_platform_shift->pri_takeoffAltitude, start_update_platform_shift->pri_yaw, start_update_platform_shift->pri_zoom_level, start_update_platform_shift->pri_gimbal_pitch, start_update_platform_shift->pri_gimbal_yaw, start_update_platform_shift->dest_wp_id);
}

/**
 * @brief Encode a start_update_platform_shift struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param start_update_platform_shift C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_start_update_platform_shift_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_start_update_platform_shift_t* start_update_platform_shift)
{
    return mavlink_msg_start_update_platform_shift_pack_chan(system_id, component_id, chan, msg, start_update_platform_shift->pri_latitude, start_update_platform_shift->pri_longitude, start_update_platform_shift->pri_altitude, start_update_platform_shift->pri_takeoffAltitude, start_update_platform_shift->pri_yaw, start_update_platform_shift->pri_zoom_level, start_update_platform_shift->pri_gimbal_pitch, start_update_platform_shift->pri_gimbal_yaw, start_update_platform_shift->dest_wp_id);
}

/**
 * @brief Send a start_update_platform_shift message
 * @param chan MAVLink channel to send the message
 *
 * @param pri_latitude  latitude of the primary platform. degrees * 1E7 
 * @param pri_longitude  longitude of the primary platform. degrees * 1E7 
 * @param pri_altitude  altitude of the primary platform. meters MSL 
 * @param pri_takeoffAltitude  takeoff altitude of the primary platform. meters MSL 
 * @param pri_yaw  yaw angle in degrees of the primary platform 
 * @param pri_zoom_level  zoom level of the camera of primary platform 
 * @param pri_gimbal_pitch  gimbal pitch angle in degrees of the primary platform. 
 * @param pri_gimbal_yaw  gimbal yaw angle in degrees of the primary platform. 
 * @param dest_wp_id  destination waypoint index of primary platform when shift command comes. 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_start_update_platform_shift_send(mavlink_channel_t chan, int32_t pri_latitude, int32_t pri_longitude, int32_t pri_altitude, int32_t pri_takeoffAltitude, float pri_yaw, float pri_zoom_level, float pri_gimbal_pitch, float pri_gimbal_yaw, int16_t dest_wp_id)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_START_UPDATE_PLATFORM_SHIFT_LEN];
    _mav_put_int32_t(buf, 0, pri_latitude);
    _mav_put_int32_t(buf, 4, pri_longitude);
    _mav_put_int32_t(buf, 8, pri_altitude);
    _mav_put_int32_t(buf, 12, pri_takeoffAltitude);
    _mav_put_float(buf, 16, pri_yaw);
    _mav_put_float(buf, 20, pri_zoom_level);
    _mav_put_float(buf, 24, pri_gimbal_pitch);
    _mav_put_float(buf, 28, pri_gimbal_yaw);
    _mav_put_int16_t(buf, 32, dest_wp_id);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_START_UPDATE_PLATFORM_SHIFT, buf, MAVLINK_MSG_ID_START_UPDATE_PLATFORM_SHIFT_MIN_LEN, MAVLINK_MSG_ID_START_UPDATE_PLATFORM_SHIFT_LEN, MAVLINK_MSG_ID_START_UPDATE_PLATFORM_SHIFT_CRC);
#else
    mavlink_start_update_platform_shift_t packet;
    packet.pri_latitude = pri_latitude;
    packet.pri_longitude = pri_longitude;
    packet.pri_altitude = pri_altitude;
    packet.pri_takeoffAltitude = pri_takeoffAltitude;
    packet.pri_yaw = pri_yaw;
    packet.pri_zoom_level = pri_zoom_level;
    packet.pri_gimbal_pitch = pri_gimbal_pitch;
    packet.pri_gimbal_yaw = pri_gimbal_yaw;
    packet.dest_wp_id = dest_wp_id;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_START_UPDATE_PLATFORM_SHIFT, (const char *)&packet, MAVLINK_MSG_ID_START_UPDATE_PLATFORM_SHIFT_MIN_LEN, MAVLINK_MSG_ID_START_UPDATE_PLATFORM_SHIFT_LEN, MAVLINK_MSG_ID_START_UPDATE_PLATFORM_SHIFT_CRC);
#endif
}

/**
 * @brief Send a start_update_platform_shift message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_start_update_platform_shift_send_struct(mavlink_channel_t chan, const mavlink_start_update_platform_shift_t* start_update_platform_shift)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_start_update_platform_shift_send(chan, start_update_platform_shift->pri_latitude, start_update_platform_shift->pri_longitude, start_update_platform_shift->pri_altitude, start_update_platform_shift->pri_takeoffAltitude, start_update_platform_shift->pri_yaw, start_update_platform_shift->pri_zoom_level, start_update_platform_shift->pri_gimbal_pitch, start_update_platform_shift->pri_gimbal_yaw, start_update_platform_shift->dest_wp_id);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_START_UPDATE_PLATFORM_SHIFT, (const char *)start_update_platform_shift, MAVLINK_MSG_ID_START_UPDATE_PLATFORM_SHIFT_MIN_LEN, MAVLINK_MSG_ID_START_UPDATE_PLATFORM_SHIFT_LEN, MAVLINK_MSG_ID_START_UPDATE_PLATFORM_SHIFT_CRC);
#endif
}

#if MAVLINK_MSG_ID_START_UPDATE_PLATFORM_SHIFT_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_start_update_platform_shift_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  int32_t pri_latitude, int32_t pri_longitude, int32_t pri_altitude, int32_t pri_takeoffAltitude, float pri_yaw, float pri_zoom_level, float pri_gimbal_pitch, float pri_gimbal_yaw, int16_t dest_wp_id)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_int32_t(buf, 0, pri_latitude);
    _mav_put_int32_t(buf, 4, pri_longitude);
    _mav_put_int32_t(buf, 8, pri_altitude);
    _mav_put_int32_t(buf, 12, pri_takeoffAltitude);
    _mav_put_float(buf, 16, pri_yaw);
    _mav_put_float(buf, 20, pri_zoom_level);
    _mav_put_float(buf, 24, pri_gimbal_pitch);
    _mav_put_float(buf, 28, pri_gimbal_yaw);
    _mav_put_int16_t(buf, 32, dest_wp_id);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_START_UPDATE_PLATFORM_SHIFT, buf, MAVLINK_MSG_ID_START_UPDATE_PLATFORM_SHIFT_MIN_LEN, MAVLINK_MSG_ID_START_UPDATE_PLATFORM_SHIFT_LEN, MAVLINK_MSG_ID_START_UPDATE_PLATFORM_SHIFT_CRC);
#else
    mavlink_start_update_platform_shift_t *packet = (mavlink_start_update_platform_shift_t *)msgbuf;
    packet->pri_latitude = pri_latitude;
    packet->pri_longitude = pri_longitude;
    packet->pri_altitude = pri_altitude;
    packet->pri_takeoffAltitude = pri_takeoffAltitude;
    packet->pri_yaw = pri_yaw;
    packet->pri_zoom_level = pri_zoom_level;
    packet->pri_gimbal_pitch = pri_gimbal_pitch;
    packet->pri_gimbal_yaw = pri_gimbal_yaw;
    packet->dest_wp_id = dest_wp_id;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_START_UPDATE_PLATFORM_SHIFT, (const char *)packet, MAVLINK_MSG_ID_START_UPDATE_PLATFORM_SHIFT_MIN_LEN, MAVLINK_MSG_ID_START_UPDATE_PLATFORM_SHIFT_LEN, MAVLINK_MSG_ID_START_UPDATE_PLATFORM_SHIFT_CRC);
#endif
}
#endif

#endif

// MESSAGE START_UPDATE_PLATFORM_SHIFT UNPACKING


/**
 * @brief Get field pri_latitude from start_update_platform_shift message
 *
 * @return  latitude of the primary platform. degrees * 1E7 
 */
static inline int32_t mavlink_msg_start_update_platform_shift_get_pri_latitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  0);
}

/**
 * @brief Get field pri_longitude from start_update_platform_shift message
 *
 * @return  longitude of the primary platform. degrees * 1E7 
 */
static inline int32_t mavlink_msg_start_update_platform_shift_get_pri_longitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  4);
}

/**
 * @brief Get field pri_altitude from start_update_platform_shift message
 *
 * @return  altitude of the primary platform. meters MSL 
 */
static inline int32_t mavlink_msg_start_update_platform_shift_get_pri_altitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  8);
}

/**
 * @brief Get field pri_takeoffAltitude from start_update_platform_shift message
 *
 * @return  takeoff altitude of the primary platform. meters MSL 
 */
static inline int32_t mavlink_msg_start_update_platform_shift_get_pri_takeoffAltitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  12);
}

/**
 * @brief Get field pri_yaw from start_update_platform_shift message
 *
 * @return  yaw angle in degrees of the primary platform 
 */
static inline float mavlink_msg_start_update_platform_shift_get_pri_yaw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field pri_zoom_level from start_update_platform_shift message
 *
 * @return  zoom level of the camera of primary platform 
 */
static inline float mavlink_msg_start_update_platform_shift_get_pri_zoom_level(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field pri_gimbal_pitch from start_update_platform_shift message
 *
 * @return  gimbal pitch angle in degrees of the primary platform. 
 */
static inline float mavlink_msg_start_update_platform_shift_get_pri_gimbal_pitch(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field pri_gimbal_yaw from start_update_platform_shift message
 *
 * @return  gimbal yaw angle in degrees of the primary platform. 
 */
static inline float mavlink_msg_start_update_platform_shift_get_pri_gimbal_yaw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field dest_wp_id from start_update_platform_shift message
 *
 * @return  destination waypoint index of primary platform when shift command comes. 
 */
static inline int16_t mavlink_msg_start_update_platform_shift_get_dest_wp_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  32);
}

/**
 * @brief Decode a start_update_platform_shift message into a struct
 *
 * @param msg The message to decode
 * @param start_update_platform_shift C-struct to decode the message contents into
 */
static inline void mavlink_msg_start_update_platform_shift_decode(const mavlink_message_t* msg, mavlink_start_update_platform_shift_t* start_update_platform_shift)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    start_update_platform_shift->pri_latitude = mavlink_msg_start_update_platform_shift_get_pri_latitude(msg);
    start_update_platform_shift->pri_longitude = mavlink_msg_start_update_platform_shift_get_pri_longitude(msg);
    start_update_platform_shift->pri_altitude = mavlink_msg_start_update_platform_shift_get_pri_altitude(msg);
    start_update_platform_shift->pri_takeoffAltitude = mavlink_msg_start_update_platform_shift_get_pri_takeoffAltitude(msg);
    start_update_platform_shift->pri_yaw = mavlink_msg_start_update_platform_shift_get_pri_yaw(msg);
    start_update_platform_shift->pri_zoom_level = mavlink_msg_start_update_platform_shift_get_pri_zoom_level(msg);
    start_update_platform_shift->pri_gimbal_pitch = mavlink_msg_start_update_platform_shift_get_pri_gimbal_pitch(msg);
    start_update_platform_shift->pri_gimbal_yaw = mavlink_msg_start_update_platform_shift_get_pri_gimbal_yaw(msg);
    start_update_platform_shift->dest_wp_id = mavlink_msg_start_update_platform_shift_get_dest_wp_id(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_START_UPDATE_PLATFORM_SHIFT_LEN? msg->len : MAVLINK_MSG_ID_START_UPDATE_PLATFORM_SHIFT_LEN;
        memset(start_update_platform_shift, 0, MAVLINK_MSG_ID_START_UPDATE_PLATFORM_SHIFT_LEN);
    memcpy(start_update_platform_shift, _MAV_PAYLOAD(msg), len);
#endif
}
