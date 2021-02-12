#pragma once
// MESSAGE ADD_UPDATE_WAYPOINT PACKING

#define MAVLINK_MSG_ID_ADD_UPDATE_WAYPOINT 198

MAVPACKED(
typedef struct __mavlink_add_update_waypoint_t {
 int32_t latitude; /*<  Latitude of waypoint in degrees * 1E7 */
 int32_t longitude; /*<  Longitude of waypoint in degrees * 1E7 */
 int32_t altitude; /*<  Altitude of waypoint in meters (MSL) */
 int16_t wp_id; /*<  Unique id of the waypoint. range[0-101] (0=unidentified, 1-100=waypoint 101=rtl_point) */
 int16_t prev_wp_id; /*<  Previous waypoint id. range[0-101] (0=no waypoint), 0 indicates that this waypoint is the first waypoint */
 int16_t next_wp_id; /*<  Next waypoint id. range[0-101] (0=no waypoint), 0 indicates that this waypoint is the last waypoint */
 int16_t navigation_speed; /*<  Defines max navigation speed of the aircraft while navigating through this waypoint */
 int16_t action_wait_time; /*<  Defines wait action on waypoint. Aircraft waits specified seconds on waypoint */
 int16_t action_activate_camera; /*<  enum TOGAN_CAMERA_TYPE. Specified camera selected on waypoint. (NONE=camera selection not to be changed) */
 int16_t action_zoom_control; /*<  TBD */
 int16_t action_gimbal_pitch; /*<  TBD */
 int16_t action_gimbal_yaw; /*<  TBD */
 int8_t type; /*<  enum MISSION_WAYPOINT_TYPE. Type of waypoint */
}) mavlink_add_update_waypoint_t;

#define MAVLINK_MSG_ID_ADD_UPDATE_WAYPOINT_LEN 31
#define MAVLINK_MSG_ID_ADD_UPDATE_WAYPOINT_MIN_LEN 31
#define MAVLINK_MSG_ID_198_LEN 31
#define MAVLINK_MSG_ID_198_MIN_LEN 31

#define MAVLINK_MSG_ID_ADD_UPDATE_WAYPOINT_CRC 20
#define MAVLINK_MSG_ID_198_CRC 20



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ADD_UPDATE_WAYPOINT { \
    198, \
    "ADD_UPDATE_WAYPOINT", \
    13, \
    {  { "latitude", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_add_update_waypoint_t, latitude) }, \
         { "longitude", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_add_update_waypoint_t, longitude) }, \
         { "altitude", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_add_update_waypoint_t, altitude) }, \
         { "wp_id", NULL, MAVLINK_TYPE_INT16_T, 0, 12, offsetof(mavlink_add_update_waypoint_t, wp_id) }, \
         { "prev_wp_id", NULL, MAVLINK_TYPE_INT16_T, 0, 14, offsetof(mavlink_add_update_waypoint_t, prev_wp_id) }, \
         { "next_wp_id", NULL, MAVLINK_TYPE_INT16_T, 0, 16, offsetof(mavlink_add_update_waypoint_t, next_wp_id) }, \
         { "navigation_speed", NULL, MAVLINK_TYPE_INT16_T, 0, 18, offsetof(mavlink_add_update_waypoint_t, navigation_speed) }, \
         { "action_wait_time", NULL, MAVLINK_TYPE_INT16_T, 0, 20, offsetof(mavlink_add_update_waypoint_t, action_wait_time) }, \
         { "action_activate_camera", NULL, MAVLINK_TYPE_INT16_T, 0, 22, offsetof(mavlink_add_update_waypoint_t, action_activate_camera) }, \
         { "action_zoom_control", NULL, MAVLINK_TYPE_INT16_T, 0, 24, offsetof(mavlink_add_update_waypoint_t, action_zoom_control) }, \
         { "action_gimbal_pitch", NULL, MAVLINK_TYPE_INT16_T, 0, 26, offsetof(mavlink_add_update_waypoint_t, action_gimbal_pitch) }, \
         { "action_gimbal_yaw", NULL, MAVLINK_TYPE_INT16_T, 0, 28, offsetof(mavlink_add_update_waypoint_t, action_gimbal_yaw) }, \
         { "type", NULL, MAVLINK_TYPE_INT8_T, 0, 30, offsetof(mavlink_add_update_waypoint_t, type) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ADD_UPDATE_WAYPOINT { \
    "ADD_UPDATE_WAYPOINT", \
    13, \
    {  { "latitude", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_add_update_waypoint_t, latitude) }, \
         { "longitude", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_add_update_waypoint_t, longitude) }, \
         { "altitude", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_add_update_waypoint_t, altitude) }, \
         { "wp_id", NULL, MAVLINK_TYPE_INT16_T, 0, 12, offsetof(mavlink_add_update_waypoint_t, wp_id) }, \
         { "prev_wp_id", NULL, MAVLINK_TYPE_INT16_T, 0, 14, offsetof(mavlink_add_update_waypoint_t, prev_wp_id) }, \
         { "next_wp_id", NULL, MAVLINK_TYPE_INT16_T, 0, 16, offsetof(mavlink_add_update_waypoint_t, next_wp_id) }, \
         { "navigation_speed", NULL, MAVLINK_TYPE_INT16_T, 0, 18, offsetof(mavlink_add_update_waypoint_t, navigation_speed) }, \
         { "action_wait_time", NULL, MAVLINK_TYPE_INT16_T, 0, 20, offsetof(mavlink_add_update_waypoint_t, action_wait_time) }, \
         { "action_activate_camera", NULL, MAVLINK_TYPE_INT16_T, 0, 22, offsetof(mavlink_add_update_waypoint_t, action_activate_camera) }, \
         { "action_zoom_control", NULL, MAVLINK_TYPE_INT16_T, 0, 24, offsetof(mavlink_add_update_waypoint_t, action_zoom_control) }, \
         { "action_gimbal_pitch", NULL, MAVLINK_TYPE_INT16_T, 0, 26, offsetof(mavlink_add_update_waypoint_t, action_gimbal_pitch) }, \
         { "action_gimbal_yaw", NULL, MAVLINK_TYPE_INT16_T, 0, 28, offsetof(mavlink_add_update_waypoint_t, action_gimbal_yaw) }, \
         { "type", NULL, MAVLINK_TYPE_INT8_T, 0, 30, offsetof(mavlink_add_update_waypoint_t, type) }, \
         } \
}
#endif

/**
 * @brief Pack a add_update_waypoint message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param latitude  Latitude of waypoint in degrees * 1E7 
 * @param longitude  Longitude of waypoint in degrees * 1E7 
 * @param altitude  Altitude of waypoint in meters (MSL) 
 * @param type  enum MISSION_WAYPOINT_TYPE. Type of waypoint 
 * @param wp_id  Unique id of the waypoint. range[0-101] (0=unidentified, 1-100=waypoint 101=rtl_point) 
 * @param prev_wp_id  Previous waypoint id. range[0-101] (0=no waypoint), 0 indicates that this waypoint is the first waypoint 
 * @param next_wp_id  Next waypoint id. range[0-101] (0=no waypoint), 0 indicates that this waypoint is the last waypoint 
 * @param navigation_speed  Defines max navigation speed of the aircraft while navigating through this waypoint 
 * @param action_wait_time  Defines wait action on waypoint. Aircraft waits specified seconds on waypoint 
 * @param action_activate_camera  enum TOGAN_CAMERA_TYPE. Specified camera selected on waypoint. (NONE=camera selection not to be changed) 
 * @param action_zoom_control  TBD 
 * @param action_gimbal_pitch  TBD 
 * @param action_gimbal_yaw  TBD 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_add_update_waypoint_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               int32_t latitude, int32_t longitude, int32_t altitude, int8_t type, int16_t wp_id, int16_t prev_wp_id, int16_t next_wp_id, int16_t navigation_speed, int16_t action_wait_time, int16_t action_activate_camera, int16_t action_zoom_control, int16_t action_gimbal_pitch, int16_t action_gimbal_yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ADD_UPDATE_WAYPOINT_LEN];
    _mav_put_int32_t(buf, 0, latitude);
    _mav_put_int32_t(buf, 4, longitude);
    _mav_put_int32_t(buf, 8, altitude);
    _mav_put_int16_t(buf, 12, wp_id);
    _mav_put_int16_t(buf, 14, prev_wp_id);
    _mav_put_int16_t(buf, 16, next_wp_id);
    _mav_put_int16_t(buf, 18, navigation_speed);
    _mav_put_int16_t(buf, 20, action_wait_time);
    _mav_put_int16_t(buf, 22, action_activate_camera);
    _mav_put_int16_t(buf, 24, action_zoom_control);
    _mav_put_int16_t(buf, 26, action_gimbal_pitch);
    _mav_put_int16_t(buf, 28, action_gimbal_yaw);
    _mav_put_int8_t(buf, 30, type);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ADD_UPDATE_WAYPOINT_LEN);
#else
    mavlink_add_update_waypoint_t packet;
    packet.latitude = latitude;
    packet.longitude = longitude;
    packet.altitude = altitude;
    packet.wp_id = wp_id;
    packet.prev_wp_id = prev_wp_id;
    packet.next_wp_id = next_wp_id;
    packet.navigation_speed = navigation_speed;
    packet.action_wait_time = action_wait_time;
    packet.action_activate_camera = action_activate_camera;
    packet.action_zoom_control = action_zoom_control;
    packet.action_gimbal_pitch = action_gimbal_pitch;
    packet.action_gimbal_yaw = action_gimbal_yaw;
    packet.type = type;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ADD_UPDATE_WAYPOINT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ADD_UPDATE_WAYPOINT;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ADD_UPDATE_WAYPOINT_MIN_LEN, MAVLINK_MSG_ID_ADD_UPDATE_WAYPOINT_LEN, MAVLINK_MSG_ID_ADD_UPDATE_WAYPOINT_CRC);
}

/**
 * @brief Pack a add_update_waypoint message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param latitude  Latitude of waypoint in degrees * 1E7 
 * @param longitude  Longitude of waypoint in degrees * 1E7 
 * @param altitude  Altitude of waypoint in meters (MSL) 
 * @param type  enum MISSION_WAYPOINT_TYPE. Type of waypoint 
 * @param wp_id  Unique id of the waypoint. range[0-101] (0=unidentified, 1-100=waypoint 101=rtl_point) 
 * @param prev_wp_id  Previous waypoint id. range[0-101] (0=no waypoint), 0 indicates that this waypoint is the first waypoint 
 * @param next_wp_id  Next waypoint id. range[0-101] (0=no waypoint), 0 indicates that this waypoint is the last waypoint 
 * @param navigation_speed  Defines max navigation speed of the aircraft while navigating through this waypoint 
 * @param action_wait_time  Defines wait action on waypoint. Aircraft waits specified seconds on waypoint 
 * @param action_activate_camera  enum TOGAN_CAMERA_TYPE. Specified camera selected on waypoint. (NONE=camera selection not to be changed) 
 * @param action_zoom_control  TBD 
 * @param action_gimbal_pitch  TBD 
 * @param action_gimbal_yaw  TBD 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_add_update_waypoint_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   int32_t latitude,int32_t longitude,int32_t altitude,int8_t type,int16_t wp_id,int16_t prev_wp_id,int16_t next_wp_id,int16_t navigation_speed,int16_t action_wait_time,int16_t action_activate_camera,int16_t action_zoom_control,int16_t action_gimbal_pitch,int16_t action_gimbal_yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ADD_UPDATE_WAYPOINT_LEN];
    _mav_put_int32_t(buf, 0, latitude);
    _mav_put_int32_t(buf, 4, longitude);
    _mav_put_int32_t(buf, 8, altitude);
    _mav_put_int16_t(buf, 12, wp_id);
    _mav_put_int16_t(buf, 14, prev_wp_id);
    _mav_put_int16_t(buf, 16, next_wp_id);
    _mav_put_int16_t(buf, 18, navigation_speed);
    _mav_put_int16_t(buf, 20, action_wait_time);
    _mav_put_int16_t(buf, 22, action_activate_camera);
    _mav_put_int16_t(buf, 24, action_zoom_control);
    _mav_put_int16_t(buf, 26, action_gimbal_pitch);
    _mav_put_int16_t(buf, 28, action_gimbal_yaw);
    _mav_put_int8_t(buf, 30, type);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ADD_UPDATE_WAYPOINT_LEN);
#else
    mavlink_add_update_waypoint_t packet;
    packet.latitude = latitude;
    packet.longitude = longitude;
    packet.altitude = altitude;
    packet.wp_id = wp_id;
    packet.prev_wp_id = prev_wp_id;
    packet.next_wp_id = next_wp_id;
    packet.navigation_speed = navigation_speed;
    packet.action_wait_time = action_wait_time;
    packet.action_activate_camera = action_activate_camera;
    packet.action_zoom_control = action_zoom_control;
    packet.action_gimbal_pitch = action_gimbal_pitch;
    packet.action_gimbal_yaw = action_gimbal_yaw;
    packet.type = type;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ADD_UPDATE_WAYPOINT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ADD_UPDATE_WAYPOINT;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ADD_UPDATE_WAYPOINT_MIN_LEN, MAVLINK_MSG_ID_ADD_UPDATE_WAYPOINT_LEN, MAVLINK_MSG_ID_ADD_UPDATE_WAYPOINT_CRC);
}

/**
 * @brief Encode a add_update_waypoint struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param add_update_waypoint C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_add_update_waypoint_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_add_update_waypoint_t* add_update_waypoint)
{
    return mavlink_msg_add_update_waypoint_pack(system_id, component_id, msg, add_update_waypoint->latitude, add_update_waypoint->longitude, add_update_waypoint->altitude, add_update_waypoint->type, add_update_waypoint->wp_id, add_update_waypoint->prev_wp_id, add_update_waypoint->next_wp_id, add_update_waypoint->navigation_speed, add_update_waypoint->action_wait_time, add_update_waypoint->action_activate_camera, add_update_waypoint->action_zoom_control, add_update_waypoint->action_gimbal_pitch, add_update_waypoint->action_gimbal_yaw);
}

/**
 * @brief Encode a add_update_waypoint struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param add_update_waypoint C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_add_update_waypoint_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_add_update_waypoint_t* add_update_waypoint)
{
    return mavlink_msg_add_update_waypoint_pack_chan(system_id, component_id, chan, msg, add_update_waypoint->latitude, add_update_waypoint->longitude, add_update_waypoint->altitude, add_update_waypoint->type, add_update_waypoint->wp_id, add_update_waypoint->prev_wp_id, add_update_waypoint->next_wp_id, add_update_waypoint->navigation_speed, add_update_waypoint->action_wait_time, add_update_waypoint->action_activate_camera, add_update_waypoint->action_zoom_control, add_update_waypoint->action_gimbal_pitch, add_update_waypoint->action_gimbal_yaw);
}

/**
 * @brief Send a add_update_waypoint message
 * @param chan MAVLink channel to send the message
 *
 * @param latitude  Latitude of waypoint in degrees * 1E7 
 * @param longitude  Longitude of waypoint in degrees * 1E7 
 * @param altitude  Altitude of waypoint in meters (MSL) 
 * @param type  enum MISSION_WAYPOINT_TYPE. Type of waypoint 
 * @param wp_id  Unique id of the waypoint. range[0-101] (0=unidentified, 1-100=waypoint 101=rtl_point) 
 * @param prev_wp_id  Previous waypoint id. range[0-101] (0=no waypoint), 0 indicates that this waypoint is the first waypoint 
 * @param next_wp_id  Next waypoint id. range[0-101] (0=no waypoint), 0 indicates that this waypoint is the last waypoint 
 * @param navigation_speed  Defines max navigation speed of the aircraft while navigating through this waypoint 
 * @param action_wait_time  Defines wait action on waypoint. Aircraft waits specified seconds on waypoint 
 * @param action_activate_camera  enum TOGAN_CAMERA_TYPE. Specified camera selected on waypoint. (NONE=camera selection not to be changed) 
 * @param action_zoom_control  TBD 
 * @param action_gimbal_pitch  TBD 
 * @param action_gimbal_yaw  TBD 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_add_update_waypoint_send(mavlink_channel_t chan, int32_t latitude, int32_t longitude, int32_t altitude, int8_t type, int16_t wp_id, int16_t prev_wp_id, int16_t next_wp_id, int16_t navigation_speed, int16_t action_wait_time, int16_t action_activate_camera, int16_t action_zoom_control, int16_t action_gimbal_pitch, int16_t action_gimbal_yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ADD_UPDATE_WAYPOINT_LEN];
    _mav_put_int32_t(buf, 0, latitude);
    _mav_put_int32_t(buf, 4, longitude);
    _mav_put_int32_t(buf, 8, altitude);
    _mav_put_int16_t(buf, 12, wp_id);
    _mav_put_int16_t(buf, 14, prev_wp_id);
    _mav_put_int16_t(buf, 16, next_wp_id);
    _mav_put_int16_t(buf, 18, navigation_speed);
    _mav_put_int16_t(buf, 20, action_wait_time);
    _mav_put_int16_t(buf, 22, action_activate_camera);
    _mav_put_int16_t(buf, 24, action_zoom_control);
    _mav_put_int16_t(buf, 26, action_gimbal_pitch);
    _mav_put_int16_t(buf, 28, action_gimbal_yaw);
    _mav_put_int8_t(buf, 30, type);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ADD_UPDATE_WAYPOINT, buf, MAVLINK_MSG_ID_ADD_UPDATE_WAYPOINT_MIN_LEN, MAVLINK_MSG_ID_ADD_UPDATE_WAYPOINT_LEN, MAVLINK_MSG_ID_ADD_UPDATE_WAYPOINT_CRC);
#else
    mavlink_add_update_waypoint_t packet;
    packet.latitude = latitude;
    packet.longitude = longitude;
    packet.altitude = altitude;
    packet.wp_id = wp_id;
    packet.prev_wp_id = prev_wp_id;
    packet.next_wp_id = next_wp_id;
    packet.navigation_speed = navigation_speed;
    packet.action_wait_time = action_wait_time;
    packet.action_activate_camera = action_activate_camera;
    packet.action_zoom_control = action_zoom_control;
    packet.action_gimbal_pitch = action_gimbal_pitch;
    packet.action_gimbal_yaw = action_gimbal_yaw;
    packet.type = type;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ADD_UPDATE_WAYPOINT, (const char *)&packet, MAVLINK_MSG_ID_ADD_UPDATE_WAYPOINT_MIN_LEN, MAVLINK_MSG_ID_ADD_UPDATE_WAYPOINT_LEN, MAVLINK_MSG_ID_ADD_UPDATE_WAYPOINT_CRC);
#endif
}

/**
 * @brief Send a add_update_waypoint message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_add_update_waypoint_send_struct(mavlink_channel_t chan, const mavlink_add_update_waypoint_t* add_update_waypoint)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_add_update_waypoint_send(chan, add_update_waypoint->latitude, add_update_waypoint->longitude, add_update_waypoint->altitude, add_update_waypoint->type, add_update_waypoint->wp_id, add_update_waypoint->prev_wp_id, add_update_waypoint->next_wp_id, add_update_waypoint->navigation_speed, add_update_waypoint->action_wait_time, add_update_waypoint->action_activate_camera, add_update_waypoint->action_zoom_control, add_update_waypoint->action_gimbal_pitch, add_update_waypoint->action_gimbal_yaw);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ADD_UPDATE_WAYPOINT, (const char *)add_update_waypoint, MAVLINK_MSG_ID_ADD_UPDATE_WAYPOINT_MIN_LEN, MAVLINK_MSG_ID_ADD_UPDATE_WAYPOINT_LEN, MAVLINK_MSG_ID_ADD_UPDATE_WAYPOINT_CRC);
#endif
}

#if MAVLINK_MSG_ID_ADD_UPDATE_WAYPOINT_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_add_update_waypoint_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  int32_t latitude, int32_t longitude, int32_t altitude, int8_t type, int16_t wp_id, int16_t prev_wp_id, int16_t next_wp_id, int16_t navigation_speed, int16_t action_wait_time, int16_t action_activate_camera, int16_t action_zoom_control, int16_t action_gimbal_pitch, int16_t action_gimbal_yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_int32_t(buf, 0, latitude);
    _mav_put_int32_t(buf, 4, longitude);
    _mav_put_int32_t(buf, 8, altitude);
    _mav_put_int16_t(buf, 12, wp_id);
    _mav_put_int16_t(buf, 14, prev_wp_id);
    _mav_put_int16_t(buf, 16, next_wp_id);
    _mav_put_int16_t(buf, 18, navigation_speed);
    _mav_put_int16_t(buf, 20, action_wait_time);
    _mav_put_int16_t(buf, 22, action_activate_camera);
    _mav_put_int16_t(buf, 24, action_zoom_control);
    _mav_put_int16_t(buf, 26, action_gimbal_pitch);
    _mav_put_int16_t(buf, 28, action_gimbal_yaw);
    _mav_put_int8_t(buf, 30, type);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ADD_UPDATE_WAYPOINT, buf, MAVLINK_MSG_ID_ADD_UPDATE_WAYPOINT_MIN_LEN, MAVLINK_MSG_ID_ADD_UPDATE_WAYPOINT_LEN, MAVLINK_MSG_ID_ADD_UPDATE_WAYPOINT_CRC);
#else
    mavlink_add_update_waypoint_t *packet = (mavlink_add_update_waypoint_t *)msgbuf;
    packet->latitude = latitude;
    packet->longitude = longitude;
    packet->altitude = altitude;
    packet->wp_id = wp_id;
    packet->prev_wp_id = prev_wp_id;
    packet->next_wp_id = next_wp_id;
    packet->navigation_speed = navigation_speed;
    packet->action_wait_time = action_wait_time;
    packet->action_activate_camera = action_activate_camera;
    packet->action_zoom_control = action_zoom_control;
    packet->action_gimbal_pitch = action_gimbal_pitch;
    packet->action_gimbal_yaw = action_gimbal_yaw;
    packet->type = type;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ADD_UPDATE_WAYPOINT, (const char *)packet, MAVLINK_MSG_ID_ADD_UPDATE_WAYPOINT_MIN_LEN, MAVLINK_MSG_ID_ADD_UPDATE_WAYPOINT_LEN, MAVLINK_MSG_ID_ADD_UPDATE_WAYPOINT_CRC);
#endif
}
#endif

#endif

// MESSAGE ADD_UPDATE_WAYPOINT UNPACKING


/**
 * @brief Get field latitude from add_update_waypoint message
 *
 * @return  Latitude of waypoint in degrees * 1E7 
 */
static inline int32_t mavlink_msg_add_update_waypoint_get_latitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  0);
}

/**
 * @brief Get field longitude from add_update_waypoint message
 *
 * @return  Longitude of waypoint in degrees * 1E7 
 */
static inline int32_t mavlink_msg_add_update_waypoint_get_longitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  4);
}

/**
 * @brief Get field altitude from add_update_waypoint message
 *
 * @return  Altitude of waypoint in meters (MSL) 
 */
static inline int32_t mavlink_msg_add_update_waypoint_get_altitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  8);
}

/**
 * @brief Get field type from add_update_waypoint message
 *
 * @return  enum MISSION_WAYPOINT_TYPE. Type of waypoint 
 */
static inline int8_t mavlink_msg_add_update_waypoint_get_type(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int8_t(msg,  30);
}

/**
 * @brief Get field wp_id from add_update_waypoint message
 *
 * @return  Unique id of the waypoint. range[0-101] (0=unidentified, 1-100=waypoint 101=rtl_point) 
 */
static inline int16_t mavlink_msg_add_update_waypoint_get_wp_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  12);
}

/**
 * @brief Get field prev_wp_id from add_update_waypoint message
 *
 * @return  Previous waypoint id. range[0-101] (0=no waypoint), 0 indicates that this waypoint is the first waypoint 
 */
static inline int16_t mavlink_msg_add_update_waypoint_get_prev_wp_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  14);
}

/**
 * @brief Get field next_wp_id from add_update_waypoint message
 *
 * @return  Next waypoint id. range[0-101] (0=no waypoint), 0 indicates that this waypoint is the last waypoint 
 */
static inline int16_t mavlink_msg_add_update_waypoint_get_next_wp_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  16);
}

/**
 * @brief Get field navigation_speed from add_update_waypoint message
 *
 * @return  Defines max navigation speed of the aircraft while navigating through this waypoint 
 */
static inline int16_t mavlink_msg_add_update_waypoint_get_navigation_speed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  18);
}

/**
 * @brief Get field action_wait_time from add_update_waypoint message
 *
 * @return  Defines wait action on waypoint. Aircraft waits specified seconds on waypoint 
 */
static inline int16_t mavlink_msg_add_update_waypoint_get_action_wait_time(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  20);
}

/**
 * @brief Get field action_activate_camera from add_update_waypoint message
 *
 * @return  enum TOGAN_CAMERA_TYPE. Specified camera selected on waypoint. (NONE=camera selection not to be changed) 
 */
static inline int16_t mavlink_msg_add_update_waypoint_get_action_activate_camera(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  22);
}

/**
 * @brief Get field action_zoom_control from add_update_waypoint message
 *
 * @return  TBD 
 */
static inline int16_t mavlink_msg_add_update_waypoint_get_action_zoom_control(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  24);
}

/**
 * @brief Get field action_gimbal_pitch from add_update_waypoint message
 *
 * @return  TBD 
 */
static inline int16_t mavlink_msg_add_update_waypoint_get_action_gimbal_pitch(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  26);
}

/**
 * @brief Get field action_gimbal_yaw from add_update_waypoint message
 *
 * @return  TBD 
 */
static inline int16_t mavlink_msg_add_update_waypoint_get_action_gimbal_yaw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  28);
}

/**
 * @brief Decode a add_update_waypoint message into a struct
 *
 * @param msg The message to decode
 * @param add_update_waypoint C-struct to decode the message contents into
 */
static inline void mavlink_msg_add_update_waypoint_decode(const mavlink_message_t* msg, mavlink_add_update_waypoint_t* add_update_waypoint)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    add_update_waypoint->latitude = mavlink_msg_add_update_waypoint_get_latitude(msg);
    add_update_waypoint->longitude = mavlink_msg_add_update_waypoint_get_longitude(msg);
    add_update_waypoint->altitude = mavlink_msg_add_update_waypoint_get_altitude(msg);
    add_update_waypoint->wp_id = mavlink_msg_add_update_waypoint_get_wp_id(msg);
    add_update_waypoint->prev_wp_id = mavlink_msg_add_update_waypoint_get_prev_wp_id(msg);
    add_update_waypoint->next_wp_id = mavlink_msg_add_update_waypoint_get_next_wp_id(msg);
    add_update_waypoint->navigation_speed = mavlink_msg_add_update_waypoint_get_navigation_speed(msg);
    add_update_waypoint->action_wait_time = mavlink_msg_add_update_waypoint_get_action_wait_time(msg);
    add_update_waypoint->action_activate_camera = mavlink_msg_add_update_waypoint_get_action_activate_camera(msg);
    add_update_waypoint->action_zoom_control = mavlink_msg_add_update_waypoint_get_action_zoom_control(msg);
    add_update_waypoint->action_gimbal_pitch = mavlink_msg_add_update_waypoint_get_action_gimbal_pitch(msg);
    add_update_waypoint->action_gimbal_yaw = mavlink_msg_add_update_waypoint_get_action_gimbal_yaw(msg);
    add_update_waypoint->type = mavlink_msg_add_update_waypoint_get_type(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ADD_UPDATE_WAYPOINT_LEN? msg->len : MAVLINK_MSG_ID_ADD_UPDATE_WAYPOINT_LEN;
        memset(add_update_waypoint, 0, MAVLINK_MSG_ID_ADD_UPDATE_WAYPOINT_LEN);
    memcpy(add_update_waypoint, _MAV_PAYLOAD(msg), len);
#endif
}
