#pragma once
// MESSAGE WAYPOINT_UPDATE PACKING

#define MAVLINK_MSG_ID_WAYPOINT_UPDATE 199

MAVPACKED(
typedef struct __mavlink_waypoint_update_t {
 int16_t current_wp_id; /*<  Current waypoint id. Range [0-101] */
 int16_t dest_wp_id; /*<  Destination waypoint id. Range [0-101] */
 uint16_t lap; /*<  Specifies current lap count of mission plan. */
}) mavlink_waypoint_update_t;

#define MAVLINK_MSG_ID_WAYPOINT_UPDATE_LEN 6
#define MAVLINK_MSG_ID_WAYPOINT_UPDATE_MIN_LEN 6
#define MAVLINK_MSG_ID_199_LEN 6
#define MAVLINK_MSG_ID_199_MIN_LEN 6

#define MAVLINK_MSG_ID_WAYPOINT_UPDATE_CRC 63
#define MAVLINK_MSG_ID_199_CRC 63



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_WAYPOINT_UPDATE { \
    199, \
    "WAYPOINT_UPDATE", \
    3, \
    {  { "current_wp_id", NULL, MAVLINK_TYPE_INT16_T, 0, 0, offsetof(mavlink_waypoint_update_t, current_wp_id) }, \
         { "dest_wp_id", NULL, MAVLINK_TYPE_INT16_T, 0, 2, offsetof(mavlink_waypoint_update_t, dest_wp_id) }, \
         { "lap", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_waypoint_update_t, lap) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_WAYPOINT_UPDATE { \
    "WAYPOINT_UPDATE", \
    3, \
    {  { "current_wp_id", NULL, MAVLINK_TYPE_INT16_T, 0, 0, offsetof(mavlink_waypoint_update_t, current_wp_id) }, \
         { "dest_wp_id", NULL, MAVLINK_TYPE_INT16_T, 0, 2, offsetof(mavlink_waypoint_update_t, dest_wp_id) }, \
         { "lap", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_waypoint_update_t, lap) }, \
         } \
}
#endif

/**
 * @brief Pack a waypoint_update message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param current_wp_id  Current waypoint id. Range [0-101] 
 * @param dest_wp_id  Destination waypoint id. Range [0-101] 
 * @param lap  Specifies current lap count of mission plan. 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_waypoint_update_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               int16_t current_wp_id, int16_t dest_wp_id, uint16_t lap)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_WAYPOINT_UPDATE_LEN];
    _mav_put_int16_t(buf, 0, current_wp_id);
    _mav_put_int16_t(buf, 2, dest_wp_id);
    _mav_put_uint16_t(buf, 4, lap);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_WAYPOINT_UPDATE_LEN);
#else
    mavlink_waypoint_update_t packet;
    packet.current_wp_id = current_wp_id;
    packet.dest_wp_id = dest_wp_id;
    packet.lap = lap;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_WAYPOINT_UPDATE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_WAYPOINT_UPDATE;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_WAYPOINT_UPDATE_MIN_LEN, MAVLINK_MSG_ID_WAYPOINT_UPDATE_LEN, MAVLINK_MSG_ID_WAYPOINT_UPDATE_CRC);
}

/**
 * @brief Pack a waypoint_update message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param current_wp_id  Current waypoint id. Range [0-101] 
 * @param dest_wp_id  Destination waypoint id. Range [0-101] 
 * @param lap  Specifies current lap count of mission plan. 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_waypoint_update_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   int16_t current_wp_id,int16_t dest_wp_id,uint16_t lap)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_WAYPOINT_UPDATE_LEN];
    _mav_put_int16_t(buf, 0, current_wp_id);
    _mav_put_int16_t(buf, 2, dest_wp_id);
    _mav_put_uint16_t(buf, 4, lap);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_WAYPOINT_UPDATE_LEN);
#else
    mavlink_waypoint_update_t packet;
    packet.current_wp_id = current_wp_id;
    packet.dest_wp_id = dest_wp_id;
    packet.lap = lap;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_WAYPOINT_UPDATE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_WAYPOINT_UPDATE;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_WAYPOINT_UPDATE_MIN_LEN, MAVLINK_MSG_ID_WAYPOINT_UPDATE_LEN, MAVLINK_MSG_ID_WAYPOINT_UPDATE_CRC);
}

/**
 * @brief Encode a waypoint_update struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param waypoint_update C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_waypoint_update_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_waypoint_update_t* waypoint_update)
{
    return mavlink_msg_waypoint_update_pack(system_id, component_id, msg, waypoint_update->current_wp_id, waypoint_update->dest_wp_id, waypoint_update->lap);
}

/**
 * @brief Encode a waypoint_update struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param waypoint_update C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_waypoint_update_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_waypoint_update_t* waypoint_update)
{
    return mavlink_msg_waypoint_update_pack_chan(system_id, component_id, chan, msg, waypoint_update->current_wp_id, waypoint_update->dest_wp_id, waypoint_update->lap);
}

/**
 * @brief Send a waypoint_update message
 * @param chan MAVLink channel to send the message
 *
 * @param current_wp_id  Current waypoint id. Range [0-101] 
 * @param dest_wp_id  Destination waypoint id. Range [0-101] 
 * @param lap  Specifies current lap count of mission plan. 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_waypoint_update_send(mavlink_channel_t chan, int16_t current_wp_id, int16_t dest_wp_id, uint16_t lap)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_WAYPOINT_UPDATE_LEN];
    _mav_put_int16_t(buf, 0, current_wp_id);
    _mav_put_int16_t(buf, 2, dest_wp_id);
    _mav_put_uint16_t(buf, 4, lap);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WAYPOINT_UPDATE, buf, MAVLINK_MSG_ID_WAYPOINT_UPDATE_MIN_LEN, MAVLINK_MSG_ID_WAYPOINT_UPDATE_LEN, MAVLINK_MSG_ID_WAYPOINT_UPDATE_CRC);
#else
    mavlink_waypoint_update_t packet;
    packet.current_wp_id = current_wp_id;
    packet.dest_wp_id = dest_wp_id;
    packet.lap = lap;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WAYPOINT_UPDATE, (const char *)&packet, MAVLINK_MSG_ID_WAYPOINT_UPDATE_MIN_LEN, MAVLINK_MSG_ID_WAYPOINT_UPDATE_LEN, MAVLINK_MSG_ID_WAYPOINT_UPDATE_CRC);
#endif
}

/**
 * @brief Send a waypoint_update message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_waypoint_update_send_struct(mavlink_channel_t chan, const mavlink_waypoint_update_t* waypoint_update)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_waypoint_update_send(chan, waypoint_update->current_wp_id, waypoint_update->dest_wp_id, waypoint_update->lap);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WAYPOINT_UPDATE, (const char *)waypoint_update, MAVLINK_MSG_ID_WAYPOINT_UPDATE_MIN_LEN, MAVLINK_MSG_ID_WAYPOINT_UPDATE_LEN, MAVLINK_MSG_ID_WAYPOINT_UPDATE_CRC);
#endif
}

#if MAVLINK_MSG_ID_WAYPOINT_UPDATE_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_waypoint_update_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  int16_t current_wp_id, int16_t dest_wp_id, uint16_t lap)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_int16_t(buf, 0, current_wp_id);
    _mav_put_int16_t(buf, 2, dest_wp_id);
    _mav_put_uint16_t(buf, 4, lap);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WAYPOINT_UPDATE, buf, MAVLINK_MSG_ID_WAYPOINT_UPDATE_MIN_LEN, MAVLINK_MSG_ID_WAYPOINT_UPDATE_LEN, MAVLINK_MSG_ID_WAYPOINT_UPDATE_CRC);
#else
    mavlink_waypoint_update_t *packet = (mavlink_waypoint_update_t *)msgbuf;
    packet->current_wp_id = current_wp_id;
    packet->dest_wp_id = dest_wp_id;
    packet->lap = lap;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WAYPOINT_UPDATE, (const char *)packet, MAVLINK_MSG_ID_WAYPOINT_UPDATE_MIN_LEN, MAVLINK_MSG_ID_WAYPOINT_UPDATE_LEN, MAVLINK_MSG_ID_WAYPOINT_UPDATE_CRC);
#endif
}
#endif

#endif

// MESSAGE WAYPOINT_UPDATE UNPACKING


/**
 * @brief Get field current_wp_id from waypoint_update message
 *
 * @return  Current waypoint id. Range [0-101] 
 */
static inline int16_t mavlink_msg_waypoint_update_get_current_wp_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  0);
}

/**
 * @brief Get field dest_wp_id from waypoint_update message
 *
 * @return  Destination waypoint id. Range [0-101] 
 */
static inline int16_t mavlink_msg_waypoint_update_get_dest_wp_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  2);
}

/**
 * @brief Get field lap from waypoint_update message
 *
 * @return  Specifies current lap count of mission plan. 
 */
static inline uint16_t mavlink_msg_waypoint_update_get_lap(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  4);
}

/**
 * @brief Decode a waypoint_update message into a struct
 *
 * @param msg The message to decode
 * @param waypoint_update C-struct to decode the message contents into
 */
static inline void mavlink_msg_waypoint_update_decode(const mavlink_message_t* msg, mavlink_waypoint_update_t* waypoint_update)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    waypoint_update->current_wp_id = mavlink_msg_waypoint_update_get_current_wp_id(msg);
    waypoint_update->dest_wp_id = mavlink_msg_waypoint_update_get_dest_wp_id(msg);
    waypoint_update->lap = mavlink_msg_waypoint_update_get_lap(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_WAYPOINT_UPDATE_LEN? msg->len : MAVLINK_MSG_ID_WAYPOINT_UPDATE_LEN;
        memset(waypoint_update, 0, MAVLINK_MSG_ID_WAYPOINT_UPDATE_LEN);
    memcpy(waypoint_update, _MAV_PAYLOAD(msg), len);
#endif
}
