#pragma once
// MESSAGE UPDATE_MISSION_PLAN_PARAMS PACKING

#define MAVLINK_MSG_ID_UPDATE_MISSION_PLAN_PARAMS 205

MAVPACKED(
typedef struct __mavlink_update_mission_plan_params_t {
 uint16_t lap_count; /*<  Specifies lap count of mission plan. 0 = infinite number of lap */
}) mavlink_update_mission_plan_params_t;

#define MAVLINK_MSG_ID_UPDATE_MISSION_PLAN_PARAMS_LEN 2
#define MAVLINK_MSG_ID_UPDATE_MISSION_PLAN_PARAMS_MIN_LEN 2
#define MAVLINK_MSG_ID_205_LEN 2
#define MAVLINK_MSG_ID_205_MIN_LEN 2

#define MAVLINK_MSG_ID_UPDATE_MISSION_PLAN_PARAMS_CRC 4
#define MAVLINK_MSG_ID_205_CRC 4



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_UPDATE_MISSION_PLAN_PARAMS { \
    205, \
    "UPDATE_MISSION_PLAN_PARAMS", \
    1, \
    {  { "lap_count", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_update_mission_plan_params_t, lap_count) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_UPDATE_MISSION_PLAN_PARAMS { \
    "UPDATE_MISSION_PLAN_PARAMS", \
    1, \
    {  { "lap_count", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_update_mission_plan_params_t, lap_count) }, \
         } \
}
#endif

/**
 * @brief Pack a update_mission_plan_params message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param lap_count  Specifies lap count of mission plan. 0 = infinite number of lap 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_update_mission_plan_params_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint16_t lap_count)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_UPDATE_MISSION_PLAN_PARAMS_LEN];
    _mav_put_uint16_t(buf, 0, lap_count);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_UPDATE_MISSION_PLAN_PARAMS_LEN);
#else
    mavlink_update_mission_plan_params_t packet;
    packet.lap_count = lap_count;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_UPDATE_MISSION_PLAN_PARAMS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_UPDATE_MISSION_PLAN_PARAMS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_UPDATE_MISSION_PLAN_PARAMS_MIN_LEN, MAVLINK_MSG_ID_UPDATE_MISSION_PLAN_PARAMS_LEN, MAVLINK_MSG_ID_UPDATE_MISSION_PLAN_PARAMS_CRC);
}

/**
 * @brief Pack a update_mission_plan_params message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param lap_count  Specifies lap count of mission plan. 0 = infinite number of lap 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_update_mission_plan_params_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint16_t lap_count)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_UPDATE_MISSION_PLAN_PARAMS_LEN];
    _mav_put_uint16_t(buf, 0, lap_count);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_UPDATE_MISSION_PLAN_PARAMS_LEN);
#else
    mavlink_update_mission_plan_params_t packet;
    packet.lap_count = lap_count;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_UPDATE_MISSION_PLAN_PARAMS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_UPDATE_MISSION_PLAN_PARAMS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_UPDATE_MISSION_PLAN_PARAMS_MIN_LEN, MAVLINK_MSG_ID_UPDATE_MISSION_PLAN_PARAMS_LEN, MAVLINK_MSG_ID_UPDATE_MISSION_PLAN_PARAMS_CRC);
}

/**
 * @brief Encode a update_mission_plan_params struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param update_mission_plan_params C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_update_mission_plan_params_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_update_mission_plan_params_t* update_mission_plan_params)
{
    return mavlink_msg_update_mission_plan_params_pack(system_id, component_id, msg, update_mission_plan_params->lap_count);
}

/**
 * @brief Encode a update_mission_plan_params struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param update_mission_plan_params C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_update_mission_plan_params_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_update_mission_plan_params_t* update_mission_plan_params)
{
    return mavlink_msg_update_mission_plan_params_pack_chan(system_id, component_id, chan, msg, update_mission_plan_params->lap_count);
}

/**
 * @brief Send a update_mission_plan_params message
 * @param chan MAVLink channel to send the message
 *
 * @param lap_count  Specifies lap count of mission plan. 0 = infinite number of lap 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_update_mission_plan_params_send(mavlink_channel_t chan, uint16_t lap_count)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_UPDATE_MISSION_PLAN_PARAMS_LEN];
    _mav_put_uint16_t(buf, 0, lap_count);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UPDATE_MISSION_PLAN_PARAMS, buf, MAVLINK_MSG_ID_UPDATE_MISSION_PLAN_PARAMS_MIN_LEN, MAVLINK_MSG_ID_UPDATE_MISSION_PLAN_PARAMS_LEN, MAVLINK_MSG_ID_UPDATE_MISSION_PLAN_PARAMS_CRC);
#else
    mavlink_update_mission_plan_params_t packet;
    packet.lap_count = lap_count;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UPDATE_MISSION_PLAN_PARAMS, (const char *)&packet, MAVLINK_MSG_ID_UPDATE_MISSION_PLAN_PARAMS_MIN_LEN, MAVLINK_MSG_ID_UPDATE_MISSION_PLAN_PARAMS_LEN, MAVLINK_MSG_ID_UPDATE_MISSION_PLAN_PARAMS_CRC);
#endif
}

/**
 * @brief Send a update_mission_plan_params message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_update_mission_plan_params_send_struct(mavlink_channel_t chan, const mavlink_update_mission_plan_params_t* update_mission_plan_params)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_update_mission_plan_params_send(chan, update_mission_plan_params->lap_count);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UPDATE_MISSION_PLAN_PARAMS, (const char *)update_mission_plan_params, MAVLINK_MSG_ID_UPDATE_MISSION_PLAN_PARAMS_MIN_LEN, MAVLINK_MSG_ID_UPDATE_MISSION_PLAN_PARAMS_LEN, MAVLINK_MSG_ID_UPDATE_MISSION_PLAN_PARAMS_CRC);
#endif
}

#if MAVLINK_MSG_ID_UPDATE_MISSION_PLAN_PARAMS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_update_mission_plan_params_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint16_t lap_count)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint16_t(buf, 0, lap_count);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UPDATE_MISSION_PLAN_PARAMS, buf, MAVLINK_MSG_ID_UPDATE_MISSION_PLAN_PARAMS_MIN_LEN, MAVLINK_MSG_ID_UPDATE_MISSION_PLAN_PARAMS_LEN, MAVLINK_MSG_ID_UPDATE_MISSION_PLAN_PARAMS_CRC);
#else
    mavlink_update_mission_plan_params_t *packet = (mavlink_update_mission_plan_params_t *)msgbuf;
    packet->lap_count = lap_count;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UPDATE_MISSION_PLAN_PARAMS, (const char *)packet, MAVLINK_MSG_ID_UPDATE_MISSION_PLAN_PARAMS_MIN_LEN, MAVLINK_MSG_ID_UPDATE_MISSION_PLAN_PARAMS_LEN, MAVLINK_MSG_ID_UPDATE_MISSION_PLAN_PARAMS_CRC);
#endif
}
#endif

#endif

// MESSAGE UPDATE_MISSION_PLAN_PARAMS UNPACKING


/**
 * @brief Get field lap_count from update_mission_plan_params message
 *
 * @return  Specifies lap count of mission plan. 0 = infinite number of lap 
 */
static inline uint16_t mavlink_msg_update_mission_plan_params_get_lap_count(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Decode a update_mission_plan_params message into a struct
 *
 * @param msg The message to decode
 * @param update_mission_plan_params C-struct to decode the message contents into
 */
static inline void mavlink_msg_update_mission_plan_params_decode(const mavlink_message_t* msg, mavlink_update_mission_plan_params_t* update_mission_plan_params)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    update_mission_plan_params->lap_count = mavlink_msg_update_mission_plan_params_get_lap_count(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_UPDATE_MISSION_PLAN_PARAMS_LEN? msg->len : MAVLINK_MSG_ID_UPDATE_MISSION_PLAN_PARAMS_LEN;
        memset(update_mission_plan_params, 0, MAVLINK_MSG_ID_UPDATE_MISSION_PLAN_PARAMS_LEN);
    memcpy(update_mission_plan_params, _MAV_PAYLOAD(msg), len);
#endif
}
