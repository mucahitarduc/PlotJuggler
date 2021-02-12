#pragma once
// MESSAGE GCS_TO_MCU_COMMAND PACKING

#define MAVLINK_MSG_ID_GCS_TO_MCU_COMMAND 196

MAVPACKED(
typedef struct __mavlink_gcs_to_mcu_command_t {
 uint32_t command_type; /*<  enum GCS_TO_MCU_COMMAND_TYPE */
 uint32_t param; /*< */
}) mavlink_gcs_to_mcu_command_t;

#define MAVLINK_MSG_ID_GCS_TO_MCU_COMMAND_LEN 8
#define MAVLINK_MSG_ID_GCS_TO_MCU_COMMAND_MIN_LEN 8
#define MAVLINK_MSG_ID_196_LEN 8
#define MAVLINK_MSG_ID_196_MIN_LEN 8

#define MAVLINK_MSG_ID_GCS_TO_MCU_COMMAND_CRC 219
#define MAVLINK_MSG_ID_196_CRC 219



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_GCS_TO_MCU_COMMAND { \
    196, \
    "GCS_TO_MCU_COMMAND", \
    2, \
    {  { "command_type", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_gcs_to_mcu_command_t, command_type) }, \
         { "param", NULL, MAVLINK_TYPE_UINT32_T, 0, 4, offsetof(mavlink_gcs_to_mcu_command_t, param) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_GCS_TO_MCU_COMMAND { \
    "GCS_TO_MCU_COMMAND", \
    2, \
    {  { "command_type", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_gcs_to_mcu_command_t, command_type) }, \
         { "param", NULL, MAVLINK_TYPE_UINT32_T, 0, 4, offsetof(mavlink_gcs_to_mcu_command_t, param) }, \
         } \
}
#endif

/**
 * @brief Pack a gcs_to_mcu_command message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param command_type  enum GCS_TO_MCU_COMMAND_TYPE 
 * @param param 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gcs_to_mcu_command_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t command_type, uint32_t param)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GCS_TO_MCU_COMMAND_LEN];
    _mav_put_uint32_t(buf, 0, command_type);
    _mav_put_uint32_t(buf, 4, param);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GCS_TO_MCU_COMMAND_LEN);
#else
    mavlink_gcs_to_mcu_command_t packet;
    packet.command_type = command_type;
    packet.param = param;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GCS_TO_MCU_COMMAND_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GCS_TO_MCU_COMMAND;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_GCS_TO_MCU_COMMAND_MIN_LEN, MAVLINK_MSG_ID_GCS_TO_MCU_COMMAND_LEN, MAVLINK_MSG_ID_GCS_TO_MCU_COMMAND_CRC);
}

/**
 * @brief Pack a gcs_to_mcu_command message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param command_type  enum GCS_TO_MCU_COMMAND_TYPE 
 * @param param 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gcs_to_mcu_command_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t command_type,uint32_t param)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GCS_TO_MCU_COMMAND_LEN];
    _mav_put_uint32_t(buf, 0, command_type);
    _mav_put_uint32_t(buf, 4, param);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GCS_TO_MCU_COMMAND_LEN);
#else
    mavlink_gcs_to_mcu_command_t packet;
    packet.command_type = command_type;
    packet.param = param;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GCS_TO_MCU_COMMAND_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GCS_TO_MCU_COMMAND;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_GCS_TO_MCU_COMMAND_MIN_LEN, MAVLINK_MSG_ID_GCS_TO_MCU_COMMAND_LEN, MAVLINK_MSG_ID_GCS_TO_MCU_COMMAND_CRC);
}

/**
 * @brief Encode a gcs_to_mcu_command struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param gcs_to_mcu_command C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gcs_to_mcu_command_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_gcs_to_mcu_command_t* gcs_to_mcu_command)
{
    return mavlink_msg_gcs_to_mcu_command_pack(system_id, component_id, msg, gcs_to_mcu_command->command_type, gcs_to_mcu_command->param);
}

/**
 * @brief Encode a gcs_to_mcu_command struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param gcs_to_mcu_command C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gcs_to_mcu_command_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_gcs_to_mcu_command_t* gcs_to_mcu_command)
{
    return mavlink_msg_gcs_to_mcu_command_pack_chan(system_id, component_id, chan, msg, gcs_to_mcu_command->command_type, gcs_to_mcu_command->param);
}

/**
 * @brief Send a gcs_to_mcu_command message
 * @param chan MAVLink channel to send the message
 *
 * @param command_type  enum GCS_TO_MCU_COMMAND_TYPE 
 * @param param 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_gcs_to_mcu_command_send(mavlink_channel_t chan, uint32_t command_type, uint32_t param)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GCS_TO_MCU_COMMAND_LEN];
    _mav_put_uint32_t(buf, 0, command_type);
    _mav_put_uint32_t(buf, 4, param);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GCS_TO_MCU_COMMAND, buf, MAVLINK_MSG_ID_GCS_TO_MCU_COMMAND_MIN_LEN, MAVLINK_MSG_ID_GCS_TO_MCU_COMMAND_LEN, MAVLINK_MSG_ID_GCS_TO_MCU_COMMAND_CRC);
#else
    mavlink_gcs_to_mcu_command_t packet;
    packet.command_type = command_type;
    packet.param = param;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GCS_TO_MCU_COMMAND, (const char *)&packet, MAVLINK_MSG_ID_GCS_TO_MCU_COMMAND_MIN_LEN, MAVLINK_MSG_ID_GCS_TO_MCU_COMMAND_LEN, MAVLINK_MSG_ID_GCS_TO_MCU_COMMAND_CRC);
#endif
}

/**
 * @brief Send a gcs_to_mcu_command message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_gcs_to_mcu_command_send_struct(mavlink_channel_t chan, const mavlink_gcs_to_mcu_command_t* gcs_to_mcu_command)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_gcs_to_mcu_command_send(chan, gcs_to_mcu_command->command_type, gcs_to_mcu_command->param);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GCS_TO_MCU_COMMAND, (const char *)gcs_to_mcu_command, MAVLINK_MSG_ID_GCS_TO_MCU_COMMAND_MIN_LEN, MAVLINK_MSG_ID_GCS_TO_MCU_COMMAND_LEN, MAVLINK_MSG_ID_GCS_TO_MCU_COMMAND_CRC);
#endif
}

#if MAVLINK_MSG_ID_GCS_TO_MCU_COMMAND_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_gcs_to_mcu_command_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t command_type, uint32_t param)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, command_type);
    _mav_put_uint32_t(buf, 4, param);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GCS_TO_MCU_COMMAND, buf, MAVLINK_MSG_ID_GCS_TO_MCU_COMMAND_MIN_LEN, MAVLINK_MSG_ID_GCS_TO_MCU_COMMAND_LEN, MAVLINK_MSG_ID_GCS_TO_MCU_COMMAND_CRC);
#else
    mavlink_gcs_to_mcu_command_t *packet = (mavlink_gcs_to_mcu_command_t *)msgbuf;
    packet->command_type = command_type;
    packet->param = param;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GCS_TO_MCU_COMMAND, (const char *)packet, MAVLINK_MSG_ID_GCS_TO_MCU_COMMAND_MIN_LEN, MAVLINK_MSG_ID_GCS_TO_MCU_COMMAND_LEN, MAVLINK_MSG_ID_GCS_TO_MCU_COMMAND_CRC);
#endif
}
#endif

#endif

// MESSAGE GCS_TO_MCU_COMMAND UNPACKING


/**
 * @brief Get field command_type from gcs_to_mcu_command message
 *
 * @return  enum GCS_TO_MCU_COMMAND_TYPE 
 */
static inline uint32_t mavlink_msg_gcs_to_mcu_command_get_command_type(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field param from gcs_to_mcu_command message
 *
 * @return 
 */
static inline uint32_t mavlink_msg_gcs_to_mcu_command_get_param(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  4);
}

/**
 * @brief Decode a gcs_to_mcu_command message into a struct
 *
 * @param msg The message to decode
 * @param gcs_to_mcu_command C-struct to decode the message contents into
 */
static inline void mavlink_msg_gcs_to_mcu_command_decode(const mavlink_message_t* msg, mavlink_gcs_to_mcu_command_t* gcs_to_mcu_command)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    gcs_to_mcu_command->command_type = mavlink_msg_gcs_to_mcu_command_get_command_type(msg);
    gcs_to_mcu_command->param = mavlink_msg_gcs_to_mcu_command_get_param(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_GCS_TO_MCU_COMMAND_LEN? msg->len : MAVLINK_MSG_ID_GCS_TO_MCU_COMMAND_LEN;
        memset(gcs_to_mcu_command, 0, MAVLINK_MSG_ID_GCS_TO_MCU_COMMAND_LEN);
    memcpy(gcs_to_mcu_command, _MAV_PAYLOAD(msg), len);
#endif
}
