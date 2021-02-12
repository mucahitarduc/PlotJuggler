#pragma once
// MESSAGE GCS_TO_MCU_GIMBAL_MOTOR_COMMAND PACKING

#define MAVLINK_MSG_ID_GCS_TO_MCU_GIMBAL_MOTOR_COMMAND 209

MAVPACKED(
typedef struct __mavlink_gcs_to_mcu_gimbal_motor_command_t {
 uint32_t cmd; /*<  enum GIMBAL_MOTOR_COMMAND_TYPE*/
}) mavlink_gcs_to_mcu_gimbal_motor_command_t;

#define MAVLINK_MSG_ID_GCS_TO_MCU_GIMBAL_MOTOR_COMMAND_LEN 4
#define MAVLINK_MSG_ID_GCS_TO_MCU_GIMBAL_MOTOR_COMMAND_MIN_LEN 4
#define MAVLINK_MSG_ID_209_LEN 4
#define MAVLINK_MSG_ID_209_MIN_LEN 4

#define MAVLINK_MSG_ID_GCS_TO_MCU_GIMBAL_MOTOR_COMMAND_CRC 178
#define MAVLINK_MSG_ID_209_CRC 178



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_GCS_TO_MCU_GIMBAL_MOTOR_COMMAND { \
    209, \
    "GCS_TO_MCU_GIMBAL_MOTOR_COMMAND", \
    1, \
    {  { "cmd", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_gcs_to_mcu_gimbal_motor_command_t, cmd) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_GCS_TO_MCU_GIMBAL_MOTOR_COMMAND { \
    "GCS_TO_MCU_GIMBAL_MOTOR_COMMAND", \
    1, \
    {  { "cmd", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_gcs_to_mcu_gimbal_motor_command_t, cmd) }, \
         } \
}
#endif

/**
 * @brief Pack a gcs_to_mcu_gimbal_motor_command message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param cmd  enum GIMBAL_MOTOR_COMMAND_TYPE
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gcs_to_mcu_gimbal_motor_command_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t cmd)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GCS_TO_MCU_GIMBAL_MOTOR_COMMAND_LEN];
    _mav_put_uint32_t(buf, 0, cmd);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GCS_TO_MCU_GIMBAL_MOTOR_COMMAND_LEN);
#else
    mavlink_gcs_to_mcu_gimbal_motor_command_t packet;
    packet.cmd = cmd;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GCS_TO_MCU_GIMBAL_MOTOR_COMMAND_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GCS_TO_MCU_GIMBAL_MOTOR_COMMAND;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_GCS_TO_MCU_GIMBAL_MOTOR_COMMAND_MIN_LEN, MAVLINK_MSG_ID_GCS_TO_MCU_GIMBAL_MOTOR_COMMAND_LEN, MAVLINK_MSG_ID_GCS_TO_MCU_GIMBAL_MOTOR_COMMAND_CRC);
}

/**
 * @brief Pack a gcs_to_mcu_gimbal_motor_command message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param cmd  enum GIMBAL_MOTOR_COMMAND_TYPE
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gcs_to_mcu_gimbal_motor_command_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t cmd)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GCS_TO_MCU_GIMBAL_MOTOR_COMMAND_LEN];
    _mav_put_uint32_t(buf, 0, cmd);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GCS_TO_MCU_GIMBAL_MOTOR_COMMAND_LEN);
#else
    mavlink_gcs_to_mcu_gimbal_motor_command_t packet;
    packet.cmd = cmd;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GCS_TO_MCU_GIMBAL_MOTOR_COMMAND_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GCS_TO_MCU_GIMBAL_MOTOR_COMMAND;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_GCS_TO_MCU_GIMBAL_MOTOR_COMMAND_MIN_LEN, MAVLINK_MSG_ID_GCS_TO_MCU_GIMBAL_MOTOR_COMMAND_LEN, MAVLINK_MSG_ID_GCS_TO_MCU_GIMBAL_MOTOR_COMMAND_CRC);
}

/**
 * @brief Encode a gcs_to_mcu_gimbal_motor_command struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param gcs_to_mcu_gimbal_motor_command C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gcs_to_mcu_gimbal_motor_command_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_gcs_to_mcu_gimbal_motor_command_t* gcs_to_mcu_gimbal_motor_command)
{
    return mavlink_msg_gcs_to_mcu_gimbal_motor_command_pack(system_id, component_id, msg, gcs_to_mcu_gimbal_motor_command->cmd);
}

/**
 * @brief Encode a gcs_to_mcu_gimbal_motor_command struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param gcs_to_mcu_gimbal_motor_command C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gcs_to_mcu_gimbal_motor_command_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_gcs_to_mcu_gimbal_motor_command_t* gcs_to_mcu_gimbal_motor_command)
{
    return mavlink_msg_gcs_to_mcu_gimbal_motor_command_pack_chan(system_id, component_id, chan, msg, gcs_to_mcu_gimbal_motor_command->cmd);
}

/**
 * @brief Send a gcs_to_mcu_gimbal_motor_command message
 * @param chan MAVLink channel to send the message
 *
 * @param cmd  enum GIMBAL_MOTOR_COMMAND_TYPE
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_gcs_to_mcu_gimbal_motor_command_send(mavlink_channel_t chan, uint32_t cmd)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GCS_TO_MCU_GIMBAL_MOTOR_COMMAND_LEN];
    _mav_put_uint32_t(buf, 0, cmd);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GCS_TO_MCU_GIMBAL_MOTOR_COMMAND, buf, MAVLINK_MSG_ID_GCS_TO_MCU_GIMBAL_MOTOR_COMMAND_MIN_LEN, MAVLINK_MSG_ID_GCS_TO_MCU_GIMBAL_MOTOR_COMMAND_LEN, MAVLINK_MSG_ID_GCS_TO_MCU_GIMBAL_MOTOR_COMMAND_CRC);
#else
    mavlink_gcs_to_mcu_gimbal_motor_command_t packet;
    packet.cmd = cmd;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GCS_TO_MCU_GIMBAL_MOTOR_COMMAND, (const char *)&packet, MAVLINK_MSG_ID_GCS_TO_MCU_GIMBAL_MOTOR_COMMAND_MIN_LEN, MAVLINK_MSG_ID_GCS_TO_MCU_GIMBAL_MOTOR_COMMAND_LEN, MAVLINK_MSG_ID_GCS_TO_MCU_GIMBAL_MOTOR_COMMAND_CRC);
#endif
}

/**
 * @brief Send a gcs_to_mcu_gimbal_motor_command message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_gcs_to_mcu_gimbal_motor_command_send_struct(mavlink_channel_t chan, const mavlink_gcs_to_mcu_gimbal_motor_command_t* gcs_to_mcu_gimbal_motor_command)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_gcs_to_mcu_gimbal_motor_command_send(chan, gcs_to_mcu_gimbal_motor_command->cmd);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GCS_TO_MCU_GIMBAL_MOTOR_COMMAND, (const char *)gcs_to_mcu_gimbal_motor_command, MAVLINK_MSG_ID_GCS_TO_MCU_GIMBAL_MOTOR_COMMAND_MIN_LEN, MAVLINK_MSG_ID_GCS_TO_MCU_GIMBAL_MOTOR_COMMAND_LEN, MAVLINK_MSG_ID_GCS_TO_MCU_GIMBAL_MOTOR_COMMAND_CRC);
#endif
}

#if MAVLINK_MSG_ID_GCS_TO_MCU_GIMBAL_MOTOR_COMMAND_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_gcs_to_mcu_gimbal_motor_command_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t cmd)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, cmd);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GCS_TO_MCU_GIMBAL_MOTOR_COMMAND, buf, MAVLINK_MSG_ID_GCS_TO_MCU_GIMBAL_MOTOR_COMMAND_MIN_LEN, MAVLINK_MSG_ID_GCS_TO_MCU_GIMBAL_MOTOR_COMMAND_LEN, MAVLINK_MSG_ID_GCS_TO_MCU_GIMBAL_MOTOR_COMMAND_CRC);
#else
    mavlink_gcs_to_mcu_gimbal_motor_command_t *packet = (mavlink_gcs_to_mcu_gimbal_motor_command_t *)msgbuf;
    packet->cmd = cmd;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GCS_TO_MCU_GIMBAL_MOTOR_COMMAND, (const char *)packet, MAVLINK_MSG_ID_GCS_TO_MCU_GIMBAL_MOTOR_COMMAND_MIN_LEN, MAVLINK_MSG_ID_GCS_TO_MCU_GIMBAL_MOTOR_COMMAND_LEN, MAVLINK_MSG_ID_GCS_TO_MCU_GIMBAL_MOTOR_COMMAND_CRC);
#endif
}
#endif

#endif

// MESSAGE GCS_TO_MCU_GIMBAL_MOTOR_COMMAND UNPACKING


/**
 * @brief Get field cmd from gcs_to_mcu_gimbal_motor_command message
 *
 * @return  enum GIMBAL_MOTOR_COMMAND_TYPE
 */
static inline uint32_t mavlink_msg_gcs_to_mcu_gimbal_motor_command_get_cmd(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Decode a gcs_to_mcu_gimbal_motor_command message into a struct
 *
 * @param msg The message to decode
 * @param gcs_to_mcu_gimbal_motor_command C-struct to decode the message contents into
 */
static inline void mavlink_msg_gcs_to_mcu_gimbal_motor_command_decode(const mavlink_message_t* msg, mavlink_gcs_to_mcu_gimbal_motor_command_t* gcs_to_mcu_gimbal_motor_command)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    gcs_to_mcu_gimbal_motor_command->cmd = mavlink_msg_gcs_to_mcu_gimbal_motor_command_get_cmd(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_GCS_TO_MCU_GIMBAL_MOTOR_COMMAND_LEN? msg->len : MAVLINK_MSG_ID_GCS_TO_MCU_GIMBAL_MOTOR_COMMAND_LEN;
        memset(gcs_to_mcu_gimbal_motor_command, 0, MAVLINK_MSG_ID_GCS_TO_MCU_GIMBAL_MOTOR_COMMAND_LEN);
    memcpy(gcs_to_mcu_gimbal_motor_command, _MAV_PAYLOAD(msg), len);
#endif
}
