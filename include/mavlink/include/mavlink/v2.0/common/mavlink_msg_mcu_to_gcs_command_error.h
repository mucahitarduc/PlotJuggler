#pragma once
// MESSAGE MCU_TO_GCS_COMMAND_ERROR PACKING

#define MAVLINK_MSG_ID_MCU_TO_GCS_COMMAND_ERROR 212

MAVPACKED(
typedef struct __mavlink_mcu_to_gcs_command_error_t {
 uint32_t error_code; /*<  Command error code */
 uint32_t error_action; /*<  Recommended error action to error code */
}) mavlink_mcu_to_gcs_command_error_t;

#define MAVLINK_MSG_ID_MCU_TO_GCS_COMMAND_ERROR_LEN 8
#define MAVLINK_MSG_ID_MCU_TO_GCS_COMMAND_ERROR_MIN_LEN 8
#define MAVLINK_MSG_ID_212_LEN 8
#define MAVLINK_MSG_ID_212_MIN_LEN 8

#define MAVLINK_MSG_ID_MCU_TO_GCS_COMMAND_ERROR_CRC 134
#define MAVLINK_MSG_ID_212_CRC 134



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_MCU_TO_GCS_COMMAND_ERROR { \
    212, \
    "MCU_TO_GCS_COMMAND_ERROR", \
    2, \
    {  { "error_code", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_mcu_to_gcs_command_error_t, error_code) }, \
         { "error_action", NULL, MAVLINK_TYPE_UINT32_T, 0, 4, offsetof(mavlink_mcu_to_gcs_command_error_t, error_action) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_MCU_TO_GCS_COMMAND_ERROR { \
    "MCU_TO_GCS_COMMAND_ERROR", \
    2, \
    {  { "error_code", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_mcu_to_gcs_command_error_t, error_code) }, \
         { "error_action", NULL, MAVLINK_TYPE_UINT32_T, 0, 4, offsetof(mavlink_mcu_to_gcs_command_error_t, error_action) }, \
         } \
}
#endif

/**
 * @brief Pack a mcu_to_gcs_command_error message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param error_code  Command error code 
 * @param error_action  Recommended error action to error code 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mcu_to_gcs_command_error_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t error_code, uint32_t error_action)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MCU_TO_GCS_COMMAND_ERROR_LEN];
    _mav_put_uint32_t(buf, 0, error_code);
    _mav_put_uint32_t(buf, 4, error_action);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MCU_TO_GCS_COMMAND_ERROR_LEN);
#else
    mavlink_mcu_to_gcs_command_error_t packet;
    packet.error_code = error_code;
    packet.error_action = error_action;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MCU_TO_GCS_COMMAND_ERROR_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MCU_TO_GCS_COMMAND_ERROR;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MCU_TO_GCS_COMMAND_ERROR_MIN_LEN, MAVLINK_MSG_ID_MCU_TO_GCS_COMMAND_ERROR_LEN, MAVLINK_MSG_ID_MCU_TO_GCS_COMMAND_ERROR_CRC);
}

/**
 * @brief Pack a mcu_to_gcs_command_error message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param error_code  Command error code 
 * @param error_action  Recommended error action to error code 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mcu_to_gcs_command_error_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t error_code,uint32_t error_action)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MCU_TO_GCS_COMMAND_ERROR_LEN];
    _mav_put_uint32_t(buf, 0, error_code);
    _mav_put_uint32_t(buf, 4, error_action);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MCU_TO_GCS_COMMAND_ERROR_LEN);
#else
    mavlink_mcu_to_gcs_command_error_t packet;
    packet.error_code = error_code;
    packet.error_action = error_action;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MCU_TO_GCS_COMMAND_ERROR_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MCU_TO_GCS_COMMAND_ERROR;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MCU_TO_GCS_COMMAND_ERROR_MIN_LEN, MAVLINK_MSG_ID_MCU_TO_GCS_COMMAND_ERROR_LEN, MAVLINK_MSG_ID_MCU_TO_GCS_COMMAND_ERROR_CRC);
}

/**
 * @brief Encode a mcu_to_gcs_command_error struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param mcu_to_gcs_command_error C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mcu_to_gcs_command_error_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_mcu_to_gcs_command_error_t* mcu_to_gcs_command_error)
{
    return mavlink_msg_mcu_to_gcs_command_error_pack(system_id, component_id, msg, mcu_to_gcs_command_error->error_code, mcu_to_gcs_command_error->error_action);
}

/**
 * @brief Encode a mcu_to_gcs_command_error struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param mcu_to_gcs_command_error C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mcu_to_gcs_command_error_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_mcu_to_gcs_command_error_t* mcu_to_gcs_command_error)
{
    return mavlink_msg_mcu_to_gcs_command_error_pack_chan(system_id, component_id, chan, msg, mcu_to_gcs_command_error->error_code, mcu_to_gcs_command_error->error_action);
}

/**
 * @brief Send a mcu_to_gcs_command_error message
 * @param chan MAVLink channel to send the message
 *
 * @param error_code  Command error code 
 * @param error_action  Recommended error action to error code 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_mcu_to_gcs_command_error_send(mavlink_channel_t chan, uint32_t error_code, uint32_t error_action)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MCU_TO_GCS_COMMAND_ERROR_LEN];
    _mav_put_uint32_t(buf, 0, error_code);
    _mav_put_uint32_t(buf, 4, error_action);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MCU_TO_GCS_COMMAND_ERROR, buf, MAVLINK_MSG_ID_MCU_TO_GCS_COMMAND_ERROR_MIN_LEN, MAVLINK_MSG_ID_MCU_TO_GCS_COMMAND_ERROR_LEN, MAVLINK_MSG_ID_MCU_TO_GCS_COMMAND_ERROR_CRC);
#else
    mavlink_mcu_to_gcs_command_error_t packet;
    packet.error_code = error_code;
    packet.error_action = error_action;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MCU_TO_GCS_COMMAND_ERROR, (const char *)&packet, MAVLINK_MSG_ID_MCU_TO_GCS_COMMAND_ERROR_MIN_LEN, MAVLINK_MSG_ID_MCU_TO_GCS_COMMAND_ERROR_LEN, MAVLINK_MSG_ID_MCU_TO_GCS_COMMAND_ERROR_CRC);
#endif
}

/**
 * @brief Send a mcu_to_gcs_command_error message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_mcu_to_gcs_command_error_send_struct(mavlink_channel_t chan, const mavlink_mcu_to_gcs_command_error_t* mcu_to_gcs_command_error)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_mcu_to_gcs_command_error_send(chan, mcu_to_gcs_command_error->error_code, mcu_to_gcs_command_error->error_action);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MCU_TO_GCS_COMMAND_ERROR, (const char *)mcu_to_gcs_command_error, MAVLINK_MSG_ID_MCU_TO_GCS_COMMAND_ERROR_MIN_LEN, MAVLINK_MSG_ID_MCU_TO_GCS_COMMAND_ERROR_LEN, MAVLINK_MSG_ID_MCU_TO_GCS_COMMAND_ERROR_CRC);
#endif
}

#if MAVLINK_MSG_ID_MCU_TO_GCS_COMMAND_ERROR_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_mcu_to_gcs_command_error_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t error_code, uint32_t error_action)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, error_code);
    _mav_put_uint32_t(buf, 4, error_action);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MCU_TO_GCS_COMMAND_ERROR, buf, MAVLINK_MSG_ID_MCU_TO_GCS_COMMAND_ERROR_MIN_LEN, MAVLINK_MSG_ID_MCU_TO_GCS_COMMAND_ERROR_LEN, MAVLINK_MSG_ID_MCU_TO_GCS_COMMAND_ERROR_CRC);
#else
    mavlink_mcu_to_gcs_command_error_t *packet = (mavlink_mcu_to_gcs_command_error_t *)msgbuf;
    packet->error_code = error_code;
    packet->error_action = error_action;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MCU_TO_GCS_COMMAND_ERROR, (const char *)packet, MAVLINK_MSG_ID_MCU_TO_GCS_COMMAND_ERROR_MIN_LEN, MAVLINK_MSG_ID_MCU_TO_GCS_COMMAND_ERROR_LEN, MAVLINK_MSG_ID_MCU_TO_GCS_COMMAND_ERROR_CRC);
#endif
}
#endif

#endif

// MESSAGE MCU_TO_GCS_COMMAND_ERROR UNPACKING


/**
 * @brief Get field error_code from mcu_to_gcs_command_error message
 *
 * @return  Command error code 
 */
static inline uint32_t mavlink_msg_mcu_to_gcs_command_error_get_error_code(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field error_action from mcu_to_gcs_command_error message
 *
 * @return  Recommended error action to error code 
 */
static inline uint32_t mavlink_msg_mcu_to_gcs_command_error_get_error_action(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  4);
}

/**
 * @brief Decode a mcu_to_gcs_command_error message into a struct
 *
 * @param msg The message to decode
 * @param mcu_to_gcs_command_error C-struct to decode the message contents into
 */
static inline void mavlink_msg_mcu_to_gcs_command_error_decode(const mavlink_message_t* msg, mavlink_mcu_to_gcs_command_error_t* mcu_to_gcs_command_error)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mcu_to_gcs_command_error->error_code = mavlink_msg_mcu_to_gcs_command_error_get_error_code(msg);
    mcu_to_gcs_command_error->error_action = mavlink_msg_mcu_to_gcs_command_error_get_error_action(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_MCU_TO_GCS_COMMAND_ERROR_LEN? msg->len : MAVLINK_MSG_ID_MCU_TO_GCS_COMMAND_ERROR_LEN;
        memset(mcu_to_gcs_command_error, 0, MAVLINK_MSG_ID_MCU_TO_GCS_COMMAND_ERROR_LEN);
    memcpy(mcu_to_gcs_command_error, _MAV_PAYLOAD(msg), len);
#endif
}
