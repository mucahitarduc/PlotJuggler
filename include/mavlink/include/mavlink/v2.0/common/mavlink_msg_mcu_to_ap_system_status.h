#pragma once
// MESSAGE MCU_TO_AP_SYSTEM_STATUS PACKING

#define MAVLINK_MSG_ID_MCU_TO_AP_SYSTEM_STATUS 202

MAVPACKED(
typedef struct __mavlink_mcu_to_ap_system_status_t {
 uint32_t system_status; /*<  enum MCU_SYSTEM_STATUS */
}) mavlink_mcu_to_ap_system_status_t;

#define MAVLINK_MSG_ID_MCU_TO_AP_SYSTEM_STATUS_LEN 4
#define MAVLINK_MSG_ID_MCU_TO_AP_SYSTEM_STATUS_MIN_LEN 4
#define MAVLINK_MSG_ID_202_LEN 4
#define MAVLINK_MSG_ID_202_MIN_LEN 4

#define MAVLINK_MSG_ID_MCU_TO_AP_SYSTEM_STATUS_CRC 241
#define MAVLINK_MSG_ID_202_CRC 241



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_MCU_TO_AP_SYSTEM_STATUS { \
    202, \
    "MCU_TO_AP_SYSTEM_STATUS", \
    1, \
    {  { "system_status", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_mcu_to_ap_system_status_t, system_status) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_MCU_TO_AP_SYSTEM_STATUS { \
    "MCU_TO_AP_SYSTEM_STATUS", \
    1, \
    {  { "system_status", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_mcu_to_ap_system_status_t, system_status) }, \
         } \
}
#endif

/**
 * @brief Pack a mcu_to_ap_system_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param system_status  enum MCU_SYSTEM_STATUS 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mcu_to_ap_system_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t system_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MCU_TO_AP_SYSTEM_STATUS_LEN];
    _mav_put_uint32_t(buf, 0, system_status);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MCU_TO_AP_SYSTEM_STATUS_LEN);
#else
    mavlink_mcu_to_ap_system_status_t packet;
    packet.system_status = system_status;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MCU_TO_AP_SYSTEM_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MCU_TO_AP_SYSTEM_STATUS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MCU_TO_AP_SYSTEM_STATUS_MIN_LEN, MAVLINK_MSG_ID_MCU_TO_AP_SYSTEM_STATUS_LEN, MAVLINK_MSG_ID_MCU_TO_AP_SYSTEM_STATUS_CRC);
}

/**
 * @brief Pack a mcu_to_ap_system_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param system_status  enum MCU_SYSTEM_STATUS 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mcu_to_ap_system_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t system_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MCU_TO_AP_SYSTEM_STATUS_LEN];
    _mav_put_uint32_t(buf, 0, system_status);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MCU_TO_AP_SYSTEM_STATUS_LEN);
#else
    mavlink_mcu_to_ap_system_status_t packet;
    packet.system_status = system_status;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MCU_TO_AP_SYSTEM_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MCU_TO_AP_SYSTEM_STATUS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MCU_TO_AP_SYSTEM_STATUS_MIN_LEN, MAVLINK_MSG_ID_MCU_TO_AP_SYSTEM_STATUS_LEN, MAVLINK_MSG_ID_MCU_TO_AP_SYSTEM_STATUS_CRC);
}

/**
 * @brief Encode a mcu_to_ap_system_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param mcu_to_ap_system_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mcu_to_ap_system_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_mcu_to_ap_system_status_t* mcu_to_ap_system_status)
{
    return mavlink_msg_mcu_to_ap_system_status_pack(system_id, component_id, msg, mcu_to_ap_system_status->system_status);
}

/**
 * @brief Encode a mcu_to_ap_system_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param mcu_to_ap_system_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mcu_to_ap_system_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_mcu_to_ap_system_status_t* mcu_to_ap_system_status)
{
    return mavlink_msg_mcu_to_ap_system_status_pack_chan(system_id, component_id, chan, msg, mcu_to_ap_system_status->system_status);
}

/**
 * @brief Send a mcu_to_ap_system_status message
 * @param chan MAVLink channel to send the message
 *
 * @param system_status  enum MCU_SYSTEM_STATUS 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_mcu_to_ap_system_status_send(mavlink_channel_t chan, uint32_t system_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MCU_TO_AP_SYSTEM_STATUS_LEN];
    _mav_put_uint32_t(buf, 0, system_status);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MCU_TO_AP_SYSTEM_STATUS, buf, MAVLINK_MSG_ID_MCU_TO_AP_SYSTEM_STATUS_MIN_LEN, MAVLINK_MSG_ID_MCU_TO_AP_SYSTEM_STATUS_LEN, MAVLINK_MSG_ID_MCU_TO_AP_SYSTEM_STATUS_CRC);
#else
    mavlink_mcu_to_ap_system_status_t packet;
    packet.system_status = system_status;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MCU_TO_AP_SYSTEM_STATUS, (const char *)&packet, MAVLINK_MSG_ID_MCU_TO_AP_SYSTEM_STATUS_MIN_LEN, MAVLINK_MSG_ID_MCU_TO_AP_SYSTEM_STATUS_LEN, MAVLINK_MSG_ID_MCU_TO_AP_SYSTEM_STATUS_CRC);
#endif
}

/**
 * @brief Send a mcu_to_ap_system_status message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_mcu_to_ap_system_status_send_struct(mavlink_channel_t chan, const mavlink_mcu_to_ap_system_status_t* mcu_to_ap_system_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_mcu_to_ap_system_status_send(chan, mcu_to_ap_system_status->system_status);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MCU_TO_AP_SYSTEM_STATUS, (const char *)mcu_to_ap_system_status, MAVLINK_MSG_ID_MCU_TO_AP_SYSTEM_STATUS_MIN_LEN, MAVLINK_MSG_ID_MCU_TO_AP_SYSTEM_STATUS_LEN, MAVLINK_MSG_ID_MCU_TO_AP_SYSTEM_STATUS_CRC);
#endif
}

#if MAVLINK_MSG_ID_MCU_TO_AP_SYSTEM_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_mcu_to_ap_system_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t system_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, system_status);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MCU_TO_AP_SYSTEM_STATUS, buf, MAVLINK_MSG_ID_MCU_TO_AP_SYSTEM_STATUS_MIN_LEN, MAVLINK_MSG_ID_MCU_TO_AP_SYSTEM_STATUS_LEN, MAVLINK_MSG_ID_MCU_TO_AP_SYSTEM_STATUS_CRC);
#else
    mavlink_mcu_to_ap_system_status_t *packet = (mavlink_mcu_to_ap_system_status_t *)msgbuf;
    packet->system_status = system_status;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MCU_TO_AP_SYSTEM_STATUS, (const char *)packet, MAVLINK_MSG_ID_MCU_TO_AP_SYSTEM_STATUS_MIN_LEN, MAVLINK_MSG_ID_MCU_TO_AP_SYSTEM_STATUS_LEN, MAVLINK_MSG_ID_MCU_TO_AP_SYSTEM_STATUS_CRC);
#endif
}
#endif

#endif

// MESSAGE MCU_TO_AP_SYSTEM_STATUS UNPACKING


/**
 * @brief Get field system_status from mcu_to_ap_system_status message
 *
 * @return  enum MCU_SYSTEM_STATUS 
 */
static inline uint32_t mavlink_msg_mcu_to_ap_system_status_get_system_status(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Decode a mcu_to_ap_system_status message into a struct
 *
 * @param msg The message to decode
 * @param mcu_to_ap_system_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_mcu_to_ap_system_status_decode(const mavlink_message_t* msg, mavlink_mcu_to_ap_system_status_t* mcu_to_ap_system_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mcu_to_ap_system_status->system_status = mavlink_msg_mcu_to_ap_system_status_get_system_status(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_MCU_TO_AP_SYSTEM_STATUS_LEN? msg->len : MAVLINK_MSG_ID_MCU_TO_AP_SYSTEM_STATUS_LEN;
        memset(mcu_to_ap_system_status, 0, MAVLINK_MSG_ID_MCU_TO_AP_SYSTEM_STATUS_LEN);
    memcpy(mcu_to_ap_system_status, _MAV_PAYLOAD(msg), len);
#endif
}
