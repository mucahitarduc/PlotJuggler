#pragma once
// MESSAGE KGEO_TO_MCU_HEARTBEAT PACKING

#define MAVLINK_MSG_ID_KGEO_TO_MCU_HEARTBEAT 220

MAVPACKED(
typedef struct __mavlink_kgeo_to_mcu_heartbeat_t {
 uint8_t is_alive; /*< Alive status of kgeo*/
}) mavlink_kgeo_to_mcu_heartbeat_t;

#define MAVLINK_MSG_ID_KGEO_TO_MCU_HEARTBEAT_LEN 1
#define MAVLINK_MSG_ID_KGEO_TO_MCU_HEARTBEAT_MIN_LEN 1
#define MAVLINK_MSG_ID_220_LEN 1
#define MAVLINK_MSG_ID_220_MIN_LEN 1

#define MAVLINK_MSG_ID_KGEO_TO_MCU_HEARTBEAT_CRC 76
#define MAVLINK_MSG_ID_220_CRC 76



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_KGEO_TO_MCU_HEARTBEAT { \
    220, \
    "KGEO_TO_MCU_HEARTBEAT", \
    1, \
    {  { "is_alive", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_kgeo_to_mcu_heartbeat_t, is_alive) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_KGEO_TO_MCU_HEARTBEAT { \
    "KGEO_TO_MCU_HEARTBEAT", \
    1, \
    {  { "is_alive", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_kgeo_to_mcu_heartbeat_t, is_alive) }, \
         } \
}
#endif

/**
 * @brief Pack a kgeo_to_mcu_heartbeat message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param is_alive Alive status of kgeo
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_kgeo_to_mcu_heartbeat_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t is_alive)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_KGEO_TO_MCU_HEARTBEAT_LEN];
    _mav_put_uint8_t(buf, 0, is_alive);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_KGEO_TO_MCU_HEARTBEAT_LEN);
#else
    mavlink_kgeo_to_mcu_heartbeat_t packet;
    packet.is_alive = is_alive;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_KGEO_TO_MCU_HEARTBEAT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_KGEO_TO_MCU_HEARTBEAT;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_KGEO_TO_MCU_HEARTBEAT_MIN_LEN, MAVLINK_MSG_ID_KGEO_TO_MCU_HEARTBEAT_LEN, MAVLINK_MSG_ID_KGEO_TO_MCU_HEARTBEAT_CRC);
}

/**
 * @brief Pack a kgeo_to_mcu_heartbeat message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param is_alive Alive status of kgeo
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_kgeo_to_mcu_heartbeat_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t is_alive)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_KGEO_TO_MCU_HEARTBEAT_LEN];
    _mav_put_uint8_t(buf, 0, is_alive);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_KGEO_TO_MCU_HEARTBEAT_LEN);
#else
    mavlink_kgeo_to_mcu_heartbeat_t packet;
    packet.is_alive = is_alive;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_KGEO_TO_MCU_HEARTBEAT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_KGEO_TO_MCU_HEARTBEAT;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_KGEO_TO_MCU_HEARTBEAT_MIN_LEN, MAVLINK_MSG_ID_KGEO_TO_MCU_HEARTBEAT_LEN, MAVLINK_MSG_ID_KGEO_TO_MCU_HEARTBEAT_CRC);
}

/**
 * @brief Encode a kgeo_to_mcu_heartbeat struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param kgeo_to_mcu_heartbeat C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_kgeo_to_mcu_heartbeat_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_kgeo_to_mcu_heartbeat_t* kgeo_to_mcu_heartbeat)
{
    return mavlink_msg_kgeo_to_mcu_heartbeat_pack(system_id, component_id, msg, kgeo_to_mcu_heartbeat->is_alive);
}

/**
 * @brief Encode a kgeo_to_mcu_heartbeat struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param kgeo_to_mcu_heartbeat C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_kgeo_to_mcu_heartbeat_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_kgeo_to_mcu_heartbeat_t* kgeo_to_mcu_heartbeat)
{
    return mavlink_msg_kgeo_to_mcu_heartbeat_pack_chan(system_id, component_id, chan, msg, kgeo_to_mcu_heartbeat->is_alive);
}

/**
 * @brief Send a kgeo_to_mcu_heartbeat message
 * @param chan MAVLink channel to send the message
 *
 * @param is_alive Alive status of kgeo
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_kgeo_to_mcu_heartbeat_send(mavlink_channel_t chan, uint8_t is_alive)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_KGEO_TO_MCU_HEARTBEAT_LEN];
    _mav_put_uint8_t(buf, 0, is_alive);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_KGEO_TO_MCU_HEARTBEAT, buf, MAVLINK_MSG_ID_KGEO_TO_MCU_HEARTBEAT_MIN_LEN, MAVLINK_MSG_ID_KGEO_TO_MCU_HEARTBEAT_LEN, MAVLINK_MSG_ID_KGEO_TO_MCU_HEARTBEAT_CRC);
#else
    mavlink_kgeo_to_mcu_heartbeat_t packet;
    packet.is_alive = is_alive;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_KGEO_TO_MCU_HEARTBEAT, (const char *)&packet, MAVLINK_MSG_ID_KGEO_TO_MCU_HEARTBEAT_MIN_LEN, MAVLINK_MSG_ID_KGEO_TO_MCU_HEARTBEAT_LEN, MAVLINK_MSG_ID_KGEO_TO_MCU_HEARTBEAT_CRC);
#endif
}

/**
 * @brief Send a kgeo_to_mcu_heartbeat message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_kgeo_to_mcu_heartbeat_send_struct(mavlink_channel_t chan, const mavlink_kgeo_to_mcu_heartbeat_t* kgeo_to_mcu_heartbeat)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_kgeo_to_mcu_heartbeat_send(chan, kgeo_to_mcu_heartbeat->is_alive);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_KGEO_TO_MCU_HEARTBEAT, (const char *)kgeo_to_mcu_heartbeat, MAVLINK_MSG_ID_KGEO_TO_MCU_HEARTBEAT_MIN_LEN, MAVLINK_MSG_ID_KGEO_TO_MCU_HEARTBEAT_LEN, MAVLINK_MSG_ID_KGEO_TO_MCU_HEARTBEAT_CRC);
#endif
}

#if MAVLINK_MSG_ID_KGEO_TO_MCU_HEARTBEAT_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_kgeo_to_mcu_heartbeat_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t is_alive)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint8_t(buf, 0, is_alive);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_KGEO_TO_MCU_HEARTBEAT, buf, MAVLINK_MSG_ID_KGEO_TO_MCU_HEARTBEAT_MIN_LEN, MAVLINK_MSG_ID_KGEO_TO_MCU_HEARTBEAT_LEN, MAVLINK_MSG_ID_KGEO_TO_MCU_HEARTBEAT_CRC);
#else
    mavlink_kgeo_to_mcu_heartbeat_t *packet = (mavlink_kgeo_to_mcu_heartbeat_t *)msgbuf;
    packet->is_alive = is_alive;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_KGEO_TO_MCU_HEARTBEAT, (const char *)packet, MAVLINK_MSG_ID_KGEO_TO_MCU_HEARTBEAT_MIN_LEN, MAVLINK_MSG_ID_KGEO_TO_MCU_HEARTBEAT_LEN, MAVLINK_MSG_ID_KGEO_TO_MCU_HEARTBEAT_CRC);
#endif
}
#endif

#endif

// MESSAGE KGEO_TO_MCU_HEARTBEAT UNPACKING


/**
 * @brief Get field is_alive from kgeo_to_mcu_heartbeat message
 *
 * @return Alive status of kgeo
 */
static inline uint8_t mavlink_msg_kgeo_to_mcu_heartbeat_get_is_alive(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Decode a kgeo_to_mcu_heartbeat message into a struct
 *
 * @param msg The message to decode
 * @param kgeo_to_mcu_heartbeat C-struct to decode the message contents into
 */
static inline void mavlink_msg_kgeo_to_mcu_heartbeat_decode(const mavlink_message_t* msg, mavlink_kgeo_to_mcu_heartbeat_t* kgeo_to_mcu_heartbeat)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    kgeo_to_mcu_heartbeat->is_alive = mavlink_msg_kgeo_to_mcu_heartbeat_get_is_alive(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_KGEO_TO_MCU_HEARTBEAT_LEN? msg->len : MAVLINK_MSG_ID_KGEO_TO_MCU_HEARTBEAT_LEN;
        memset(kgeo_to_mcu_heartbeat, 0, MAVLINK_MSG_ID_KGEO_TO_MCU_HEARTBEAT_LEN);
    memcpy(kgeo_to_mcu_heartbeat, _MAV_PAYLOAD(msg), len);
#endif
}
