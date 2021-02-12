#pragma once
// MESSAGE MCU_READY_TO_GO PACKING

#define MAVLINK_MSG_ID_MCU_READY_TO_GO 219

MAVPACKED(
typedef struct __mavlink_mcu_ready_to_go_t {
 uint8_t mcu_ready; /*<  MCU ready status*/
}) mavlink_mcu_ready_to_go_t;

#define MAVLINK_MSG_ID_MCU_READY_TO_GO_LEN 1
#define MAVLINK_MSG_ID_MCU_READY_TO_GO_MIN_LEN 1
#define MAVLINK_MSG_ID_219_LEN 1
#define MAVLINK_MSG_ID_219_MIN_LEN 1

#define MAVLINK_MSG_ID_MCU_READY_TO_GO_CRC 91
#define MAVLINK_MSG_ID_219_CRC 91



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_MCU_READY_TO_GO { \
    219, \
    "MCU_READY_TO_GO", \
    1, \
    {  { "mcu_ready", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_mcu_ready_to_go_t, mcu_ready) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_MCU_READY_TO_GO { \
    "MCU_READY_TO_GO", \
    1, \
    {  { "mcu_ready", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_mcu_ready_to_go_t, mcu_ready) }, \
         } \
}
#endif

/**
 * @brief Pack a mcu_ready_to_go message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param mcu_ready  MCU ready status
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mcu_ready_to_go_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t mcu_ready)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MCU_READY_TO_GO_LEN];
    _mav_put_uint8_t(buf, 0, mcu_ready);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MCU_READY_TO_GO_LEN);
#else
    mavlink_mcu_ready_to_go_t packet;
    packet.mcu_ready = mcu_ready;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MCU_READY_TO_GO_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MCU_READY_TO_GO;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MCU_READY_TO_GO_MIN_LEN, MAVLINK_MSG_ID_MCU_READY_TO_GO_LEN, MAVLINK_MSG_ID_MCU_READY_TO_GO_CRC);
}

/**
 * @brief Pack a mcu_ready_to_go message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param mcu_ready  MCU ready status
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mcu_ready_to_go_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t mcu_ready)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MCU_READY_TO_GO_LEN];
    _mav_put_uint8_t(buf, 0, mcu_ready);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MCU_READY_TO_GO_LEN);
#else
    mavlink_mcu_ready_to_go_t packet;
    packet.mcu_ready = mcu_ready;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MCU_READY_TO_GO_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MCU_READY_TO_GO;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MCU_READY_TO_GO_MIN_LEN, MAVLINK_MSG_ID_MCU_READY_TO_GO_LEN, MAVLINK_MSG_ID_MCU_READY_TO_GO_CRC);
}

/**
 * @brief Encode a mcu_ready_to_go struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param mcu_ready_to_go C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mcu_ready_to_go_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_mcu_ready_to_go_t* mcu_ready_to_go)
{
    return mavlink_msg_mcu_ready_to_go_pack(system_id, component_id, msg, mcu_ready_to_go->mcu_ready);
}

/**
 * @brief Encode a mcu_ready_to_go struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param mcu_ready_to_go C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mcu_ready_to_go_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_mcu_ready_to_go_t* mcu_ready_to_go)
{
    return mavlink_msg_mcu_ready_to_go_pack_chan(system_id, component_id, chan, msg, mcu_ready_to_go->mcu_ready);
}

/**
 * @brief Send a mcu_ready_to_go message
 * @param chan MAVLink channel to send the message
 *
 * @param mcu_ready  MCU ready status
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_mcu_ready_to_go_send(mavlink_channel_t chan, uint8_t mcu_ready)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MCU_READY_TO_GO_LEN];
    _mav_put_uint8_t(buf, 0, mcu_ready);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MCU_READY_TO_GO, buf, MAVLINK_MSG_ID_MCU_READY_TO_GO_MIN_LEN, MAVLINK_MSG_ID_MCU_READY_TO_GO_LEN, MAVLINK_MSG_ID_MCU_READY_TO_GO_CRC);
#else
    mavlink_mcu_ready_to_go_t packet;
    packet.mcu_ready = mcu_ready;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MCU_READY_TO_GO, (const char *)&packet, MAVLINK_MSG_ID_MCU_READY_TO_GO_MIN_LEN, MAVLINK_MSG_ID_MCU_READY_TO_GO_LEN, MAVLINK_MSG_ID_MCU_READY_TO_GO_CRC);
#endif
}

/**
 * @brief Send a mcu_ready_to_go message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_mcu_ready_to_go_send_struct(mavlink_channel_t chan, const mavlink_mcu_ready_to_go_t* mcu_ready_to_go)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_mcu_ready_to_go_send(chan, mcu_ready_to_go->mcu_ready);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MCU_READY_TO_GO, (const char *)mcu_ready_to_go, MAVLINK_MSG_ID_MCU_READY_TO_GO_MIN_LEN, MAVLINK_MSG_ID_MCU_READY_TO_GO_LEN, MAVLINK_MSG_ID_MCU_READY_TO_GO_CRC);
#endif
}

#if MAVLINK_MSG_ID_MCU_READY_TO_GO_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_mcu_ready_to_go_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t mcu_ready)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint8_t(buf, 0, mcu_ready);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MCU_READY_TO_GO, buf, MAVLINK_MSG_ID_MCU_READY_TO_GO_MIN_LEN, MAVLINK_MSG_ID_MCU_READY_TO_GO_LEN, MAVLINK_MSG_ID_MCU_READY_TO_GO_CRC);
#else
    mavlink_mcu_ready_to_go_t *packet = (mavlink_mcu_ready_to_go_t *)msgbuf;
    packet->mcu_ready = mcu_ready;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MCU_READY_TO_GO, (const char *)packet, MAVLINK_MSG_ID_MCU_READY_TO_GO_MIN_LEN, MAVLINK_MSG_ID_MCU_READY_TO_GO_LEN, MAVLINK_MSG_ID_MCU_READY_TO_GO_CRC);
#endif
}
#endif

#endif

// MESSAGE MCU_READY_TO_GO UNPACKING


/**
 * @brief Get field mcu_ready from mcu_ready_to_go message
 *
 * @return  MCU ready status
 */
static inline uint8_t mavlink_msg_mcu_ready_to_go_get_mcu_ready(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Decode a mcu_ready_to_go message into a struct
 *
 * @param msg The message to decode
 * @param mcu_ready_to_go C-struct to decode the message contents into
 */
static inline void mavlink_msg_mcu_ready_to_go_decode(const mavlink_message_t* msg, mavlink_mcu_ready_to_go_t* mcu_ready_to_go)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mcu_ready_to_go->mcu_ready = mavlink_msg_mcu_ready_to_go_get_mcu_ready(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_MCU_READY_TO_GO_LEN? msg->len : MAVLINK_MSG_ID_MCU_READY_TO_GO_LEN;
        memset(mcu_ready_to_go, 0, MAVLINK_MSG_ID_MCU_READY_TO_GO_LEN);
    memcpy(mcu_ready_to_go, _MAV_PAYLOAD(msg), len);
#endif
}
