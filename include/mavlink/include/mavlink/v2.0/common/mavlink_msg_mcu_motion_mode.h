#pragma once
// MESSAGE MCU_MOTION_MODE PACKING

#define MAVLINK_MSG_ID_MCU_MOTION_MODE 215

MAVPACKED(
typedef struct __mavlink_mcu_motion_mode_t {
 uint32_t motion_mode; /*<  enum field to represent mcu motion mode */
}) mavlink_mcu_motion_mode_t;

#define MAVLINK_MSG_ID_MCU_MOTION_MODE_LEN 4
#define MAVLINK_MSG_ID_MCU_MOTION_MODE_MIN_LEN 4
#define MAVLINK_MSG_ID_215_LEN 4
#define MAVLINK_MSG_ID_215_MIN_LEN 4

#define MAVLINK_MSG_ID_MCU_MOTION_MODE_CRC 182
#define MAVLINK_MSG_ID_215_CRC 182



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_MCU_MOTION_MODE { \
    215, \
    "MCU_MOTION_MODE", \
    1, \
    {  { "motion_mode", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_mcu_motion_mode_t, motion_mode) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_MCU_MOTION_MODE { \
    "MCU_MOTION_MODE", \
    1, \
    {  { "motion_mode", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_mcu_motion_mode_t, motion_mode) }, \
         } \
}
#endif

/**
 * @brief Pack a mcu_motion_mode message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param motion_mode  enum field to represent mcu motion mode 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mcu_motion_mode_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t motion_mode)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MCU_MOTION_MODE_LEN];
    _mav_put_uint32_t(buf, 0, motion_mode);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MCU_MOTION_MODE_LEN);
#else
    mavlink_mcu_motion_mode_t packet;
    packet.motion_mode = motion_mode;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MCU_MOTION_MODE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MCU_MOTION_MODE;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MCU_MOTION_MODE_MIN_LEN, MAVLINK_MSG_ID_MCU_MOTION_MODE_LEN, MAVLINK_MSG_ID_MCU_MOTION_MODE_CRC);
}

/**
 * @brief Pack a mcu_motion_mode message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param motion_mode  enum field to represent mcu motion mode 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mcu_motion_mode_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t motion_mode)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MCU_MOTION_MODE_LEN];
    _mav_put_uint32_t(buf, 0, motion_mode);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MCU_MOTION_MODE_LEN);
#else
    mavlink_mcu_motion_mode_t packet;
    packet.motion_mode = motion_mode;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MCU_MOTION_MODE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MCU_MOTION_MODE;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MCU_MOTION_MODE_MIN_LEN, MAVLINK_MSG_ID_MCU_MOTION_MODE_LEN, MAVLINK_MSG_ID_MCU_MOTION_MODE_CRC);
}

/**
 * @brief Encode a mcu_motion_mode struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param mcu_motion_mode C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mcu_motion_mode_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_mcu_motion_mode_t* mcu_motion_mode)
{
    return mavlink_msg_mcu_motion_mode_pack(system_id, component_id, msg, mcu_motion_mode->motion_mode);
}

/**
 * @brief Encode a mcu_motion_mode struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param mcu_motion_mode C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mcu_motion_mode_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_mcu_motion_mode_t* mcu_motion_mode)
{
    return mavlink_msg_mcu_motion_mode_pack_chan(system_id, component_id, chan, msg, mcu_motion_mode->motion_mode);
}

/**
 * @brief Send a mcu_motion_mode message
 * @param chan MAVLink channel to send the message
 *
 * @param motion_mode  enum field to represent mcu motion mode 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_mcu_motion_mode_send(mavlink_channel_t chan, uint32_t motion_mode)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MCU_MOTION_MODE_LEN];
    _mav_put_uint32_t(buf, 0, motion_mode);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MCU_MOTION_MODE, buf, MAVLINK_MSG_ID_MCU_MOTION_MODE_MIN_LEN, MAVLINK_MSG_ID_MCU_MOTION_MODE_LEN, MAVLINK_MSG_ID_MCU_MOTION_MODE_CRC);
#else
    mavlink_mcu_motion_mode_t packet;
    packet.motion_mode = motion_mode;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MCU_MOTION_MODE, (const char *)&packet, MAVLINK_MSG_ID_MCU_MOTION_MODE_MIN_LEN, MAVLINK_MSG_ID_MCU_MOTION_MODE_LEN, MAVLINK_MSG_ID_MCU_MOTION_MODE_CRC);
#endif
}

/**
 * @brief Send a mcu_motion_mode message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_mcu_motion_mode_send_struct(mavlink_channel_t chan, const mavlink_mcu_motion_mode_t* mcu_motion_mode)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_mcu_motion_mode_send(chan, mcu_motion_mode->motion_mode);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MCU_MOTION_MODE, (const char *)mcu_motion_mode, MAVLINK_MSG_ID_MCU_MOTION_MODE_MIN_LEN, MAVLINK_MSG_ID_MCU_MOTION_MODE_LEN, MAVLINK_MSG_ID_MCU_MOTION_MODE_CRC);
#endif
}

#if MAVLINK_MSG_ID_MCU_MOTION_MODE_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_mcu_motion_mode_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t motion_mode)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, motion_mode);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MCU_MOTION_MODE, buf, MAVLINK_MSG_ID_MCU_MOTION_MODE_MIN_LEN, MAVLINK_MSG_ID_MCU_MOTION_MODE_LEN, MAVLINK_MSG_ID_MCU_MOTION_MODE_CRC);
#else
    mavlink_mcu_motion_mode_t *packet = (mavlink_mcu_motion_mode_t *)msgbuf;
    packet->motion_mode = motion_mode;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MCU_MOTION_MODE, (const char *)packet, MAVLINK_MSG_ID_MCU_MOTION_MODE_MIN_LEN, MAVLINK_MSG_ID_MCU_MOTION_MODE_LEN, MAVLINK_MSG_ID_MCU_MOTION_MODE_CRC);
#endif
}
#endif

#endif

// MESSAGE MCU_MOTION_MODE UNPACKING


/**
 * @brief Get field motion_mode from mcu_motion_mode message
 *
 * @return  enum field to represent mcu motion mode 
 */
static inline uint32_t mavlink_msg_mcu_motion_mode_get_motion_mode(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Decode a mcu_motion_mode message into a struct
 *
 * @param msg The message to decode
 * @param mcu_motion_mode C-struct to decode the message contents into
 */
static inline void mavlink_msg_mcu_motion_mode_decode(const mavlink_message_t* msg, mavlink_mcu_motion_mode_t* mcu_motion_mode)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mcu_motion_mode->motion_mode = mavlink_msg_mcu_motion_mode_get_motion_mode(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_MCU_MOTION_MODE_LEN? msg->len : MAVLINK_MSG_ID_MCU_MOTION_MODE_LEN;
        memset(mcu_motion_mode, 0, MAVLINK_MSG_ID_MCU_MOTION_MODE_LEN);
    memcpy(mcu_motion_mode, _MAV_PAYLOAD(msg), len);
#endif
}
