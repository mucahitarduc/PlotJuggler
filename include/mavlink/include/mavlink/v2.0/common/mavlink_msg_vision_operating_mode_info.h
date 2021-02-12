#pragma once
// MESSAGE VISION_OPERATING_MODE_INFO PACKING

#define MAVLINK_MSG_ID_VISION_OPERATING_MODE_INFO 191

MAVPACKED(
typedef struct __mavlink_vision_operating_mode_info_t {
 uint32_t operating_mode; /*<  enum VISION_OPERATING_MODE_TYPE */
}) mavlink_vision_operating_mode_info_t;

#define MAVLINK_MSG_ID_VISION_OPERATING_MODE_INFO_LEN 4
#define MAVLINK_MSG_ID_VISION_OPERATING_MODE_INFO_MIN_LEN 4
#define MAVLINK_MSG_ID_191_LEN 4
#define MAVLINK_MSG_ID_191_MIN_LEN 4

#define MAVLINK_MSG_ID_VISION_OPERATING_MODE_INFO_CRC 213
#define MAVLINK_MSG_ID_191_CRC 213



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_VISION_OPERATING_MODE_INFO { \
    191, \
    "VISION_OPERATING_MODE_INFO", \
    1, \
    {  { "operating_mode", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_vision_operating_mode_info_t, operating_mode) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_VISION_OPERATING_MODE_INFO { \
    "VISION_OPERATING_MODE_INFO", \
    1, \
    {  { "operating_mode", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_vision_operating_mode_info_t, operating_mode) }, \
         } \
}
#endif

/**
 * @brief Pack a vision_operating_mode_info message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param operating_mode  enum VISION_OPERATING_MODE_TYPE 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_vision_operating_mode_info_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t operating_mode)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_VISION_OPERATING_MODE_INFO_LEN];
    _mav_put_uint32_t(buf, 0, operating_mode);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_VISION_OPERATING_MODE_INFO_LEN);
#else
    mavlink_vision_operating_mode_info_t packet;
    packet.operating_mode = operating_mode;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_VISION_OPERATING_MODE_INFO_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_VISION_OPERATING_MODE_INFO;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_VISION_OPERATING_MODE_INFO_MIN_LEN, MAVLINK_MSG_ID_VISION_OPERATING_MODE_INFO_LEN, MAVLINK_MSG_ID_VISION_OPERATING_MODE_INFO_CRC);
}

/**
 * @brief Pack a vision_operating_mode_info message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param operating_mode  enum VISION_OPERATING_MODE_TYPE 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_vision_operating_mode_info_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t operating_mode)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_VISION_OPERATING_MODE_INFO_LEN];
    _mav_put_uint32_t(buf, 0, operating_mode);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_VISION_OPERATING_MODE_INFO_LEN);
#else
    mavlink_vision_operating_mode_info_t packet;
    packet.operating_mode = operating_mode;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_VISION_OPERATING_MODE_INFO_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_VISION_OPERATING_MODE_INFO;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_VISION_OPERATING_MODE_INFO_MIN_LEN, MAVLINK_MSG_ID_VISION_OPERATING_MODE_INFO_LEN, MAVLINK_MSG_ID_VISION_OPERATING_MODE_INFO_CRC);
}

/**
 * @brief Encode a vision_operating_mode_info struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param vision_operating_mode_info C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_vision_operating_mode_info_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_vision_operating_mode_info_t* vision_operating_mode_info)
{
    return mavlink_msg_vision_operating_mode_info_pack(system_id, component_id, msg, vision_operating_mode_info->operating_mode);
}

/**
 * @brief Encode a vision_operating_mode_info struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param vision_operating_mode_info C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_vision_operating_mode_info_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_vision_operating_mode_info_t* vision_operating_mode_info)
{
    return mavlink_msg_vision_operating_mode_info_pack_chan(system_id, component_id, chan, msg, vision_operating_mode_info->operating_mode);
}

/**
 * @brief Send a vision_operating_mode_info message
 * @param chan MAVLink channel to send the message
 *
 * @param operating_mode  enum VISION_OPERATING_MODE_TYPE 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_vision_operating_mode_info_send(mavlink_channel_t chan, uint32_t operating_mode)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_VISION_OPERATING_MODE_INFO_LEN];
    _mav_put_uint32_t(buf, 0, operating_mode);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VISION_OPERATING_MODE_INFO, buf, MAVLINK_MSG_ID_VISION_OPERATING_MODE_INFO_MIN_LEN, MAVLINK_MSG_ID_VISION_OPERATING_MODE_INFO_LEN, MAVLINK_MSG_ID_VISION_OPERATING_MODE_INFO_CRC);
#else
    mavlink_vision_operating_mode_info_t packet;
    packet.operating_mode = operating_mode;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VISION_OPERATING_MODE_INFO, (const char *)&packet, MAVLINK_MSG_ID_VISION_OPERATING_MODE_INFO_MIN_LEN, MAVLINK_MSG_ID_VISION_OPERATING_MODE_INFO_LEN, MAVLINK_MSG_ID_VISION_OPERATING_MODE_INFO_CRC);
#endif
}

/**
 * @brief Send a vision_operating_mode_info message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_vision_operating_mode_info_send_struct(mavlink_channel_t chan, const mavlink_vision_operating_mode_info_t* vision_operating_mode_info)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_vision_operating_mode_info_send(chan, vision_operating_mode_info->operating_mode);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VISION_OPERATING_MODE_INFO, (const char *)vision_operating_mode_info, MAVLINK_MSG_ID_VISION_OPERATING_MODE_INFO_MIN_LEN, MAVLINK_MSG_ID_VISION_OPERATING_MODE_INFO_LEN, MAVLINK_MSG_ID_VISION_OPERATING_MODE_INFO_CRC);
#endif
}

#if MAVLINK_MSG_ID_VISION_OPERATING_MODE_INFO_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_vision_operating_mode_info_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t operating_mode)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, operating_mode);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VISION_OPERATING_MODE_INFO, buf, MAVLINK_MSG_ID_VISION_OPERATING_MODE_INFO_MIN_LEN, MAVLINK_MSG_ID_VISION_OPERATING_MODE_INFO_LEN, MAVLINK_MSG_ID_VISION_OPERATING_MODE_INFO_CRC);
#else
    mavlink_vision_operating_mode_info_t *packet = (mavlink_vision_operating_mode_info_t *)msgbuf;
    packet->operating_mode = operating_mode;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VISION_OPERATING_MODE_INFO, (const char *)packet, MAVLINK_MSG_ID_VISION_OPERATING_MODE_INFO_MIN_LEN, MAVLINK_MSG_ID_VISION_OPERATING_MODE_INFO_LEN, MAVLINK_MSG_ID_VISION_OPERATING_MODE_INFO_CRC);
#endif
}
#endif

#endif

// MESSAGE VISION_OPERATING_MODE_INFO UNPACKING


/**
 * @brief Get field operating_mode from vision_operating_mode_info message
 *
 * @return  enum VISION_OPERATING_MODE_TYPE 
 */
static inline uint32_t mavlink_msg_vision_operating_mode_info_get_operating_mode(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Decode a vision_operating_mode_info message into a struct
 *
 * @param msg The message to decode
 * @param vision_operating_mode_info C-struct to decode the message contents into
 */
static inline void mavlink_msg_vision_operating_mode_info_decode(const mavlink_message_t* msg, mavlink_vision_operating_mode_info_t* vision_operating_mode_info)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    vision_operating_mode_info->operating_mode = mavlink_msg_vision_operating_mode_info_get_operating_mode(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_VISION_OPERATING_MODE_INFO_LEN? msg->len : MAVLINK_MSG_ID_VISION_OPERATING_MODE_INFO_LEN;
        memset(vision_operating_mode_info, 0, MAVLINK_MSG_ID_VISION_OPERATING_MODE_INFO_LEN);
    memcpy(vision_operating_mode_info, _MAV_PAYLOAD(msg), len);
#endif
}
