#pragma once
// MESSAGE SAVE_UPDATE_IMAGE PACKING

#define MAVLINK_MSG_ID_SAVE_UPDATE_IMAGE 208

MAVPACKED(
typedef struct __mavlink_save_update_image_t {
 char image_name[32]; /*<  image file name. should be empty when save requested. */
}) mavlink_save_update_image_t;

#define MAVLINK_MSG_ID_SAVE_UPDATE_IMAGE_LEN 32
#define MAVLINK_MSG_ID_SAVE_UPDATE_IMAGE_MIN_LEN 32
#define MAVLINK_MSG_ID_208_LEN 32
#define MAVLINK_MSG_ID_208_MIN_LEN 32

#define MAVLINK_MSG_ID_SAVE_UPDATE_IMAGE_CRC 79
#define MAVLINK_MSG_ID_208_CRC 79

#define MAVLINK_MSG_SAVE_UPDATE_IMAGE_FIELD_IMAGE_NAME_LEN 32

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_SAVE_UPDATE_IMAGE { \
    208, \
    "SAVE_UPDATE_IMAGE", \
    1, \
    {  { "image_name", NULL, MAVLINK_TYPE_CHAR, 32, 0, offsetof(mavlink_save_update_image_t, image_name) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_SAVE_UPDATE_IMAGE { \
    "SAVE_UPDATE_IMAGE", \
    1, \
    {  { "image_name", NULL, MAVLINK_TYPE_CHAR, 32, 0, offsetof(mavlink_save_update_image_t, image_name) }, \
         } \
}
#endif

/**
 * @brief Pack a save_update_image message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param image_name  image file name. should be empty when save requested. 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_save_update_image_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               const char *image_name)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SAVE_UPDATE_IMAGE_LEN];

    _mav_put_char_array(buf, 0, image_name, 32);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SAVE_UPDATE_IMAGE_LEN);
#else
    mavlink_save_update_image_t packet;

    mav_array_memcpy(packet.image_name, image_name, sizeof(char)*32);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SAVE_UPDATE_IMAGE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SAVE_UPDATE_IMAGE;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SAVE_UPDATE_IMAGE_MIN_LEN, MAVLINK_MSG_ID_SAVE_UPDATE_IMAGE_LEN, MAVLINK_MSG_ID_SAVE_UPDATE_IMAGE_CRC);
}

/**
 * @brief Pack a save_update_image message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param image_name  image file name. should be empty when save requested. 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_save_update_image_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   const char *image_name)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SAVE_UPDATE_IMAGE_LEN];

    _mav_put_char_array(buf, 0, image_name, 32);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SAVE_UPDATE_IMAGE_LEN);
#else
    mavlink_save_update_image_t packet;

    mav_array_memcpy(packet.image_name, image_name, sizeof(char)*32);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SAVE_UPDATE_IMAGE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SAVE_UPDATE_IMAGE;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SAVE_UPDATE_IMAGE_MIN_LEN, MAVLINK_MSG_ID_SAVE_UPDATE_IMAGE_LEN, MAVLINK_MSG_ID_SAVE_UPDATE_IMAGE_CRC);
}

/**
 * @brief Encode a save_update_image struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param save_update_image C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_save_update_image_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_save_update_image_t* save_update_image)
{
    return mavlink_msg_save_update_image_pack(system_id, component_id, msg, save_update_image->image_name);
}

/**
 * @brief Encode a save_update_image struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param save_update_image C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_save_update_image_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_save_update_image_t* save_update_image)
{
    return mavlink_msg_save_update_image_pack_chan(system_id, component_id, chan, msg, save_update_image->image_name);
}

/**
 * @brief Send a save_update_image message
 * @param chan MAVLink channel to send the message
 *
 * @param image_name  image file name. should be empty when save requested. 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_save_update_image_send(mavlink_channel_t chan, const char *image_name)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SAVE_UPDATE_IMAGE_LEN];

    _mav_put_char_array(buf, 0, image_name, 32);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SAVE_UPDATE_IMAGE, buf, MAVLINK_MSG_ID_SAVE_UPDATE_IMAGE_MIN_LEN, MAVLINK_MSG_ID_SAVE_UPDATE_IMAGE_LEN, MAVLINK_MSG_ID_SAVE_UPDATE_IMAGE_CRC);
#else
    mavlink_save_update_image_t packet;

    mav_array_memcpy(packet.image_name, image_name, sizeof(char)*32);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SAVE_UPDATE_IMAGE, (const char *)&packet, MAVLINK_MSG_ID_SAVE_UPDATE_IMAGE_MIN_LEN, MAVLINK_MSG_ID_SAVE_UPDATE_IMAGE_LEN, MAVLINK_MSG_ID_SAVE_UPDATE_IMAGE_CRC);
#endif
}

/**
 * @brief Send a save_update_image message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_save_update_image_send_struct(mavlink_channel_t chan, const mavlink_save_update_image_t* save_update_image)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_save_update_image_send(chan, save_update_image->image_name);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SAVE_UPDATE_IMAGE, (const char *)save_update_image, MAVLINK_MSG_ID_SAVE_UPDATE_IMAGE_MIN_LEN, MAVLINK_MSG_ID_SAVE_UPDATE_IMAGE_LEN, MAVLINK_MSG_ID_SAVE_UPDATE_IMAGE_CRC);
#endif
}

#if MAVLINK_MSG_ID_SAVE_UPDATE_IMAGE_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_save_update_image_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  const char *image_name)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;

    _mav_put_char_array(buf, 0, image_name, 32);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SAVE_UPDATE_IMAGE, buf, MAVLINK_MSG_ID_SAVE_UPDATE_IMAGE_MIN_LEN, MAVLINK_MSG_ID_SAVE_UPDATE_IMAGE_LEN, MAVLINK_MSG_ID_SAVE_UPDATE_IMAGE_CRC);
#else
    mavlink_save_update_image_t *packet = (mavlink_save_update_image_t *)msgbuf;

    mav_array_memcpy(packet->image_name, image_name, sizeof(char)*32);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SAVE_UPDATE_IMAGE, (const char *)packet, MAVLINK_MSG_ID_SAVE_UPDATE_IMAGE_MIN_LEN, MAVLINK_MSG_ID_SAVE_UPDATE_IMAGE_LEN, MAVLINK_MSG_ID_SAVE_UPDATE_IMAGE_CRC);
#endif
}
#endif

#endif

// MESSAGE SAVE_UPDATE_IMAGE UNPACKING


/**
 * @brief Get field image_name from save_update_image message
 *
 * @return  image file name. should be empty when save requested. 
 */
static inline uint16_t mavlink_msg_save_update_image_get_image_name(const mavlink_message_t* msg, char *image_name)
{
    return _MAV_RETURN_char_array(msg, image_name, 32,  0);
}

/**
 * @brief Decode a save_update_image message into a struct
 *
 * @param msg The message to decode
 * @param save_update_image C-struct to decode the message contents into
 */
static inline void mavlink_msg_save_update_image_decode(const mavlink_message_t* msg, mavlink_save_update_image_t* save_update_image)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_save_update_image_get_image_name(msg, save_update_image->image_name);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_SAVE_UPDATE_IMAGE_LEN? msg->len : MAVLINK_MSG_ID_SAVE_UPDATE_IMAGE_LEN;
        memset(save_update_image, 0, MAVLINK_MSG_ID_SAVE_UPDATE_IMAGE_LEN);
    memcpy(save_update_image, _MAV_PAYLOAD(msg), len);
#endif
}
