#pragma once
// MESSAGE KFLOW_HORIZONTAL_MOTION PACKING

#define MAVLINK_MSG_ID_KFLOW_HORIZONTAL_MOTION 214

MAVPACKED(
typedef struct __mavlink_kflow_horizontal_motion_t {
 float ox; /*<  Offset in x direction to the last reference [-0.5, 0.5] */
 float oy; /*<  Offset in y direction to the last reference [-0.5, 0.5] */
}) mavlink_kflow_horizontal_motion_t;

#define MAVLINK_MSG_ID_KFLOW_HORIZONTAL_MOTION_LEN 8
#define MAVLINK_MSG_ID_KFLOW_HORIZONTAL_MOTION_MIN_LEN 8
#define MAVLINK_MSG_ID_214_LEN 8
#define MAVLINK_MSG_ID_214_MIN_LEN 8

#define MAVLINK_MSG_ID_KFLOW_HORIZONTAL_MOTION_CRC 111
#define MAVLINK_MSG_ID_214_CRC 111



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_KFLOW_HORIZONTAL_MOTION { \
    214, \
    "KFLOW_HORIZONTAL_MOTION", \
    2, \
    {  { "ox", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_kflow_horizontal_motion_t, ox) }, \
         { "oy", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_kflow_horizontal_motion_t, oy) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_KFLOW_HORIZONTAL_MOTION { \
    "KFLOW_HORIZONTAL_MOTION", \
    2, \
    {  { "ox", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_kflow_horizontal_motion_t, ox) }, \
         { "oy", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_kflow_horizontal_motion_t, oy) }, \
         } \
}
#endif

/**
 * @brief Pack a kflow_horizontal_motion message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param ox  Offset in x direction to the last reference [-0.5, 0.5] 
 * @param oy  Offset in y direction to the last reference [-0.5, 0.5] 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_kflow_horizontal_motion_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float ox, float oy)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_KFLOW_HORIZONTAL_MOTION_LEN];
    _mav_put_float(buf, 0, ox);
    _mav_put_float(buf, 4, oy);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_KFLOW_HORIZONTAL_MOTION_LEN);
#else
    mavlink_kflow_horizontal_motion_t packet;
    packet.ox = ox;
    packet.oy = oy;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_KFLOW_HORIZONTAL_MOTION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_KFLOW_HORIZONTAL_MOTION;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_KFLOW_HORIZONTAL_MOTION_MIN_LEN, MAVLINK_MSG_ID_KFLOW_HORIZONTAL_MOTION_LEN, MAVLINK_MSG_ID_KFLOW_HORIZONTAL_MOTION_CRC);
}

/**
 * @brief Pack a kflow_horizontal_motion message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param ox  Offset in x direction to the last reference [-0.5, 0.5] 
 * @param oy  Offset in y direction to the last reference [-0.5, 0.5] 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_kflow_horizontal_motion_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float ox,float oy)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_KFLOW_HORIZONTAL_MOTION_LEN];
    _mav_put_float(buf, 0, ox);
    _mav_put_float(buf, 4, oy);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_KFLOW_HORIZONTAL_MOTION_LEN);
#else
    mavlink_kflow_horizontal_motion_t packet;
    packet.ox = ox;
    packet.oy = oy;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_KFLOW_HORIZONTAL_MOTION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_KFLOW_HORIZONTAL_MOTION;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_KFLOW_HORIZONTAL_MOTION_MIN_LEN, MAVLINK_MSG_ID_KFLOW_HORIZONTAL_MOTION_LEN, MAVLINK_MSG_ID_KFLOW_HORIZONTAL_MOTION_CRC);
}

/**
 * @brief Encode a kflow_horizontal_motion struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param kflow_horizontal_motion C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_kflow_horizontal_motion_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_kflow_horizontal_motion_t* kflow_horizontal_motion)
{
    return mavlink_msg_kflow_horizontal_motion_pack(system_id, component_id, msg, kflow_horizontal_motion->ox, kflow_horizontal_motion->oy);
}

/**
 * @brief Encode a kflow_horizontal_motion struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param kflow_horizontal_motion C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_kflow_horizontal_motion_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_kflow_horizontal_motion_t* kflow_horizontal_motion)
{
    return mavlink_msg_kflow_horizontal_motion_pack_chan(system_id, component_id, chan, msg, kflow_horizontal_motion->ox, kflow_horizontal_motion->oy);
}

/**
 * @brief Send a kflow_horizontal_motion message
 * @param chan MAVLink channel to send the message
 *
 * @param ox  Offset in x direction to the last reference [-0.5, 0.5] 
 * @param oy  Offset in y direction to the last reference [-0.5, 0.5] 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_kflow_horizontal_motion_send(mavlink_channel_t chan, float ox, float oy)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_KFLOW_HORIZONTAL_MOTION_LEN];
    _mav_put_float(buf, 0, ox);
    _mav_put_float(buf, 4, oy);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_KFLOW_HORIZONTAL_MOTION, buf, MAVLINK_MSG_ID_KFLOW_HORIZONTAL_MOTION_MIN_LEN, MAVLINK_MSG_ID_KFLOW_HORIZONTAL_MOTION_LEN, MAVLINK_MSG_ID_KFLOW_HORIZONTAL_MOTION_CRC);
#else
    mavlink_kflow_horizontal_motion_t packet;
    packet.ox = ox;
    packet.oy = oy;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_KFLOW_HORIZONTAL_MOTION, (const char *)&packet, MAVLINK_MSG_ID_KFLOW_HORIZONTAL_MOTION_MIN_LEN, MAVLINK_MSG_ID_KFLOW_HORIZONTAL_MOTION_LEN, MAVLINK_MSG_ID_KFLOW_HORIZONTAL_MOTION_CRC);
#endif
}

/**
 * @brief Send a kflow_horizontal_motion message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_kflow_horizontal_motion_send_struct(mavlink_channel_t chan, const mavlink_kflow_horizontal_motion_t* kflow_horizontal_motion)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_kflow_horizontal_motion_send(chan, kflow_horizontal_motion->ox, kflow_horizontal_motion->oy);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_KFLOW_HORIZONTAL_MOTION, (const char *)kflow_horizontal_motion, MAVLINK_MSG_ID_KFLOW_HORIZONTAL_MOTION_MIN_LEN, MAVLINK_MSG_ID_KFLOW_HORIZONTAL_MOTION_LEN, MAVLINK_MSG_ID_KFLOW_HORIZONTAL_MOTION_CRC);
#endif
}

#if MAVLINK_MSG_ID_KFLOW_HORIZONTAL_MOTION_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_kflow_horizontal_motion_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float ox, float oy)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, ox);
    _mav_put_float(buf, 4, oy);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_KFLOW_HORIZONTAL_MOTION, buf, MAVLINK_MSG_ID_KFLOW_HORIZONTAL_MOTION_MIN_LEN, MAVLINK_MSG_ID_KFLOW_HORIZONTAL_MOTION_LEN, MAVLINK_MSG_ID_KFLOW_HORIZONTAL_MOTION_CRC);
#else
    mavlink_kflow_horizontal_motion_t *packet = (mavlink_kflow_horizontal_motion_t *)msgbuf;
    packet->ox = ox;
    packet->oy = oy;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_KFLOW_HORIZONTAL_MOTION, (const char *)packet, MAVLINK_MSG_ID_KFLOW_HORIZONTAL_MOTION_MIN_LEN, MAVLINK_MSG_ID_KFLOW_HORIZONTAL_MOTION_LEN, MAVLINK_MSG_ID_KFLOW_HORIZONTAL_MOTION_CRC);
#endif
}
#endif

#endif

// MESSAGE KFLOW_HORIZONTAL_MOTION UNPACKING


/**
 * @brief Get field ox from kflow_horizontal_motion message
 *
 * @return  Offset in x direction to the last reference [-0.5, 0.5] 
 */
static inline float mavlink_msg_kflow_horizontal_motion_get_ox(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field oy from kflow_horizontal_motion message
 *
 * @return  Offset in y direction to the last reference [-0.5, 0.5] 
 */
static inline float mavlink_msg_kflow_horizontal_motion_get_oy(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Decode a kflow_horizontal_motion message into a struct
 *
 * @param msg The message to decode
 * @param kflow_horizontal_motion C-struct to decode the message contents into
 */
static inline void mavlink_msg_kflow_horizontal_motion_decode(const mavlink_message_t* msg, mavlink_kflow_horizontal_motion_t* kflow_horizontal_motion)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    kflow_horizontal_motion->ox = mavlink_msg_kflow_horizontal_motion_get_ox(msg);
    kflow_horizontal_motion->oy = mavlink_msg_kflow_horizontal_motion_get_oy(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_KFLOW_HORIZONTAL_MOTION_LEN? msg->len : MAVLINK_MSG_ID_KFLOW_HORIZONTAL_MOTION_LEN;
        memset(kflow_horizontal_motion, 0, MAVLINK_MSG_ID_KFLOW_HORIZONTAL_MOTION_LEN);
    memcpy(kflow_horizontal_motion, _MAV_PAYLOAD(msg), len);
#endif
}
