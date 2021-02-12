#pragma once
// MESSAGE KFLOW_GIMBAL_STATUS PACKING

#define MAVLINK_MSG_ID_KFLOW_GIMBAL_STATUS 216

MAVPACKED(
typedef struct __mavlink_kflow_gimbal_status_t {
 float pitch; /*<  IMU pitch */
 float roll; /*<  IMU roll */
 float yaw; /*<  IMU yaw */
 float gyrox; /*<  gyrox */
 float gyroy; /*<  gyroy */
 float gyroz; /*<  gyroz */
}) mavlink_kflow_gimbal_status_t;

#define MAVLINK_MSG_ID_KFLOW_GIMBAL_STATUS_LEN 24
#define MAVLINK_MSG_ID_KFLOW_GIMBAL_STATUS_MIN_LEN 24
#define MAVLINK_MSG_ID_216_LEN 24
#define MAVLINK_MSG_ID_216_MIN_LEN 24

#define MAVLINK_MSG_ID_KFLOW_GIMBAL_STATUS_CRC 41
#define MAVLINK_MSG_ID_216_CRC 41



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_KFLOW_GIMBAL_STATUS { \
    216, \
    "KFLOW_GIMBAL_STATUS", \
    6, \
    {  { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_kflow_gimbal_status_t, pitch) }, \
         { "roll", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_kflow_gimbal_status_t, roll) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_kflow_gimbal_status_t, yaw) }, \
         { "gyrox", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_kflow_gimbal_status_t, gyrox) }, \
         { "gyroy", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_kflow_gimbal_status_t, gyroy) }, \
         { "gyroz", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_kflow_gimbal_status_t, gyroz) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_KFLOW_GIMBAL_STATUS { \
    "KFLOW_GIMBAL_STATUS", \
    6, \
    {  { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_kflow_gimbal_status_t, pitch) }, \
         { "roll", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_kflow_gimbal_status_t, roll) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_kflow_gimbal_status_t, yaw) }, \
         { "gyrox", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_kflow_gimbal_status_t, gyrox) }, \
         { "gyroy", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_kflow_gimbal_status_t, gyroy) }, \
         { "gyroz", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_kflow_gimbal_status_t, gyroz) }, \
         } \
}
#endif

/**
 * @brief Pack a kflow_gimbal_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param pitch  IMU pitch 
 * @param roll  IMU roll 
 * @param yaw  IMU yaw 
 * @param gyrox  gyrox 
 * @param gyroy  gyroy 
 * @param gyroz  gyroz 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_kflow_gimbal_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float pitch, float roll, float yaw, float gyrox, float gyroy, float gyroz)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_KFLOW_GIMBAL_STATUS_LEN];
    _mav_put_float(buf, 0, pitch);
    _mav_put_float(buf, 4, roll);
    _mav_put_float(buf, 8, yaw);
    _mav_put_float(buf, 12, gyrox);
    _mav_put_float(buf, 16, gyroy);
    _mav_put_float(buf, 20, gyroz);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_KFLOW_GIMBAL_STATUS_LEN);
#else
    mavlink_kflow_gimbal_status_t packet;
    packet.pitch = pitch;
    packet.roll = roll;
    packet.yaw = yaw;
    packet.gyrox = gyrox;
    packet.gyroy = gyroy;
    packet.gyroz = gyroz;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_KFLOW_GIMBAL_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_KFLOW_GIMBAL_STATUS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_KFLOW_GIMBAL_STATUS_MIN_LEN, MAVLINK_MSG_ID_KFLOW_GIMBAL_STATUS_LEN, MAVLINK_MSG_ID_KFLOW_GIMBAL_STATUS_CRC);
}

/**
 * @brief Pack a kflow_gimbal_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param pitch  IMU pitch 
 * @param roll  IMU roll 
 * @param yaw  IMU yaw 
 * @param gyrox  gyrox 
 * @param gyroy  gyroy 
 * @param gyroz  gyroz 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_kflow_gimbal_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float pitch,float roll,float yaw,float gyrox,float gyroy,float gyroz)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_KFLOW_GIMBAL_STATUS_LEN];
    _mav_put_float(buf, 0, pitch);
    _mav_put_float(buf, 4, roll);
    _mav_put_float(buf, 8, yaw);
    _mav_put_float(buf, 12, gyrox);
    _mav_put_float(buf, 16, gyroy);
    _mav_put_float(buf, 20, gyroz);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_KFLOW_GIMBAL_STATUS_LEN);
#else
    mavlink_kflow_gimbal_status_t packet;
    packet.pitch = pitch;
    packet.roll = roll;
    packet.yaw = yaw;
    packet.gyrox = gyrox;
    packet.gyroy = gyroy;
    packet.gyroz = gyroz;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_KFLOW_GIMBAL_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_KFLOW_GIMBAL_STATUS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_KFLOW_GIMBAL_STATUS_MIN_LEN, MAVLINK_MSG_ID_KFLOW_GIMBAL_STATUS_LEN, MAVLINK_MSG_ID_KFLOW_GIMBAL_STATUS_CRC);
}

/**
 * @brief Encode a kflow_gimbal_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param kflow_gimbal_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_kflow_gimbal_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_kflow_gimbal_status_t* kflow_gimbal_status)
{
    return mavlink_msg_kflow_gimbal_status_pack(system_id, component_id, msg, kflow_gimbal_status->pitch, kflow_gimbal_status->roll, kflow_gimbal_status->yaw, kflow_gimbal_status->gyrox, kflow_gimbal_status->gyroy, kflow_gimbal_status->gyroz);
}

/**
 * @brief Encode a kflow_gimbal_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param kflow_gimbal_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_kflow_gimbal_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_kflow_gimbal_status_t* kflow_gimbal_status)
{
    return mavlink_msg_kflow_gimbal_status_pack_chan(system_id, component_id, chan, msg, kflow_gimbal_status->pitch, kflow_gimbal_status->roll, kflow_gimbal_status->yaw, kflow_gimbal_status->gyrox, kflow_gimbal_status->gyroy, kflow_gimbal_status->gyroz);
}

/**
 * @brief Send a kflow_gimbal_status message
 * @param chan MAVLink channel to send the message
 *
 * @param pitch  IMU pitch 
 * @param roll  IMU roll 
 * @param yaw  IMU yaw 
 * @param gyrox  gyrox 
 * @param gyroy  gyroy 
 * @param gyroz  gyroz 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_kflow_gimbal_status_send(mavlink_channel_t chan, float pitch, float roll, float yaw, float gyrox, float gyroy, float gyroz)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_KFLOW_GIMBAL_STATUS_LEN];
    _mav_put_float(buf, 0, pitch);
    _mav_put_float(buf, 4, roll);
    _mav_put_float(buf, 8, yaw);
    _mav_put_float(buf, 12, gyrox);
    _mav_put_float(buf, 16, gyroy);
    _mav_put_float(buf, 20, gyroz);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_KFLOW_GIMBAL_STATUS, buf, MAVLINK_MSG_ID_KFLOW_GIMBAL_STATUS_MIN_LEN, MAVLINK_MSG_ID_KFLOW_GIMBAL_STATUS_LEN, MAVLINK_MSG_ID_KFLOW_GIMBAL_STATUS_CRC);
#else
    mavlink_kflow_gimbal_status_t packet;
    packet.pitch = pitch;
    packet.roll = roll;
    packet.yaw = yaw;
    packet.gyrox = gyrox;
    packet.gyroy = gyroy;
    packet.gyroz = gyroz;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_KFLOW_GIMBAL_STATUS, (const char *)&packet, MAVLINK_MSG_ID_KFLOW_GIMBAL_STATUS_MIN_LEN, MAVLINK_MSG_ID_KFLOW_GIMBAL_STATUS_LEN, MAVLINK_MSG_ID_KFLOW_GIMBAL_STATUS_CRC);
#endif
}

/**
 * @brief Send a kflow_gimbal_status message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_kflow_gimbal_status_send_struct(mavlink_channel_t chan, const mavlink_kflow_gimbal_status_t* kflow_gimbal_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_kflow_gimbal_status_send(chan, kflow_gimbal_status->pitch, kflow_gimbal_status->roll, kflow_gimbal_status->yaw, kflow_gimbal_status->gyrox, kflow_gimbal_status->gyroy, kflow_gimbal_status->gyroz);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_KFLOW_GIMBAL_STATUS, (const char *)kflow_gimbal_status, MAVLINK_MSG_ID_KFLOW_GIMBAL_STATUS_MIN_LEN, MAVLINK_MSG_ID_KFLOW_GIMBAL_STATUS_LEN, MAVLINK_MSG_ID_KFLOW_GIMBAL_STATUS_CRC);
#endif
}

#if MAVLINK_MSG_ID_KFLOW_GIMBAL_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_kflow_gimbal_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float pitch, float roll, float yaw, float gyrox, float gyroy, float gyroz)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, pitch);
    _mav_put_float(buf, 4, roll);
    _mav_put_float(buf, 8, yaw);
    _mav_put_float(buf, 12, gyrox);
    _mav_put_float(buf, 16, gyroy);
    _mav_put_float(buf, 20, gyroz);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_KFLOW_GIMBAL_STATUS, buf, MAVLINK_MSG_ID_KFLOW_GIMBAL_STATUS_MIN_LEN, MAVLINK_MSG_ID_KFLOW_GIMBAL_STATUS_LEN, MAVLINK_MSG_ID_KFLOW_GIMBAL_STATUS_CRC);
#else
    mavlink_kflow_gimbal_status_t *packet = (mavlink_kflow_gimbal_status_t *)msgbuf;
    packet->pitch = pitch;
    packet->roll = roll;
    packet->yaw = yaw;
    packet->gyrox = gyrox;
    packet->gyroy = gyroy;
    packet->gyroz = gyroz;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_KFLOW_GIMBAL_STATUS, (const char *)packet, MAVLINK_MSG_ID_KFLOW_GIMBAL_STATUS_MIN_LEN, MAVLINK_MSG_ID_KFLOW_GIMBAL_STATUS_LEN, MAVLINK_MSG_ID_KFLOW_GIMBAL_STATUS_CRC);
#endif
}
#endif

#endif

// MESSAGE KFLOW_GIMBAL_STATUS UNPACKING


/**
 * @brief Get field pitch from kflow_gimbal_status message
 *
 * @return  IMU pitch 
 */
static inline float mavlink_msg_kflow_gimbal_status_get_pitch(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field roll from kflow_gimbal_status message
 *
 * @return  IMU roll 
 */
static inline float mavlink_msg_kflow_gimbal_status_get_roll(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field yaw from kflow_gimbal_status message
 *
 * @return  IMU yaw 
 */
static inline float mavlink_msg_kflow_gimbal_status_get_yaw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field gyrox from kflow_gimbal_status message
 *
 * @return  gyrox 
 */
static inline float mavlink_msg_kflow_gimbal_status_get_gyrox(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field gyroy from kflow_gimbal_status message
 *
 * @return  gyroy 
 */
static inline float mavlink_msg_kflow_gimbal_status_get_gyroy(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field gyroz from kflow_gimbal_status message
 *
 * @return  gyroz 
 */
static inline float mavlink_msg_kflow_gimbal_status_get_gyroz(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Decode a kflow_gimbal_status message into a struct
 *
 * @param msg The message to decode
 * @param kflow_gimbal_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_kflow_gimbal_status_decode(const mavlink_message_t* msg, mavlink_kflow_gimbal_status_t* kflow_gimbal_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    kflow_gimbal_status->pitch = mavlink_msg_kflow_gimbal_status_get_pitch(msg);
    kflow_gimbal_status->roll = mavlink_msg_kflow_gimbal_status_get_roll(msg);
    kflow_gimbal_status->yaw = mavlink_msg_kflow_gimbal_status_get_yaw(msg);
    kflow_gimbal_status->gyrox = mavlink_msg_kflow_gimbal_status_get_gyrox(msg);
    kflow_gimbal_status->gyroy = mavlink_msg_kflow_gimbal_status_get_gyroy(msg);
    kflow_gimbal_status->gyroz = mavlink_msg_kflow_gimbal_status_get_gyroz(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_KFLOW_GIMBAL_STATUS_LEN? msg->len : MAVLINK_MSG_ID_KFLOW_GIMBAL_STATUS_LEN;
        memset(kflow_gimbal_status, 0, MAVLINK_MSG_ID_KFLOW_GIMBAL_STATUS_LEN);
    memcpy(kflow_gimbal_status, _MAV_PAYLOAD(msg), len);
#endif
}
