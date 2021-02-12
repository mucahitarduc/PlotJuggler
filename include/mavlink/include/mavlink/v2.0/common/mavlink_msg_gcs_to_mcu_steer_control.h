#pragma once
// MESSAGE GCS_TO_MCU_STEER_CONTROL PACKING

#define MAVLINK_MSG_ID_GCS_TO_MCU_STEER_CONTROL 195

MAVPACKED(
typedef struct __mavlink_gcs_to_mcu_steer_control_t {
 float thrust_value; /*<  thrust value in range of [0.0, 1.0]. min thrust(0.0); max thrust(1.0) */
 float pitch_value; /*<  pitch value in range of [-1.0, 1.0]. forward (0, 1.0); backward [-1.0, 0); no pitch = 0 */
 float roll_value; /*<  roll value in range of [-1.0, 1.0]. right (0, 1.0); left [-1.0, 0); no roll = 0 */
 float yaw_value; /*<  yaw value in range of [-1.0, 1.0]. counter-clockwise (0, 1.0); clockwise [-1.0, 0); no yaw = 0 */
}) mavlink_gcs_to_mcu_steer_control_t;

#define MAVLINK_MSG_ID_GCS_TO_MCU_STEER_CONTROL_LEN 16
#define MAVLINK_MSG_ID_GCS_TO_MCU_STEER_CONTROL_MIN_LEN 16
#define MAVLINK_MSG_ID_195_LEN 16
#define MAVLINK_MSG_ID_195_MIN_LEN 16

#define MAVLINK_MSG_ID_GCS_TO_MCU_STEER_CONTROL_CRC 95
#define MAVLINK_MSG_ID_195_CRC 95



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_GCS_TO_MCU_STEER_CONTROL { \
    195, \
    "GCS_TO_MCU_STEER_CONTROL", \
    4, \
    {  { "thrust_value", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_gcs_to_mcu_steer_control_t, thrust_value) }, \
         { "pitch_value", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_gcs_to_mcu_steer_control_t, pitch_value) }, \
         { "roll_value", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_gcs_to_mcu_steer_control_t, roll_value) }, \
         { "yaw_value", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_gcs_to_mcu_steer_control_t, yaw_value) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_GCS_TO_MCU_STEER_CONTROL { \
    "GCS_TO_MCU_STEER_CONTROL", \
    4, \
    {  { "thrust_value", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_gcs_to_mcu_steer_control_t, thrust_value) }, \
         { "pitch_value", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_gcs_to_mcu_steer_control_t, pitch_value) }, \
         { "roll_value", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_gcs_to_mcu_steer_control_t, roll_value) }, \
         { "yaw_value", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_gcs_to_mcu_steer_control_t, yaw_value) }, \
         } \
}
#endif

/**
 * @brief Pack a gcs_to_mcu_steer_control message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param thrust_value  thrust value in range of [0.0, 1.0]. min thrust(0.0); max thrust(1.0) 
 * @param pitch_value  pitch value in range of [-1.0, 1.0]. forward (0, 1.0); backward [-1.0, 0); no pitch = 0 
 * @param roll_value  roll value in range of [-1.0, 1.0]. right (0, 1.0); left [-1.0, 0); no roll = 0 
 * @param yaw_value  yaw value in range of [-1.0, 1.0]. counter-clockwise (0, 1.0); clockwise [-1.0, 0); no yaw = 0 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gcs_to_mcu_steer_control_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float thrust_value, float pitch_value, float roll_value, float yaw_value)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GCS_TO_MCU_STEER_CONTROL_LEN];
    _mav_put_float(buf, 0, thrust_value);
    _mav_put_float(buf, 4, pitch_value);
    _mav_put_float(buf, 8, roll_value);
    _mav_put_float(buf, 12, yaw_value);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GCS_TO_MCU_STEER_CONTROL_LEN);
#else
    mavlink_gcs_to_mcu_steer_control_t packet;
    packet.thrust_value = thrust_value;
    packet.pitch_value = pitch_value;
    packet.roll_value = roll_value;
    packet.yaw_value = yaw_value;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GCS_TO_MCU_STEER_CONTROL_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GCS_TO_MCU_STEER_CONTROL;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_GCS_TO_MCU_STEER_CONTROL_MIN_LEN, MAVLINK_MSG_ID_GCS_TO_MCU_STEER_CONTROL_LEN, MAVLINK_MSG_ID_GCS_TO_MCU_STEER_CONTROL_CRC);
}

/**
 * @brief Pack a gcs_to_mcu_steer_control message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param thrust_value  thrust value in range of [0.0, 1.0]. min thrust(0.0); max thrust(1.0) 
 * @param pitch_value  pitch value in range of [-1.0, 1.0]. forward (0, 1.0); backward [-1.0, 0); no pitch = 0 
 * @param roll_value  roll value in range of [-1.0, 1.0]. right (0, 1.0); left [-1.0, 0); no roll = 0 
 * @param yaw_value  yaw value in range of [-1.0, 1.0]. counter-clockwise (0, 1.0); clockwise [-1.0, 0); no yaw = 0 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gcs_to_mcu_steer_control_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float thrust_value,float pitch_value,float roll_value,float yaw_value)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GCS_TO_MCU_STEER_CONTROL_LEN];
    _mav_put_float(buf, 0, thrust_value);
    _mav_put_float(buf, 4, pitch_value);
    _mav_put_float(buf, 8, roll_value);
    _mav_put_float(buf, 12, yaw_value);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GCS_TO_MCU_STEER_CONTROL_LEN);
#else
    mavlink_gcs_to_mcu_steer_control_t packet;
    packet.thrust_value = thrust_value;
    packet.pitch_value = pitch_value;
    packet.roll_value = roll_value;
    packet.yaw_value = yaw_value;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GCS_TO_MCU_STEER_CONTROL_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GCS_TO_MCU_STEER_CONTROL;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_GCS_TO_MCU_STEER_CONTROL_MIN_LEN, MAVLINK_MSG_ID_GCS_TO_MCU_STEER_CONTROL_LEN, MAVLINK_MSG_ID_GCS_TO_MCU_STEER_CONTROL_CRC);
}

/**
 * @brief Encode a gcs_to_mcu_steer_control struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param gcs_to_mcu_steer_control C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gcs_to_mcu_steer_control_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_gcs_to_mcu_steer_control_t* gcs_to_mcu_steer_control)
{
    return mavlink_msg_gcs_to_mcu_steer_control_pack(system_id, component_id, msg, gcs_to_mcu_steer_control->thrust_value, gcs_to_mcu_steer_control->pitch_value, gcs_to_mcu_steer_control->roll_value, gcs_to_mcu_steer_control->yaw_value);
}

/**
 * @brief Encode a gcs_to_mcu_steer_control struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param gcs_to_mcu_steer_control C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gcs_to_mcu_steer_control_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_gcs_to_mcu_steer_control_t* gcs_to_mcu_steer_control)
{
    return mavlink_msg_gcs_to_mcu_steer_control_pack_chan(system_id, component_id, chan, msg, gcs_to_mcu_steer_control->thrust_value, gcs_to_mcu_steer_control->pitch_value, gcs_to_mcu_steer_control->roll_value, gcs_to_mcu_steer_control->yaw_value);
}

/**
 * @brief Send a gcs_to_mcu_steer_control message
 * @param chan MAVLink channel to send the message
 *
 * @param thrust_value  thrust value in range of [0.0, 1.0]. min thrust(0.0); max thrust(1.0) 
 * @param pitch_value  pitch value in range of [-1.0, 1.0]. forward (0, 1.0); backward [-1.0, 0); no pitch = 0 
 * @param roll_value  roll value in range of [-1.0, 1.0]. right (0, 1.0); left [-1.0, 0); no roll = 0 
 * @param yaw_value  yaw value in range of [-1.0, 1.0]. counter-clockwise (0, 1.0); clockwise [-1.0, 0); no yaw = 0 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_gcs_to_mcu_steer_control_send(mavlink_channel_t chan, float thrust_value, float pitch_value, float roll_value, float yaw_value)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GCS_TO_MCU_STEER_CONTROL_LEN];
    _mav_put_float(buf, 0, thrust_value);
    _mav_put_float(buf, 4, pitch_value);
    _mav_put_float(buf, 8, roll_value);
    _mav_put_float(buf, 12, yaw_value);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GCS_TO_MCU_STEER_CONTROL, buf, MAVLINK_MSG_ID_GCS_TO_MCU_STEER_CONTROL_MIN_LEN, MAVLINK_MSG_ID_GCS_TO_MCU_STEER_CONTROL_LEN, MAVLINK_MSG_ID_GCS_TO_MCU_STEER_CONTROL_CRC);
#else
    mavlink_gcs_to_mcu_steer_control_t packet;
    packet.thrust_value = thrust_value;
    packet.pitch_value = pitch_value;
    packet.roll_value = roll_value;
    packet.yaw_value = yaw_value;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GCS_TO_MCU_STEER_CONTROL, (const char *)&packet, MAVLINK_MSG_ID_GCS_TO_MCU_STEER_CONTROL_MIN_LEN, MAVLINK_MSG_ID_GCS_TO_MCU_STEER_CONTROL_LEN, MAVLINK_MSG_ID_GCS_TO_MCU_STEER_CONTROL_CRC);
#endif
}

/**
 * @brief Send a gcs_to_mcu_steer_control message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_gcs_to_mcu_steer_control_send_struct(mavlink_channel_t chan, const mavlink_gcs_to_mcu_steer_control_t* gcs_to_mcu_steer_control)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_gcs_to_mcu_steer_control_send(chan, gcs_to_mcu_steer_control->thrust_value, gcs_to_mcu_steer_control->pitch_value, gcs_to_mcu_steer_control->roll_value, gcs_to_mcu_steer_control->yaw_value);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GCS_TO_MCU_STEER_CONTROL, (const char *)gcs_to_mcu_steer_control, MAVLINK_MSG_ID_GCS_TO_MCU_STEER_CONTROL_MIN_LEN, MAVLINK_MSG_ID_GCS_TO_MCU_STEER_CONTROL_LEN, MAVLINK_MSG_ID_GCS_TO_MCU_STEER_CONTROL_CRC);
#endif
}

#if MAVLINK_MSG_ID_GCS_TO_MCU_STEER_CONTROL_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_gcs_to_mcu_steer_control_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float thrust_value, float pitch_value, float roll_value, float yaw_value)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, thrust_value);
    _mav_put_float(buf, 4, pitch_value);
    _mav_put_float(buf, 8, roll_value);
    _mav_put_float(buf, 12, yaw_value);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GCS_TO_MCU_STEER_CONTROL, buf, MAVLINK_MSG_ID_GCS_TO_MCU_STEER_CONTROL_MIN_LEN, MAVLINK_MSG_ID_GCS_TO_MCU_STEER_CONTROL_LEN, MAVLINK_MSG_ID_GCS_TO_MCU_STEER_CONTROL_CRC);
#else
    mavlink_gcs_to_mcu_steer_control_t *packet = (mavlink_gcs_to_mcu_steer_control_t *)msgbuf;
    packet->thrust_value = thrust_value;
    packet->pitch_value = pitch_value;
    packet->roll_value = roll_value;
    packet->yaw_value = yaw_value;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GCS_TO_MCU_STEER_CONTROL, (const char *)packet, MAVLINK_MSG_ID_GCS_TO_MCU_STEER_CONTROL_MIN_LEN, MAVLINK_MSG_ID_GCS_TO_MCU_STEER_CONTROL_LEN, MAVLINK_MSG_ID_GCS_TO_MCU_STEER_CONTROL_CRC);
#endif
}
#endif

#endif

// MESSAGE GCS_TO_MCU_STEER_CONTROL UNPACKING


/**
 * @brief Get field thrust_value from gcs_to_mcu_steer_control message
 *
 * @return  thrust value in range of [0.0, 1.0]. min thrust(0.0); max thrust(1.0) 
 */
static inline float mavlink_msg_gcs_to_mcu_steer_control_get_thrust_value(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field pitch_value from gcs_to_mcu_steer_control message
 *
 * @return  pitch value in range of [-1.0, 1.0]. forward (0, 1.0); backward [-1.0, 0); no pitch = 0 
 */
static inline float mavlink_msg_gcs_to_mcu_steer_control_get_pitch_value(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field roll_value from gcs_to_mcu_steer_control message
 *
 * @return  roll value in range of [-1.0, 1.0]. right (0, 1.0); left [-1.0, 0); no roll = 0 
 */
static inline float mavlink_msg_gcs_to_mcu_steer_control_get_roll_value(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field yaw_value from gcs_to_mcu_steer_control message
 *
 * @return  yaw value in range of [-1.0, 1.0]. counter-clockwise (0, 1.0); clockwise [-1.0, 0); no yaw = 0 
 */
static inline float mavlink_msg_gcs_to_mcu_steer_control_get_yaw_value(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Decode a gcs_to_mcu_steer_control message into a struct
 *
 * @param msg The message to decode
 * @param gcs_to_mcu_steer_control C-struct to decode the message contents into
 */
static inline void mavlink_msg_gcs_to_mcu_steer_control_decode(const mavlink_message_t* msg, mavlink_gcs_to_mcu_steer_control_t* gcs_to_mcu_steer_control)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    gcs_to_mcu_steer_control->thrust_value = mavlink_msg_gcs_to_mcu_steer_control_get_thrust_value(msg);
    gcs_to_mcu_steer_control->pitch_value = mavlink_msg_gcs_to_mcu_steer_control_get_pitch_value(msg);
    gcs_to_mcu_steer_control->roll_value = mavlink_msg_gcs_to_mcu_steer_control_get_roll_value(msg);
    gcs_to_mcu_steer_control->yaw_value = mavlink_msg_gcs_to_mcu_steer_control_get_yaw_value(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_GCS_TO_MCU_STEER_CONTROL_LEN? msg->len : MAVLINK_MSG_ID_GCS_TO_MCU_STEER_CONTROL_LEN;
        memset(gcs_to_mcu_steer_control, 0, MAVLINK_MSG_ID_GCS_TO_MCU_STEER_CONTROL_LEN);
    memcpy(gcs_to_mcu_steer_control, _MAV_PAYLOAD(msg), len);
#endif
}
