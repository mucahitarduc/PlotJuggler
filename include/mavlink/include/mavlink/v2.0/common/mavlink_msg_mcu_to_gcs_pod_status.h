#pragma once
// MESSAGE MCU_TO_GCS_POD_STATUS PACKING

#define MAVLINK_MSG_ID_MCU_TO_GCS_POD_STATUS 194

MAVPACKED(
typedef struct __mavlink_mcu_to_gcs_pod_status_t {
 float current_zoom_level; /*< */
 float current_pitch; /*< */
 float current_roll; /*< */
 float current_yaw; /*< */
 uint8_t current_camera; /*< */
}) mavlink_mcu_to_gcs_pod_status_t;

#define MAVLINK_MSG_ID_MCU_TO_GCS_POD_STATUS_LEN 17
#define MAVLINK_MSG_ID_MCU_TO_GCS_POD_STATUS_MIN_LEN 17
#define MAVLINK_MSG_ID_194_LEN 17
#define MAVLINK_MSG_ID_194_MIN_LEN 17

#define MAVLINK_MSG_ID_MCU_TO_GCS_POD_STATUS_CRC 151
#define MAVLINK_MSG_ID_194_CRC 151



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_MCU_TO_GCS_POD_STATUS { \
    194, \
    "MCU_TO_GCS_POD_STATUS", \
    5, \
    {  { "current_zoom_level", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_mcu_to_gcs_pod_status_t, current_zoom_level) }, \
         { "current_pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_mcu_to_gcs_pod_status_t, current_pitch) }, \
         { "current_roll", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_mcu_to_gcs_pod_status_t, current_roll) }, \
         { "current_yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_mcu_to_gcs_pod_status_t, current_yaw) }, \
         { "current_camera", NULL, MAVLINK_TYPE_UINT8_T, 0, 16, offsetof(mavlink_mcu_to_gcs_pod_status_t, current_camera) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_MCU_TO_GCS_POD_STATUS { \
    "MCU_TO_GCS_POD_STATUS", \
    5, \
    {  { "current_zoom_level", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_mcu_to_gcs_pod_status_t, current_zoom_level) }, \
         { "current_pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_mcu_to_gcs_pod_status_t, current_pitch) }, \
         { "current_roll", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_mcu_to_gcs_pod_status_t, current_roll) }, \
         { "current_yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_mcu_to_gcs_pod_status_t, current_yaw) }, \
         { "current_camera", NULL, MAVLINK_TYPE_UINT8_T, 0, 16, offsetof(mavlink_mcu_to_gcs_pod_status_t, current_camera) }, \
         } \
}
#endif

/**
 * @brief Pack a mcu_to_gcs_pod_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param current_zoom_level 
 * @param current_pitch 
 * @param current_roll 
 * @param current_yaw 
 * @param current_camera 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mcu_to_gcs_pod_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float current_zoom_level, float current_pitch, float current_roll, float current_yaw, uint8_t current_camera)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MCU_TO_GCS_POD_STATUS_LEN];
    _mav_put_float(buf, 0, current_zoom_level);
    _mav_put_float(buf, 4, current_pitch);
    _mav_put_float(buf, 8, current_roll);
    _mav_put_float(buf, 12, current_yaw);
    _mav_put_uint8_t(buf, 16, current_camera);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MCU_TO_GCS_POD_STATUS_LEN);
#else
    mavlink_mcu_to_gcs_pod_status_t packet;
    packet.current_zoom_level = current_zoom_level;
    packet.current_pitch = current_pitch;
    packet.current_roll = current_roll;
    packet.current_yaw = current_yaw;
    packet.current_camera = current_camera;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MCU_TO_GCS_POD_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MCU_TO_GCS_POD_STATUS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MCU_TO_GCS_POD_STATUS_MIN_LEN, MAVLINK_MSG_ID_MCU_TO_GCS_POD_STATUS_LEN, MAVLINK_MSG_ID_MCU_TO_GCS_POD_STATUS_CRC);
}

/**
 * @brief Pack a mcu_to_gcs_pod_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param current_zoom_level 
 * @param current_pitch 
 * @param current_roll 
 * @param current_yaw 
 * @param current_camera 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mcu_to_gcs_pod_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float current_zoom_level,float current_pitch,float current_roll,float current_yaw,uint8_t current_camera)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MCU_TO_GCS_POD_STATUS_LEN];
    _mav_put_float(buf, 0, current_zoom_level);
    _mav_put_float(buf, 4, current_pitch);
    _mav_put_float(buf, 8, current_roll);
    _mav_put_float(buf, 12, current_yaw);
    _mav_put_uint8_t(buf, 16, current_camera);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MCU_TO_GCS_POD_STATUS_LEN);
#else
    mavlink_mcu_to_gcs_pod_status_t packet;
    packet.current_zoom_level = current_zoom_level;
    packet.current_pitch = current_pitch;
    packet.current_roll = current_roll;
    packet.current_yaw = current_yaw;
    packet.current_camera = current_camera;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MCU_TO_GCS_POD_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MCU_TO_GCS_POD_STATUS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MCU_TO_GCS_POD_STATUS_MIN_LEN, MAVLINK_MSG_ID_MCU_TO_GCS_POD_STATUS_LEN, MAVLINK_MSG_ID_MCU_TO_GCS_POD_STATUS_CRC);
}

/**
 * @brief Encode a mcu_to_gcs_pod_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param mcu_to_gcs_pod_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mcu_to_gcs_pod_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_mcu_to_gcs_pod_status_t* mcu_to_gcs_pod_status)
{
    return mavlink_msg_mcu_to_gcs_pod_status_pack(system_id, component_id, msg, mcu_to_gcs_pod_status->current_zoom_level, mcu_to_gcs_pod_status->current_pitch, mcu_to_gcs_pod_status->current_roll, mcu_to_gcs_pod_status->current_yaw, mcu_to_gcs_pod_status->current_camera);
}

/**
 * @brief Encode a mcu_to_gcs_pod_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param mcu_to_gcs_pod_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mcu_to_gcs_pod_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_mcu_to_gcs_pod_status_t* mcu_to_gcs_pod_status)
{
    return mavlink_msg_mcu_to_gcs_pod_status_pack_chan(system_id, component_id, chan, msg, mcu_to_gcs_pod_status->current_zoom_level, mcu_to_gcs_pod_status->current_pitch, mcu_to_gcs_pod_status->current_roll, mcu_to_gcs_pod_status->current_yaw, mcu_to_gcs_pod_status->current_camera);
}

/**
 * @brief Send a mcu_to_gcs_pod_status message
 * @param chan MAVLink channel to send the message
 *
 * @param current_zoom_level 
 * @param current_pitch 
 * @param current_roll 
 * @param current_yaw 
 * @param current_camera 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_mcu_to_gcs_pod_status_send(mavlink_channel_t chan, float current_zoom_level, float current_pitch, float current_roll, float current_yaw, uint8_t current_camera)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MCU_TO_GCS_POD_STATUS_LEN];
    _mav_put_float(buf, 0, current_zoom_level);
    _mav_put_float(buf, 4, current_pitch);
    _mav_put_float(buf, 8, current_roll);
    _mav_put_float(buf, 12, current_yaw);
    _mav_put_uint8_t(buf, 16, current_camera);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MCU_TO_GCS_POD_STATUS, buf, MAVLINK_MSG_ID_MCU_TO_GCS_POD_STATUS_MIN_LEN, MAVLINK_MSG_ID_MCU_TO_GCS_POD_STATUS_LEN, MAVLINK_MSG_ID_MCU_TO_GCS_POD_STATUS_CRC);
#else
    mavlink_mcu_to_gcs_pod_status_t packet;
    packet.current_zoom_level = current_zoom_level;
    packet.current_pitch = current_pitch;
    packet.current_roll = current_roll;
    packet.current_yaw = current_yaw;
    packet.current_camera = current_camera;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MCU_TO_GCS_POD_STATUS, (const char *)&packet, MAVLINK_MSG_ID_MCU_TO_GCS_POD_STATUS_MIN_LEN, MAVLINK_MSG_ID_MCU_TO_GCS_POD_STATUS_LEN, MAVLINK_MSG_ID_MCU_TO_GCS_POD_STATUS_CRC);
#endif
}

/**
 * @brief Send a mcu_to_gcs_pod_status message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_mcu_to_gcs_pod_status_send_struct(mavlink_channel_t chan, const mavlink_mcu_to_gcs_pod_status_t* mcu_to_gcs_pod_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_mcu_to_gcs_pod_status_send(chan, mcu_to_gcs_pod_status->current_zoom_level, mcu_to_gcs_pod_status->current_pitch, mcu_to_gcs_pod_status->current_roll, mcu_to_gcs_pod_status->current_yaw, mcu_to_gcs_pod_status->current_camera);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MCU_TO_GCS_POD_STATUS, (const char *)mcu_to_gcs_pod_status, MAVLINK_MSG_ID_MCU_TO_GCS_POD_STATUS_MIN_LEN, MAVLINK_MSG_ID_MCU_TO_GCS_POD_STATUS_LEN, MAVLINK_MSG_ID_MCU_TO_GCS_POD_STATUS_CRC);
#endif
}

#if MAVLINK_MSG_ID_MCU_TO_GCS_POD_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_mcu_to_gcs_pod_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float current_zoom_level, float current_pitch, float current_roll, float current_yaw, uint8_t current_camera)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, current_zoom_level);
    _mav_put_float(buf, 4, current_pitch);
    _mav_put_float(buf, 8, current_roll);
    _mav_put_float(buf, 12, current_yaw);
    _mav_put_uint8_t(buf, 16, current_camera);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MCU_TO_GCS_POD_STATUS, buf, MAVLINK_MSG_ID_MCU_TO_GCS_POD_STATUS_MIN_LEN, MAVLINK_MSG_ID_MCU_TO_GCS_POD_STATUS_LEN, MAVLINK_MSG_ID_MCU_TO_GCS_POD_STATUS_CRC);
#else
    mavlink_mcu_to_gcs_pod_status_t *packet = (mavlink_mcu_to_gcs_pod_status_t *)msgbuf;
    packet->current_zoom_level = current_zoom_level;
    packet->current_pitch = current_pitch;
    packet->current_roll = current_roll;
    packet->current_yaw = current_yaw;
    packet->current_camera = current_camera;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MCU_TO_GCS_POD_STATUS, (const char *)packet, MAVLINK_MSG_ID_MCU_TO_GCS_POD_STATUS_MIN_LEN, MAVLINK_MSG_ID_MCU_TO_GCS_POD_STATUS_LEN, MAVLINK_MSG_ID_MCU_TO_GCS_POD_STATUS_CRC);
#endif
}
#endif

#endif

// MESSAGE MCU_TO_GCS_POD_STATUS UNPACKING


/**
 * @brief Get field current_zoom_level from mcu_to_gcs_pod_status message
 *
 * @return 
 */
static inline float mavlink_msg_mcu_to_gcs_pod_status_get_current_zoom_level(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field current_pitch from mcu_to_gcs_pod_status message
 *
 * @return 
 */
static inline float mavlink_msg_mcu_to_gcs_pod_status_get_current_pitch(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field current_roll from mcu_to_gcs_pod_status message
 *
 * @return 
 */
static inline float mavlink_msg_mcu_to_gcs_pod_status_get_current_roll(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field current_yaw from mcu_to_gcs_pod_status message
 *
 * @return 
 */
static inline float mavlink_msg_mcu_to_gcs_pod_status_get_current_yaw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field current_camera from mcu_to_gcs_pod_status message
 *
 * @return 
 */
static inline uint8_t mavlink_msg_mcu_to_gcs_pod_status_get_current_camera(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  16);
}

/**
 * @brief Decode a mcu_to_gcs_pod_status message into a struct
 *
 * @param msg The message to decode
 * @param mcu_to_gcs_pod_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_mcu_to_gcs_pod_status_decode(const mavlink_message_t* msg, mavlink_mcu_to_gcs_pod_status_t* mcu_to_gcs_pod_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mcu_to_gcs_pod_status->current_zoom_level = mavlink_msg_mcu_to_gcs_pod_status_get_current_zoom_level(msg);
    mcu_to_gcs_pod_status->current_pitch = mavlink_msg_mcu_to_gcs_pod_status_get_current_pitch(msg);
    mcu_to_gcs_pod_status->current_roll = mavlink_msg_mcu_to_gcs_pod_status_get_current_roll(msg);
    mcu_to_gcs_pod_status->current_yaw = mavlink_msg_mcu_to_gcs_pod_status_get_current_yaw(msg);
    mcu_to_gcs_pod_status->current_camera = mavlink_msg_mcu_to_gcs_pod_status_get_current_camera(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_MCU_TO_GCS_POD_STATUS_LEN? msg->len : MAVLINK_MSG_ID_MCU_TO_GCS_POD_STATUS_LEN;
        memset(mcu_to_gcs_pod_status, 0, MAVLINK_MSG_ID_MCU_TO_GCS_POD_STATUS_LEN);
    memcpy(mcu_to_gcs_pod_status, _MAV_PAYLOAD(msg), len);
#endif
}
