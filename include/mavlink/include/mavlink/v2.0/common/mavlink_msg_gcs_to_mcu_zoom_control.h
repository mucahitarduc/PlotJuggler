#pragma once
// MESSAGE GCS_TO_MCU_ZOOM_CONTROL PACKING

#define MAVLINK_MSG_ID_GCS_TO_MCU_ZOOM_CONTROL 192

MAVPACKED(
typedef struct __mavlink_gcs_to_mcu_zoom_control_t {
 float requested_zoom_delta; /*< */
}) mavlink_gcs_to_mcu_zoom_control_t;

#define MAVLINK_MSG_ID_GCS_TO_MCU_ZOOM_CONTROL_LEN 4
#define MAVLINK_MSG_ID_GCS_TO_MCU_ZOOM_CONTROL_MIN_LEN 4
#define MAVLINK_MSG_ID_192_LEN 4
#define MAVLINK_MSG_ID_192_MIN_LEN 4

#define MAVLINK_MSG_ID_GCS_TO_MCU_ZOOM_CONTROL_CRC 8
#define MAVLINK_MSG_ID_192_CRC 8



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_GCS_TO_MCU_ZOOM_CONTROL { \
    192, \
    "GCS_TO_MCU_ZOOM_CONTROL", \
    1, \
    {  { "requested_zoom_delta", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_gcs_to_mcu_zoom_control_t, requested_zoom_delta) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_GCS_TO_MCU_ZOOM_CONTROL { \
    "GCS_TO_MCU_ZOOM_CONTROL", \
    1, \
    {  { "requested_zoom_delta", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_gcs_to_mcu_zoom_control_t, requested_zoom_delta) }, \
         } \
}
#endif

/**
 * @brief Pack a gcs_to_mcu_zoom_control message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param requested_zoom_delta 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gcs_to_mcu_zoom_control_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float requested_zoom_delta)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GCS_TO_MCU_ZOOM_CONTROL_LEN];
    _mav_put_float(buf, 0, requested_zoom_delta);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GCS_TO_MCU_ZOOM_CONTROL_LEN);
#else
    mavlink_gcs_to_mcu_zoom_control_t packet;
    packet.requested_zoom_delta = requested_zoom_delta;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GCS_TO_MCU_ZOOM_CONTROL_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GCS_TO_MCU_ZOOM_CONTROL;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_GCS_TO_MCU_ZOOM_CONTROL_MIN_LEN, MAVLINK_MSG_ID_GCS_TO_MCU_ZOOM_CONTROL_LEN, MAVLINK_MSG_ID_GCS_TO_MCU_ZOOM_CONTROL_CRC);
}

/**
 * @brief Pack a gcs_to_mcu_zoom_control message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param requested_zoom_delta 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gcs_to_mcu_zoom_control_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float requested_zoom_delta)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GCS_TO_MCU_ZOOM_CONTROL_LEN];
    _mav_put_float(buf, 0, requested_zoom_delta);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GCS_TO_MCU_ZOOM_CONTROL_LEN);
#else
    mavlink_gcs_to_mcu_zoom_control_t packet;
    packet.requested_zoom_delta = requested_zoom_delta;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GCS_TO_MCU_ZOOM_CONTROL_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GCS_TO_MCU_ZOOM_CONTROL;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_GCS_TO_MCU_ZOOM_CONTROL_MIN_LEN, MAVLINK_MSG_ID_GCS_TO_MCU_ZOOM_CONTROL_LEN, MAVLINK_MSG_ID_GCS_TO_MCU_ZOOM_CONTROL_CRC);
}

/**
 * @brief Encode a gcs_to_mcu_zoom_control struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param gcs_to_mcu_zoom_control C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gcs_to_mcu_zoom_control_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_gcs_to_mcu_zoom_control_t* gcs_to_mcu_zoom_control)
{
    return mavlink_msg_gcs_to_mcu_zoom_control_pack(system_id, component_id, msg, gcs_to_mcu_zoom_control->requested_zoom_delta);
}

/**
 * @brief Encode a gcs_to_mcu_zoom_control struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param gcs_to_mcu_zoom_control C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gcs_to_mcu_zoom_control_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_gcs_to_mcu_zoom_control_t* gcs_to_mcu_zoom_control)
{
    return mavlink_msg_gcs_to_mcu_zoom_control_pack_chan(system_id, component_id, chan, msg, gcs_to_mcu_zoom_control->requested_zoom_delta);
}

/**
 * @brief Send a gcs_to_mcu_zoom_control message
 * @param chan MAVLink channel to send the message
 *
 * @param requested_zoom_delta 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_gcs_to_mcu_zoom_control_send(mavlink_channel_t chan, float requested_zoom_delta)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GCS_TO_MCU_ZOOM_CONTROL_LEN];
    _mav_put_float(buf, 0, requested_zoom_delta);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GCS_TO_MCU_ZOOM_CONTROL, buf, MAVLINK_MSG_ID_GCS_TO_MCU_ZOOM_CONTROL_MIN_LEN, MAVLINK_MSG_ID_GCS_TO_MCU_ZOOM_CONTROL_LEN, MAVLINK_MSG_ID_GCS_TO_MCU_ZOOM_CONTROL_CRC);
#else
    mavlink_gcs_to_mcu_zoom_control_t packet;
    packet.requested_zoom_delta = requested_zoom_delta;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GCS_TO_MCU_ZOOM_CONTROL, (const char *)&packet, MAVLINK_MSG_ID_GCS_TO_MCU_ZOOM_CONTROL_MIN_LEN, MAVLINK_MSG_ID_GCS_TO_MCU_ZOOM_CONTROL_LEN, MAVLINK_MSG_ID_GCS_TO_MCU_ZOOM_CONTROL_CRC);
#endif
}

/**
 * @brief Send a gcs_to_mcu_zoom_control message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_gcs_to_mcu_zoom_control_send_struct(mavlink_channel_t chan, const mavlink_gcs_to_mcu_zoom_control_t* gcs_to_mcu_zoom_control)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_gcs_to_mcu_zoom_control_send(chan, gcs_to_mcu_zoom_control->requested_zoom_delta);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GCS_TO_MCU_ZOOM_CONTROL, (const char *)gcs_to_mcu_zoom_control, MAVLINK_MSG_ID_GCS_TO_MCU_ZOOM_CONTROL_MIN_LEN, MAVLINK_MSG_ID_GCS_TO_MCU_ZOOM_CONTROL_LEN, MAVLINK_MSG_ID_GCS_TO_MCU_ZOOM_CONTROL_CRC);
#endif
}

#if MAVLINK_MSG_ID_GCS_TO_MCU_ZOOM_CONTROL_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_gcs_to_mcu_zoom_control_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float requested_zoom_delta)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, requested_zoom_delta);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GCS_TO_MCU_ZOOM_CONTROL, buf, MAVLINK_MSG_ID_GCS_TO_MCU_ZOOM_CONTROL_MIN_LEN, MAVLINK_MSG_ID_GCS_TO_MCU_ZOOM_CONTROL_LEN, MAVLINK_MSG_ID_GCS_TO_MCU_ZOOM_CONTROL_CRC);
#else
    mavlink_gcs_to_mcu_zoom_control_t *packet = (mavlink_gcs_to_mcu_zoom_control_t *)msgbuf;
    packet->requested_zoom_delta = requested_zoom_delta;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GCS_TO_MCU_ZOOM_CONTROL, (const char *)packet, MAVLINK_MSG_ID_GCS_TO_MCU_ZOOM_CONTROL_MIN_LEN, MAVLINK_MSG_ID_GCS_TO_MCU_ZOOM_CONTROL_LEN, MAVLINK_MSG_ID_GCS_TO_MCU_ZOOM_CONTROL_CRC);
#endif
}
#endif

#endif

// MESSAGE GCS_TO_MCU_ZOOM_CONTROL UNPACKING


/**
 * @brief Get field requested_zoom_delta from gcs_to_mcu_zoom_control message
 *
 * @return 
 */
static inline float mavlink_msg_gcs_to_mcu_zoom_control_get_requested_zoom_delta(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Decode a gcs_to_mcu_zoom_control message into a struct
 *
 * @param msg The message to decode
 * @param gcs_to_mcu_zoom_control C-struct to decode the message contents into
 */
static inline void mavlink_msg_gcs_to_mcu_zoom_control_decode(const mavlink_message_t* msg, mavlink_gcs_to_mcu_zoom_control_t* gcs_to_mcu_zoom_control)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    gcs_to_mcu_zoom_control->requested_zoom_delta = mavlink_msg_gcs_to_mcu_zoom_control_get_requested_zoom_delta(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_GCS_TO_MCU_ZOOM_CONTROL_LEN? msg->len : MAVLINK_MSG_ID_GCS_TO_MCU_ZOOM_CONTROL_LEN;
        memset(gcs_to_mcu_zoom_control, 0, MAVLINK_MSG_ID_GCS_TO_MCU_ZOOM_CONTROL_LEN);
    memcpy(gcs_to_mcu_zoom_control, _MAV_PAYLOAD(msg), len);
#endif
}
