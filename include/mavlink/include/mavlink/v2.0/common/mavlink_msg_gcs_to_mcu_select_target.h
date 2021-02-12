#pragma once
// MESSAGE GCS_TO_MCU_SELECT_TARGET PACKING

#define MAVLINK_MSG_ID_GCS_TO_MCU_SELECT_TARGET 190

MAVPACKED(
typedef struct __mavlink_gcs_to_mcu_select_target_t {
 uint64_t selection_time; /*<  selection epoch time in milliseconds. */
 float center_x_ratio; /*<  ratio of center.x to screen width. */
 float center_y_ratio; /*<  ratio of center.y to screen width. */
 float width_ratio; /*<  ratio of width of selected area to width of screen. */
 float height_ratio; /*<  ratio of height of selected area to height of screen. */
}) mavlink_gcs_to_mcu_select_target_t;

#define MAVLINK_MSG_ID_GCS_TO_MCU_SELECT_TARGET_LEN 24
#define MAVLINK_MSG_ID_GCS_TO_MCU_SELECT_TARGET_MIN_LEN 24
#define MAVLINK_MSG_ID_190_LEN 24
#define MAVLINK_MSG_ID_190_MIN_LEN 24

#define MAVLINK_MSG_ID_GCS_TO_MCU_SELECT_TARGET_CRC 144
#define MAVLINK_MSG_ID_190_CRC 144



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_GCS_TO_MCU_SELECT_TARGET { \
    190, \
    "GCS_TO_MCU_SELECT_TARGET", \
    5, \
    {  { "selection_time", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_gcs_to_mcu_select_target_t, selection_time) }, \
         { "center_x_ratio", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_gcs_to_mcu_select_target_t, center_x_ratio) }, \
         { "center_y_ratio", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_gcs_to_mcu_select_target_t, center_y_ratio) }, \
         { "width_ratio", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_gcs_to_mcu_select_target_t, width_ratio) }, \
         { "height_ratio", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_gcs_to_mcu_select_target_t, height_ratio) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_GCS_TO_MCU_SELECT_TARGET { \
    "GCS_TO_MCU_SELECT_TARGET", \
    5, \
    {  { "selection_time", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_gcs_to_mcu_select_target_t, selection_time) }, \
         { "center_x_ratio", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_gcs_to_mcu_select_target_t, center_x_ratio) }, \
         { "center_y_ratio", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_gcs_to_mcu_select_target_t, center_y_ratio) }, \
         { "width_ratio", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_gcs_to_mcu_select_target_t, width_ratio) }, \
         { "height_ratio", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_gcs_to_mcu_select_target_t, height_ratio) }, \
         } \
}
#endif

/**
 * @brief Pack a gcs_to_mcu_select_target message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param center_x_ratio  ratio of center.x to screen width. 
 * @param center_y_ratio  ratio of center.y to screen width. 
 * @param width_ratio  ratio of width of selected area to width of screen. 
 * @param height_ratio  ratio of height of selected area to height of screen. 
 * @param selection_time  selection epoch time in milliseconds. 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gcs_to_mcu_select_target_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float center_x_ratio, float center_y_ratio, float width_ratio, float height_ratio, uint64_t selection_time)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GCS_TO_MCU_SELECT_TARGET_LEN];
    _mav_put_uint64_t(buf, 0, selection_time);
    _mav_put_float(buf, 8, center_x_ratio);
    _mav_put_float(buf, 12, center_y_ratio);
    _mav_put_float(buf, 16, width_ratio);
    _mav_put_float(buf, 20, height_ratio);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GCS_TO_MCU_SELECT_TARGET_LEN);
#else
    mavlink_gcs_to_mcu_select_target_t packet;
    packet.selection_time = selection_time;
    packet.center_x_ratio = center_x_ratio;
    packet.center_y_ratio = center_y_ratio;
    packet.width_ratio = width_ratio;
    packet.height_ratio = height_ratio;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GCS_TO_MCU_SELECT_TARGET_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GCS_TO_MCU_SELECT_TARGET;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_GCS_TO_MCU_SELECT_TARGET_MIN_LEN, MAVLINK_MSG_ID_GCS_TO_MCU_SELECT_TARGET_LEN, MAVLINK_MSG_ID_GCS_TO_MCU_SELECT_TARGET_CRC);
}

/**
 * @brief Pack a gcs_to_mcu_select_target message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param center_x_ratio  ratio of center.x to screen width. 
 * @param center_y_ratio  ratio of center.y to screen width. 
 * @param width_ratio  ratio of width of selected area to width of screen. 
 * @param height_ratio  ratio of height of selected area to height of screen. 
 * @param selection_time  selection epoch time in milliseconds. 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gcs_to_mcu_select_target_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float center_x_ratio,float center_y_ratio,float width_ratio,float height_ratio,uint64_t selection_time)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GCS_TO_MCU_SELECT_TARGET_LEN];
    _mav_put_uint64_t(buf, 0, selection_time);
    _mav_put_float(buf, 8, center_x_ratio);
    _mav_put_float(buf, 12, center_y_ratio);
    _mav_put_float(buf, 16, width_ratio);
    _mav_put_float(buf, 20, height_ratio);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GCS_TO_MCU_SELECT_TARGET_LEN);
#else
    mavlink_gcs_to_mcu_select_target_t packet;
    packet.selection_time = selection_time;
    packet.center_x_ratio = center_x_ratio;
    packet.center_y_ratio = center_y_ratio;
    packet.width_ratio = width_ratio;
    packet.height_ratio = height_ratio;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GCS_TO_MCU_SELECT_TARGET_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GCS_TO_MCU_SELECT_TARGET;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_GCS_TO_MCU_SELECT_TARGET_MIN_LEN, MAVLINK_MSG_ID_GCS_TO_MCU_SELECT_TARGET_LEN, MAVLINK_MSG_ID_GCS_TO_MCU_SELECT_TARGET_CRC);
}

/**
 * @brief Encode a gcs_to_mcu_select_target struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param gcs_to_mcu_select_target C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gcs_to_mcu_select_target_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_gcs_to_mcu_select_target_t* gcs_to_mcu_select_target)
{
    return mavlink_msg_gcs_to_mcu_select_target_pack(system_id, component_id, msg, gcs_to_mcu_select_target->center_x_ratio, gcs_to_mcu_select_target->center_y_ratio, gcs_to_mcu_select_target->width_ratio, gcs_to_mcu_select_target->height_ratio, gcs_to_mcu_select_target->selection_time);
}

/**
 * @brief Encode a gcs_to_mcu_select_target struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param gcs_to_mcu_select_target C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gcs_to_mcu_select_target_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_gcs_to_mcu_select_target_t* gcs_to_mcu_select_target)
{
    return mavlink_msg_gcs_to_mcu_select_target_pack_chan(system_id, component_id, chan, msg, gcs_to_mcu_select_target->center_x_ratio, gcs_to_mcu_select_target->center_y_ratio, gcs_to_mcu_select_target->width_ratio, gcs_to_mcu_select_target->height_ratio, gcs_to_mcu_select_target->selection_time);
}

/**
 * @brief Send a gcs_to_mcu_select_target message
 * @param chan MAVLink channel to send the message
 *
 * @param center_x_ratio  ratio of center.x to screen width. 
 * @param center_y_ratio  ratio of center.y to screen width. 
 * @param width_ratio  ratio of width of selected area to width of screen. 
 * @param height_ratio  ratio of height of selected area to height of screen. 
 * @param selection_time  selection epoch time in milliseconds. 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_gcs_to_mcu_select_target_send(mavlink_channel_t chan, float center_x_ratio, float center_y_ratio, float width_ratio, float height_ratio, uint64_t selection_time)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GCS_TO_MCU_SELECT_TARGET_LEN];
    _mav_put_uint64_t(buf, 0, selection_time);
    _mav_put_float(buf, 8, center_x_ratio);
    _mav_put_float(buf, 12, center_y_ratio);
    _mav_put_float(buf, 16, width_ratio);
    _mav_put_float(buf, 20, height_ratio);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GCS_TO_MCU_SELECT_TARGET, buf, MAVLINK_MSG_ID_GCS_TO_MCU_SELECT_TARGET_MIN_LEN, MAVLINK_MSG_ID_GCS_TO_MCU_SELECT_TARGET_LEN, MAVLINK_MSG_ID_GCS_TO_MCU_SELECT_TARGET_CRC);
#else
    mavlink_gcs_to_mcu_select_target_t packet;
    packet.selection_time = selection_time;
    packet.center_x_ratio = center_x_ratio;
    packet.center_y_ratio = center_y_ratio;
    packet.width_ratio = width_ratio;
    packet.height_ratio = height_ratio;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GCS_TO_MCU_SELECT_TARGET, (const char *)&packet, MAVLINK_MSG_ID_GCS_TO_MCU_SELECT_TARGET_MIN_LEN, MAVLINK_MSG_ID_GCS_TO_MCU_SELECT_TARGET_LEN, MAVLINK_MSG_ID_GCS_TO_MCU_SELECT_TARGET_CRC);
#endif
}

/**
 * @brief Send a gcs_to_mcu_select_target message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_gcs_to_mcu_select_target_send_struct(mavlink_channel_t chan, const mavlink_gcs_to_mcu_select_target_t* gcs_to_mcu_select_target)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_gcs_to_mcu_select_target_send(chan, gcs_to_mcu_select_target->center_x_ratio, gcs_to_mcu_select_target->center_y_ratio, gcs_to_mcu_select_target->width_ratio, gcs_to_mcu_select_target->height_ratio, gcs_to_mcu_select_target->selection_time);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GCS_TO_MCU_SELECT_TARGET, (const char *)gcs_to_mcu_select_target, MAVLINK_MSG_ID_GCS_TO_MCU_SELECT_TARGET_MIN_LEN, MAVLINK_MSG_ID_GCS_TO_MCU_SELECT_TARGET_LEN, MAVLINK_MSG_ID_GCS_TO_MCU_SELECT_TARGET_CRC);
#endif
}

#if MAVLINK_MSG_ID_GCS_TO_MCU_SELECT_TARGET_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_gcs_to_mcu_select_target_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float center_x_ratio, float center_y_ratio, float width_ratio, float height_ratio, uint64_t selection_time)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, selection_time);
    _mav_put_float(buf, 8, center_x_ratio);
    _mav_put_float(buf, 12, center_y_ratio);
    _mav_put_float(buf, 16, width_ratio);
    _mav_put_float(buf, 20, height_ratio);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GCS_TO_MCU_SELECT_TARGET, buf, MAVLINK_MSG_ID_GCS_TO_MCU_SELECT_TARGET_MIN_LEN, MAVLINK_MSG_ID_GCS_TO_MCU_SELECT_TARGET_LEN, MAVLINK_MSG_ID_GCS_TO_MCU_SELECT_TARGET_CRC);
#else
    mavlink_gcs_to_mcu_select_target_t *packet = (mavlink_gcs_to_mcu_select_target_t *)msgbuf;
    packet->selection_time = selection_time;
    packet->center_x_ratio = center_x_ratio;
    packet->center_y_ratio = center_y_ratio;
    packet->width_ratio = width_ratio;
    packet->height_ratio = height_ratio;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GCS_TO_MCU_SELECT_TARGET, (const char *)packet, MAVLINK_MSG_ID_GCS_TO_MCU_SELECT_TARGET_MIN_LEN, MAVLINK_MSG_ID_GCS_TO_MCU_SELECT_TARGET_LEN, MAVLINK_MSG_ID_GCS_TO_MCU_SELECT_TARGET_CRC);
#endif
}
#endif

#endif

// MESSAGE GCS_TO_MCU_SELECT_TARGET UNPACKING


/**
 * @brief Get field center_x_ratio from gcs_to_mcu_select_target message
 *
 * @return  ratio of center.x to screen width. 
 */
static inline float mavlink_msg_gcs_to_mcu_select_target_get_center_x_ratio(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field center_y_ratio from gcs_to_mcu_select_target message
 *
 * @return  ratio of center.y to screen width. 
 */
static inline float mavlink_msg_gcs_to_mcu_select_target_get_center_y_ratio(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field width_ratio from gcs_to_mcu_select_target message
 *
 * @return  ratio of width of selected area to width of screen. 
 */
static inline float mavlink_msg_gcs_to_mcu_select_target_get_width_ratio(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field height_ratio from gcs_to_mcu_select_target message
 *
 * @return  ratio of height of selected area to height of screen. 
 */
static inline float mavlink_msg_gcs_to_mcu_select_target_get_height_ratio(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field selection_time from gcs_to_mcu_select_target message
 *
 * @return  selection epoch time in milliseconds. 
 */
static inline uint64_t mavlink_msg_gcs_to_mcu_select_target_get_selection_time(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Decode a gcs_to_mcu_select_target message into a struct
 *
 * @param msg The message to decode
 * @param gcs_to_mcu_select_target C-struct to decode the message contents into
 */
static inline void mavlink_msg_gcs_to_mcu_select_target_decode(const mavlink_message_t* msg, mavlink_gcs_to_mcu_select_target_t* gcs_to_mcu_select_target)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    gcs_to_mcu_select_target->selection_time = mavlink_msg_gcs_to_mcu_select_target_get_selection_time(msg);
    gcs_to_mcu_select_target->center_x_ratio = mavlink_msg_gcs_to_mcu_select_target_get_center_x_ratio(msg);
    gcs_to_mcu_select_target->center_y_ratio = mavlink_msg_gcs_to_mcu_select_target_get_center_y_ratio(msg);
    gcs_to_mcu_select_target->width_ratio = mavlink_msg_gcs_to_mcu_select_target_get_width_ratio(msg);
    gcs_to_mcu_select_target->height_ratio = mavlink_msg_gcs_to_mcu_select_target_get_height_ratio(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_GCS_TO_MCU_SELECT_TARGET_LEN? msg->len : MAVLINK_MSG_ID_GCS_TO_MCU_SELECT_TARGET_LEN;
        memset(gcs_to_mcu_select_target, 0, MAVLINK_MSG_ID_GCS_TO_MCU_SELECT_TARGET_LEN);
    memcpy(gcs_to_mcu_select_target, _MAV_PAYLOAD(msg), len);
#endif
}
