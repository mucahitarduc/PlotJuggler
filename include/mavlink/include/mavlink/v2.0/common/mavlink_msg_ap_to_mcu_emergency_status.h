#pragma once
// MESSAGE AP_TO_MCU_EMERGENCY_STATUS PACKING

#define MAVLINK_MSG_ID_AP_TO_MCU_EMERGENCY_STATUS 203

MAVPACKED(
typedef struct __mavlink_ap_to_mcu_emergency_status_t {
 uint32_t reason; /*<  enum AP_EMERGENCY_REASON */
 uint32_t action; /*<  enum AP_EMERGENCY_ACTION */
 uint32_t sensor_health; /*<  enum AP_EMERGENCY_SENSOR_HEALTH */
}) mavlink_ap_to_mcu_emergency_status_t;

#define MAVLINK_MSG_ID_AP_TO_MCU_EMERGENCY_STATUS_LEN 12
#define MAVLINK_MSG_ID_AP_TO_MCU_EMERGENCY_STATUS_MIN_LEN 12
#define MAVLINK_MSG_ID_203_LEN 12
#define MAVLINK_MSG_ID_203_MIN_LEN 12

#define MAVLINK_MSG_ID_AP_TO_MCU_EMERGENCY_STATUS_CRC 110
#define MAVLINK_MSG_ID_203_CRC 110



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_AP_TO_MCU_EMERGENCY_STATUS { \
    203, \
    "AP_TO_MCU_EMERGENCY_STATUS", \
    3, \
    {  { "reason", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_ap_to_mcu_emergency_status_t, reason) }, \
         { "action", NULL, MAVLINK_TYPE_UINT32_T, 0, 4, offsetof(mavlink_ap_to_mcu_emergency_status_t, action) }, \
         { "sensor_health", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_ap_to_mcu_emergency_status_t, sensor_health) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_AP_TO_MCU_EMERGENCY_STATUS { \
    "AP_TO_MCU_EMERGENCY_STATUS", \
    3, \
    {  { "reason", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_ap_to_mcu_emergency_status_t, reason) }, \
         { "action", NULL, MAVLINK_TYPE_UINT32_T, 0, 4, offsetof(mavlink_ap_to_mcu_emergency_status_t, action) }, \
         { "sensor_health", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_ap_to_mcu_emergency_status_t, sensor_health) }, \
         } \
}
#endif

/**
 * @brief Pack a ap_to_mcu_emergency_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param reason  enum AP_EMERGENCY_REASON 
 * @param action  enum AP_EMERGENCY_ACTION 
 * @param sensor_health  enum AP_EMERGENCY_SENSOR_HEALTH 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ap_to_mcu_emergency_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t reason, uint32_t action, uint32_t sensor_health)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_AP_TO_MCU_EMERGENCY_STATUS_LEN];
    _mav_put_uint32_t(buf, 0, reason);
    _mav_put_uint32_t(buf, 4, action);
    _mav_put_uint32_t(buf, 8, sensor_health);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_AP_TO_MCU_EMERGENCY_STATUS_LEN);
#else
    mavlink_ap_to_mcu_emergency_status_t packet;
    packet.reason = reason;
    packet.action = action;
    packet.sensor_health = sensor_health;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_AP_TO_MCU_EMERGENCY_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_AP_TO_MCU_EMERGENCY_STATUS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_AP_TO_MCU_EMERGENCY_STATUS_MIN_LEN, MAVLINK_MSG_ID_AP_TO_MCU_EMERGENCY_STATUS_LEN, MAVLINK_MSG_ID_AP_TO_MCU_EMERGENCY_STATUS_CRC);
}

/**
 * @brief Pack a ap_to_mcu_emergency_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param reason  enum AP_EMERGENCY_REASON 
 * @param action  enum AP_EMERGENCY_ACTION 
 * @param sensor_health  enum AP_EMERGENCY_SENSOR_HEALTH 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ap_to_mcu_emergency_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t reason,uint32_t action,uint32_t sensor_health)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_AP_TO_MCU_EMERGENCY_STATUS_LEN];
    _mav_put_uint32_t(buf, 0, reason);
    _mav_put_uint32_t(buf, 4, action);
    _mav_put_uint32_t(buf, 8, sensor_health);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_AP_TO_MCU_EMERGENCY_STATUS_LEN);
#else
    mavlink_ap_to_mcu_emergency_status_t packet;
    packet.reason = reason;
    packet.action = action;
    packet.sensor_health = sensor_health;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_AP_TO_MCU_EMERGENCY_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_AP_TO_MCU_EMERGENCY_STATUS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_AP_TO_MCU_EMERGENCY_STATUS_MIN_LEN, MAVLINK_MSG_ID_AP_TO_MCU_EMERGENCY_STATUS_LEN, MAVLINK_MSG_ID_AP_TO_MCU_EMERGENCY_STATUS_CRC);
}

/**
 * @brief Encode a ap_to_mcu_emergency_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param ap_to_mcu_emergency_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_ap_to_mcu_emergency_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_ap_to_mcu_emergency_status_t* ap_to_mcu_emergency_status)
{
    return mavlink_msg_ap_to_mcu_emergency_status_pack(system_id, component_id, msg, ap_to_mcu_emergency_status->reason, ap_to_mcu_emergency_status->action, ap_to_mcu_emergency_status->sensor_health);
}

/**
 * @brief Encode a ap_to_mcu_emergency_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param ap_to_mcu_emergency_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_ap_to_mcu_emergency_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_ap_to_mcu_emergency_status_t* ap_to_mcu_emergency_status)
{
    return mavlink_msg_ap_to_mcu_emergency_status_pack_chan(system_id, component_id, chan, msg, ap_to_mcu_emergency_status->reason, ap_to_mcu_emergency_status->action, ap_to_mcu_emergency_status->sensor_health);
}

/**
 * @brief Send a ap_to_mcu_emergency_status message
 * @param chan MAVLink channel to send the message
 *
 * @param reason  enum AP_EMERGENCY_REASON 
 * @param action  enum AP_EMERGENCY_ACTION 
 * @param sensor_health  enum AP_EMERGENCY_SENSOR_HEALTH 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_ap_to_mcu_emergency_status_send(mavlink_channel_t chan, uint32_t reason, uint32_t action, uint32_t sensor_health)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_AP_TO_MCU_EMERGENCY_STATUS_LEN];
    _mav_put_uint32_t(buf, 0, reason);
    _mav_put_uint32_t(buf, 4, action);
    _mav_put_uint32_t(buf, 8, sensor_health);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AP_TO_MCU_EMERGENCY_STATUS, buf, MAVLINK_MSG_ID_AP_TO_MCU_EMERGENCY_STATUS_MIN_LEN, MAVLINK_MSG_ID_AP_TO_MCU_EMERGENCY_STATUS_LEN, MAVLINK_MSG_ID_AP_TO_MCU_EMERGENCY_STATUS_CRC);
#else
    mavlink_ap_to_mcu_emergency_status_t packet;
    packet.reason = reason;
    packet.action = action;
    packet.sensor_health = sensor_health;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AP_TO_MCU_EMERGENCY_STATUS, (const char *)&packet, MAVLINK_MSG_ID_AP_TO_MCU_EMERGENCY_STATUS_MIN_LEN, MAVLINK_MSG_ID_AP_TO_MCU_EMERGENCY_STATUS_LEN, MAVLINK_MSG_ID_AP_TO_MCU_EMERGENCY_STATUS_CRC);
#endif
}

/**
 * @brief Send a ap_to_mcu_emergency_status message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_ap_to_mcu_emergency_status_send_struct(mavlink_channel_t chan, const mavlink_ap_to_mcu_emergency_status_t* ap_to_mcu_emergency_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_ap_to_mcu_emergency_status_send(chan, ap_to_mcu_emergency_status->reason, ap_to_mcu_emergency_status->action, ap_to_mcu_emergency_status->sensor_health);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AP_TO_MCU_EMERGENCY_STATUS, (const char *)ap_to_mcu_emergency_status, MAVLINK_MSG_ID_AP_TO_MCU_EMERGENCY_STATUS_MIN_LEN, MAVLINK_MSG_ID_AP_TO_MCU_EMERGENCY_STATUS_LEN, MAVLINK_MSG_ID_AP_TO_MCU_EMERGENCY_STATUS_CRC);
#endif
}

#if MAVLINK_MSG_ID_AP_TO_MCU_EMERGENCY_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_ap_to_mcu_emergency_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t reason, uint32_t action, uint32_t sensor_health)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, reason);
    _mav_put_uint32_t(buf, 4, action);
    _mav_put_uint32_t(buf, 8, sensor_health);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AP_TO_MCU_EMERGENCY_STATUS, buf, MAVLINK_MSG_ID_AP_TO_MCU_EMERGENCY_STATUS_MIN_LEN, MAVLINK_MSG_ID_AP_TO_MCU_EMERGENCY_STATUS_LEN, MAVLINK_MSG_ID_AP_TO_MCU_EMERGENCY_STATUS_CRC);
#else
    mavlink_ap_to_mcu_emergency_status_t *packet = (mavlink_ap_to_mcu_emergency_status_t *)msgbuf;
    packet->reason = reason;
    packet->action = action;
    packet->sensor_health = sensor_health;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AP_TO_MCU_EMERGENCY_STATUS, (const char *)packet, MAVLINK_MSG_ID_AP_TO_MCU_EMERGENCY_STATUS_MIN_LEN, MAVLINK_MSG_ID_AP_TO_MCU_EMERGENCY_STATUS_LEN, MAVLINK_MSG_ID_AP_TO_MCU_EMERGENCY_STATUS_CRC);
#endif
}
#endif

#endif

// MESSAGE AP_TO_MCU_EMERGENCY_STATUS UNPACKING


/**
 * @brief Get field reason from ap_to_mcu_emergency_status message
 *
 * @return  enum AP_EMERGENCY_REASON 
 */
static inline uint32_t mavlink_msg_ap_to_mcu_emergency_status_get_reason(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field action from ap_to_mcu_emergency_status message
 *
 * @return  enum AP_EMERGENCY_ACTION 
 */
static inline uint32_t mavlink_msg_ap_to_mcu_emergency_status_get_action(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  4);
}

/**
 * @brief Get field sensor_health from ap_to_mcu_emergency_status message
 *
 * @return  enum AP_EMERGENCY_SENSOR_HEALTH 
 */
static inline uint32_t mavlink_msg_ap_to_mcu_emergency_status_get_sensor_health(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  8);
}

/**
 * @brief Decode a ap_to_mcu_emergency_status message into a struct
 *
 * @param msg The message to decode
 * @param ap_to_mcu_emergency_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_ap_to_mcu_emergency_status_decode(const mavlink_message_t* msg, mavlink_ap_to_mcu_emergency_status_t* ap_to_mcu_emergency_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    ap_to_mcu_emergency_status->reason = mavlink_msg_ap_to_mcu_emergency_status_get_reason(msg);
    ap_to_mcu_emergency_status->action = mavlink_msg_ap_to_mcu_emergency_status_get_action(msg);
    ap_to_mcu_emergency_status->sensor_health = mavlink_msg_ap_to_mcu_emergency_status_get_sensor_health(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_AP_TO_MCU_EMERGENCY_STATUS_LEN? msg->len : MAVLINK_MSG_ID_AP_TO_MCU_EMERGENCY_STATUS_LEN;
        memset(ap_to_mcu_emergency_status, 0, MAVLINK_MSG_ID_AP_TO_MCU_EMERGENCY_STATUS_LEN);
    memcpy(ap_to_mcu_emergency_status, _MAV_PAYLOAD(msg), len);
#endif
}
