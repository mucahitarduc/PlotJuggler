#pragma once
// MESSAGE MCU_TO_GCS_HEARTBEAT PACKING

#define MAVLINK_MSG_ID_MCU_TO_GCS_HEARTBEAT 197

MAVPACKED(
typedef struct __mavlink_mcu_to_gcs_heartbeat_t {
 uint32_t status; /*< Integer bitmask indicating which fields are enabled. See definition for MCU_TO_GCS_STATUS_FLAGS*/
 uint32_t error_code; /*< Indicating MCU error code. Equals 0 if there is no error*/
 uint32_t emergency_code; /*< Indicating AP emergency code. Equals 0 if no emergency */
 uint32_t flight_time; /*< Indicating flight time in seconds*/
 uint32_t sensor_health; /*< Indicating sensors' health if REASON_MASK_SENSORS_FAIL flag is set */
}) mavlink_mcu_to_gcs_heartbeat_t;

#define MAVLINK_MSG_ID_MCU_TO_GCS_HEARTBEAT_LEN 20
#define MAVLINK_MSG_ID_MCU_TO_GCS_HEARTBEAT_MIN_LEN 20
#define MAVLINK_MSG_ID_197_LEN 20
#define MAVLINK_MSG_ID_197_MIN_LEN 20

#define MAVLINK_MSG_ID_MCU_TO_GCS_HEARTBEAT_CRC 60
#define MAVLINK_MSG_ID_197_CRC 60



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_MCU_TO_GCS_HEARTBEAT { \
    197, \
    "MCU_TO_GCS_HEARTBEAT", \
    5, \
    {  { "status", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_mcu_to_gcs_heartbeat_t, status) }, \
         { "error_code", NULL, MAVLINK_TYPE_UINT32_T, 0, 4, offsetof(mavlink_mcu_to_gcs_heartbeat_t, error_code) }, \
         { "emergency_code", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_mcu_to_gcs_heartbeat_t, emergency_code) }, \
         { "flight_time", NULL, MAVLINK_TYPE_UINT32_T, 0, 12, offsetof(mavlink_mcu_to_gcs_heartbeat_t, flight_time) }, \
         { "sensor_health", NULL, MAVLINK_TYPE_UINT32_T, 0, 16, offsetof(mavlink_mcu_to_gcs_heartbeat_t, sensor_health) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_MCU_TO_GCS_HEARTBEAT { \
    "MCU_TO_GCS_HEARTBEAT", \
    5, \
    {  { "status", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_mcu_to_gcs_heartbeat_t, status) }, \
         { "error_code", NULL, MAVLINK_TYPE_UINT32_T, 0, 4, offsetof(mavlink_mcu_to_gcs_heartbeat_t, error_code) }, \
         { "emergency_code", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_mcu_to_gcs_heartbeat_t, emergency_code) }, \
         { "flight_time", NULL, MAVLINK_TYPE_UINT32_T, 0, 12, offsetof(mavlink_mcu_to_gcs_heartbeat_t, flight_time) }, \
         { "sensor_health", NULL, MAVLINK_TYPE_UINT32_T, 0, 16, offsetof(mavlink_mcu_to_gcs_heartbeat_t, sensor_health) }, \
         } \
}
#endif

/**
 * @brief Pack a mcu_to_gcs_heartbeat message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param status Integer bitmask indicating which fields are enabled. See definition for MCU_TO_GCS_STATUS_FLAGS
 * @param error_code Indicating MCU error code. Equals 0 if there is no error
 * @param emergency_code Indicating AP emergency code. Equals 0 if no emergency 
 * @param flight_time Indicating flight time in seconds
 * @param sensor_health Indicating sensors' health if REASON_MASK_SENSORS_FAIL flag is set 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mcu_to_gcs_heartbeat_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t status, uint32_t error_code, uint32_t emergency_code, uint32_t flight_time, uint32_t sensor_health)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MCU_TO_GCS_HEARTBEAT_LEN];
    _mav_put_uint32_t(buf, 0, status);
    _mav_put_uint32_t(buf, 4, error_code);
    _mav_put_uint32_t(buf, 8, emergency_code);
    _mav_put_uint32_t(buf, 12, flight_time);
    _mav_put_uint32_t(buf, 16, sensor_health);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MCU_TO_GCS_HEARTBEAT_LEN);
#else
    mavlink_mcu_to_gcs_heartbeat_t packet;
    packet.status = status;
    packet.error_code = error_code;
    packet.emergency_code = emergency_code;
    packet.flight_time = flight_time;
    packet.sensor_health = sensor_health;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MCU_TO_GCS_HEARTBEAT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MCU_TO_GCS_HEARTBEAT;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MCU_TO_GCS_HEARTBEAT_MIN_LEN, MAVLINK_MSG_ID_MCU_TO_GCS_HEARTBEAT_LEN, MAVLINK_MSG_ID_MCU_TO_GCS_HEARTBEAT_CRC);
}

/**
 * @brief Pack a mcu_to_gcs_heartbeat message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param status Integer bitmask indicating which fields are enabled. See definition for MCU_TO_GCS_STATUS_FLAGS
 * @param error_code Indicating MCU error code. Equals 0 if there is no error
 * @param emergency_code Indicating AP emergency code. Equals 0 if no emergency 
 * @param flight_time Indicating flight time in seconds
 * @param sensor_health Indicating sensors' health if REASON_MASK_SENSORS_FAIL flag is set 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mcu_to_gcs_heartbeat_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t status,uint32_t error_code,uint32_t emergency_code,uint32_t flight_time,uint32_t sensor_health)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MCU_TO_GCS_HEARTBEAT_LEN];
    _mav_put_uint32_t(buf, 0, status);
    _mav_put_uint32_t(buf, 4, error_code);
    _mav_put_uint32_t(buf, 8, emergency_code);
    _mav_put_uint32_t(buf, 12, flight_time);
    _mav_put_uint32_t(buf, 16, sensor_health);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MCU_TO_GCS_HEARTBEAT_LEN);
#else
    mavlink_mcu_to_gcs_heartbeat_t packet;
    packet.status = status;
    packet.error_code = error_code;
    packet.emergency_code = emergency_code;
    packet.flight_time = flight_time;
    packet.sensor_health = sensor_health;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MCU_TO_GCS_HEARTBEAT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MCU_TO_GCS_HEARTBEAT;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MCU_TO_GCS_HEARTBEAT_MIN_LEN, MAVLINK_MSG_ID_MCU_TO_GCS_HEARTBEAT_LEN, MAVLINK_MSG_ID_MCU_TO_GCS_HEARTBEAT_CRC);
}

/**
 * @brief Encode a mcu_to_gcs_heartbeat struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param mcu_to_gcs_heartbeat C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mcu_to_gcs_heartbeat_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_mcu_to_gcs_heartbeat_t* mcu_to_gcs_heartbeat)
{
    return mavlink_msg_mcu_to_gcs_heartbeat_pack(system_id, component_id, msg, mcu_to_gcs_heartbeat->status, mcu_to_gcs_heartbeat->error_code, mcu_to_gcs_heartbeat->emergency_code, mcu_to_gcs_heartbeat->flight_time, mcu_to_gcs_heartbeat->sensor_health);
}

/**
 * @brief Encode a mcu_to_gcs_heartbeat struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param mcu_to_gcs_heartbeat C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mcu_to_gcs_heartbeat_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_mcu_to_gcs_heartbeat_t* mcu_to_gcs_heartbeat)
{
    return mavlink_msg_mcu_to_gcs_heartbeat_pack_chan(system_id, component_id, chan, msg, mcu_to_gcs_heartbeat->status, mcu_to_gcs_heartbeat->error_code, mcu_to_gcs_heartbeat->emergency_code, mcu_to_gcs_heartbeat->flight_time, mcu_to_gcs_heartbeat->sensor_health);
}

/**
 * @brief Send a mcu_to_gcs_heartbeat message
 * @param chan MAVLink channel to send the message
 *
 * @param status Integer bitmask indicating which fields are enabled. See definition for MCU_TO_GCS_STATUS_FLAGS
 * @param error_code Indicating MCU error code. Equals 0 if there is no error
 * @param emergency_code Indicating AP emergency code. Equals 0 if no emergency 
 * @param flight_time Indicating flight time in seconds
 * @param sensor_health Indicating sensors' health if REASON_MASK_SENSORS_FAIL flag is set 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_mcu_to_gcs_heartbeat_send(mavlink_channel_t chan, uint32_t status, uint32_t error_code, uint32_t emergency_code, uint32_t flight_time, uint32_t sensor_health)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MCU_TO_GCS_HEARTBEAT_LEN];
    _mav_put_uint32_t(buf, 0, status);
    _mav_put_uint32_t(buf, 4, error_code);
    _mav_put_uint32_t(buf, 8, emergency_code);
    _mav_put_uint32_t(buf, 12, flight_time);
    _mav_put_uint32_t(buf, 16, sensor_health);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MCU_TO_GCS_HEARTBEAT, buf, MAVLINK_MSG_ID_MCU_TO_GCS_HEARTBEAT_MIN_LEN, MAVLINK_MSG_ID_MCU_TO_GCS_HEARTBEAT_LEN, MAVLINK_MSG_ID_MCU_TO_GCS_HEARTBEAT_CRC);
#else
    mavlink_mcu_to_gcs_heartbeat_t packet;
    packet.status = status;
    packet.error_code = error_code;
    packet.emergency_code = emergency_code;
    packet.flight_time = flight_time;
    packet.sensor_health = sensor_health;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MCU_TO_GCS_HEARTBEAT, (const char *)&packet, MAVLINK_MSG_ID_MCU_TO_GCS_HEARTBEAT_MIN_LEN, MAVLINK_MSG_ID_MCU_TO_GCS_HEARTBEAT_LEN, MAVLINK_MSG_ID_MCU_TO_GCS_HEARTBEAT_CRC);
#endif
}

/**
 * @brief Send a mcu_to_gcs_heartbeat message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_mcu_to_gcs_heartbeat_send_struct(mavlink_channel_t chan, const mavlink_mcu_to_gcs_heartbeat_t* mcu_to_gcs_heartbeat)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_mcu_to_gcs_heartbeat_send(chan, mcu_to_gcs_heartbeat->status, mcu_to_gcs_heartbeat->error_code, mcu_to_gcs_heartbeat->emergency_code, mcu_to_gcs_heartbeat->flight_time, mcu_to_gcs_heartbeat->sensor_health);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MCU_TO_GCS_HEARTBEAT, (const char *)mcu_to_gcs_heartbeat, MAVLINK_MSG_ID_MCU_TO_GCS_HEARTBEAT_MIN_LEN, MAVLINK_MSG_ID_MCU_TO_GCS_HEARTBEAT_LEN, MAVLINK_MSG_ID_MCU_TO_GCS_HEARTBEAT_CRC);
#endif
}

#if MAVLINK_MSG_ID_MCU_TO_GCS_HEARTBEAT_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_mcu_to_gcs_heartbeat_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t status, uint32_t error_code, uint32_t emergency_code, uint32_t flight_time, uint32_t sensor_health)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, status);
    _mav_put_uint32_t(buf, 4, error_code);
    _mav_put_uint32_t(buf, 8, emergency_code);
    _mav_put_uint32_t(buf, 12, flight_time);
    _mav_put_uint32_t(buf, 16, sensor_health);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MCU_TO_GCS_HEARTBEAT, buf, MAVLINK_MSG_ID_MCU_TO_GCS_HEARTBEAT_MIN_LEN, MAVLINK_MSG_ID_MCU_TO_GCS_HEARTBEAT_LEN, MAVLINK_MSG_ID_MCU_TO_GCS_HEARTBEAT_CRC);
#else
    mavlink_mcu_to_gcs_heartbeat_t *packet = (mavlink_mcu_to_gcs_heartbeat_t *)msgbuf;
    packet->status = status;
    packet->error_code = error_code;
    packet->emergency_code = emergency_code;
    packet->flight_time = flight_time;
    packet->sensor_health = sensor_health;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MCU_TO_GCS_HEARTBEAT, (const char *)packet, MAVLINK_MSG_ID_MCU_TO_GCS_HEARTBEAT_MIN_LEN, MAVLINK_MSG_ID_MCU_TO_GCS_HEARTBEAT_LEN, MAVLINK_MSG_ID_MCU_TO_GCS_HEARTBEAT_CRC);
#endif
}
#endif

#endif

// MESSAGE MCU_TO_GCS_HEARTBEAT UNPACKING


/**
 * @brief Get field status from mcu_to_gcs_heartbeat message
 *
 * @return Integer bitmask indicating which fields are enabled. See definition for MCU_TO_GCS_STATUS_FLAGS
 */
static inline uint32_t mavlink_msg_mcu_to_gcs_heartbeat_get_status(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field error_code from mcu_to_gcs_heartbeat message
 *
 * @return Indicating MCU error code. Equals 0 if there is no error
 */
static inline uint32_t mavlink_msg_mcu_to_gcs_heartbeat_get_error_code(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  4);
}

/**
 * @brief Get field emergency_code from mcu_to_gcs_heartbeat message
 *
 * @return Indicating AP emergency code. Equals 0 if no emergency 
 */
static inline uint32_t mavlink_msg_mcu_to_gcs_heartbeat_get_emergency_code(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  8);
}

/**
 * @brief Get field flight_time from mcu_to_gcs_heartbeat message
 *
 * @return Indicating flight time in seconds
 */
static inline uint32_t mavlink_msg_mcu_to_gcs_heartbeat_get_flight_time(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  12);
}

/**
 * @brief Get field sensor_health from mcu_to_gcs_heartbeat message
 *
 * @return Indicating sensors' health if REASON_MASK_SENSORS_FAIL flag is set 
 */
static inline uint32_t mavlink_msg_mcu_to_gcs_heartbeat_get_sensor_health(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  16);
}

/**
 * @brief Decode a mcu_to_gcs_heartbeat message into a struct
 *
 * @param msg The message to decode
 * @param mcu_to_gcs_heartbeat C-struct to decode the message contents into
 */
static inline void mavlink_msg_mcu_to_gcs_heartbeat_decode(const mavlink_message_t* msg, mavlink_mcu_to_gcs_heartbeat_t* mcu_to_gcs_heartbeat)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mcu_to_gcs_heartbeat->status = mavlink_msg_mcu_to_gcs_heartbeat_get_status(msg);
    mcu_to_gcs_heartbeat->error_code = mavlink_msg_mcu_to_gcs_heartbeat_get_error_code(msg);
    mcu_to_gcs_heartbeat->emergency_code = mavlink_msg_mcu_to_gcs_heartbeat_get_emergency_code(msg);
    mcu_to_gcs_heartbeat->flight_time = mavlink_msg_mcu_to_gcs_heartbeat_get_flight_time(msg);
    mcu_to_gcs_heartbeat->sensor_health = mavlink_msg_mcu_to_gcs_heartbeat_get_sensor_health(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_MCU_TO_GCS_HEARTBEAT_LEN? msg->len : MAVLINK_MSG_ID_MCU_TO_GCS_HEARTBEAT_LEN;
        memset(mcu_to_gcs_heartbeat, 0, MAVLINK_MSG_ID_MCU_TO_GCS_HEARTBEAT_LEN);
    memcpy(mcu_to_gcs_heartbeat, _MAV_PAYLOAD(msg), len);
#endif
}
