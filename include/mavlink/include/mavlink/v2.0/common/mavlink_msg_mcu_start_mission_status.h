#pragma once
// MESSAGE MCU_START_MISSION_STATUS PACKING

#define MAVLINK_MSG_ID_MCU_START_MISSION_STATUS 204

MAVPACKED(
typedef struct __mavlink_mcu_start_mission_status_t {
 int16_t mode; /*<  enum KERKES_MODE_TYPE. Specifies current mode. */
 int16_t status; /*<  enum START_MISSION_STATUS_TYPE. Specifies current status of mission. */
}) mavlink_mcu_start_mission_status_t;

#define MAVLINK_MSG_ID_MCU_START_MISSION_STATUS_LEN 4
#define MAVLINK_MSG_ID_MCU_START_MISSION_STATUS_MIN_LEN 4
#define MAVLINK_MSG_ID_204_LEN 4
#define MAVLINK_MSG_ID_204_MIN_LEN 4

#define MAVLINK_MSG_ID_MCU_START_MISSION_STATUS_CRC 204
#define MAVLINK_MSG_ID_204_CRC 204



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_MCU_START_MISSION_STATUS { \
    204, \
    "MCU_START_MISSION_STATUS", \
    2, \
    {  { "mode", NULL, MAVLINK_TYPE_INT16_T, 0, 0, offsetof(mavlink_mcu_start_mission_status_t, mode) }, \
         { "status", NULL, MAVLINK_TYPE_INT16_T, 0, 2, offsetof(mavlink_mcu_start_mission_status_t, status) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_MCU_START_MISSION_STATUS { \
    "MCU_START_MISSION_STATUS", \
    2, \
    {  { "mode", NULL, MAVLINK_TYPE_INT16_T, 0, 0, offsetof(mavlink_mcu_start_mission_status_t, mode) }, \
         { "status", NULL, MAVLINK_TYPE_INT16_T, 0, 2, offsetof(mavlink_mcu_start_mission_status_t, status) }, \
         } \
}
#endif

/**
 * @brief Pack a mcu_start_mission_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param mode  enum KERKES_MODE_TYPE. Specifies current mode. 
 * @param status  enum START_MISSION_STATUS_TYPE. Specifies current status of mission. 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mcu_start_mission_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               int16_t mode, int16_t status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MCU_START_MISSION_STATUS_LEN];
    _mav_put_int16_t(buf, 0, mode);
    _mav_put_int16_t(buf, 2, status);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MCU_START_MISSION_STATUS_LEN);
#else
    mavlink_mcu_start_mission_status_t packet;
    packet.mode = mode;
    packet.status = status;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MCU_START_MISSION_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MCU_START_MISSION_STATUS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MCU_START_MISSION_STATUS_MIN_LEN, MAVLINK_MSG_ID_MCU_START_MISSION_STATUS_LEN, MAVLINK_MSG_ID_MCU_START_MISSION_STATUS_CRC);
}

/**
 * @brief Pack a mcu_start_mission_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param mode  enum KERKES_MODE_TYPE. Specifies current mode. 
 * @param status  enum START_MISSION_STATUS_TYPE. Specifies current status of mission. 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mcu_start_mission_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   int16_t mode,int16_t status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MCU_START_MISSION_STATUS_LEN];
    _mav_put_int16_t(buf, 0, mode);
    _mav_put_int16_t(buf, 2, status);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MCU_START_MISSION_STATUS_LEN);
#else
    mavlink_mcu_start_mission_status_t packet;
    packet.mode = mode;
    packet.status = status;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MCU_START_MISSION_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MCU_START_MISSION_STATUS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MCU_START_MISSION_STATUS_MIN_LEN, MAVLINK_MSG_ID_MCU_START_MISSION_STATUS_LEN, MAVLINK_MSG_ID_MCU_START_MISSION_STATUS_CRC);
}

/**
 * @brief Encode a mcu_start_mission_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param mcu_start_mission_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mcu_start_mission_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_mcu_start_mission_status_t* mcu_start_mission_status)
{
    return mavlink_msg_mcu_start_mission_status_pack(system_id, component_id, msg, mcu_start_mission_status->mode, mcu_start_mission_status->status);
}

/**
 * @brief Encode a mcu_start_mission_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param mcu_start_mission_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mcu_start_mission_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_mcu_start_mission_status_t* mcu_start_mission_status)
{
    return mavlink_msg_mcu_start_mission_status_pack_chan(system_id, component_id, chan, msg, mcu_start_mission_status->mode, mcu_start_mission_status->status);
}

/**
 * @brief Send a mcu_start_mission_status message
 * @param chan MAVLink channel to send the message
 *
 * @param mode  enum KERKES_MODE_TYPE. Specifies current mode. 
 * @param status  enum START_MISSION_STATUS_TYPE. Specifies current status of mission. 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_mcu_start_mission_status_send(mavlink_channel_t chan, int16_t mode, int16_t status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MCU_START_MISSION_STATUS_LEN];
    _mav_put_int16_t(buf, 0, mode);
    _mav_put_int16_t(buf, 2, status);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MCU_START_MISSION_STATUS, buf, MAVLINK_MSG_ID_MCU_START_MISSION_STATUS_MIN_LEN, MAVLINK_MSG_ID_MCU_START_MISSION_STATUS_LEN, MAVLINK_MSG_ID_MCU_START_MISSION_STATUS_CRC);
#else
    mavlink_mcu_start_mission_status_t packet;
    packet.mode = mode;
    packet.status = status;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MCU_START_MISSION_STATUS, (const char *)&packet, MAVLINK_MSG_ID_MCU_START_MISSION_STATUS_MIN_LEN, MAVLINK_MSG_ID_MCU_START_MISSION_STATUS_LEN, MAVLINK_MSG_ID_MCU_START_MISSION_STATUS_CRC);
#endif
}

/**
 * @brief Send a mcu_start_mission_status message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_mcu_start_mission_status_send_struct(mavlink_channel_t chan, const mavlink_mcu_start_mission_status_t* mcu_start_mission_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_mcu_start_mission_status_send(chan, mcu_start_mission_status->mode, mcu_start_mission_status->status);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MCU_START_MISSION_STATUS, (const char *)mcu_start_mission_status, MAVLINK_MSG_ID_MCU_START_MISSION_STATUS_MIN_LEN, MAVLINK_MSG_ID_MCU_START_MISSION_STATUS_LEN, MAVLINK_MSG_ID_MCU_START_MISSION_STATUS_CRC);
#endif
}

#if MAVLINK_MSG_ID_MCU_START_MISSION_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_mcu_start_mission_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  int16_t mode, int16_t status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_int16_t(buf, 0, mode);
    _mav_put_int16_t(buf, 2, status);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MCU_START_MISSION_STATUS, buf, MAVLINK_MSG_ID_MCU_START_MISSION_STATUS_MIN_LEN, MAVLINK_MSG_ID_MCU_START_MISSION_STATUS_LEN, MAVLINK_MSG_ID_MCU_START_MISSION_STATUS_CRC);
#else
    mavlink_mcu_start_mission_status_t *packet = (mavlink_mcu_start_mission_status_t *)msgbuf;
    packet->mode = mode;
    packet->status = status;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MCU_START_MISSION_STATUS, (const char *)packet, MAVLINK_MSG_ID_MCU_START_MISSION_STATUS_MIN_LEN, MAVLINK_MSG_ID_MCU_START_MISSION_STATUS_LEN, MAVLINK_MSG_ID_MCU_START_MISSION_STATUS_CRC);
#endif
}
#endif

#endif

// MESSAGE MCU_START_MISSION_STATUS UNPACKING


/**
 * @brief Get field mode from mcu_start_mission_status message
 *
 * @return  enum KERKES_MODE_TYPE. Specifies current mode. 
 */
static inline int16_t mavlink_msg_mcu_start_mission_status_get_mode(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  0);
}

/**
 * @brief Get field status from mcu_start_mission_status message
 *
 * @return  enum START_MISSION_STATUS_TYPE. Specifies current status of mission. 
 */
static inline int16_t mavlink_msg_mcu_start_mission_status_get_status(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  2);
}

/**
 * @brief Decode a mcu_start_mission_status message into a struct
 *
 * @param msg The message to decode
 * @param mcu_start_mission_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_mcu_start_mission_status_decode(const mavlink_message_t* msg, mavlink_mcu_start_mission_status_t* mcu_start_mission_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mcu_start_mission_status->mode = mavlink_msg_mcu_start_mission_status_get_mode(msg);
    mcu_start_mission_status->status = mavlink_msg_mcu_start_mission_status_get_status(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_MCU_START_MISSION_STATUS_LEN? msg->len : MAVLINK_MSG_ID_MCU_START_MISSION_STATUS_LEN;
        memset(mcu_start_mission_status, 0, MAVLINK_MSG_ID_MCU_START_MISSION_STATUS_LEN);
    memcpy(mcu_start_mission_status, _MAV_PAYLOAD(msg), len);
#endif
}
