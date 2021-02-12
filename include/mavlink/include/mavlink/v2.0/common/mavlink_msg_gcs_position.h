#pragma once
// MESSAGE GCS_POSITION PACKING

#define MAVLINK_MSG_ID_GCS_POSITION 210

MAVPACKED(
typedef struct __mavlink_gcs_position_t {
 int32_t gcs_latitude; /*<  latitude of gcs in degrees * 1E7*/
 int32_t gcs_longitude; /*<  longitude of gcs in degrees * 1E7*/
 uint8_t is_valid; /*<  specifies whether gcs position is valid*/
}) mavlink_gcs_position_t;

#define MAVLINK_MSG_ID_GCS_POSITION_LEN 9
#define MAVLINK_MSG_ID_GCS_POSITION_MIN_LEN 9
#define MAVLINK_MSG_ID_210_LEN 9
#define MAVLINK_MSG_ID_210_MIN_LEN 9

#define MAVLINK_MSG_ID_GCS_POSITION_CRC 241
#define MAVLINK_MSG_ID_210_CRC 241



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_GCS_POSITION { \
    210, \
    "GCS_POSITION", \
    3, \
    {  { "gcs_latitude", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_gcs_position_t, gcs_latitude) }, \
         { "gcs_longitude", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_gcs_position_t, gcs_longitude) }, \
         { "is_valid", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_gcs_position_t, is_valid) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_GCS_POSITION { \
    "GCS_POSITION", \
    3, \
    {  { "gcs_latitude", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_gcs_position_t, gcs_latitude) }, \
         { "gcs_longitude", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_gcs_position_t, gcs_longitude) }, \
         { "is_valid", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_gcs_position_t, is_valid) }, \
         } \
}
#endif

/**
 * @brief Pack a gcs_position message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param gcs_latitude  latitude of gcs in degrees * 1E7
 * @param gcs_longitude  longitude of gcs in degrees * 1E7
 * @param is_valid  specifies whether gcs position is valid
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gcs_position_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               int32_t gcs_latitude, int32_t gcs_longitude, uint8_t is_valid)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GCS_POSITION_LEN];
    _mav_put_int32_t(buf, 0, gcs_latitude);
    _mav_put_int32_t(buf, 4, gcs_longitude);
    _mav_put_uint8_t(buf, 8, is_valid);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GCS_POSITION_LEN);
#else
    mavlink_gcs_position_t packet;
    packet.gcs_latitude = gcs_latitude;
    packet.gcs_longitude = gcs_longitude;
    packet.is_valid = is_valid;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GCS_POSITION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GCS_POSITION;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_GCS_POSITION_MIN_LEN, MAVLINK_MSG_ID_GCS_POSITION_LEN, MAVLINK_MSG_ID_GCS_POSITION_CRC);
}

/**
 * @brief Pack a gcs_position message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param gcs_latitude  latitude of gcs in degrees * 1E7
 * @param gcs_longitude  longitude of gcs in degrees * 1E7
 * @param is_valid  specifies whether gcs position is valid
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gcs_position_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   int32_t gcs_latitude,int32_t gcs_longitude,uint8_t is_valid)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GCS_POSITION_LEN];
    _mav_put_int32_t(buf, 0, gcs_latitude);
    _mav_put_int32_t(buf, 4, gcs_longitude);
    _mav_put_uint8_t(buf, 8, is_valid);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GCS_POSITION_LEN);
#else
    mavlink_gcs_position_t packet;
    packet.gcs_latitude = gcs_latitude;
    packet.gcs_longitude = gcs_longitude;
    packet.is_valid = is_valid;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GCS_POSITION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GCS_POSITION;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_GCS_POSITION_MIN_LEN, MAVLINK_MSG_ID_GCS_POSITION_LEN, MAVLINK_MSG_ID_GCS_POSITION_CRC);
}

/**
 * @brief Encode a gcs_position struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param gcs_position C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gcs_position_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_gcs_position_t* gcs_position)
{
    return mavlink_msg_gcs_position_pack(system_id, component_id, msg, gcs_position->gcs_latitude, gcs_position->gcs_longitude, gcs_position->is_valid);
}

/**
 * @brief Encode a gcs_position struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param gcs_position C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gcs_position_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_gcs_position_t* gcs_position)
{
    return mavlink_msg_gcs_position_pack_chan(system_id, component_id, chan, msg, gcs_position->gcs_latitude, gcs_position->gcs_longitude, gcs_position->is_valid);
}

/**
 * @brief Send a gcs_position message
 * @param chan MAVLink channel to send the message
 *
 * @param gcs_latitude  latitude of gcs in degrees * 1E7
 * @param gcs_longitude  longitude of gcs in degrees * 1E7
 * @param is_valid  specifies whether gcs position is valid
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_gcs_position_send(mavlink_channel_t chan, int32_t gcs_latitude, int32_t gcs_longitude, uint8_t is_valid)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GCS_POSITION_LEN];
    _mav_put_int32_t(buf, 0, gcs_latitude);
    _mav_put_int32_t(buf, 4, gcs_longitude);
    _mav_put_uint8_t(buf, 8, is_valid);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GCS_POSITION, buf, MAVLINK_MSG_ID_GCS_POSITION_MIN_LEN, MAVLINK_MSG_ID_GCS_POSITION_LEN, MAVLINK_MSG_ID_GCS_POSITION_CRC);
#else
    mavlink_gcs_position_t packet;
    packet.gcs_latitude = gcs_latitude;
    packet.gcs_longitude = gcs_longitude;
    packet.is_valid = is_valid;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GCS_POSITION, (const char *)&packet, MAVLINK_MSG_ID_GCS_POSITION_MIN_LEN, MAVLINK_MSG_ID_GCS_POSITION_LEN, MAVLINK_MSG_ID_GCS_POSITION_CRC);
#endif
}

/**
 * @brief Send a gcs_position message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_gcs_position_send_struct(mavlink_channel_t chan, const mavlink_gcs_position_t* gcs_position)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_gcs_position_send(chan, gcs_position->gcs_latitude, gcs_position->gcs_longitude, gcs_position->is_valid);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GCS_POSITION, (const char *)gcs_position, MAVLINK_MSG_ID_GCS_POSITION_MIN_LEN, MAVLINK_MSG_ID_GCS_POSITION_LEN, MAVLINK_MSG_ID_GCS_POSITION_CRC);
#endif
}

#if MAVLINK_MSG_ID_GCS_POSITION_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_gcs_position_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  int32_t gcs_latitude, int32_t gcs_longitude, uint8_t is_valid)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_int32_t(buf, 0, gcs_latitude);
    _mav_put_int32_t(buf, 4, gcs_longitude);
    _mav_put_uint8_t(buf, 8, is_valid);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GCS_POSITION, buf, MAVLINK_MSG_ID_GCS_POSITION_MIN_LEN, MAVLINK_MSG_ID_GCS_POSITION_LEN, MAVLINK_MSG_ID_GCS_POSITION_CRC);
#else
    mavlink_gcs_position_t *packet = (mavlink_gcs_position_t *)msgbuf;
    packet->gcs_latitude = gcs_latitude;
    packet->gcs_longitude = gcs_longitude;
    packet->is_valid = is_valid;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GCS_POSITION, (const char *)packet, MAVLINK_MSG_ID_GCS_POSITION_MIN_LEN, MAVLINK_MSG_ID_GCS_POSITION_LEN, MAVLINK_MSG_ID_GCS_POSITION_CRC);
#endif
}
#endif

#endif

// MESSAGE GCS_POSITION UNPACKING


/**
 * @brief Get field gcs_latitude from gcs_position message
 *
 * @return  latitude of gcs in degrees * 1E7
 */
static inline int32_t mavlink_msg_gcs_position_get_gcs_latitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  0);
}

/**
 * @brief Get field gcs_longitude from gcs_position message
 *
 * @return  longitude of gcs in degrees * 1E7
 */
static inline int32_t mavlink_msg_gcs_position_get_gcs_longitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  4);
}

/**
 * @brief Get field is_valid from gcs_position message
 *
 * @return  specifies whether gcs position is valid
 */
static inline uint8_t mavlink_msg_gcs_position_get_is_valid(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  8);
}

/**
 * @brief Decode a gcs_position message into a struct
 *
 * @param msg The message to decode
 * @param gcs_position C-struct to decode the message contents into
 */
static inline void mavlink_msg_gcs_position_decode(const mavlink_message_t* msg, mavlink_gcs_position_t* gcs_position)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    gcs_position->gcs_latitude = mavlink_msg_gcs_position_get_gcs_latitude(msg);
    gcs_position->gcs_longitude = mavlink_msg_gcs_position_get_gcs_longitude(msg);
    gcs_position->is_valid = mavlink_msg_gcs_position_get_is_valid(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_GCS_POSITION_LEN? msg->len : MAVLINK_MSG_ID_GCS_POSITION_LEN;
        memset(gcs_position, 0, MAVLINK_MSG_ID_GCS_POSITION_LEN);
    memcpy(gcs_position, _MAV_PAYLOAD(msg), len);
#endif
}
