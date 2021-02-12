#pragma once
// MESSAGE GCS_TO_MCU_GO_TO_LOCATION PACKING

#define MAVLINK_MSG_ID_GCS_TO_MCU_GO_TO_LOCATION 200

MAVPACKED(
typedef struct __mavlink_gcs_to_mcu_go_to_location_t {
 int32_t latitude; /*<  latitude of requested location */
 int32_t longitude; /*<  longitude of requested location */
 int32_t home_latitude; /*<  latitude of home location */
 int32_t home_longitude; /*<  longitude of home location */
}) mavlink_gcs_to_mcu_go_to_location_t;

#define MAVLINK_MSG_ID_GCS_TO_MCU_GO_TO_LOCATION_LEN 16
#define MAVLINK_MSG_ID_GCS_TO_MCU_GO_TO_LOCATION_MIN_LEN 16
#define MAVLINK_MSG_ID_200_LEN 16
#define MAVLINK_MSG_ID_200_MIN_LEN 16

#define MAVLINK_MSG_ID_GCS_TO_MCU_GO_TO_LOCATION_CRC 156
#define MAVLINK_MSG_ID_200_CRC 156



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_GCS_TO_MCU_GO_TO_LOCATION { \
    200, \
    "GCS_TO_MCU_GO_TO_LOCATION", \
    4, \
    {  { "latitude", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_gcs_to_mcu_go_to_location_t, latitude) }, \
         { "longitude", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_gcs_to_mcu_go_to_location_t, longitude) }, \
         { "home_latitude", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_gcs_to_mcu_go_to_location_t, home_latitude) }, \
         { "home_longitude", NULL, MAVLINK_TYPE_INT32_T, 0, 12, offsetof(mavlink_gcs_to_mcu_go_to_location_t, home_longitude) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_GCS_TO_MCU_GO_TO_LOCATION { \
    "GCS_TO_MCU_GO_TO_LOCATION", \
    4, \
    {  { "latitude", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_gcs_to_mcu_go_to_location_t, latitude) }, \
         { "longitude", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_gcs_to_mcu_go_to_location_t, longitude) }, \
         { "home_latitude", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_gcs_to_mcu_go_to_location_t, home_latitude) }, \
         { "home_longitude", NULL, MAVLINK_TYPE_INT32_T, 0, 12, offsetof(mavlink_gcs_to_mcu_go_to_location_t, home_longitude) }, \
         } \
}
#endif

/**
 * @brief Pack a gcs_to_mcu_go_to_location message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param latitude  latitude of requested location 
 * @param longitude  longitude of requested location 
 * @param home_latitude  latitude of home location 
 * @param home_longitude  longitude of home location 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gcs_to_mcu_go_to_location_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               int32_t latitude, int32_t longitude, int32_t home_latitude, int32_t home_longitude)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GCS_TO_MCU_GO_TO_LOCATION_LEN];
    _mav_put_int32_t(buf, 0, latitude);
    _mav_put_int32_t(buf, 4, longitude);
    _mav_put_int32_t(buf, 8, home_latitude);
    _mav_put_int32_t(buf, 12, home_longitude);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GCS_TO_MCU_GO_TO_LOCATION_LEN);
#else
    mavlink_gcs_to_mcu_go_to_location_t packet;
    packet.latitude = latitude;
    packet.longitude = longitude;
    packet.home_latitude = home_latitude;
    packet.home_longitude = home_longitude;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GCS_TO_MCU_GO_TO_LOCATION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GCS_TO_MCU_GO_TO_LOCATION;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_GCS_TO_MCU_GO_TO_LOCATION_MIN_LEN, MAVLINK_MSG_ID_GCS_TO_MCU_GO_TO_LOCATION_LEN, MAVLINK_MSG_ID_GCS_TO_MCU_GO_TO_LOCATION_CRC);
}

/**
 * @brief Pack a gcs_to_mcu_go_to_location message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param latitude  latitude of requested location 
 * @param longitude  longitude of requested location 
 * @param home_latitude  latitude of home location 
 * @param home_longitude  longitude of home location 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gcs_to_mcu_go_to_location_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   int32_t latitude,int32_t longitude,int32_t home_latitude,int32_t home_longitude)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GCS_TO_MCU_GO_TO_LOCATION_LEN];
    _mav_put_int32_t(buf, 0, latitude);
    _mav_put_int32_t(buf, 4, longitude);
    _mav_put_int32_t(buf, 8, home_latitude);
    _mav_put_int32_t(buf, 12, home_longitude);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GCS_TO_MCU_GO_TO_LOCATION_LEN);
#else
    mavlink_gcs_to_mcu_go_to_location_t packet;
    packet.latitude = latitude;
    packet.longitude = longitude;
    packet.home_latitude = home_latitude;
    packet.home_longitude = home_longitude;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GCS_TO_MCU_GO_TO_LOCATION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GCS_TO_MCU_GO_TO_LOCATION;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_GCS_TO_MCU_GO_TO_LOCATION_MIN_LEN, MAVLINK_MSG_ID_GCS_TO_MCU_GO_TO_LOCATION_LEN, MAVLINK_MSG_ID_GCS_TO_MCU_GO_TO_LOCATION_CRC);
}

/**
 * @brief Encode a gcs_to_mcu_go_to_location struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param gcs_to_mcu_go_to_location C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gcs_to_mcu_go_to_location_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_gcs_to_mcu_go_to_location_t* gcs_to_mcu_go_to_location)
{
    return mavlink_msg_gcs_to_mcu_go_to_location_pack(system_id, component_id, msg, gcs_to_mcu_go_to_location->latitude, gcs_to_mcu_go_to_location->longitude, gcs_to_mcu_go_to_location->home_latitude, gcs_to_mcu_go_to_location->home_longitude);
}

/**
 * @brief Encode a gcs_to_mcu_go_to_location struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param gcs_to_mcu_go_to_location C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gcs_to_mcu_go_to_location_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_gcs_to_mcu_go_to_location_t* gcs_to_mcu_go_to_location)
{
    return mavlink_msg_gcs_to_mcu_go_to_location_pack_chan(system_id, component_id, chan, msg, gcs_to_mcu_go_to_location->latitude, gcs_to_mcu_go_to_location->longitude, gcs_to_mcu_go_to_location->home_latitude, gcs_to_mcu_go_to_location->home_longitude);
}

/**
 * @brief Send a gcs_to_mcu_go_to_location message
 * @param chan MAVLink channel to send the message
 *
 * @param latitude  latitude of requested location 
 * @param longitude  longitude of requested location 
 * @param home_latitude  latitude of home location 
 * @param home_longitude  longitude of home location 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_gcs_to_mcu_go_to_location_send(mavlink_channel_t chan, int32_t latitude, int32_t longitude, int32_t home_latitude, int32_t home_longitude)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GCS_TO_MCU_GO_TO_LOCATION_LEN];
    _mav_put_int32_t(buf, 0, latitude);
    _mav_put_int32_t(buf, 4, longitude);
    _mav_put_int32_t(buf, 8, home_latitude);
    _mav_put_int32_t(buf, 12, home_longitude);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GCS_TO_MCU_GO_TO_LOCATION, buf, MAVLINK_MSG_ID_GCS_TO_MCU_GO_TO_LOCATION_MIN_LEN, MAVLINK_MSG_ID_GCS_TO_MCU_GO_TO_LOCATION_LEN, MAVLINK_MSG_ID_GCS_TO_MCU_GO_TO_LOCATION_CRC);
#else
    mavlink_gcs_to_mcu_go_to_location_t packet;
    packet.latitude = latitude;
    packet.longitude = longitude;
    packet.home_latitude = home_latitude;
    packet.home_longitude = home_longitude;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GCS_TO_MCU_GO_TO_LOCATION, (const char *)&packet, MAVLINK_MSG_ID_GCS_TO_MCU_GO_TO_LOCATION_MIN_LEN, MAVLINK_MSG_ID_GCS_TO_MCU_GO_TO_LOCATION_LEN, MAVLINK_MSG_ID_GCS_TO_MCU_GO_TO_LOCATION_CRC);
#endif
}

/**
 * @brief Send a gcs_to_mcu_go_to_location message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_gcs_to_mcu_go_to_location_send_struct(mavlink_channel_t chan, const mavlink_gcs_to_mcu_go_to_location_t* gcs_to_mcu_go_to_location)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_gcs_to_mcu_go_to_location_send(chan, gcs_to_mcu_go_to_location->latitude, gcs_to_mcu_go_to_location->longitude, gcs_to_mcu_go_to_location->home_latitude, gcs_to_mcu_go_to_location->home_longitude);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GCS_TO_MCU_GO_TO_LOCATION, (const char *)gcs_to_mcu_go_to_location, MAVLINK_MSG_ID_GCS_TO_MCU_GO_TO_LOCATION_MIN_LEN, MAVLINK_MSG_ID_GCS_TO_MCU_GO_TO_LOCATION_LEN, MAVLINK_MSG_ID_GCS_TO_MCU_GO_TO_LOCATION_CRC);
#endif
}

#if MAVLINK_MSG_ID_GCS_TO_MCU_GO_TO_LOCATION_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_gcs_to_mcu_go_to_location_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  int32_t latitude, int32_t longitude, int32_t home_latitude, int32_t home_longitude)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_int32_t(buf, 0, latitude);
    _mav_put_int32_t(buf, 4, longitude);
    _mav_put_int32_t(buf, 8, home_latitude);
    _mav_put_int32_t(buf, 12, home_longitude);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GCS_TO_MCU_GO_TO_LOCATION, buf, MAVLINK_MSG_ID_GCS_TO_MCU_GO_TO_LOCATION_MIN_LEN, MAVLINK_MSG_ID_GCS_TO_MCU_GO_TO_LOCATION_LEN, MAVLINK_MSG_ID_GCS_TO_MCU_GO_TO_LOCATION_CRC);
#else
    mavlink_gcs_to_mcu_go_to_location_t *packet = (mavlink_gcs_to_mcu_go_to_location_t *)msgbuf;
    packet->latitude = latitude;
    packet->longitude = longitude;
    packet->home_latitude = home_latitude;
    packet->home_longitude = home_longitude;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GCS_TO_MCU_GO_TO_LOCATION, (const char *)packet, MAVLINK_MSG_ID_GCS_TO_MCU_GO_TO_LOCATION_MIN_LEN, MAVLINK_MSG_ID_GCS_TO_MCU_GO_TO_LOCATION_LEN, MAVLINK_MSG_ID_GCS_TO_MCU_GO_TO_LOCATION_CRC);
#endif
}
#endif

#endif

// MESSAGE GCS_TO_MCU_GO_TO_LOCATION UNPACKING


/**
 * @brief Get field latitude from gcs_to_mcu_go_to_location message
 *
 * @return  latitude of requested location 
 */
static inline int32_t mavlink_msg_gcs_to_mcu_go_to_location_get_latitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  0);
}

/**
 * @brief Get field longitude from gcs_to_mcu_go_to_location message
 *
 * @return  longitude of requested location 
 */
static inline int32_t mavlink_msg_gcs_to_mcu_go_to_location_get_longitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  4);
}

/**
 * @brief Get field home_latitude from gcs_to_mcu_go_to_location message
 *
 * @return  latitude of home location 
 */
static inline int32_t mavlink_msg_gcs_to_mcu_go_to_location_get_home_latitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  8);
}

/**
 * @brief Get field home_longitude from gcs_to_mcu_go_to_location message
 *
 * @return  longitude of home location 
 */
static inline int32_t mavlink_msg_gcs_to_mcu_go_to_location_get_home_longitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  12);
}

/**
 * @brief Decode a gcs_to_mcu_go_to_location message into a struct
 *
 * @param msg The message to decode
 * @param gcs_to_mcu_go_to_location C-struct to decode the message contents into
 */
static inline void mavlink_msg_gcs_to_mcu_go_to_location_decode(const mavlink_message_t* msg, mavlink_gcs_to_mcu_go_to_location_t* gcs_to_mcu_go_to_location)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    gcs_to_mcu_go_to_location->latitude = mavlink_msg_gcs_to_mcu_go_to_location_get_latitude(msg);
    gcs_to_mcu_go_to_location->longitude = mavlink_msg_gcs_to_mcu_go_to_location_get_longitude(msg);
    gcs_to_mcu_go_to_location->home_latitude = mavlink_msg_gcs_to_mcu_go_to_location_get_home_latitude(msg);
    gcs_to_mcu_go_to_location->home_longitude = mavlink_msg_gcs_to_mcu_go_to_location_get_home_longitude(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_GCS_TO_MCU_GO_TO_LOCATION_LEN? msg->len : MAVLINK_MSG_ID_GCS_TO_MCU_GO_TO_LOCATION_LEN;
        memset(gcs_to_mcu_go_to_location, 0, MAVLINK_MSG_ID_GCS_TO_MCU_GO_TO_LOCATION_LEN);
    memcpy(gcs_to_mcu_go_to_location, _MAV_PAYLOAD(msg), len);
#endif
}
