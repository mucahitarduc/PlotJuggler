#pragma once
// MESSAGE AS5048A_MAGNETIC_ROTARY_ENCODER PACKING

#define MAVLINK_MSG_ID_AS5048A_MAGNETIC_ROTARY_ENCODER 201

MAVPACKED(
typedef struct __mavlink_as5048a_magnetic_rotary_encoder_t {
 float angle[3]; /*<  Angle output value. angle[0] is yaw, angle[1] is pitch and angle[2] empty. */
}) mavlink_as5048a_magnetic_rotary_encoder_t;

#define MAVLINK_MSG_ID_AS5048A_MAGNETIC_ROTARY_ENCODER_LEN 12
#define MAVLINK_MSG_ID_AS5048A_MAGNETIC_ROTARY_ENCODER_MIN_LEN 12
#define MAVLINK_MSG_ID_201_LEN 12
#define MAVLINK_MSG_ID_201_MIN_LEN 12

#define MAVLINK_MSG_ID_AS5048A_MAGNETIC_ROTARY_ENCODER_CRC 85
#define MAVLINK_MSG_ID_201_CRC 85

#define MAVLINK_MSG_AS5048A_MAGNETIC_ROTARY_ENCODER_FIELD_ANGLE_LEN 3

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_AS5048A_MAGNETIC_ROTARY_ENCODER { \
    201, \
    "AS5048A_MAGNETIC_ROTARY_ENCODER", \
    1, \
    {  { "angle", NULL, MAVLINK_TYPE_FLOAT, 3, 0, offsetof(mavlink_as5048a_magnetic_rotary_encoder_t, angle) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_AS5048A_MAGNETIC_ROTARY_ENCODER { \
    "AS5048A_MAGNETIC_ROTARY_ENCODER", \
    1, \
    {  { "angle", NULL, MAVLINK_TYPE_FLOAT, 3, 0, offsetof(mavlink_as5048a_magnetic_rotary_encoder_t, angle) }, \
         } \
}
#endif

/**
 * @brief Pack a as5048a_magnetic_rotary_encoder message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param angle  Angle output value. angle[0] is yaw, angle[1] is pitch and angle[2] empty. 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_as5048a_magnetic_rotary_encoder_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               const float *angle)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_AS5048A_MAGNETIC_ROTARY_ENCODER_LEN];

    _mav_put_float_array(buf, 0, angle, 3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_AS5048A_MAGNETIC_ROTARY_ENCODER_LEN);
#else
    mavlink_as5048a_magnetic_rotary_encoder_t packet;

    mav_array_memcpy(packet.angle, angle, sizeof(float)*3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_AS5048A_MAGNETIC_ROTARY_ENCODER_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_AS5048A_MAGNETIC_ROTARY_ENCODER;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_AS5048A_MAGNETIC_ROTARY_ENCODER_MIN_LEN, MAVLINK_MSG_ID_AS5048A_MAGNETIC_ROTARY_ENCODER_LEN, MAVLINK_MSG_ID_AS5048A_MAGNETIC_ROTARY_ENCODER_CRC);
}

/**
 * @brief Pack a as5048a_magnetic_rotary_encoder message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param angle  Angle output value. angle[0] is yaw, angle[1] is pitch and angle[2] empty. 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_as5048a_magnetic_rotary_encoder_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   const float *angle)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_AS5048A_MAGNETIC_ROTARY_ENCODER_LEN];

    _mav_put_float_array(buf, 0, angle, 3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_AS5048A_MAGNETIC_ROTARY_ENCODER_LEN);
#else
    mavlink_as5048a_magnetic_rotary_encoder_t packet;

    mav_array_memcpy(packet.angle, angle, sizeof(float)*3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_AS5048A_MAGNETIC_ROTARY_ENCODER_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_AS5048A_MAGNETIC_ROTARY_ENCODER;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_AS5048A_MAGNETIC_ROTARY_ENCODER_MIN_LEN, MAVLINK_MSG_ID_AS5048A_MAGNETIC_ROTARY_ENCODER_LEN, MAVLINK_MSG_ID_AS5048A_MAGNETIC_ROTARY_ENCODER_CRC);
}

/**
 * @brief Encode a as5048a_magnetic_rotary_encoder struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param as5048a_magnetic_rotary_encoder C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_as5048a_magnetic_rotary_encoder_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_as5048a_magnetic_rotary_encoder_t* as5048a_magnetic_rotary_encoder)
{
    return mavlink_msg_as5048a_magnetic_rotary_encoder_pack(system_id, component_id, msg, as5048a_magnetic_rotary_encoder->angle);
}

/**
 * @brief Encode a as5048a_magnetic_rotary_encoder struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param as5048a_magnetic_rotary_encoder C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_as5048a_magnetic_rotary_encoder_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_as5048a_magnetic_rotary_encoder_t* as5048a_magnetic_rotary_encoder)
{
    return mavlink_msg_as5048a_magnetic_rotary_encoder_pack_chan(system_id, component_id, chan, msg, as5048a_magnetic_rotary_encoder->angle);
}

/**
 * @brief Send a as5048a_magnetic_rotary_encoder message
 * @param chan MAVLink channel to send the message
 *
 * @param angle  Angle output value. angle[0] is yaw, angle[1] is pitch and angle[2] empty. 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_as5048a_magnetic_rotary_encoder_send(mavlink_channel_t chan, const float *angle)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_AS5048A_MAGNETIC_ROTARY_ENCODER_LEN];

    _mav_put_float_array(buf, 0, angle, 3);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AS5048A_MAGNETIC_ROTARY_ENCODER, buf, MAVLINK_MSG_ID_AS5048A_MAGNETIC_ROTARY_ENCODER_MIN_LEN, MAVLINK_MSG_ID_AS5048A_MAGNETIC_ROTARY_ENCODER_LEN, MAVLINK_MSG_ID_AS5048A_MAGNETIC_ROTARY_ENCODER_CRC);
#else
    mavlink_as5048a_magnetic_rotary_encoder_t packet;

    mav_array_memcpy(packet.angle, angle, sizeof(float)*3);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AS5048A_MAGNETIC_ROTARY_ENCODER, (const char *)&packet, MAVLINK_MSG_ID_AS5048A_MAGNETIC_ROTARY_ENCODER_MIN_LEN, MAVLINK_MSG_ID_AS5048A_MAGNETIC_ROTARY_ENCODER_LEN, MAVLINK_MSG_ID_AS5048A_MAGNETIC_ROTARY_ENCODER_CRC);
#endif
}

/**
 * @brief Send a as5048a_magnetic_rotary_encoder message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_as5048a_magnetic_rotary_encoder_send_struct(mavlink_channel_t chan, const mavlink_as5048a_magnetic_rotary_encoder_t* as5048a_magnetic_rotary_encoder)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_as5048a_magnetic_rotary_encoder_send(chan, as5048a_magnetic_rotary_encoder->angle);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AS5048A_MAGNETIC_ROTARY_ENCODER, (const char *)as5048a_magnetic_rotary_encoder, MAVLINK_MSG_ID_AS5048A_MAGNETIC_ROTARY_ENCODER_MIN_LEN, MAVLINK_MSG_ID_AS5048A_MAGNETIC_ROTARY_ENCODER_LEN, MAVLINK_MSG_ID_AS5048A_MAGNETIC_ROTARY_ENCODER_CRC);
#endif
}

#if MAVLINK_MSG_ID_AS5048A_MAGNETIC_ROTARY_ENCODER_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_as5048a_magnetic_rotary_encoder_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  const float *angle)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;

    _mav_put_float_array(buf, 0, angle, 3);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AS5048A_MAGNETIC_ROTARY_ENCODER, buf, MAVLINK_MSG_ID_AS5048A_MAGNETIC_ROTARY_ENCODER_MIN_LEN, MAVLINK_MSG_ID_AS5048A_MAGNETIC_ROTARY_ENCODER_LEN, MAVLINK_MSG_ID_AS5048A_MAGNETIC_ROTARY_ENCODER_CRC);
#else
    mavlink_as5048a_magnetic_rotary_encoder_t *packet = (mavlink_as5048a_magnetic_rotary_encoder_t *)msgbuf;

    mav_array_memcpy(packet->angle, angle, sizeof(float)*3);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AS5048A_MAGNETIC_ROTARY_ENCODER, (const char *)packet, MAVLINK_MSG_ID_AS5048A_MAGNETIC_ROTARY_ENCODER_MIN_LEN, MAVLINK_MSG_ID_AS5048A_MAGNETIC_ROTARY_ENCODER_LEN, MAVLINK_MSG_ID_AS5048A_MAGNETIC_ROTARY_ENCODER_CRC);
#endif
}
#endif

#endif

// MESSAGE AS5048A_MAGNETIC_ROTARY_ENCODER UNPACKING


/**
 * @brief Get field angle from as5048a_magnetic_rotary_encoder message
 *
 * @return  Angle output value. angle[0] is yaw, angle[1] is pitch and angle[2] empty. 
 */
static inline uint16_t mavlink_msg_as5048a_magnetic_rotary_encoder_get_angle(const mavlink_message_t* msg, float *angle)
{
    return _MAV_RETURN_float_array(msg, angle, 3,  0);
}

/**
 * @brief Decode a as5048a_magnetic_rotary_encoder message into a struct
 *
 * @param msg The message to decode
 * @param as5048a_magnetic_rotary_encoder C-struct to decode the message contents into
 */
static inline void mavlink_msg_as5048a_magnetic_rotary_encoder_decode(const mavlink_message_t* msg, mavlink_as5048a_magnetic_rotary_encoder_t* as5048a_magnetic_rotary_encoder)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_as5048a_magnetic_rotary_encoder_get_angle(msg, as5048a_magnetic_rotary_encoder->angle);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_AS5048A_MAGNETIC_ROTARY_ENCODER_LEN? msg->len : MAVLINK_MSG_ID_AS5048A_MAGNETIC_ROTARY_ENCODER_LEN;
        memset(as5048a_magnetic_rotary_encoder, 0, MAVLINK_MSG_ID_AS5048A_MAGNETIC_ROTARY_ENCODER_LEN);
    memcpy(as5048a_magnetic_rotary_encoder, _MAV_PAYLOAD(msg), len);
#endif
}
