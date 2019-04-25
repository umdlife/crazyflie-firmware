#pragma once
// MESSAGE QUATERNION PACKING

#define MAVLINK_MSG_ID_QUATERNION 3

MAVPACKED(
typedef struct __mavlink_quaternion_t {
 float qx; /*<  X quaternion*/
 float qy; /*<  Y quaternion*/
 float qz; /*<  Z quaternion*/
 float qw; /*<  W quaternion*/
 float Vqx; /*<  X quaternion covariance*/
 float Vqy; /*<  Y quaternion covariance*/
 float Vqz; /*<  Z quaternion covariance*/
 float Vqw; /*<  W quaternion covariance*/
}) mavlink_quaternion_t;

#define MAVLINK_MSG_ID_QUATERNION_LEN 32
#define MAVLINK_MSG_ID_QUATERNION_MIN_LEN 32
#define MAVLINK_MSG_ID_3_LEN 32
#define MAVLINK_MSG_ID_3_MIN_LEN 32

#define MAVLINK_MSG_ID_QUATERNION_CRC 182
#define MAVLINK_MSG_ID_3_CRC 182



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_QUATERNION { \
    3, \
    "QUATERNION", \
    8, \
    {  { "qx", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_quaternion_t, qx) }, \
         { "qy", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_quaternion_t, qy) }, \
         { "qz", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_quaternion_t, qz) }, \
         { "qw", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_quaternion_t, qw) }, \
         { "Vqx", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_quaternion_t, Vqx) }, \
         { "Vqy", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_quaternion_t, Vqy) }, \
         { "Vqz", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_quaternion_t, Vqz) }, \
         { "Vqw", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_quaternion_t, Vqw) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_QUATERNION { \
    "QUATERNION", \
    8, \
    {  { "qx", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_quaternion_t, qx) }, \
         { "qy", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_quaternion_t, qy) }, \
         { "qz", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_quaternion_t, qz) }, \
         { "qw", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_quaternion_t, qw) }, \
         { "Vqx", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_quaternion_t, Vqx) }, \
         { "Vqy", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_quaternion_t, Vqy) }, \
         { "Vqz", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_quaternion_t, Vqz) }, \
         { "Vqw", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_quaternion_t, Vqw) }, \
         } \
}
#endif

/**
 * @brief Pack a quaternion message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param qx  X quaternion
 * @param qy  Y quaternion
 * @param qz  Z quaternion
 * @param qw  W quaternion
 * @param Vqx  X quaternion covariance
 * @param Vqy  Y quaternion covariance
 * @param Vqz  Z quaternion covariance
 * @param Vqw  W quaternion covariance
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_quaternion_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float qx, float qy, float qz, float qw, float Vqx, float Vqy, float Vqz, float Vqw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_QUATERNION_LEN];
    _mav_put_float(buf, 0, qx);
    _mav_put_float(buf, 4, qy);
    _mav_put_float(buf, 8, qz);
    _mav_put_float(buf, 12, qw);
    _mav_put_float(buf, 16, Vqx);
    _mav_put_float(buf, 20, Vqy);
    _mav_put_float(buf, 24, Vqz);
    _mav_put_float(buf, 28, Vqw);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_QUATERNION_LEN);
#else
    mavlink_quaternion_t packet;
    packet.qx = qx;
    packet.qy = qy;
    packet.qz = qz;
    packet.qw = qw;
    packet.Vqx = Vqx;
    packet.Vqy = Vqy;
    packet.Vqz = Vqz;
    packet.Vqw = Vqw;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_QUATERNION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_QUATERNION;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_QUATERNION_MIN_LEN, MAVLINK_MSG_ID_QUATERNION_LEN, MAVLINK_MSG_ID_QUATERNION_CRC);
}

/**
 * @brief Pack a quaternion message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param qx  X quaternion
 * @param qy  Y quaternion
 * @param qz  Z quaternion
 * @param qw  W quaternion
 * @param Vqx  X quaternion covariance
 * @param Vqy  Y quaternion covariance
 * @param Vqz  Z quaternion covariance
 * @param Vqw  W quaternion covariance
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_quaternion_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float qx,float qy,float qz,float qw,float Vqx,float Vqy,float Vqz,float Vqw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_QUATERNION_LEN];
    _mav_put_float(buf, 0, qx);
    _mav_put_float(buf, 4, qy);
    _mav_put_float(buf, 8, qz);
    _mav_put_float(buf, 12, qw);
    _mav_put_float(buf, 16, Vqx);
    _mav_put_float(buf, 20, Vqy);
    _mav_put_float(buf, 24, Vqz);
    _mav_put_float(buf, 28, Vqw);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_QUATERNION_LEN);
#else
    mavlink_quaternion_t packet;
    packet.qx = qx;
    packet.qy = qy;
    packet.qz = qz;
    packet.qw = qw;
    packet.Vqx = Vqx;
    packet.Vqy = Vqy;
    packet.Vqz = Vqz;
    packet.Vqw = Vqw;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_QUATERNION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_QUATERNION;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_QUATERNION_MIN_LEN, MAVLINK_MSG_ID_QUATERNION_LEN, MAVLINK_MSG_ID_QUATERNION_CRC);
}

/**
 * @brief Encode a quaternion struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param quaternion C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_quaternion_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_quaternion_t* quaternion)
{
    return mavlink_msg_quaternion_pack(system_id, component_id, msg, quaternion->qx, quaternion->qy, quaternion->qz, quaternion->qw, quaternion->Vqx, quaternion->Vqy, quaternion->Vqz, quaternion->Vqw);
}

/**
 * @brief Encode a quaternion struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param quaternion C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_quaternion_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_quaternion_t* quaternion)
{
    return mavlink_msg_quaternion_pack_chan(system_id, component_id, chan, msg, quaternion->qx, quaternion->qy, quaternion->qz, quaternion->qw, quaternion->Vqx, quaternion->Vqy, quaternion->Vqz, quaternion->Vqw);
}

/**
 * @brief Send a quaternion message
 * @param chan MAVLink channel to send the message
 *
 * @param qx  X quaternion
 * @param qy  Y quaternion
 * @param qz  Z quaternion
 * @param qw  W quaternion
 * @param Vqx  X quaternion covariance
 * @param Vqy  Y quaternion covariance
 * @param Vqz  Z quaternion covariance
 * @param Vqw  W quaternion covariance
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_quaternion_send(mavlink_channel_t chan, float qx, float qy, float qz, float qw, float Vqx, float Vqy, float Vqz, float Vqw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_QUATERNION_LEN];
    _mav_put_float(buf, 0, qx);
    _mav_put_float(buf, 4, qy);
    _mav_put_float(buf, 8, qz);
    _mav_put_float(buf, 12, qw);
    _mav_put_float(buf, 16, Vqx);
    _mav_put_float(buf, 20, Vqy);
    _mav_put_float(buf, 24, Vqz);
    _mav_put_float(buf, 28, Vqw);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_QUATERNION, buf, MAVLINK_MSG_ID_QUATERNION_MIN_LEN, MAVLINK_MSG_ID_QUATERNION_LEN, MAVLINK_MSG_ID_QUATERNION_CRC);
#else
    mavlink_quaternion_t packet;
    packet.qx = qx;
    packet.qy = qy;
    packet.qz = qz;
    packet.qw = qw;
    packet.Vqx = Vqx;
    packet.Vqy = Vqy;
    packet.Vqz = Vqz;
    packet.Vqw = Vqw;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_QUATERNION, (const char *)&packet, MAVLINK_MSG_ID_QUATERNION_MIN_LEN, MAVLINK_MSG_ID_QUATERNION_LEN, MAVLINK_MSG_ID_QUATERNION_CRC);
#endif
}

/**
 * @brief Send a quaternion message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_quaternion_send_struct(mavlink_channel_t chan, const mavlink_quaternion_t* quaternion)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_quaternion_send(chan, quaternion->qx, quaternion->qy, quaternion->qz, quaternion->qw, quaternion->Vqx, quaternion->Vqy, quaternion->Vqz, quaternion->Vqw);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_QUATERNION, (const char *)quaternion, MAVLINK_MSG_ID_QUATERNION_MIN_LEN, MAVLINK_MSG_ID_QUATERNION_LEN, MAVLINK_MSG_ID_QUATERNION_CRC);
#endif
}

#if MAVLINK_MSG_ID_QUATERNION_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_quaternion_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float qx, float qy, float qz, float qw, float Vqx, float Vqy, float Vqz, float Vqw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, qx);
    _mav_put_float(buf, 4, qy);
    _mav_put_float(buf, 8, qz);
    _mav_put_float(buf, 12, qw);
    _mav_put_float(buf, 16, Vqx);
    _mav_put_float(buf, 20, Vqy);
    _mav_put_float(buf, 24, Vqz);
    _mav_put_float(buf, 28, Vqw);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_QUATERNION, buf, MAVLINK_MSG_ID_QUATERNION_MIN_LEN, MAVLINK_MSG_ID_QUATERNION_LEN, MAVLINK_MSG_ID_QUATERNION_CRC);
#else
    mavlink_quaternion_t *packet = (mavlink_quaternion_t *)msgbuf;
    packet->qx = qx;
    packet->qy = qy;
    packet->qz = qz;
    packet->qw = qw;
    packet->Vqx = Vqx;
    packet->Vqy = Vqy;
    packet->Vqz = Vqz;
    packet->Vqw = Vqw;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_QUATERNION, (const char *)packet, MAVLINK_MSG_ID_QUATERNION_MIN_LEN, MAVLINK_MSG_ID_QUATERNION_LEN, MAVLINK_MSG_ID_QUATERNION_CRC);
#endif
}
#endif

#endif

// MESSAGE QUATERNION UNPACKING


/**
 * @brief Get field qx from quaternion message
 *
 * @return  X quaternion
 */
static inline float mavlink_msg_quaternion_get_qx(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field qy from quaternion message
 *
 * @return  Y quaternion
 */
static inline float mavlink_msg_quaternion_get_qy(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field qz from quaternion message
 *
 * @return  Z quaternion
 */
static inline float mavlink_msg_quaternion_get_qz(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field qw from quaternion message
 *
 * @return  W quaternion
 */
static inline float mavlink_msg_quaternion_get_qw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field Vqx from quaternion message
 *
 * @return  X quaternion covariance
 */
static inline float mavlink_msg_quaternion_get_Vqx(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field Vqy from quaternion message
 *
 * @return  Y quaternion covariance
 */
static inline float mavlink_msg_quaternion_get_Vqy(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field Vqz from quaternion message
 *
 * @return  Z quaternion covariance
 */
static inline float mavlink_msg_quaternion_get_Vqz(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field Vqw from quaternion message
 *
 * @return  W quaternion covariance
 */
static inline float mavlink_msg_quaternion_get_Vqw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Decode a quaternion message into a struct
 *
 * @param msg The message to decode
 * @param quaternion C-struct to decode the message contents into
 */
static inline void mavlink_msg_quaternion_decode(const mavlink_message_t* msg, mavlink_quaternion_t* quaternion)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    quaternion->qx = mavlink_msg_quaternion_get_qx(msg);
    quaternion->qy = mavlink_msg_quaternion_get_qy(msg);
    quaternion->qz = mavlink_msg_quaternion_get_qz(msg);
    quaternion->qw = mavlink_msg_quaternion_get_qw(msg);
    quaternion->Vqx = mavlink_msg_quaternion_get_Vqx(msg);
    quaternion->Vqy = mavlink_msg_quaternion_get_Vqy(msg);
    quaternion->Vqz = mavlink_msg_quaternion_get_Vqz(msg);
    quaternion->Vqw = mavlink_msg_quaternion_get_Vqw(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_QUATERNION_LEN? msg->len : MAVLINK_MSG_ID_QUATERNION_LEN;
        memset(quaternion, 0, MAVLINK_MSG_ID_QUATERNION_LEN);
    memcpy(quaternion, _MAV_PAYLOAD(msg), len);
#endif
}
