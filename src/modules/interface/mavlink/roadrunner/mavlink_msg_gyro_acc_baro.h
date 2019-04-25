#pragma once
// MESSAGE GYRO_ACC_BARO PACKING

#define MAVLINK_MSG_ID_GYRO_ACC_BARO 1

MAVPACKED(
typedef struct __mavlink_gyro_acc_baro_t {
 float xacc; /*< [m/s/s] X acceleration*/
 float yacc; /*< [m/s/s] Y acceleration*/
 float zacc; /*< [m/s/s] Z acceleration*/
 float xgyro; /*< [rad/s] Angular speed around X axis*/
 float ygyro; /*< [rad/s] Angular speed around Y axis*/
 float zgyro; /*< [rad/s] Angular speed around Z axis*/
 float temperature; /*< [degC] Temperature*/
}) mavlink_gyro_acc_baro_t;

#define MAVLINK_MSG_ID_GYRO_ACC_BARO_LEN 28
#define MAVLINK_MSG_ID_GYRO_ACC_BARO_MIN_LEN 28
#define MAVLINK_MSG_ID_1_LEN 28
#define MAVLINK_MSG_ID_1_MIN_LEN 28

#define MAVLINK_MSG_ID_GYRO_ACC_BARO_CRC 107
#define MAVLINK_MSG_ID_1_CRC 107



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_GYRO_ACC_BARO { \
    1, \
    "GYRO_ACC_BARO", \
    7, \
    {  { "xacc", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_gyro_acc_baro_t, xacc) }, \
         { "yacc", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_gyro_acc_baro_t, yacc) }, \
         { "zacc", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_gyro_acc_baro_t, zacc) }, \
         { "xgyro", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_gyro_acc_baro_t, xgyro) }, \
         { "ygyro", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_gyro_acc_baro_t, ygyro) }, \
         { "zgyro", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_gyro_acc_baro_t, zgyro) }, \
         { "temperature", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_gyro_acc_baro_t, temperature) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_GYRO_ACC_BARO { \
    "GYRO_ACC_BARO", \
    7, \
    {  { "xacc", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_gyro_acc_baro_t, xacc) }, \
         { "yacc", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_gyro_acc_baro_t, yacc) }, \
         { "zacc", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_gyro_acc_baro_t, zacc) }, \
         { "xgyro", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_gyro_acc_baro_t, xgyro) }, \
         { "ygyro", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_gyro_acc_baro_t, ygyro) }, \
         { "zgyro", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_gyro_acc_baro_t, zgyro) }, \
         { "temperature", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_gyro_acc_baro_t, temperature) }, \
         } \
}
#endif

/**
 * @brief Pack a gyro_acc_baro message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param xacc [m/s/s] X acceleration
 * @param yacc [m/s/s] Y acceleration
 * @param zacc [m/s/s] Z acceleration
 * @param xgyro [rad/s] Angular speed around X axis
 * @param ygyro [rad/s] Angular speed around Y axis
 * @param zgyro [rad/s] Angular speed around Z axis
 * @param temperature [degC] Temperature
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gyro_acc_baro_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float xacc, float yacc, float zacc, float xgyro, float ygyro, float zgyro, float temperature)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GYRO_ACC_BARO_LEN];
    _mav_put_float(buf, 0, xacc);
    _mav_put_float(buf, 4, yacc);
    _mav_put_float(buf, 8, zacc);
    _mav_put_float(buf, 12, xgyro);
    _mav_put_float(buf, 16, ygyro);
    _mav_put_float(buf, 20, zgyro);
    _mav_put_float(buf, 24, temperature);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GYRO_ACC_BARO_LEN);
#else
    mavlink_gyro_acc_baro_t packet;
    packet.xacc = xacc;
    packet.yacc = yacc;
    packet.zacc = zacc;
    packet.xgyro = xgyro;
    packet.ygyro = ygyro;
    packet.zgyro = zgyro;
    packet.temperature = temperature;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GYRO_ACC_BARO_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GYRO_ACC_BARO;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_GYRO_ACC_BARO_MIN_LEN, MAVLINK_MSG_ID_GYRO_ACC_BARO_LEN, MAVLINK_MSG_ID_GYRO_ACC_BARO_CRC);
}

/**
 * @brief Pack a gyro_acc_baro message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param xacc [m/s/s] X acceleration
 * @param yacc [m/s/s] Y acceleration
 * @param zacc [m/s/s] Z acceleration
 * @param xgyro [rad/s] Angular speed around X axis
 * @param ygyro [rad/s] Angular speed around Y axis
 * @param zgyro [rad/s] Angular speed around Z axis
 * @param temperature [degC] Temperature
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gyro_acc_baro_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float xacc,float yacc,float zacc,float xgyro,float ygyro,float zgyro,float temperature)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GYRO_ACC_BARO_LEN];
    _mav_put_float(buf, 0, xacc);
    _mav_put_float(buf, 4, yacc);
    _mav_put_float(buf, 8, zacc);
    _mav_put_float(buf, 12, xgyro);
    _mav_put_float(buf, 16, ygyro);
    _mav_put_float(buf, 20, zgyro);
    _mav_put_float(buf, 24, temperature);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GYRO_ACC_BARO_LEN);
#else
    mavlink_gyro_acc_baro_t packet;
    packet.xacc = xacc;
    packet.yacc = yacc;
    packet.zacc = zacc;
    packet.xgyro = xgyro;
    packet.ygyro = ygyro;
    packet.zgyro = zgyro;
    packet.temperature = temperature;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GYRO_ACC_BARO_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GYRO_ACC_BARO;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_GYRO_ACC_BARO_MIN_LEN, MAVLINK_MSG_ID_GYRO_ACC_BARO_LEN, MAVLINK_MSG_ID_GYRO_ACC_BARO_CRC);
}

/**
 * @brief Encode a gyro_acc_baro struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param gyro_acc_baro C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gyro_acc_baro_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_gyro_acc_baro_t* gyro_acc_baro)
{
    return mavlink_msg_gyro_acc_baro_pack(system_id, component_id, msg, gyro_acc_baro->xacc, gyro_acc_baro->yacc, gyro_acc_baro->zacc, gyro_acc_baro->xgyro, gyro_acc_baro->ygyro, gyro_acc_baro->zgyro, gyro_acc_baro->temperature);
}

/**
 * @brief Encode a gyro_acc_baro struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param gyro_acc_baro C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gyro_acc_baro_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_gyro_acc_baro_t* gyro_acc_baro)
{
    return mavlink_msg_gyro_acc_baro_pack_chan(system_id, component_id, chan, msg, gyro_acc_baro->xacc, gyro_acc_baro->yacc, gyro_acc_baro->zacc, gyro_acc_baro->xgyro, gyro_acc_baro->ygyro, gyro_acc_baro->zgyro, gyro_acc_baro->temperature);
}

/**
 * @brief Send a gyro_acc_baro message
 * @param chan MAVLink channel to send the message
 *
 * @param xacc [m/s/s] X acceleration
 * @param yacc [m/s/s] Y acceleration
 * @param zacc [m/s/s] Z acceleration
 * @param xgyro [rad/s] Angular speed around X axis
 * @param ygyro [rad/s] Angular speed around Y axis
 * @param zgyro [rad/s] Angular speed around Z axis
 * @param temperature [degC] Temperature
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_gyro_acc_baro_send(mavlink_channel_t chan, float xacc, float yacc, float zacc, float xgyro, float ygyro, float zgyro, float temperature)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GYRO_ACC_BARO_LEN];
    _mav_put_float(buf, 0, xacc);
    _mav_put_float(buf, 4, yacc);
    _mav_put_float(buf, 8, zacc);
    _mav_put_float(buf, 12, xgyro);
    _mav_put_float(buf, 16, ygyro);
    _mav_put_float(buf, 20, zgyro);
    _mav_put_float(buf, 24, temperature);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GYRO_ACC_BARO, buf, MAVLINK_MSG_ID_GYRO_ACC_BARO_MIN_LEN, MAVLINK_MSG_ID_GYRO_ACC_BARO_LEN, MAVLINK_MSG_ID_GYRO_ACC_BARO_CRC);
#else
    mavlink_gyro_acc_baro_t packet;
    packet.xacc = xacc;
    packet.yacc = yacc;
    packet.zacc = zacc;
    packet.xgyro = xgyro;
    packet.ygyro = ygyro;
    packet.zgyro = zgyro;
    packet.temperature = temperature;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GYRO_ACC_BARO, (const char *)&packet, MAVLINK_MSG_ID_GYRO_ACC_BARO_MIN_LEN, MAVLINK_MSG_ID_GYRO_ACC_BARO_LEN, MAVLINK_MSG_ID_GYRO_ACC_BARO_CRC);
#endif
}

/**
 * @brief Send a gyro_acc_baro message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_gyro_acc_baro_send_struct(mavlink_channel_t chan, const mavlink_gyro_acc_baro_t* gyro_acc_baro)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_gyro_acc_baro_send(chan, gyro_acc_baro->xacc, gyro_acc_baro->yacc, gyro_acc_baro->zacc, gyro_acc_baro->xgyro, gyro_acc_baro->ygyro, gyro_acc_baro->zgyro, gyro_acc_baro->temperature);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GYRO_ACC_BARO, (const char *)gyro_acc_baro, MAVLINK_MSG_ID_GYRO_ACC_BARO_MIN_LEN, MAVLINK_MSG_ID_GYRO_ACC_BARO_LEN, MAVLINK_MSG_ID_GYRO_ACC_BARO_CRC);
#endif
}

#if MAVLINK_MSG_ID_GYRO_ACC_BARO_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_gyro_acc_baro_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float xacc, float yacc, float zacc, float xgyro, float ygyro, float zgyro, float temperature)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, xacc);
    _mav_put_float(buf, 4, yacc);
    _mav_put_float(buf, 8, zacc);
    _mav_put_float(buf, 12, xgyro);
    _mav_put_float(buf, 16, ygyro);
    _mav_put_float(buf, 20, zgyro);
    _mav_put_float(buf, 24, temperature);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GYRO_ACC_BARO, buf, MAVLINK_MSG_ID_GYRO_ACC_BARO_MIN_LEN, MAVLINK_MSG_ID_GYRO_ACC_BARO_LEN, MAVLINK_MSG_ID_GYRO_ACC_BARO_CRC);
#else
    mavlink_gyro_acc_baro_t *packet = (mavlink_gyro_acc_baro_t *)msgbuf;
    packet->xacc = xacc;
    packet->yacc = yacc;
    packet->zacc = zacc;
    packet->xgyro = xgyro;
    packet->ygyro = ygyro;
    packet->zgyro = zgyro;
    packet->temperature = temperature;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GYRO_ACC_BARO, (const char *)packet, MAVLINK_MSG_ID_GYRO_ACC_BARO_MIN_LEN, MAVLINK_MSG_ID_GYRO_ACC_BARO_LEN, MAVLINK_MSG_ID_GYRO_ACC_BARO_CRC);
#endif
}
#endif

#endif

// MESSAGE GYRO_ACC_BARO UNPACKING


/**
 * @brief Get field xacc from gyro_acc_baro message
 *
 * @return [m/s/s] X acceleration
 */
static inline float mavlink_msg_gyro_acc_baro_get_xacc(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field yacc from gyro_acc_baro message
 *
 * @return [m/s/s] Y acceleration
 */
static inline float mavlink_msg_gyro_acc_baro_get_yacc(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field zacc from gyro_acc_baro message
 *
 * @return [m/s/s] Z acceleration
 */
static inline float mavlink_msg_gyro_acc_baro_get_zacc(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field xgyro from gyro_acc_baro message
 *
 * @return [rad/s] Angular speed around X axis
 */
static inline float mavlink_msg_gyro_acc_baro_get_xgyro(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field ygyro from gyro_acc_baro message
 *
 * @return [rad/s] Angular speed around Y axis
 */
static inline float mavlink_msg_gyro_acc_baro_get_ygyro(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field zgyro from gyro_acc_baro message
 *
 * @return [rad/s] Angular speed around Z axis
 */
static inline float mavlink_msg_gyro_acc_baro_get_zgyro(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field temperature from gyro_acc_baro message
 *
 * @return [degC] Temperature
 */
static inline float mavlink_msg_gyro_acc_baro_get_temperature(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Decode a gyro_acc_baro message into a struct
 *
 * @param msg The message to decode
 * @param gyro_acc_baro C-struct to decode the message contents into
 */
static inline void mavlink_msg_gyro_acc_baro_decode(const mavlink_message_t* msg, mavlink_gyro_acc_baro_t* gyro_acc_baro)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    gyro_acc_baro->xacc = mavlink_msg_gyro_acc_baro_get_xacc(msg);
    gyro_acc_baro->yacc = mavlink_msg_gyro_acc_baro_get_yacc(msg);
    gyro_acc_baro->zacc = mavlink_msg_gyro_acc_baro_get_zacc(msg);
    gyro_acc_baro->xgyro = mavlink_msg_gyro_acc_baro_get_xgyro(msg);
    gyro_acc_baro->ygyro = mavlink_msg_gyro_acc_baro_get_ygyro(msg);
    gyro_acc_baro->zgyro = mavlink_msg_gyro_acc_baro_get_zgyro(msg);
    gyro_acc_baro->temperature = mavlink_msg_gyro_acc_baro_get_temperature(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_GYRO_ACC_BARO_LEN? msg->len : MAVLINK_MSG_ID_GYRO_ACC_BARO_LEN;
        memset(gyro_acc_baro, 0, MAVLINK_MSG_ID_GYRO_ACC_BARO_LEN);
    memcpy(gyro_acc_baro, _MAV_PAYLOAD(msg), len);
#endif
}
