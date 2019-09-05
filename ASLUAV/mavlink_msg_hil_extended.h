#pragma once
// MESSAGE HIL_EXTENDED PACKING

#define MAVLINK_MSG_ID_HIL_EXTENDED 216

MAVPACKED(
typedef struct __mavlink_hil_extended_t {
 uint64_t timestamp; /*< [us] Timestamp*/
 float var1; /*<  Variable 1*/
 float var2; /*<  Variable 2*/
 float var3; /*<  Variable 3*/
 float var4; /*<  Variable 4*/
 float var5; /*<  Variable 5*/
 float var6; /*<  Variable 6*/
 float var7; /*<  Variable 7*/
 float var8; /*<  Variable 8*/
}) mavlink_hil_extended_t;

#define MAVLINK_MSG_ID_HIL_EXTENDED_LEN 40
#define MAVLINK_MSG_ID_HIL_EXTENDED_MIN_LEN 40
#define MAVLINK_MSG_ID_216_LEN 40
#define MAVLINK_MSG_ID_216_MIN_LEN 40

#define MAVLINK_MSG_ID_HIL_EXTENDED_CRC 19
#define MAVLINK_MSG_ID_216_CRC 19



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_HIL_EXTENDED { \
    216, \
    "HIL_EXTENDED", \
    9, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_hil_extended_t, timestamp) }, \
         { "var1", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_hil_extended_t, var1) }, \
         { "var2", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_hil_extended_t, var2) }, \
         { "var3", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_hil_extended_t, var3) }, \
         { "var4", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_hil_extended_t, var4) }, \
         { "var5", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_hil_extended_t, var5) }, \
         { "var6", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_hil_extended_t, var6) }, \
         { "var7", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_hil_extended_t, var7) }, \
         { "var8", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_hil_extended_t, var8) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_HIL_EXTENDED { \
    "HIL_EXTENDED", \
    9, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_hil_extended_t, timestamp) }, \
         { "var1", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_hil_extended_t, var1) }, \
         { "var2", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_hil_extended_t, var2) }, \
         { "var3", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_hil_extended_t, var3) }, \
         { "var4", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_hil_extended_t, var4) }, \
         { "var5", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_hil_extended_t, var5) }, \
         { "var6", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_hil_extended_t, var6) }, \
         { "var7", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_hil_extended_t, var7) }, \
         { "var8", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_hil_extended_t, var8) }, \
         } \
}
#endif

/**
 * @brief Pack a hil_extended message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp [us] Timestamp
 * @param var1  Variable 1
 * @param var2  Variable 2
 * @param var3  Variable 3
 * @param var4  Variable 4
 * @param var5  Variable 5
 * @param var6  Variable 6
 * @param var7  Variable 7
 * @param var8  Variable 8
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_hil_extended_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t timestamp, float var1, float var2, float var3, float var4, float var5, float var6, float var7, float var8)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_HIL_EXTENDED_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, var1);
    _mav_put_float(buf, 12, var2);
    _mav_put_float(buf, 16, var3);
    _mav_put_float(buf, 20, var4);
    _mav_put_float(buf, 24, var5);
    _mav_put_float(buf, 28, var6);
    _mav_put_float(buf, 32, var7);
    _mav_put_float(buf, 36, var8);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_HIL_EXTENDED_LEN);
#else
    mavlink_hil_extended_t packet;
    packet.timestamp = timestamp;
    packet.var1 = var1;
    packet.var2 = var2;
    packet.var3 = var3;
    packet.var4 = var4;
    packet.var5 = var5;
    packet.var6 = var6;
    packet.var7 = var7;
    packet.var8 = var8;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_HIL_EXTENDED_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_HIL_EXTENDED;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_HIL_EXTENDED_MIN_LEN, MAVLINK_MSG_ID_HIL_EXTENDED_LEN, MAVLINK_MSG_ID_HIL_EXTENDED_CRC);
}

/**
 * @brief Pack a hil_extended message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp [us] Timestamp
 * @param var1  Variable 1
 * @param var2  Variable 2
 * @param var3  Variable 3
 * @param var4  Variable 4
 * @param var5  Variable 5
 * @param var6  Variable 6
 * @param var7  Variable 7
 * @param var8  Variable 8
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_hil_extended_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t timestamp,float var1,float var2,float var3,float var4,float var5,float var6,float var7,float var8)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_HIL_EXTENDED_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, var1);
    _mav_put_float(buf, 12, var2);
    _mav_put_float(buf, 16, var3);
    _mav_put_float(buf, 20, var4);
    _mav_put_float(buf, 24, var5);
    _mav_put_float(buf, 28, var6);
    _mav_put_float(buf, 32, var7);
    _mav_put_float(buf, 36, var8);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_HIL_EXTENDED_LEN);
#else
    mavlink_hil_extended_t packet;
    packet.timestamp = timestamp;
    packet.var1 = var1;
    packet.var2 = var2;
    packet.var3 = var3;
    packet.var4 = var4;
    packet.var5 = var5;
    packet.var6 = var6;
    packet.var7 = var7;
    packet.var8 = var8;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_HIL_EXTENDED_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_HIL_EXTENDED;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_HIL_EXTENDED_MIN_LEN, MAVLINK_MSG_ID_HIL_EXTENDED_LEN, MAVLINK_MSG_ID_HIL_EXTENDED_CRC);
}

/**
 * @brief Encode a hil_extended struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param hil_extended C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_hil_extended_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_hil_extended_t* hil_extended)
{
    return mavlink_msg_hil_extended_pack(system_id, component_id, msg, hil_extended->timestamp, hil_extended->var1, hil_extended->var2, hil_extended->var3, hil_extended->var4, hil_extended->var5, hil_extended->var6, hil_extended->var7, hil_extended->var8);
}

/**
 * @brief Encode a hil_extended struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param hil_extended C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_hil_extended_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_hil_extended_t* hil_extended)
{
    return mavlink_msg_hil_extended_pack_chan(system_id, component_id, chan, msg, hil_extended->timestamp, hil_extended->var1, hil_extended->var2, hil_extended->var3, hil_extended->var4, hil_extended->var5, hil_extended->var6, hil_extended->var7, hil_extended->var8);
}

/**
 * @brief Send a hil_extended message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp [us] Timestamp
 * @param var1  Variable 1
 * @param var2  Variable 2
 * @param var3  Variable 3
 * @param var4  Variable 4
 * @param var5  Variable 5
 * @param var6  Variable 6
 * @param var7  Variable 7
 * @param var8  Variable 8
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_hil_extended_send(mavlink_channel_t chan, uint64_t timestamp, float var1, float var2, float var3, float var4, float var5, float var6, float var7, float var8)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_HIL_EXTENDED_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, var1);
    _mav_put_float(buf, 12, var2);
    _mav_put_float(buf, 16, var3);
    _mav_put_float(buf, 20, var4);
    _mav_put_float(buf, 24, var5);
    _mav_put_float(buf, 28, var6);
    _mav_put_float(buf, 32, var7);
    _mav_put_float(buf, 36, var8);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HIL_EXTENDED, buf, MAVLINK_MSG_ID_HIL_EXTENDED_MIN_LEN, MAVLINK_MSG_ID_HIL_EXTENDED_LEN, MAVLINK_MSG_ID_HIL_EXTENDED_CRC);
#else
    mavlink_hil_extended_t packet;
    packet.timestamp = timestamp;
    packet.var1 = var1;
    packet.var2 = var2;
    packet.var3 = var3;
    packet.var4 = var4;
    packet.var5 = var5;
    packet.var6 = var6;
    packet.var7 = var7;
    packet.var8 = var8;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HIL_EXTENDED, (const char *)&packet, MAVLINK_MSG_ID_HIL_EXTENDED_MIN_LEN, MAVLINK_MSG_ID_HIL_EXTENDED_LEN, MAVLINK_MSG_ID_HIL_EXTENDED_CRC);
#endif
}

/**
 * @brief Send a hil_extended message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_hil_extended_send_struct(mavlink_channel_t chan, const mavlink_hil_extended_t* hil_extended)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_hil_extended_send(chan, hil_extended->timestamp, hil_extended->var1, hil_extended->var2, hil_extended->var3, hil_extended->var4, hil_extended->var5, hil_extended->var6, hil_extended->var7, hil_extended->var8);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HIL_EXTENDED, (const char *)hil_extended, MAVLINK_MSG_ID_HIL_EXTENDED_MIN_LEN, MAVLINK_MSG_ID_HIL_EXTENDED_LEN, MAVLINK_MSG_ID_HIL_EXTENDED_CRC);
#endif
}

#if MAVLINK_MSG_ID_HIL_EXTENDED_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_hil_extended_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp, float var1, float var2, float var3, float var4, float var5, float var6, float var7, float var8)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, var1);
    _mav_put_float(buf, 12, var2);
    _mav_put_float(buf, 16, var3);
    _mav_put_float(buf, 20, var4);
    _mav_put_float(buf, 24, var5);
    _mav_put_float(buf, 28, var6);
    _mav_put_float(buf, 32, var7);
    _mav_put_float(buf, 36, var8);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HIL_EXTENDED, buf, MAVLINK_MSG_ID_HIL_EXTENDED_MIN_LEN, MAVLINK_MSG_ID_HIL_EXTENDED_LEN, MAVLINK_MSG_ID_HIL_EXTENDED_CRC);
#else
    mavlink_hil_extended_t *packet = (mavlink_hil_extended_t *)msgbuf;
    packet->timestamp = timestamp;
    packet->var1 = var1;
    packet->var2 = var2;
    packet->var3 = var3;
    packet->var4 = var4;
    packet->var5 = var5;
    packet->var6 = var6;
    packet->var7 = var7;
    packet->var8 = var8;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HIL_EXTENDED, (const char *)packet, MAVLINK_MSG_ID_HIL_EXTENDED_MIN_LEN, MAVLINK_MSG_ID_HIL_EXTENDED_LEN, MAVLINK_MSG_ID_HIL_EXTENDED_CRC);
#endif
}
#endif

#endif

// MESSAGE HIL_EXTENDED UNPACKING


/**
 * @brief Get field timestamp from hil_extended message
 *
 * @return [us] Timestamp
 */
static inline uint64_t mavlink_msg_hil_extended_get_timestamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field var1 from hil_extended message
 *
 * @return  Variable 1
 */
static inline float mavlink_msg_hil_extended_get_var1(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field var2 from hil_extended message
 *
 * @return  Variable 2
 */
static inline float mavlink_msg_hil_extended_get_var2(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field var3 from hil_extended message
 *
 * @return  Variable 3
 */
static inline float mavlink_msg_hil_extended_get_var3(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field var4 from hil_extended message
 *
 * @return  Variable 4
 */
static inline float mavlink_msg_hil_extended_get_var4(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field var5 from hil_extended message
 *
 * @return  Variable 5
 */
static inline float mavlink_msg_hil_extended_get_var5(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field var6 from hil_extended message
 *
 * @return  Variable 6
 */
static inline float mavlink_msg_hil_extended_get_var6(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field var7 from hil_extended message
 *
 * @return  Variable 7
 */
static inline float mavlink_msg_hil_extended_get_var7(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field var8 from hil_extended message
 *
 * @return  Variable 8
 */
static inline float mavlink_msg_hil_extended_get_var8(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Decode a hil_extended message into a struct
 *
 * @param msg The message to decode
 * @param hil_extended C-struct to decode the message contents into
 */
static inline void mavlink_msg_hil_extended_decode(const mavlink_message_t* msg, mavlink_hil_extended_t* hil_extended)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    hil_extended->timestamp = mavlink_msg_hil_extended_get_timestamp(msg);
    hil_extended->var1 = mavlink_msg_hil_extended_get_var1(msg);
    hil_extended->var2 = mavlink_msg_hil_extended_get_var2(msg);
    hil_extended->var3 = mavlink_msg_hil_extended_get_var3(msg);
    hil_extended->var4 = mavlink_msg_hil_extended_get_var4(msg);
    hil_extended->var5 = mavlink_msg_hil_extended_get_var5(msg);
    hil_extended->var6 = mavlink_msg_hil_extended_get_var6(msg);
    hil_extended->var7 = mavlink_msg_hil_extended_get_var7(msg);
    hil_extended->var8 = mavlink_msg_hil_extended_get_var8(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_HIL_EXTENDED_LEN? msg->len : MAVLINK_MSG_ID_HIL_EXTENDED_LEN;
        memset(hil_extended, 0, MAVLINK_MSG_ID_HIL_EXTENDED_LEN);
    memcpy(hil_extended, _MAV_PAYLOAD(msg), len);
#endif
}
