#pragma once
// MESSAGE PATH_REPRESENTATION_DUBINS PACKING

#define MAVLINK_MSG_ID_PATH_REPRESENTATION_DUBINS 341


typedef struct __mavlink_path_representation_dubins_t {
 uint64_t time_usec; /*< [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.*/
 float curvature[5]; /*< [m] 0: line segment*/
 float pos_x[5]; /*< [m] X-coordinate of dupins path segment point*/
 float pos_y[5]; /*< [m] Y-coordinate of dupins path segment point*/
 float pos_z[5]; /*< [m] Z-coordinate of dupins path segment point*/
 uint8_t type; /*<  Path segment type 0: Curvature between the points 1: Curvature on points*/
} mavlink_path_representation_dubins_t;

#define MAVLINK_MSG_ID_PATH_REPRESENTATION_DUBINS_LEN 89
#define MAVLINK_MSG_ID_PATH_REPRESENTATION_DUBINS_MIN_LEN 89
#define MAVLINK_MSG_ID_341_LEN 89
#define MAVLINK_MSG_ID_341_MIN_LEN 89

#define MAVLINK_MSG_ID_PATH_REPRESENTATION_DUBINS_CRC 208
#define MAVLINK_MSG_ID_341_CRC 208

#define MAVLINK_MSG_PATH_REPRESENTATION_DUBINS_FIELD_CURVATURE_LEN 5
#define MAVLINK_MSG_PATH_REPRESENTATION_DUBINS_FIELD_POS_X_LEN 5
#define MAVLINK_MSG_PATH_REPRESENTATION_DUBINS_FIELD_POS_Y_LEN 5
#define MAVLINK_MSG_PATH_REPRESENTATION_DUBINS_FIELD_POS_Z_LEN 5

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_PATH_REPRESENTATION_DUBINS { \
    341, \
    "PATH_REPRESENTATION_DUBINS", \
    6, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_path_representation_dubins_t, time_usec) }, \
         { "type", NULL, MAVLINK_TYPE_UINT8_T, 0, 88, offsetof(mavlink_path_representation_dubins_t, type) }, \
         { "curvature", NULL, MAVLINK_TYPE_FLOAT, 5, 8, offsetof(mavlink_path_representation_dubins_t, curvature) }, \
         { "pos_x", NULL, MAVLINK_TYPE_FLOAT, 5, 28, offsetof(mavlink_path_representation_dubins_t, pos_x) }, \
         { "pos_y", NULL, MAVLINK_TYPE_FLOAT, 5, 48, offsetof(mavlink_path_representation_dubins_t, pos_y) }, \
         { "pos_z", NULL, MAVLINK_TYPE_FLOAT, 5, 68, offsetof(mavlink_path_representation_dubins_t, pos_z) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_PATH_REPRESENTATION_DUBINS { \
    "PATH_REPRESENTATION_DUBINS", \
    6, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_path_representation_dubins_t, time_usec) }, \
         { "type", NULL, MAVLINK_TYPE_UINT8_T, 0, 88, offsetof(mavlink_path_representation_dubins_t, type) }, \
         { "curvature", NULL, MAVLINK_TYPE_FLOAT, 5, 8, offsetof(mavlink_path_representation_dubins_t, curvature) }, \
         { "pos_x", NULL, MAVLINK_TYPE_FLOAT, 5, 28, offsetof(mavlink_path_representation_dubins_t, pos_x) }, \
         { "pos_y", NULL, MAVLINK_TYPE_FLOAT, 5, 48, offsetof(mavlink_path_representation_dubins_t, pos_y) }, \
         { "pos_z", NULL, MAVLINK_TYPE_FLOAT, 5, 68, offsetof(mavlink_path_representation_dubins_t, pos_z) }, \
         } \
}
#endif

/**
 * @brief Pack a path_representation_dubins message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 * @param type  Path segment type 0: Curvature between the points 1: Curvature on points
 * @param curvature [m] 0: line segment
 * @param pos_x [m] X-coordinate of dupins path segment point
 * @param pos_y [m] Y-coordinate of dupins path segment point
 * @param pos_z [m] Z-coordinate of dupins path segment point
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_path_representation_dubins_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t time_usec, uint8_t type, const float *curvature, const float *pos_x, const float *pos_y, const float *pos_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PATH_REPRESENTATION_DUBINS_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_uint8_t(buf, 88, type);
    _mav_put_float_array(buf, 8, curvature, 5);
    _mav_put_float_array(buf, 28, pos_x, 5);
    _mav_put_float_array(buf, 48, pos_y, 5);
    _mav_put_float_array(buf, 68, pos_z, 5);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PATH_REPRESENTATION_DUBINS_LEN);
#else
    mavlink_path_representation_dubins_t packet;
    packet.time_usec = time_usec;
    packet.type = type;
    mav_array_memcpy(packet.curvature, curvature, sizeof(float)*5);
    mav_array_memcpy(packet.pos_x, pos_x, sizeof(float)*5);
    mav_array_memcpy(packet.pos_y, pos_y, sizeof(float)*5);
    mav_array_memcpy(packet.pos_z, pos_z, sizeof(float)*5);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PATH_REPRESENTATION_DUBINS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_PATH_REPRESENTATION_DUBINS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_PATH_REPRESENTATION_DUBINS_MIN_LEN, MAVLINK_MSG_ID_PATH_REPRESENTATION_DUBINS_LEN, MAVLINK_MSG_ID_PATH_REPRESENTATION_DUBINS_CRC);
}

/**
 * @brief Pack a path_representation_dubins message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 * @param type  Path segment type 0: Curvature between the points 1: Curvature on points
 * @param curvature [m] 0: line segment
 * @param pos_x [m] X-coordinate of dupins path segment point
 * @param pos_y [m] Y-coordinate of dupins path segment point
 * @param pos_z [m] Z-coordinate of dupins path segment point
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_path_representation_dubins_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t time_usec,uint8_t type,const float *curvature,const float *pos_x,const float *pos_y,const float *pos_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PATH_REPRESENTATION_DUBINS_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_uint8_t(buf, 88, type);
    _mav_put_float_array(buf, 8, curvature, 5);
    _mav_put_float_array(buf, 28, pos_x, 5);
    _mav_put_float_array(buf, 48, pos_y, 5);
    _mav_put_float_array(buf, 68, pos_z, 5);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PATH_REPRESENTATION_DUBINS_LEN);
#else
    mavlink_path_representation_dubins_t packet;
    packet.time_usec = time_usec;
    packet.type = type;
    mav_array_memcpy(packet.curvature, curvature, sizeof(float)*5);
    mav_array_memcpy(packet.pos_x, pos_x, sizeof(float)*5);
    mav_array_memcpy(packet.pos_y, pos_y, sizeof(float)*5);
    mav_array_memcpy(packet.pos_z, pos_z, sizeof(float)*5);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PATH_REPRESENTATION_DUBINS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_PATH_REPRESENTATION_DUBINS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_PATH_REPRESENTATION_DUBINS_MIN_LEN, MAVLINK_MSG_ID_PATH_REPRESENTATION_DUBINS_LEN, MAVLINK_MSG_ID_PATH_REPRESENTATION_DUBINS_CRC);
}

/**
 * @brief Encode a path_representation_dubins struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param path_representation_dubins C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_path_representation_dubins_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_path_representation_dubins_t* path_representation_dubins)
{
    return mavlink_msg_path_representation_dubins_pack(system_id, component_id, msg, path_representation_dubins->time_usec, path_representation_dubins->type, path_representation_dubins->curvature, path_representation_dubins->pos_x, path_representation_dubins->pos_y, path_representation_dubins->pos_z);
}

/**
 * @brief Encode a path_representation_dubins struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param path_representation_dubins C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_path_representation_dubins_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_path_representation_dubins_t* path_representation_dubins)
{
    return mavlink_msg_path_representation_dubins_pack_chan(system_id, component_id, chan, msg, path_representation_dubins->time_usec, path_representation_dubins->type, path_representation_dubins->curvature, path_representation_dubins->pos_x, path_representation_dubins->pos_y, path_representation_dubins->pos_z);
}

/**
 * @brief Send a path_representation_dubins message
 * @param chan MAVLink channel to send the message
 *
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 * @param type  Path segment type 0: Curvature between the points 1: Curvature on points
 * @param curvature [m] 0: line segment
 * @param pos_x [m] X-coordinate of dupins path segment point
 * @param pos_y [m] Y-coordinate of dupins path segment point
 * @param pos_z [m] Z-coordinate of dupins path segment point
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_path_representation_dubins_send(mavlink_channel_t chan, uint64_t time_usec, uint8_t type, const float *curvature, const float *pos_x, const float *pos_y, const float *pos_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PATH_REPRESENTATION_DUBINS_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_uint8_t(buf, 88, type);
    _mav_put_float_array(buf, 8, curvature, 5);
    _mav_put_float_array(buf, 28, pos_x, 5);
    _mav_put_float_array(buf, 48, pos_y, 5);
    _mav_put_float_array(buf, 68, pos_z, 5);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PATH_REPRESENTATION_DUBINS, buf, MAVLINK_MSG_ID_PATH_REPRESENTATION_DUBINS_MIN_LEN, MAVLINK_MSG_ID_PATH_REPRESENTATION_DUBINS_LEN, MAVLINK_MSG_ID_PATH_REPRESENTATION_DUBINS_CRC);
#else
    mavlink_path_representation_dubins_t packet;
    packet.time_usec = time_usec;
    packet.type = type;
    mav_array_memcpy(packet.curvature, curvature, sizeof(float)*5);
    mav_array_memcpy(packet.pos_x, pos_x, sizeof(float)*5);
    mav_array_memcpy(packet.pos_y, pos_y, sizeof(float)*5);
    mav_array_memcpy(packet.pos_z, pos_z, sizeof(float)*5);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PATH_REPRESENTATION_DUBINS, (const char *)&packet, MAVLINK_MSG_ID_PATH_REPRESENTATION_DUBINS_MIN_LEN, MAVLINK_MSG_ID_PATH_REPRESENTATION_DUBINS_LEN, MAVLINK_MSG_ID_PATH_REPRESENTATION_DUBINS_CRC);
#endif
}

/**
 * @brief Send a path_representation_dubins message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_path_representation_dubins_send_struct(mavlink_channel_t chan, const mavlink_path_representation_dubins_t* path_representation_dubins)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_path_representation_dubins_send(chan, path_representation_dubins->time_usec, path_representation_dubins->type, path_representation_dubins->curvature, path_representation_dubins->pos_x, path_representation_dubins->pos_y, path_representation_dubins->pos_z);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PATH_REPRESENTATION_DUBINS, (const char *)path_representation_dubins, MAVLINK_MSG_ID_PATH_REPRESENTATION_DUBINS_MIN_LEN, MAVLINK_MSG_ID_PATH_REPRESENTATION_DUBINS_LEN, MAVLINK_MSG_ID_PATH_REPRESENTATION_DUBINS_CRC);
#endif
}

#if MAVLINK_MSG_ID_PATH_REPRESENTATION_DUBINS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_path_representation_dubins_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t time_usec, uint8_t type, const float *curvature, const float *pos_x, const float *pos_y, const float *pos_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_uint8_t(buf, 88, type);
    _mav_put_float_array(buf, 8, curvature, 5);
    _mav_put_float_array(buf, 28, pos_x, 5);
    _mav_put_float_array(buf, 48, pos_y, 5);
    _mav_put_float_array(buf, 68, pos_z, 5);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PATH_REPRESENTATION_DUBINS, buf, MAVLINK_MSG_ID_PATH_REPRESENTATION_DUBINS_MIN_LEN, MAVLINK_MSG_ID_PATH_REPRESENTATION_DUBINS_LEN, MAVLINK_MSG_ID_PATH_REPRESENTATION_DUBINS_CRC);
#else
    mavlink_path_representation_dubins_t *packet = (mavlink_path_representation_dubins_t *)msgbuf;
    packet->time_usec = time_usec;
    packet->type = type;
    mav_array_memcpy(packet->curvature, curvature, sizeof(float)*5);
    mav_array_memcpy(packet->pos_x, pos_x, sizeof(float)*5);
    mav_array_memcpy(packet->pos_y, pos_y, sizeof(float)*5);
    mav_array_memcpy(packet->pos_z, pos_z, sizeof(float)*5);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PATH_REPRESENTATION_DUBINS, (const char *)packet, MAVLINK_MSG_ID_PATH_REPRESENTATION_DUBINS_MIN_LEN, MAVLINK_MSG_ID_PATH_REPRESENTATION_DUBINS_LEN, MAVLINK_MSG_ID_PATH_REPRESENTATION_DUBINS_CRC);
#endif
}
#endif

#endif

// MESSAGE PATH_REPRESENTATION_DUBINS UNPACKING


/**
 * @brief Get field time_usec from path_representation_dubins message
 *
 * @return [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 */
static inline uint64_t mavlink_msg_path_representation_dubins_get_time_usec(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field type from path_representation_dubins message
 *
 * @return  Path segment type 0: Curvature between the points 1: Curvature on points
 */
static inline uint8_t mavlink_msg_path_representation_dubins_get_type(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  88);
}

/**
 * @brief Get field curvature from path_representation_dubins message
 *
 * @return [m] 0: line segment
 */
static inline uint16_t mavlink_msg_path_representation_dubins_get_curvature(const mavlink_message_t* msg, float *curvature)
{
    return _MAV_RETURN_float_array(msg, curvature, 5,  8);
}

/**
 * @brief Get field pos_x from path_representation_dubins message
 *
 * @return [m] X-coordinate of dupins path segment point
 */
static inline uint16_t mavlink_msg_path_representation_dubins_get_pos_x(const mavlink_message_t* msg, float *pos_x)
{
    return _MAV_RETURN_float_array(msg, pos_x, 5,  28);
}

/**
 * @brief Get field pos_y from path_representation_dubins message
 *
 * @return [m] Y-coordinate of dupins path segment point
 */
static inline uint16_t mavlink_msg_path_representation_dubins_get_pos_y(const mavlink_message_t* msg, float *pos_y)
{
    return _MAV_RETURN_float_array(msg, pos_y, 5,  48);
}

/**
 * @brief Get field pos_z from path_representation_dubins message
 *
 * @return [m] Z-coordinate of dupins path segment point
 */
static inline uint16_t mavlink_msg_path_representation_dubins_get_pos_z(const mavlink_message_t* msg, float *pos_z)
{
    return _MAV_RETURN_float_array(msg, pos_z, 5,  68);
}

/**
 * @brief Decode a path_representation_dubins message into a struct
 *
 * @param msg The message to decode
 * @param path_representation_dubins C-struct to decode the message contents into
 */
static inline void mavlink_msg_path_representation_dubins_decode(const mavlink_message_t* msg, mavlink_path_representation_dubins_t* path_representation_dubins)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    path_representation_dubins->time_usec = mavlink_msg_path_representation_dubins_get_time_usec(msg);
    mavlink_msg_path_representation_dubins_get_curvature(msg, path_representation_dubins->curvature);
    mavlink_msg_path_representation_dubins_get_pos_x(msg, path_representation_dubins->pos_x);
    mavlink_msg_path_representation_dubins_get_pos_y(msg, path_representation_dubins->pos_y);
    mavlink_msg_path_representation_dubins_get_pos_z(msg, path_representation_dubins->pos_z);
    path_representation_dubins->type = mavlink_msg_path_representation_dubins_get_type(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_PATH_REPRESENTATION_DUBINS_LEN? msg->len : MAVLINK_MSG_ID_PATH_REPRESENTATION_DUBINS_LEN;
        memset(path_representation_dubins, 0, MAVLINK_MSG_ID_PATH_REPRESENTATION_DUBINS_LEN);
    memcpy(path_representation_dubins, _MAV_PAYLOAD(msg), len);
#endif
}
