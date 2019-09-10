#pragma once
// MESSAGE NDI_STATUS PACKING

#define MAVLINK_MSG_ID_NDI_STATUS 217

MAVPACKED(
typedef struct __mavlink_ndi_status_t {
 uint64_t timestamp; /*< [us] Timestamp*/
 float airsp_setpoint_raw; /*< [m/s] Airspeed setpoint before filtering/constraining*/
 float fpa_setpoint_raw; /*< [deg] Flight path angle setpoint before filtering/constraining*/
 float roll_setpoint_raw; /*< [deg] Roll angle setpoint before filtering/constraining*/
 float airsp_setpoint; /*< [m/s] Filtered airspeed setpoint*/
 float fpa_setpoint; /*< [deg] Filtered flight path angle setpoint*/
 float roll_setpoint; /*< [deg] Filtered roll angle setpoint*/
 float throt_setpoint; /*<  Throttle setpoint*/
 float throt_fb; /*<  Throttle feedback control*/
 float throt_trim; /*<  Throttle trim control*/
 float pitch_setpoint; /*< [deg] Pitch setpoint*/
 float pitch_fb; /*< [deg] Pitch feedback control*/
 float pitch_trim; /*< [deg] Pitch trim control*/
 float err_bound; /*< [m] Track error boundary*/
 float filtered_airsp; /*< [m/s] Filtered airspeed state*/
 float filtered_fpa; /*< [deg] Filtered flight path angle state*/
 float air_density; /*< [kg/m^3] Calculated air density*/
}) mavlink_ndi_status_t;

#define MAVLINK_MSG_ID_NDI_STATUS_LEN 72
#define MAVLINK_MSG_ID_NDI_STATUS_MIN_LEN 72
#define MAVLINK_MSG_ID_217_LEN 72
#define MAVLINK_MSG_ID_217_MIN_LEN 72

#define MAVLINK_MSG_ID_NDI_STATUS_CRC 72
#define MAVLINK_MSG_ID_217_CRC 72



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_NDI_STATUS { \
    217, \
    "NDI_STATUS", \
    17, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_ndi_status_t, timestamp) }, \
         { "airsp_setpoint_raw", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_ndi_status_t, airsp_setpoint_raw) }, \
         { "fpa_setpoint_raw", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_ndi_status_t, fpa_setpoint_raw) }, \
         { "roll_setpoint_raw", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_ndi_status_t, roll_setpoint_raw) }, \
         { "airsp_setpoint", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_ndi_status_t, airsp_setpoint) }, \
         { "fpa_setpoint", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_ndi_status_t, fpa_setpoint) }, \
         { "roll_setpoint", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_ndi_status_t, roll_setpoint) }, \
         { "throt_setpoint", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_ndi_status_t, throt_setpoint) }, \
         { "throt_fb", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_ndi_status_t, throt_fb) }, \
         { "throt_trim", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_ndi_status_t, throt_trim) }, \
         { "pitch_setpoint", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_ndi_status_t, pitch_setpoint) }, \
         { "pitch_fb", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_ndi_status_t, pitch_fb) }, \
         { "pitch_trim", NULL, MAVLINK_TYPE_FLOAT, 0, 52, offsetof(mavlink_ndi_status_t, pitch_trim) }, \
         { "err_bound", NULL, MAVLINK_TYPE_FLOAT, 0, 56, offsetof(mavlink_ndi_status_t, err_bound) }, \
         { "filtered_airsp", NULL, MAVLINK_TYPE_FLOAT, 0, 60, offsetof(mavlink_ndi_status_t, filtered_airsp) }, \
         { "filtered_fpa", NULL, MAVLINK_TYPE_FLOAT, 0, 64, offsetof(mavlink_ndi_status_t, filtered_fpa) }, \
         { "air_density", NULL, MAVLINK_TYPE_FLOAT, 0, 68, offsetof(mavlink_ndi_status_t, air_density) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_NDI_STATUS { \
    "NDI_STATUS", \
    17, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_ndi_status_t, timestamp) }, \
         { "airsp_setpoint_raw", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_ndi_status_t, airsp_setpoint_raw) }, \
         { "fpa_setpoint_raw", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_ndi_status_t, fpa_setpoint_raw) }, \
         { "roll_setpoint_raw", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_ndi_status_t, roll_setpoint_raw) }, \
         { "airsp_setpoint", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_ndi_status_t, airsp_setpoint) }, \
         { "fpa_setpoint", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_ndi_status_t, fpa_setpoint) }, \
         { "roll_setpoint", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_ndi_status_t, roll_setpoint) }, \
         { "throt_setpoint", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_ndi_status_t, throt_setpoint) }, \
         { "throt_fb", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_ndi_status_t, throt_fb) }, \
         { "throt_trim", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_ndi_status_t, throt_trim) }, \
         { "pitch_setpoint", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_ndi_status_t, pitch_setpoint) }, \
         { "pitch_fb", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_ndi_status_t, pitch_fb) }, \
         { "pitch_trim", NULL, MAVLINK_TYPE_FLOAT, 0, 52, offsetof(mavlink_ndi_status_t, pitch_trim) }, \
         { "err_bound", NULL, MAVLINK_TYPE_FLOAT, 0, 56, offsetof(mavlink_ndi_status_t, err_bound) }, \
         { "filtered_airsp", NULL, MAVLINK_TYPE_FLOAT, 0, 60, offsetof(mavlink_ndi_status_t, filtered_airsp) }, \
         { "filtered_fpa", NULL, MAVLINK_TYPE_FLOAT, 0, 64, offsetof(mavlink_ndi_status_t, filtered_fpa) }, \
         { "air_density", NULL, MAVLINK_TYPE_FLOAT, 0, 68, offsetof(mavlink_ndi_status_t, air_density) }, \
         } \
}
#endif

/**
 * @brief Pack a ndi_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp [us] Timestamp
 * @param airsp_setpoint_raw [m/s] Airspeed setpoint before filtering/constraining
 * @param fpa_setpoint_raw [deg] Flight path angle setpoint before filtering/constraining
 * @param roll_setpoint_raw [deg] Roll angle setpoint before filtering/constraining
 * @param airsp_setpoint [m/s] Filtered airspeed setpoint
 * @param fpa_setpoint [deg] Filtered flight path angle setpoint
 * @param roll_setpoint [deg] Filtered roll angle setpoint
 * @param throt_setpoint  Throttle setpoint
 * @param throt_fb  Throttle feedback control
 * @param throt_trim  Throttle trim control
 * @param pitch_setpoint [deg] Pitch setpoint
 * @param pitch_fb [deg] Pitch feedback control
 * @param pitch_trim [deg] Pitch trim control
 * @param err_bound [m] Track error boundary
 * @param filtered_airsp [m/s] Filtered airspeed state
 * @param filtered_fpa [deg] Filtered flight path angle state
 * @param air_density [kg/m^3] Calculated air density
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ndi_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t timestamp, float airsp_setpoint_raw, float fpa_setpoint_raw, float roll_setpoint_raw, float airsp_setpoint, float fpa_setpoint, float roll_setpoint, float throt_setpoint, float throt_fb, float throt_trim, float pitch_setpoint, float pitch_fb, float pitch_trim, float err_bound, float filtered_airsp, float filtered_fpa, float air_density)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_NDI_STATUS_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, airsp_setpoint_raw);
    _mav_put_float(buf, 12, fpa_setpoint_raw);
    _mav_put_float(buf, 16, roll_setpoint_raw);
    _mav_put_float(buf, 20, airsp_setpoint);
    _mav_put_float(buf, 24, fpa_setpoint);
    _mav_put_float(buf, 28, roll_setpoint);
    _mav_put_float(buf, 32, throt_setpoint);
    _mav_put_float(buf, 36, throt_fb);
    _mav_put_float(buf, 40, throt_trim);
    _mav_put_float(buf, 44, pitch_setpoint);
    _mav_put_float(buf, 48, pitch_fb);
    _mav_put_float(buf, 52, pitch_trim);
    _mav_put_float(buf, 56, err_bound);
    _mav_put_float(buf, 60, filtered_airsp);
    _mav_put_float(buf, 64, filtered_fpa);
    _mav_put_float(buf, 68, air_density);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_NDI_STATUS_LEN);
#else
    mavlink_ndi_status_t packet;
    packet.timestamp = timestamp;
    packet.airsp_setpoint_raw = airsp_setpoint_raw;
    packet.fpa_setpoint_raw = fpa_setpoint_raw;
    packet.roll_setpoint_raw = roll_setpoint_raw;
    packet.airsp_setpoint = airsp_setpoint;
    packet.fpa_setpoint = fpa_setpoint;
    packet.roll_setpoint = roll_setpoint;
    packet.throt_setpoint = throt_setpoint;
    packet.throt_fb = throt_fb;
    packet.throt_trim = throt_trim;
    packet.pitch_setpoint = pitch_setpoint;
    packet.pitch_fb = pitch_fb;
    packet.pitch_trim = pitch_trim;
    packet.err_bound = err_bound;
    packet.filtered_airsp = filtered_airsp;
    packet.filtered_fpa = filtered_fpa;
    packet.air_density = air_density;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_NDI_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_NDI_STATUS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_NDI_STATUS_MIN_LEN, MAVLINK_MSG_ID_NDI_STATUS_LEN, MAVLINK_MSG_ID_NDI_STATUS_CRC);
}

/**
 * @brief Pack a ndi_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp [us] Timestamp
 * @param airsp_setpoint_raw [m/s] Airspeed setpoint before filtering/constraining
 * @param fpa_setpoint_raw [deg] Flight path angle setpoint before filtering/constraining
 * @param roll_setpoint_raw [deg] Roll angle setpoint before filtering/constraining
 * @param airsp_setpoint [m/s] Filtered airspeed setpoint
 * @param fpa_setpoint [deg] Filtered flight path angle setpoint
 * @param roll_setpoint [deg] Filtered roll angle setpoint
 * @param throt_setpoint  Throttle setpoint
 * @param throt_fb  Throttle feedback control
 * @param throt_trim  Throttle trim control
 * @param pitch_setpoint [deg] Pitch setpoint
 * @param pitch_fb [deg] Pitch feedback control
 * @param pitch_trim [deg] Pitch trim control
 * @param err_bound [m] Track error boundary
 * @param filtered_airsp [m/s] Filtered airspeed state
 * @param filtered_fpa [deg] Filtered flight path angle state
 * @param air_density [kg/m^3] Calculated air density
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ndi_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t timestamp,float airsp_setpoint_raw,float fpa_setpoint_raw,float roll_setpoint_raw,float airsp_setpoint,float fpa_setpoint,float roll_setpoint,float throt_setpoint,float throt_fb,float throt_trim,float pitch_setpoint,float pitch_fb,float pitch_trim,float err_bound,float filtered_airsp,float filtered_fpa,float air_density)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_NDI_STATUS_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, airsp_setpoint_raw);
    _mav_put_float(buf, 12, fpa_setpoint_raw);
    _mav_put_float(buf, 16, roll_setpoint_raw);
    _mav_put_float(buf, 20, airsp_setpoint);
    _mav_put_float(buf, 24, fpa_setpoint);
    _mav_put_float(buf, 28, roll_setpoint);
    _mav_put_float(buf, 32, throt_setpoint);
    _mav_put_float(buf, 36, throt_fb);
    _mav_put_float(buf, 40, throt_trim);
    _mav_put_float(buf, 44, pitch_setpoint);
    _mav_put_float(buf, 48, pitch_fb);
    _mav_put_float(buf, 52, pitch_trim);
    _mav_put_float(buf, 56, err_bound);
    _mav_put_float(buf, 60, filtered_airsp);
    _mav_put_float(buf, 64, filtered_fpa);
    _mav_put_float(buf, 68, air_density);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_NDI_STATUS_LEN);
#else
    mavlink_ndi_status_t packet;
    packet.timestamp = timestamp;
    packet.airsp_setpoint_raw = airsp_setpoint_raw;
    packet.fpa_setpoint_raw = fpa_setpoint_raw;
    packet.roll_setpoint_raw = roll_setpoint_raw;
    packet.airsp_setpoint = airsp_setpoint;
    packet.fpa_setpoint = fpa_setpoint;
    packet.roll_setpoint = roll_setpoint;
    packet.throt_setpoint = throt_setpoint;
    packet.throt_fb = throt_fb;
    packet.throt_trim = throt_trim;
    packet.pitch_setpoint = pitch_setpoint;
    packet.pitch_fb = pitch_fb;
    packet.pitch_trim = pitch_trim;
    packet.err_bound = err_bound;
    packet.filtered_airsp = filtered_airsp;
    packet.filtered_fpa = filtered_fpa;
    packet.air_density = air_density;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_NDI_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_NDI_STATUS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_NDI_STATUS_MIN_LEN, MAVLINK_MSG_ID_NDI_STATUS_LEN, MAVLINK_MSG_ID_NDI_STATUS_CRC);
}

/**
 * @brief Encode a ndi_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param ndi_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_ndi_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_ndi_status_t* ndi_status)
{
    return mavlink_msg_ndi_status_pack(system_id, component_id, msg, ndi_status->timestamp, ndi_status->airsp_setpoint_raw, ndi_status->fpa_setpoint_raw, ndi_status->roll_setpoint_raw, ndi_status->airsp_setpoint, ndi_status->fpa_setpoint, ndi_status->roll_setpoint, ndi_status->throt_setpoint, ndi_status->throt_fb, ndi_status->throt_trim, ndi_status->pitch_setpoint, ndi_status->pitch_fb, ndi_status->pitch_trim, ndi_status->err_bound, ndi_status->filtered_airsp, ndi_status->filtered_fpa, ndi_status->air_density);
}

/**
 * @brief Encode a ndi_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param ndi_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_ndi_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_ndi_status_t* ndi_status)
{
    return mavlink_msg_ndi_status_pack_chan(system_id, component_id, chan, msg, ndi_status->timestamp, ndi_status->airsp_setpoint_raw, ndi_status->fpa_setpoint_raw, ndi_status->roll_setpoint_raw, ndi_status->airsp_setpoint, ndi_status->fpa_setpoint, ndi_status->roll_setpoint, ndi_status->throt_setpoint, ndi_status->throt_fb, ndi_status->throt_trim, ndi_status->pitch_setpoint, ndi_status->pitch_fb, ndi_status->pitch_trim, ndi_status->err_bound, ndi_status->filtered_airsp, ndi_status->filtered_fpa, ndi_status->air_density);
}

/**
 * @brief Send a ndi_status message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp [us] Timestamp
 * @param airsp_setpoint_raw [m/s] Airspeed setpoint before filtering/constraining
 * @param fpa_setpoint_raw [deg] Flight path angle setpoint before filtering/constraining
 * @param roll_setpoint_raw [deg] Roll angle setpoint before filtering/constraining
 * @param airsp_setpoint [m/s] Filtered airspeed setpoint
 * @param fpa_setpoint [deg] Filtered flight path angle setpoint
 * @param roll_setpoint [deg] Filtered roll angle setpoint
 * @param throt_setpoint  Throttle setpoint
 * @param throt_fb  Throttle feedback control
 * @param throt_trim  Throttle trim control
 * @param pitch_setpoint [deg] Pitch setpoint
 * @param pitch_fb [deg] Pitch feedback control
 * @param pitch_trim [deg] Pitch trim control
 * @param err_bound [m] Track error boundary
 * @param filtered_airsp [m/s] Filtered airspeed state
 * @param filtered_fpa [deg] Filtered flight path angle state
 * @param air_density [kg/m^3] Calculated air density
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_ndi_status_send(mavlink_channel_t chan, uint64_t timestamp, float airsp_setpoint_raw, float fpa_setpoint_raw, float roll_setpoint_raw, float airsp_setpoint, float fpa_setpoint, float roll_setpoint, float throt_setpoint, float throt_fb, float throt_trim, float pitch_setpoint, float pitch_fb, float pitch_trim, float err_bound, float filtered_airsp, float filtered_fpa, float air_density)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_NDI_STATUS_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, airsp_setpoint_raw);
    _mav_put_float(buf, 12, fpa_setpoint_raw);
    _mav_put_float(buf, 16, roll_setpoint_raw);
    _mav_put_float(buf, 20, airsp_setpoint);
    _mav_put_float(buf, 24, fpa_setpoint);
    _mav_put_float(buf, 28, roll_setpoint);
    _mav_put_float(buf, 32, throt_setpoint);
    _mav_put_float(buf, 36, throt_fb);
    _mav_put_float(buf, 40, throt_trim);
    _mav_put_float(buf, 44, pitch_setpoint);
    _mav_put_float(buf, 48, pitch_fb);
    _mav_put_float(buf, 52, pitch_trim);
    _mav_put_float(buf, 56, err_bound);
    _mav_put_float(buf, 60, filtered_airsp);
    _mav_put_float(buf, 64, filtered_fpa);
    _mav_put_float(buf, 68, air_density);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NDI_STATUS, buf, MAVLINK_MSG_ID_NDI_STATUS_MIN_LEN, MAVLINK_MSG_ID_NDI_STATUS_LEN, MAVLINK_MSG_ID_NDI_STATUS_CRC);
#else
    mavlink_ndi_status_t packet;
    packet.timestamp = timestamp;
    packet.airsp_setpoint_raw = airsp_setpoint_raw;
    packet.fpa_setpoint_raw = fpa_setpoint_raw;
    packet.roll_setpoint_raw = roll_setpoint_raw;
    packet.airsp_setpoint = airsp_setpoint;
    packet.fpa_setpoint = fpa_setpoint;
    packet.roll_setpoint = roll_setpoint;
    packet.throt_setpoint = throt_setpoint;
    packet.throt_fb = throt_fb;
    packet.throt_trim = throt_trim;
    packet.pitch_setpoint = pitch_setpoint;
    packet.pitch_fb = pitch_fb;
    packet.pitch_trim = pitch_trim;
    packet.err_bound = err_bound;
    packet.filtered_airsp = filtered_airsp;
    packet.filtered_fpa = filtered_fpa;
    packet.air_density = air_density;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NDI_STATUS, (const char *)&packet, MAVLINK_MSG_ID_NDI_STATUS_MIN_LEN, MAVLINK_MSG_ID_NDI_STATUS_LEN, MAVLINK_MSG_ID_NDI_STATUS_CRC);
#endif
}

/**
 * @brief Send a ndi_status message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_ndi_status_send_struct(mavlink_channel_t chan, const mavlink_ndi_status_t* ndi_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_ndi_status_send(chan, ndi_status->timestamp, ndi_status->airsp_setpoint_raw, ndi_status->fpa_setpoint_raw, ndi_status->roll_setpoint_raw, ndi_status->airsp_setpoint, ndi_status->fpa_setpoint, ndi_status->roll_setpoint, ndi_status->throt_setpoint, ndi_status->throt_fb, ndi_status->throt_trim, ndi_status->pitch_setpoint, ndi_status->pitch_fb, ndi_status->pitch_trim, ndi_status->err_bound, ndi_status->filtered_airsp, ndi_status->filtered_fpa, ndi_status->air_density);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NDI_STATUS, (const char *)ndi_status, MAVLINK_MSG_ID_NDI_STATUS_MIN_LEN, MAVLINK_MSG_ID_NDI_STATUS_LEN, MAVLINK_MSG_ID_NDI_STATUS_CRC);
#endif
}

#if MAVLINK_MSG_ID_NDI_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_ndi_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp, float airsp_setpoint_raw, float fpa_setpoint_raw, float roll_setpoint_raw, float airsp_setpoint, float fpa_setpoint, float roll_setpoint, float throt_setpoint, float throt_fb, float throt_trim, float pitch_setpoint, float pitch_fb, float pitch_trim, float err_bound, float filtered_airsp, float filtered_fpa, float air_density)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, airsp_setpoint_raw);
    _mav_put_float(buf, 12, fpa_setpoint_raw);
    _mav_put_float(buf, 16, roll_setpoint_raw);
    _mav_put_float(buf, 20, airsp_setpoint);
    _mav_put_float(buf, 24, fpa_setpoint);
    _mav_put_float(buf, 28, roll_setpoint);
    _mav_put_float(buf, 32, throt_setpoint);
    _mav_put_float(buf, 36, throt_fb);
    _mav_put_float(buf, 40, throt_trim);
    _mav_put_float(buf, 44, pitch_setpoint);
    _mav_put_float(buf, 48, pitch_fb);
    _mav_put_float(buf, 52, pitch_trim);
    _mav_put_float(buf, 56, err_bound);
    _mav_put_float(buf, 60, filtered_airsp);
    _mav_put_float(buf, 64, filtered_fpa);
    _mav_put_float(buf, 68, air_density);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NDI_STATUS, buf, MAVLINK_MSG_ID_NDI_STATUS_MIN_LEN, MAVLINK_MSG_ID_NDI_STATUS_LEN, MAVLINK_MSG_ID_NDI_STATUS_CRC);
#else
    mavlink_ndi_status_t *packet = (mavlink_ndi_status_t *)msgbuf;
    packet->timestamp = timestamp;
    packet->airsp_setpoint_raw = airsp_setpoint_raw;
    packet->fpa_setpoint_raw = fpa_setpoint_raw;
    packet->roll_setpoint_raw = roll_setpoint_raw;
    packet->airsp_setpoint = airsp_setpoint;
    packet->fpa_setpoint = fpa_setpoint;
    packet->roll_setpoint = roll_setpoint;
    packet->throt_setpoint = throt_setpoint;
    packet->throt_fb = throt_fb;
    packet->throt_trim = throt_trim;
    packet->pitch_setpoint = pitch_setpoint;
    packet->pitch_fb = pitch_fb;
    packet->pitch_trim = pitch_trim;
    packet->err_bound = err_bound;
    packet->filtered_airsp = filtered_airsp;
    packet->filtered_fpa = filtered_fpa;
    packet->air_density = air_density;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NDI_STATUS, (const char *)packet, MAVLINK_MSG_ID_NDI_STATUS_MIN_LEN, MAVLINK_MSG_ID_NDI_STATUS_LEN, MAVLINK_MSG_ID_NDI_STATUS_CRC);
#endif
}
#endif

#endif

// MESSAGE NDI_STATUS UNPACKING


/**
 * @brief Get field timestamp from ndi_status message
 *
 * @return [us] Timestamp
 */
static inline uint64_t mavlink_msg_ndi_status_get_timestamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field airsp_setpoint_raw from ndi_status message
 *
 * @return [m/s] Airspeed setpoint before filtering/constraining
 */
static inline float mavlink_msg_ndi_status_get_airsp_setpoint_raw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field fpa_setpoint_raw from ndi_status message
 *
 * @return [deg] Flight path angle setpoint before filtering/constraining
 */
static inline float mavlink_msg_ndi_status_get_fpa_setpoint_raw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field roll_setpoint_raw from ndi_status message
 *
 * @return [deg] Roll angle setpoint before filtering/constraining
 */
static inline float mavlink_msg_ndi_status_get_roll_setpoint_raw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field airsp_setpoint from ndi_status message
 *
 * @return [m/s] Filtered airspeed setpoint
 */
static inline float mavlink_msg_ndi_status_get_airsp_setpoint(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field fpa_setpoint from ndi_status message
 *
 * @return [deg] Filtered flight path angle setpoint
 */
static inline float mavlink_msg_ndi_status_get_fpa_setpoint(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field roll_setpoint from ndi_status message
 *
 * @return [deg] Filtered roll angle setpoint
 */
static inline float mavlink_msg_ndi_status_get_roll_setpoint(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field throt_setpoint from ndi_status message
 *
 * @return  Throttle setpoint
 */
static inline float mavlink_msg_ndi_status_get_throt_setpoint(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field throt_fb from ndi_status message
 *
 * @return  Throttle feedback control
 */
static inline float mavlink_msg_ndi_status_get_throt_fb(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Get field throt_trim from ndi_status message
 *
 * @return  Throttle trim control
 */
static inline float mavlink_msg_ndi_status_get_throt_trim(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  40);
}

/**
 * @brief Get field pitch_setpoint from ndi_status message
 *
 * @return [deg] Pitch setpoint
 */
static inline float mavlink_msg_ndi_status_get_pitch_setpoint(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  44);
}

/**
 * @brief Get field pitch_fb from ndi_status message
 *
 * @return [deg] Pitch feedback control
 */
static inline float mavlink_msg_ndi_status_get_pitch_fb(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  48);
}

/**
 * @brief Get field pitch_trim from ndi_status message
 *
 * @return [deg] Pitch trim control
 */
static inline float mavlink_msg_ndi_status_get_pitch_trim(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  52);
}

/**
 * @brief Get field err_bound from ndi_status message
 *
 * @return [m] Track error boundary
 */
static inline float mavlink_msg_ndi_status_get_err_bound(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  56);
}

/**
 * @brief Get field filtered_airsp from ndi_status message
 *
 * @return [m/s] Filtered airspeed state
 */
static inline float mavlink_msg_ndi_status_get_filtered_airsp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  60);
}

/**
 * @brief Get field filtered_fpa from ndi_status message
 *
 * @return [deg] Filtered flight path angle state
 */
static inline float mavlink_msg_ndi_status_get_filtered_fpa(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  64);
}

/**
 * @brief Get field air_density from ndi_status message
 *
 * @return [kg/m^3] Calculated air density
 */
static inline float mavlink_msg_ndi_status_get_air_density(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  68);
}

/**
 * @brief Decode a ndi_status message into a struct
 *
 * @param msg The message to decode
 * @param ndi_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_ndi_status_decode(const mavlink_message_t* msg, mavlink_ndi_status_t* ndi_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    ndi_status->timestamp = mavlink_msg_ndi_status_get_timestamp(msg);
    ndi_status->airsp_setpoint_raw = mavlink_msg_ndi_status_get_airsp_setpoint_raw(msg);
    ndi_status->fpa_setpoint_raw = mavlink_msg_ndi_status_get_fpa_setpoint_raw(msg);
    ndi_status->roll_setpoint_raw = mavlink_msg_ndi_status_get_roll_setpoint_raw(msg);
    ndi_status->airsp_setpoint = mavlink_msg_ndi_status_get_airsp_setpoint(msg);
    ndi_status->fpa_setpoint = mavlink_msg_ndi_status_get_fpa_setpoint(msg);
    ndi_status->roll_setpoint = mavlink_msg_ndi_status_get_roll_setpoint(msg);
    ndi_status->throt_setpoint = mavlink_msg_ndi_status_get_throt_setpoint(msg);
    ndi_status->throt_fb = mavlink_msg_ndi_status_get_throt_fb(msg);
    ndi_status->throt_trim = mavlink_msg_ndi_status_get_throt_trim(msg);
    ndi_status->pitch_setpoint = mavlink_msg_ndi_status_get_pitch_setpoint(msg);
    ndi_status->pitch_fb = mavlink_msg_ndi_status_get_pitch_fb(msg);
    ndi_status->pitch_trim = mavlink_msg_ndi_status_get_pitch_trim(msg);
    ndi_status->err_bound = mavlink_msg_ndi_status_get_err_bound(msg);
    ndi_status->filtered_airsp = mavlink_msg_ndi_status_get_filtered_airsp(msg);
    ndi_status->filtered_fpa = mavlink_msg_ndi_status_get_filtered_fpa(msg);
    ndi_status->air_density = mavlink_msg_ndi_status_get_air_density(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_NDI_STATUS_LEN? msg->len : MAVLINK_MSG_ID_NDI_STATUS_LEN;
        memset(ndi_status, 0, MAVLINK_MSG_ID_NDI_STATUS_LEN);
    memcpy(ndi_status, _MAV_PAYLOAD(msg), len);
#endif
}
