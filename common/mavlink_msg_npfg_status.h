#pragma once
// MESSAGE NPFG_STATUS PACKING

#define MAVLINK_MSG_ID_NPFG_STATUS 8001


typedef struct __mavlink_npfg_status_t {
 uint64_t timestamp; /*< [us] Timestamp*/
 float lat_accel; /*< [m/s^2] Resultant lateral acceleration reference*/
 float lat_accel_ff; /*< [m/s^2] Feed-forward lateral acceleration for maintaining path curvature*/
 float bearing_feas; /*<  Bearing feasibility*/
 float bearing_feas_on_track; /*<  On-track bearing feasibility*/
 float signed_track_error; /*< [m] Signed track error*/
 float track_error_bound; /*< [m] Track error bound*/
 float airspeed_ref; /*< [m/s] Airspeed reference*/
 float bearing; /*< [deg] Bearing angle*/
 float heading_ref; /*< [deg] Heading angle reference*/
 float min_ground_speed_ref; /*< [m/s] Minimum forward ground speed reference*/
 float adapted_period; /*< [s] Adapted period (if auto-tuning enabled)*/
 float p_gain; /*< [rad/s] Controller proportional gain*/
 float time_const; /*< [s] Controller time constant*/
 uint8_t wind_est_valid; /*<  Wind estimate is valid and being used*/
} mavlink_npfg_status_t;

#define MAVLINK_MSG_ID_NPFG_STATUS_LEN 61
#define MAVLINK_MSG_ID_NPFG_STATUS_MIN_LEN 61
#define MAVLINK_MSG_ID_8001_LEN 61
#define MAVLINK_MSG_ID_8001_MIN_LEN 61

#define MAVLINK_MSG_ID_NPFG_STATUS_CRC 167
#define MAVLINK_MSG_ID_8001_CRC 167



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_NPFG_STATUS { \
    8001, \
    "NPFG_STATUS", \
    15, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_npfg_status_t, timestamp) }, \
         { "wind_est_valid", NULL, MAVLINK_TYPE_UINT8_T, 0, 60, offsetof(mavlink_npfg_status_t, wind_est_valid) }, \
         { "lat_accel", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_npfg_status_t, lat_accel) }, \
         { "lat_accel_ff", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_npfg_status_t, lat_accel_ff) }, \
         { "bearing_feas", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_npfg_status_t, bearing_feas) }, \
         { "bearing_feas_on_track", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_npfg_status_t, bearing_feas_on_track) }, \
         { "signed_track_error", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_npfg_status_t, signed_track_error) }, \
         { "track_error_bound", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_npfg_status_t, track_error_bound) }, \
         { "airspeed_ref", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_npfg_status_t, airspeed_ref) }, \
         { "bearing", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_npfg_status_t, bearing) }, \
         { "heading_ref", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_npfg_status_t, heading_ref) }, \
         { "min_ground_speed_ref", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_npfg_status_t, min_ground_speed_ref) }, \
         { "adapted_period", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_npfg_status_t, adapted_period) }, \
         { "p_gain", NULL, MAVLINK_TYPE_FLOAT, 0, 52, offsetof(mavlink_npfg_status_t, p_gain) }, \
         { "time_const", NULL, MAVLINK_TYPE_FLOAT, 0, 56, offsetof(mavlink_npfg_status_t, time_const) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_NPFG_STATUS { \
    "NPFG_STATUS", \
    15, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_npfg_status_t, timestamp) }, \
         { "wind_est_valid", NULL, MAVLINK_TYPE_UINT8_T, 0, 60, offsetof(mavlink_npfg_status_t, wind_est_valid) }, \
         { "lat_accel", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_npfg_status_t, lat_accel) }, \
         { "lat_accel_ff", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_npfg_status_t, lat_accel_ff) }, \
         { "bearing_feas", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_npfg_status_t, bearing_feas) }, \
         { "bearing_feas_on_track", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_npfg_status_t, bearing_feas_on_track) }, \
         { "signed_track_error", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_npfg_status_t, signed_track_error) }, \
         { "track_error_bound", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_npfg_status_t, track_error_bound) }, \
         { "airspeed_ref", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_npfg_status_t, airspeed_ref) }, \
         { "bearing", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_npfg_status_t, bearing) }, \
         { "heading_ref", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_npfg_status_t, heading_ref) }, \
         { "min_ground_speed_ref", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_npfg_status_t, min_ground_speed_ref) }, \
         { "adapted_period", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_npfg_status_t, adapted_period) }, \
         { "p_gain", NULL, MAVLINK_TYPE_FLOAT, 0, 52, offsetof(mavlink_npfg_status_t, p_gain) }, \
         { "time_const", NULL, MAVLINK_TYPE_FLOAT, 0, 56, offsetof(mavlink_npfg_status_t, time_const) }, \
         } \
}
#endif

/**
 * @brief Pack a npfg_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp [us] Timestamp
 * @param wind_est_valid  Wind estimate is valid and being used
 * @param lat_accel [m/s^2] Resultant lateral acceleration reference
 * @param lat_accel_ff [m/s^2] Feed-forward lateral acceleration for maintaining path curvature
 * @param bearing_feas  Bearing feasibility
 * @param bearing_feas_on_track  On-track bearing feasibility
 * @param signed_track_error [m] Signed track error
 * @param track_error_bound [m] Track error bound
 * @param airspeed_ref [m/s] Airspeed reference
 * @param bearing [deg] Bearing angle
 * @param heading_ref [deg] Heading angle reference
 * @param min_ground_speed_ref [m/s] Minimum forward ground speed reference
 * @param adapted_period [s] Adapted period (if auto-tuning enabled)
 * @param p_gain [rad/s] Controller proportional gain
 * @param time_const [s] Controller time constant
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_npfg_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t timestamp, uint8_t wind_est_valid, float lat_accel, float lat_accel_ff, float bearing_feas, float bearing_feas_on_track, float signed_track_error, float track_error_bound, float airspeed_ref, float bearing, float heading_ref, float min_ground_speed_ref, float adapted_period, float p_gain, float time_const)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_NPFG_STATUS_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, lat_accel);
    _mav_put_float(buf, 12, lat_accel_ff);
    _mav_put_float(buf, 16, bearing_feas);
    _mav_put_float(buf, 20, bearing_feas_on_track);
    _mav_put_float(buf, 24, signed_track_error);
    _mav_put_float(buf, 28, track_error_bound);
    _mav_put_float(buf, 32, airspeed_ref);
    _mav_put_float(buf, 36, bearing);
    _mav_put_float(buf, 40, heading_ref);
    _mav_put_float(buf, 44, min_ground_speed_ref);
    _mav_put_float(buf, 48, adapted_period);
    _mav_put_float(buf, 52, p_gain);
    _mav_put_float(buf, 56, time_const);
    _mav_put_uint8_t(buf, 60, wind_est_valid);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_NPFG_STATUS_LEN);
#else
    mavlink_npfg_status_t packet;
    packet.timestamp = timestamp;
    packet.lat_accel = lat_accel;
    packet.lat_accel_ff = lat_accel_ff;
    packet.bearing_feas = bearing_feas;
    packet.bearing_feas_on_track = bearing_feas_on_track;
    packet.signed_track_error = signed_track_error;
    packet.track_error_bound = track_error_bound;
    packet.airspeed_ref = airspeed_ref;
    packet.bearing = bearing;
    packet.heading_ref = heading_ref;
    packet.min_ground_speed_ref = min_ground_speed_ref;
    packet.adapted_period = adapted_period;
    packet.p_gain = p_gain;
    packet.time_const = time_const;
    packet.wind_est_valid = wind_est_valid;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_NPFG_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_NPFG_STATUS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_NPFG_STATUS_MIN_LEN, MAVLINK_MSG_ID_NPFG_STATUS_LEN, MAVLINK_MSG_ID_NPFG_STATUS_CRC);
}

/**
 * @brief Pack a npfg_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp [us] Timestamp
 * @param wind_est_valid  Wind estimate is valid and being used
 * @param lat_accel [m/s^2] Resultant lateral acceleration reference
 * @param lat_accel_ff [m/s^2] Feed-forward lateral acceleration for maintaining path curvature
 * @param bearing_feas  Bearing feasibility
 * @param bearing_feas_on_track  On-track bearing feasibility
 * @param signed_track_error [m] Signed track error
 * @param track_error_bound [m] Track error bound
 * @param airspeed_ref [m/s] Airspeed reference
 * @param bearing [deg] Bearing angle
 * @param heading_ref [deg] Heading angle reference
 * @param min_ground_speed_ref [m/s] Minimum forward ground speed reference
 * @param adapted_period [s] Adapted period (if auto-tuning enabled)
 * @param p_gain [rad/s] Controller proportional gain
 * @param time_const [s] Controller time constant
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_npfg_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t timestamp,uint8_t wind_est_valid,float lat_accel,float lat_accel_ff,float bearing_feas,float bearing_feas_on_track,float signed_track_error,float track_error_bound,float airspeed_ref,float bearing,float heading_ref,float min_ground_speed_ref,float adapted_period,float p_gain,float time_const)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_NPFG_STATUS_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, lat_accel);
    _mav_put_float(buf, 12, lat_accel_ff);
    _mav_put_float(buf, 16, bearing_feas);
    _mav_put_float(buf, 20, bearing_feas_on_track);
    _mav_put_float(buf, 24, signed_track_error);
    _mav_put_float(buf, 28, track_error_bound);
    _mav_put_float(buf, 32, airspeed_ref);
    _mav_put_float(buf, 36, bearing);
    _mav_put_float(buf, 40, heading_ref);
    _mav_put_float(buf, 44, min_ground_speed_ref);
    _mav_put_float(buf, 48, adapted_period);
    _mav_put_float(buf, 52, p_gain);
    _mav_put_float(buf, 56, time_const);
    _mav_put_uint8_t(buf, 60, wind_est_valid);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_NPFG_STATUS_LEN);
#else
    mavlink_npfg_status_t packet;
    packet.timestamp = timestamp;
    packet.lat_accel = lat_accel;
    packet.lat_accel_ff = lat_accel_ff;
    packet.bearing_feas = bearing_feas;
    packet.bearing_feas_on_track = bearing_feas_on_track;
    packet.signed_track_error = signed_track_error;
    packet.track_error_bound = track_error_bound;
    packet.airspeed_ref = airspeed_ref;
    packet.bearing = bearing;
    packet.heading_ref = heading_ref;
    packet.min_ground_speed_ref = min_ground_speed_ref;
    packet.adapted_period = adapted_period;
    packet.p_gain = p_gain;
    packet.time_const = time_const;
    packet.wind_est_valid = wind_est_valid;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_NPFG_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_NPFG_STATUS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_NPFG_STATUS_MIN_LEN, MAVLINK_MSG_ID_NPFG_STATUS_LEN, MAVLINK_MSG_ID_NPFG_STATUS_CRC);
}

/**
 * @brief Encode a npfg_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param npfg_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_npfg_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_npfg_status_t* npfg_status)
{
    return mavlink_msg_npfg_status_pack(system_id, component_id, msg, npfg_status->timestamp, npfg_status->wind_est_valid, npfg_status->lat_accel, npfg_status->lat_accel_ff, npfg_status->bearing_feas, npfg_status->bearing_feas_on_track, npfg_status->signed_track_error, npfg_status->track_error_bound, npfg_status->airspeed_ref, npfg_status->bearing, npfg_status->heading_ref, npfg_status->min_ground_speed_ref, npfg_status->adapted_period, npfg_status->p_gain, npfg_status->time_const);
}

/**
 * @brief Encode a npfg_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param npfg_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_npfg_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_npfg_status_t* npfg_status)
{
    return mavlink_msg_npfg_status_pack_chan(system_id, component_id, chan, msg, npfg_status->timestamp, npfg_status->wind_est_valid, npfg_status->lat_accel, npfg_status->lat_accel_ff, npfg_status->bearing_feas, npfg_status->bearing_feas_on_track, npfg_status->signed_track_error, npfg_status->track_error_bound, npfg_status->airspeed_ref, npfg_status->bearing, npfg_status->heading_ref, npfg_status->min_ground_speed_ref, npfg_status->adapted_period, npfg_status->p_gain, npfg_status->time_const);
}

/**
 * @brief Send a npfg_status message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp [us] Timestamp
 * @param wind_est_valid  Wind estimate is valid and being used
 * @param lat_accel [m/s^2] Resultant lateral acceleration reference
 * @param lat_accel_ff [m/s^2] Feed-forward lateral acceleration for maintaining path curvature
 * @param bearing_feas  Bearing feasibility
 * @param bearing_feas_on_track  On-track bearing feasibility
 * @param signed_track_error [m] Signed track error
 * @param track_error_bound [m] Track error bound
 * @param airspeed_ref [m/s] Airspeed reference
 * @param bearing [deg] Bearing angle
 * @param heading_ref [deg] Heading angle reference
 * @param min_ground_speed_ref [m/s] Minimum forward ground speed reference
 * @param adapted_period [s] Adapted period (if auto-tuning enabled)
 * @param p_gain [rad/s] Controller proportional gain
 * @param time_const [s] Controller time constant
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_npfg_status_send(mavlink_channel_t chan, uint64_t timestamp, uint8_t wind_est_valid, float lat_accel, float lat_accel_ff, float bearing_feas, float bearing_feas_on_track, float signed_track_error, float track_error_bound, float airspeed_ref, float bearing, float heading_ref, float min_ground_speed_ref, float adapted_period, float p_gain, float time_const)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_NPFG_STATUS_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, lat_accel);
    _mav_put_float(buf, 12, lat_accel_ff);
    _mav_put_float(buf, 16, bearing_feas);
    _mav_put_float(buf, 20, bearing_feas_on_track);
    _mav_put_float(buf, 24, signed_track_error);
    _mav_put_float(buf, 28, track_error_bound);
    _mav_put_float(buf, 32, airspeed_ref);
    _mav_put_float(buf, 36, bearing);
    _mav_put_float(buf, 40, heading_ref);
    _mav_put_float(buf, 44, min_ground_speed_ref);
    _mav_put_float(buf, 48, adapted_period);
    _mav_put_float(buf, 52, p_gain);
    _mav_put_float(buf, 56, time_const);
    _mav_put_uint8_t(buf, 60, wind_est_valid);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NPFG_STATUS, buf, MAVLINK_MSG_ID_NPFG_STATUS_MIN_LEN, MAVLINK_MSG_ID_NPFG_STATUS_LEN, MAVLINK_MSG_ID_NPFG_STATUS_CRC);
#else
    mavlink_npfg_status_t packet;
    packet.timestamp = timestamp;
    packet.lat_accel = lat_accel;
    packet.lat_accel_ff = lat_accel_ff;
    packet.bearing_feas = bearing_feas;
    packet.bearing_feas_on_track = bearing_feas_on_track;
    packet.signed_track_error = signed_track_error;
    packet.track_error_bound = track_error_bound;
    packet.airspeed_ref = airspeed_ref;
    packet.bearing = bearing;
    packet.heading_ref = heading_ref;
    packet.min_ground_speed_ref = min_ground_speed_ref;
    packet.adapted_period = adapted_period;
    packet.p_gain = p_gain;
    packet.time_const = time_const;
    packet.wind_est_valid = wind_est_valid;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NPFG_STATUS, (const char *)&packet, MAVLINK_MSG_ID_NPFG_STATUS_MIN_LEN, MAVLINK_MSG_ID_NPFG_STATUS_LEN, MAVLINK_MSG_ID_NPFG_STATUS_CRC);
#endif
}

/**
 * @brief Send a npfg_status message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_npfg_status_send_struct(mavlink_channel_t chan, const mavlink_npfg_status_t* npfg_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_npfg_status_send(chan, npfg_status->timestamp, npfg_status->wind_est_valid, npfg_status->lat_accel, npfg_status->lat_accel_ff, npfg_status->bearing_feas, npfg_status->bearing_feas_on_track, npfg_status->signed_track_error, npfg_status->track_error_bound, npfg_status->airspeed_ref, npfg_status->bearing, npfg_status->heading_ref, npfg_status->min_ground_speed_ref, npfg_status->adapted_period, npfg_status->p_gain, npfg_status->time_const);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NPFG_STATUS, (const char *)npfg_status, MAVLINK_MSG_ID_NPFG_STATUS_MIN_LEN, MAVLINK_MSG_ID_NPFG_STATUS_LEN, MAVLINK_MSG_ID_NPFG_STATUS_CRC);
#endif
}

#if MAVLINK_MSG_ID_NPFG_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_npfg_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp, uint8_t wind_est_valid, float lat_accel, float lat_accel_ff, float bearing_feas, float bearing_feas_on_track, float signed_track_error, float track_error_bound, float airspeed_ref, float bearing, float heading_ref, float min_ground_speed_ref, float adapted_period, float p_gain, float time_const)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, lat_accel);
    _mav_put_float(buf, 12, lat_accel_ff);
    _mav_put_float(buf, 16, bearing_feas);
    _mav_put_float(buf, 20, bearing_feas_on_track);
    _mav_put_float(buf, 24, signed_track_error);
    _mav_put_float(buf, 28, track_error_bound);
    _mav_put_float(buf, 32, airspeed_ref);
    _mav_put_float(buf, 36, bearing);
    _mav_put_float(buf, 40, heading_ref);
    _mav_put_float(buf, 44, min_ground_speed_ref);
    _mav_put_float(buf, 48, adapted_period);
    _mav_put_float(buf, 52, p_gain);
    _mav_put_float(buf, 56, time_const);
    _mav_put_uint8_t(buf, 60, wind_est_valid);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NPFG_STATUS, buf, MAVLINK_MSG_ID_NPFG_STATUS_MIN_LEN, MAVLINK_MSG_ID_NPFG_STATUS_LEN, MAVLINK_MSG_ID_NPFG_STATUS_CRC);
#else
    mavlink_npfg_status_t *packet = (mavlink_npfg_status_t *)msgbuf;
    packet->timestamp = timestamp;
    packet->lat_accel = lat_accel;
    packet->lat_accel_ff = lat_accel_ff;
    packet->bearing_feas = bearing_feas;
    packet->bearing_feas_on_track = bearing_feas_on_track;
    packet->signed_track_error = signed_track_error;
    packet->track_error_bound = track_error_bound;
    packet->airspeed_ref = airspeed_ref;
    packet->bearing = bearing;
    packet->heading_ref = heading_ref;
    packet->min_ground_speed_ref = min_ground_speed_ref;
    packet->adapted_period = adapted_period;
    packet->p_gain = p_gain;
    packet->time_const = time_const;
    packet->wind_est_valid = wind_est_valid;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NPFG_STATUS, (const char *)packet, MAVLINK_MSG_ID_NPFG_STATUS_MIN_LEN, MAVLINK_MSG_ID_NPFG_STATUS_LEN, MAVLINK_MSG_ID_NPFG_STATUS_CRC);
#endif
}
#endif

#endif

// MESSAGE NPFG_STATUS UNPACKING


/**
 * @brief Get field timestamp from npfg_status message
 *
 * @return [us] Timestamp
 */
static inline uint64_t mavlink_msg_npfg_status_get_timestamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field wind_est_valid from npfg_status message
 *
 * @return  Wind estimate is valid and being used
 */
static inline uint8_t mavlink_msg_npfg_status_get_wind_est_valid(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  60);
}

/**
 * @brief Get field lat_accel from npfg_status message
 *
 * @return [m/s^2] Resultant lateral acceleration reference
 */
static inline float mavlink_msg_npfg_status_get_lat_accel(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field lat_accel_ff from npfg_status message
 *
 * @return [m/s^2] Feed-forward lateral acceleration for maintaining path curvature
 */
static inline float mavlink_msg_npfg_status_get_lat_accel_ff(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field bearing_feas from npfg_status message
 *
 * @return  Bearing feasibility
 */
static inline float mavlink_msg_npfg_status_get_bearing_feas(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field bearing_feas_on_track from npfg_status message
 *
 * @return  On-track bearing feasibility
 */
static inline float mavlink_msg_npfg_status_get_bearing_feas_on_track(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field signed_track_error from npfg_status message
 *
 * @return [m] Signed track error
 */
static inline float mavlink_msg_npfg_status_get_signed_track_error(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field track_error_bound from npfg_status message
 *
 * @return [m] Track error bound
 */
static inline float mavlink_msg_npfg_status_get_track_error_bound(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field airspeed_ref from npfg_status message
 *
 * @return [m/s] Airspeed reference
 */
static inline float mavlink_msg_npfg_status_get_airspeed_ref(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field bearing from npfg_status message
 *
 * @return [deg] Bearing angle
 */
static inline float mavlink_msg_npfg_status_get_bearing(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Get field heading_ref from npfg_status message
 *
 * @return [deg] Heading angle reference
 */
static inline float mavlink_msg_npfg_status_get_heading_ref(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  40);
}

/**
 * @brief Get field min_ground_speed_ref from npfg_status message
 *
 * @return [m/s] Minimum forward ground speed reference
 */
static inline float mavlink_msg_npfg_status_get_min_ground_speed_ref(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  44);
}

/**
 * @brief Get field adapted_period from npfg_status message
 *
 * @return [s] Adapted period (if auto-tuning enabled)
 */
static inline float mavlink_msg_npfg_status_get_adapted_period(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  48);
}

/**
 * @brief Get field p_gain from npfg_status message
 *
 * @return [rad/s] Controller proportional gain
 */
static inline float mavlink_msg_npfg_status_get_p_gain(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  52);
}

/**
 * @brief Get field time_const from npfg_status message
 *
 * @return [s] Controller time constant
 */
static inline float mavlink_msg_npfg_status_get_time_const(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  56);
}

/**
 * @brief Decode a npfg_status message into a struct
 *
 * @param msg The message to decode
 * @param npfg_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_npfg_status_decode(const mavlink_message_t* msg, mavlink_npfg_status_t* npfg_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    npfg_status->timestamp = mavlink_msg_npfg_status_get_timestamp(msg);
    npfg_status->lat_accel = mavlink_msg_npfg_status_get_lat_accel(msg);
    npfg_status->lat_accel_ff = mavlink_msg_npfg_status_get_lat_accel_ff(msg);
    npfg_status->bearing_feas = mavlink_msg_npfg_status_get_bearing_feas(msg);
    npfg_status->bearing_feas_on_track = mavlink_msg_npfg_status_get_bearing_feas_on_track(msg);
    npfg_status->signed_track_error = mavlink_msg_npfg_status_get_signed_track_error(msg);
    npfg_status->track_error_bound = mavlink_msg_npfg_status_get_track_error_bound(msg);
    npfg_status->airspeed_ref = mavlink_msg_npfg_status_get_airspeed_ref(msg);
    npfg_status->bearing = mavlink_msg_npfg_status_get_bearing(msg);
    npfg_status->heading_ref = mavlink_msg_npfg_status_get_heading_ref(msg);
    npfg_status->min_ground_speed_ref = mavlink_msg_npfg_status_get_min_ground_speed_ref(msg);
    npfg_status->adapted_period = mavlink_msg_npfg_status_get_adapted_period(msg);
    npfg_status->p_gain = mavlink_msg_npfg_status_get_p_gain(msg);
    npfg_status->time_const = mavlink_msg_npfg_status_get_time_const(msg);
    npfg_status->wind_est_valid = mavlink_msg_npfg_status_get_wind_est_valid(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_NPFG_STATUS_LEN? msg->len : MAVLINK_MSG_ID_NPFG_STATUS_LEN;
        memset(npfg_status, 0, MAVLINK_MSG_ID_NPFG_STATUS_LEN);
    memcpy(npfg_status, _MAV_PAYLOAD(msg), len);
#endif
}
