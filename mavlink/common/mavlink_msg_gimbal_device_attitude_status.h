#pragma once
// MESSAGE GIMBAL_DEVICE_ATTITUDE_STATUS PACKING

#define MAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS 285


typedef struct __mavlink_gimbal_device_attitude_status_t {
 uint32_t time_boot_ms; /*< [ms] Timestamp (time since system boot).*/
 float q[4]; /*<  Quaternion components, w, x, y, z (1 0 0 0 is the null-rotation, the frame is depends on whether the flag GIMBAL_DEVICE_FLAGS_YAW_LOCK is set)*/
 float angular_velocity_x; /*< [rad/s] X component of angular velocity (NaN if unknown)*/
 float angular_velocity_y; /*< [rad/s] Y component of angular velocity (NaN if unknown)*/
 float angular_velocity_z; /*< [rad/s] Z component of angular velocity (NaN if unknown)*/
 uint32_t failure_flags; /*<  Failure flags (0 for no failure)*/
 uint16_t flags; /*<  Current gimbal flags set.*/
 Uint8_t target_system; /*<  System ID*/
 Uint8_t target_component; /*<  Component ID*/
} mavlink_gimbal_device_attitude_status_t;

#define MAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS_LEN 40
#define MAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS_MIN_LEN 40
#define MAVLINK_MSG_ID_285_LEN 40
#define MAVLINK_MSG_ID_285_MIN_LEN 40

#define MAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS_CRC 137
#define MAVLINK_MSG_ID_285_CRC 137

#define MAVLINK_MSG_GIMBAL_DEVICE_ATTITUDE_STATUS_FIELD_Q_LEN 4

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_GIMBAL_DEVICE_ATTITUDE_STATUS { \
    285, \
    "GIMBAL_DEVICE_ATTITUDE_STATUS", \
    9, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 38, offsetof(mavlink_gimbal_device_attitude_status_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 39, offsetof(mavlink_gimbal_device_attitude_status_t, target_component) }, \
         { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_gimbal_device_attitude_status_t, time_boot_ms) }, \
         { "flags", NULL, MAVLINK_TYPE_UINT16_T, 0, 36, offsetof(mavlink_gimbal_device_attitude_status_t, flags) }, \
         { "q", NULL, MAVLINK_TYPE_FLOAT, 4, 4, offsetof(mavlink_gimbal_device_attitude_status_t, q) }, \
         { "angular_velocity_x", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_gimbal_device_attitude_status_t, angular_velocity_x) }, \
         { "angular_velocity_y", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_gimbal_device_attitude_status_t, angular_velocity_y) }, \
         { "angular_velocity_z", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_gimbal_device_attitude_status_t, angular_velocity_z) }, \
         { "failure_flags", NULL, MAVLINK_TYPE_UINT32_T, 0, 32, offsetof(mavlink_gimbal_device_attitude_status_t, failure_flags) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_GIMBAL_DEVICE_ATTITUDE_STATUS { \
    "GIMBAL_DEVICE_ATTITUDE_STATUS", \
    9, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 38, offsetof(mavlink_gimbal_device_attitude_status_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 39, offsetof(mavlink_gimbal_device_attitude_status_t, target_component) }, \
         { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_gimbal_device_attitude_status_t, time_boot_ms) }, \
         { "flags", NULL, MAVLINK_TYPE_UINT16_T, 0, 36, offsetof(mavlink_gimbal_device_attitude_status_t, flags) }, \
         { "q", NULL, MAVLINK_TYPE_FLOAT, 4, 4, offsetof(mavlink_gimbal_device_attitude_status_t, q) }, \
         { "angular_velocity_x", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_gimbal_device_attitude_status_t, angular_velocity_x) }, \
         { "angular_velocity_y", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_gimbal_device_attitude_status_t, angular_velocity_y) }, \
         { "angular_velocity_z", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_gimbal_device_attitude_status_t, angular_velocity_z) }, \
         { "failure_flags", NULL, MAVLINK_TYPE_UINT32_T, 0, 32, offsetof(mavlink_gimbal_device_attitude_status_t, failure_flags) }, \
         } \
}
#endif

/**
 * @brief Pack a gimbal_device_attitude_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param flags  Current gimbal flags set.
 * @param q  Quaternion components, w, x, y, z (1 0 0 0 is the null-rotation, the frame is depends on whether the flag GIMBAL_DEVICE_FLAGS_YAW_LOCK is set)
 * @param angular_velocity_x [rad/s] X component of angular velocity (NaN if unknown)
 * @param angular_velocity_y [rad/s] Y component of angular velocity (NaN if unknown)
 * @param angular_velocity_z [rad/s] Z component of angular velocity (NaN if unknown)
 * @param failure_flags  Failure flags (0 for no failure)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gimbal_device_attitude_status_pack(Uint8_t system_id, Uint8_t component_id, mavlink_message_t* msg,
                               Uint8_t target_system, Uint8_t target_component, uint32_t time_boot_ms, uint16_t flags, const float *q, float angular_velocity_x, float angular_velocity_y, float angular_velocity_z, uint32_t failure_flags)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 20, angular_velocity_x);
    _mav_put_float(buf, 24, angular_velocity_y);
    _mav_put_float(buf, 28, angular_velocity_z);
    _mav_put_uint32_t(buf, 32, failure_flags);
    _mav_put_uint16_t(buf, 36, flags);
    _mav_put_uint8_t(buf, 38, target_system);
    _mav_put_uint8_t(buf, 39, target_component);
    _mav_put_float_array(buf, 4, q, 4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS_LEN);
#else
    mavlink_gimbal_device_attitude_status_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.angular_velocity_x = angular_velocity_x;
    packet.angular_velocity_y = angular_velocity_y;
    packet.angular_velocity_z = angular_velocity_z;
    packet.failure_flags = failure_flags;
    packet.flags = flags;
    packet.target_system = target_system;
    packet.target_component = target_component;
    mav_array_memcpy(packet.q, q, sizeof(float)*4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS_MIN_LEN, MAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS_LEN, MAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS_CRC);
}

/**
 * @brief Pack a gimbal_device_attitude_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param flags  Current gimbal flags set.
 * @param q  Quaternion components, w, x, y, z (1 0 0 0 is the null-rotation, the frame is depends on whether the flag GIMBAL_DEVICE_FLAGS_YAW_LOCK is set)
 * @param angular_velocity_x [rad/s] X component of angular velocity (NaN if unknown)
 * @param angular_velocity_y [rad/s] Y component of angular velocity (NaN if unknown)
 * @param angular_velocity_z [rad/s] Z component of angular velocity (NaN if unknown)
 * @param failure_flags  Failure flags (0 for no failure)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gimbal_device_attitude_status_pack_chan(Uint8_t system_id, Uint8_t component_id, Uint8_t chan,
                               mavlink_message_t* msg,
                                   Uint8_t target_system,Uint8_t target_component,uint32_t time_boot_ms,uint16_t flags,const float *q,float angular_velocity_x,float angular_velocity_y,float angular_velocity_z,uint32_t failure_flags)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 20, angular_velocity_x);
    _mav_put_float(buf, 24, angular_velocity_y);
    _mav_put_float(buf, 28, angular_velocity_z);
    _mav_put_uint32_t(buf, 32, failure_flags);
    _mav_put_uint16_t(buf, 36, flags);
    _mav_put_uint8_t(buf, 38, target_system);
    _mav_put_uint8_t(buf, 39, target_component);
    _mav_put_float_array(buf, 4, q, 4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS_LEN);
#else
    mavlink_gimbal_device_attitude_status_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.angular_velocity_x = angular_velocity_x;
    packet.angular_velocity_y = angular_velocity_y;
    packet.angular_velocity_z = angular_velocity_z;
    packet.failure_flags = failure_flags;
    packet.flags = flags;
    packet.target_system = target_system;
    packet.target_component = target_component;
    mav_array_memcpy(packet.q, q, sizeof(float)*4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS_MIN_LEN, MAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS_LEN, MAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS_CRC);
}

/**
 * @brief Encode a gimbal_device_attitude_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param gimbal_device_attitude_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gimbal_device_attitude_status_encode(Uint8_t system_id, Uint8_t component_id, mavlink_message_t* msg, const mavlink_gimbal_device_attitude_status_t* gimbal_device_attitude_status)
{
    return mavlink_msg_gimbal_device_attitude_status_pack(system_id, component_id, msg, gimbal_device_attitude_status->target_system, gimbal_device_attitude_status->target_component, gimbal_device_attitude_status->time_boot_ms, gimbal_device_attitude_status->flags, gimbal_device_attitude_status->q, gimbal_device_attitude_status->angular_velocity_x, gimbal_device_attitude_status->angular_velocity_y, gimbal_device_attitude_status->angular_velocity_z, gimbal_device_attitude_status->failure_flags);
}

/**
 * @brief Encode a gimbal_device_attitude_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param gimbal_device_attitude_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gimbal_device_attitude_status_encode_chan(Uint8_t system_id, Uint8_t component_id, Uint8_t chan, mavlink_message_t* msg, const mavlink_gimbal_device_attitude_status_t* gimbal_device_attitude_status)
{
    return mavlink_msg_gimbal_device_attitude_status_pack_chan(system_id, component_id, chan, msg, gimbal_device_attitude_status->target_system, gimbal_device_attitude_status->target_component, gimbal_device_attitude_status->time_boot_ms, gimbal_device_attitude_status->flags, gimbal_device_attitude_status->q, gimbal_device_attitude_status->angular_velocity_x, gimbal_device_attitude_status->angular_velocity_y, gimbal_device_attitude_status->angular_velocity_z, gimbal_device_attitude_status->failure_flags);
}

/**
 * @brief Send a gimbal_device_attitude_status message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param flags  Current gimbal flags set.
 * @param q  Quaternion components, w, x, y, z (1 0 0 0 is the null-rotation, the frame is depends on whether the flag GIMBAL_DEVICE_FLAGS_YAW_LOCK is set)
 * @param angular_velocity_x [rad/s] X component of angular velocity (NaN if unknown)
 * @param angular_velocity_y [rad/s] Y component of angular velocity (NaN if unknown)
 * @param angular_velocity_z [rad/s] Z component of angular velocity (NaN if unknown)
 * @param failure_flags  Failure flags (0 for no failure)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_gimbal_device_attitude_status_send(mavlink_channel_t chan, Uint8_t target_system, Uint8_t target_component, uint32_t time_boot_ms, uint16_t flags, const float *q, float angular_velocity_x, float angular_velocity_y, float angular_velocity_z, uint32_t failure_flags)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 20, angular_velocity_x);
    _mav_put_float(buf, 24, angular_velocity_y);
    _mav_put_float(buf, 28, angular_velocity_z);
    _mav_put_uint32_t(buf, 32, failure_flags);
    _mav_put_uint16_t(buf, 36, flags);
    _mav_put_uint8_t(buf, 38, target_system);
    _mav_put_uint8_t(buf, 39, target_component);
    _mav_put_float_array(buf, 4, q, 4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS, buf, MAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS_MIN_LEN, MAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS_LEN, MAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS_CRC);
#else
    mavlink_gimbal_device_attitude_status_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.angular_velocity_x = angular_velocity_x;
    packet.angular_velocity_y = angular_velocity_y;
    packet.angular_velocity_z = angular_velocity_z;
    packet.failure_flags = failure_flags;
    packet.flags = flags;
    packet.target_system = target_system;
    packet.target_component = target_component;
    mav_array_memcpy(packet.q, q, sizeof(float)*4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS, (const char *)&packet, MAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS_MIN_LEN, MAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS_LEN, MAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS_CRC);
#endif
}

/**
 * @brief Send a gimbal_device_attitude_status message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_gimbal_device_attitude_status_send_struct(mavlink_channel_t chan, const mavlink_gimbal_device_attitude_status_t* gimbal_device_attitude_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_gimbal_device_attitude_status_send(chan, gimbal_device_attitude_status->target_system, gimbal_device_attitude_status->target_component, gimbal_device_attitude_status->time_boot_ms, gimbal_device_attitude_status->flags, gimbal_device_attitude_status->q, gimbal_device_attitude_status->angular_velocity_x, gimbal_device_attitude_status->angular_velocity_y, gimbal_device_attitude_status->angular_velocity_z, gimbal_device_attitude_status->failure_flags);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS, (const char *)gimbal_device_attitude_status, MAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS_MIN_LEN, MAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS_LEN, MAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS_CRC);
#endif
}

#if MAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_gimbal_device_attitude_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  Uint8_t target_system, Uint8_t target_component, uint32_t time_boot_ms, uint16_t flags, const float *q, float angular_velocity_x, float angular_velocity_y, float angular_velocity_z, uint32_t failure_flags)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 20, angular_velocity_x);
    _mav_put_float(buf, 24, angular_velocity_y);
    _mav_put_float(buf, 28, angular_velocity_z);
    _mav_put_uint32_t(buf, 32, failure_flags);
    _mav_put_uint16_t(buf, 36, flags);
    _mav_put_uint8_t(buf, 38, target_system);
    _mav_put_uint8_t(buf, 39, target_component);
    _mav_put_float_array(buf, 4, q, 4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS, buf, MAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS_MIN_LEN, MAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS_LEN, MAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS_CRC);
#else
    mavlink_gimbal_device_attitude_status_t *packet = (mavlink_gimbal_device_attitude_status_t *)msgbuf;
    packet->time_boot_ms = time_boot_ms;
    packet->angular_velocity_x = angular_velocity_x;
    packet->angular_velocity_y = angular_velocity_y;
    packet->angular_velocity_z = angular_velocity_z;
    packet->failure_flags = failure_flags;
    packet->flags = flags;
    packet->target_system = target_system;
    packet->target_component = target_component;
    mav_array_memcpy(packet->q, q, sizeof(float)*4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS, (const char *)packet, MAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS_MIN_LEN, MAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS_LEN, MAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS_CRC);
#endif
}
#endif

#endif

// MESSAGE GIMBAL_DEVICE_ATTITUDE_STATUS UNPACKING


/**
 * @brief Get field target_system from gimbal_device_attitude_status message
 *
 * @return  System ID
 */
static inline Uint8_t mavlink_msg_gimbal_device_attitude_status_get_target_system(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  38);
}

/**
 * @brief Get field target_component from gimbal_device_attitude_status message
 *
 * @return  Component ID
 */
static inline Uint8_t mavlink_msg_gimbal_device_attitude_status_get_target_component(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  39);
}

/**
 * @brief Get field time_boot_ms from gimbal_device_attitude_status message
 *
 * @return [ms] Timestamp (time since system boot).
 */
static inline uint32_t mavlink_msg_gimbal_device_attitude_status_get_time_boot_ms(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field flags from gimbal_device_attitude_status message
 *
 * @return  Current gimbal flags set.
 */
static inline uint16_t mavlink_msg_gimbal_device_attitude_status_get_flags(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  36);
}

/**
 * @brief Get field q from gimbal_device_attitude_status message
 *
 * @return  Quaternion components, w, x, y, z (1 0 0 0 is the null-rotation, the frame is depends on whether the flag GIMBAL_DEVICE_FLAGS_YAW_LOCK is set)
 */
static inline uint16_t mavlink_msg_gimbal_device_attitude_status_get_q(const mavlink_message_t* msg, float *q)
{
    return _MAV_RETURN_float_array(msg, q, 4,  4);
}

/**
 * @brief Get field angular_velocity_x from gimbal_device_attitude_status message
 *
 * @return [rad/s] X component of angular velocity (NaN if unknown)
 */
static inline float mavlink_msg_gimbal_device_attitude_status_get_angular_velocity_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field angular_velocity_y from gimbal_device_attitude_status message
 *
 * @return [rad/s] Y component of angular velocity (NaN if unknown)
 */
static inline float mavlink_msg_gimbal_device_attitude_status_get_angular_velocity_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field angular_velocity_z from gimbal_device_attitude_status message
 *
 * @return [rad/s] Z component of angular velocity (NaN if unknown)
 */
static inline float mavlink_msg_gimbal_device_attitude_status_get_angular_velocity_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field failure_flags from gimbal_device_attitude_status message
 *
 * @return  Failure flags (0 for no failure)
 */
static inline uint32_t mavlink_msg_gimbal_device_attitude_status_get_failure_flags(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  32);
}

/**
 * @brief Decode a gimbal_device_attitude_status message into a struct
 *
 * @param msg The message to decode
 * @param gimbal_device_attitude_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_gimbal_device_attitude_status_decode(const mavlink_message_t* msg, mavlink_gimbal_device_attitude_status_t* gimbal_device_attitude_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    gimbal_device_attitude_status->time_boot_ms = mavlink_msg_gimbal_device_attitude_status_get_time_boot_ms(msg);
    mavlink_msg_gimbal_device_attitude_status_get_q(msg, gimbal_device_attitude_status->q);
    gimbal_device_attitude_status->angular_velocity_x = mavlink_msg_gimbal_device_attitude_status_get_angular_velocity_x(msg);
    gimbal_device_attitude_status->angular_velocity_y = mavlink_msg_gimbal_device_attitude_status_get_angular_velocity_y(msg);
    gimbal_device_attitude_status->angular_velocity_z = mavlink_msg_gimbal_device_attitude_status_get_angular_velocity_z(msg);
    gimbal_device_attitude_status->failure_flags = mavlink_msg_gimbal_device_attitude_status_get_failure_flags(msg);
    gimbal_device_attitude_status->flags = mavlink_msg_gimbal_device_attitude_status_get_flags(msg);
    gimbal_device_attitude_status->target_system = mavlink_msg_gimbal_device_attitude_status_get_target_system(msg);
    gimbal_device_attitude_status->target_component = mavlink_msg_gimbal_device_attitude_status_get_target_component(msg);
#else
        Uint8_t len = msg->len < MAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS_LEN? msg->len : MAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS_LEN;
        memset(gimbal_device_attitude_status, 0, MAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS_LEN);
    memcpy(gimbal_device_attitude_status, _MAV_PAYLOAD(msg), len);
#endif
}
