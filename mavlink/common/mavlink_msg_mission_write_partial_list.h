#pragma once
// MESSAGE MISSION_WRITE_PARTIAL_LIST PACKING

#define MAVLINK_MSG_ID_MISSION_WRITE_PARTIAL_LIST 38


typedef struct __mavlink_mission_write_partial_list_t {
 int16_t start_index; /*<  Start index. Must be smaller / equal to the largest index of the current onboard list.*/
 int16_t end_index; /*<  End index, equal or greater than start index.*/
 Uint8_t target_system; /*<  System ID*/
 Uint8_t target_component; /*<  Component ID*/
 Uint8_t mission_type; /*<  Mission type.*/
} mavlink_mission_write_partial_list_t;

#define MAVLINK_MSG_ID_MISSION_WRITE_PARTIAL_LIST_LEN 7
#define MAVLINK_MSG_ID_MISSION_WRITE_PARTIAL_LIST_MIN_LEN 6
#define MAVLINK_MSG_ID_38_LEN 7
#define MAVLINK_MSG_ID_38_MIN_LEN 6

#define MAVLINK_MSG_ID_MISSION_WRITE_PARTIAL_LIST_CRC 9
#define MAVLINK_MSG_ID_38_CRC 9



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_MISSION_WRITE_PARTIAL_LIST { \
    38, \
    "MISSION_WRITE_PARTIAL_LIST", \
    5, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_mission_write_partial_list_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 5, offsetof(mavlink_mission_write_partial_list_t, target_component) }, \
         { "start_index", NULL, MAVLINK_TYPE_INT16_T, 0, 0, offsetof(mavlink_mission_write_partial_list_t, start_index) }, \
         { "end_index", NULL, MAVLINK_TYPE_INT16_T, 0, 2, offsetof(mavlink_mission_write_partial_list_t, end_index) }, \
         { "mission_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 6, offsetof(mavlink_mission_write_partial_list_t, mission_type) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_MISSION_WRITE_PARTIAL_LIST { \
    "MISSION_WRITE_PARTIAL_LIST", \
    5, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_mission_write_partial_list_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 5, offsetof(mavlink_mission_write_partial_list_t, target_component) }, \
         { "start_index", NULL, MAVLINK_TYPE_INT16_T, 0, 0, offsetof(mavlink_mission_write_partial_list_t, start_index) }, \
         { "end_index", NULL, MAVLINK_TYPE_INT16_T, 0, 2, offsetof(mavlink_mission_write_partial_list_t, end_index) }, \
         { "mission_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 6, offsetof(mavlink_mission_write_partial_list_t, mission_type) }, \
         } \
}
#endif

/**
 * @brief Pack a mission_write_partial_list message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param start_index  Start index. Must be smaller / equal to the largest index of the current onboard list.
 * @param end_index  End index, equal or greater than start index.
 * @param mission_type  Mission type.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mission_write_partial_list_pack(Uint8_t system_id, Uint8_t component_id, mavlink_message_t* msg,
                               Uint8_t target_system, Uint8_t target_component, int16_t start_index, int16_t end_index, Uint8_t mission_type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MISSION_WRITE_PARTIAL_LIST_LEN];
    _mav_put_int16_t(buf, 0, start_index);
    _mav_put_int16_t(buf, 2, end_index);
    _mav_put_uint8_t(buf, 4, target_system);
    _mav_put_uint8_t(buf, 5, target_component);
    _mav_put_uint8_t(buf, 6, mission_type);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MISSION_WRITE_PARTIAL_LIST_LEN);
#else
    mavlink_mission_write_partial_list_t packet;
    packet.start_index = start_index;
    packet.end_index = end_index;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.mission_type = mission_type;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MISSION_WRITE_PARTIAL_LIST_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MISSION_WRITE_PARTIAL_LIST;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MISSION_WRITE_PARTIAL_LIST_MIN_LEN, MAVLINK_MSG_ID_MISSION_WRITE_PARTIAL_LIST_LEN, MAVLINK_MSG_ID_MISSION_WRITE_PARTIAL_LIST_CRC);
}

/**
 * @brief Pack a mission_write_partial_list message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param start_index  Start index. Must be smaller / equal to the largest index of the current onboard list.
 * @param end_index  End index, equal or greater than start index.
 * @param mission_type  Mission type.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mission_write_partial_list_pack_chan(Uint8_t system_id, Uint8_t component_id, Uint8_t chan,
                               mavlink_message_t* msg,
                                   Uint8_t target_system,Uint8_t target_component,int16_t start_index,int16_t end_index,Uint8_t mission_type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MISSION_WRITE_PARTIAL_LIST_LEN];
    _mav_put_int16_t(buf, 0, start_index);
    _mav_put_int16_t(buf, 2, end_index);
    _mav_put_uint8_t(buf, 4, target_system);
    _mav_put_uint8_t(buf, 5, target_component);
    _mav_put_uint8_t(buf, 6, mission_type);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MISSION_WRITE_PARTIAL_LIST_LEN);
#else
    mavlink_mission_write_partial_list_t packet;
    packet.start_index = start_index;
    packet.end_index = end_index;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.mission_type = mission_type;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MISSION_WRITE_PARTIAL_LIST_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MISSION_WRITE_PARTIAL_LIST;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MISSION_WRITE_PARTIAL_LIST_MIN_LEN, MAVLINK_MSG_ID_MISSION_WRITE_PARTIAL_LIST_LEN, MAVLINK_MSG_ID_MISSION_WRITE_PARTIAL_LIST_CRC);
}

/**
 * @brief Encode a mission_write_partial_list struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param mission_write_partial_list C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mission_write_partial_list_encode(Uint8_t system_id, Uint8_t component_id, mavlink_message_t* msg, const mavlink_mission_write_partial_list_t* mission_write_partial_list)
{
    return mavlink_msg_mission_write_partial_list_pack(system_id, component_id, msg, mission_write_partial_list->target_system, mission_write_partial_list->target_component, mission_write_partial_list->start_index, mission_write_partial_list->end_index, mission_write_partial_list->mission_type);
}

/**
 * @brief Encode a mission_write_partial_list struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param mission_write_partial_list C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mission_write_partial_list_encode_chan(Uint8_t system_id, Uint8_t component_id, Uint8_t chan, mavlink_message_t* msg, const mavlink_mission_write_partial_list_t* mission_write_partial_list)
{
    return mavlink_msg_mission_write_partial_list_pack_chan(system_id, component_id, chan, msg, mission_write_partial_list->target_system, mission_write_partial_list->target_component, mission_write_partial_list->start_index, mission_write_partial_list->end_index, mission_write_partial_list->mission_type);
}

/**
 * @brief Send a mission_write_partial_list message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param start_index  Start index. Must be smaller / equal to the largest index of the current onboard list.
 * @param end_index  End index, equal or greater than start index.
 * @param mission_type  Mission type.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_mission_write_partial_list_send(mavlink_channel_t chan, Uint8_t target_system, Uint8_t target_component, int16_t start_index, int16_t end_index, Uint8_t mission_type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MISSION_WRITE_PARTIAL_LIST_LEN];
    _mav_put_int16_t(buf, 0, start_index);
    _mav_put_int16_t(buf, 2, end_index);
    _mav_put_uint8_t(buf, 4, target_system);
    _mav_put_uint8_t(buf, 5, target_component);
    _mav_put_uint8_t(buf, 6, mission_type);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MISSION_WRITE_PARTIAL_LIST, buf, MAVLINK_MSG_ID_MISSION_WRITE_PARTIAL_LIST_MIN_LEN, MAVLINK_MSG_ID_MISSION_WRITE_PARTIAL_LIST_LEN, MAVLINK_MSG_ID_MISSION_WRITE_PARTIAL_LIST_CRC);
#else
    mavlink_mission_write_partial_list_t packet;
    packet.start_index = start_index;
    packet.end_index = end_index;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.mission_type = mission_type;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MISSION_WRITE_PARTIAL_LIST, (const char *)&packet, MAVLINK_MSG_ID_MISSION_WRITE_PARTIAL_LIST_MIN_LEN, MAVLINK_MSG_ID_MISSION_WRITE_PARTIAL_LIST_LEN, MAVLINK_MSG_ID_MISSION_WRITE_PARTIAL_LIST_CRC);
#endif
}

/**
 * @brief Send a mission_write_partial_list message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_mission_write_partial_list_send_struct(mavlink_channel_t chan, const mavlink_mission_write_partial_list_t* mission_write_partial_list)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_mission_write_partial_list_send(chan, mission_write_partial_list->target_system, mission_write_partial_list->target_component, mission_write_partial_list->start_index, mission_write_partial_list->end_index, mission_write_partial_list->mission_type);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MISSION_WRITE_PARTIAL_LIST, (const char *)mission_write_partial_list, MAVLINK_MSG_ID_MISSION_WRITE_PARTIAL_LIST_MIN_LEN, MAVLINK_MSG_ID_MISSION_WRITE_PARTIAL_LIST_LEN, MAVLINK_MSG_ID_MISSION_WRITE_PARTIAL_LIST_CRC);
#endif
}

#if MAVLINK_MSG_ID_MISSION_WRITE_PARTIAL_LIST_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_mission_write_partial_list_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  Uint8_t target_system, Uint8_t target_component, int16_t start_index, int16_t end_index, Uint8_t mission_type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_int16_t(buf, 0, start_index);
    _mav_put_int16_t(buf, 2, end_index);
    _mav_put_uint8_t(buf, 4, target_system);
    _mav_put_uint8_t(buf, 5, target_component);
    _mav_put_uint8_t(buf, 6, mission_type);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MISSION_WRITE_PARTIAL_LIST, buf, MAVLINK_MSG_ID_MISSION_WRITE_PARTIAL_LIST_MIN_LEN, MAVLINK_MSG_ID_MISSION_WRITE_PARTIAL_LIST_LEN, MAVLINK_MSG_ID_MISSION_WRITE_PARTIAL_LIST_CRC);
#else
    mavlink_mission_write_partial_list_t *packet = (mavlink_mission_write_partial_list_t *)msgbuf;
    packet->start_index = start_index;
    packet->end_index = end_index;
    packet->target_system = target_system;
    packet->target_component = target_component;
    packet->mission_type = mission_type;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MISSION_WRITE_PARTIAL_LIST, (const char *)packet, MAVLINK_MSG_ID_MISSION_WRITE_PARTIAL_LIST_MIN_LEN, MAVLINK_MSG_ID_MISSION_WRITE_PARTIAL_LIST_LEN, MAVLINK_MSG_ID_MISSION_WRITE_PARTIAL_LIST_CRC);
#endif
}
#endif

#endif

// MESSAGE MISSION_WRITE_PARTIAL_LIST UNPACKING


/**
 * @brief Get field target_system from mission_write_partial_list message
 *
 * @return  System ID
 */
static inline Uint8_t mavlink_msg_mission_write_partial_list_get_target_system(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field target_component from mission_write_partial_list message
 *
 * @return  Component ID
 */
static inline Uint8_t mavlink_msg_mission_write_partial_list_get_target_component(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  5);
}

/**
 * @brief Get field start_index from mission_write_partial_list message
 *
 * @return  Start index. Must be smaller / equal to the largest index of the current onboard list.
 */
static inline int16_t mavlink_msg_mission_write_partial_list_get_start_index(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  0);
}

/**
 * @brief Get field end_index from mission_write_partial_list message
 *
 * @return  End index, equal or greater than start index.
 */
static inline int16_t mavlink_msg_mission_write_partial_list_get_end_index(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  2);
}

/**
 * @brief Get field mission_type from mission_write_partial_list message
 *
 * @return  Mission type.
 */
static inline Uint8_t mavlink_msg_mission_write_partial_list_get_mission_type(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  6);
}

/**
 * @brief Decode a mission_write_partial_list message into a struct
 *
 * @param msg The message to decode
 * @param mission_write_partial_list C-struct to decode the message contents into
 */
static inline void mavlink_msg_mission_write_partial_list_decode(const mavlink_message_t* msg, mavlink_mission_write_partial_list_t* mission_write_partial_list)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mission_write_partial_list->start_index = mavlink_msg_mission_write_partial_list_get_start_index(msg);
    mission_write_partial_list->end_index = mavlink_msg_mission_write_partial_list_get_end_index(msg);
    mission_write_partial_list->target_system = mavlink_msg_mission_write_partial_list_get_target_system(msg);
    mission_write_partial_list->target_component = mavlink_msg_mission_write_partial_list_get_target_component(msg);
    mission_write_partial_list->mission_type = mavlink_msg_mission_write_partial_list_get_mission_type(msg);
#else
        Uint8_t len = msg->len < MAVLINK_MSG_ID_MISSION_WRITE_PARTIAL_LIST_LEN? msg->len : MAVLINK_MSG_ID_MISSION_WRITE_PARTIAL_LIST_LEN;
        memset(mission_write_partial_list, 0, MAVLINK_MSG_ID_MISSION_WRITE_PARTIAL_LIST_LEN);
    memcpy(mission_write_partial_list, _MAV_PAYLOAD(msg), len);
#endif
}
