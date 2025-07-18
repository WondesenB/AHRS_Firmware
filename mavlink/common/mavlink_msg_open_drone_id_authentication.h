#pragma once
// MESSAGE OPEN_DRONE_ID_AUTHENTICATION PACKING

#define MAVLINK_MSG_ID_OPEN_DRONE_ID_AUTHENTICATION 12902


typedef struct __mavlink_open_drone_id_authentication_t {
 uint32_t timestamp; /*< [s] This field is only present for page 0. 32 bit Unix Timestamp in seconds since 00:00:00 01/01/2019.*/
 Uint8_t target_system; /*<  System ID (0 for broadcast).*/
 Uint8_t target_component; /*<  Component ID (0 for broadcast).*/
 Uint8_t id_or_mac[20]; /*<  Only used for drone ID data received from other UAs. See detailed description at https://mavlink.io/en/services/opendroneid.html. */
 Uint8_t authentication_type; /*<  Indicates the type of authentication.*/
 Uint8_t data_page; /*<  Allowed range is 0 - 4.*/
 Uint8_t page_count; /*<  This field is only present for page 0. Allowed range is 0 - 5.*/
 Uint8_t length; /*< [bytes] This field is only present for page 0. Total bytes of authentication_data from all data pages. Allowed range is 0 - 109 (17 + 23*4).*/
 Uint8_t authentication_data[23]; /*<  Opaque authentication data. For page 0, the size is only 17 bytes. For other pages, the size is 23 bytes. Shall be filled with nulls in the unused portion of the field.*/
} mavlink_open_drone_id_authentication_t;

#define MAVLINK_MSG_ID_OPEN_DRONE_ID_AUTHENTICATION_LEN 53
#define MAVLINK_MSG_ID_OPEN_DRONE_ID_AUTHENTICATION_MIN_LEN 53
#define MAVLINK_MSG_ID_12902_LEN 53
#define MAVLINK_MSG_ID_12902_MIN_LEN 53

#define MAVLINK_MSG_ID_OPEN_DRONE_ID_AUTHENTICATION_CRC 49
#define MAVLINK_MSG_ID_12902_CRC 49

#define MAVLINK_MSG_OPEN_DRONE_ID_AUTHENTICATION_FIELD_ID_OR_MAC_LEN 20
#define MAVLINK_MSG_OPEN_DRONE_ID_AUTHENTICATION_FIELD_AUTHENTICATION_DATA_LEN 23

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_OPEN_DRONE_ID_AUTHENTICATION { \
    12902, \
    "OPEN_DRONE_ID_AUTHENTICATION", \
    9, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_open_drone_id_authentication_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 5, offsetof(mavlink_open_drone_id_authentication_t, target_component) }, \
         { "id_or_mac", NULL, MAVLINK_TYPE_UINT8_T, 20, 6, offsetof(mavlink_open_drone_id_authentication_t, id_or_mac) }, \
         { "authentication_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 26, offsetof(mavlink_open_drone_id_authentication_t, authentication_type) }, \
         { "data_page", NULL, MAVLINK_TYPE_UINT8_T, 0, 27, offsetof(mavlink_open_drone_id_authentication_t, data_page) }, \
         { "page_count", NULL, MAVLINK_TYPE_UINT8_T, 0, 28, offsetof(mavlink_open_drone_id_authentication_t, page_count) }, \
         { "length", NULL, MAVLINK_TYPE_UINT8_T, 0, 29, offsetof(mavlink_open_drone_id_authentication_t, length) }, \
         { "timestamp", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_open_drone_id_authentication_t, timestamp) }, \
         { "authentication_data", NULL, MAVLINK_TYPE_UINT8_T, 23, 30, offsetof(mavlink_open_drone_id_authentication_t, authentication_data) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_OPEN_DRONE_ID_AUTHENTICATION { \
    "OPEN_DRONE_ID_AUTHENTICATION", \
    9, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_open_drone_id_authentication_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 5, offsetof(mavlink_open_drone_id_authentication_t, target_component) }, \
         { "id_or_mac", NULL, MAVLINK_TYPE_UINT8_T, 20, 6, offsetof(mavlink_open_drone_id_authentication_t, id_or_mac) }, \
         { "authentication_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 26, offsetof(mavlink_open_drone_id_authentication_t, authentication_type) }, \
         { "data_page", NULL, MAVLINK_TYPE_UINT8_T, 0, 27, offsetof(mavlink_open_drone_id_authentication_t, data_page) }, \
         { "page_count", NULL, MAVLINK_TYPE_UINT8_T, 0, 28, offsetof(mavlink_open_drone_id_authentication_t, page_count) }, \
         { "length", NULL, MAVLINK_TYPE_UINT8_T, 0, 29, offsetof(mavlink_open_drone_id_authentication_t, length) }, \
         { "timestamp", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_open_drone_id_authentication_t, timestamp) }, \
         { "authentication_data", NULL, MAVLINK_TYPE_UINT8_T, 23, 30, offsetof(mavlink_open_drone_id_authentication_t, authentication_data) }, \
         } \
}
#endif

/**
 * @brief Pack a open_drone_id_authentication message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system  System ID (0 for broadcast).
 * @param target_component  Component ID (0 for broadcast).
 * @param id_or_mac  Only used for drone ID data received from other UAs. See detailed description at https://mavlink.io/en/services/opendroneid.html. 
 * @param authentication_type  Indicates the type of authentication.
 * @param data_page  Allowed range is 0 - 4.
 * @param page_count  This field is only present for page 0. Allowed range is 0 - 5.
 * @param length [bytes] This field is only present for page 0. Total bytes of authentication_data from all data pages. Allowed range is 0 - 109 (17 + 23*4).
 * @param timestamp [s] This field is only present for page 0. 32 bit Unix Timestamp in seconds since 00:00:00 01/01/2019.
 * @param authentication_data  Opaque authentication data. For page 0, the size is only 17 bytes. For other pages, the size is 23 bytes. Shall be filled with nulls in the unused portion of the field.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_open_drone_id_authentication_pack(Uint8_t system_id, Uint8_t component_id, mavlink_message_t* msg,
                               Uint8_t target_system, Uint8_t target_component, const Uint8_t *id_or_mac, Uint8_t authentication_type, Uint8_t data_page, Uint8_t page_count, Uint8_t length, uint32_t timestamp, const Uint8_t *authentication_data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_OPEN_DRONE_ID_AUTHENTICATION_LEN];
    _mav_put_uint32_t(buf, 0, timestamp);
    _mav_put_uint8_t(buf, 4, target_system);
    _mav_put_uint8_t(buf, 5, target_component);
    _mav_put_uint8_t(buf, 26, authentication_type);
    _mav_put_uint8_t(buf, 27, data_page);
    _mav_put_uint8_t(buf, 28, page_count);
    _mav_put_uint8_t(buf, 29, length);
    _mav_put_uint8_t_array(buf, 6, id_or_mac, 20);
    _mav_put_uint8_t_array(buf, 30, authentication_data, 23);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_OPEN_DRONE_ID_AUTHENTICATION_LEN);
#else
    mavlink_open_drone_id_authentication_t packet;
    packet.timestamp = timestamp;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.authentication_type = authentication_type;
    packet.data_page = data_page;
    packet.page_count = page_count;
    packet.length = length;
    mav_array_memcpy(packet.id_or_mac, id_or_mac, sizeof(Uint8_t)*20);
    mav_array_memcpy(packet.authentication_data, authentication_data, sizeof(Uint8_t)*23);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_OPEN_DRONE_ID_AUTHENTICATION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_OPEN_DRONE_ID_AUTHENTICATION;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_OPEN_DRONE_ID_AUTHENTICATION_MIN_LEN, MAVLINK_MSG_ID_OPEN_DRONE_ID_AUTHENTICATION_LEN, MAVLINK_MSG_ID_OPEN_DRONE_ID_AUTHENTICATION_CRC);
}

/**
 * @brief Pack a open_drone_id_authentication message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system  System ID (0 for broadcast).
 * @param target_component  Component ID (0 for broadcast).
 * @param id_or_mac  Only used for drone ID data received from other UAs. See detailed description at https://mavlink.io/en/services/opendroneid.html. 
 * @param authentication_type  Indicates the type of authentication.
 * @param data_page  Allowed range is 0 - 4.
 * @param page_count  This field is only present for page 0. Allowed range is 0 - 5.
 * @param length [bytes] This field is only present for page 0. Total bytes of authentication_data from all data pages. Allowed range is 0 - 109 (17 + 23*4).
 * @param timestamp [s] This field is only present for page 0. 32 bit Unix Timestamp in seconds since 00:00:00 01/01/2019.
 * @param authentication_data  Opaque authentication data. For page 0, the size is only 17 bytes. For other pages, the size is 23 bytes. Shall be filled with nulls in the unused portion of the field.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_open_drone_id_authentication_pack_chan(Uint8_t system_id, Uint8_t component_id, Uint8_t chan,
                               mavlink_message_t* msg,
                                   Uint8_t target_system,Uint8_t target_component,const Uint8_t *id_or_mac,Uint8_t authentication_type,Uint8_t data_page,Uint8_t page_count,Uint8_t length,uint32_t timestamp,const Uint8_t *authentication_data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_OPEN_DRONE_ID_AUTHENTICATION_LEN];
    _mav_put_uint32_t(buf, 0, timestamp);
    _mav_put_uint8_t(buf, 4, target_system);
    _mav_put_uint8_t(buf, 5, target_component);
    _mav_put_uint8_t(buf, 26, authentication_type);
    _mav_put_uint8_t(buf, 27, data_page);
    _mav_put_uint8_t(buf, 28, page_count);
    _mav_put_uint8_t(buf, 29, length);
    _mav_put_uint8_t_array(buf, 6, id_or_mac, 20);
    _mav_put_uint8_t_array(buf, 30, authentication_data, 23);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_OPEN_DRONE_ID_AUTHENTICATION_LEN);
#else
    mavlink_open_drone_id_authentication_t packet;
    packet.timestamp = timestamp;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.authentication_type = authentication_type;
    packet.data_page = data_page;
    packet.page_count = page_count;
    packet.length = length;
    mav_array_memcpy(packet.id_or_mac, id_or_mac, sizeof(Uint8_t)*20);
    mav_array_memcpy(packet.authentication_data, authentication_data, sizeof(Uint8_t)*23);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_OPEN_DRONE_ID_AUTHENTICATION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_OPEN_DRONE_ID_AUTHENTICATION;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_OPEN_DRONE_ID_AUTHENTICATION_MIN_LEN, MAVLINK_MSG_ID_OPEN_DRONE_ID_AUTHENTICATION_LEN, MAVLINK_MSG_ID_OPEN_DRONE_ID_AUTHENTICATION_CRC);
}

/**
 * @brief Encode a open_drone_id_authentication struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param open_drone_id_authentication C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_open_drone_id_authentication_encode(Uint8_t system_id, Uint8_t component_id, mavlink_message_t* msg, const mavlink_open_drone_id_authentication_t* open_drone_id_authentication)
{
    return mavlink_msg_open_drone_id_authentication_pack(system_id, component_id, msg, open_drone_id_authentication->target_system, open_drone_id_authentication->target_component, open_drone_id_authentication->id_or_mac, open_drone_id_authentication->authentication_type, open_drone_id_authentication->data_page, open_drone_id_authentication->page_count, open_drone_id_authentication->length, open_drone_id_authentication->timestamp, open_drone_id_authentication->authentication_data);
}

/**
 * @brief Encode a open_drone_id_authentication struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param open_drone_id_authentication C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_open_drone_id_authentication_encode_chan(Uint8_t system_id, Uint8_t component_id, Uint8_t chan, mavlink_message_t* msg, const mavlink_open_drone_id_authentication_t* open_drone_id_authentication)
{
    return mavlink_msg_open_drone_id_authentication_pack_chan(system_id, component_id, chan, msg, open_drone_id_authentication->target_system, open_drone_id_authentication->target_component, open_drone_id_authentication->id_or_mac, open_drone_id_authentication->authentication_type, open_drone_id_authentication->data_page, open_drone_id_authentication->page_count, open_drone_id_authentication->length, open_drone_id_authentication->timestamp, open_drone_id_authentication->authentication_data);
}

/**
 * @brief Send a open_drone_id_authentication message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system  System ID (0 for broadcast).
 * @param target_component  Component ID (0 for broadcast).
 * @param id_or_mac  Only used for drone ID data received from other UAs. See detailed description at https://mavlink.io/en/services/opendroneid.html. 
 * @param authentication_type  Indicates the type of authentication.
 * @param data_page  Allowed range is 0 - 4.
 * @param page_count  This field is only present for page 0. Allowed range is 0 - 5.
 * @param length [bytes] This field is only present for page 0. Total bytes of authentication_data from all data pages. Allowed range is 0 - 109 (17 + 23*4).
 * @param timestamp [s] This field is only present for page 0. 32 bit Unix Timestamp in seconds since 00:00:00 01/01/2019.
 * @param authentication_data  Opaque authentication data. For page 0, the size is only 17 bytes. For other pages, the size is 23 bytes. Shall be filled with nulls in the unused portion of the field.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_open_drone_id_authentication_send(mavlink_channel_t chan, Uint8_t target_system, Uint8_t target_component, const Uint8_t *id_or_mac, Uint8_t authentication_type, Uint8_t data_page, Uint8_t page_count, Uint8_t length, uint32_t timestamp, const Uint8_t *authentication_data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_OPEN_DRONE_ID_AUTHENTICATION_LEN];
    _mav_put_uint32_t(buf, 0, timestamp);
    _mav_put_uint8_t(buf, 4, target_system);
    _mav_put_uint8_t(buf, 5, target_component);
    _mav_put_uint8_t(buf, 26, authentication_type);
    _mav_put_uint8_t(buf, 27, data_page);
    _mav_put_uint8_t(buf, 28, page_count);
    _mav_put_uint8_t(buf, 29, length);
    _mav_put_uint8_t_array(buf, 6, id_or_mac, 20);
    _mav_put_uint8_t_array(buf, 30, authentication_data, 23);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OPEN_DRONE_ID_AUTHENTICATION, buf, MAVLINK_MSG_ID_OPEN_DRONE_ID_AUTHENTICATION_MIN_LEN, MAVLINK_MSG_ID_OPEN_DRONE_ID_AUTHENTICATION_LEN, MAVLINK_MSG_ID_OPEN_DRONE_ID_AUTHENTICATION_CRC);
#else
    mavlink_open_drone_id_authentication_t packet;
    packet.timestamp = timestamp;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.authentication_type = authentication_type;
    packet.data_page = data_page;
    packet.page_count = page_count;
    packet.length = length;
    mav_array_memcpy(packet.id_or_mac, id_or_mac, sizeof(Uint8_t)*20);
    mav_array_memcpy(packet.authentication_data, authentication_data, sizeof(Uint8_t)*23);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OPEN_DRONE_ID_AUTHENTICATION, (const char *)&packet, MAVLINK_MSG_ID_OPEN_DRONE_ID_AUTHENTICATION_MIN_LEN, MAVLINK_MSG_ID_OPEN_DRONE_ID_AUTHENTICATION_LEN, MAVLINK_MSG_ID_OPEN_DRONE_ID_AUTHENTICATION_CRC);
#endif
}

/**
 * @brief Send a open_drone_id_authentication message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_open_drone_id_authentication_send_struct(mavlink_channel_t chan, const mavlink_open_drone_id_authentication_t* open_drone_id_authentication)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_open_drone_id_authentication_send(chan, open_drone_id_authentication->target_system, open_drone_id_authentication->target_component, open_drone_id_authentication->id_or_mac, open_drone_id_authentication->authentication_type, open_drone_id_authentication->data_page, open_drone_id_authentication->page_count, open_drone_id_authentication->length, open_drone_id_authentication->timestamp, open_drone_id_authentication->authentication_data);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OPEN_DRONE_ID_AUTHENTICATION, (const char *)open_drone_id_authentication, MAVLINK_MSG_ID_OPEN_DRONE_ID_AUTHENTICATION_MIN_LEN, MAVLINK_MSG_ID_OPEN_DRONE_ID_AUTHENTICATION_LEN, MAVLINK_MSG_ID_OPEN_DRONE_ID_AUTHENTICATION_CRC);
#endif
}

#if MAVLINK_MSG_ID_OPEN_DRONE_ID_AUTHENTICATION_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_open_drone_id_authentication_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  Uint8_t target_system, Uint8_t target_component, const Uint8_t *id_or_mac, Uint8_t authentication_type, Uint8_t data_page, Uint8_t page_count, Uint8_t length, uint32_t timestamp, const Uint8_t *authentication_data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, timestamp);
    _mav_put_uint8_t(buf, 4, target_system);
    _mav_put_uint8_t(buf, 5, target_component);
    _mav_put_uint8_t(buf, 26, authentication_type);
    _mav_put_uint8_t(buf, 27, data_page);
    _mav_put_uint8_t(buf, 28, page_count);
    _mav_put_uint8_t(buf, 29, length);
    _mav_put_uint8_t_array(buf, 6, id_or_mac, 20);
    _mav_put_uint8_t_array(buf, 30, authentication_data, 23);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OPEN_DRONE_ID_AUTHENTICATION, buf, MAVLINK_MSG_ID_OPEN_DRONE_ID_AUTHENTICATION_MIN_LEN, MAVLINK_MSG_ID_OPEN_DRONE_ID_AUTHENTICATION_LEN, MAVLINK_MSG_ID_OPEN_DRONE_ID_AUTHENTICATION_CRC);
#else
    mavlink_open_drone_id_authentication_t *packet = (mavlink_open_drone_id_authentication_t *)msgbuf;
    packet->timestamp = timestamp;
    packet->target_system = target_system;
    packet->target_component = target_component;
    packet->authentication_type = authentication_type;
    packet->data_page = data_page;
    packet->page_count = page_count;
    packet->length = length;
    mav_array_memcpy(packet->id_or_mac, id_or_mac, sizeof(Uint8_t)*20);
    mav_array_memcpy(packet->authentication_data, authentication_data, sizeof(Uint8_t)*23);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OPEN_DRONE_ID_AUTHENTICATION, (const char *)packet, MAVLINK_MSG_ID_OPEN_DRONE_ID_AUTHENTICATION_MIN_LEN, MAVLINK_MSG_ID_OPEN_DRONE_ID_AUTHENTICATION_LEN, MAVLINK_MSG_ID_OPEN_DRONE_ID_AUTHENTICATION_CRC);
#endif
}
#endif

#endif

// MESSAGE OPEN_DRONE_ID_AUTHENTICATION UNPACKING


/**
 * @brief Get field target_system from open_drone_id_authentication message
 *
 * @return  System ID (0 for broadcast).
 */
static inline Uint8_t mavlink_msg_open_drone_id_authentication_get_target_system(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field target_component from open_drone_id_authentication message
 *
 * @return  Component ID (0 for broadcast).
 */
static inline Uint8_t mavlink_msg_open_drone_id_authentication_get_target_component(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  5);
}

/**
 * @brief Get field id_or_mac from open_drone_id_authentication message
 *
 * @return  Only used for drone ID data received from other UAs. See detailed description at https://mavlink.io/en/services/opendroneid.html. 
 */
static inline uint16_t mavlink_msg_open_drone_id_authentication_get_id_or_mac(const mavlink_message_t* msg, Uint8_t *id_or_mac)
{
    return _MAV_RETURN_uint8_t_array(msg, id_or_mac, 20,  6);
}

/**
 * @brief Get field authentication_type from open_drone_id_authentication message
 *
 * @return  Indicates the type of authentication.
 */
static inline Uint8_t mavlink_msg_open_drone_id_authentication_get_authentication_type(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  26);
}

/**
 * @brief Get field data_page from open_drone_id_authentication message
 *
 * @return  Allowed range is 0 - 4.
 */
static inline Uint8_t mavlink_msg_open_drone_id_authentication_get_data_page(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  27);
}

/**
 * @brief Get field page_count from open_drone_id_authentication message
 *
 * @return  This field is only present for page 0. Allowed range is 0 - 5.
 */
static inline Uint8_t mavlink_msg_open_drone_id_authentication_get_page_count(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  28);
}

/**
 * @brief Get field length from open_drone_id_authentication message
 *
 * @return [bytes] This field is only present for page 0. Total bytes of authentication_data from all data pages. Allowed range is 0 - 109 (17 + 23*4).
 */
static inline Uint8_t mavlink_msg_open_drone_id_authentication_get_length(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  29);
}

/**
 * @brief Get field timestamp from open_drone_id_authentication message
 *
 * @return [s] This field is only present for page 0. 32 bit Unix Timestamp in seconds since 00:00:00 01/01/2019.
 */
static inline uint32_t mavlink_msg_open_drone_id_authentication_get_timestamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field authentication_data from open_drone_id_authentication message
 *
 * @return  Opaque authentication data. For page 0, the size is only 17 bytes. For other pages, the size is 23 bytes. Shall be filled with nulls in the unused portion of the field.
 */
static inline uint16_t mavlink_msg_open_drone_id_authentication_get_authentication_data(const mavlink_message_t* msg, Uint8_t *authentication_data)
{
    return _MAV_RETURN_uint8_t_array(msg, authentication_data, 23,  30);
}

/**
 * @brief Decode a open_drone_id_authentication message into a struct
 *
 * @param msg The message to decode
 * @param open_drone_id_authentication C-struct to decode the message contents into
 */
static inline void mavlink_msg_open_drone_id_authentication_decode(const mavlink_message_t* msg, mavlink_open_drone_id_authentication_t* open_drone_id_authentication)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    open_drone_id_authentication->timestamp = mavlink_msg_open_drone_id_authentication_get_timestamp(msg);
    open_drone_id_authentication->target_system = mavlink_msg_open_drone_id_authentication_get_target_system(msg);
    open_drone_id_authentication->target_component = mavlink_msg_open_drone_id_authentication_get_target_component(msg);
    mavlink_msg_open_drone_id_authentication_get_id_or_mac(msg, open_drone_id_authentication->id_or_mac);
    open_drone_id_authentication->authentication_type = mavlink_msg_open_drone_id_authentication_get_authentication_type(msg);
    open_drone_id_authentication->data_page = mavlink_msg_open_drone_id_authentication_get_data_page(msg);
    open_drone_id_authentication->page_count = mavlink_msg_open_drone_id_authentication_get_page_count(msg);
    open_drone_id_authentication->length = mavlink_msg_open_drone_id_authentication_get_length(msg);
    mavlink_msg_open_drone_id_authentication_get_authentication_data(msg, open_drone_id_authentication->authentication_data);
#else
        Uint8_t len = msg->len < MAVLINK_MSG_ID_OPEN_DRONE_ID_AUTHENTICATION_LEN? msg->len : MAVLINK_MSG_ID_OPEN_DRONE_ID_AUTHENTICATION_LEN;
        memset(open_drone_id_authentication, 0, MAVLINK_MSG_ID_OPEN_DRONE_ID_AUTHENTICATION_LEN);
    memcpy(open_drone_id_authentication, _MAV_PAYLOAD(msg), len);
#endif
}
