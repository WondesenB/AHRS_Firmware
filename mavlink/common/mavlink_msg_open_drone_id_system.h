#pragma once
// MESSAGE OPEN_DRONE_ID_SYSTEM PACKING

#define MAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM 12904


typedef struct __mavlink_open_drone_id_system_t {
 int32_t operator_latitude; /*< [degE7] Latitude of the operator. If unknown: 0 (both Lat/Lon).*/
 int32_t operator_longitude; /*< [degE7] Longitude of the operator. If unknown: 0 (both Lat/Lon).*/
 float area_ceiling; /*< [m] Area Operations Ceiling relative to WGS84. If unknown: -1000 m.*/
 float area_floor; /*< [m] Area Operations Floor relative to WGS84. If unknown: -1000 m.*/
 uint16_t area_count; /*<  Number of aircraft in the area, group or formation (default 1).*/
 uint16_t area_radius; /*< [m] Radius of the cylindrical area of the group or formation (default 0).*/
 Uint8_t target_system; /*<  System ID (0 for broadcast).*/
 Uint8_t target_component; /*<  Component ID (0 for broadcast).*/
 Uint8_t id_or_mac[20]; /*<  Only used for drone ID data received from other UAs. See detailed description at https://mavlink.io/en/services/opendroneid.html. */
 Uint8_t operator_location_type; /*<  Specifies the operator location type.*/
 Uint8_t classification_type; /*<  Specifies the classification type of the UA.*/
 Uint8_t category_eu; /*<  When classification_type is MAV_ODID_CLASSIFICATION_TYPE_EU, specifies the category of the UA.*/
 Uint8_t class_eu; /*<  When classification_type is MAV_ODID_CLASSIFICATION_TYPE_EU, specifies the class of the UA.*/
} mavlink_open_drone_id_system_t;

#define MAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM_LEN 46
#define MAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM_MIN_LEN 46
#define MAVLINK_MSG_ID_12904_LEN 46
#define MAVLINK_MSG_ID_12904_MIN_LEN 46

#define MAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM_CRC 203
#define MAVLINK_MSG_ID_12904_CRC 203

#define MAVLINK_MSG_OPEN_DRONE_ID_SYSTEM_FIELD_ID_OR_MAC_LEN 20

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_OPEN_DRONE_ID_SYSTEM { \
    12904, \
    "OPEN_DRONE_ID_SYSTEM", \
    13, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 20, offsetof(mavlink_open_drone_id_system_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 21, offsetof(mavlink_open_drone_id_system_t, target_component) }, \
         { "id_or_mac", NULL, MAVLINK_TYPE_UINT8_T, 20, 22, offsetof(mavlink_open_drone_id_system_t, id_or_mac) }, \
         { "operator_location_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 42, offsetof(mavlink_open_drone_id_system_t, operator_location_type) }, \
         { "classification_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 43, offsetof(mavlink_open_drone_id_system_t, classification_type) }, \
         { "operator_latitude", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_open_drone_id_system_t, operator_latitude) }, \
         { "operator_longitude", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_open_drone_id_system_t, operator_longitude) }, \
         { "area_count", NULL, MAVLINK_TYPE_UINT16_T, 0, 16, offsetof(mavlink_open_drone_id_system_t, area_count) }, \
         { "area_radius", NULL, MAVLINK_TYPE_UINT16_T, 0, 18, offsetof(mavlink_open_drone_id_system_t, area_radius) }, \
         { "area_ceiling", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_open_drone_id_system_t, area_ceiling) }, \
         { "area_floor", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_open_drone_id_system_t, area_floor) }, \
         { "category_eu", NULL, MAVLINK_TYPE_UINT8_T, 0, 44, offsetof(mavlink_open_drone_id_system_t, category_eu) }, \
         { "class_eu", NULL, MAVLINK_TYPE_UINT8_T, 0, 45, offsetof(mavlink_open_drone_id_system_t, class_eu) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_OPEN_DRONE_ID_SYSTEM { \
    "OPEN_DRONE_ID_SYSTEM", \
    13, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 20, offsetof(mavlink_open_drone_id_system_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 21, offsetof(mavlink_open_drone_id_system_t, target_component) }, \
         { "id_or_mac", NULL, MAVLINK_TYPE_UINT8_T, 20, 22, offsetof(mavlink_open_drone_id_system_t, id_or_mac) }, \
         { "operator_location_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 42, offsetof(mavlink_open_drone_id_system_t, operator_location_type) }, \
         { "classification_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 43, offsetof(mavlink_open_drone_id_system_t, classification_type) }, \
         { "operator_latitude", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_open_drone_id_system_t, operator_latitude) }, \
         { "operator_longitude", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_open_drone_id_system_t, operator_longitude) }, \
         { "area_count", NULL, MAVLINK_TYPE_UINT16_T, 0, 16, offsetof(mavlink_open_drone_id_system_t, area_count) }, \
         { "area_radius", NULL, MAVLINK_TYPE_UINT16_T, 0, 18, offsetof(mavlink_open_drone_id_system_t, area_radius) }, \
         { "area_ceiling", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_open_drone_id_system_t, area_ceiling) }, \
         { "area_floor", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_open_drone_id_system_t, area_floor) }, \
         { "category_eu", NULL, MAVLINK_TYPE_UINT8_T, 0, 44, offsetof(mavlink_open_drone_id_system_t, category_eu) }, \
         { "class_eu", NULL, MAVLINK_TYPE_UINT8_T, 0, 45, offsetof(mavlink_open_drone_id_system_t, class_eu) }, \
         } \
}
#endif

/**
 * @brief Pack a open_drone_id_system message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system  System ID (0 for broadcast).
 * @param target_component  Component ID (0 for broadcast).
 * @param id_or_mac  Only used for drone ID data received from other UAs. See detailed description at https://mavlink.io/en/services/opendroneid.html. 
 * @param operator_location_type  Specifies the operator location type.
 * @param classification_type  Specifies the classification type of the UA.
 * @param operator_latitude [degE7] Latitude of the operator. If unknown: 0 (both Lat/Lon).
 * @param operator_longitude [degE7] Longitude of the operator. If unknown: 0 (both Lat/Lon).
 * @param area_count  Number of aircraft in the area, group or formation (default 1).
 * @param area_radius [m] Radius of the cylindrical area of the group or formation (default 0).
 * @param area_ceiling [m] Area Operations Ceiling relative to WGS84. If unknown: -1000 m.
 * @param area_floor [m] Area Operations Floor relative to WGS84. If unknown: -1000 m.
 * @param category_eu  When classification_type is MAV_ODID_CLASSIFICATION_TYPE_EU, specifies the category of the UA.
 * @param class_eu  When classification_type is MAV_ODID_CLASSIFICATION_TYPE_EU, specifies the class of the UA.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_open_drone_id_system_pack(Uint8_t system_id, Uint8_t component_id, mavlink_message_t* msg,
                               Uint8_t target_system, Uint8_t target_component, const Uint8_t *id_or_mac, Uint8_t operator_location_type, Uint8_t classification_type, int32_t operator_latitude, int32_t operator_longitude, uint16_t area_count, uint16_t area_radius, float area_ceiling, float area_floor, Uint8_t category_eu, Uint8_t class_eu)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM_LEN];
    _mav_put_int32_t(buf, 0, operator_latitude);
    _mav_put_int32_t(buf, 4, operator_longitude);
    _mav_put_float(buf, 8, area_ceiling);
    _mav_put_float(buf, 12, area_floor);
    _mav_put_uint16_t(buf, 16, area_count);
    _mav_put_uint16_t(buf, 18, area_radius);
    _mav_put_uint8_t(buf, 20, target_system);
    _mav_put_uint8_t(buf, 21, target_component);
    _mav_put_uint8_t(buf, 42, operator_location_type);
    _mav_put_uint8_t(buf, 43, classification_type);
    _mav_put_uint8_t(buf, 44, category_eu);
    _mav_put_uint8_t(buf, 45, class_eu);
    _mav_put_uint8_t_array(buf, 22, id_or_mac, 20);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM_LEN);
#else
    mavlink_open_drone_id_system_t packet;
    packet.operator_latitude = operator_latitude;
    packet.operator_longitude = operator_longitude;
    packet.area_ceiling = area_ceiling;
    packet.area_floor = area_floor;
    packet.area_count = area_count;
    packet.area_radius = area_radius;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.operator_location_type = operator_location_type;
    packet.classification_type = classification_type;
    packet.category_eu = category_eu;
    packet.class_eu = class_eu;
    mav_array_memcpy(packet.id_or_mac, id_or_mac, sizeof(Uint8_t)*20);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM_MIN_LEN, MAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM_LEN, MAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM_CRC);
}

/**
 * @brief Pack a open_drone_id_system message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system  System ID (0 for broadcast).
 * @param target_component  Component ID (0 for broadcast).
 * @param id_or_mac  Only used for drone ID data received from other UAs. See detailed description at https://mavlink.io/en/services/opendroneid.html. 
 * @param operator_location_type  Specifies the operator location type.
 * @param classification_type  Specifies the classification type of the UA.
 * @param operator_latitude [degE7] Latitude of the operator. If unknown: 0 (both Lat/Lon).
 * @param operator_longitude [degE7] Longitude of the operator. If unknown: 0 (both Lat/Lon).
 * @param area_count  Number of aircraft in the area, group or formation (default 1).
 * @param area_radius [m] Radius of the cylindrical area of the group or formation (default 0).
 * @param area_ceiling [m] Area Operations Ceiling relative to WGS84. If unknown: -1000 m.
 * @param area_floor [m] Area Operations Floor relative to WGS84. If unknown: -1000 m.
 * @param category_eu  When classification_type is MAV_ODID_CLASSIFICATION_TYPE_EU, specifies the category of the UA.
 * @param class_eu  When classification_type is MAV_ODID_CLASSIFICATION_TYPE_EU, specifies the class of the UA.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_open_drone_id_system_pack_chan(Uint8_t system_id, Uint8_t component_id, Uint8_t chan,
                               mavlink_message_t* msg,
                                   Uint8_t target_system,Uint8_t target_component,const Uint8_t *id_or_mac,Uint8_t operator_location_type,Uint8_t classification_type,int32_t operator_latitude,int32_t operator_longitude,uint16_t area_count,uint16_t area_radius,float area_ceiling,float area_floor,Uint8_t category_eu,Uint8_t class_eu)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM_LEN];
    _mav_put_int32_t(buf, 0, operator_latitude);
    _mav_put_int32_t(buf, 4, operator_longitude);
    _mav_put_float(buf, 8, area_ceiling);
    _mav_put_float(buf, 12, area_floor);
    _mav_put_uint16_t(buf, 16, area_count);
    _mav_put_uint16_t(buf, 18, area_radius);
    _mav_put_uint8_t(buf, 20, target_system);
    _mav_put_uint8_t(buf, 21, target_component);
    _mav_put_uint8_t(buf, 42, operator_location_type);
    _mav_put_uint8_t(buf, 43, classification_type);
    _mav_put_uint8_t(buf, 44, category_eu);
    _mav_put_uint8_t(buf, 45, class_eu);
    _mav_put_uint8_t_array(buf, 22, id_or_mac, 20);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM_LEN);
#else
    mavlink_open_drone_id_system_t packet;
    packet.operator_latitude = operator_latitude;
    packet.operator_longitude = operator_longitude;
    packet.area_ceiling = area_ceiling;
    packet.area_floor = area_floor;
    packet.area_count = area_count;
    packet.area_radius = area_radius;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.operator_location_type = operator_location_type;
    packet.classification_type = classification_type;
    packet.category_eu = category_eu;
    packet.class_eu = class_eu;
    mav_array_memcpy(packet.id_or_mac, id_or_mac, sizeof(Uint8_t)*20);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM_MIN_LEN, MAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM_LEN, MAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM_CRC);
}

/**
 * @brief Encode a open_drone_id_system struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param open_drone_id_system C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_open_drone_id_system_encode(Uint8_t system_id, Uint8_t component_id, mavlink_message_t* msg, const mavlink_open_drone_id_system_t* open_drone_id_system)
{
    return mavlink_msg_open_drone_id_system_pack(system_id, component_id, msg, open_drone_id_system->target_system, open_drone_id_system->target_component, open_drone_id_system->id_or_mac, open_drone_id_system->operator_location_type, open_drone_id_system->classification_type, open_drone_id_system->operator_latitude, open_drone_id_system->operator_longitude, open_drone_id_system->area_count, open_drone_id_system->area_radius, open_drone_id_system->area_ceiling, open_drone_id_system->area_floor, open_drone_id_system->category_eu, open_drone_id_system->class_eu);
}

/**
 * @brief Encode a open_drone_id_system struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param open_drone_id_system C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_open_drone_id_system_encode_chan(Uint8_t system_id, Uint8_t component_id, Uint8_t chan, mavlink_message_t* msg, const mavlink_open_drone_id_system_t* open_drone_id_system)
{
    return mavlink_msg_open_drone_id_system_pack_chan(system_id, component_id, chan, msg, open_drone_id_system->target_system, open_drone_id_system->target_component, open_drone_id_system->id_or_mac, open_drone_id_system->operator_location_type, open_drone_id_system->classification_type, open_drone_id_system->operator_latitude, open_drone_id_system->operator_longitude, open_drone_id_system->area_count, open_drone_id_system->area_radius, open_drone_id_system->area_ceiling, open_drone_id_system->area_floor, open_drone_id_system->category_eu, open_drone_id_system->class_eu);
}

/**
 * @brief Send a open_drone_id_system message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system  System ID (0 for broadcast).
 * @param target_component  Component ID (0 for broadcast).
 * @param id_or_mac  Only used for drone ID data received from other UAs. See detailed description at https://mavlink.io/en/services/opendroneid.html. 
 * @param operator_location_type  Specifies the operator location type.
 * @param classification_type  Specifies the classification type of the UA.
 * @param operator_latitude [degE7] Latitude of the operator. If unknown: 0 (both Lat/Lon).
 * @param operator_longitude [degE7] Longitude of the operator. If unknown: 0 (both Lat/Lon).
 * @param area_count  Number of aircraft in the area, group or formation (default 1).
 * @param area_radius [m] Radius of the cylindrical area of the group or formation (default 0).
 * @param area_ceiling [m] Area Operations Ceiling relative to WGS84. If unknown: -1000 m.
 * @param area_floor [m] Area Operations Floor relative to WGS84. If unknown: -1000 m.
 * @param category_eu  When classification_type is MAV_ODID_CLASSIFICATION_TYPE_EU, specifies the category of the UA.
 * @param class_eu  When classification_type is MAV_ODID_CLASSIFICATION_TYPE_EU, specifies the class of the UA.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_open_drone_id_system_send(mavlink_channel_t chan, Uint8_t target_system, Uint8_t target_component, const Uint8_t *id_or_mac, Uint8_t operator_location_type, Uint8_t classification_type, int32_t operator_latitude, int32_t operator_longitude, uint16_t area_count, uint16_t area_radius, float area_ceiling, float area_floor, Uint8_t category_eu, Uint8_t class_eu)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM_LEN];
    _mav_put_int32_t(buf, 0, operator_latitude);
    _mav_put_int32_t(buf, 4, operator_longitude);
    _mav_put_float(buf, 8, area_ceiling);
    _mav_put_float(buf, 12, area_floor);
    _mav_put_uint16_t(buf, 16, area_count);
    _mav_put_uint16_t(buf, 18, area_radius);
    _mav_put_uint8_t(buf, 20, target_system);
    _mav_put_uint8_t(buf, 21, target_component);
    _mav_put_uint8_t(buf, 42, operator_location_type);
    _mav_put_uint8_t(buf, 43, classification_type);
    _mav_put_uint8_t(buf, 44, category_eu);
    _mav_put_uint8_t(buf, 45, class_eu);
    _mav_put_uint8_t_array(buf, 22, id_or_mac, 20);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM, buf, MAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM_MIN_LEN, MAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM_LEN, MAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM_CRC);
#else
    mavlink_open_drone_id_system_t packet;
    packet.operator_latitude = operator_latitude;
    packet.operator_longitude = operator_longitude;
    packet.area_ceiling = area_ceiling;
    packet.area_floor = area_floor;
    packet.area_count = area_count;
    packet.area_radius = area_radius;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.operator_location_type = operator_location_type;
    packet.classification_type = classification_type;
    packet.category_eu = category_eu;
    packet.class_eu = class_eu;
    mav_array_memcpy(packet.id_or_mac, id_or_mac, sizeof(Uint8_t)*20);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM, (const char *)&packet, MAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM_MIN_LEN, MAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM_LEN, MAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM_CRC);
#endif
}

/**
 * @brief Send a open_drone_id_system message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_open_drone_id_system_send_struct(mavlink_channel_t chan, const mavlink_open_drone_id_system_t* open_drone_id_system)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_open_drone_id_system_send(chan, open_drone_id_system->target_system, open_drone_id_system->target_component, open_drone_id_system->id_or_mac, open_drone_id_system->operator_location_type, open_drone_id_system->classification_type, open_drone_id_system->operator_latitude, open_drone_id_system->operator_longitude, open_drone_id_system->area_count, open_drone_id_system->area_radius, open_drone_id_system->area_ceiling, open_drone_id_system->area_floor, open_drone_id_system->category_eu, open_drone_id_system->class_eu);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM, (const char *)open_drone_id_system, MAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM_MIN_LEN, MAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM_LEN, MAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM_CRC);
#endif
}

#if MAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_open_drone_id_system_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  Uint8_t target_system, Uint8_t target_component, const Uint8_t *id_or_mac, Uint8_t operator_location_type, Uint8_t classification_type, int32_t operator_latitude, int32_t operator_longitude, uint16_t area_count, uint16_t area_radius, float area_ceiling, float area_floor, Uint8_t category_eu, Uint8_t class_eu)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_int32_t(buf, 0, operator_latitude);
    _mav_put_int32_t(buf, 4, operator_longitude);
    _mav_put_float(buf, 8, area_ceiling);
    _mav_put_float(buf, 12, area_floor);
    _mav_put_uint16_t(buf, 16, area_count);
    _mav_put_uint16_t(buf, 18, area_radius);
    _mav_put_uint8_t(buf, 20, target_system);
    _mav_put_uint8_t(buf, 21, target_component);
    _mav_put_uint8_t(buf, 42, operator_location_type);
    _mav_put_uint8_t(buf, 43, classification_type);
    _mav_put_uint8_t(buf, 44, category_eu);
    _mav_put_uint8_t(buf, 45, class_eu);
    _mav_put_uint8_t_array(buf, 22, id_or_mac, 20);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM, buf, MAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM_MIN_LEN, MAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM_LEN, MAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM_CRC);
#else
    mavlink_open_drone_id_system_t *packet = (mavlink_open_drone_id_system_t *)msgbuf;
    packet->operator_latitude = operator_latitude;
    packet->operator_longitude = operator_longitude;
    packet->area_ceiling = area_ceiling;
    packet->area_floor = area_floor;
    packet->area_count = area_count;
    packet->area_radius = area_radius;
    packet->target_system = target_system;
    packet->target_component = target_component;
    packet->operator_location_type = operator_location_type;
    packet->classification_type = classification_type;
    packet->category_eu = category_eu;
    packet->class_eu = class_eu;
    mav_array_memcpy(packet->id_or_mac, id_or_mac, sizeof(Uint8_t)*20);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM, (const char *)packet, MAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM_MIN_LEN, MAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM_LEN, MAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM_CRC);
#endif
}
#endif

#endif

// MESSAGE OPEN_DRONE_ID_SYSTEM UNPACKING


/**
 * @brief Get field target_system from open_drone_id_system message
 *
 * @return  System ID (0 for broadcast).
 */
static inline Uint8_t mavlink_msg_open_drone_id_system_get_target_system(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  20);
}

/**
 * @brief Get field target_component from open_drone_id_system message
 *
 * @return  Component ID (0 for broadcast).
 */
static inline Uint8_t mavlink_msg_open_drone_id_system_get_target_component(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  21);
}

/**
 * @brief Get field id_or_mac from open_drone_id_system message
 *
 * @return  Only used for drone ID data received from other UAs. See detailed description at https://mavlink.io/en/services/opendroneid.html. 
 */
static inline uint16_t mavlink_msg_open_drone_id_system_get_id_or_mac(const mavlink_message_t* msg, Uint8_t *id_or_mac)
{
    return _MAV_RETURN_uint8_t_array(msg, id_or_mac, 20,  22);
}

/**
 * @brief Get field operator_location_type from open_drone_id_system message
 *
 * @return  Specifies the operator location type.
 */
static inline Uint8_t mavlink_msg_open_drone_id_system_get_operator_location_type(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  42);
}

/**
 * @brief Get field classification_type from open_drone_id_system message
 *
 * @return  Specifies the classification type of the UA.
 */
static inline Uint8_t mavlink_msg_open_drone_id_system_get_classification_type(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  43);
}

/**
 * @brief Get field operator_latitude from open_drone_id_system message
 *
 * @return [degE7] Latitude of the operator. If unknown: 0 (both Lat/Lon).
 */
static inline int32_t mavlink_msg_open_drone_id_system_get_operator_latitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  0);
}

/**
 * @brief Get field operator_longitude from open_drone_id_system message
 *
 * @return [degE7] Longitude of the operator. If unknown: 0 (both Lat/Lon).
 */
static inline int32_t mavlink_msg_open_drone_id_system_get_operator_longitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  4);
}

/**
 * @brief Get field area_count from open_drone_id_system message
 *
 * @return  Number of aircraft in the area, group or formation (default 1).
 */
static inline uint16_t mavlink_msg_open_drone_id_system_get_area_count(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  16);
}

/**
 * @brief Get field area_radius from open_drone_id_system message
 *
 * @return [m] Radius of the cylindrical area of the group or formation (default 0).
 */
static inline uint16_t mavlink_msg_open_drone_id_system_get_area_radius(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  18);
}

/**
 * @brief Get field area_ceiling from open_drone_id_system message
 *
 * @return [m] Area Operations Ceiling relative to WGS84. If unknown: -1000 m.
 */
static inline float mavlink_msg_open_drone_id_system_get_area_ceiling(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field area_floor from open_drone_id_system message
 *
 * @return [m] Area Operations Floor relative to WGS84. If unknown: -1000 m.
 */
static inline float mavlink_msg_open_drone_id_system_get_area_floor(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field category_eu from open_drone_id_system message
 *
 * @return  When classification_type is MAV_ODID_CLASSIFICATION_TYPE_EU, specifies the category of the UA.
 */
static inline Uint8_t mavlink_msg_open_drone_id_system_get_category_eu(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  44);
}

/**
 * @brief Get field class_eu from open_drone_id_system message
 *
 * @return  When classification_type is MAV_ODID_CLASSIFICATION_TYPE_EU, specifies the class of the UA.
 */
static inline Uint8_t mavlink_msg_open_drone_id_system_get_class_eu(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  45);
}

/**
 * @brief Decode a open_drone_id_system message into a struct
 *
 * @param msg The message to decode
 * @param open_drone_id_system C-struct to decode the message contents into
 */
static inline void mavlink_msg_open_drone_id_system_decode(const mavlink_message_t* msg, mavlink_open_drone_id_system_t* open_drone_id_system)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    open_drone_id_system->operator_latitude = mavlink_msg_open_drone_id_system_get_operator_latitude(msg);
    open_drone_id_system->operator_longitude = mavlink_msg_open_drone_id_system_get_operator_longitude(msg);
    open_drone_id_system->area_ceiling = mavlink_msg_open_drone_id_system_get_area_ceiling(msg);
    open_drone_id_system->area_floor = mavlink_msg_open_drone_id_system_get_area_floor(msg);
    open_drone_id_system->area_count = mavlink_msg_open_drone_id_system_get_area_count(msg);
    open_drone_id_system->area_radius = mavlink_msg_open_drone_id_system_get_area_radius(msg);
    open_drone_id_system->target_system = mavlink_msg_open_drone_id_system_get_target_system(msg);
    open_drone_id_system->target_component = mavlink_msg_open_drone_id_system_get_target_component(msg);
    mavlink_msg_open_drone_id_system_get_id_or_mac(msg, open_drone_id_system->id_or_mac);
    open_drone_id_system->operator_location_type = mavlink_msg_open_drone_id_system_get_operator_location_type(msg);
    open_drone_id_system->classification_type = mavlink_msg_open_drone_id_system_get_classification_type(msg);
    open_drone_id_system->category_eu = mavlink_msg_open_drone_id_system_get_category_eu(msg);
    open_drone_id_system->class_eu = mavlink_msg_open_drone_id_system_get_class_eu(msg);
#else
        Uint8_t len = msg->len < MAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM_LEN? msg->len : MAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM_LEN;
        memset(open_drone_id_system, 0, MAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM_LEN);
    memcpy(open_drone_id_system, _MAV_PAYLOAD(msg), len);
#endif
}
