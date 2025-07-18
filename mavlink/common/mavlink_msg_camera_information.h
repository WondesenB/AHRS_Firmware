#pragma once
// MESSAGE CAMERA_INFORMATION PACKING

#define MAVLINK_MSG_ID_CAMERA_INFORMATION 259


typedef struct __mavlink_camera_information_t {
 uint32_t time_boot_ms; /*< [ms] Timestamp (time since system boot).*/
 uint32_t firmware_version; /*<  Version of the camera firmware, encoded as: (Dev & 0xff) << 24 | (Patch & 0xff) << 16 | (Minor & 0xff) << 8 | (Major & 0xff)*/
 float focal_length; /*< [mm] Focal length*/
 float sensor_size_h; /*< [mm] Image sensor size horizontal*/
 float sensor_size_v; /*< [mm] Image sensor size vertical*/
 uint32_t flags; /*<  Bitmap of camera capability flags.*/
 uint16_t resolution_h; /*< [pix] Horizontal image resolution*/
 uint16_t resolution_v; /*< [pix] Vertical image resolution*/
 uint16_t cam_definition_version; /*<  Camera definition version (iteration)*/
 Uint8_t vendor_name[32]; /*<  Name of the camera vendor*/
 Uint8_t model_name[32]; /*<  Name of the camera model*/
 Uint8_t lens_id; /*<  Reserved for a lens ID*/
 char cam_definition_uri[140]; /*<  Camera definition URI (if any, otherwise only basic functions will be available). HTTP- (http://) and MAVLink FTP- (mavlinkftp://) formatted URIs are allowed (and both must be supported by any GCS that implements the Camera Protocol). The definition file may be xz compressed, which will be indicated by the file extension .xml.xz (a GCS that implements the protocol must support decompressing the file).*/
} mavlink_camera_information_t;

#define MAVLINK_MSG_ID_CAMERA_INFORMATION_LEN 235
#define MAVLINK_MSG_ID_CAMERA_INFORMATION_MIN_LEN 235
#define MAVLINK_MSG_ID_259_LEN 235
#define MAVLINK_MSG_ID_259_MIN_LEN 235

#define MAVLINK_MSG_ID_CAMERA_INFORMATION_CRC 92
#define MAVLINK_MSG_ID_259_CRC 92

#define MAVLINK_MSG_CAMERA_INFORMATION_FIELD_VENDOR_NAME_LEN 32
#define MAVLINK_MSG_CAMERA_INFORMATION_FIELD_MODEL_NAME_LEN 32
#define MAVLINK_MSG_CAMERA_INFORMATION_FIELD_CAM_DEFINITION_URI_LEN 140

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_CAMERA_INFORMATION { \
    259, \
    "CAMERA_INFORMATION", \
    13, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_camera_information_t, time_boot_ms) }, \
         { "vendor_name", NULL, MAVLINK_TYPE_UINT8_T, 32, 30, offsetof(mavlink_camera_information_t, vendor_name) }, \
         { "model_name", NULL, MAVLINK_TYPE_UINT8_T, 32, 62, offsetof(mavlink_camera_information_t, model_name) }, \
         { "firmware_version", NULL, MAVLINK_TYPE_UINT32_T, 0, 4, offsetof(mavlink_camera_information_t, firmware_version) }, \
         { "focal_length", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_camera_information_t, focal_length) }, \
         { "sensor_size_h", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_camera_information_t, sensor_size_h) }, \
         { "sensor_size_v", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_camera_information_t, sensor_size_v) }, \
         { "resolution_h", NULL, MAVLINK_TYPE_UINT16_T, 0, 24, offsetof(mavlink_camera_information_t, resolution_h) }, \
         { "resolution_v", NULL, MAVLINK_TYPE_UINT16_T, 0, 26, offsetof(mavlink_camera_information_t, resolution_v) }, \
         { "lens_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 94, offsetof(mavlink_camera_information_t, lens_id) }, \
         { "flags", NULL, MAVLINK_TYPE_UINT32_T, 0, 20, offsetof(mavlink_camera_information_t, flags) }, \
         { "cam_definition_version", NULL, MAVLINK_TYPE_UINT16_T, 0, 28, offsetof(mavlink_camera_information_t, cam_definition_version) }, \
         { "cam_definition_uri", NULL, MAVLINK_TYPE_CHAR, 140, 95, offsetof(mavlink_camera_information_t, cam_definition_uri) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_CAMERA_INFORMATION { \
    "CAMERA_INFORMATION", \
    13, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_camera_information_t, time_boot_ms) }, \
         { "vendor_name", NULL, MAVLINK_TYPE_UINT8_T, 32, 30, offsetof(mavlink_camera_information_t, vendor_name) }, \
         { "model_name", NULL, MAVLINK_TYPE_UINT8_T, 32, 62, offsetof(mavlink_camera_information_t, model_name) }, \
         { "firmware_version", NULL, MAVLINK_TYPE_UINT32_T, 0, 4, offsetof(mavlink_camera_information_t, firmware_version) }, \
         { "focal_length", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_camera_information_t, focal_length) }, \
         { "sensor_size_h", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_camera_information_t, sensor_size_h) }, \
         { "sensor_size_v", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_camera_information_t, sensor_size_v) }, \
         { "resolution_h", NULL, MAVLINK_TYPE_UINT16_T, 0, 24, offsetof(mavlink_camera_information_t, resolution_h) }, \
         { "resolution_v", NULL, MAVLINK_TYPE_UINT16_T, 0, 26, offsetof(mavlink_camera_information_t, resolution_v) }, \
         { "lens_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 94, offsetof(mavlink_camera_information_t, lens_id) }, \
         { "flags", NULL, MAVLINK_TYPE_UINT32_T, 0, 20, offsetof(mavlink_camera_information_t, flags) }, \
         { "cam_definition_version", NULL, MAVLINK_TYPE_UINT16_T, 0, 28, offsetof(mavlink_camera_information_t, cam_definition_version) }, \
         { "cam_definition_uri", NULL, MAVLINK_TYPE_CHAR, 140, 95, offsetof(mavlink_camera_information_t, cam_definition_uri) }, \
         } \
}
#endif

/**
 * @brief Pack a camera_information message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param vendor_name  Name of the camera vendor
 * @param model_name  Name of the camera model
 * @param firmware_version  Version of the camera firmware, encoded as: (Dev & 0xff) << 24 | (Patch & 0xff) << 16 | (Minor & 0xff) << 8 | (Major & 0xff)
 * @param focal_length [mm] Focal length
 * @param sensor_size_h [mm] Image sensor size horizontal
 * @param sensor_size_v [mm] Image sensor size vertical
 * @param resolution_h [pix] Horizontal image resolution
 * @param resolution_v [pix] Vertical image resolution
 * @param lens_id  Reserved for a lens ID
 * @param flags  Bitmap of camera capability flags.
 * @param cam_definition_version  Camera definition version (iteration)
 * @param cam_definition_uri  Camera definition URI (if any, otherwise only basic functions will be available). HTTP- (http://) and MAVLink FTP- (mavlinkftp://) formatted URIs are allowed (and both must be supported by any GCS that implements the Camera Protocol). The definition file may be xz compressed, which will be indicated by the file extension .xml.xz (a GCS that implements the protocol must support decompressing the file).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_camera_information_pack(Uint8_t system_id, Uint8_t component_id, mavlink_message_t* msg,
                               uint32_t time_boot_ms, const Uint8_t *vendor_name, const Uint8_t *model_name, uint32_t firmware_version, float focal_length, float sensor_size_h, float sensor_size_v, uint16_t resolution_h, uint16_t resolution_v, Uint8_t lens_id, uint32_t flags, uint16_t cam_definition_version, const char *cam_definition_uri)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CAMERA_INFORMATION_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_uint32_t(buf, 4, firmware_version);
    _mav_put_float(buf, 8, focal_length);
    _mav_put_float(buf, 12, sensor_size_h);
    _mav_put_float(buf, 16, sensor_size_v);
    _mav_put_uint32_t(buf, 20, flags);
    _mav_put_uint16_t(buf, 24, resolution_h);
    _mav_put_uint16_t(buf, 26, resolution_v);
    _mav_put_uint16_t(buf, 28, cam_definition_version);
    _mav_put_uint8_t(buf, 94, lens_id);
    _mav_put_uint8_t_array(buf, 30, vendor_name, 32);
    _mav_put_uint8_t_array(buf, 62, model_name, 32);
    _mav_put_char_array(buf, 95, cam_definition_uri, 140);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CAMERA_INFORMATION_LEN);
#else
    mavlink_camera_information_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.firmware_version = firmware_version;
    packet.focal_length = focal_length;
    packet.sensor_size_h = sensor_size_h;
    packet.sensor_size_v = sensor_size_v;
    packet.flags = flags;
    packet.resolution_h = resolution_h;
    packet.resolution_v = resolution_v;
    packet.cam_definition_version = cam_definition_version;
    packet.lens_id = lens_id;
    mav_array_memcpy(packet.vendor_name, vendor_name, sizeof(Uint8_t)*32);
    mav_array_memcpy(packet.model_name, model_name, sizeof(Uint8_t)*32);
    mav_array_memcpy(packet.cam_definition_uri, cam_definition_uri, sizeof(char)*140);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CAMERA_INFORMATION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CAMERA_INFORMATION;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_CAMERA_INFORMATION_MIN_LEN, MAVLINK_MSG_ID_CAMERA_INFORMATION_LEN, MAVLINK_MSG_ID_CAMERA_INFORMATION_CRC);
}

/**
 * @brief Pack a camera_information message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param vendor_name  Name of the camera vendor
 * @param model_name  Name of the camera model
 * @param firmware_version  Version of the camera firmware, encoded as: (Dev & 0xff) << 24 | (Patch & 0xff) << 16 | (Minor & 0xff) << 8 | (Major & 0xff)
 * @param focal_length [mm] Focal length
 * @param sensor_size_h [mm] Image sensor size horizontal
 * @param sensor_size_v [mm] Image sensor size vertical
 * @param resolution_h [pix] Horizontal image resolution
 * @param resolution_v [pix] Vertical image resolution
 * @param lens_id  Reserved for a lens ID
 * @param flags  Bitmap of camera capability flags.
 * @param cam_definition_version  Camera definition version (iteration)
 * @param cam_definition_uri  Camera definition URI (if any, otherwise only basic functions will be available). HTTP- (http://) and MAVLink FTP- (mavlinkftp://) formatted URIs are allowed (and both must be supported by any GCS that implements the Camera Protocol). The definition file may be xz compressed, which will be indicated by the file extension .xml.xz (a GCS that implements the protocol must support decompressing the file).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_camera_information_pack_chan(Uint8_t system_id, Uint8_t component_id, Uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t time_boot_ms,const Uint8_t *vendor_name,const Uint8_t *model_name,uint32_t firmware_version,float focal_length,float sensor_size_h,float sensor_size_v,uint16_t resolution_h,uint16_t resolution_v,Uint8_t lens_id,uint32_t flags,uint16_t cam_definition_version,const char *cam_definition_uri)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CAMERA_INFORMATION_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_uint32_t(buf, 4, firmware_version);
    _mav_put_float(buf, 8, focal_length);
    _mav_put_float(buf, 12, sensor_size_h);
    _mav_put_float(buf, 16, sensor_size_v);
    _mav_put_uint32_t(buf, 20, flags);
    _mav_put_uint16_t(buf, 24, resolution_h);
    _mav_put_uint16_t(buf, 26, resolution_v);
    _mav_put_uint16_t(buf, 28, cam_definition_version);
    _mav_put_uint8_t(buf, 94, lens_id);
    _mav_put_uint8_t_array(buf, 30, vendor_name, 32);
    _mav_put_uint8_t_array(buf, 62, model_name, 32);
    _mav_put_char_array(buf, 95, cam_definition_uri, 140);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CAMERA_INFORMATION_LEN);
#else
    mavlink_camera_information_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.firmware_version = firmware_version;
    packet.focal_length = focal_length;
    packet.sensor_size_h = sensor_size_h;
    packet.sensor_size_v = sensor_size_v;
    packet.flags = flags;
    packet.resolution_h = resolution_h;
    packet.resolution_v = resolution_v;
    packet.cam_definition_version = cam_definition_version;
    packet.lens_id = lens_id;
    mav_array_memcpy(packet.vendor_name, vendor_name, sizeof(Uint8_t)*32);
    mav_array_memcpy(packet.model_name, model_name, sizeof(Uint8_t)*32);
    mav_array_memcpy(packet.cam_definition_uri, cam_definition_uri, sizeof(char)*140);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CAMERA_INFORMATION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CAMERA_INFORMATION;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_CAMERA_INFORMATION_MIN_LEN, MAVLINK_MSG_ID_CAMERA_INFORMATION_LEN, MAVLINK_MSG_ID_CAMERA_INFORMATION_CRC);
}

/**
 * @brief Encode a camera_information struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param camera_information C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_camera_information_encode(Uint8_t system_id, Uint8_t component_id, mavlink_message_t* msg, const mavlink_camera_information_t* camera_information)
{
    return mavlink_msg_camera_information_pack(system_id, component_id, msg, camera_information->time_boot_ms, camera_information->vendor_name, camera_information->model_name, camera_information->firmware_version, camera_information->focal_length, camera_information->sensor_size_h, camera_information->sensor_size_v, camera_information->resolution_h, camera_information->resolution_v, camera_information->lens_id, camera_information->flags, camera_information->cam_definition_version, camera_information->cam_definition_uri);
}

/**
 * @brief Encode a camera_information struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param camera_information C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_camera_information_encode_chan(Uint8_t system_id, Uint8_t component_id, Uint8_t chan, mavlink_message_t* msg, const mavlink_camera_information_t* camera_information)
{
    return mavlink_msg_camera_information_pack_chan(system_id, component_id, chan, msg, camera_information->time_boot_ms, camera_information->vendor_name, camera_information->model_name, camera_information->firmware_version, camera_information->focal_length, camera_information->sensor_size_h, camera_information->sensor_size_v, camera_information->resolution_h, camera_information->resolution_v, camera_information->lens_id, camera_information->flags, camera_information->cam_definition_version, camera_information->cam_definition_uri);
}

/**
 * @brief Send a camera_information message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param vendor_name  Name of the camera vendor
 * @param model_name  Name of the camera model
 * @param firmware_version  Version of the camera firmware, encoded as: (Dev & 0xff) << 24 | (Patch & 0xff) << 16 | (Minor & 0xff) << 8 | (Major & 0xff)
 * @param focal_length [mm] Focal length
 * @param sensor_size_h [mm] Image sensor size horizontal
 * @param sensor_size_v [mm] Image sensor size vertical
 * @param resolution_h [pix] Horizontal image resolution
 * @param resolution_v [pix] Vertical image resolution
 * @param lens_id  Reserved for a lens ID
 * @param flags  Bitmap of camera capability flags.
 * @param cam_definition_version  Camera definition version (iteration)
 * @param cam_definition_uri  Camera definition URI (if any, otherwise only basic functions will be available). HTTP- (http://) and MAVLink FTP- (mavlinkftp://) formatted URIs are allowed (and both must be supported by any GCS that implements the Camera Protocol). The definition file may be xz compressed, which will be indicated by the file extension .xml.xz (a GCS that implements the protocol must support decompressing the file).
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_camera_information_send(mavlink_channel_t chan, uint32_t time_boot_ms, const Uint8_t *vendor_name, const Uint8_t *model_name, uint32_t firmware_version, float focal_length, float sensor_size_h, float sensor_size_v, uint16_t resolution_h, uint16_t resolution_v, Uint8_t lens_id, uint32_t flags, uint16_t cam_definition_version, const char *cam_definition_uri)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CAMERA_INFORMATION_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_uint32_t(buf, 4, firmware_version);
    _mav_put_float(buf, 8, focal_length);
    _mav_put_float(buf, 12, sensor_size_h);
    _mav_put_float(buf, 16, sensor_size_v);
    _mav_put_uint32_t(buf, 20, flags);
    _mav_put_uint16_t(buf, 24, resolution_h);
    _mav_put_uint16_t(buf, 26, resolution_v);
    _mav_put_uint16_t(buf, 28, cam_definition_version);
    _mav_put_uint8_t(buf, 94, lens_id);
    _mav_put_uint8_t_array(buf, 30, vendor_name, 32);
    _mav_put_uint8_t_array(buf, 62, model_name, 32);
    _mav_put_char_array(buf, 95, cam_definition_uri, 140);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CAMERA_INFORMATION, buf, MAVLINK_MSG_ID_CAMERA_INFORMATION_MIN_LEN, MAVLINK_MSG_ID_CAMERA_INFORMATION_LEN, MAVLINK_MSG_ID_CAMERA_INFORMATION_CRC);
#else
    mavlink_camera_information_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.firmware_version = firmware_version;
    packet.focal_length = focal_length;
    packet.sensor_size_h = sensor_size_h;
    packet.sensor_size_v = sensor_size_v;
    packet.flags = flags;
    packet.resolution_h = resolution_h;
    packet.resolution_v = resolution_v;
    packet.cam_definition_version = cam_definition_version;
    packet.lens_id = lens_id;
    mav_array_memcpy(packet.vendor_name, vendor_name, sizeof(Uint8_t)*32);
    mav_array_memcpy(packet.model_name, model_name, sizeof(Uint8_t)*32);
    mav_array_memcpy(packet.cam_definition_uri, cam_definition_uri, sizeof(char)*140);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CAMERA_INFORMATION, (const char *)&packet, MAVLINK_MSG_ID_CAMERA_INFORMATION_MIN_LEN, MAVLINK_MSG_ID_CAMERA_INFORMATION_LEN, MAVLINK_MSG_ID_CAMERA_INFORMATION_CRC);
#endif
}

/**
 * @brief Send a camera_information message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_camera_information_send_struct(mavlink_channel_t chan, const mavlink_camera_information_t* camera_information)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_camera_information_send(chan, camera_information->time_boot_ms, camera_information->vendor_name, camera_information->model_name, camera_information->firmware_version, camera_information->focal_length, camera_information->sensor_size_h, camera_information->sensor_size_v, camera_information->resolution_h, camera_information->resolution_v, camera_information->lens_id, camera_information->flags, camera_information->cam_definition_version, camera_information->cam_definition_uri);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CAMERA_INFORMATION, (const char *)camera_information, MAVLINK_MSG_ID_CAMERA_INFORMATION_MIN_LEN, MAVLINK_MSG_ID_CAMERA_INFORMATION_LEN, MAVLINK_MSG_ID_CAMERA_INFORMATION_CRC);
#endif
}

#if MAVLINK_MSG_ID_CAMERA_INFORMATION_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_camera_information_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t time_boot_ms, const Uint8_t *vendor_name, const Uint8_t *model_name, uint32_t firmware_version, float focal_length, float sensor_size_h, float sensor_size_v, uint16_t resolution_h, uint16_t resolution_v, Uint8_t lens_id, uint32_t flags, uint16_t cam_definition_version, const char *cam_definition_uri)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_uint32_t(buf, 4, firmware_version);
    _mav_put_float(buf, 8, focal_length);
    _mav_put_float(buf, 12, sensor_size_h);
    _mav_put_float(buf, 16, sensor_size_v);
    _mav_put_uint32_t(buf, 20, flags);
    _mav_put_uint16_t(buf, 24, resolution_h);
    _mav_put_uint16_t(buf, 26, resolution_v);
    _mav_put_uint16_t(buf, 28, cam_definition_version);
    _mav_put_uint8_t(buf, 94, lens_id);
    _mav_put_uint8_t_array(buf, 30, vendor_name, 32);
    _mav_put_uint8_t_array(buf, 62, model_name, 32);
    _mav_put_char_array(buf, 95, cam_definition_uri, 140);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CAMERA_INFORMATION, buf, MAVLINK_MSG_ID_CAMERA_INFORMATION_MIN_LEN, MAVLINK_MSG_ID_CAMERA_INFORMATION_LEN, MAVLINK_MSG_ID_CAMERA_INFORMATION_CRC);
#else
    mavlink_camera_information_t *packet = (mavlink_camera_information_t *)msgbuf;
    packet->time_boot_ms = time_boot_ms;
    packet->firmware_version = firmware_version;
    packet->focal_length = focal_length;
    packet->sensor_size_h = sensor_size_h;
    packet->sensor_size_v = sensor_size_v;
    packet->flags = flags;
    packet->resolution_h = resolution_h;
    packet->resolution_v = resolution_v;
    packet->cam_definition_version = cam_definition_version;
    packet->lens_id = lens_id;
    mav_array_memcpy(packet->vendor_name, vendor_name, sizeof(Uint8_t)*32);
    mav_array_memcpy(packet->model_name, model_name, sizeof(Uint8_t)*32);
    mav_array_memcpy(packet->cam_definition_uri, cam_definition_uri, sizeof(char)*140);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CAMERA_INFORMATION, (const char *)packet, MAVLINK_MSG_ID_CAMERA_INFORMATION_MIN_LEN, MAVLINK_MSG_ID_CAMERA_INFORMATION_LEN, MAVLINK_MSG_ID_CAMERA_INFORMATION_CRC);
#endif
}
#endif

#endif

// MESSAGE CAMERA_INFORMATION UNPACKING


/**
 * @brief Get field time_boot_ms from camera_information message
 *
 * @return [ms] Timestamp (time since system boot).
 */
static inline uint32_t mavlink_msg_camera_information_get_time_boot_ms(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field vendor_name from camera_information message
 *
 * @return  Name of the camera vendor
 */
static inline uint16_t mavlink_msg_camera_information_get_vendor_name(const mavlink_message_t* msg, Uint8_t *vendor_name)
{
    return _MAV_RETURN_uint8_t_array(msg, vendor_name, 32,  30);
}

/**
 * @brief Get field model_name from camera_information message
 *
 * @return  Name of the camera model
 */
static inline uint16_t mavlink_msg_camera_information_get_model_name(const mavlink_message_t* msg, Uint8_t *model_name)
{
    return _MAV_RETURN_uint8_t_array(msg, model_name, 32,  62);
}

/**
 * @brief Get field firmware_version from camera_information message
 *
 * @return  Version of the camera firmware, encoded as: (Dev & 0xff) << 24 | (Patch & 0xff) << 16 | (Minor & 0xff) << 8 | (Major & 0xff)
 */
static inline uint32_t mavlink_msg_camera_information_get_firmware_version(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  4);
}

/**
 * @brief Get field focal_length from camera_information message
 *
 * @return [mm] Focal length
 */
static inline float mavlink_msg_camera_information_get_focal_length(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field sensor_size_h from camera_information message
 *
 * @return [mm] Image sensor size horizontal
 */
static inline float mavlink_msg_camera_information_get_sensor_size_h(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field sensor_size_v from camera_information message
 *
 * @return [mm] Image sensor size vertical
 */
static inline float mavlink_msg_camera_information_get_sensor_size_v(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field resolution_h from camera_information message
 *
 * @return [pix] Horizontal image resolution
 */
static inline uint16_t mavlink_msg_camera_information_get_resolution_h(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  24);
}

/**
 * @brief Get field resolution_v from camera_information message
 *
 * @return [pix] Vertical image resolution
 */
static inline uint16_t mavlink_msg_camera_information_get_resolution_v(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  26);
}

/**
 * @brief Get field lens_id from camera_information message
 *
 * @return  Reserved for a lens ID
 */
static inline Uint8_t mavlink_msg_camera_information_get_lens_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  94);
}

/**
 * @brief Get field flags from camera_information message
 *
 * @return  Bitmap of camera capability flags.
 */
static inline uint32_t mavlink_msg_camera_information_get_flags(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  20);
}

/**
 * @brief Get field cam_definition_version from camera_information message
 *
 * @return  Camera definition version (iteration)
 */
static inline uint16_t mavlink_msg_camera_information_get_cam_definition_version(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  28);
}

/**
 * @brief Get field cam_definition_uri from camera_information message
 *
 * @return  Camera definition URI (if any, otherwise only basic functions will be available). HTTP- (http://) and MAVLink FTP- (mavlinkftp://) formatted URIs are allowed (and both must be supported by any GCS that implements the Camera Protocol). The definition file may be xz compressed, which will be indicated by the file extension .xml.xz (a GCS that implements the protocol must support decompressing the file).
 */
static inline uint16_t mavlink_msg_camera_information_get_cam_definition_uri(const mavlink_message_t* msg, char *cam_definition_uri)
{
    return _MAV_RETURN_char_array(msg, cam_definition_uri, 140,  95);
}

/**
 * @brief Decode a camera_information message into a struct
 *
 * @param msg The message to decode
 * @param camera_information C-struct to decode the message contents into
 */
static inline void mavlink_msg_camera_information_decode(const mavlink_message_t* msg, mavlink_camera_information_t* camera_information)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    camera_information->time_boot_ms = mavlink_msg_camera_information_get_time_boot_ms(msg);
    camera_information->firmware_version = mavlink_msg_camera_information_get_firmware_version(msg);
    camera_information->focal_length = mavlink_msg_camera_information_get_focal_length(msg);
    camera_information->sensor_size_h = mavlink_msg_camera_information_get_sensor_size_h(msg);
    camera_information->sensor_size_v = mavlink_msg_camera_information_get_sensor_size_v(msg);
    camera_information->flags = mavlink_msg_camera_information_get_flags(msg);
    camera_information->resolution_h = mavlink_msg_camera_information_get_resolution_h(msg);
    camera_information->resolution_v = mavlink_msg_camera_information_get_resolution_v(msg);
    camera_information->cam_definition_version = mavlink_msg_camera_information_get_cam_definition_version(msg);
    mavlink_msg_camera_information_get_vendor_name(msg, camera_information->vendor_name);
    mavlink_msg_camera_information_get_model_name(msg, camera_information->model_name);
    camera_information->lens_id = mavlink_msg_camera_information_get_lens_id(msg);
    mavlink_msg_camera_information_get_cam_definition_uri(msg, camera_information->cam_definition_uri);
#else
        Uint8_t len = msg->len < MAVLINK_MSG_ID_CAMERA_INFORMATION_LEN? msg->len : MAVLINK_MSG_ID_CAMERA_INFORMATION_LEN;
        memset(camera_information, 0, MAVLINK_MSG_ID_CAMERA_INFORMATION_LEN);
    memcpy(camera_information, _MAV_PAYLOAD(msg), len);
#endif
}
