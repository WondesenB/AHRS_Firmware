#pragma once

#if defined(MAVLINK_USE_CXX_NAMESPACE)
namespace mavlink {
#elif defined(__cplusplus)
extern "C" {
#endif

// Visual Studio versions before 2010 don't have stdint.h, so we just error out.
#if (defined _MSC_VER) && (_MSC_VER < 1600)
#error "The C-MAVLink implementation requires Visual Studio 2010 or greater"
#endif

#include <stdint.h>
/**
 *
 *  CALCULATE THE CHECKSUM
 *
 */

#define X25_INIT_CRC 0xffff
#define X25_VALIDATE_CRC 0xf0b8

#ifndef HAVE_CRC_ACCUMULATE
/**
 * @brief Accumulate the CRC16_MCRF4XX checksum by adding one char at a time.
 *
 * The checksum function adds the hash of one char at a time to the
 * 16 bit checksum (uint16_t).
 *
 * @param data new char to hash
 * @param crcAccum the already accumulated checksum
 **/
static inline void crc_accumulate(Uint8_t data, uint16_t *crcAccum)
{
    /*Accumulate one byte of data into the CRC*/
    Uint8_t tmp;

    tmp = data ^ (Uint8_t)(*crcAccum &0xff);
    tmp ^= (tmp<<4);
    tmp &=0xff;
    *crcAccum = (*crcAccum>>8) ^ (tmp<<8) ^ (tmp <<3) ^ (tmp>>4);
}
#endif


/**
 * @brief Initiliaze the buffer for the MCRF4XX CRC16
 *
 * @param crcAccum the 16 bit MCRF4XX CRC16
 */
static inline void crc_init(uint16_t* crcAccum)
{
        *crcAccum = X25_INIT_CRC;
}


/**
 * @brief Calculates the CRC16_MCRF4XX checksum on a byte buffer
 *
 * @param  pBuffer buffer containing the byte array to hash
 * @param  length  length of the byte array
 * @return the checksum over the buffer bytes
 **/
static inline uint16_t crc_calculate(const Uint8_t* pBuffer, uint16_t length)
{
        uint16_t crcTmp;
        crc_init(&crcTmp);
    while (length--) {
                crc_accumulate(*pBuffer++, &crcTmp);
        }
        return crcTmp;
}

/**
 * @brief Accumulate the MCRF4XX CRC16 by adding an array of bytes
 *
 * The checksum function adds the hash of one char at a time to the
 * 16 bit checksum (uint16_t).
 *
 * @param data new bytes to hash
 * @param crcAccum the already accumulated checksum
 **/
static inline void crc_accumulate_buffer(uint16_t *crcAccum, const char *pBuffer, uint16_t length)
{
    const Uint8_t *p = (const Uint8_t *)pBuffer;
    while (length--) {
                crc_accumulate(*p++, crcAccum);
        }
}

static inline void crc_accumulate_buffer_new(uint16_t *crcAccum, const char *pBuffer, uint16_t length)
{
    const Uint8_t  *p = ( const Uint8_t *)pBuffer;
    Uint8_t data;
    while (length--) {
        data = *p;
        crc_accumulate(data&0xFF, crcAccum);
        if(length!=0)
        {
            crc_accumulate((data>>8)&0xFF, crcAccum);
            p++;
            length--;
        }

        }
}
#if defined(MAVLINK_USE_CXX_NAMESPACE) || defined(__cplusplus)
}
#endif
