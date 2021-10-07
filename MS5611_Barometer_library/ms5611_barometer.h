/*
 * ms5611_barometer.h
 *
 *  Created on: Sep 30, 2021
 *      Author: wb
 */

#ifndef MS5611_BAROMETER_LIBRARY_MS5611_BAROMETER_H_
#define MS5611_BAROMETER_LIBRARY_MS5611_BAROMETER_H_

#include "driverlib.h"
#include "device.h"
#include <math.h>

#define MS5611_ADDRESS                (0x76)

#define MS5611_CMD_ADC_READ           (0x00)
#define MS5611_CMD_RESET              (0x1E)
#define MS5611_CMD_CONV_D1            (0x48)
#define MS5611_CMD_CONV_D2            (0x58)
#define MS5611_CMD_READ_PROM          (0xA2)

 typedef enum
{
    PROM_read_C1 = 0,
    PROM_read_C2,
    PROM_read_C3,
    PROM_read_C4,
    PROM_read_C5,
    PROM_read_C6,
    D1_conversion,
    D2_conversion,

}steps;
void initI2C(void);

__interrupt void i2cBISR(void);
void i2c_send_byte(uint16_t slave_addr, uint8_t data, bool withstopbit);
void i2c_start_reading(uint16_t slave_addr, uint8_t numbytes);
void initialize_Barosensor(void);
int32_t calculate_pressure(bool compensation);
double getAltitude(double pressure,double refpressure  );
void estimateAltitude(void);
// extern enum Stps steps;
 static int step = PROM_read_C1;
 static int check = 1;
 static uint16_t C1, C2, C3, C4, C5, C6;
 static uint32_t D1, D2;
 extern int64_t   refP,P;
 extern double Alt, relativeAlt;
 static double seaLevelPressure = 101325;
 static int32_t TEMP2;
 static int64_t OFF2, SENS2;

#endif /* MS5611_BAROMETER_LIBRARY_MS5611_BAROMETER_H_ */
