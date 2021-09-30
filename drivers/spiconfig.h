/*
 * F28377S_Interface_Config.h
 *
 *  Created on: 4 ??? 2021
 *      Author: WB
 */

#ifndef F28377S_INTERFACE_CONFIG_H_
#define F28377S_INTERFACE_CONFIG_H_
#include "ICM20948.h"
#include "F28x_Project.h"

//SPI Interface
#define READ_FLAG 0x80
#define CS_PIN  19  // 15 (training kit)  or 19 (sensor module)
#if (CS_PIN==15)
#define CS_HIGH GpioDataRegs.GPASET.bit.GPIO15 = 1;
#define CS_LOW GpioDataRegs.GPACLEAR.bit.GPIO15 = 1;
#else
#define CS_HIGH GpioDataRegs.GPASET.bit.GPIO19 = 1;
#define CS_LOW GpioDataRegs.GPACLEAR.bit.GPIO19 = 1;
#endif

void spi_pin_config (void);
void DESELECT (void);
void SELECT (void);
void xmit_spi (BYTE dat);
BYTE rcvr_spi (void);
void rcvr_spi_m (BYTE *dst);

#endif /* F28377S_INTERFACE_CONFIG_H_ */
