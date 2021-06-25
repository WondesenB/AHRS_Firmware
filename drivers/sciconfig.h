/*
 * sciconfig.h
 *
 *  Created on: 15 ጁን 2021
 *      Author: WB
 */

#ifndef DRIVERS_SCICONFIG_H_
#define DRIVERS_SCICONFIG_H_
#include "F28x_Project.h"
#define  SerialPort 2   // 1 (SCIA) 2(SCIB)
void sci_pin_config(void);
void scia_xmit(int16_t a);
#endif /* DRIVERS_SCICONFIG_H_ */
