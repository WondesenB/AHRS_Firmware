/*
 * timercinfig.h
 *
 *  Created on: 15 ጁን 2021
 *      Author: WB
 */

#ifndef DRIVERS_TIMERCONFIG_H_
#define DRIVERS_TIMERCONFIG_H_

#include "F28x_Project.h"

void timer_config(void);
__interrupt void cpu_timer0_isr(void);
__interrupt void cpu_timer1_isr(void);



#endif /* DRIVERS_TIMERCONFIG_H_ */
