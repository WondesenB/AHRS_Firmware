/*
 * cla_shared.h
 *
 *  Created on: Sep 14, 2021
 *      Author: wb
 */

#ifndef CLA_SHARED_H_
#define CLA_SHARED_H_

//
// Included Files
//
#include "F2837xS_Cla_defines.h"
#include <stdint.h>

//
// Globals
//

//
//Task 1 (C) Variables
//
extern float x;           //Holds the input argument to the task
extern float y;        //The arsine of the input argument
//
//Task 2 (C) Variables
//

//
//Task 3 (C) Variables
//

//
//Task 4 (C) Variables
//

//
//Task 5 (C) Variables
//

//
//Task 6 (C) Variables
//

//
//Task 7 (C) Variables
//

//
//Task 8 (C) Variables
//

//
// Function Prototypes
//
// The following are symbols defined in the CLA assembly code
// Including them in the shared header file makes them
// .global and the main CPU can make use of them.
//
__interrupt void Cla1Task1();
__interrupt void Cla1Task2();
__interrupt void Cla1Task3();
__interrupt void Cla1Task4();
__interrupt void Cla1Task5();
__interrupt void Cla1Task6();
__interrupt void Cla1Task7();
__interrupt void Cla1Task8();





#endif /* CLA_SHARED_H_ */
