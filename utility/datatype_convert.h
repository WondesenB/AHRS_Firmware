/*
 * datatype_convert.h
 *
 *  Created on: 4 ??? 2021
 *      Author: WB
 */

#ifndef DATATYPE_CONVERT_H_
#define DATATYPE_CONVERT_H_

#include "F28x_Project.h"
#include <math.h>

extern char buf[20] ;
//
bool writeFloat(float value);
void splitFloat(float value, uint32_t *integralPart, uint32_t *decimalPart, int16_t *exponent);
int normalizeFloat(float *value);
void writeInteger(uint32_t value, int *pos);
void writeDecimals(uint32_t value, int *pos);

float headingwrap(float angle);

#endif /* DATATYPE_CONVERT_H_ */
