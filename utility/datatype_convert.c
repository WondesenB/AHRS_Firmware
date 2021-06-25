/*
 * datatype_convert.c
 *
 *  Created on: 4 ??? 2021
 *      Author: WB
 */

#include"datatype_convert.h"
//
bool writeFloat(float value) {
    int pos = 0;
    if (isnan(value))
    {
        return false;
//        asm(" ESTOP0");             //Emulation stop
//        for(;;){};                  //Loop forever
    }
    if (value < 0.0) {
        buf[pos]='-';
        pos +=1;
        value = -value;
    }

    if (isinf(value))
    {
        return false;
//        asm(" ESTOP0");             //Emulation stop
//        for(;;){};                  //Loop forever
    }

    uint32_t integralPart, decimalPart;
    int16_t exponent;
    splitFloat(value, &integralPart, &decimalPart, &exponent);

    writeInteger(integralPart,&pos);
    if (decimalPart)
    {
        writeDecimals(decimalPart, &pos);
    }

    if (exponent < 0) {
        //    puts("e-");
        buf[pos]='e';
        pos +=1;
        buf[pos]='-';
        pos +=1;
        writeInteger(-exponent,&pos);
    }

    if (exponent > 0) {
        //    puts('e');
        buf[pos]='e';
        pos +=1;
        writeInteger(exponent,&pos);
    }
    return true;
}
//
void splitFloat(float value, uint32_t *integralPart,  uint32_t *decimalPart, int16_t *exponent) {
    *exponent = normalizeFloat(&value);

    *integralPart = (uint32_t)value;
    float remainder = value - *integralPart;

    remainder *= 1e9;
    *decimalPart = (uint32_t)remainder;

    // rounding
    remainder -= *decimalPart;
    if (remainder >= 0.5) {
        *decimalPart++;
        if (*decimalPart >= 1000000000) {
            *decimalPart = 0;
            *integralPart++;
            if (*exponent != 0 && *integralPart >= 10) {
                *exponent++;
                *integralPart = 1;
            }
        }
    }
}
//
int normalizeFloat(float  *value) {
    const float positiveExpThreshold = 1e7;
    const float negativeExpThreshold = 1e-5;
    int exponent = 0;

    if (*value >= positiveExpThreshold) {
        if (*value >= 1e32) {
            *value /= 1e32;
            exponent += 32;
        }
        if (*value >= 1e16) {
            *value /= 1e16;
            exponent += 16;
        }
        if (*value >= 1e8) {
            *value /= 1e8;
            exponent += 8;
        }
        if (*value >= 1e4) {
            *value /= 1e4;
            exponent += 4;
        }
        if (*value >= 1e2) {
            *value /= 1e2;
            exponent += 2;
        }
        if (*value >= 1e1) {
            *value /= 1e1;
            exponent += 1;
        }
    }

    if (*value > 0 && *value <= negativeExpThreshold) {
        if (*value < 1e-31) {
            *value *= 1e32;
            exponent -= 32;
        }
        if (*value < 1e-15) {
            *value *= 1e16;
            exponent -= 16;
        }
        if (*value < 1e-7) {
            *value *= 1e8;
            exponent -= 8;
        }
        if (*value < 1e-3) {
            *value *= 1e4;
            exponent -= 4;
        }
        if (*value < 1e-1) {
            *value *= 1e2;
            exponent -= 2;
        }
        if (*value < 1e0) {
            *value *= 1e1;
            exponent -= 1;
        }
    }
    return exponent;
}
//
void writeInteger(uint32_t value, int *pos) {
    char buffer[16];
    int i = *pos ;
    char *ptr = buffer + sizeof(buffer) - 1;
    *ptr = 0;
    do {
        *--ptr = (char)(value % 10) + '0';
        value = (uint32_t)(value / 10);
        *pos +=1;
    } while (value);
    int j=0;
    while(i<=*pos)
    {
        buf[i]=*(ptr+j);
        i++;
        j++;
    }
    // *pos+=1;
}
//
void writeDecimals(uint32_t value, int *pos) {
    int width = 9;
    int i = *pos;
    // remove trailing zeros
    while (value % 10 == 0 && width > 0) {
        value /= 10;
        width--;
    }

    char buffer[16];
    char *ptr = buffer + sizeof(buffer) - 1;

    // write the string in reverse order
    *ptr = 0;
    while (width--) {
        *--ptr = value % 10 + '0';
        value /= 10;
        *pos +=1;
    }
    *--ptr = '.';
    *pos +=1;
    // and dump it in the right order
    int j=0 ;
    while(i<=*pos)
    {
        buf[i]=*(ptr+j);
        i++;
        j++;
    }
    *pos+=1;
    //puts(ptr);
}

// wrap angle [-180,180]
float headingwrap(float angle)
{
    if (angle < -180)
    {
        return (angle + 360);
    }
    else if (angle > 180)
    {
        return (angle - 360);
    }
    else
    {
        return angle;
    }

}
