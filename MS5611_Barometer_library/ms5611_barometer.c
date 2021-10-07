/*
 * ms5611_barometer.c
 *
 *  Created on: Sep 30, 2021
 *      Author: wb
 */

#include"ms5611_barometer.h"


//
// initI2C - Function to configure I2C A in FIFO mode.
//
void initI2C()
{
    //
    // Initialize GPIOs 32 and 33 for use as SDA A and SCL A respectively
    //
    GPIO_setPinConfig(GPIO_66_SDAB);
    GPIO_setPadConfig(66, GPIO_PIN_TYPE_PULLUP);
    GPIO_setQualificationMode(66, GPIO_QUAL_ASYNC);

    GPIO_setPinConfig(GPIO_69_SCLB);
    GPIO_setPadConfig(69, GPIO_PIN_TYPE_PULLUP);
    GPIO_setQualificationMode(69, GPIO_QUAL_ASYNC);

    Interrupt_register(INT_I2CB_FIFO, &i2cBISR);
    //
    // Must put I2C into reset before configuring it
    //
    I2C_disableModule(I2CB_BASE);

    //
    // I2C configuration. Use a 400kHz I2CCLK with a 33% duty cycle.
    //
    I2C_initMaster(I2CB_BASE, DEVICE_SYSCLK_FREQ, 400000, I2C_DUTYCYCLE_33);
    I2C_setBitCount(I2CB_BASE, I2C_BITCOUNT_8);
    I2C_setSlaveAddress(I2CB_BASE, MS5611_ADDRESS);
    I2C_setEmulationMode(I2CB_BASE, I2C_EMULATION_FREE_RUN);
    I2C_setAddressMode(I2CB_BASE, I2C_ADDR_MODE_7BITS);
    //
    // Enable stop condition and register-access-ready interrupts
    //
    /*  I2C_enableInterrupt(I2CB_BASE, I2C_INT_STOP_CONDITION |
     I2C_INT_REG_ACCESS_RDY);*/

    //
    // FIFO configuration
    //
    I2C_enableFIFO(I2CB_BASE);
    I2C_clearInterruptStatus(I2CB_BASE, I2C_INT_RXFF | I2C_INT_TXFF);
    I2C_setFIFOInterruptLevel(I2CB_BASE, I2C_FIFO_TX2, I2C_FIFO_RX2);
    I2C_enableInterrupt(I2CB_BASE, I2C_INT_RXFF | I2C_INT_TXFF);
    //
    // Configuration complete. Enable the module.
    //
    I2C_enableModule(I2CB_BASE);
    //
    // Enable interrupts required for this example
    //
    Interrupt_enable(INT_I2CB_FIFO);
}
void i2c_send_byte(uint16_t slave_addr, uint8_t data, bool withstopbit)
{
    // Setup slave address
    //
    I2C_setSlaveAddress(I2CB_BASE, slave_addr);
    //
    //
    // Send start as master transmitter
    //
    I2C_setConfig(I2CB_BASE, I2C_MASTER_SEND_MODE);

    // Setup number of bytes to send
    //
    I2C_setDataCount(I2CB_BASE, 1);

    //
    // Setup data to send
    //
    // sende start condition
    I2C_sendStartCondition(I2CB_BASE);
    I2C_putData(I2CB_BASE, data);
    if (withstopbit)
    {
        I2C_sendStopCondition(I2CB_BASE);
    }
}

void i2c_start_reading(uint16_t slave_addr, uint8_t numbytes)
{
    I2C_setSlaveAddress(I2CB_BASE, slave_addr);
    I2C_setDataCount(I2CB_BASE, numbytes);
    I2C_setConfig(I2CB_BASE, I2C_MASTER_RECEIVE_MODE);
    I2C_sendStartCondition(I2CB_BASE);
}
//
void initialize_Barosensor(void)
{
    // ========== Reset =====================
        i2c_send_byte(MS5611_ADDRESS, MS5611_CMD_RESET, true);

        // ===============Read PROM=======================
        //--------- read C1 -------
        DEVICE_DELAY_US(2800);                         // 2.8 milliseconds delay
        i2c_send_byte(MS5611_ADDRESS, MS5611_CMD_READ_PROM, false);
        DEVICE_DELAY_US(2300);                         // 2.3 milliseconds delay
        // ---------- start reading  ------------
        i2c_start_reading(MS5611_ADDRESS, 2);
    // Wait until FIFO Interrupt occur
        while ((step == PROM_read_C1) == true)
        {

        }

        //--------- read C2 -------
        DEVICE_DELAY_US(2300);                         // 2.3 milliseconds delay
        i2c_send_byte(MS5611_ADDRESS, 0xA4, false);
        DEVICE_DELAY_US(2300);                         // 2.3 milliseconds delay
        // ---------- start reading  ------------
        i2c_start_reading(MS5611_ADDRESS, 2);
        while ((step == PROM_read_C2) == true)
        {

        }

        //--------- read C3 -------
        DEVICE_DELAY_US(2300);                         // 2.3 milliseconds delay
        i2c_send_byte(MS5611_ADDRESS, 0xA6, false);
        DEVICE_DELAY_US(2300);                         // 2.3 milliseconds delay
        // ---------- start reading  ------------
        i2c_start_reading(MS5611_ADDRESS, 2);
        while ((step == PROM_read_C3) == true)
        {

        }

        //--------- read C4 -------
        DEVICE_DELAY_US(2300);                         // 2.3 milliseconds delay
        i2c_send_byte(MS5611_ADDRESS, 0xA8, false);
        DEVICE_DELAY_US(2300);                         // 2.3 milliseconds delay
        // ---------- start reading  ------------
        i2c_start_reading(MS5611_ADDRESS, 2);
        while ((step == PROM_read_C4) == true)
        {

        }

        //--------- read C5 -------
        DEVICE_DELAY_US(2300);                         // 2.3 milliseconds delay
        i2c_send_byte(MS5611_ADDRESS, 0xAA, false);
        DEVICE_DELAY_US(2300);                         // 2.3 milliseconds delay
        // ---------- start reading  ------------
        i2c_start_reading(MS5611_ADDRESS, 2);
        while ((step == PROM_read_C5) == true)
        {

        }

        //--------- read C6 -------
        DEVICE_DELAY_US(2300);                         // 2.3 milliseconds delay
        i2c_send_byte(MS5611_ADDRESS, 0xAC, false);
        DEVICE_DELAY_US(2300);                         // 2.3 milliseconds delay
        // ---------- start reading  ------------
        i2c_start_reading(MS5611_ADDRESS, 2);
        while ((step == PROM_read_C6) == true)
        {

        }

         // sednd D1 command to start conversion
        i2c_send_byte(MS5611_ADDRESS, MS5611_CMD_CONV_D1, true);
        DEVICE_DELAY_US(10300);                         // 10.3 milliseconds delay
        //read digital pressure value
        i2c_send_byte(MS5611_ADDRESS, MS5611_CMD_ADC_READ, false);
        DEVICE_DELAY_US(2300);                         // 2.3 milliseconds delay
        // ---------- start reading  ------------
        i2c_start_reading(MS5611_ADDRESS, 3);

        while ((step == D1_conversion) == true)
        {

        }

        // sednd D1 command to start conversion
        i2c_send_byte(MS5611_ADDRESS, MS5611_CMD_CONV_D2, true);
        DEVICE_DELAY_US(10300);
        //read digital temprature value
        i2c_send_byte(MS5611_ADDRESS, MS5611_CMD_ADC_READ, false);
        DEVICE_DELAY_US(2300);                         // 2.3 milliseconds delay
        // ---------- start reading  ------------
        i2c_start_reading(MS5611_ADDRESS, 3);

        while ((step == D2_conversion) == true)
        {

        }
        refP = calculate_pressure(true);
        Alt = getAltitude(refP,seaLevelPressure);
}
//
void estimateAltitude(void)
{
    step = D1_conversion;
     // sednd D1 command to start conversion
     i2c_send_byte(MS5611_ADDRESS, MS5611_CMD_CONV_D1, true);
     DEVICE_DELAY_US(10300);                         // 10.3 milliseconds delay
     //read digital pressure value
     i2c_send_byte(MS5611_ADDRESS, MS5611_CMD_ADC_READ, false);
     DEVICE_DELAY_US(2300);                         // 2.3 milliseconds delay
     // ---------- start reading  ------------
     i2c_start_reading(MS5611_ADDRESS, 3);

     while ((step == D1_conversion) == true)
     {

     }

     // send D1 command to start conversion
     i2c_send_byte(MS5611_ADDRESS, MS5611_CMD_CONV_D2, true);
     DEVICE_DELAY_US(10300);
     //read digital temprature value
     i2c_send_byte(MS5611_ADDRESS, MS5611_CMD_ADC_READ, false);
     DEVICE_DELAY_US(2300);                         // 2.3 milliseconds delay
     // ---------- start reading  ------------
     i2c_start_reading(MS5611_ADDRESS, 3);

     while ((step == D2_conversion) == true)
     {

     }

     P = calculate_pressure(true);
     relativeAlt = getAltitude(P,refP);
}
//
// i2cAISR - I2C A ISR (non-FIFO)
//
__interrupt void i2cBISR(void)
{

//    uint16_t i;
    if ((I2C_getInterruptStatus(I2CB_BASE) & I2C_INT_RXFF) != 0)
    {
        // In this case it doesn't work without the delay
        DEVICE_DELAY_US(120);                     // 12 microseconds  delay
        check = 1;
        if ((step == PROM_read_C1) && check == 1)
        {
            C1 = I2C_getData(I2CB_BASE);
            C1 = (C1 << 8) | (I2C_getData(I2CB_BASE) & 0x0FF);
            step = PROM_read_C2;
            check = 0;
        }

        if ((step == PROM_read_C2) && check == 1)
        {
            C2 = I2C_getData(I2CB_BASE);
            C2 = (C2 << 8) | (I2C_getData(I2CB_BASE) & 0x0FF);
            step = PROM_read_C3;
            check = 0;
        }

        if ((step == PROM_read_C3) && check == 1)
        {
            C3 = I2C_getData(I2CB_BASE);
            C3 = (C3  << 8) | (I2C_getData(I2CB_BASE) & 0x0FF);
            step = PROM_read_C4;
            check = 0;
        }

        if ((step == PROM_read_C4) && check == 1)
        {
            C4 = I2C_getData(I2CB_BASE);
            C4 = (C4  << 8) | (I2C_getData(I2CB_BASE) & 0x0FF);
            step = PROM_read_C5;
            check = 0;
        }

        if ((step == PROM_read_C5) && check == 1)
        {
            C5 = I2C_getData(I2CB_BASE);
            C5 = (C5  << 8) | (I2C_getData(I2CB_BASE) & 0x0FF);
            step = PROM_read_C6;
            check = 0;
        }

        if ((step == PROM_read_C6) && check == 1)
        {
            C6 = I2C_getData(I2CB_BASE);
            C6 = (C6  << 8) | (I2C_getData(I2CB_BASE) & 0x0FF);
            step = D1_conversion;
            check = 0;
        }

        if ((step == D1_conversion) && check == 1)
        {
            D1 = I2C_getData(I2CB_BASE);
            D1 = ((D1  << 8)& 0x0FF00) | (I2C_getData(I2CB_BASE) & 0x0FF);
            D1 = ((D1  << 8)& 0x0FFFF00)| (I2C_getData(I2CB_BASE) & 0x0FF);
            step = D2_conversion;
            check = 0;
        }

        if ((step == D2_conversion) && check == 1)
        {
            D2 = I2C_getData(I2CB_BASE);
            D2 = ((D2  << 8)& 0x0FF00) | (I2C_getData(I2CB_BASE) & 0x0FF);
            D2 = ((D2  << 8)& 0x0FFFF00)| (I2C_getData(I2CB_BASE) & 0x0FF);
            step = 100;
            check = 0;
        }
        //
        // Clear interrupt flag
        //
        I2C_clearInterruptStatus(I2CB_BASE, I2C_INT_RXFF);

    }

    //
    // Issue ACK to enable future group 8 interrupts
    //
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP8);
}


int32_t calculate_pressure(bool compensation)
{
       int32_t dT = D2 - (uint32_t)C5 * 256;

       int64_t OFF = (int64_t)C2 * 65536 + (int64_t)C4 * dT / 128;
       int64_t SENS = (int64_t)C1 * 32768 + (int64_t)C3 * dT / 256;

       if (compensation)
       {
       int32_t TEMP = 2000 + ((int64_t) dT * C6) / 8388608;

       OFF2 = 0;
       SENS2 = 0;

       if (TEMP < 2000)
       {
           OFF2 = 5 * ((TEMP - 2000) * (TEMP - 2000)) / 2;
           SENS2 = 5 * ((TEMP - 2000) * (TEMP - 2000)) / 4;
       }

       if (TEMP < -1500)
       {
           OFF2 = OFF2 + 7 * ((TEMP + 1500) * (TEMP + 1500));
           SENS2 = SENS2 + 11 * ((TEMP + 1500) * (TEMP + 1500)) / 2;
       }

       OFF = OFF - OFF2;
       SENS = SENS - SENS2;
       }

       uint32_t P = (D1 * SENS / 2097152 - OFF) / 32768;
       return P;
}

// Calculate altitude from Pressure & Sea level pressure
double getAltitude(double pressure ,double refpressure )
{
    return (44330.0f * (1.0f - pow((double)pressure / (double)refpressure, 0.1902949f)));
}


