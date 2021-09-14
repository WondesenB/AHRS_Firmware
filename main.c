
/**
 * main.c
 */
#include <string.h>
#include <stdio.h>
#include "spiconfig.h"
#include "sciconfig.h"
#include "timerconfig.h"
#include "datatype_convert.h"
#include <common/mavlink.h>
#define SENSORS_GRAVITY_EARTH  9.80665  /**< Earth's gravity in m/s^2 */
#define  testled  0
struct parameters para;
struct sensors_raw imu_raw;
struct sensors imu;
struct attitude att;
struct timeinfo tm;
struct sensorConfigdata sensorConfig;
struct EKF_Q ekf_t;
//
mavlink_message_t message;
char buff[300];

//float quat[8]= {1.0f, 0.0f, 0.0f, 0.0f,1.0f, 0.0f, 0.0f, 0.0f};
char buf[20] = { '.', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0',
                 '0', '0', '0', '0', '0', '0', '0', '0' };
// Magnetometer calibration coefficient
float const A[9] = { 0.9725, -0.0033, 0.0087, -0.0033, 1.0049, 0.0449, 0.0087,
                     0.0449, 1.0254 };
float const B[3] = { 318.6698, -104.7335, 543.1834 };
//
//#pragma DATA_SECTION(imu, "CpuToCla1MsgRAM");
//#pragma DATA_SECTION(quat, "Cla1ToCpuMsgRAM"); //Cla1ToCpuMsgRAM
//
void init(void);
void reset(void);
void scisend_data(float ax, float ay, float az, float gx, float gy, float gz,
                  float mx, float my, float mz);
void scisend_Magdata(float mx, float my, float mz);
void scisend_Euler(void);
//
int MagCalibrate = 0;
void main(void)
{
    // Local Variables to hold latest sensor data values
    float Accelx, Accely, Accelz, Gyrox, Gyroy, Gyroz, Magx, Magy, Magz, mmx,
            mmy, mmz;
    float q0, q1, q2, q3, q4, q5, q6, q7, theta, phi, psi;  //,  Bx, By, Bz, ;
    int i;
    //  Step 1. Initialize System Control:
    //  PLL, WatchDog, enable Peripheral Clocks
    //  This example function is found in the F2837xD_SysCtrl.c file.
    InitSysCtrl();
    //   Step 2. Initalize GPIO:
    //  This example function is found in the F2837xd_Gpio.c file and
    //  illustrates how to set the GPIO to it's default state.
    //  InitGpio();  // Skipped for this example
    //  Setup only the GP I/O only for SPI-A functionality
    //  This function is found in F2837xD_Spi.c
//    InitSpiaGpio();
    //  Step 3. Clear all interrupts and initialize PIE vector table:
    //  Disable CPU interrupts
    DINT;
    //  Initialize PIE control registers to their default state.
    //  The default state is all PIE interrupts disabled and flags
    //  are cleared.
    //  This function is found in the F2837xD_PieCtrl.c file.
    InitPieCtrl();
    //  Disable CPU interrupts and clear all CPU interrupt flags:
    IER = 0x0000;
    IFR = 0x0000;
    //  Initialize the PIE vector table with pointers to the shell Interrupt
    //  Service Routines (ISR).
    //  This will populate the entire table, even if the interrupt
    //  is not used in this example.  This is useful for debug purposes.
    //  The shell ISR routines are found in F2837xD_DefaultIsr.c.
    //  This function is found in F2837xD_PieVect.c.
    InitPieVectTable();
    // configure timer 0 and 1 for timing
    timer_config();
    // Configure SPI pin
    spi_pin_config();
    // sci configuration
    sci_pin_config();
    // parameter initialization
    param_initialize();
    //led test
    if (testled == 1)
    {
        for (;;)
        {
            GpioDataRegs.GPASET.bit.GPIO2 = 1;
            GpioDataRegs.GPBSET.bit.GPIO34 = 1;
            DELAY_US(500000);
            GpioDataRegs.GPACLEAR.bit.GPIO2 = 1;
            GpioDataRegs.GPBCLEAR.bit.GPIO34 = 1;
            DELAY_US(500000);
        }
    }
    //calibrate sensors
    //calibrateICM20948(para.gyroBias, para.accelBias);
    // Sensor init
    init();
    //check sensor configuration
    readSensorConfig();
    // Gyro calibration
    Gyrox = 0.0;
    Gyroy = 0.0;
    Gyroz = 0.0;
    for (i = 0; i < 1000; i++)
    {
        readGyroData(imu_raw.gyroCount);  // Read the x/y/z adc values
        Gyrox += (float) imu_raw.gyroCount[0];
        Gyroy += (float) imu_raw.gyroCount[1];
        Gyroz += (float) imu_raw.gyroCount[2];
    }
    para.gyroBias[0] = Gyrox / 1000;
    para.gyroBias[1] = Gyroy / 1000;
    para.gyroBias[2] = Gyroz / 1000;
    while (1)
    {
        // Switch to user bank 0
        writeByte(ICM20948_ADDRESS, REG_BANK_SEL, 0x00);
        // If intPin goes high, all data registers have new data
        // On interrupt, check if data ready interrupt
        if (readByte(ICM20948_ADDRESS, INT_STATUS_1) & 0x01)
        {
            readAccelData(imu_raw.accelCount);  // Read the x/y/z adc values
            // Now we'll calculate the accleration value into actual g's
            // This depends on scale being set
            Accelx = ((float) imu_raw.accelCount[0] * para.aRes
                    - para.accelBias[0]) * SENSORS_GRAVITY_EARTH;
            Accely = ((float) imu_raw.accelCount[1] * para.aRes
                    - para.accelBias[1]) * SENSORS_GRAVITY_EARTH;
            Accelz = ((float) imu_raw.accelCount[2] * para.aRes
                    - para.accelBias[2]) * SENSORS_GRAVITY_EARTH;
            //>>>>> the accelerometer reading is found to be inverted. Therefore, the value need to be negated <<<<
            // swap ax and ay, invert az to align to NED axes, apply low pass filter
            imu.ax = 0.5 * (-Accely + imu.ax);
            imu.ay = 0.5 * (-Accelx + imu.ay);
            imu.az = 0.5 * (Accelz + imu.az);
            //
            readGyroData(imu_raw.gyroCount);  // Read the x/y/z adc values
            // Calculate the gyro value into actual degrees per second
            // This depends on scale being set

            Gyrox = ((float) imu_raw.gyroCount[0] - para.gyroBias[0])
                    * para.gRes * DEG_TO_RAD;
            Gyroy = ((float) imu_raw.gyroCount[1] - para.gyroBias[1])
                    * para.gRes * DEG_TO_RAD;
            Gyroz = ((float) imu_raw.gyroCount[2] - para.gyroBias[2])
                    * para.gRes * DEG_TO_RAD;
            // swap gx and gy, invert gz to align to NED axes
            imu.gx = 0.5 * (Gyroy + imu.gx);
            imu.gy = 0.5 * (Gyrox + imu.gy);
            imu.gz = 0.5 * (-Gyroz + imu.gz);
            readMagData(imu_raw.magCount);  // Read the x/y/z adc values

            // Calculate the magnetometer values in milliGauss
            // Include factory calibration per data sheet and user environmental
            // corrections
            // Get actual magnetometer value, this depends on scale being set
            Magx = (float) imu_raw.magCount[0] * para.mRes - para.magBias[0];
            Magy = (float) imu_raw.magCount[1] * para.mRes - para.magBias[1];
            Magz = (float) imu_raw.magCount[2] * para.mRes - para.magBias[2];
            if (MagCalibrate)
            {
                // swap mx and -my to align to NED axes , apply lowpass filter and calibration
                mmx = (0.5 * (-Magy + imu.mx)) - B[0];
                mmy = (0.5 * (Magx + imu.my)) - B[1];
                mmz = (0.5 * (Magz + imu.mz)) - B[2];
                imu.mx = mmx * A[0] + mmy * A[3] + mmz * A[6];
                imu.my = mmx * A[1] + mmy * A[4] + mmz * A[7];
                imu.mz = mmx * A[2] + mmy * A[5] + mmz * A[8];
            }
            else
            {
                // swap mx and -my to align to NED axes , apply lowpass filter
                imu.mx = (0.5 * (-Magy + imu.mx));
                imu.my = (0.5 * (Magx + imu.my));
                imu.mz = (0.5 * (Magz + imu.mz));
            }
        } // if (readByte(ICM20948_ADDRESS, INT_STATUS) & 0x01)
          // Must be called before updating quaternions!
        updateTime();
        // AHRS
        EKFQ(imu.ax, imu.ay, imu.az, imu.gx, imu.gy, imu.gz, imu.mx, imu.my,
             imu.mz, tm.deltat);
        q0 = ekf_t.x[0];
        q1 = ekf_t.x[1];
        q2 = ekf_t.x[2];
        q3 = ekf_t.x[3];
        q4 = ekf_t.x[4];
        q5 = ekf_t.x[5];
        q6 = ekf_t.x[6];
        q7 = ekf_t.x[7];
        // http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
        float qkk = q1 * q3 - q0 * q2;
        if (qkk > 0.499) // pitch approaching 90 and above
        {
            theta = -asinf(1.0);
        }
        else if (qkk < -0.499)
        {
            theta = -asinf(-1.0);
        }
        else
        {
            theta = -asinf(2.0f * qkk);
        }
        phi = atan2f(2.0f * (q0 * q1 + q2 * q3),
                     q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3);
        psi = atan2f(2.0f * (q5 * q6 + q4 * q7),
                     q4 * q4 + q5 * q5 - q6 * q6 - q7 * q7);
        //
        att.pitch = theta;// * RAD_TO_DEG;
        att.roll = phi ;//* RAD_TO_DEG;
//        psi *= RAD_TO_DEG;
        // Declination of Busan (35�13'56.6"N 129�05'14.8"E) is
        //    8.16�  W  � 0.32�   on 2020-10-05
        // - http://www.ngdc.noaa.gov/geomag-web/#declination
        att.yaw = psi;//headingwrap(psi - 8.16);
        //send over serial port
//         scisend_data(imu.ax,imu.ay,imu.az,imu.gx,imu.gy, imu.gz, imu.mx,imu.my,imu.mz);
//         scisend_Magdata(imu.mx,imu.my,imu.mz);
//         scisend_Euler();
// send data via serial with mavlink protocol
// Encode
        /* IMU data exporting */
        float rollspeed = 0.2;
        float pitchspeed = 0.2;
        float yawspeed = 0.2;
//
        mavlink_msg_attitude_pack(1, 1, &message, tm.time_millis,att.roll, att.pitch, att.yaw,
                                  rollspeed, pitchspeed, yawspeed);
//        mavlink_msg_raw_imu_pack(100,200, &message, 0, imu.ax, imu.ay, imu.az, imu.gx, imu.gy, imu.gz, imu.mx, imu.my, imu.mz,0,20);
//        mavlink_msg_command_long_encode(system_id, companion_id, &message, &com);
        unsigned  leng = mavlink_msg_to_send_buffer((uint8_t*) buff, &message);
        for (i = 0; i <= leng; i++)
        {
            scia_xmit(buff[i]);
        }
    }
}
/*------------------------------------------------------------------------*/
/* Initialization */
/*-----------------------------------------------------------------------*/
void init(void)
{
    // Switch to user bank 0
    writeByte(ICM20948_ADDRESS, REG_BANK_SEL, 0x00);
    // check chip ID
    if (readByte(ICM20948_ADDRESS, WHO_AM_I_ICM20948) == 0xEA)
    {
        // Reset
        reset();
        // Select clock source
        writeByte(ICM20948_ADDRESS, PWR_MGMT_1, 0x01);
        DELAY_US(1000);
        // Get sensor resolutions, only need to do this once
        getAres();
        getGres();
        getMres();
        // Initialize device for active mode read of acclerometer, gyroscope, and
        // temperature
        initICM20948();
        // Initialize magnetometer
        magInit();
        DELAY_US(200000);
        // Read the WHO_AM_I register of the magnetometer, this is a good test of
        // communication
        if (ak09916WhoAmI_SPI() != 0x09)
        {
            asm(" ESTOP0");
            //Emulation stop
            for (;;)
            {
            };                  //Loop forever
        }
    }
    else
    {
        asm(" ESTOP0");
        //Emulation stop
        for (;;)
        {
        };                  //Loop forever
    }
}
//
void reset(void)
{
    writeByte(ICM20948_ADDRESS, PWR_MGMT_1, READ_FLAG);
    DELAY_US(20000);
    while (readByte(ICM20948_ADDRESS, PWR_MGMT_1) == READ_FLAG)
    {
        DELAY_US(10000);
    }
    DELAY_US(50000);
}
//
void scisend_data(float ax, float ay, float az, float gx, float gy, float gz,
                  float mx, float my, float mz)
{
    int k;
    int atStep = 0;
    // Raw accel
    if (writeFloat(tm.time))
    {
        for (k = 0; k <= 5; k++)
        {
            scia_xmit(buf[k]);
        }
        scia_xmit(',');
        atStep = 1;
    }
    if (writeFloat(tm.deltat) && atStep == 1)
    {

        for (k = 0; k <= 5; k++)
        {
            scia_xmit(buf[k]);
        }
        scia_xmit(',');
        atStep = 2;
    }
    // Accel
    if (writeFloat(ax) && atStep == 2)
    {
        for (k = 0; k <= 5; k++)
        {
            scia_xmit(buf[k]);
        }
        scia_xmit(',');
        atStep = 3;
    }
    if (writeFloat(ay) && atStep == 3)
    {
        for (k = 0; k <= 5; k++)
        {
            scia_xmit(buf[k]);
        }
        scia_xmit(',');
        atStep = 4;
    }
    if (writeFloat(az) && atStep == 4)
    {
        for (k = 0; k <= 5; k++)
        {
            scia_xmit(buf[k]);
        }
        scia_xmit(',');
        atStep = 5;
    }
    // Gyro
    if (writeFloat(gx) && atStep == 5)
    {
        for (k = 0; k <= 5; k++)
        {
            scia_xmit(buf[k]);
        }
        scia_xmit(',');
        atStep = 6;
    }
    if (writeFloat(gy) && atStep == 6)
    {
        for (k = 0; k <= 5; k++)
        {
            scia_xmit(buf[k]);
        }
        scia_xmit(',');
        atStep = 7;
    }
    if (writeFloat(gz) && atStep == 7)
    {
        for (k = 0; k <= 5; k++)
        {
            scia_xmit(buf[k]);
        }
        scia_xmit(',');
        atStep = 8;
    }
    // Magneto
    if (writeFloat(mx) && atStep == 8)
    {
        for (k = 0; k <= 5; k++)
        {
            scia_xmit(buf[k]);
        }
        scia_xmit(',');
        atStep = 9;
    }
    if (writeFloat(my) && atStep == 9)
    {
        for (k = 0; k <= 5; k++)
        {
            scia_xmit(buf[k]);
        }
        scia_xmit(',');
        atStep = 10;
    }
    if (writeFloat(mz) && atStep == 10)
    {
        for (k = 0; k <= 5; k++)
        {
            scia_xmit(buf[k]);
        }
        scia_xmit(',');
        atStep = 11;
    }

// Quaternion
    if (writeFloat(*(getQ())) && atStep == 11)
    {
        for (k = 0; k <= 5; k++)
        {
            scia_xmit(buf[k]);
        }
        scia_xmit(',');
        atStep = 12;
    }
    if (writeFloat(*(getQ() + 1)) && atStep == 12)
    {
        for (k = 0; k <= 5; k++)
        {
            scia_xmit(buf[k]);
        }
        scia_xmit(',');
        atStep = 13;
    }
    if (writeFloat(*(getQ() + 2)) && atStep == 13)
    {
        for (k = 0; k <= 5; k++)
        {
            scia_xmit(buf[k]);
        }
        scia_xmit(',');
        atStep = 14;
    }
    if (writeFloat(*(getQ() + 3)) && atStep == 14)
    {
        for (k = 0; k <= 5; k++)
        {
            scia_xmit(buf[k]);
        }
        scia_xmit(',');
        atStep = 15;
    }
// Euler
    if (writeFloat(att.pitch) && atStep == 15)
    {
        for (k = 0; k <= 5; k++)
        {
            scia_xmit(buf[k]);
        }
        scia_xmit(',');
        atStep = 16;
    }
    if (writeFloat(att.roll) && atStep == 16)
    {
        for (k = 0; k <= 5; k++)
        {
            scia_xmit(buf[k]);
        }
        scia_xmit(',');
        atStep = 17;
    }
    if (writeFloat(att.yaw) && atStep == 17)
    {
        for (k = 0; k <= 5; k++)
        {
            scia_xmit(buf[k]);
        }
        scia_xmit(' ');
        scia_xmit(' ');
        scia_xmit('\n');
        atStep = 0;
    }
}
//
void scisend_Magdata(float mx, float my, float mz)
{
    int k;
    int atStep = 0;
    // Magneto
    if (writeFloat(mx))
    {
        for (k = 0; k <= 7; k++)
        {
            scia_xmit(buf[k]);
            DELAY_US(500);
        }
        scia_xmit(',');
        DELAY_US(500);
        atStep = 1;
    }
    if (writeFloat(my) && atStep == 1)
    {
        for (k = 0; k <= 7; k++)
        {
            scia_xmit(buf[k]);
            DELAY_US(500);
        }
        scia_xmit(',');
        DELAY_US(500);
        atStep = 2;
    }
    if (writeFloat(my) && atStep == 2)
    {
        for (k = 0; k <= 7; k++)
        {
            scia_xmit(buf[k]);
            DELAY_US(500);
        }
        DELAY_US(500);
        scia_xmit('\n');
        atStep = 0;
    }
}

//
void scisend_Euler(void)
{
    int k;
    int atStep = 0;
    if (writeFloat(tm.deltat))
    {

        for (k = 0; k <= 5; k++)
        {
            scia_xmit(buf[k]);
        }
        scia_xmit(',');
        atStep = 1;
    }
// Euler
    if (writeFloat(att.pitch) && atStep == 1)
    {
        for (k = 0; k <= 5; k++)
        {
            scia_xmit(buf[k]);
        }
        scia_xmit(',');
        atStep = 2;
    }
    if (writeFloat(att.roll) && atStep == 2)
    {
        for (k = 0; k <= 5; k++)
        {
            scia_xmit(buf[k]);
        }
        scia_xmit(',');
        atStep = 3;
    }
    if (writeFloat(att.yaw) && atStep == 3)
    {
        for (k = 0; k <= 5; k++)
        {
            scia_xmit(buf[k]);
        }
        scia_xmit('\n');
        DELAY_US(10000);
        atStep = 0;
    }
}

