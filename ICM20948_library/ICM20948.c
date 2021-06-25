#include "ICM20948.h"
#include "spiconfig.h"
//==============================================================================
//====== Set of useful function to access acceleration. gyroscope, magnetometer,
//====== and temperature data
//==============================================================================

void param_initialize(void)
{
    para.Gscale = GFS_2000DPS;
    para.Ascale = AFS_16G;
    para.Mmode = M_8HZ;
    tm.deltat = 0.0f;
    tm.time = 0.0f;
    tm.lastUpdate = 0;
    tm.firstUpdate = 0;
    tm.Now = 0;
    tm.time_millis = 0;
    int i;
    for (i = 0; i <= 2; i++)
    {
        para.factoryMagCalibration[i] = 0;
        para.factoryMagBias[i] = 0;
        para.gyroBias[i] = 0;
        para.accelBias[i] = 0;
        para.magBias[i] = 0;
        para.magScale[i] = 0;
    }
   imu_raw.magDOR_count = 0;
}
void getMres()
{
    para.mRes = 10.0f * 4912.0f / 32760.0f; // Proper scale to return milliGauss
}

void getGres()
{
    switch (para.Gscale)
    {
    // Possible gyro scales (and their register bit settings) are:
    // 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS (11).
    // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that
    // 2-bit value:
    case GFS_250DPS:
        para.gRes = 250.0f / 32768.0f;
        break;
    case GFS_500DPS:
        para.gRes = 500.0f / 32768.0f;
        break;
    case GFS_1000DPS:
        para.gRes = 1000.0f / 32768.0f;
        break;
    case GFS_2000DPS:
        para.gRes = 2000.0f / 32768.0f;
        break;
    }
}


void getAres()
{
    switch (para.Ascale)
    {
    // Possible accelerometer scales (and their register bit settings) are:
    // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
    // Here's a bit of an algorithm to calculate DPS/(ADC tick) based on that
    // 2-bit value:
    case AFS_2G:
        para.aRes = 2.0f / 32768.0f;
        break;
    case AFS_4G:
        para.aRes = 4.0f / 32768.0f;
        break;
    case AFS_8G:
        para.aRes = 8.0f / 32768.0f;
        break;
    case AFS_16G:
        para.aRes = 16.0f / 32768.0f;
        break;
    }
}
void readAccelData(int16_t * destination)
{
    char rawData[6];  // x/y/z accel register data stored here
    // Read the six raw data registers into data array
    readBytes(ICM20948_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);

    // Turn the MSB and LSB into a signed 16-bit value
    destination[0] = (int16_t) (rawData[0] << 8) | rawData[1];
    destination[1] = (int16_t) (rawData[2] << 8) | rawData[3];
    destination[2] = (int16_t) (rawData[4] << 8) | rawData[5];
}
void readGyroData(int16_t * destination)
{
    char rawData[6];  // x/y/z gyro register data stored here
    // Read the six raw data registers sequentially into data array
    readBytes(ICM20948_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);
    // Turn the MSB and LSB into a signed 16-bit value
    destination[0] = (int16_t) (rawData[0] << 8) | rawData[1];
    destination[1] = (int16_t) (rawData[2] << 8) | rawData[3];
    destination[2] = (int16_t) (rawData[4] << 8) | rawData[5];
}
void readMagData(int16_t * destination)
{
    // x/y/z gyro register data, ST2 register stored here, must read ST2 at end
    // of data acquisition
    char rawData[8];
    char ST1_Status;
    // Switch to user bank 3
    writeByte(AK09916_ADDRESS, REG_BANK_SEL, 0x30);
    // Set the I2C slave addres of AK09916 and set for read
    writeByte(AK09916_ADDRESS, I2C_SLV0_ADDR, AK09916_ADDRESS | READ_FLAG);
    // I2C slave 0 register address from where to begin data transfer
    writeByte(AK09916_ADDRESS, I2C_SLV0_REG, AK09916_ST1);
    // Enable 1-byte reads on slave 0
    writeByte(AK09916_ADDRESS, I2C_SLV0_CTRL, 0x81);
    DELAY_US(100);
    // Switch to user bank 0
    writeByte(AK09916_ADDRESS, REG_BANK_SEL, 0x00);
    // Wait for magnetometer data ready bit to be set
    ST1_Status = readByte(AK09916_ADDRESS, EXT_SENS_DATA_00 | READ_FLAG);
    DELAY_US(50);
    if (ST1_Status & 0x01)
    {
        // Switch to user bank 3
        writeByte(AK09916_ADDRESS, REG_BANK_SEL, 0x30);
        // Set the I2C slave address of AK09916 and set for read.
        writeByte(AK09916_ADDRESS, I2C_SLV0_ADDR, AK09916_ADDRESS | READ_FLAG);
        // I2C slave 0 register address from where to begin data transfer
        writeByte(AK09916_ADDRESS, I2C_SLV0_REG, AK09916_XOUT_L);
        // Enable I2C and read 7 bytes
        writeByte(AK09916_ADDRESS, I2C_SLV0_CTRL, 0x88);
        DELAY_US(2000);
        // Switch to user bank 0
        writeByte(AK09916_ADDRESS, REG_BANK_SEL, 0x00);
        // Read the six raw data and ST2 registers sequentially into data array
        readBytes(AK09916_ADDRESS, EXT_SENS_DATA_00, 8, &rawData[0]);
        char c = rawData[7]; // End data read by reading ST2 register
        // Check if magnetic sensor overflow set, if not then report data
        // Remove once finished

        if (!(c & 0x08))
        {
            // Turn the MSB and LSB into a signed 16-bit value
            destination[0] = ((int16_t) rawData[1] << 8) | rawData[0];
            // Data stored as little Endian
            destination[1] = ((int16_t) rawData[3] << 8) | rawData[2];
            destination[2] = ((int16_t) rawData[5] << 8) | rawData[4];
        }
    }
    if (ST1_Status & 0x02)
    {
        // Switch to user bank 3
        writeByte(AK09916_ADDRESS, REG_BANK_SEL, 0x30);
        // Set the I2C slave address of AK09916 and set for read.
        writeByte(AK09916_ADDRESS, I2C_SLV0_ADDR, AK09916_ADDRESS | READ_FLAG);
        // I2C slave 0 register address from where to begin data transfer
        writeByte(AK09916_ADDRESS, I2C_SLV0_REG, AK09916_ST2);
        // Enable I2C and read 7 bytes
        writeByte(AK09916_ADDRESS, I2C_SLV0_CTRL, 0x81);
        DELAY_US(2000);
        // Switch to user bank 0
        writeByte(AK09916_ADDRESS, REG_BANK_SEL, 0x00);
        // Read the six raw data and ST2 registers sequentially into data array
        readByte(AK09916_ADDRESS, EXT_SENS_DATA_00 | READ_FLAG);
        imu_raw.magDOR_count++;
    }
}


int16_t readTempData()
{
    char rawData[2]; // x/y/z gyro register data stored here
    // Read the two raw data registers sequentially into data array
    readBytes(ICM20948_ADDRESS, TEMP_OUT_H, 2, &rawData[0]);
    // Turn the MSB and LSB into a 16-bit value
    return ((int16_t) rawData[0] << 8) | rawData[1];
}

//
void readSensorConfig(void)
{
    char rawData[2];
    // Switch to user bank 2
    writeByte(ICM20948_ADDRESS, REG_BANK_SEL, 0x20);
    //
    readBytes(ICM20948_ADDRESS, GYRO_CONFIG_1, 2, &rawData[0]);
    sensorConfig.gyroConfig1 = rawData[0];// readByte(ICM20948_ADDRESS, GYRO_CONFIG_1);
    sensorConfig.gyroConfig2 = rawData[1];// readByte(ICM20948_ADDRESS, GYRO_CONFIG_2);
    sensorConfig.gyrosamplerate =  readByte(ICM20948_ADDRESS, GYRO_SMPLRT_DIV);
    sensorConfig.accelConfig1 =  readByte(ICM20948_ADDRESS, ACCEL_CONFIG);
    sensorConfig.accelConfig2 =  readByte(ICM20948_ADDRESS, ACCEL_CONFIG_2);
    readBytes(ICM20948_ADDRESS, ACCEL_SMPLRT_DIV_1, 2, &rawData[0]);
    sensorConfig.accelsamplerate = (uint16_t) (rawData[0] << 8) | rawData[1];
    sensorConfig.tempConfig = readByte(ICM20948_ADDRESS, TEMP_CONFIG);
    // Switch to user bank 3
    writeByte(AK09916_ADDRESS, REG_BANK_SEL, 0x30);
    // Set the I2C slave addres of AK09916 and set for read
    writeByte(AK09916_ADDRESS, I2C_SLV0_ADDR, AK09916_ADDRESS | READ_FLAG);
    // I2C slave 0 register address from where to begin data transfer
    writeByte(AK09916_ADDRESS, I2C_SLV0_REG, AK09916_CNTL2);
    // Enable 1-byte reads on slave 0
    writeByte(AK09916_ADDRESS, I2C_SLV0_CTRL, 0x81);
    // Switch to user bank 0
    writeByte(AK09916_ADDRESS, REG_BANK_SEL, 0x00);
    DELAY_US(200000);
    // Read WIA register
    sensorConfig.mag_ctrl2 = readByte(AK09916_ADDRESS, EXT_SENS_DATA_00 | READ_FLAG);
    // Switch to user bank 0
    writeByte(ICM20948_ADDRESS, REG_BANK_SEL, 0x00);
    sensorConfig.whoamI = readByte(ICM20948_ADDRESS, WHO_AM_I_ICM20948);
    sensorConfig.userctrl = readByte(ICM20948_ADDRESS, USER_CTRL);
    sensorConfig.LPconfig = readByte(ICM20948_ADDRESS, LP_CONFIG);
    readBytes(ICM20948_ADDRESS, PWR_MGMT_1, 2, &rawData[0]);
    sensorConfig.pwrmgmt = (uint16_t) (rawData[1] << 8) | rawData[0];
    sensorConfig.INT_PIN_config = readByte(ICM20948_ADDRESS, INT_PIN_CFG);
    readBytes(ICM20948_ADDRESS, INT_ENABLE, 2, &rawData[0]);
    sensorConfig.interrupt01Config =  (uint16_t) (rawData[1] << 8)| rawData[0];
    readBytes(ICM20948_ADDRESS, INT_ENABLE_2, 2, &rawData[0]);
    sensorConfig.interrupt23Config =  (uint16_t) (rawData[1] << 8)| rawData[0];
}
// Calculate the time the last update took for use in the quaternion filters
// TODO: This doesn't really belong in this class.
void updateTime()
{
    // Set integration time by time elapsed since last filter update
    tm.deltat = ((tm.Now - tm.lastUpdate) / 1000000.0f);
    tm.time = tm.Now/ 1000000.0f;
    tm.lastUpdate = tm.Now;

}

void initAK09916()
{
    // Switch to user bank 3
    writeByte(AK09916_ADDRESS, REG_BANK_SEL, 0x30);
    // Set the I2C slave address of AK09916 and set for write.
    writeByte(AK09916_ADDRESS, I2C_SLV0_ADDR, AK09916_ADDRESS);
    // I2C slave 0 register address from where to begin data transfer
    writeByte(AK09916_ADDRESS, I2C_SLV0_REG, AK09916_CNTL3);
    // Reset AK09916
    writeByte(AK09916_ADDRESS, I2C_SLV0_DO, 0x01);
    // Enable I2C and write 1 byte
    writeByte(AK09916_ADDRESS, I2C_SLV0_CTRL, 0x81);
    DELAY_US(50000);
    // Set the I2C slave address of AK09916 and set for write.
    writeByte(AK09916_ADDRESS, I2C_SLV0_ADDR, AK09916_ADDRESS);
    // I2C slave 0 register address from where to begin data transfer
    writeByte(AK09916_ADDRESS, I2C_SLV0_REG, AK09916_CNTL2);
    // Power down magnetometer
    writeByte(AK09916_ADDRESS, I2C_SLV0_DO, 0x00);
    // Enable I2C and write 1 byte
    writeByte(AK09916_ADDRESS, I2C_SLV0_CTRL, 0x81);
    DELAY_US(50000);
    // Set the I2C slave address of AK09916 and set for write.
    writeByte(AK09916_ADDRESS, I2C_SLV0_ADDR, AK09916_ADDRESS);
    // I2C slave 0 register address from where to begin data transfer
    writeByte(AK09916_ADDRESS, I2C_SLV0_REG, AK09916_CNTL2);
    // Enter  Continuous measurement mode 4
    writeByte(AK09916_ADDRESS, I2C_SLV0_DO, 0x08);
    // Enable I2C and write 1 byte
    writeByte(AK09916_ADDRESS, I2C_SLV0_CTRL, 0x81);
    DELAY_US(50000);
    // Switch to user bank 0
    writeByte(AK09916_ADDRESS, REG_BANK_SEL, 0x00);
}

// return magnetometer sensor ID
BYTE ak09916WhoAmI_SPI()
{
    BYTE response;
    // Switch to user bank 3
    writeByte(AK09916_ADDRESS, REG_BANK_SEL, 0x30);
    // Set the I2C slave addres of AK09916 and set for read
    writeByte(AK09916_ADDRESS, I2C_SLV0_ADDR, AK09916_ADDRESS | READ_FLAG);
    // I2C slave 0 register address from where to begin data transfer
    writeByte(AK09916_ADDRESS, I2C_SLV0_REG, WHO_AM_I_AK09916);
    // Enable 1-byte reads on slave 0
    writeByte(AK09916_ADDRESS, I2C_SLV0_CTRL, 0x81);
    // Switch to user bank 0
    writeByte(AK09916_ADDRESS, REG_BANK_SEL, 0x00);
    DELAY_US(200000);
    // Read WIA register
    response = readByte(AK09916_ADDRESS, EXT_SENS_DATA_00 | READ_FLAG);

    return response;
}

// Initialize Magnetometer
void magInit()
{
    // Switch to user bank 0
    writeByte(AK09916_ADDRESS, REG_BANK_SEL, 0x00);
    // Enable I2C master mode
    // TODO Why not do this 11-100 ms after power up?
//    writeByte(AK09916_ADDRESS, USER_CTRL, 0x20);
    regedit(AK09916_ADDRESS, USER_CTRL, 1, 1, 5);
    // Switch to user bank 3
    writeByte(AK09916_ADDRESS, REG_BANK_SEL, 0x30);
    // Disable multi-master and set I2C master clock to 400 kHz
    writeByte(AK09916_ADDRESS, I2C_MST_CTRL, 0x17);
    //==========Reset==============//
    // Set the I2C slave address of AK09916 and set for write.
    writeByte(AK09916_ADDRESS, I2C_SLV0_ADDR, AK09916_ADDRESS);
    // I2C slave 0 register address from where to begin data transfer
    writeByte(AK09916_ADDRESS, I2C_SLV0_REG, AK09916_CNTL3);
    // Reset AK09916
    writeByte(AK09916_ADDRESS, I2C_SLV0_DO, 0x01);
    // Enable I2C and write 1 byte
    writeByte(AK09916_ADDRESS, I2C_SLV0_CTRL, 0x81);
    DELAY_US(50000);
    //==========Mode change==============//
    // Point save 0 register at AK09916's control 2 (soft reset) register
    writeByte(AK09916_ADDRESS, I2C_SLV0_REG, AK09916_CNTL2);
    // Send 0x01 to AK09916 via slave 0 to trigger a soft restart
    writeByte(AK09916_ADDRESS, I2C_SLV0_DO, 0x00);
    // Enable simple 1-byte I2C reads from slave 0
    writeByte(AK09916_ADDRESS, I2C_SLV0_CTRL, 0x81);
    DELAY_US(100);
    // Point save 0 register at AK09916's control 1 (mode) register
    writeByte(AK09916_ADDRESS, I2C_SLV0_REG, AK09916_CNTL2);
    // 16-bit continuous measurement mode 4
    writeByte(AK09916_ADDRESS, I2C_SLV0_DO, 0x08);
    // Enable simple 1-byte I2C reads from slave 0
    writeByte(AK09916_ADDRESS, I2C_SLV0_CTRL, 0x81);
    DELAY_US(100);
    // Switch to user bank 0
    writeByte(AK09916_ADDRESS, REG_BANK_SEL, 0x00);
}
void initICM20948()
{
    // Switch to user bank 2
    writeByte(ICM20948_ADDRESS, REG_BANK_SEL, 0x20);
    // Set gyroscope full scale range to 2000 dps
    // Set sample rate = gyroscope output rate/(1 + GYRO_SMPLRT_DIV)
    // Use a 102 Hz rate; a rate consistent with the filter update rate
//    regedit(ICM20948_ADDRESS, GYRO_CONFIG_1, 0x03, 2, 1);
    writeByte(ICM20948_ADDRESS, GYRO_CONFIG_1, 0x17);
    regedit(ICM20948_ADDRESS, GYRO_CONFIG_2, 0x03, 2, 0);
    writeByte(ICM20948_ADDRESS, GYRO_SMPLRT_DIV, 0x0A);
    // Write new ACCEL_CONFIG register value
    writeByte(ICM20948_ADDRESS, ACCEL_CONFIG, 0x3F);
    regedit(ICM20948_ADDRESS, ACCEL_CONFIG_2, 0x01, 2, 0);
    // Set accelerometer sample rate configuration
    writeByte(ICM20948_ADDRESS, ACCEL_SMPLRT_DIV_1, 0x00);
    writeByte(ICM20948_ADDRESS, ACCEL_SMPLRT_DIV_2, 0x0A);
    // Temprature configuration
    writeByte(ICM20948_ADDRESS, TEMP_CONFIG, 0x03);
    // Switch to user bank 0
    writeByte(ICM20948_ADDRESS, REG_BANK_SEL, 0x00);
    // configure low power mode
    writeByte(ICM20948_ADDRESS, LP_CONFIG, 0x70);
    // enable Raw data ready interrupt
    writeByte(ICM20948_ADDRESS, INT_ENABLE_1, 0x01);

}

// Function which accumulates gyro and accelerometer data after device
// initialization. It calculates the average of the at-rest readings and then
// loads the resulting offsets into accelerometer and gyro bias registers.
void calibrateICM20948(float * gyroBias, float * accelBias)
{
    char data[12]; // data array to hold accelerometer and gyro x, y, z, data
    uint16_t ii, packet_count, fifo_count;
    int32_t gyro_bias[3] = { 0, 0, 0 }, accel_bias[3] = { 0, 0, 0 };

    // reset device
    // Write a one to bit 7 reset bit; toggle reset device
    writeByte(ICM20948_ADDRESS, PWR_MGMT_1, READ_FLAG);
    DELAY_US(200000);

    // get stable time source; Auto select clock source to be PLL gyroscope
    // reference if ready else use the internal oscillator, bits 2:0 = 001
    writeByte(ICM20948_ADDRESS, PWR_MGMT_1, 0x01);
    DELAY_US(200000);

    // Configure device for bias calculation
    // Disable all interrupts
    writeByte(ICM20948_ADDRESS, INT_ENABLE, 0x00);
    // Disable FIFO
    writeByte(ICM20948_ADDRESS, FIFO_EN_1, 0x00);
    writeByte(ICM20948_ADDRESS, FIFO_EN_2, 0x00);
    // Turn on internal clock source
    writeByte(ICM20948_ADDRESS, PWR_MGMT_1, 0x00);
    // Disable I2C master
    //writeByte(ICM20948_ADDRESS, I2C_MST_CTRL, 0x00); Already disabled
    // Disable FIFO and I2C master modes
    writeByte(ICM20948_ADDRESS, USER_CTRL, 0x00);
    // Reset FIFO and DMP
    writeByte(ICM20948_ADDRESS, USER_CTRL, 0x08);
    writeByte(ICM20948_ADDRESS, FIFO_RST, 0x1F);
    DELAY_US(10000);
    writeByte(ICM20948_ADDRESS, FIFO_RST, 0x00);
    DELAY_US(15000);

    // Set FIFO mode to snapshot
    writeByte(ICM20948_ADDRESS, FIFO_MODE, 0x1F);
    // Switch to user bank 2
    writeByte(ICM20948_ADDRESS, REG_BANK_SEL, 0x20);
    // Configure ICM20948 gyro and accelerometer for bias calculation
    // Set low-pass filter to 188 Hz
    writeByte(ICM20948_ADDRESS, GYRO_CONFIG_1, 0x01);
    // Set sample rate to 1 kHz
    writeByte(ICM20948_ADDRESS, GYRO_SMPLRT_DIV, 0x00);
    // Set gyro full-scale to 250 degrees per second, maximum sensitivity
    writeByte(ICM20948_ADDRESS, GYRO_CONFIG_1, 0x00);
    // Set accelerometer full-scale to 2 g, maximum sensitivity
    writeByte(ICM20948_ADDRESS, ACCEL_CONFIG, 0x00);

    uint16_t gyrosensitivity = 131;   // = 131 LSB/degrees/sec
    uint16_t accelsensitivity = 16384; // = 16384 LSB/g

    // Switch to user bank 0
    writeByte(ICM20948_ADDRESS, REG_BANK_SEL, 0x00);
    // Configure FIFO to capture accelerometer and gyro data for bias calculation
    writeByte(ICM20948_ADDRESS, USER_CTRL, 0x40);  // Enable FIFO
    // Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in
    // ICM20948)
    writeByte(ICM20948_ADDRESS, FIFO_EN_2, 0x1E);
    DELAY_US(40000);  // accumulate 40 samples in 40 milliseconds = 480 bytes

    // At end of sample accumulation, turn off FIFO sensor read
    // Disable gyro and accelerometer sensors for FIFO
    writeByte(ICM20948_ADDRESS, FIFO_EN_2, 0x00);
    // Read FIFO sample count
    readBytes(ICM20948_ADDRESS, FIFO_COUNTH, 2, &data[0]);
    fifo_count = ((uint16_t) data[0] << 8) | data[1];
    // How many sets of full gyro and accelerometer data for averaging
    packet_count = fifo_count / 12;

    for (ii = 0; ii < packet_count; ii++)
    {
        int16_t accel_temp[3] = { 0, 0, 0 }, gyro_temp[3] = { 0, 0, 0 };
        // Read data for averaging
        readBytes(ICM20948_ADDRESS, FIFO_R_W, 12, &data[0]);
        // Form signed 16-bit integer for each sample in FIFO
        accel_temp[0] = (int16_t) (((int16_t) data[0] << 8) | data[1]);
        accel_temp[1] = (int16_t) (((int16_t) data[2] << 8) | data[3]);
        accel_temp[2] = (int16_t) (((int16_t) data[4] << 8) | data[5]);
        gyro_temp[0] = (int16_t) (((int16_t) data[6] << 8) | data[7]);
        gyro_temp[1] = (int16_t) (((int16_t) data[8] << 8) | data[9]);
        gyro_temp[2] = (int16_t) (((int16_t) data[10] << 8) | data[11]);

        // Sum individual signed 16-bit biases to get accumulated signed 32-bit
        // biases.
        accel_bias[0] += (int32_t) accel_temp[0];
        accel_bias[1] += (int32_t) accel_temp[1];
        accel_bias[2] += (int32_t) accel_temp[2];
        gyro_bias[0] += (int32_t) gyro_temp[0];
        gyro_bias[1] += (int32_t) gyro_temp[1];
        gyro_bias[2] += (int32_t) gyro_temp[2];
    }
    // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
    accel_bias[0] /= (int32_t) packet_count;
    accel_bias[1] /= (int32_t) packet_count;
    accel_bias[2] /= (int32_t) packet_count;
    gyro_bias[0] /= (int32_t) packet_count;
    gyro_bias[1] /= (int32_t) packet_count;
    gyro_bias[2] /= (int32_t) packet_count;

    // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
    if (accel_bias[2] > 0L)
    {
        accel_bias[2] -= (int32_t) accelsensitivity;
    }
    else
    {
        accel_bias[2] += (int32_t) accelsensitivity;
    }

    // Construct the gyro biases for push to the hardware gyro bias registers,
    // which are reset to zero upon device startup.
    // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input
    // format.
    data[0] = (-gyro_bias[0] / 4 >> 8) & 0xFF;
    // Biases are additive, so change sign on calculated average gyro biases
    data[1] = (-gyro_bias[0] / 4) & 0xFF;
    data[2] = (-gyro_bias[1] / 4 >> 8) & 0xFF;
    data[3] = (-gyro_bias[1] / 4) & 0xFF;
    data[4] = (-gyro_bias[2] / 4 >> 8) & 0xFF;
    data[5] = (-gyro_bias[2] / 4) & 0xFF;

    // Switch to user bank 2
    writeByte(ICM20948_ADDRESS, REG_BANK_SEL, 0x20);

    // Push gyro biases to hardware registers
    writeByte(ICM20948_ADDRESS, XG_OFFSET_H, data[0]);
    writeByte(ICM20948_ADDRESS, XG_OFFSET_L, data[1]);
    writeByte(ICM20948_ADDRESS, YG_OFFSET_H, data[2]);
    writeByte(ICM20948_ADDRESS, YG_OFFSET_L, data[3]);
    writeByte(ICM20948_ADDRESS, ZG_OFFSET_H, data[4]);
    writeByte(ICM20948_ADDRESS, ZG_OFFSET_L, data[5]);

    // Output scaled gyro biases for display in the main program
    gyroBias[0] = (float) gyro_bias[0] / (float) gyrosensitivity;
    gyroBias[1] = (float) gyro_bias[1] / (float) gyrosensitivity;
    gyroBias[2] = (float) gyro_bias[2] / (float) gyrosensitivity;

    // Construct the accelerometer biases for push to the hardware accelerometer
    // bias registers. These registers contain factory trim values which must be
    // added to the calculated accelerometer biases; on boot up these registers
    // will hold non-zero values. In addition, bit 0 of the lower byte must be
    // preserved since it is used for temperature compensation calculations.
    // Accelerometer bias registers expect bias input as 2048 LSB per g, so that
    // the accelerometer biases calculated above must be divided by 8.

    // Switch to user bank 1
    writeByte(ICM20948_ADDRESS, REG_BANK_SEL, 0x10);
    // A place to hold the factory accelerometer trim biases
    int32_t accel_bias_reg[3] = { 0, 0, 0 };
    // Read factory accelerometer trim values
    readBytes(ICM20948_ADDRESS, XA_OFFSET_H, 2, &data[0]);
    accel_bias_reg[0] = (int32_t) (((int16_t) data[0] << 8) | data[1]);
    readBytes(ICM20948_ADDRESS, YA_OFFSET_H, 2, &data[0]);
    accel_bias_reg[1] = (int32_t) (((int16_t) data[0] << 8) | data[1]);
    readBytes(ICM20948_ADDRESS, ZA_OFFSET_H, 2, &data[0]);
    accel_bias_reg[2] = (int32_t) (((int16_t) data[0] << 8) | data[1]);

    // Define mask for temperature compensation bit 0 of lower byte of
    // accelerometer bias registers
    uint32_t mask = 1uL;
    // Define array to hold mask bit for each accelerometer bias axis
    char mask_bit[3] = { 0, 0, 0 };

    for (ii = 0; ii < 3; ii++)
    {
        // If temperature compensation bit is set, record that fact in mask_bit
        if ((accel_bias_reg[ii] & mask))
        {
            mask_bit[ii] = 0x01;
        }
    }

    // Construct total accelerometer bias, including calculated average
    // accelerometer bias from above
    // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g
    // (16 g full scale)
    accel_bias_reg[0] -= (accel_bias[0] / 8);
    accel_bias_reg[1] -= (accel_bias[1] / 8);
    accel_bias_reg[2] -= (accel_bias[2] / 8);

    data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
    data[1] = (accel_bias_reg[0]) & 0xFF;
    // preserve temperature compensation bit when writing back to accelerometer
    // bias registers
    data[1] = data[1] | mask_bit[0];
    data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
    data[3] = (accel_bias_reg[1]) & 0xFF;
    // Preserve temperature compensation bit when writing back to accelerometer
    // bias registers
    data[3] = data[3] | mask_bit[1];
    data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
    data[5] = (accel_bias_reg[2]) & 0xFF;
    // Preserve temperature compensation bit when writing back to accelerometer
    // bias registers
    data[5] = data[5] | mask_bit[2];

    // Apparently this is not working for the acceleration biases in the ICM-20948
    // Are we handling the temperature correction bit properly?
    // Push accelerometer biases to hardware registers
    writeByte(ICM20948_ADDRESS, XA_OFFSET_H, data[0]);
    writeByte(ICM20948_ADDRESS, XA_OFFSET_L, data[1]);
    writeByte(ICM20948_ADDRESS, YA_OFFSET_H, data[2]);
    writeByte(ICM20948_ADDRESS, YA_OFFSET_L, data[3]);
    writeByte(ICM20948_ADDRESS, ZA_OFFSET_H, data[4]);
    writeByte(ICM20948_ADDRESS, ZA_OFFSET_L, data[5]);

    // Output scaled accelerometer biases for display in the main program
    accelBias[0] = (float) accel_bias[0] / (float) accelsensitivity;
    accelBias[1] = (float) accel_bias[1] / (float) accelsensitivity;
    accelBias[2] = (float) accel_bias[2] / (float) accelsensitivity;
    // Switch to user bank 0
    writeByte(ICM20948_ADDRESS, REG_BANK_SEL, 0x00);
}

// Accelerometer and gyroscope self test; check calibration wrt factory settings
// Should return percent deviation from factory trim values, +/- 14 or less
// deviation is a pass.
void ICM20948SelfTest(float * destination)
{
    char rawData[6] = { 0, 0, 0, 0, 0, 0 };
    char selfTest[6];
    int32_t gAvg[3] = { 0 }, aAvg[3] = { 0 }, aSTAvg[3] = { 0 }, gSTAvg[3] =
            { 0 };
    float factoryTrim[6];
    char FS = 0;

    // Get stable time source
    // Auto select clock source to be PLL gyroscope reference if ready else
    writeByte(ICM20948_ADDRESS, PWR_MGMT_1, 0x01);
    DELAY_US(200000);

    // Switch to user bank 2
    writeByte(ICM20948_ADDRESS, REG_BANK_SEL, 0x20);
    // Set gyro sample rate to 1 kHz
    writeByte(ICM20948_ADDRESS, GYRO_SMPLRT_DIV, 0x00);
    // Set gyro sample rate to 1 kHz, DLPF to 119.5 Hz and FSR to 250 dps
    writeByte(ICM20948_ADDRESS, GYRO_CONFIG_1, 0x11);
    // Set accelerometer rate to 1 kHz and bandwidth to 111.4 Hz
    // Set full scale range for the accelerometer to 2 g
    writeByte(ICM20948_ADDRESS, ACCEL_CONFIG, 0x11);
    // Switch to user bank 0
    writeByte(ICM20948_ADDRESS, REG_BANK_SEL, 0x00);

    // Get average current values of gyro and acclerometer
    int ii;
    for (ii = 0; ii < 200; ii++)
    {
        // Read the six raw data registers into data array
        readBytes(ICM20948_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);
        // Turn the MSB and LSB into a signed 16-bit value
        aAvg[0] += (int16_t) (((int16_t) rawData[0] << 8) | rawData[1]);
        aAvg[1] += (int16_t) (((int16_t) rawData[2] << 8) | rawData[3]);
        aAvg[2] += (int16_t) (((int16_t) rawData[4] << 8) | rawData[5]);

        // Read the six raw data registers sequentially into data array
        readBytes(ICM20948_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);
        // Turn the MSB and LSB into a signed 16-bit value
        gAvg[0] += (int16_t) (((int16_t) rawData[0] << 8) | rawData[1]);
        gAvg[1] += (int16_t) (((int16_t) rawData[2] << 8) | rawData[3]);
        gAvg[2] += (int16_t) (((int16_t) rawData[4] << 8) | rawData[5]);
    }

    // Get average of 200 values and store as average current readings

    for (ii = 0; ii < 3; ii++)
    {
        aAvg[ii] /= 200;
        gAvg[ii] /= 200;
    }

    // Switch to user bank 2
    writeByte(ICM20948_ADDRESS, REG_BANK_SEL, 0x20);

    // Configure the accelerometer for self-test
    // Enable self test on all three axes and set accelerometer range to +/- 2 g
    writeByte(ICM20948_ADDRESS, ACCEL_CONFIG_2, 0x1C);
    // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
    writeByte(ICM20948_ADDRESS, GYRO_CONFIG_2, 0x38);
    DELAY_US(25000);  // Delay a while to let the device stabilize

    // Switch to user bank 0
    writeByte(ICM20948_ADDRESS, REG_BANK_SEL, 0x00);

    // Get average self-test values of gyro and acclerometer
    for (ii = 0; ii < 200; ii++)
    {
        // Read the six raw data registers into data array
        readBytes(ICM20948_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);
        // Turn the MSB and LSB into a signed 16-bit value
        aSTAvg[0] += (int16_t) (((int16_t) rawData[0] << 8) | rawData[1]);
        aSTAvg[1] += (int16_t) (((int16_t) rawData[2] << 8) | rawData[3]);
        aSTAvg[2] += (int16_t) (((int16_t) rawData[4] << 8) | rawData[5]);

        // Read the six raw data registers sequentially into data array
        readBytes(ICM20948_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);
        // Turn the MSB and LSB into a signed 16-bit value
        gSTAvg[0] += (int16_t) (((int16_t) rawData[0] << 8) | rawData[1]);
        gSTAvg[1] += (int16_t) (((int16_t) rawData[2] << 8) | rawData[3]);
        gSTAvg[2] += (int16_t) (((int16_t) rawData[4] << 8) | rawData[5]);
    }

    // Get average of 200 values and store as average self-test readings
    for (ii = 0; ii < 3; ii++)
    {
        aSTAvg[ii] /= 200;
        gSTAvg[ii] /= 200;
    }

    // Switch to user bank 2
    writeByte(ICM20948_ADDRESS, REG_BANK_SEL, 0x20);

    // Configure the gyro and accelerometer for normal operation
    writeByte(ICM20948_ADDRESS, ACCEL_CONFIG_2, 0x00);
    writeByte(ICM20948_ADDRESS, GYRO_CONFIG_2, 0x00);
    DELAY_US(25000);  // Delay a while to let the device stabilize

    // Switch to user bank 1
    writeByte(ICM20948_ADDRESS, REG_BANK_SEL, 0x10);

    // Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg
    // X-axis accel self-test results
    selfTest[0] = readByte(ICM20948_ADDRESS, SELF_TEST_X_ACCEL);
    // Y-axis accel self-test results
    selfTest[1] = readByte(ICM20948_ADDRESS, SELF_TEST_Y_ACCEL);
    // Z-axis accel self-test results
    selfTest[2] = readByte(ICM20948_ADDRESS, SELF_TEST_Z_ACCEL);
    // X-axis gyro self-test results
    selfTest[3] = readByte(ICM20948_ADDRESS, SELF_TEST_X_GYRO);
    // Y-axis gyro self-test results
    selfTest[4] = readByte(ICM20948_ADDRESS, SELF_TEST_Y_GYRO);
    // Z-axis gyro self-test results
    selfTest[5] = readByte(ICM20948_ADDRESS, SELF_TEST_Z_GYRO);

    // Switch to user bank 0
    writeByte(ICM20948_ADDRESS, REG_BANK_SEL, 0x00);

    // Retrieve factory self-test value from self-test code reads
    // FT[Xa] factory trim calculation
    factoryTrim[0] = (float) (2620 / 1 << FS)
            * (pow(1.01, ((float) selfTest[0] - 1.0)));
    // FT[Ya] factory trim calculation
    factoryTrim[1] = (float) (2620 / 1 << FS)
            * (pow(1.01, ((float) selfTest[1] - 1.0)));
    // FT[Za] factory trim calculation
    factoryTrim[2] = (float) (2620 / 1 << FS)
            * (pow(1.01, ((float) selfTest[2] - 1.0)));
    // FT[Xg] factory trim calculation
    factoryTrim[3] = (float) (2620 / 1 << FS)
            * (pow(1.01, ((float) selfTest[3] - 1.0)));
    // FT[Yg] factory trim calculation
    factoryTrim[4] = (float) (2620 / 1 << FS)
            * (pow(1.01, ((float) selfTest[4] - 1.0)));
    // FT[Zg] factory trim calculation
    factoryTrim[5] = (float) (2620 / 1 << FS)
            * (pow(1.01, ((float) selfTest[5] - 1.0)));

    // Report results as a ratio of (STR - FT)/FT; the change from Factory Trim
    // of the Self-Test Response
    // To get percent, must multiply by 100
    int i;
    for (i = 0; i < 3; i++)
    {
        // Report percent differences
        destination[i] = 100.0 * ((float) (aSTAvg[i] - aAvg[i]))
                / factoryTrim[i] - 100.0;
        // Report percent differences
        destination[i + 3] = 100.0 * ((float) (gSTAvg[i] - gAvg[i]))
                / factoryTrim[i + 3] - 100.0;
    }
}

// Function which accumulates magnetometer data after device initialization.
// It calculates the bias and scale in the x, y, and z axes.
void magCalICM20948(float * bias_dest, float * scale_dest)
{
    uint16_t ii = 0, sample_count = 0;
    int32_t mag_bias[3] = { 0, 0, 0 }, mag_scale[3] = { 0, 0, 0 };
    int16_t mag_max[3] = { 0x8000, 0x8000, 0x8000 }, mag_min[3] = { 0x7FFF,
                                                                    0x7FFF,
                                                                    0x7FFF },
            mag_temp[3] = { 0, 0, 0 };

    // Make sure resolution has been calculated
    getMres();

    //  Serial.println(F("Mag Calibration: Wave device in a figure 8 until done!"));
    //  Serial.println(F("  4 seconds to get ready followed by 15 seconds of sampling)"));
    DELAY_US(4000000);

    // shoot for ~fifteen seconds of mag data
    // at 8 Hz ODR, new mag data is available every 125 ms
    if (para.Mmode == M_8HZ)
    {
        sample_count = 128;
    }
    // at 100 Hz ODR, new mag data is available every 10 ms
    if (para.Mmode == M_100HZ)
    {
        sample_count = 1500;
    }

    for (ii = 0; ii < sample_count; ii++)
    {
        readMagData(mag_temp);  // Read the mag data
        int jj;
        for (jj = 0; jj < 3; jj++)
        {
            if (mag_temp[jj] > mag_max[jj])
            {
                mag_max[jj] = mag_temp[jj];
            }
            if (mag_temp[jj] < mag_min[jj])
            {
                mag_min[jj] = mag_temp[jj];
            }
        }

        if (para.Mmode == M_8HZ)
        {
            DELAY_US(135000); // At 8 Hz ODR, new mag data is available every 125 ms
        }
        if (para.Mmode == M_100HZ)
        {
            DELAY_US(12000); // At 100 Hz ODR, new mag data is available every 10 ms
        }
    }

    // Serial.println("mag x min/max:"); Serial.println(mag_max[0]); Serial.println(mag_min[0]);
    // Serial.println("mag y min/max:"); Serial.println(mag_max[1]); Serial.println(mag_min[1]);
    // Serial.println("mag z min/max:"); Serial.println(mag_max[2]); Serial.println(mag_min[2]);

    // Get hard iron correction
    // Get 'average' x mag bias in counts
    mag_bias[0] = (mag_max[0] + mag_min[0]) / 2;
    // Get 'average' y mag bias in counts
    mag_bias[1] = (mag_max[1] + mag_min[1]) / 2;
    // Get 'average' z mag bias in counts
    mag_bias[2] = (mag_max[2] + mag_min[2]) / 2;

    // Save mag biases in G for main program
    bias_dest[0] = (float) mag_bias[0] * para.mRes; // * factoryMagCalibration[0];
    bias_dest[1] = (float) mag_bias[1] * para.mRes; // * factoryMagCalibration[1];
    bias_dest[2] = (float) mag_bias[2] * para.mRes; // * factoryMagCalibration[2];

    // Get soft iron correction estimate
    // Get average x axis max chord length in counts
    mag_scale[0] = (mag_max[0] - mag_min[0]) / 2;
    // Get average y axis max chord length in counts
    mag_scale[1] = (mag_max[1] - mag_min[1]) / 2;
    // Get average z axis max chord length in counts
    mag_scale[2] = (mag_max[2] - mag_min[2]) / 2;

    float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
    avg_rad /= 3.0;

    scale_dest[0] = avg_rad / ((float) mag_scale[0]);
    scale_dest[1] = avg_rad / ((float) mag_scale[1]);
    scale_dest[2] = avg_rad / ((float) mag_scale[2]);

    //  Serial.println(F("Mag Calibration done!"));
}

    // Wire.h read and write protocols
char writeByte(char deviceAddress, char registerAddress, char data)
{
    CS_LOW;
    xmit_spi(registerAddress);
    xmit_spi(data);
    CS_HIGH;
    DELAY_US(50);
    return 0;
}

    // Read a byte from given register on device. Calls necessary SPI or I2C
    // implementation. This was configured in the constructor.
char readByte(char deviceAddress, char registerAddress)
{
    char data; // `data` will store the register data
    CS_LOW;
    xmit_spi(registerAddress | READ_FLAG);
    //    DELAY_US(10);
    data = rcvr_spi();
    CS_HIGH;
    DELAY_US(50);
    return data;
}

char readBytes(char deviceAddress, char registerAddress, char count,
               char * dest)
{

    char i;
    CS_LOW;
    xmit_spi(registerAddress| READ_FLAG);
    for (i = 0; i < count; i++)
    {
        dest[i] = rcvr_spi();
    }
    CS_HIGH;
    DELAY_US(50);                     // 50 microseconds  delay
    return i;
}
// edit the value of register
// @val data to be written
// @numbits number of bits to be edited
// @shifts  number of shifts from lsb to arrive at bits to be edited
char regedit(char deviceAddress, char registerAddress, char val, int numbits, int shifts)
{
    char value = readByte( deviceAddress, registerAddress);
    // mask off the data before writing
    char mask = (1 << (numbits)) - 1;
    val &= mask;

    mask <<= shifts;
    value &= ~mask;          // remove the current data at that spot
    value |= val << shifts; // and add in the new data

    return writeByte( deviceAddress,  registerAddress,  value);
}

