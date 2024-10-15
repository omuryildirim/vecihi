#include "MPUXX50.h"

// Constructor for MPUXX50 I2C
void MPUXX50_Init(MPUXX50 *mpuxx50, I2C_HandleTypeDef *pI2Cx, uint8_t addr) {
    mpuxx50->pI2Cx = pI2Cx;
    mpuxx50->addr = addr << 1;
}

// Boot up the IMU and ensure we have a valid connection
uint8_t MPUXX50_Begin(MPUXX50 *mpuxx50) {
    uint8_t check, select;

    // Set attitude to zero conditions
    mpuxx50->attitude.r = 0;
    mpuxx50->attitude.p = 0;
    mpuxx50->attitude.yaw = 0;
    mpuxx50->attitude.ax = 0;
    mpuxx50->attitude.ay = 0;
    mpuxx50->attitude.az = 0;
    mpuxx50->attitude.gx = 0;
    mpuxx50->attitude.gy = 0;
    mpuxx50->attitude.gz = 0;

    // Confirm device
    HAL_I2C_Mem_Read(mpuxx50->pI2Cx, mpuxx50->addr, WHO_AM_I, 1, &check, 1, I2C_TIMOUT_MS);

    if ((check == WHO_AM_I_9250_ANS) || (check == WHO_AM_I_6050_ANS)) {
        select = 0x00;
        HAL_I2C_Mem_Write(mpuxx50->pI2Cx, mpuxx50->addr, PWR_MGMT_1, 1, &select, 1, I2C_TIMOUT_MS);

        MPUXX50_WriteAccFullScaleRange(mpuxx50, mpuxx50->aFSR);
        MPUXX50_WriteGyroFullScaleRange(mpuxx50, mpuxx50->gFSR);

        return 1;
    } else {
        return 0;
    }
}

// Set the accelerometer full scale range
void MPUXX50_WriteAccFullScaleRange(MPUXX50 *mpuxx50, uint8_t aFSR) {
    uint8_t select;

    switch (aFSR) {
        case AFSR_2G:
            mpuxx50->aScaleFactor = 16384.0;
            select = 0x00;
            break;
        case AFSR_4G:
            mpuxx50->aScaleFactor = 8192.0;
            select = 0x08;
            break;
        case AFSR_8G:
            mpuxx50->aScaleFactor = 4096.0;
            select = 0x10;
            break;
        case AFSR_16G:
            mpuxx50->aScaleFactor = 2048.0;
            select = 0x18;
            break;
        default:
            mpuxx50->aScaleFactor = 8192.0;
            select = 0x08;
            break;
    }

    HAL_I2C_Mem_Write(mpuxx50->pI2Cx, mpuxx50->addr, ACCEL_CONFIG, 1, &select, 1, I2C_TIMOUT_MS);
}

// Set the gyroscope full scale range
void MPUXX50_WriteGyroFullScaleRange(MPUXX50 *mpuxx50, uint8_t gFSR) {
    uint8_t select;

    switch (gFSR) {
        case GFSR_250DPS:
            mpuxx50->gScaleFactor = 131.0;
            select = 0x00;
            break;
        case GFSR_500DPS:
            mpuxx50->gScaleFactor = 65.5;
            select = 0x08;
            break;
        case GFSR_1000DPS:
            mpuxx50->gScaleFactor = 32.8;
            select = 0x10;
            break;
        case GFSR_2000DPS:
            mpuxx50->gScaleFactor = 16.4;
            select = 0x18;
            break;
        default:
            mpuxx50->gScaleFactor = 65.5;
            select = 0x08;
            break;
    }

    HAL_I2C_Mem_Write(mpuxx50->pI2Cx, mpuxx50->addr, GYRO_CONFIG, 1, &select, 1, I2C_TIMOUT_MS);
}

// Set the gyroscope full scale range
void MPUXX50_SetGyroFullScaleRange(MPUXX50 *mpuxx50, uint8_t gFSR) {
    mpuxx50->gFSR = gFSR;
}

// Set the accelerometer full scale range
void MPUXX50_SetAccFullScaleRange(MPUXX50 *mpuxx50, uint8_t aFSR) {
    mpuxx50->aFSR = aFSR;
}

// Set the sampling duration (delta time) in seconds
void MPUXX50_SetDeltaTime(MPUXX50 *mpuxx50, float dt) {
    mpuxx50->dt = dt;
}

// Set the time constant of the complementary filter
void MPUXX50_SetTau(MPUXX50 *mpuxx50, float tau) {
    mpuxx50->tau = tau;
}

// Find offsets for each axis of gyroscope
void MPUXX50_CalibrateGyro(MPUXX50 *mpuxx50, uint16_t numCalPoints) {
    RawData rawData;
    int32_t x = 0;
    int32_t y = 0;
    int32_t z = 0;

    if (numCalPoints == 0) {
        numCalPoints = 1;
    }

    for (uint16_t ii = 0; ii < numCalPoints; ii++) {
        rawData = MPUXX50_ReadRawData(mpuxx50);
        x += rawData.gx;
        y += rawData.gy;
        z += rawData.gz;
        HAL_Delay(3);
    }

    mpuxx50->gyroCal.x = (float)x / (float)numCalPoints;
    mpuxx50->gyroCal.y = (float)y / (float)numCalPoints;
    mpuxx50->gyroCal.z = (float)z / (float)numCalPoints;
}

// Read raw data from IMU
RawData MPUXX50_ReadRawData(MPUXX50 *mpuxx50) {
    RawData rawData;
    uint8_t buf[14];

    HAL_I2C_Mem_Read(mpuxx50->pI2Cx, mpuxx50->addr, ACCEL_XOUT_H, 1, buf, 14, I2C_TIMOUT_MS);

    rawData.ax = (int16_t)(buf[0] << 8 | buf[1]);
    rawData.ay = (int16_t)(buf[2] << 8 | buf[3]);
    rawData.az = (int16_t)(buf[4] << 8 | buf[5]);
    rawData.gx = (int16_t)(buf[8] << 8 | buf[9]);
    rawData.gy = (int16_t)(buf[10] << 8 | buf[11]);
    rawData.gz = (int16_t)(buf[12] << 8 | buf[13]);

    return rawData;
}

// Process the raw data into real-world sensor values
ProcessedData MPUXX50_ProcessData(MPUXX50 *mpuxx50) {
    ProcessedData processedData;
    RawData rawData = MPUXX50_ReadRawData(mpuxx50);

    processedData.ax = rawData.ax / mpuxx50->aScaleFactor;
    processedData.ay = rawData.ay / mpuxx50->aScaleFactor;
    processedData.az = rawData.az / mpuxx50->aScaleFactor;

    processedData.gx = (rawData.gx - mpuxx50->gyroCal.x) / mpuxx50->gScaleFactor;
    processedData.gy = (rawData.gy - mpuxx50->gyroCal.y) / mpuxx50->gScaleFactor;
    processedData.gz = (rawData.gz - mpuxx50->gyroCal.z) / mpuxx50->gScaleFactor;

    /*
	printf("[%.4f,%.4f,%.4f,%.4f,%.4f,%.4f]\r\n",
			rawData.gx,
			rawData.gy,
			rawData.gz,
			processedData.gx,
			processedData.gy,
			processedData.gz);
			*/
    return processedData;
}

// Calculate the attitude of the sensor in degrees using a complementary filter
Attitude MPUXX50_CalcAttitude(MPUXX50 *mpuxx50) {
    ProcessedData sensorData = MPUXX50_ProcessData(mpuxx50);

    float accelPitch = atan2(sensorData.ay, sensorData.az) * RAD2DEG;
    float accelRoll = atan2(sensorData.ax, sensorData.az) * RAD2DEG;

    mpuxx50->attitude.r = mpuxx50->tau * (mpuxx50->attitude.r - sensorData.gy * mpuxx50->dt) + (1 - mpuxx50->tau) * accelRoll;
    mpuxx50->attitude.p = mpuxx50->tau * (mpuxx50->attitude.p - sensorData.gx * mpuxx50->dt) + (1 - mpuxx50->tau) * accelPitch;
    mpuxx50->attitude.yaw += (sensorData.gz * mpuxx50->dt);

    mpuxx50->attitude.ax = sensorData.ax;
    mpuxx50->attitude.ay = sensorData.ay;
    mpuxx50->attitude.az = sensorData.az;

    mpuxx50->attitude.gz = sensorData.gx;
    mpuxx50->attitude.gy = sensorData.gy;
    mpuxx50->attitude.gz = sensorData.gz;


    return mpuxx50->attitude;
}
