/*
 * MPUXX50.h
 *
 *  Created on: Apr 26, 2022
 *      Author: MarkSherstan
 */

#ifndef SRC_MPUXX50_H_
#define SRC_MPUXX50_H_

#include <stdint.h>
#include <math.h>
#include "stm32f4xx_hal.h"

// Constants
#define RAD2DEG 57.2957795131

// Defines
#define WHO_AM_I_6050_ANS 0x68
#define WHO_AM_I_9250_ANS 0x70
#define WHO_AM_I          0x75
#define AD0_LOW           0x68
#define AD0_HIGH          0x69
#define GYRO_CONFIG       0x1B
#define ACCEL_CONFIG      0x1C
#define PWR_MGMT_1        0x6B
#define ACCEL_XOUT_H      0x3B
#define I2C_TIMOUT_MS     1000

// Structs
typedef struct {
    int16_t ax, ay, az, gx, gy, gz;
} RawData;

typedef struct {
    float ax, ay, az, gx, gy, gz;
} ProcessedData;

typedef struct {
    float x, y, z;
} GyroCal;

typedef struct {
    float r, p, yaw, ax, ay, az, gx, gy, gz;
} Attitude;

// Full scale ranges
typedef enum {
    GFSR_250DPS,
    GFSR_500DPS,
    GFSR_1000DPS,
    GFSR_2000DPS
} gyroscopeFullScaleRange;

typedef enum {
    AFSR_2G,
    AFSR_4G,
    AFSR_8G,
    AFSR_16G
} accelerometerFullScaleRange;

// MPUXX50 structure to replace C++ class
typedef struct {
    float aScaleFactor;
    float gScaleFactor;
    I2C_HandleTypeDef *pI2Cx;
    uint8_t addr;
    uint8_t gFSR;
    uint8_t aFSR;
    float tau;
    float dt;
    GyroCal gyroCal;
    Attitude attitude;
} MPUXX50;

// Function prototypes
void MPUXX50_Init(MPUXX50 *mpuxx50, I2C_HandleTypeDef *pI2Cx, uint8_t addr);
uint8_t MPUXX50_Begin(MPUXX50 *mpuxx50);
void MPUXX50_WriteAccFullScaleRange(MPUXX50 *mpuxx50, uint8_t aFSR);
void MPUXX50_WriteGyroFullScaleRange(MPUXX50 *mpuxx50, uint8_t gFSR);
void MPUXX50_SetGyroFullScaleRange(MPUXX50 *mpuxx50, uint8_t gFSR);
void MPUXX50_SetAccFullScaleRange(MPUXX50 *mpuxx50, uint8_t aFSR);
void MPUXX50_SetDeltaTime(MPUXX50 *mpuxx50, float dt);
void MPUXX50_SetTau(MPUXX50 *mpuxx50, float tau);
void MPUXX50_CalibrateGyro(MPUXX50 *mpuxx50, uint16_t numCalPoints);
RawData MPUXX50_ReadRawData(MPUXX50 *mpuxx50);
ProcessedData MPUXX50_ProcessData(MPUXX50 *mpuxx50);
Attitude MPUXX50_CalcAttitude(MPUXX50 *mpuxx50);

#endif /* SRC_MPUXX50_H_ */
