#ifndef MPU6050_H
#define MPU6050_H

#include "main.h"
#include "i2c.h"

typedef struct {
    float gyro_bias_z;
    float yaw_rad;
    float gz_rad_s;
} MPU6050_t;

HAL_StatusTypeDef MPU6050_Init(MPU6050_t *imu);
HAL_StatusTypeDef MPU6050_CalibrateGyro(MPU6050_t *imu, uint16_t samples);
HAL_StatusTypeDef MPU6050_Update(MPU6050_t *imu, float dt);

#endif
