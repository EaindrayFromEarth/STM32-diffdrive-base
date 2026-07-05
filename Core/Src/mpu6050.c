#include "mpu6050.h"
#include <math.h>

#define MPU6050_ADDR (0x68 << 1)
#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_CONFIG 0x1A
#define MPU6050_GYRO_CONFIG 0x1B
#define MPU6050_ACCEL_CONFIG 0x1C
#define MPU6050_ACCEL_XOUT_H 0x3B

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

static HAL_StatusTypeDef mpu_write(uint8_t reg, uint8_t val)
{
    return HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &val, 1, 100);
}

static HAL_StatusTypeDef mpu_read(uint8_t reg, uint8_t *buf, uint16_t len)
{
    return HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, reg, I2C_MEMADD_SIZE_8BIT, buf, len, 100);
}

HAL_StatusTypeDef MPU6050_Init(MPU6050_t *imu)
{
    if (!imu)
        return HAL_ERROR;

    imu->gyro_bias_z = 0.0f;
    imu->yaw_rad = 0.0f;
    imu->gz_rad_s = 0.0f;

    HAL_Delay(100);

    if (mpu_write(MPU6050_PWR_MGMT_1, 0x00) != HAL_OK)
        return HAL_ERROR; // wake up
    HAL_Delay(10);

    if (mpu_write(MPU6050_CONFIG, 0x03) != HAL_OK)
        return HAL_ERROR; // low-pass filter
    if (mpu_write(MPU6050_GYRO_CONFIG, 0x08) != HAL_OK)
        return HAL_ERROR; // ±500 dps
    if (mpu_write(MPU6050_ACCEL_CONFIG, 0x00) != HAL_OK)
        return HAL_ERROR; // ±2g

    return HAL_OK;
}

HAL_StatusTypeDef MPU6050_CalibrateGyro(MPU6050_t *imu, uint16_t samples)
{
    if (!imu || samples == 0)
        return HAL_ERROR;

    float sum = 0.0f;
    uint8_t buf[14];

    for (uint16_t i = 0; i < samples; i++)
    {
        if (mpu_read(MPU6050_ACCEL_XOUT_H, buf, 14) != HAL_OK)
            return HAL_ERROR;

        int16_t raw_gx = (int16_t)((buf[8] << 8) | buf[9]);
        int16_t raw_gy = (int16_t)((buf[10] << 8) | buf[11]);
        int16_t raw_gz = (int16_t)((buf[12] << 8) | buf[13]);

        float yaw_dps = raw_gz / 65.5f;

        sum += yaw_dps;
        HAL_Delay(5);
    }

    imu->gyro_bias_z = sum / samples;
    imu->yaw_rad = 0.0f;
    return HAL_OK;
}

HAL_StatusTypeDef MPU6050_Update(MPU6050_t *imu, float dt)
{
    if (!imu)
        return HAL_ERROR;

    uint8_t buf[14];
    if (mpu_read(MPU6050_ACCEL_XOUT_H, buf, 14) != HAL_OK)
        return HAL_ERROR;

    int16_t raw_gx = (int16_t)((buf[8] << 8) | buf[9]);
    int16_t raw_gy = (int16_t)((buf[10] << 8) | buf[11]);
    int16_t raw_gz = (int16_t)((buf[12] << 8) | buf[13]);

    float yaw_dps = raw_gz / 65.5f;
    yaw_dps -= imu->gyro_bias_z;

    imu->gz_rad_s = yaw_dps * (M_PI / 180.0f);

    if (fabsf(imu->gz_rad_s) < 0.005f)
    {
        imu->gz_rad_s = 0.0f;
    }

    imu->yaw_rad += imu->gz_rad_s * dt;

    if (imu->yaw_rad > M_PI)
        imu->yaw_rad -= 2.0f * M_PI;
    if (imu->yaw_rad < -M_PI)
        imu->yaw_rad += 2.0f * M_PI;

    return HAL_OK;
}
