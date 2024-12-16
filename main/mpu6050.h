#ifndef __MPU6050_H__
#define __MPU6050_H__

#include "driver/i2c.h"
#include "esp_err.h"

#define MPU6050_ADDR 0x68 // MPU6050 Device Address

// Filter Constants
#define WINDOW_SIZE 10

// Register Map
#define MPU6050_REG_SMPLRT_DIV 0x19
#define MPU6050_REG_CONFIG 0x1A
#define MPU6050_REG_GYRO_CONFIG 0x1B
#define MPU6050_REG_GYRO_XOUT_H 0x43
#define MPU6050_REG_ACCEL_CONFIG 0x1C
#define MPU6050_REG_PWR_MGMT_1 0x6B
#define MPU6050_REG_WHO_AM_I 0x75
#define MPU6050_REG_ACCEL_XOUT_H 0x3B
#define MPU6050_REG_TEMP_OUT_H 0x41

// Offset Registers
#define MPU6050_REG_XG_OFFS_USRH 0x13
#define MPU6050_REG_YG_OFFS_USRH 0x15
#define MPU6050_REG_ZG_OFFS_USRH 0x17

// Function Declarations
esp_err_t MPU6050_Init(i2c_port_t i2c_num, uint8_t i2c_addr);
esp_err_t MPU6050_ReadAccelData(i2c_port_t i2c_num, uint8_t i2c_addr, int16_t* accelData);
esp_err_t MPU6050_ReadGyroData(i2c_port_t i2c_num, uint8_t i2c_addr, int16_t* gyroData);
esp_err_t MPU6050_ReadTemperature(i2c_port_t i2c_num, uint8_t i2c_addr, int16_t* tempData);
esp_err_t calibrate_mpu6050(i2c_port_t i2c_num, uint8_t i2c_addr);
void sliding_window_filter(int16_t *data);
float kalman_filter(float measurement, int axis);

#endif // __MPU6050_H__
