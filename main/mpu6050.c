#include "mpu6050.h"
#include "esp_log.h"

// MPU6050 Initialization
esp_err_t MPU6050_Init(i2c_port_t i2c_num, uint8_t i2c_addr) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (i2c_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, MPU6050_REG_PWR_MGMT_1, true);
    i2c_master_write_byte(cmd, 0x00, true); // Wake Up MPU6050
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) {
        printf("Failed To Initialize MPU6050, Error: %s\n", esp_err_to_name(ret));
    }

    // Call Calibration Function
    ret = calibrate_mpu6050(i2c_num, i2c_addr);
    if (ret == ESP_OK) {
        printf("MPU6050 Calibration Successful.\n");
    } else {
        printf("MPU6050 Calibration Failed.\n");
    }

    return ret;
}

// Read Accelerometer Data
esp_err_t MPU6050_ReadAccelData(i2c_port_t i2c_num, uint8_t i2c_addr, int16_t* accelData) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (i2c_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, MPU6050_REG_ACCEL_XOUT_H, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (i2c_addr << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, (uint8_t*) accelData, I2C_MASTER_LAST_NACK);
    i2c_master_read_byte(cmd, ((uint8_t*) accelData) + 1, I2C_MASTER_LAST_NACK);
    i2c_master_read_byte(cmd, ((uint8_t*) accelData) + 2, I2C_MASTER_LAST_NACK);
    i2c_master_read_byte(cmd, ((uint8_t*) accelData) + 3, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

// Sliding Window Filter
static int16_t window[3][WINDOW_SIZE] = {0};
static int window_index = 0;

void sliding_window_filter(int16_t *data) {
    for (int axis = 0; axis < 3; axis++) {
        window[axis][window_index] = data[axis];
        int32_t sum = 0;
        for (int i = 0; i < WINDOW_SIZE; i++) {
            sum += window[axis][i];
        }
        data[axis] = sum / WINDOW_SIZE;
    }
    window_index = (window_index + 1) % WINDOW_SIZE;
}

// Kalman Filter
typedef struct {
    float x;
    float P;
    float K;
} KalmanFilter;

static KalmanFilter kalmanFilters[3] = {{0, 1, 0}, {0, 1, 0}, {0, 1, 0}}; // Initialize Kalman Filters For Each Axis
const float Q = 0.001; // 过程噪声协方差
const float R = 0.1;   // 测量噪声协方差

float kalman_filter(float measurement, int axis) {
    // Prediction
    kalmanFilters[axis].P += Q;

    // Update
    kalmanFilters[axis].K = kalmanFilters[axis].P / (kalmanFilters[axis].P + R);
    kalmanFilters[axis].x = kalmanFilters[axis].x + kalmanFilters[axis].K * (measurement - kalmanFilters[axis].x);
    kalmanFilters[axis].P = (1 - kalmanFilters[axis].K) * kalmanFilters[axis].P;

    return kalmanFilters[axis].x;
}

// Temperature Compensation
float temperature_compensation(int16_t tempData, int16_t *gyroData) {
    float temp = (tempData / 340.0) + 36.53; // Transform Raw Temperature Data To Celsius
    float temp_compensation_factor = 0.03; // Factor For Temperature Compensation
    
    for (int axis = 0; axis < 3; axis++) {
        gyroData[axis] -= (temp - 35) * temp_compensation_factor * 32768.0 / 250; // 250 Degrees Per Second
    }

    return temp;
}

// Read Gyroscope Data
esp_err_t MPU6050_ReadGyroData(i2c_port_t i2c_num, uint8_t i2c_addr, int16_t* gyroData) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (i2c_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, MPU6050_REG_GYRO_XOUT_H, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (i2c_addr << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, (uint8_t*) gyroData, 6, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    /*
    if (ret == ESP_OK) {
        // Read Temperature Data And Perform Temperature Compensation
        int16_t tempData;
        ret = MPU6050_ReadTemperature(i2c_num, i2c_addr, &tempData);
        if (ret == ESP_OK) {
            float temp = (tempData / 340.0) + 36.53; // Transform Raw Temperature Data To Celsius
            float temp_compensation_factor = 0.03; // Factor For Temperature Compensation
            
            for (int axis = 0; axis < 3; axis++) {
                // Temperature Compensation
                gyroData[axis] -= (temp - 35) * temp_compensation_factor * 32768.0 / 250; // 250 Degrees Per Second
            }

            // Sliding Window Filter
            sliding_window_filter(gyroData);

            // Kalman Filter
            for (int axis = 0; axis < 3; axis++) {
                gyroData[axis] = (int16_t)kalman_filter((float)gyroData[axis], axis);
            }
        }
    }
    */

    return ret;
}


// Read Temperature Data
esp_err_t MPU6050_ReadTemperature(i2c_port_t i2c_num, uint8_t i2c_addr, int16_t* tempData) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (i2c_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, MPU6050_REG_TEMP_OUT_H, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (i2c_addr << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, (uint8_t*) tempData, I2C_MASTER_LAST_NACK);
    i2c_master_read_byte(cmd, ((uint8_t*) tempData) + 1, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

// Write Offset Data
static esp_err_t write_mpu6050_offset(i2c_port_t i2c_num, uint8_t i2c_addr, int16_t* offset) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (i2c_addr << 1) | I2C_MASTER_WRITE, true);

    // Write X Axis Offset
    i2c_master_write_byte(cmd, MPU6050_REG_XG_OFFS_USRH, true);
    i2c_master_write_byte(cmd, (offset[0] >> 8) & 0xFF, true);
    i2c_master_write_byte(cmd, offset[0] & 0xFF, true);

    // Write Y Axis Offset
    i2c_master_write_byte(cmd, MPU6050_REG_YG_OFFS_USRH, true);
    i2c_master_write_byte(cmd, (offset[1] >> 8) & 0xFF, true);
    i2c_master_write_byte(cmd, offset[1] & 0xFF, true);

    // Write Z Axis Offset
    i2c_master_write_byte(cmd, MPU6050_REG_ZG_OFFS_USRH, true);
    i2c_master_write_byte(cmd, (offset[2] >> 8) & 0xFF, true);
    i2c_master_write_byte(cmd, offset[2] & 0xFF, true);

    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}

// Calibrate MPU6050
esp_err_t calibrate_mpu6050(i2c_port_t i2c_num, uint8_t i2c_addr) {
    int16_t gyro_offset[3] = {0, 0, 0};
    int samples = 1000; // Number Of Samples To Average
    for (int i = 0; i < samples; i++) {
        int16_t gyroData[3];
        MPU6050_ReadGyroData(i2c_num, i2c_addr, gyroData);
        for (int axis = 0; axis < 3; axis++) {
            gyro_offset[axis] += gyroData[axis];
        }
        vTaskDelay(pdMS_TO_TICKS(1)); // Delay 1ms Between Samples To Prevent Overheating
    }

    for (int axis = 0; axis < 3; axis++) {
        gyro_offset[axis] /= samples;
    }

    // Return Offset Data
    return write_mpu6050_offset(i2c_num, i2c_addr, gyro_offset);
}