#ifndef SD_CARD_H
#define SD_CARD_H

#include "esp_err.h"
#include "mpu6050.h"

esp_err_t sd_card_init(void);
esp_err_t sd_card_write_data(const char* data);
esp_err_t sd_card_read_file(char** buffer, size_t* size);

#endif