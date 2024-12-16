#ifndef SD_CARD_CONFIG_H
#define SD_CARD_CONFIG_H

#include "driver/spi_common.h"

// SD Card SPI Configuration
#define SD_SPI_HOST    SPI2_HOST
#define SD_SPI_DMA_CHAN    SPI_DMA_CH_AUTO

// SD Card Pins
#define PIN_NUM_MISO 11
#define PIN_NUM_MOSI 13
#define PIN_NUM_CLK  12
#define PIN_NUM_CS   10

// Mount Point For The SD Card
#define MOUNT_POINT "/sdcard"
#define DATA_FILE MOUNT_POINT "/sensor_data.txt"

#endif