#include <string.h>
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "driver/sdspi_host.h"
#include "driver/spi_common.h"
#include "sdmmc_cmd.h"
#include "sd_card.h"
#include "sd_card_config.h"

static const char *TAG = "SD_CARD";
static sdmmc_card_t *card;

esp_err_t sd_card_init(void) {
    esp_err_t ret;

    // Configure SPI Bus
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };

    ret = spi_bus_initialize(SD_SPI_HOST, &bus_cfg, SD_SPI_DMA_CHAN);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed To Initialize SPI Bus");
        return ret;
    }

    // Configure SD Card
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = SD_SPI_HOST;  // Set Correct SPI Host

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = PIN_NUM_CS;
    slot_config.host_id = SD_SPI_HOST;

    // Mount FAT Filesystem
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = true,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };

    ret = esp_vfs_fat_sdspi_mount(MOUNT_POINT, &host, &slot_config, &mount_config, &card);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed To Mount SD Card");
        return ret;
    }

    ESP_LOGI(TAG, "SD Card Mounted Successfully");
    return ESP_OK;
}

esp_err_t sd_card_write_data(const char* data) {
    FILE* f = fopen(DATA_FILE, "a");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed To Open File For Writing");
        return ESP_FAIL;
    }

    fprintf(f, "%s\n", data);
    fclose(f);
    return ESP_OK;
}

esp_err_t sd_card_read_file(char** buffer, size_t* size) {
    FILE* f = fopen(DATA_FILE, "r");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed To Open File For Reading");
        return ESP_FAIL;
    }

    // Get File Size
    fseek(f, 0, SEEK_END);
    *size = ftell(f);
    fseek(f, 0, SEEK_SET);

    // Allocate Memory For The File Content
    *buffer = malloc(*size + 1);
    if (*buffer == NULL) {
        fclose(f);
        return ESP_ERR_NO_MEM;
    }

    // Read File Content
    fread(*buffer, 1, *size, f);
    (*buffer)[*size] = '\0';
    
    fclose(f);
    return ESP_OK;
}