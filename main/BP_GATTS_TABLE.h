#ifndef GATTS_TABLE_CREAT_DEMO_H
#define GATTS_TABLE_CREAT_DEMO_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gatt_common_api.h"

#define I2C_MASTER_NUM          I2C_NUM_0
#define MPU6050_ADDR            0x68

/* Attributes State Machine */
enum
{
    IDX_SVC,
    IDX_CHAR_A,
    IDX_CHAR_VAL_A,
    IDX_CHAR_CFG_A,

    IDX_CHAR_B,
    IDX_CHAR_VAL_B,

    IDX_CHAR_C,
    IDX_CHAR_VAL_C,

    HRS_IDX_NB,
};

#endif /* GATTS_TABLE_CREAT_DEMO_H */